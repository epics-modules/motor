/*
FILENAME...     devOmsCom.cc
USAGE... Data and functions common to all OMS device level support.

Version:        $Revision: 1.13 $
Modified By:    $Author: sluiter $
Last Modified:  $Date: 2008-05-14 16:37:41 $
*/

/*
 *      Original Author: Jim Kowalkowski
 *      Previous Author: Joe Sullivan
 *      Date: 11/14/94
 *      Current  Author: Ron Sluiter
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  01-18-93 jbk initialized
 * .02  11-14-94 jps copy devOMS.c and modify to point to vme58 driver
 * .03  03-19-96 tmm v1.10: modified encoder-ratio calculation
 * .04  06-20-96 jps allow for bumpless-reboot on position
 * .04a 02-19-97 tmm fixed for EPICS 3.13
 * .05  05-14-01 rls Support for jog velocity and acceleration commands.
 *                   Added "CA" to home and jog commands so JVEL does not
 *                   see done flag from previous operation.
 * .06  03-27-02 rls Eliminated RES. All input positions, velocities and
 *                   accelerations are in motor steps.
 * .07  07-05-02 rls Work around for OMS bug that ignores MR+/-1.
 *                   Use OMS UU command to support reference is always in
 *                   motor steps.
 * .08  06-04-03 rls Convert to R3.14.x.
 * .09  06-04-03 rls extended device directive support PREM and POST.
 * .10  06-16-03 rls Converted to R3.14.x.
 * .11  06-16-04 rls Terminate "LP" command with ";" to prevent MAXv stale data.
 * .12  08-27-04 rls Terminate "JG" command with ";" to prevent MAXv stale data.
 * .13  03-21-05 rls OSI - built for solaris and linux hosts.
 * .14  03-23-05 rls restrict acceleration to valid values.
 * .15  08-18-06 rls Output "slew <= base" error message only one time.
 * .16  02-16-07 rls Bug fix for overwriting PID parameter fields during
 *                   normalization calculation.
 * .17  05-14-08 rls Fixed stop acceleration calculation.
 *
 */

#include <string.h>
#include <stdio.h>
#include <errlog.h>
#include <math.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <dbAccess.h>

#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "devOmsCom.h"

/*
Command set used by record support.  WARNING! this must match "motor_cmnd" in
motor.h
*/

struct motor_table
{
    msg_types type;
    const char *command;
    int num_parms;
};

struct motor_table const oms_table[] =
{
    {MOTION, " MA", 1},         /* MOVE_ABS */
    {MOTION, " MR", 1},         /* MOVE_REL */
    {MOTION, " CA HM", 1},      /* HOME_FOR */
    {MOTION, " CA HR", 1},      /* HOME_REV */
    {IMMEDIATE, " LP", 1},      /* LOAD_POS */
    {IMMEDIATE, " VB", 1},      /* SET_VEL_BASE */
    {IMMEDIATE, " VL", 1},      /* SET_VELO */
    {IMMEDIATE, " AC", 1},      /* SET_ACCEL */
    {IMMEDIATE, " GD", 0},      /* jps: from GO to GD */
    {IMMEDIATE, " ER", 2},      /* SET_ENC_RATIO */
    {INFO,      " ",   0},      /* GET_INFO */
    {MOVE_TERM, " ST", 0},      /* STOP_AXIS */
    {VELOCITY, " CA JG", 1},    /* JOG */
    {IMMEDIATE," KP", 1},       /* SET_PGAIN */
    {IMMEDIATE," KI", 1},       /* SET_IGAIN */
    {IMMEDIATE," KD", 1},       /* SET_DGAIN */
    {IMMEDIATE," HN", 0},       /* ENABLE_TORQUE */
    {IMMEDIATE," HF", 0},       /* DISABL_TORQUE */
    {IMMEDIATE,   "", 0},       /* PRIMITIVE */
    {IMMEDIATE,   "", 0},       /* SET_HIGH_LIMIT */
    {IMMEDIATE,   "", 0},       /* SET_LOW_LIMIT */
    {VELOCITY, " JG", 1},       /* JOG_VELOCITY */
};


/*
FUNCTION... long oms_build_trans(motor_cmnd, double *, struct motorRecord *)
USAGE... Add a part to the transaction.

INPUT...        command - index into oms_table[].
                *parms  - one or more input parameters; determined by
                            "num_parms" in oms_table.
                *mr     - motor record pointer.
LOGIC...
    Initialize pointers; return value to OK.
    Set highest command type priority.
    IF transaction is NOT initialized (i.e., state != BUILD_STATE)
        ERROR RETURN.
    ENDIF
    
    IF input command is a PRIMITIVE command, AND, the INIT field is not empty.
        IF input command is a "device directive".
            Find terminating '@'.
            IF valid "device directive" (i.e, directive bracketed by '@'s).
                IF "Driver Power Monitoring ON" directive.
                    Flush response buffer.
                    Read motor controller's GPIO configuration.
                    IF bit corresponding to current axis is configured as an
                            output bit.
                        Set "Driver Power Monitoring" indicator true.
                    ELSE
                        Output ERROR message.
                    ENDIF
                ENDIF
                Strip device directive and copy rest of INIT field to local buffer.
            ELSE
                Copy INIT field to local buffer.
            ENDIF
        ELSE
            Copy INIT field to local buffer.
        ENDIF
    ELSE
        IF input command is set HIGH or LOW travel limit.
            Null command. Set transaction state to IDLE.
        ELSE
            IF input command is Proportional Gain, AND, gain < valid minimum.
                Set the gain and the PCOF field to the minimum.
                Set return indicator to ERROR.
            ELSE IF input command is STOP_AXIS.
                NOTE: This logic is here as a workaround for an OMS firmware
                error.  When a STOP command was issued during the target move,
                the backlash acceleration would go into effect.
                
                Prefix command with ACCL field acceleration.
            ENDIF
            IF command type is MOTION or VELOCITY.
                Process PREM and POST fields.
            ENDIF
            IF Overtravel status indicator true, AND, input command is a MOVE.
                NOTE: This logic is here as a workaround for the "Moving off a
                limit switch" OMS firmware error.

                IF incremental distance to target position is positive.
                    Set OMS controller "direction logic" positive.
                ELSE
                    Set OMS controller "direction logic" negative.
                ENDIF
            ENDIF
            Copy input command into output message.
            Copy input command parameters into output message.
        ENDIF
    ENDIF    
*/

RTN_STATUS oms_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    char buffer[40];
    msg_types cmnd_type;
    RTN_STATUS rtnind;
    static bool invalid_velmsg_latch = false;

    rtnind = OK;
    motor_call = &trans->motor_call;

    cmnd_type = oms_table[command].type;
    if (cmnd_type > motor_call->type)
        motor_call->type = cmnd_type;
    
    /* concatenate onto the dpvt message field */
    if (trans->state != BUILD_STATE)
        return(rtnind = ERROR);

    if ((command == PRIMITIVE) && (mr->init != NULL) &&
        (strlen(mr->init) != 0))
    {
        extern struct driver_table oms58_access;
        /* Test for a "device directive" in the Initialization string. */
        if ((mr->init[0] == '@') && (trans->tabptr == &oms58_access))
        {
            char *end = strrchr(&mr->init[1], '@');
            if (end != NULL)
            {
                struct driver_table *tabptr = trans->tabptr;
                int size = (end - &mr->init[0]) + 1;
                strncpy(buffer, mr->init, size);
                buffer[size] = (char) NULL;
                if (strcmp(buffer, "@DPM_ON@") == 0)
                {
                    int response, bitselect;
                    char respbuf[10];

                    (*tabptr->getmsg)(motor_call->card, respbuf, -1);
                    (*tabptr->sendmsg)(motor_call->card, "RB\r",
                                       (char) NULL);
                    (*tabptr->getmsg)(motor_call->card, respbuf, 1);
                    if (sscanf(respbuf, "%x", &response) == 0)
                        response = 0;   /* Force an error. */
                    bitselect = (1 << motor_call->signal);
                    if ((response & bitselect) == 0)
                        trans->dpm = true;
                    else
                        errPrintf(0, __FILE__, __LINE__,
                            "Invalid VME58 configuration; RB = 0x%x\n", response);
                }
                end++;
                strcpy(buffer, end);
            }
            else
                strcpy(buffer, mr->init);
        }
        else
            strcpy(buffer, mr->init);

        strcat(motor_call->message, " ");
        strcat(motor_call->message, buffer);
    }
    else
    {
        int first_one, itera;

        if (command == SET_HIGH_LIMIT || command == SET_LOW_LIMIT)
            trans->state = IDLE_STATE;  /* No command sent to the controller. */
        else
        {
            msta_field msta;
            
            /* Silently enforce minimum range on KP command. */
            if (command == SET_PGAIN && *parms < 0.00005)
            {
                *parms = 0.00005;
                mr->pcof = 0.00005;
                rtnind = ERROR;
            }
            else if (command == STOP_AXIS)
            {
                double acc = ((mr->velo - mr->vbas) / fabs(mr->mres)) / mr->accl;
            
                /* Put in acceleration. */
                strcat(motor_call->message, oms_table[SET_ACCEL].command);
                sprintf(buffer, "%ld", NINT(acc));
                strcat(motor_call->message, buffer);
            }

            if (cmnd_type == MOTION || cmnd_type == VELOCITY)
            {
                if (strlen(mr->prem) != 0)
                {
                    char prem_buff[40];

                    /* Test for a "device directive" in the PREM string. */
                    if (mr->prem[0] == '@')
                    {
                        bool errind = false;
                        char *end = strchr(&mr->prem[1], '@');
                        if (end == NULL)
                            errind = true;
                        else
                        {
                            DBADDR addr;
                            double delay;
                            char *start, *tail;
                            int size = (end - &mr->prem[0]) + 1;

                            /* Copy device directive to prem_buff. */
                            strncpy(prem_buff, mr->prem, size);
                            prem_buff[size] = (char) NULL;

                            if (strncmp(prem_buff, "@PUT(", 5) != 0)
                                goto errorexit;

                            /* Point "start" to PV name argument. */
                            tail = NULL;
                            start = epicsStrtok_r(&prem_buff[5], ",", &tail);
                            if (tail == NULL)
                                goto errorexit;

                            if (dbNameToAddr(start, &addr))     /* Get address of PV. */
                            {
                                errPrintf(-1, __FILE__, __LINE__, "Invalid PV name: %s", start);
                                goto errorexit;
                            }

                            /* Point "start" to PV value argument. */
                            start = epicsStrtok_r(NULL, ",", &tail);
                            if (tail == NULL)
                            {
                                delay = 0.0;
                                tail = start;
                                start = epicsStrtok_r(NULL, ")", &tail);
                                if (tail == NULL)
                                    goto errorexit;
                            }
                            else
                            {
                                char *last;

                                last = epicsStrtok_r(NULL, ")", &tail);
                                if (last == NULL)
                                    goto errorexit;
                                delay = atof(last);
                            }
                            if (dbPutField(&addr, DBR_STRING, start, 1L))
                            {
                                errPrintf(-1, __FILE__, __LINE__, "invalid value: %s", start);
                                goto errorexit;
                            }
                            if (delay != 0.0)
                            {
                                if (delay > 10.0)
                                    delay = 10.0;
                                epicsThreadSleep(delay);
                            }
                        }

                        if (errind == true)
errorexit:                  errMessage(-1, "Invalid device directive");
                        end++;
                        strcpy(prem_buff, end);
                    }
                    else
                        strcpy(prem_buff, mr->prem);

                    strcat(motor_call->message, " ");
                    strcat(motor_call->message, prem_buff);
                    strcat(motor_call->message, " ");
                }
                if (strlen(mr->post) != 0)
                    motor_call->postmsgptr = (char *) &mr->post;
            }
            
            /* Code to hide Oms58 moving off limit switch problem. */
            msta.All = mr->msta;
            if ((msta.Bits.RA_PLUS_LS || msta.Bits.RA_MINUS_LS) &&
                (command == MOVE_ABS || command == MOVE_REL))
            {
                if (mr->rdif >= 0)
                    strcat(motor_call->message, " MP");
                else
                    strcat(motor_call->message, " MM");
            }

            /* put in command */
            strcat(motor_call->message, oms_table[command].command);
   
            /* put in parameters */
            for (first_one = YES, itera = 0; itera < oms_table[command].num_parms;
                 itera++)
            {
                if (first_one == YES)
                    first_one = NO;
                else
                    strcat(motor_call->message, ",");

                switch (command)
                {
                case SET_PGAIN:
                case SET_IGAIN:
                case SET_DGAIN:
                    sprintf(buffer, "%.1f", (parms[itera] * 1999.9));
                    break;

                case SET_VELOCITY:  /* OMS errors if VB = VL. */
                    {
                        long vbase, vel;

                        vbase = NINT(mr->vbas / fabs(mr->mres));
                        vel = NINT(parms[itera]);

                        if (vel <= vbase)
                        {
                            if (invalid_velmsg_latch == false)
                            {
                                invalid_velmsg_latch = true;  /* Ouput msg. one time. */
                                errPrintf(-1, __FILE__, __LINE__,
                                "Overriding invalid velocity; slew <= base.\n");
                            }
                            vel = vbase + 1;
                        }
                        sprintf(buffer, "%ld", vel);
                    }
                    break;

                case MOVE_REL:
                    {
                        /* Code around OMS bug that ignores MR+/-1. */
                        long relmove;

                        relmove = NINT(parms[itera]);
                        if (relmove == 1 || relmove == -1)
                            sprintf(buffer, "%ld.5", relmove);
                        else
                            sprintf(buffer, "%ld", relmove);
                    }
                    break;

                case SET_ACCEL: /* Prevent invalid acceleration values. */
                    {
                        long valid_acc = NINT(parms[itera]);

                        if (valid_acc < 1 || valid_acc > 1000000000)
                        {
                            errPrintf(-1, __FILE__, __LINE__,
                                      "Overriding invalid acceleration.\n");

                            if (valid_acc < 1)
                                valid_acc = 1;
                            else
                                valid_acc = 1000000000;
                        }
                        sprintf(buffer, "%ld", valid_acc);
                    }
                    break;

                default:
                    sprintf(buffer, "%ld", NINT(parms[itera]));
                }
                strcat(motor_call->message, buffer);
            }

            switch (command)
            {
            case SET_ENC_RATIO:
                sprintf(buffer, " UU%f", parms[0]/parms[1]);
                strcat(motor_call->message, buffer);
                break;

            case LOAD_POS:
            case JOG:
            case JOG_VELOCITY:
                strcat(motor_call->message, ";");

            default:
                break;   
            }       
        }
    }
    return(rtnind);
}


