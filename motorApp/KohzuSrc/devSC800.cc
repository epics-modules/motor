/*
FILENAME...     devSC800.cc
USAGE...        Motor record device level support for Kohzu SC800 motor controller.


*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 11/08/2007
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
 * .01 11-08-07 rls copied from devMDT695.cc
 */


#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <errlog.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvSC800.h"
#include "epicsExport.h"

extern struct driver_table SC800_access;

/* ----------------Create the dsets for devSC800----------------- */
static struct driver_table *drvtabptr;
static long SC800_init(void *);
static long SC800_init_record(void *);
static long SC800_start_trans(struct motorRecord *);
static RTN_STATUS SC800_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS SC800_end_trans(struct motorRecord *);
static RTN_STATUS write_parms(char *, int, struct SC800Controller *,
                              struct mess_node *, motor_cmnd,
                              struct motorRecord *);

struct motor_dset devSC800 =
{
    {8, NULL, (DEVSUPFUN) SC800_init, (DEVSUPFUN) SC800_init_record, NULL},
    motor_update_values,
    SC800_start_trans,
    SC800_build_trans,
    SC800_end_trans
};

extern "C" {epicsExportAddress(dset,devSC800);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types SC800_table[] = {
    MOTION,     /* MOVE_ABS */
    MOTION,     /* MOVE_REL */
    MOTION,     /* HOME_FOR */
    MOTION,     /* HOME_REV */
    IMMEDIATE,  /* LOAD_POS */
    IMMEDIATE,  /* SET_VEL_BASE */
    IMMEDIATE,  /* SET_VELOCITY */
    IMMEDIATE,  /* SET_ACCEL */
    IMMEDIATE,  /* GO */
    IMMEDIATE,  /* SET_ENC_RATIO */
    INFO,       /* GET_INFO */
    MOVE_TERM,  /* STOP_AXIS */
    VELOCITY,   /* JOG */
    IMMEDIATE,  /* SET_PGAIN */
    IMMEDIATE,  /* SET_IGAIN */
    IMMEDIATE,  /* SET_DGAIN */
    IMMEDIATE,  /* ENABLE_TORQUE */
    IMMEDIATE,  /* DISABL_TORQUE */
    IMMEDIATE,  /* PRIMITIVE */
    IMMEDIATE,  /* SET_HIGH_LIMIT */
    IMMEDIATE,  /* SET_LOW_LIMIT */
    VELOCITY,   /* JOG_VELOCITY */
    IMMEDIATE   /* SET_RESOLUTION */
};


static struct board_stat **SC800_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for SC800 stepper motor */
static long SC800_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
        drvtabptr = &SC800_access;
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &SC800_cards);
    return(rtnval);
}


/* initialize a record instance */
static long SC800_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, SC800_cards));
}


/* start building a transaction */
static long SC800_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
static RTN_STATUS SC800_end_trans(struct motorRecord *mr)
{
    return(OK);
}


/* add a part to the transaction */
static RTN_STATUS SC800_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    // struct mess_info *motor_info;
    struct SC800Controller *cntrl;
    char buff[110], polarity, *pbuff;
    int signal, card, intval, axis;
    double dval, cntrl_units;
    unsigned int size;
    RTN_STATUS rtnval;
    bool send;

    send = true;		/* Default to send motor command. */
    rtnval = OK;
    pbuff = &buff[0];
    *pbuff = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    intval = (parms == NULL) ? 0 : NINT(parms[0]);
    dval = (parms == NULL) ? 0 : *parms;

    motor_start_trans_com(mr, SC800_cards);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    signal = motor_call->signal;
    axis = signal + 1;

    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct SC800Controller *) brdptr->DevicePrivate;
    cntrl_units = dval;


    if (SC800_table[command] > motor_call->type)
        motor_call->type = SC800_table[command];

    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
        strcat(motor_call->message, mr->init);
        strcat(motor_call->message, SC800_OUT_EOS);
    }

    switch (command)
    {
        case MOVE_ABS:
        case MOVE_REL:
        case HOME_FOR:
        case HOME_REV:
        case JOG:
            if (strlen(mr->prem) != 0)
            {
                strcat(motor_call->message, mr->prem);
                strcat(motor_call->message, SC800_OUT_EOS);
    
            }
            if (strlen(mr->post) != 0)
                motor_call->postmsgptr = (char *) &mr->post;
            break;
    
        default:
            break;
    }

    switch (command)
    {
        case MOVE_REL:
            if (cntrl->slew_speed[signal] == NINT(mr->bvel / fabs(mr->mres)))
                cntrl->accl_rate[signal] = NINT(mr->bacc * 100.0);  /* Backlash accel. rate. */
            else
                cntrl->accl_rate[signal] = NINT(mr->accl * 100.0);  /* Slew accel. rate. */

            rtnval = write_parms(pbuff, signal, cntrl, motor_call, command, mr);
            
            sprintf(buff, "RPS%d/2/1/0/%d/0/0/1", axis, intval);
            
            break;
        case MOVE_ABS:
            if (cntrl->slew_speed[signal] == NINT(mr->bvel / fabs(mr->mres)))
                cntrl->accl_rate[signal] = NINT(mr->bacc * 100.0);  /* Backlash accel. rate. */
            else
                cntrl->accl_rate[signal] = NINT(mr->accl * 100.0);  /* Slew accel. rate. */

            rtnval = write_parms(pbuff, signal, cntrl, motor_call, command, mr);
            
            sprintf(buff, "APS%d/2/0/0/%d/0/0/1", axis, intval);
            break;
    
        case HOME_FOR:
        case HOME_REV:
            rtnval = write_parms(pbuff, signal, cntrl, motor_call, command, mr);

            sprintf(buff, "ORG%d/2/0/0/3/1", axis);
            break;
    
        case LOAD_POS:
            sprintf(buff, "WRP%d/%d", axis, intval);
            break;
    
        case SET_VEL_BASE:
            if (intval < 1) /* Validity check. */
                intval = 1;
            else if (intval > 4095500)
                intval = 4095500;
            cntrl->base_speed[signal] = intval;  /* Save for later. */
	    send = false;
            break;
    
        case SET_VELOCITY:
            if (intval < 1) /* Validity check. */
                intval = 1;
            else if (intval > 4095500)
                intval = 4095500;
            cntrl->slew_speed[signal] = intval;  /* Save for later. */
	    send = false;
            break;
    
        case SET_ACCEL:
            cntrl->accl_rate[signal] = intval;
	    send = false;
            break;
    
        case GO:
	    send = false;
            break;
    
        case GET_INFO:
            break;
    
        case STOP_AXIS:
            sprintf(buff, "STP%d/0", axis);
            break;
    
        case JOG:
            polarity = (intval > 0) ? '1' : '0';
            intval = abs(intval);
            if (intval < 1) /* Validity check. */
                intval = 1;
            else if (intval > 4095500)
                intval = 4095500;
            cntrl->slew_speed[signal] = intval;
            if (cntrl->base_speed[signal] > cntrl->slew_speed[signal])
                cntrl->base_speed[signal] = cntrl->slew_speed[signal];

            /* Calculate Jog accel/decel time. */
            cntrl->accl_rate[signal] = NINT(((mr->jvel - mr->vbas) / mr->jar) * 100.0);

            rtnval = write_parms(pbuff, signal, cntrl, motor_call, command, mr);
    
            sprintf(buff, "FRP%d/2/0/0/%c/1", axis, polarity);
            break;
    
        case ENABLE_TORQUE:
            sprintf(buff, "COF%d/0", axis);
            break;
        
        case DISABL_TORQUE:
            sprintf(buff, "COF%d/1", axis);
            break;
    
        case JOG_VELOCITY:
        case SET_ENC_RATIO:
        case SET_HIGH_LIMIT:
        case SET_LOW_LIMIT:
        case SET_RESOLUTION:
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */
	    send = false;
            break;
    
        case SET_PGAIN:
        case SET_IGAIN:
        case SET_DGAIN:
        default:
	    send = false;
            rtnval = ERROR;
    }

    size = strlen(buff);
    if (send == false)
	return(rtnval);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
        errlogMessage("SC800_build_trans(): buffer overflow.\n");
    else
    {
        strcat(motor_call->message, buff);
        motor_end_trans_com(mr, drvtabptr);
    }

    return(rtnval);
}

static RTN_STATUS write_parms(char *buff, int signal,
                              struct SC800Controller *cntrl,
                              struct mess_node *motor_call, motor_cmnd command,
                              struct motorRecord *mr)
{
    RTN_STATUS rtnval;
    int axis = signal + 1;

    sprintf(buff, "ASI%d/%d/%d/%d/%d/0/0/0/0/0/0/0/0/0", axis,
            cntrl->base_speed[signal], cntrl->slew_speed[signal],
            cntrl->accl_rate[signal], cntrl->accl_rate[signal]);
    strcpy(motor_call->message, buff);
    rtnval = motor_end_trans_com(mr, drvtabptr);
    rtnval = (RTN_STATUS) motor_start_trans_com(mr, SC800_cards);
    motor_call->type = SC800_table[command];
    return(rtnval);
}

