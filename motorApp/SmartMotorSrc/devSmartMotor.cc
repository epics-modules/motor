/*
FILENAME... devSmartMotor.cc
USAGE...    Motor record driver level support for Animatics Corporation SmartMotors.

Version:        $Revision: 1.2 $
Modified By:    $Author: sluiter $
Last Modified:  $Date: 2008-03-14 20:21:56 $
*/

/*
 *      Original Author: Shifu Xu 
 *      Date: 03/21/03
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
 * NOTES
 * -----
 * - the velocity and acceleration conversion factors are based on the default
 *   PID update rate of 4,068.969 updates per second.
 *
 * Modification Log:
 * -----------------
 * .01 02/20/07 ses copied from drvSmartMotorPL.c
 * .02 04/18/07 rls fix velocity and acceleration conversion factors.
 */

#include <string.h>

#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvSmartMotor.h"
#include "epicsExport.h"

/*
 * SmartMotor velocity command (V) units; scaled encoder counts / PID sample.
 * VCONFAC converts encoder counts / sec. to scaled encoder counts / PID sample.
 * VCONFAC = (65,536 s-e-cts / 1 e-ct) * (1s / 4068.969 PID samples)
 */
 
#define VCONFAC 16.1063

/*
 * SmartMotor acceleration command (A) units; scaled encoder counts / PID sample^2.
 * ACONFAC converts encoder counts / sec.^2 to scaled encoder counts / PID sample^2.
 * ACONFAC = (65,536 s-e-cts / 1 e-ct) * (1s / 4068.969^2 PID samples)
 */

#define ACONFAC 3.958322E-3


extern struct driver_table SmartMotor_access;

/* ----------------Create the dsets for devSmartMotor----------------- */
static struct driver_table *drvtabptr;
static long SmartMotor_init(void *);
static long SmartMotor_init_record(void *);
static long SmartMotor_start_trans(struct motorRecord *);
static RTN_STATUS SmartMotor_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS SmartMotor_end_trans(struct motorRecord *);

struct motor_dset devSmartMotor =
{
    {8, NULL, (DEVSUPFUN) SmartMotor_init, (DEVSUPFUN) SmartMotor_init_record, NULL},
    motor_update_values,
    SmartMotor_start_trans,
    SmartMotor_build_trans,
    SmartMotor_end_trans
};

extern "C"
{
    epicsExportAddress(dset, devSmartMotor);
}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types SmartMotor_table[] = {
    MOTION,      /* MOVE_ABS */
    MOTION,      /* MOVE_REL */
    MOTION,      /* HOME_FOR */
    MOTION,      /* HOME_REV */
    IMMEDIATE,   /* LOAD_POS */
    IMMEDIATE,   /* SET_VEL_BASE */
    IMMEDIATE,   /* SET_VELOCITY */
    IMMEDIATE,   /* SET_ACCEL */
    IMMEDIATE,   /* GO */
    IMMEDIATE,   /* SET_ENC_RATIO */
    INFO,        /* GET_INFO */
    MOVE_TERM,   /* STOP_AXIS */
    VELOCITY,    /* JOG */
    IMMEDIATE,   /* SET_PGAIN */
    IMMEDIATE,   /* SET_IGAIN */
    IMMEDIATE,   /* SET_DGAIN */
    IMMEDIATE,   /* ENABLE_TORQUE */
    IMMEDIATE,   /* DISABL_TORQUE */
    IMMEDIATE,   /* PRIMITIVE */
    IMMEDIATE,   /* SET_HIGH_LIMIT */
    IMMEDIATE,   /* SET_LOW_LIMIT */
    VELOCITY     /* JOG_VELOCITY */
};


static struct board_stat **SmartMotor_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for SmartMotor stepper motor */
static long SmartMotor_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
        drvtabptr = &SmartMotor_access;
        (drvtabptr->init) ();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &SmartMotor_cards);
    return(rtnval);
}


/* initialize a record instance */
static long SmartMotor_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, SmartMotor_cards));
}


/* start building a transaction */
static long SmartMotor_start_trans(struct motorRecord * mr)
{
    return(OK);
}


/* end building a transaction */
static RTN_STATUS SmartMotor_end_trans(struct motorRecord * mr)
{
    return(OK);
}


/* add a part to the transaction */
static RTN_STATUS SmartMotor_build_trans(motor_cmnd command, double *parms, struct motorRecord * mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct SmartMotorcontroller *cntrl;
    char buff[110];
    int card, intval, cvel, cacc;
    unsigned int size;
    double dval;
    RTN_STATUS rtnval;
    bool send;
    msta_field msta;
    static bool invalid_accmsg_latch = false;

    send = true;        /* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;
    intval = NINT(dval);
    cvel = NINT(dval * VCONFAC);
    cacc = NINT(dval * ACONFAC);

    msta.All = mr->msta;

    motor_start_trans_com(mr, SmartMotor_cards);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct SmartMotorcontroller *) brdptr->DevicePrivate;

    if (SmartMotor_table[command] > motor_call->type)
        motor_call->type = SmartMotor_table[command];

    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
        strcat(motor_call->message, " ");
        strcat(motor_call->message, mr->init);
    }

    switch (command)
    {
    case MOVE_ABS:
        sprintf(buff, "P=%d", intval);
        break;

    case MOVE_REL:
        sprintf(buff, "D=%d", intval);
        break;

    case HOME_FOR:
        /* Animatics SmartMotors do not use home positions */
        send = false;
        rtnval = ERROR;
        break;

    case HOME_REV:
        /* Animatics SmartMotors do not use home positions */
        send = false;
        rtnval = ERROR;
        break;

    case LOAD_POS:
        sprintf(buff, "O=%d", intval);
        break;

    case SET_VEL_BASE:
        /* Animatics SmartMotors do not support stepper motors. */
        send = false;
        rtnval = ERROR;
        break;

    case SET_VELOCITY:
        sprintf(buff, "V=%d", cvel);
        break;

    case SET_ACCEL:
        if (cacc <= 1)
        {
            cacc = 2;
            if (invalid_accmsg_latch == false)
            {
                invalid_accmsg_latch = true;  /* Ouput msg. one time. */
                errPrintf(-1, __FILE__, __LINE__,
                "Overriding invalid acceleration; A < 2.\n");
            }
        }
        sprintf(buff, "A=%d", cacc);
        break;

    case GO:
        sprintf(buff, "G");
        break;

    case PRIMITIVE:
        /* Animatics SmartMotors do not use 'primatives' */
        send = false;
        rtnval = ERROR;
        break;

    case GET_INFO:
        break;

    case STOP_AXIS:
        sprintf(buff, "S");
        break;

    case JOG_VELOCITY:
        send = false;
        rtnval = ERROR;
        break;

    case JOG:
        sprintf(buff, "MV\rV=%d\rG", cvel);
        break;

    case SET_PGAIN:
        send = false;
        rtnval = ERROR;
        break;

    case SET_IGAIN:
        send = false;
        rtnval = ERROR;
        break;

    case SET_DGAIN:
        send = false;
        rtnval = ERROR;
        break;

    case ENABLE_TORQUE:
        sprintf(buff, "MP\ra=@P\rP=a\rG");
        break;

    case DISABL_TORQUE:
        sprintf(buff, "OFF");
        break;

    case SET_HIGH_LIMIT:
        send = false;
        rtnval = ERROR;
        break;

    case SET_LOW_LIMIT:
        send = false;
        rtnval = ERROR;
        break;

    case SET_ENC_RATIO:
        trans->state = IDLE_STATE;  /* No command sent to the controller. */
        send = false;
        break;

    default:
        send = false;
        rtnval = ERROR;
    }

    size = strlen(buff);
    if (send == false)
        return(rtnval);
    else if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
        errlogMessage("SmartMotor_build_trans(): buffer overflow.\n");
    else
    {
        strcat(motor_call->message, buff);
        motor_end_trans_com(mr, drvtabptr);
    }
    return(rtnval);
}

