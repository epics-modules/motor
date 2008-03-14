/* File: devPIC630.cc                     */

/* Device Support Routines for motor */
/*
 *      Original Author: Kurt Goetze
 *      Date: 02-07-2005
 *
 * Modification Log:
 * -----------------
 * .00  02-07-2005      kag     initialized from devMicos.c
 */

/* Notes regarding the motor record / PI C-630: 
 * 1. JogF and JogR are implemented as moves to the corresponding soft limit.
 * 2. HomeF and HomeR will simply move the positioner to the "0" position.
 * 3. The only position you may "set" is 0.  Others will be ignored.
 *
 */
 
#include <string.h>
#include <math.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPIC630.h"
#include "epicsExport.h"

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef DEBUG
        volatile int devPIC630debug = 0;
        #define Debug(L, FMT, V...) { if(L <= devPIC630debug) \
                { errlogPrintf("%s(%d):",__FILE__,__LINE__); \
                  errlogPrintf(FMT,##V); } }
    #else
        #define Debug(L, FMT, V...)
    #endif
#else
    #define Debug()
#endif

/* Debugging levels: 
 *      devPIC630debug >= 3  Print new part of command and command string so far
 *                          at the end of PIC630_build_trans
 */

extern struct driver_table PIC630_access;

/* ----------------Create the dsets for devPIC630----------------- */
static struct driver_table *drvtabptr;
static long PIC630_init(void *);
static long PIC630_init_record(void *);
static long PIC630_start_trans(struct motorRecord *);
static RTN_STATUS PIC630_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS PIC630_end_trans(struct motorRecord *);

struct motor_dset devPIC630 =
{
    {8, NULL, (DEVSUPFUN) PIC630_init, (DEVSUPFUN) PIC630_init_record, NULL},
    motor_update_values,
    PIC630_start_trans,
    PIC630_build_trans,
    PIC630_end_trans
};

extern "C" {epicsExportAddress(dset,devPIC630);}

/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static msg_types PIC630_table[] = {
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
    VELOCITY    /* JOG_VELOCITY */
};


static struct board_stat **PIC630_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PIC630 stepper motor */
static long PIC630_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    Debug(5, "PIC630_init: entry\n");
    if (after == 0)
    {
        drvtabptr = &PIC630_access;
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PIC630_cards);
    Debug(5, "PIC630_init: exit\n");
    return(rtnval);
}


/* initialize a record instance */
static long PIC630_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PIC630_cards));
}


/* start building a transaction */
static long PIC630_start_trans(struct motorRecord *mr)
{
    motor_start_trans_com(mr, PIC630_cards);
    return(OK);
}


/* end building a transaction */
static RTN_STATUS PIC630_end_trans(struct motorRecord *mr)
{
    motor_end_trans_com(mr, drvtabptr);
    return(OK);
}

/* add a part to the transaction */
static RTN_STATUS PIC630_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct PIC630Controller *cntrl;
    char buff[30];
    int axis, card;
    RTN_STATUS rtnval;
    bool send;
    double dval;   /* placeholder for double values passed from motor record */
    long ival;     /* placeholder for ints passed from motor record */

    send = true;   /* default to send motor command */
    rtnval = OK;
    buff[0] = '\0';
    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;

    ival = NINT(dval);

    rtnval = (RTN_STATUS) motor_start_trans_com(mr, PIC630_cards); 
    Debug(5, "PIC630_build_trans: entry, motor_start_trans_com=%d\n", rtnval);

    motor_call = &(trans->motor_call);
    motor_call->type = PIC630_table[command];
    card = motor_call->card;    /* card is the group of drivers per unique serial port */
    axis = motor_call->signal;  /* axis is PIC630 address, up to 9 (1-9) per serial port */
    axis++;                            /* Note: Each PIC630 driver drives up to 3 motors */
    brdptr = (*trans->tabptr->card_array)[card];
    Debug(5, "PIC630_build_trans: axis=%d, command=%d\n", axis, command);
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct PIC630Controller *) brdptr->DevicePrivate;


    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
        strcpy(motor_call->message, mr->init);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, PIC630_cards);
        motor_call->type = PIC630_table[command];
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
                strcpy(motor_call->message, mr->prem);
		rtnval = motor_end_trans_com(mr, drvtabptr);
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, PIC630_cards);
                motor_call->type = PIC630_table[command];
            }
            if (strlen(mr->post) != 0)
                motor_call->postmsgptr = (char *) &mr->post;
            break;

        default:
            break;
    }

    switch (command)
    {
    case MOVE_ABS:
        sprintf(motor_call->message, "%dMA%ld", axis, ival);
        break;
    case MOVE_REL:
        sprintf(motor_call->message, "%dMR%ld", axis, ival);
        break;
    case HOME_FOR:
        sprintf(motor_call->message, "%dMA0", axis);
        break;
    case HOME_REV:
        sprintf(motor_call->message, "%dMA0", axis);
        break;
    case LOAD_POS:        /* PIC630 allows you to define the zero position only */
        if (dval == 0.0)
            sprintf(motor_call->message, "%dDH", axis);
        else
            rtnval = ERROR;
        break;
    case SET_VEL_BASE:
	    send = false;
        trans->state = IDLE_STATE;
        break;          /* PIC630 does not use base velocity */
    case SET_VELOCITY:
        if (ival < 1) ival = 1;
        if (ival > 200000) ival = 200000;
        sprintf(motor_call->message, "%dSV%ld", axis, ival);
        break;
    case SET_ACCEL:
        /* dval is acceleration in steps/sec/sec */
        if (ival < 0) ival = 0;
        if (ival > 500000) ival = 500000;
        sprintf(motor_call->message, "%dSA%ld", axis, ival);
        break;
    case GO:
    /*
         * The PIC630 starts moving immediately on move commands, GO command
         * does nothing
         */
		send = false;
        trans->state = IDLE_STATE;
        break;
    case SET_ENC_RATIO:
    /*
         * The PIC630 does not have the concept of encoder ratio, ignore this
         * command
         */
		send = false;
        trans->state = IDLE_STATE;
        break;
    case GET_INFO:
        /* ? what is this for ? */
        break;
    case STOP_AXIS:   /* (decelerate to a) stop */
        sprintf(motor_call->message, "%dST", axis);
        break;
    case JOG:
    /* 
          * PIC630 does not have a jog command.  Simulate with move absolute
          * to the appropriate software limit.  The record will prevent JOG motion 
          * beyond its soft limits
          * First we send the Jog velocity...
          */
        ival = (long) fabs((double) ival);
        if (ival < 1) ival = 1;
        if (ival > 200000) ival = 200000;
        sprintf(motor_call->message, "%dSV%ld", axis, ival);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, PIC630_cards);
        motor_call->type = PIC630_table[command];
	if (dval > 0.) /* jog pos */
	    sprintf(motor_call->message, "%dMA%ld", axis, (long)(mr->dhlm / mr->mres));
	else /* jog neg */
            sprintf(motor_call->message, "%dMA%ld", axis, (long)(mr->dllm / mr->mres));
	break;

    case SET_PGAIN:
        break;

    case SET_IGAIN:
        break;

    case SET_DGAIN:
        break;

    case ENABLE_TORQUE:
        break;

    case DISABL_TORQUE:
        break;

    case SET_HIGH_LIMIT:
    case SET_LOW_LIMIT:
        trans->state = IDLE_STATE;
        break;

    default:
	    send = false;
        rtnval = ERROR;
    }

    if (send == false)
        return(rtnval);
    else
    {
        rtnval = motor_end_trans_com(mr, drvtabptr);
        Debug(5, "PIC630_send_msg: motor_end_trans_com status=%d, exit\n", rtnval);
        return(rtnval);
    }
}
