/* File: devMicos.cc                    */

/* Device Support Routines for Micos MoCo dc motor controller. */
/*
 *      Original Author: Kurt Goetze
 *      Date: 11-24-2003
 *
 * Modification Log:
 * -----------------
 * .00  11-24-2003   kag  initialized from drvMCB4B.c
 * .01  02-06-2004   rls  Eliminate erroneous "Motor motion timeout ERROR".
 * .02  02-12-2004   rls  copied from devMicos.c; ported to R3.14.x
 */


#define VERSION 2.00

#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvMicos.h"
#include "epicsExport.h"

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int devMicosDebug = 0;
	#define Debug(l, f, args...) {if (l <= devMicosDebug) printf(f, ## args);}
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

/* Debugging levels: 
 *      devMicosDebug >= 3  Print new part of command and command string so far
 *                          at the end of Micos_build_trans
 */

extern struct driver_table Micos_access;

/* ----------------Create the dsets for devMicos----------------- */
static struct driver_table *drvtabptr;
static long Micos_init(void *);
static long Micos_init_record(void *);
static long Micos_start_trans(struct motorRecord *);
static RTN_STATUS Micos_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS Micos_end_trans(struct motorRecord *);

struct motor_dset devMicos =
{
    {8, NULL, (DEVSUPFUN) Micos_init, (DEVSUPFUN) Micos_init_record, NULL},
    motor_update_values,
    Micos_start_trans,
    Micos_build_trans,
    Micos_end_trans
};

epicsExportAddress(dset,devMicos);

/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static msg_types Micos_table[] = {
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
    IMMEDIATE   /* SET_LOW_LIMIT */
};


static struct board_stat **Micos_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for Micos DC motor */
static long Micos_init(void *arg)
{
    long rtnval;
    int after = (int) arg;

    Debug(5, "Micos_init: entry\n");
    if (after == 0)
    {
	drvtabptr = &Micos_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &Micos_cards);
    Debug(5, "Micos_init: exit\n");
    return(rtnval);
}


/* initialize a record instance */
static long Micos_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    long rtnval;

    Debug(5, "Micos_init_record: entry\n");
    rtnval = motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, 
                                   drvtabptr, Micos_cards);
    Debug(5, "Micos_init_record: exit\n");
    return(rtnval);
}


/* start building a transaction */
static long Micos_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
static RTN_STATUS Micos_end_trans(struct motorRecord *mr)
{
    return(OK);
}

/* add a part to the transaction */
static RTN_STATUS Micos_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct MicosController *cntrl;
    char buff[30];
    int axis, card;
    RTN_STATUS rtnval;
    double dval;   /* placeholder for double values passed from motor record */
    long ival;     /* placeholder for ints passed from motor record */

    rtnval = OK;
    buff[0] = '\0';
    dval = parms[0];
    ival = NINT(parms[0]);

    rtnval = (RTN_STATUS) motor_start_trans_com(mr, Micos_cards);
    Debug(5, "Micos_build_trans: entry, motor_start_trans_com=%ld\n", rtnval);

    motor_call = &(trans->motor_call);
    motor_call->type = Micos_table[command];
    card = motor_call->card;    /* card is the group of drivers per unique serial port */
    axis = motor_call->signal;  /* axis is Micos address, up to 16 (0-15) per serial port */
                                /* Note: Each Micos driver drives _1_ motor */
    brdptr = (*trans->tabptr->card_array)[card];
    Debug(5, "Micos_build_trans: axis=%d, command=%d\n", axis, command);
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct MicosController *) brdptr->DevicePrivate;


    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
        strcpy(motor_call->message, mr->init);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, Micos_cards);
        motor_call->type = Micos_table[command];
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
                rtnval = (RTN_STATUS) motor_start_trans_com(mr, Micos_cards);
                motor_call->type = Micos_table[command];
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
        sprintf(motor_call->message, "%c%dma%ld", CTLA, axis, ival);
        break;
    case MOVE_REL:
        sprintf(motor_call->message, "%c%dmr%ld", CTLA, axis, ival);
        break;
    case HOME_FOR:
        sprintf(motor_call->message, "%c%dgh", CTLA, axis);
        break;
    case HOME_REV:
        sprintf(motor_call->message, "%c%dgh", CTLA, axis);
        break;
    case LOAD_POS:        /* Micos allows you to define the zero position only */
        if (dval == 0.0)
            sprintf(motor_call->message, "%c%ddh,ud", CTLA, axis);
        else
            rtnval = ERROR;
        break;
    case SET_VEL_BASE:
        trans->state = IDLE_STATE;
        break;          /* Micos does not use base velocity */
    case SET_VELOCITY:
        if (ival < 0) ival = 0;
        if (ival > 1000000) ival = 1000000;
        sprintf(motor_call->message, "%c%ddv%ld,ud", CTLA, axis, ival);
        break;
    case SET_ACCEL:
        /* dval is acceleration in steps/sec/sec */
        if (ival < 0) ival = 0;
        if (ival > 1000000) ival = 1000000;
        sprintf(motor_call->message, "%c%dda%ld,ud", CTLA, axis, ival);
        break;
    case GO:
    /*
         * The Micos starts moving immediately on move commands, GO command
         * does nothing
         */
        trans->state = IDLE_STATE;
        break;
    case SET_ENC_RATIO:
    /*
         * The Micos does not have the concept of encoder ratio, ignore this
         * command
         */
        trans->state = IDLE_STATE;
        break;
    case GET_INFO:
        /* These commands are not actually done by sending a message, but
           rather they will indirectly cause the driver to read the status
           of all motors */
        break;
    case STOP_AXIS:   /* (decelerate to a) stop */
        sprintf(motor_call->message, "%c%dab1", CTLA, axis);
        break;
    case JOG:
    /* 
          * Micos does not have a jog command.  Simulate with move absolute
          * to the appropriate software limit.  The record will prevent JOG motion 
          * beyond its soft limits
          */
	if (dval > 0.)
	    sprintf(motor_call->message, "%c%dma%ld", CTLA, axis, (long)(mr->dhlm / mr->mres));
	else
            sprintf(motor_call->message, "%c%dma%ld", CTLA, axis, (long)(mr->dllm / mr->mres));
	break;

    case SET_PGAIN:
        if (ival < 0) ival = 0;
        if (ival > 32767) ival = 32767;
        sprintf(motor_call->message, "%c%ddp%ld,ud", CTLA, axis, ival);
        break;

    case SET_IGAIN:
        if (ival < 0) ival = 0;
        if (ival > 32767) ival = 32767;
        sprintf(motor_call->message, "%c%ddi%ld,ud", CTLA, axis, ival);
        break;

    case SET_DGAIN:
        if (ival < 0) ival = 0;
        if (ival > 32767) ival = 32767;
        sprintf(motor_call->message, "%c%ddd%ld,ud", CTLA, axis, ival);
        break;

    case ENABLE_TORQUE:
        sprintf(motor_call->message, "%c%dmn", CTLA, axis);
        break;

    case DISABL_TORQUE:
        sprintf(motor_call->message, "%c%dmf", CTLA, axis);
        break;

    case SET_HIGH_LIMIT:
    case SET_LOW_LIMIT:
        trans->state = IDLE_STATE;
        break;

    default:
        rtnval = ERROR;
    }

    rtnval = motor_end_trans_com(mr, drvtabptr);
    Debug(5, "Micos_send_msg: motor_end_trans_com status=%ld, exit\n", rtnval); 
    return (rtnval);
}
