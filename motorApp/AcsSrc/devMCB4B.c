/* File: devMCB4B.c                     */

/* Device Support Routines for motor */
/*
 *      Original Author: Mark Rivers
 *      Date: 02-24-2002
 *
 * Modification Log:
 * -----------------
 * .00  02-24-2002      mlr     initialized from devPM304.c
 */


#define VERSION 1.00

#include        <vxWorks.h>
#include        <stdioLib.h>
#include        <string.h>
#include        <semLib.h>
#include        <math.h>

#ifdef __cplusplus
extern "C" {
#include        <epicsDynLink.h>
}
#else
#include        <epicsDynLink.h>
#endif
#include        <sysSymTbl.h>   /* for sysSymTbl*/

#include        <alarm.h>
#include        <callback.h>
#include        <dbDefs.h>
#include        <dbAccess.h>
#include        <dbCommon.h>
#include        <fast_lock.h>
#include        <recSup.h>
#include        <devSup.h>
#include        <drvSup.h>

#include        "motorRecord.h"
#include        "motor.h"
#include        "motordevCom.h"
#include        "drvMCB4B.h"

#define STATIC static

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

#ifdef NODEBUG
#define Debug(L,FMT,V) ;
#else
#define Debug(L,FMT,V...) {  if(L <= devMCB4BDebug) \
                        { errlogPrintf("%s(%d):",__FILE__,__LINE__); \
                          errlogPrintf(FMT,##V); } }
#endif

/* Debugging levels: 
 *      devMCB4BDebug >= 3  Print new part of command and command string so far
 *                          at the end of MCB4B_build_trans
 */


/* ----------------Create the dsets for devMCB4B----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long MCB4B_init(int);
STATIC long MCB4B_init_record(struct motorRecord *);
STATIC long MCB4B_start_trans(struct motorRecord *);
STATIC long MCB4B_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC long MCB4B_end_trans(struct motorRecord *);

struct motor_dset devMCB4B =
{
    {8, NULL, MCB4B_init, MCB4B_init_record, NULL},
    motor_update_values,
    MCB4B_start_trans,
    MCB4B_build_trans,
    MCB4B_end_trans
};



/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static int MCB4B_table[] = {
    MOTION,     /* MOVE_ABS */
    MOTION,     /* MOVE_REL */
    MOTION,     /* HOME_FOR */
    MOTION,     /* HOME_REV */
    IMMEDIATE,  /* LOAD_POS */
    IMMEDIATE,  /* SET_VEL_BASE */
    IMMEDIATE,  /* SET_VELOCITY */
    IMMEDIATE,  /* SET_ACCEL */
    IMMEDIATE,  /* GO */
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


static struct board_stat **MCB4B_cards;

volatile int devMCB4BDebug = 0;

/* --------------------------- program data --------------------- */


/* initialize device support for MCB4B stepper motor */
STATIC long MCB4B_init(int after)
{
    SYM_TYPE type;
    long rtnval;

    
    Debug(5, "MCB4B_init: entry\n");
    if (after == 0)
    {
        rtnval = symFindByNameEPICS(sysSymTbl, "_MCB4B_access",
                    (void *) &drvtabptr, &type);
        if (rtnval != OK)
            return(rtnval);
    /*
        IF before DB initialization.
            Initialize MCB4B driver (i.e., call init()). See comment in
                drvMCB4B.c init().
        ENDIF
    */
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MCB4B_cards);
    Debug(5, "MCB4B_init: exit\n");
    return(rtnval);
}


/* initialize a record instance */
STATIC long MCB4B_init_record(struct motorRecord *mr)
{
    long rtnval;

    Debug(5, "MCB4B_init_record: entry\n");
    rtnval = motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, 
                                   drvtabptr, MCB4B_cards);
    return(rtnval);
    Debug(5, "MCB4B_init_record: exit\n");
}


/* start building a transaction */
STATIC long MCB4B_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
STATIC long MCB4B_end_trans(struct motorRecord *mr)
{
    return(OK);
}

/* add a part to the transaction */
STATIC long MCB4B_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct MCB4Bcontroller *cntrl;
    char buff[30];
    int axis, card;
    long rtnval;
    double dval;
    long ival;

    rtnval = OK;
    buff[0] = '\0';
    dval = parms[0];
    ival = NINT(parms[0]);

    rtnval = motor_start_trans_com(mr, MCB4B_cards);
    Debug(5, "MCB4B_build_trans: entry, motor_start_trans_com=%ld\n", rtnval);

    motor_call = &(trans->motor_call);
    motor_call->type = MCB4B_table[command];
    card = motor_call->card;
    axis = motor_call->signal;
    brdptr = (*trans->tabptr->card_array)[card];
    Debug(5, "MCB4B_build_trans: axis=%d, command=%d\n", axis, command);
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct MCB4Bcontroller *) brdptr->DevicePrivate;


    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
        strcpy(motor_call->message, mr->init);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = motor_start_trans_com(mr, MCB4B_cards);
        motor_call->type = MCB4B_table[command];
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
                rtnval = motor_start_trans_com(mr, MCB4B_cards);
                motor_call->type = MCB4B_table[command];
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
        sprintf(motor_call->message, "#%02dG%+ld", axis, ival);
        break;
    case MOVE_REL:
        sprintf(motor_call->message, "#%02dI%+ld", axis, ival);
        break;
    case HOME_FOR:
        sprintf(motor_call->message, "#%02dH+", axis);
        break;
    case HOME_REV:
        sprintf(motor_call->message, "#%02dH-", axis);
        break;
    case LOAD_POS:
        sprintf(motor_call->message, "#%02dP=%+ld", axis, ival);
        break;
    case SET_VEL_BASE:
        trans->state = IDLE_STATE;
        break;          /* MCB4B does not use base velocity */
    case SET_VELOCITY:
        ival = fabs(115200./dval) + 0.5;
        if (ival < 2) ival=2;
        if (ival > 255) ival = 255;
        sprintf(motor_call->message, "#%02dV=%ld", axis, ival);
        break;
    case SET_ACCEL:
        /* dval is acceleration in steps/sec/sec */
        /* MCB is programmed with Ramp Index (R) where: */
        /* dval (steps/sec/sec) = 720,000/(256-R) */
        /* or R=256-(720,000/dval) */
        ival = 256-(720000./dval)+0.5;
        if (ival < 1) ival=1;
        if (ival > 255) ival=255;
        sprintf(motor_call->message, "#%02dR=%ld", axis, ival);
        break;
    case GO:
        /*
         * The MCB4B starts moving immediately on move commands, GO command
         * does nothing
         */
        trans->state = IDLE_STATE;
        break;
    case GET_INFO:
        /* These commands are not actually done by sending a message, but
           rather they will indirectly cause the driver to read the status
           of all motors */
        break;
    case STOP_AXIS:
        sprintf(motor_call->message, "#%02dQ", axis);
        break;
    case JOG:
        /* MCB-4B does not have jog command. Move 1 million steps */
        ival = fabs(115200./dval) + 0.5;
        if (ival < 2) ival=2;
        if (ival > 65535) ival = 65535;
        sprintf(motor_call->message, "#%02dC=%ld", axis, ival);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = motor_start_trans_com(mr, MCB4B_cards);
        motor_call->type = MCB4B_table[command];
        if (dval > 0.) {
            /* This is a positive move in MCB4B coordinates */
            sprintf(motor_call->message, "#%02dM+1000000", axis);
        } else {
            /* This is a negative move in MCB4B coordinates */
            sprintf(motor_call->message, "#%02dM-1000000", axis);
        }
        break;
    case SET_PGAIN:
        trans->state = IDLE_STATE;
        break;

    case SET_IGAIN:
        trans->state = IDLE_STATE;
        break;

    case SET_DGAIN:
        trans->state = IDLE_STATE;
        break;

    case ENABLE_TORQUE:
        sprintf(motor_call->message, "#%02dW=1", axis);
        break;

    case DISABL_TORQUE:
        sprintf(motor_call->message, "#%02dW=0", axis);
        break;

    case SET_HIGH_LIMIT:
    case SET_LOW_LIMIT:
        trans->state = IDLE_STATE;
        break;

    default:
        rtnval = ERROR;
    }

    rtnval = motor_end_trans_com(mr, drvtabptr);
    Debug(5, "MCB4B_send_msg: motor_end_trans_com status=%ld, exit\n", rtnval); 
    return (rtnval);
}
