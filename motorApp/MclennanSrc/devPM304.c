/* File: devPM304.c                     */
/* Version: 2.00                        */
/* Date Last Modified: 09/29/99         */

/* Device Support Routines for motor */
/*
 *      Original Author: Mark Rivers
 *      Date: 11/20/98
 *
 * Modification Log:
 * -----------------
 * .00  11-20-99        mlr     initialized from devMM4000.c
 * .01  09-29-99        mlr     Version 2.0, compatible with V4.04 of 
 *                              motorRecord
 * .02  10-26-99        mlr     Version 2.01, minor fixes for V4.0 of 
 *                              motorRecord
 *      ...
 */


#define VERSION 2.01

#include        <vxWorks.h>
#include        <stdioLib.h>
#include        <string.h>
#include        <semLib.h>      /* jps: include for init_record wait */

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
#include        "drvPM304.h"

#define STATIC static

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

#ifdef NODEBUG
#define Debug(L,FMT,V) ;
#else
#define Debug(L,FMT,V...) {  if(L <= devPM304Debug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,##V); } }
#endif

/* Debugging levels: 
 *      devPM304Debug >= 3  Print new part of command and command string so far
 *                          at the end of PM304_build_trans
 */


/* ----------------Create the dsets for devPM304----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long PM304_init(int);
STATIC long PM304_init_record(struct motorRecord *);
STATIC long PM304_start_trans(struct motorRecord *);
STATIC long PM304_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC long PM304_end_trans(struct motorRecord *);

struct motor_dset devPM304 =
{
    {8, NULL, PM304_init, PM304_init_record, NULL},
    motor_update_values,
    PM304_start_trans,
    PM304_build_trans,
    PM304_end_trans
};



/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static int PM304_table[] = {
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


static struct board_stat **PM304_cards;

volatile int devPM304Debug = 0;

/* --------------------------- program data --------------------- */


/* initialize device support for PM304 stepper motor */
STATIC long PM304_init(int after)
{
    SYM_TYPE type;
    long rtnval;

    if (after == 0)
    {
        rtnval = symFindByNameEPICS(sysSymTbl, "_PM304_access",
                    (void *) &drvtabptr, &type);
        if (rtnval != OK)
            return(rtnval);
    /*
        IF before DB initialization.
            Initialize PM304 driver (i.e., call init()). See comment in
                drvPM304.c init().
        ENDIF
    */
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PM304_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long PM304_init_record(struct motorRecord *mr)
{
    long rtnval;

    rtnval = motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, 
                                   drvtabptr, PM304_cards);
    return(rtnval);
}


/* start building a transaction */
STATIC long PM304_start_trans(struct motorRecord *mr)
{
    long rtnval;
    rtnval = motor_start_trans_com(mr, PM304_cards);
    return(rtnval);
}


/* end building a transaction */
STATIC long PM304_end_trans(struct motorRecord *mr)
{
    long rtnval;
    rtnval = motor_end_trans_com(mr, drvtabptr);
    return(rtnval);
    
}


/* add a part to the transaction */
STATIC long PM304_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct PM304controller *cntrl;
    char buff[30];
    int axis, card;
    long rtnval;
    double dval;
    long ival;

    rtnval = OK;
    buff[0] = '\0';
    dval = parms[0];
    ival = NINT(parms[0]);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct PM304controller *) brdptr->DevicePrivate;

    if (PM304_table[command] > motor_call->type)
        motor_call->type = PM304_table[command];

    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
        strcat(motor_call->message, mr->init);
        strcat(motor_call->message, "\r");
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
                strcat(motor_call->message, ";");
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
        sprintf(buff, "%dMA%ld;", axis, ival);
        break;
    case MOVE_REL:
        sprintf(buff, "%dMR%ld;", axis, ival);
        break;
    case HOME_FOR:
        sprintf(buff, "%dIX;", axis);
        break;
    case HOME_REV:
        sprintf(buff, "%dIX-1;", axis);
        break;
    case LOAD_POS:
        sprintf(buff, "%dAP%ld;", axis, ival);
        break;
    case SET_VEL_BASE:
        break;          /* PM304 does not use base velocity */
    case SET_VELOCITY:
        sprintf(buff, "%dSV%ld;", axis, ival);
        break;
    case SET_ACCEL:
        sprintf(buff, "%dSA%ld;", axis, ival);
        break;
    case GO:
        /*
         * The PM304 starts moving immediately on move commands, GO command
         * does nothing
         */
        break;
    case GET_INFO:
        /* These commands are not actually done by sending a message, but
           rather they will indirectly cause the driver to read the status
           of all motors */
        break;
    case STOP_AXIS:
        sprintf(buff, "%dST;", axis);
        break;
    case JOG:
        sprintf(buff, "%dSV%ld;", axis, ival);
        strcat(motor_call->message, buff);
        if (ival > 0) {
            /* This is a positive move in PM304 coordinates */
            sprintf(buff, "%dCV1;", axis);
        } else {
            /* This is a negative move in PM304 coordinates */
            sprintf(buff, "%dCV-1;", axis);
        }
        break;
    case SET_PGAIN:
        sprintf(buff, "%dKP%ld;", axis, ival);
        break;

    case SET_IGAIN:
        sprintf(buff, "%dKS%ld;", axis, ival);
        break;

    case SET_DGAIN:
        sprintf(buff, "%dKV%ld;", axis, ival);
        break;

    case ENABLE_TORQUE:
        sprintf(buff, "%dRS;", axis);
        break;

    case DISABL_TORQUE:
        sprintf(buff, "%dAB;", axis);
        break;

    case SET_HIGH_LIMIT:
    case SET_LOW_LIMIT:
        trans->state = IDLE_STATE;  /* No command sent to the controller. */
        /* The PM304 internal soft limits are very difficult to retrieve, not
         * implemented yet */
        break;

    default:
        rtnval = ERROR;
    }
    strcat(motor_call->message, buff);
    Debug(3, "PM304_build_trans: buff=%s, motor_call->message=%s\n", 
            buff, motor_call->message);

    return (rtnval);
}
