/* File: devPM304.cc                    */
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
 * .03  02-11-03        mlr     Version 3.0, added support for PM600 model.
 *                              Added SD for decceleration.
 * .04  05/27/03    rls     R3.14 conversion.
 *      ...
 */


#define VERSION 3.00


#include    <string.h>
#include        "motorRecord.h"
#include        "motor.h"
#include        "motordevCom.h"
#include        "drvPM304.h"
#include    "epicsExport.h"

#define STATIC static

extern struct driver_table PM304_access;

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

/*----------------debugging-----------------*/
volatile int devPM304Debug = 0;
extern "C" {epicsExportAddress(int, devPM304Debug);}

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < devPM304Debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}


/* Debugging levels: 
 *      devPM304Debug >= 3  Print new part of command and command string so far
 *                          at the end of PM304_build_trans
 */


/* ----------------Create the dsets for devPM304----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long PM304_init(void *);
STATIC long PM304_init_record(void *);
STATIC long PM304_start_trans(struct motorRecord *);
STATIC RTN_STATUS PM304_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS PM304_end_trans(struct motorRecord *);
STATIC long VELO = 0;

struct motor_dset devPM304 =
{
    {8, NULL, (DEVSUPFUN) PM304_init, (DEVSUPFUN) PM304_init_record, NULL},
    motor_update_values,
    PM304_start_trans,
    PM304_build_trans,
    PM304_end_trans
};

extern "C" {epicsExportAddress(dset,devPM304);}


/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static msg_types PM304_table[] = {
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


static struct board_stat **PM304_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PM304 stepper motor */
STATIC long PM304_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
    drvtabptr = &PM304_access;
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PM304_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long PM304_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
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
STATIC RTN_STATUS PM304_end_trans(struct motorRecord *mr)
{
    RTN_STATUS rtnval;
    rtnval = motor_end_trans_com(mr, drvtabptr);
    return(rtnval);
    
}

/* request homing move */
STATIC void request_home(char* buff, int model, int axis, int home_direction, int home_mode) {
    if (model == MODEL_PM304){
        sprintf(buff, "%dIX%d;", axis, home_direction);
    } else {
        int motor_default=0, constant_velocity=1, reverse_and_zero=2;
        if ( home_mode==motor_default || home_mode==reverse_and_zero ) {
            if ( home_mode==reverse_and_zero ) {
                home_direction = -1;
            }
            sprintf(buff, "%dHD%d;", axis, home_direction);
        } else {
            // Let SNL take care of everything. See homing.st
        }
    }
}

/* add a part to the transaction */
STATIC RTN_STATUS PM304_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct PM304controller *cntrl;
    char buff[30];
    int axis, card;
    RTN_STATUS rtnval;
    double dval;
    long ival;

    rtnval = OK;
    buff[0] = '\0';
    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;
    ival = NINT(dval);

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
	        /* Send a reset command before any move */
			if (cntrl->reset_before_move==1) {
				sprintf(buff, "%dRS;", axis);
			}
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
    case HOME_REV:
        request_home(buff, cntrl->model, axis, -1, cntrl->home_mode[axis-1]);
        break;
    case HOME_FOR:
        request_home(buff, cntrl->model, axis, 1, cntrl->home_mode[axis-1]);
        break;
    case LOAD_POS:
        if (cntrl->use_encoder[axis-1]){
           sprintf(buff, "%dAP%ld;", axis, ival);
        }
        break;
    case SET_VEL_BASE:
        break;          /* PM304 does not use base velocity */
    case SET_VELOCITY:
        sprintf(buff, "%dSV%ld;", axis, ival);
        VELO = ival;
        break;
    case SET_ACCEL:
        sprintf(buff, "%dSA%ld;", axis, ival);
        strcat(motor_call->message, buff);
        sprintf(buff, "%dSD%ld;", axis, ival);
        break;
    case GO:
        /*
         * The PM304 starts moving immediately on move commands, GO command
         * does nothing
         */
        break;
    case SET_ENC_RATIO:
        /*
         * The PM304 does not have the concept of encoder ratio, ignore this
         * command
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
        if (cntrl->model == MODEL_PM304) {
            sprintf(buff, "%dSV%ld;", axis, ival);
            strcat(motor_call->message, buff);
            if (ival > 0) {
                /* This is a positive move in PM304 coordinates */
                sprintf(buff, "%dCV1;", axis);
            } else {
                /* This is a negative move in PM304 coordinates */
                sprintf(buff, "%dCV-1;", axis);
            }
        } else {
            sprintf(buff, "%dCV%ld;", axis, ival);
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
    Debug(3, "PM304_build_trans: buff=%s, motor_call->message=%s\n", buff, motor_call->message);

    return (rtnval);
}