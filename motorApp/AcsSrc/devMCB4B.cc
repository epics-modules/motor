/* File: devMCB4B.cc                    */

/* Device Support Routines for motor */
/*
 *      Original Author: Mark Rivers
 *      Date: 02-24-2002
 *
 * Modification Log:
 * -----------------
 * .00  02-24-2002      mlr     initialized from devPM304.c
 * .01  05-23-2003      rls     Converted to R3.14.x.
 * .02  02-19-2004      mlr     Bug fix when not sending anything
 */


#define VERSION 1.00

#include 	<string.h>
#include 	<math.h>
#include 	"motorRecord.h"
#include 	"motor.h"
#include 	"motordevCom.h"
#include        "drvMCB4B.h"
#include 	"epicsExport.h"

#define STATIC static

extern struct driver_table MCB4B_access;

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

volatile int devMCB4BDebug = 0;
extern "C" {epicsExportAddress(int, devMCB4BDebug);}

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < devMCB4BDebug)
    {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* Debugging levels: 
 *      devMCB4BDebug >= 3  Print new part of command and command string so far
 *                          at the end of MCB4B_build_trans
 */


/* ----------------Create the dsets for devMCB4B----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long MCB4B_init(int);
STATIC long MCB4B_init_record(void *);
STATIC long MCB4B_start_trans(struct motorRecord *);
STATIC RTN_STATUS MCB4B_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS MCB4B_end_trans(struct motorRecord *);

struct motor_dset devMCB4B =
{
    {8, NULL, (DEVSUPFUN) MCB4B_init, (DEVSUPFUN) MCB4B_init_record, NULL},
    motor_update_values,
    MCB4B_start_trans,
    MCB4B_build_trans,
    MCB4B_end_trans
};

extern "C" {epicsExportAddress(dset,devMCB4B);}


/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static msg_types MCB4B_table[] = {
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
    VELOCITY    /* JOB_VELOCITY */
};


static struct board_stat **MCB4B_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for MCB4B stepper motor */
STATIC long MCB4B_init(int after)
{
    long rtnval;

    Debug(5, "MCB4B_init: entry\n");
    if (!after)
    {
	drvtabptr = &MCB4B_access;
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MCB4B_cards);
    Debug(5, "MCB4B_init: exit\n");
    return(rtnval);
}


/* initialize a record instance */
STATIC long MCB4B_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
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
STATIC RTN_STATUS MCB4B_end_trans(struct motorRecord *mr)
{
    return(OK);
}

/* add a part to the transaction */
STATIC RTN_STATUS MCB4B_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct MCB4Bcontroller *cntrl;
    char buff[30];
    int axis, card;
    RTN_STATUS rtnval;
    double dval;
    long ival;
    bool send;

    send = true;
    rtnval = OK;
    buff[0] = '\0';
    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;
    ival = NINT(dval);

    rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCB4B_cards);
    Debug(5, "MCB4B_build_trans: entry, motor_start_trans_com=%d\n", rtnval);

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
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCB4B_cards);
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
                rtnval = motor_end_trans_com(mr, drvtabptr);
                rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCB4B_cards);
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
        send=false;
        trans->state = IDLE_STATE;
        break;          /* MCB4B does not use base velocity */
    case SET_VELOCITY:
        ival = (int) (fabs(115200./dval) + 0.5);
        if (ival < 2) ival=2;
        if (ival > 255) ival = 255;
        sprintf(motor_call->message, "#%02dV=%ld", axis, ival);
        break;
    case SET_ACCEL:
        /* dval is acceleration in steps/sec/sec */
        /* MCB is programmed with Ramp Index (R) where: */
        /* dval (steps/sec/sec) = 720,000/(256-R) */
        /* or R=256-(720,000/dval) */
        ival = (int) (256-(720000./dval)+0.5);
        if (ival < 1) ival=1;
        if (ival > 255) ival=255;
        sprintf(motor_call->message, "#%02dR=%ld", axis, ival);
        break;
    case GO:
        /*
         * The MCB4B starts moving immediately on move commands, GO command
         * does nothing
         */
        send=false;
        trans->state = IDLE_STATE;
        break;
    case SET_ENC_RATIO:
        /*
         * The MCB4B does not have the concept of encoder ratio, ignore this
         * command
         */
        send=false;
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
        ival = (int) (fabs(115200./dval) + 0.5);
        if (ival < 2) ival=2;
        if (ival > 65535) ival = 65535;
        sprintf(motor_call->message, "#%02dC=%ld", axis, ival);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCB4B_cards);
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
    case SET_IGAIN:
    case SET_DGAIN:
        send=false;
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
        send=false;
        trans->state = IDLE_STATE;
        break;

    default:
        rtnval = ERROR;
    }

    if(send == false)
       return(rtnval);
    else { 
        rtnval = motor_end_trans_com(mr, drvtabptr);
        Debug(5, "MCB4B_send_msg: motor_end_trans_com status=%d, exit\n", rtnval); 
        return (rtnval);
    }
}
