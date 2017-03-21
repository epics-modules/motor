#define VERSION 3.00


#include    <string.h>
#include        "motorRecord.h"
#include        "motor.h"
#include        "motordevCom.h"
#include        "drvLinMot.h"
#include    "epicsExport.h"

#define STATIC static

extern struct driver_table LinMot_access;

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

/*----------------debugging-----------------*/
volatile int devLinMotDebug = 0;
extern "C" {epicsExportAddress(int, devLinMotDebug);}

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < devLinMotDebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}


/* Debugging levels: 
 *      devLinMotDebug >= 3  Print new part of command and command string so far
 *                          at the end of LinMot_build_trans
 */

/* ----------------Create the dsets for devLinMot----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long LinMot_init(void *);
STATIC long LinMot_init_record(void *);
STATIC long LinMot_start_trans(struct motorRecord *);
STATIC RTN_STATUS LinMot_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS LinMot_end_trans(struct motorRecord *);
STATIC long VELO = 0;

struct motor_dset devLinMot =
{
    {8, NULL, (DEVSUPFUN) LinMot_init, (DEVSUPFUN) LinMot_init_record, NULL},
    motor_update_values,
    LinMot_start_trans,
    LinMot_build_trans,
    LinMot_end_trans
};

extern "C" {epicsExportAddress(dset,devLinMot);}

/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static msg_types LinMot_table[] = {
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


static struct board_stat **LinMot_cards;

/* --------------------------- program data --------------------- */

/* initialize device support for LinMot stepper motor */
STATIC long LinMot_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
    drvtabptr = &LinMot_access;
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &LinMot_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long LinMot_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    long rtnval;

    rtnval = motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, 
                                   drvtabptr, LinMot_cards);
    return(rtnval);
}


/* start building a transaction */
STATIC long LinMot_start_trans(struct motorRecord *mr)
{
    long rtnval;
    rtnval = motor_start_trans_com(mr, LinMot_cards);
    return(rtnval);
}


/* end building a transaction */
STATIC RTN_STATUS LinMot_end_trans(struct motorRecord *mr)
{
    RTN_STATUS rtnval;
    rtnval = motor_end_trans_com(mr, drvtabptr);
    return(rtnval);
    
}

/* add a part to the transaction */
STATIC RTN_STATUS LinMot_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct LinMotController *cntrl;
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

    cntrl = (struct LinMotController *) brdptr->DevicePrivate;

    if (LinMot_table[command] > motor_call->type)
        motor_call->type = LinMot_table[command];

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
    case HOME_REV:
        break;
    case HOME_FOR:
        break;
    case LOAD_POS:
        break;
    case SET_VEL_BASE:
        break;          /* LinMot does not use base velocity */
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
         * The LinMot starts moving immediately on move commands, GO command
         * does nothing
         */
        break;
    case SET_ENC_RATIO:
        /*
         * The LinMot does not have the concept of encoder ratio, ignore this
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
        sprintf(buff, "%dCV%ld;", axis, ival);
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
        /* The LinMot internal soft limits are very difficult to retrieve, not
         * implemented yet */
        break;

    default:
        rtnval = ERROR;
    }
    strcat(motor_call->message, buff);
    Debug(3, "LinMot_build_trans: buff=%s, motor_call->message=%s\n", buff, motor_call->message);

    return (rtnval);
}