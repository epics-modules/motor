#include    <string.h>
#include    "motorRecord.h"
#include    "motor.h"
#include    "motordevCom.h"
#include    "drvLinMot.h"
#include    "epicsExport.h"

#define STATIC static
#define ASCII_0_TO_A 65     /* ASCII offset between 0 and A */
#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

extern struct driver_table LinMot_access;

/*----------------debugging-----------------*/
volatile int devLinMotDebug = 0;
extern "C" {epicsExportAddress(int, devLinMotDebug);}

static inline void Debug(int level, const char *format, ...) {
    if (level < devLinMotDebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
	}
}

/* ----------------Create the dsets for devLinMot----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long LinMot_init(void *);
STATIC long LinMot_init_record(void *);
STATIC long LinMot_start_trans(struct motorRecord *);
STATIC RTN_STATUS LinMot_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS LinMot_end_trans(struct motorRecord *);

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
    int card;
	char axis;	
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
    axis = motor_call->signal + ASCII_0_TO_A;
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
        sprintf(buff, "!SP%ld%c;", ival, axis);
        break;
    case SET_VELOCITY:
        sprintf(buff, "!SV%ld%c;", ival, axis);
        break;
    case SET_ACCEL:
        sprintf(buff, "!SA%ld%c;", ival, axis);
        break;
    case SET_PGAIN:
        sprintf(buff, "!DP%ld%c;", ival, axis);
        break;
    case SET_IGAIN:
        sprintf(buff, "!DI%ld%c;", ival, axis);
        break;
    case SET_DGAIN:
        sprintf(buff, "!DD%ld%c;", ival, axis);
        break;
    case HOME_REV:
    case HOME_FOR:
        sprintf(buff, "!SR-1;");
        strcat(motor_call->message, buff);
		break;
    case LOAD_POS:
        sprintf(buff, "!RP%ld%c;", ival, axis);
        break;
	/* These commands are unimplemented because there are no suitable motor commands
	   that fit them */
    case STOP_AXIS:
    case GO:
    case SET_ENC_RATIO:
    case GET_INFO:
    case JOG:
    case MOVE_REL:
    case SET_VEL_BASE:
    case ENABLE_TORQUE:
    case DISABL_TORQUE:
    case SET_HIGH_LIMIT:
    case SET_LOW_LIMIT:
        break;
    default:
        rtnval = ERROR;
    }
    strcat(motor_call->message, buff);
    Debug(3, "LinMot_build_trans: buff=%s, motor_call->message=%s\n", buff, motor_call->message);
    return (rtnval);
}