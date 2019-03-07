/*
FILENAME...	devPIC862.cc
USAGE...	Motor record device level support for Physik Instrumente (PI)
		GmbH & Co. C-862 motor controller.
*/

/*
 *      Original Author: Ron Sluiter
 *      Current Author: Mohan Ramanathan
 *      Date: 09/04/2006
 *
 * Modification Log:
 * -----------------
 * .00  09/05/2006  mr  copied from devPIC848.cc
 * .01  09/25/2006 rls  - strip trailing cmnd separator (",") from message.
 *                      - simulate jogging with absolute moves to soft limit.
 */


#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <errlog.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPIC862.h"
#include "epicsExport.h"

extern struct driver_table PIC862_access;

/* ----------------Create the dsets for devPIC862----------------- */
static struct driver_table *drvtabptr;
static long PIC862_init(int);
static long PIC862_init_record(void *);
static long PIC862_start_trans(struct motorRecord *);
static RTN_STATUS PIC862_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS PIC862_end_trans(struct motorRecord *);

struct motor_dset devPIC862 =
{
    {8, NULL, (DEVSUPFUN) PIC862_init, (DEVSUPFUN) PIC862_init_record, NULL},
    motor_update_values,
    PIC862_start_trans,
    PIC862_build_trans,
    PIC862_end_trans
};

extern "C" {epicsExportAddress(dset,devPIC862);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types PIC862_table[] = {
    MOTION, 	/* MOVE_ABS */
    MOTION, 	/* MOVE_REL */
    MOTION, 	/* HOME_FOR */
    MOTION, 	/* HOME_REV */
    IMMEDIATE,	/* LOAD_POS */
    IMMEDIATE,	/* SET_VEL_BASE */
    IMMEDIATE,	/* SET_VELOCITY */
    IMMEDIATE,	/* SET_ACCEL */
    IMMEDIATE,	/* GO */
    IMMEDIATE,	/* SET_ENC_RATIO */
    INFO,	/* GET_INFO */
    MOVE_TERM,	/* STOP_AXIS */
    VELOCITY,	/* JOG */
    IMMEDIATE,	/* SET_PGAIN */
    IMMEDIATE,	/* SET_IGAIN */
    IMMEDIATE,	/* SET_DGAIN */
    IMMEDIATE,	/* ENABLE_TORQUE */
    IMMEDIATE,	/* DISABL_TORQUE */
    IMMEDIATE,	/* PRIMITIVE */
    IMMEDIATE,	/* SET_HIGH_LIMIT */
    IMMEDIATE,	/* SET_LOW_LIMIT */
    VELOCITY	/* JOG_VELOCITY */
};


static struct board_stat **PIC862_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PIC862 servo motor */
static long PIC862_init(int after)
{
    long rtnval;

    if (!after)
    {
	drvtabptr = &PIC862_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PIC862_cards);
    return(rtnval);
}


/* initialize a record instance */
static long PIC862_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PIC862_cards));
}


/* start building a transaction */
static long PIC862_start_trans(struct motorRecord *mr)
{
    motor_start_trans_com(mr, PIC862_cards);
    return(OK);
}


/* end building a transaction */
static RTN_STATUS PIC862_end_trans(struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    char *msgptr;
    int last;

    /* Remove trailing cmnd separator (",") from message. */
    motor_call = &(trans->motor_call);
    msgptr = motor_call->message;
    last = strlen(msgptr) - 1;
    if (msgptr[last] == ',')
	msgptr[last] = 0;
    
    motor_end_trans_com(mr, drvtabptr);
    return(OK);
}


/* add a part to the transaction */
static RTN_STATUS PIC862_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    char buff[110];
    int card;
    unsigned int size;
    double dval;
    int cntrl_units;
    RTN_STATUS rtnval;
    bool send;

    send = true;		/* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';
    
    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;
    

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);


    cntrl_units = (int) dval;
    
    if (PIC862_table[command] > motor_call->type)
	motor_call->type = PIC862_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
	strcat(motor_call->message, mr->init);

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
		strcat(motor_call->message, ",");
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
	    sprintf(buff, "MA%d,", cntrl_units);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "MR%d,", cntrl_units);
	    break;
	
	case HOME_FOR:
	    sprintf(buff, "FE0");
	    break;
	case HOME_REV:
	    sprintf(buff, "FE1");
	    break;
	
	case LOAD_POS:
	    if (cntrl_units == 0.0)
		sprintf(buff, "DH");
	    else
		rtnval = ERROR;
	    break;
	
	case SET_VEL_BASE:
	    send = false;	/* DC motor; not base velocity. */
	    break;
	
        case JOG_VELOCITY:
	case SET_VELOCITY:
	    sprintf(buff, "SV%d,", cntrl_units);
	    break;
	
	case SET_ACCEL:
	    sprintf(buff, "SA%d,", cntrl_units);
	    break;

	case ENABLE_TORQUE:
	    sprintf(buff, "MN");
	    break;

	case DISABL_TORQUE:
	    sprintf(buff, "MF");
	    break;

	case GO:
	    /* The PIC862 starts moving immediately on move commands, GO command
	     * does nothing. */
	    send = false;
	    break;
	
	case PRIMITIVE:
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	    sprintf(buff, "ST");
	    break;
	
	case JOG:
	    /* 
	    * C-862 does not have a jog command.  Simulate with move absolute
	    * to the appropriate software limit.
	    */
	    sprintf(buff, "SV%d,", abs(cntrl_units));
	    strcat(motor_call->message, buff);
	    if (dval > 0.)
		sprintf(buff, "MA%d,", (int) (mr->dhlm / mr->mres));
	    else
		sprintf(buff, "MA%d,", (int) (mr->dllm / mr->mres));
	    break;
	
	case SET_PGAIN:
	    cntrl_units = (int) (32767 * dval);
	    sprintf(buff, "DP%d", cntrl_units);
	    break;
	case SET_IGAIN:
	    cntrl_units = (int) (32767 * dval);
	    sprintf(buff, "DI%d", cntrl_units);
	    break;
	case SET_DGAIN:
	    cntrl_units = (int) (32767 * dval);
	    sprintf(buff, "DD%d", cntrl_units);
	    break;
	
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	case SET_ENC_RATIO:
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */
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
	errlogMessage("PIC862_build_trans(): buffer overflow.\n");
    else
    {
	strcat(motor_call->message, buff);
    }
    return(rtnval);
}
