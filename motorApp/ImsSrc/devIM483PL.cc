/*
FILENAME...	devIM483PL.cc
USAGE...	Motor record device level support for Intelligent Motion
		Systems, Inc. IM483(I/IE).

*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 07/10/2000
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
 *	      The Controls and Automation Group (AT-8)
 *	      Ground Test Accelerator
 *	      Accelerator Technology Division
 *	      Los Alamos National Laboratory
 *
 *      Co-developed with
 *	      The Controls and Computing Group
 *	      Accelerator Systems Division
 *	      Advanced Photon Source
 *	      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01	07/10/00 rls copied from devIM483SM.c
 * .02  05/16/01 rls Added support for changing jog velocity while jogging.
 * .03  03/01/02 rls eliminated "ASCII record separator (IS2) = /x1E".
 * .04  04/15/02 rls Must support PRIMITIVE in build_trans() for INIT field to
 *			work, and add axis name place holder (?) to message.
 * .05  03/07/03 rls R3.14 conversion.
 */

#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvIM483.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table IM483PL_access;

/* ----------------Create the dsets for devIM483PL----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long IM483PL_init(void *);
STATIC long IM483PL_init_record(void *);
STATIC long IM483PL_start_trans(struct motorRecord *);
STATIC RTN_STATUS IM483PL_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS IM483PL_end_trans(struct motorRecord *);

struct motor_dset devIM483PL =
{
    {8, NULL, (DEVSUPFUN) IM483PL_init, (DEVSUPFUN) IM483PL_init_record, NULL},
    motor_update_values,
    IM483PL_start_trans,
    IM483PL_build_trans,
    IM483PL_end_trans
};

extern "C" {epicsExportAddress(dset,devIM483PL);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types IM483PL_table[] = {
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


static struct board_stat **IM483PL_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for IM483PL stepper motor */
STATIC long IM483PL_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
	drvtabptr = &IM483PL_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &IM483PL_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long IM483PL_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, IM483PL_cards));
}


/* start building a transaction */
STATIC long IM483PL_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
STATIC RTN_STATUS IM483PL_end_trans(struct motorRecord *mr)
{
    return(OK);
}


/* add a part to the transaction */
STATIC RTN_STATUS IM483PL_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct IM483controller *cntrl;
    char buff[110];
    int axis, card, maxdigits;
    unsigned int size;
    double dval, cntrl_units;
    RTN_STATUS rtnval;
    bool send;

    send = true;		/* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;

    motor_start_trans_com(mr, IM483PL_cards);
    
    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct IM483controller *) brdptr->DevicePrivate;
    cntrl_units = dval;
    maxdigits = 2;
    
    if (IM483PL_table[command] > motor_call->type)
	motor_call->type = IM483PL_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
	strcat(motor_call->message, "? ");
	strcat(motor_call->message, mr->init);
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
	    sprintf(buff, " R%.*f", maxdigits, cntrl_units);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, " %+.*f", maxdigits, cntrl_units);
	    break;
	
	case HOME_FOR:
	    sprintf(buff, " F1000 0");
	    break;

	case HOME_REV:
	    sprintf(buff, " F1000 1");
	    break;
	
	case LOAD_POS:
	    if (cntrl_units == 0.0)
		sprintf(buff, " O");
	    else
	    {
		send = false;
		rtnval = ERROR;
	    }
	    break;
	
	case SET_VEL_BASE:
	    sprintf(buff, " I%.*f;", maxdigits, cntrl_units);
	    break;
	
	case SET_VELOCITY:
	    sprintf(buff, " V%.*f;", maxdigits, cntrl_units);
	    break;
	
	case SET_ACCEL:
	    /* ????? MAKE SENSE OF ACCELERATION PARAMETER ??????*/
	    send = false;
	    break;
	
	case GO:
	    /* The IM483PL starts moving immediately on move commands, GO command
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
	    sprintf(buff, " @ 0");
	    break;
	
	case JOG_VELOCITY:
	case JOG:
	    sprintf(buff, " M%.*f;", maxdigits, cntrl_units);
	    break;
	
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    send = false;
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, " MO;");
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, " MF;");
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
	errlogMessage("IM483PL_build_trans(): buffer overflow.\n");
    else
    {
	strcat(motor_call->message, buff);
	motor_end_trans_com(mr, drvtabptr);
    }
    return(rtnval);
}
