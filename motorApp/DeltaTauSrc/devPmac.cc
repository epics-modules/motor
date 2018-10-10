/*
FILENAME...	devPmac.cc
USAGE... Device level support for Delta Tau PMAC.

*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 04/16/04
 *      Current Author: Ron Sluiter
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
 * 01 rls 04/16/04 Copied from devOms.cc
 */


#include <string.h>
#include <math.h>
#include <errlog.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPmac.h"

#include "epicsExport.h"

extern int Pmac_num_cards;
extern struct driver_table Pmac_access;

/* ----------------Create the dsets for devOMS----------------- */
static long Pmac_init(int);
static long Pmac_init_record(void *);
static long Pmac_start_trans(struct motorRecord *);
static RTN_STATUS Pmac_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS Pmac_end_trans(struct motorRecord *);

struct motor_dset devPmac =
{
    {8, NULL, (DEVSUPFUN) Pmac_init, Pmac_init_record, NULL},
    motor_update_values,
    Pmac_start_trans,
    Pmac_build_trans,
    Pmac_end_trans
};

extern "C" {epicsExportAddress(dset,devPmac);}

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types Pmac_table[] = {
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

static struct board_stat **Pmac_cards;
static const char errmsg[] = {"\n\n!!!ERROR!!! - Oms driver uninitialized.\n"};

static long Pmac_init(int after)
{
    if (*(Pmac_access.init_indicator) == NO)
    {
	errlogSevPrintf(errlogMinor, "%s", errmsg);
    	return(ERROR);
    }
    else
	return(motor_init_com(after, Pmac_num_cards, &Pmac_access, &Pmac_cards));
}

static long Pmac_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, Pmac_num_cards, &Pmac_access, Pmac_cards));
}

static long Pmac_start_trans(struct motorRecord *mr)
{
    long rtnval;

    rtnval = motor_start_trans_com(mr, Pmac_cards);
    return(rtnval);
}

static RTN_STATUS Pmac_end_trans(struct motorRecord *mr)
{
    if (*(Pmac_access.init_indicator) == NO)
    {
	errlogSevPrintf(errlogMinor, "%s", errmsg);
	return(ERROR);
    }
    else
	return(motor_end_trans_com(mr, &Pmac_access));
}

/* add a part to the transaction */
static RTN_STATUS Pmac_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    char buff[110];
    int axis, card, intval;
    unsigned int size;
    RTN_STATUS rtnval;
    bool send;

    send = true;		/* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    intval = (parms == NULL) ? 0 : NINT(parms[0]);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);
    
    if (Pmac_table[command] > motor_call->type)
	motor_call->type = Pmac_table[command];

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
		strcat(motor_call->message, " ");
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
	    sprintf(buff, " J=%d ", intval);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "?MR %d", intval);
	    break;
	
	case HOME_FOR:
	    sprintf(buff, "? F1000 0");
	    break;

	case HOME_REV:
	    sprintf(buff, "? F1000 1");
	    break;
	
	case LOAD_POS:
    	    /*??? How to do this???? */
	    send = false;
	    break;
	
	case SET_VEL_BASE:
	    send = false;	// No way to set VBAS with PMAC???
	    break;
	
	case SET_VELOCITY:
    	    /* Convert input (steps/s) to steps/ms. */
	    sprintf(buff, " I%.2d22=%f ", axis, (*parms / 1000.0));
	    break;
	
	case SET_ACCEL:
    	    /* Convert input (steps/s^2) to steps/ms^2. */
	    sprintf(buff, " I%.2d19=%f ", axis, (*parms / 1000.0));
	    break;
	
	case GO:
	    /* The PMAC starts moving immediately on J=# command, GO command
	     * does nothing.*/
	    break;
	
	case PRIMITIVE:
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	    strcpy(buff, "J/");
	    break;
	
	case JOG_VELOCITY:
	    sprintf(buff, "I%.2d22=%f",  axis, (fabs(*parms) / 1000.0));
	    break;

	case JOG:
	    sprintf(buff, "I%.2d22=%f ", axis, (fabs(*parms) / 1000.0));
	    if (intval >= 0)
		strcat(buff, "J+");
	    else
		strcat(buff, "J-");
	    break;
	
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    send = false;
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "?DE=1");
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "?DE=0");
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
	errlogMessage("MDrive_build_trans(): buffer overflow.\n");
    else
	strcat(motor_call->message, buff);
    return(rtnval);
}
