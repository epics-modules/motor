/*
FILENAME...	devPIC662.cc
USAGE...	Motor record device level support for Physik Instrumente (PI)
		GmbH & Co. C-844 motor controller.

Version:	$Revision: 1.2 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2008-03-14 20:21:36 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 12/17/03
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
 * .01 03/08/06 jps - copied from devPI.cc
 */							      


#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "epicsExport.h"
#include "drvPIC662.h"

extern struct driver_table PIC662_access;

/* ----------------Create the dsets for devPIC662----------------- */
static struct driver_table *drvtabptr;
static long PIC662_init(void *);
static long PIC662_init_record(void *);
static long PIC662_start_trans(struct motorRecord *);
static RTN_STATUS PIC662_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS PIC662_end_trans(struct motorRecord *);

struct motor_dset devPIC662 =
{
    {8, NULL, (DEVSUPFUN) PIC662_init, (DEVSUPFUN) PIC662_init_record, NULL},
    motor_update_values,
    PIC662_start_trans,
    PIC662_build_trans,
    PIC662_end_trans
};

extern "C" {epicsExportAddress(dset,devPIC662);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types PIC662_table[] = {
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


static struct board_stat **PIC662_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PIC662 stepper motor */
static long PIC662_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
	drvtabptr = &PIC662_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PIC662_cards);
    return(rtnval);
}


/* initialize a record instance */
static long PIC662_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PIC662_cards));
}


/* start building a transaction */
static long PIC662_start_trans(struct motorRecord *mr)
{
    motor_start_trans_com(mr, PIC662_cards);
    return(OK);
}


/* end building a transaction */
static RTN_STATUS PIC662_end_trans(struct motorRecord *mr)
{
    motor_end_trans_com(mr, drvtabptr);
    return(OK);
}


/* add a part to the transaction */
static RTN_STATUS PIC662_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct PIC662controller *cntrl;
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

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct PIC662controller *) brdptr->DevicePrivate;

    maxdigits = cntrl->res_decpts;
    cntrl_units = dval * cntrl->drive_resolution;
    
    if (PIC662_table[command] > motor_call->type)
	motor_call->type = PIC662_table[command];

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
	    sprintf(buff, "DEV:CONT REM\nPOS %.*f", maxdigits, cntrl_units);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "DEV:CONT REM\nPOS:REL %.*f", maxdigits, cntrl_units);
	    break;
	
	case HOME_FOR:
	case HOME_REV:
	    send = false;
	    break;
	
	case LOAD_POS:
	    send = false;	/* Can't Load a Position */
	    break;
	
	case SET_VEL_BASE:

	    break;
	
	case SET_VELOCITY:
	    send = false;	/* DC motor; not base velocity. */
	    break;
	
	case SET_ACCEL:
	    send = false;	/* DC motor; not base velocity. */
	    break;
	
	case GO:
	    /* The PIC662 starts moving immediately on move commands, GO command
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
	  // Force device polling to stop.
	  // There is no HALT command to the motor controller
	    cntrl->stop_status = true;
	    send = false;
	    break;
	
	case JOG_VELOCITY:
	case JOG:
	    send = false;
	    break;
	
	case SET_PGAIN:
	    send = false;
	    break;
	case SET_IGAIN:
	    send = false;
	    break;
	case SET_DGAIN:
	    send = false;
	    break;
	
	case ENABLE_TORQUE:
	    send = false;
	    break;
	
	case DISABL_TORQUE:
	    send = false;
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
	errlogMessage("PIC662_build_trans(): buffer overflow.\n");
    else
	strcat(motor_call->message, buff);
    return(rtnval);
}
