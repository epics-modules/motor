/*
FILENAME...	devESP300.cc
USAGE...	Motor record device level support for Newport ESP300.

Version:	$Revision: 1.4 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2008-03-14 20:17:14 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 02/19/03
 *	Current Author: Ron Sluiter
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
 * .01  05-23-03 rls Converted to R3.14.x.
 * .02  10-28-03 rls User must set MRES to drive resolution.
 */

#include <string.h>
#include <math.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvMMCom.h"
#include "epicsExport.h"

extern struct driver_table ESP300_access;

/* ----------------Create the dsets for devESP300----------------- */
/* static long report(); */
static struct driver_table *drvtabptr;
static long ESP300_init(void *);
static long ESP300_init_record(void *);
static long ESP300_start_trans(struct motorRecord *);
static RTN_STATUS ESP300_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS ESP300_end_trans(struct motorRecord *);

struct motor_dset devESP300 =
{
    {8, NULL, (DEVSUPFUN) ESP300_init, (DEVSUPFUN) ESP300_init_record, NULL},
    motor_update_values,
    ESP300_start_trans,
    ESP300_build_trans,
    ESP300_end_trans
};

extern "C" {epicsExportAddress(dset,devESP300);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types ESP300_table[] = {
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


static struct board_stat **ESP300_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for ESP300 stepper motor */
static long ESP300_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
	drvtabptr = &ESP300_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &ESP300_cards);
    return(rtnval);
}


/* initialize a record instance */
static long ESP300_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    long rtnval;

    rtnval = motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, ESP300_cards);
    
    return(rtnval);
}


/* start building a transaction */
static long ESP300_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, ESP300_cards));
}


/* end building a transaction */
static RTN_STATUS ESP300_end_trans(struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    char *msgptr;
    int last;

    /* Remove trailing ';'s from message. */
    motor_call = &(trans->motor_call);
    msgptr = motor_call->message;
    last = strlen(msgptr) - 1;
    if (msgptr[last] == ';')
	msgptr[last] = (char) NULL;

    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
static RTN_STATUS ESP300_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct MMcontroller *cntrl;
    char buff[80];
    int axis, card;
    unsigned int size;
    double dval, cntrl_units;
    RTN_STATUS rtnval;

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

    cntrl = (struct MMcontroller *) brdptr->DevicePrivate;
    cntrl_units = dval * cntrl->drive_resolution[axis - 1];

    if (ESP300_table[command] > motor_call->type)
	motor_call->type = ESP300_table[command];

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
        default:
	    break;
    }


    switch (command)
    {
	case MOVE_ABS:
	    sprintf(buff, "%.2dPA%f;", axis, cntrl_units);
	    break;
	case MOVE_REL:
	    sprintf(buff, "%.2dPR%f;", axis, cntrl_units);
	    break;
	case HOME_FOR:
	case HOME_REV:
	    sprintf(buff, "%.2dOR;", axis);
	    break;
	case LOAD_POS:
	    sprintf(buff, "%.2dDH%f", axis, cntrl_units);
	    break;
	case SET_VEL_BASE:
	    sprintf(buff, "%.2dVB%f;", axis, cntrl_units);
	    break;
	case SET_VELOCITY:
	    sprintf(buff, "%.2dVA%f;", axis, cntrl_units);
	    break;
	case SET_ACCEL:
	    sprintf(buff, "%.2dAC%f;", axis, cntrl_units);
	    break;
	case GO:
	    /* 
	     * The ESP300 starts moving immediately on move commands, GO command
	     * does nothing
	     */
	    break;
	case SET_ENC_RATIO:
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	case STOP_AXIS:
	    sprintf(buff, "%.2dST;", axis);
	    break;
	case JOG:
	    sprintf(buff, "%.2dVA%f;", axis, fabs(cntrl_units));
	    strcat(motor_call->message, buff);
	    if (cntrl_units > 0)
		sprintf(buff, "%.2dMV+;", axis);
	    else
		sprintf(buff, "%.2dMV-;", axis);
	    break;
	case SET_PGAIN:
	    sprintf(buff, "%dKP%f;%dUF;", axis, cntrl_units, axis);
	    break;
	case SET_IGAIN:
	    sprintf(buff, "%dKI%f;%dUF;", axis, cntrl_units, axis);
	    break;
	case SET_DGAIN:
	    sprintf(buff, "%dKD%f;%dUF;", axis, cntrl_units, axis);
	    break;
	case ENABLE_TORQUE:
	    sprintf(buff, "%.2dMO;", axis);
	    break;
	case DISABL_TORQUE:
	    sprintf(buff, "%.2dMF;", axis);
	    break;
	case SET_HIGH_LIMIT:
	    sprintf(buff, "%.2dSR%f;", axis, cntrl_units);
	    break;
	case SET_LOW_LIMIT:
	    sprintf(buff, "%.2dSL%f;", axis, cntrl_units);
	    break;
	default:
	    rtnval = ERROR;
    }
    
    size = strlen(buff);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	errlogMessage("ESP300_build_trans(): buffer overflow.\n");
    else
	strcat(motor_call->message, buff);

    return(rtnval);
}
