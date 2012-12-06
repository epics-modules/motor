/*
FILENAME...	devMM3000.cc
USAGE...	Motor record device level support for Newport MM3000.

Version:	$Revision: 1.4 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2008-03-14 20:17:14 $
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/16/97
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
 * .00  10-16-97 mlr initialized from devOms58
 * .01  07-19-99 rls initialized from drvMM4000
 * .02  04-21-01 rls Added jog velocity motor command.
 * .03  05-22-03 rls Converted to R3.14.x.
 */


#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvMMCom.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table MM3000_access;

/* ----------------Create the dsets for devMM3000----------------- */
/* static long report(); */
STATIC struct driver_table *drvtabptr;
STATIC long MM3000_init(void *);
STATIC long MM3000_init_record(void *);
STATIC long MM3000_start_trans(struct motorRecord *);
STATIC RTN_STATUS MM3000_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS MM3000_end_trans(struct motorRecord *);

struct motor_dset devMM3000 =
{
    {8, NULL, (DEVSUPFUN) MM3000_init, (DEVSUPFUN) MM3000_init_record, NULL},
    motor_update_values,
    MM3000_start_trans,
    MM3000_build_trans,
    MM3000_end_trans
};

extern "C" {epicsExportAddress(dset,devMM3000);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types MM3000_table[] = {
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


static struct board_stat **MM3000_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for MM3000 stepper motor */
STATIC long MM3000_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
	drvtabptr = &MM3000_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MM3000_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long MM3000_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, MM3000_cards));
}


/* start building a transaction */
STATIC long MM3000_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, MM3000_cards));
}


/* end building a transaction */
STATIC RTN_STATUS MM3000_end_trans(struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    char *msgptr;
    size_t last;

    /* Remove trailing ';'s from message. */
    motor_call = &(trans->motor_call);
    msgptr = motor_call->message;
    last = strlen(msgptr) - 1;
    if (msgptr[last] == ';')
        msgptr[last] = (char) NULL;

    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC RTN_STATUS MM3000_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct MMcontroller *cntrl;
    char buff[30];
    int axis, card;
    size_t size;
    int intval;
    RTN_STATUS rtnval;

    rtnval = OK;
    buff[0] = '\0';

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);
    cntrl = (struct MMcontroller *) brdptr->DevicePrivate;

    if (MM3000_table[command] > motor_call->type)
	motor_call->type = MM3000_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
	strcat(motor_call->message, mr->init);
	strcat(motor_call->message, "\r");
    }

    switch (command)
    {
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    {
		double temp_dbl;

		temp_dbl = parms[0] * 32767.0;
		intval = NINT(temp_dbl);
		break;
	    }

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
	    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
	    intval = (parms == NULL) ? 0 : NINT(parms[0]);
	    break;
    }


    switch (command)
    {
	case MOVE_ABS:
	    sprintf(buff, "%dPA%d;", axis, intval);
	    break;
	case MOVE_REL:
	    sprintf(buff, "%dPR%d;", axis, intval);
	    break;
	case HOME_FOR:
	case HOME_REV:
	    sprintf(buff, "%dOR1;", axis);
	    break;
	case LOAD_POS:
	    if (*parms == 0.0)
		sprintf(buff, "%dDH", axis);
	    else
		rtnval = ERROR;
	    break;
	case SET_VEL_BASE:
	    if (cntrl->type[axis - 1] == STEPPER)
	    {
		if (intval < 100)
		    intval = 100;
		sprintf(buff, "%dVB%d;", axis, intval);
	    }
	    break;
	case SET_VELOCITY:
	    if (cntrl->type[axis - 1] == STEPPER && intval < 100)
		    intval = 100;
	    sprintf(buff, "%dVA%d;", axis, intval);
	    break;
	case SET_ACCEL:
	    /* 
	     * The value passed is in steps/sec/sec.  
	     * Convert to user units/sec/sec
	     */
	    if (cntrl->type[axis - 1] == STEPPER && intval < 15000)
		    intval = 15000;
	    if (cntrl->type[axis - 1] == DC && intval < 250)
		    intval = 250;
	    sprintf(buff, "%dAC%d;", axis, intval);
	    break;
	case GO:
	    /* 
	     * The MM3000 starts moving immediately on move commands, GO command
	     * does nothing
	     */
	    break;
	case SET_ENC_RATIO:
	    /* MM3000 valid encoder ratio values < 10,000. */
	    while (parms[0] > 10000.0 || parms[1] > 10000.0)
	    {
		parms[0] /= 10;
		parms[1] /= 10;
	    }
	    if (cntrl->type[axis - 1] == STEPPER)
		sprintf(buff, "%dER%d:%d", axis, (int) parms[0], (int) parms[1]);
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
	    /* 
	    * MM3000 does not have a jog command.  Simulate with move absolute
	    * to the appropriate software limit. We can move to MM3000 soft limits.
	    * If the record soft limits are set tighter than the MM3000 limits
	    * the record will prevent JOG motion beyond its soft limits 
	    */
	    sprintf(buff, "%dVA%d;", axis, abs(intval));
	    strcat(motor_call->message, buff);
	    if (intval > 0)
		sprintf(buff, "%dPA%d;", axis, (int) (mr->dhlm / mr->mres));
	    else
		sprintf(buff, "%dPA%d;", axis, (int) (mr->dllm / mr->mres));
	    break;
	case SET_PGAIN:
	    sprintf(buff, "%dKP%d;%dUF;", axis, intval, axis);
	    break;
	case SET_IGAIN:
	    sprintf(buff, "%dKI%d;%dUF;", axis, intval, axis);
	    break;
	case SET_DGAIN:
	    sprintf(buff, "%dKD%d;%dUF;", axis, intval, axis);
	    break;
	case ENABLE_TORQUE:
	    sprintf(buff, "MO;");
	    break;
	case DISABL_TORQUE:
	    sprintf(buff, "MF;");
	    break;
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	    sprintf(buff, "%dSL%d;", axis, intval);
	    break;
	default:
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	errlogMessage("MM3000_build_trans(): buffer overflow.\n");
    else
	strcat(motor_call->message, buff);

    return(rtnval);
}
