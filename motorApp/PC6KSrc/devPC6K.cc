/*
FILENAME...	devPC6K.cc
USAGE...	Motor record device level support for Parker Compumotor drivers

Version:	$Revision: 1.1 $
Modified By:	$Author: rivers $
Last Modified:	$Date: 2005-12-21 22:58:24 $
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/97
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
 * .01  04-07-05    jps     initialized from devMM4000.cc
 */


#include <string.h>
#include <math.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPC6K.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table PC6K_access;

/* ----------------Create the dsets for devPC6K----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long PC6K_init(void *);
STATIC long PC6K_init_record(void *);
STATIC long PC6K_start_trans(struct motorRecord *);
STATIC RTN_STATUS PC6K_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS PC6K_end_trans(struct motorRecord *);

struct motor_dset devPC6K =
{
    {8, NULL, (DEVSUPFUN) PC6K_init, (DEVSUPFUN) PC6K_init_record, NULL},
    motor_update_values,
    PC6K_start_trans,
    PC6K_build_trans,
    PC6K_end_trans
};

extern "C" {epicsExportAddress(dset,devPC6K);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types PC6K_table[] = {
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


static struct board_stat **PC6K_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PC6K stepper motor */
STATIC long PC6K_init(void *arg)
{
    long rtnval;
    int after = (int) arg;

    if (after == 0)
    {
	drvtabptr = &PC6K_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PC6K_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long PC6K_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PC6K_cards));
}


/* start building a transaction */
STATIC long PC6K_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, PC6K_cards));
}


/* end building a transaction */
STATIC RTN_STATUS PC6K_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC RTN_STATUS PC6K_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct mess_info *motor_info;
    struct PC6KController *cntrl;
    char buff[110];
    int signal, axis, card, intval;
    int maxdigits;
    double dval, cntrl_units;
    unsigned int size;
    RTN_STATUS rtnval;

    rtnval = OK;
    buff[0] = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    intval = (parms == NULL) ? 0 : NINT(parms[0]);
    dval = (parms == NULL) ? 0 : *parms;

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    signal = motor_call->signal;
    axis = signal+1; /* Motors start at 1 */
    brdptr = (*trans->tabptr->card_array)[card];
      if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct PC6KController *) brdptr->DevicePrivate;
    cntrl_units = dval * cntrl->drive_resolution[signal];
    maxdigits = cntrl->res_decpts[signal];

    
    if (PC6K_table[command] > motor_call->type)
	motor_call->type = PC6K_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
	strcat(motor_call->message, mr->init);
	strcat(motor_call->message, PC6K_OUT_EOS);
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
		strcat(motor_call->message, ":");
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
	  sprintf(buff, "%dMA1:%dMC0:%dD%d:", axis, axis, axis, intval);
	    break;
	
        case MOVE_REL:
	  sprintf(buff, "%dMA0:%dMC0:%dD%d:", axis, axis, axis, intval);
	    break;
	
	case HOME_FOR:
	    sprintf(buff, "%dHOM0:", axis);
	    break;
	case HOME_REV:
	    sprintf(buff, "%dHOM1:", axis);
	    break;
	
	case LOAD_POS:
	  /* The Feedback position follows the Reference set position */
	    sprintf(buff, "%dPSET%d:", axis, intval);
	    break;
	
	case SET_VEL_BASE:
	    break;	    /* PC6K does not use base velocity */
	
	case SET_VELOCITY:
	    sprintf(buff, "%dV%.*f:", axis, maxdigits, cntrl_units);
	    break;
	
	case SET_ACCEL:
	  /* Set Accel, Avg Accel, Decel, Avg Decel - assume avg to be 1/2 accel */
	    sprintf(buff, "%dA%.*f:%dAA%.*f:%dAD%.*f:%dADA%.*f:", 
		    axis, maxdigits, cntrl_units,
		    axis, maxdigits, cntrl_units/2,
		    axis, maxdigits, cntrl_units,
		    axis, maxdigits, cntrl_units/2);

	    break;
	
	case GO:
	    sprintf(buff, "%dGO:", axis); 
	    break;
	
	case SET_ENC_RATIO:
	  // sprintf(buff, "%dERES%d:%dDRESET:", axis, intval, axis);
	    break;
	
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
   	    sprintf(buff, "!%dK:", axis);
	    break;
	
	case JOG:
   	    if (intval > 0)
	      sprintf(buff, "%dMC1:%dV%.*f:%dD+:%dGO:", axis, axis, maxdigits, cntrl_units, axis, axis);
            else
	      sprintf(buff, "%dMC1:%dV%.*f:%dD-:%dGO:", axis, axis, maxdigits, cntrl_units, axis, axis);
	    break;
	
	case SET_PGAIN:
	    sprintf(buff, "%dSGP%d:", axis, intval);
	    break;
	
	case SET_IGAIN:
	    sprintf(buff, "%dSGI%d:", axis, intval);
	    break;
	
	case SET_DGAIN:
	    sprintf(buff, "%dSGV%d:", axis, intval); /* Velocity Gain */
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "%dDRIVE1:", axis);
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "%dDRIVE0:", axis);
	    break;
	
	case SET_HIGH_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[signal];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    if (motor_info->high_limit > motor_info->low_limit && 
		intval > motor_info->high_limit)
	    {
		mr->dhlm = motor_info->high_limit * mr->mres;
		rtnval = ERROR;
	    }
	    break;
	
	case SET_LOW_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[signal];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    if (motor_info->low_limit < motor_info->high_limit && 
		intval < motor_info->low_limit)
	    {
		mr->dllm = motor_info->low_limit * mr->mres;
		rtnval = ERROR;
	    }
	    break;
	
	default:
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	errlogMessage("PC6K_build_trans(): buffer overflow.\n");
    else
	strcat(motor_call->message, buff);

    return(rtnval);
}
