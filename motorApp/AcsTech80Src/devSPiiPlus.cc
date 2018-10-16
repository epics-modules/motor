/*
FILENAME...	devSPiiPlus.cc
USAGE...	Motor record device level support for ACS Tech80 SPiiPlus

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
#include <errlog.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvSPiiPlus.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table SPiiPlus_access;

/* ----------------Create the dsets for devSPiiPlus----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long SPiiPlus_init(int);
STATIC long SPiiPlus_init_record(void *);
STATIC long SPiiPlus_start_trans(struct motorRecord *);
STATIC RTN_STATUS SPiiPlus_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS SPiiPlus_end_trans(struct motorRecord *);

struct motor_dset devSPiiPlus =
{
    {8, NULL, (DEVSUPFUN) SPiiPlus_init, (DEVSUPFUN) SPiiPlus_init_record, NULL},
    motor_update_values,
    SPiiPlus_start_trans,
    SPiiPlus_build_trans,
    SPiiPlus_end_trans
};

extern "C" {epicsExportAddress(dset,devSPiiPlus);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types SPiiPlus_table[] = {
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


static struct board_stat **SPiiPlus_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for SPiiPlus stepper motor */
STATIC long SPiiPlus_init(int after)
{
    long rtnval;

    if (!after)
    {
	drvtabptr = &SPiiPlus_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &SPiiPlus_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long SPiiPlus_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;

    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, SPiiPlus_cards));
}


/* start building a transaction */
STATIC long SPiiPlus_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, SPiiPlus_cards));
}


/* end building a transaction */
STATIC RTN_STATUS SPiiPlus_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC RTN_STATUS SPiiPlus_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct mess_info *motor_info;
    struct SPiiPlusController *cntrl;
    char buff[110];
    int axis, card;
    int intval;
    double dval;
    unsigned int size;
    RTN_STATUS rtnval;

    rtnval = OK;
    buff[0] = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    intval = (parms == NULL) ? 0 : NINT(parms[0]);
    dval = (parms == NULL) ? 0 : *parms;

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal;
    brdptr = (*trans->tabptr->card_array)[card];
      if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct SPiiPlusController *) brdptr->DevicePrivate;
    
    if (SPiiPlus_table[command] > motor_call->type)
	motor_call->type = SPiiPlus_table[command];

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
	  if (cntrl->cmndMode == BUFFER)
	    sprintf(buff, "Done(%d)=0;target_pos(%d)=%d;opReq(%d)=%d;", axis, axis, intval, axis, OP_ABS_MOVE);
	  else
	    sprintf(buff, "ptp (%d), %d;", axis, intval);

	  break;
	
       case MOVE_REL:
	  if (cntrl->cmndMode == BUFFER)
	    sprintf(buff, "Done(%d)=0;target_pos(%d)=%d;opReq(%d)=%d;", axis, axis, intval, axis, OP_REL_MOVE);
	  else
	    sprintf(buff, "ptp/r (%d), %d;", axis, intval);
	 break;
	
	case HOME_FOR:
	  if (cntrl->cmndMode == BUFFER)
	    sprintf(buff, "Done(%d)=0;opReq(%d)=%d;", axis, axis, OP_HOME_F);
	  break;

	case HOME_REV:
	  if (cntrl->cmndMode == BUFFER)
	    sprintf(buff, "Done(%d)=0;opReq(%d)=%d;", axis, axis, OP_HOME_R);
	  break;
	
	case LOAD_POS:
	  /* The Feedback position follows the Reference set position */
	  sprintf(buff, "set APOS(%d)=%d;", axis, intval);
	  break;
	
	case SET_VEL_BASE:
	  break;	    /* SPiiPlus does not use base velocity */
	
	case SET_VELOCITY:
	  sprintf(buff, "VEL%d=%d;", axis, intval);
	  break;
	
	case SET_ACCEL:
	  if (cntrl->cmndMode == BUFFER)
	    sprintf(buff, "ACC%d=%d;", axis, intval);
     //  else (DIRECT)
	  //    Leave setting to Controller MMI - too many other dependant settings 

	  break;
	
	case GO:
	  if (cntrl->cmndMode == BUFFER)
	    {
	      // Execute the ACS program buffer that corresponds to the axis
	      // ACS Buffer Execution command must be alone
	      rtnval = motor_end_trans_com(mr, drvtabptr);
	      rtnval = (RTN_STATUS) motor_start_trans_com(mr, SPiiPlus_cards);
	      motor_call->type = SPiiPlus_table[command];

	      sprintf(buff, "start %d, 1", axis);
	    }
	  // else (DIRECT)
	  //    Motion command initiates move 
	  break;
	
	case SET_ENC_RATIO:
	  // Leave this setting to the controller MMI - too much interdependence 
	  //    sprintf(buff, "EFAC%d=%f;", axis, dval);
	    break;
	
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	  if (cntrl->cmndMode == BUFFER)
	    sprintf(buff, "Done(%d)=0;stop_all(%d)=1;", axis, axis);
	  else
	    sprintf(buff, "halt (%d);", axis);

	    break;
	
	case JOG:
	  if (cntrl->cmndMode == BUFFER)
	    {
	      sprintf(buff, "Done(%d)=0;jog_vel(%d)=%d; opReq(%d)=%d;", axis, axis, intval, axis, OP_JOG_MOVE);

	      strcat(motor_call->message, buff);

	      // ACS Buffer Execution command must be alone
	      rtnval = motor_end_trans_com(mr, drvtabptr);
	      rtnval = (RTN_STATUS) motor_start_trans_com(mr, SPiiPlus_cards);
	      motor_call->type = SPiiPlus_table[command];

	      sprintf(buff, "start %d,1", axis);
	    }
	  else
	    sprintf(buff, "jog/v (%d), %d;", axis, intval);

	  break;
	
	case SET_PGAIN:
	    break;
	
	case SET_IGAIN:
	    break;
	
	case SET_DGAIN:
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "enable(%d);", axis);
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "disable(%d);", axis);
	    break;
	
	case SET_HIGH_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    // if (intval > motor_info->high_limit)
	    // {
	    // mr->dhlm = motor_info->high_limit * mr->mres;
	    //		rtnval = ERROR;
	    // }
	    break;
	
	case SET_LOW_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    // if (intval < motor_info->low_limit)
	    // {
	    //	mr->dllm = motor_info->low_limit * mr->mres;
	    // 	rtnval = ERROR;
	    // }
	    break;
	
	default:
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	errlogMessage("SPiiPlus_build_trans(): buffer overflow.\n");
    else
	strcat(motor_call->message, buff);

    return(rtnval);
}
