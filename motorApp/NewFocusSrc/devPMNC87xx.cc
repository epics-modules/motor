/*
FILENAME...	devPMNC87xx.cc
USAGE...	Motor record device level support for NewFocus 8750 and 8752 Controller .

Version:	1.3
Modified By:	sluiter
Last Modified:	2004/12/20 21:10:53
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/16/97
 *	Current Author: Joe Sullivan
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
 * .04  06-20-06 jps initialized from drvPMNC8750
 */


#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPMNCCom.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table PMNC87xx_access;

/* ----------------Create the dsets for devPMNC87xx----------------- */
/* static long report(); */
STATIC struct driver_table *drvtabptr;
STATIC long PMNC87xx_init(void *);
STATIC long PMNC87xx_init_record(void *);
STATIC long PMNC87xx_start_trans(struct motorRecord *);
STATIC RTN_STATUS PMNC87xx_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS PMNC87xx_end_trans(struct motorRecord *);

struct motor_dset devPMNC87xx =
{
    {8, NULL, (DEVSUPFUN) PMNC87xx_init, (DEVSUPFUN) PMNC87xx_init_record, NULL},
    motor_update_values,
    PMNC87xx_start_trans,
    PMNC87xx_build_trans,
    PMNC87xx_end_trans
};

extern "C" {epicsExportAddress(dset,devPMNC87xx);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types PMNC87xx_table[] = {
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


static struct board_stat **PMNC87xx_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PMNC87xx stepper motor */
STATIC long PMNC87xx_init(void *arg)
{
    long rtnval;
    int after = (int) arg;

    if (after == 0)
    {
	drvtabptr = &PMNC87xx_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PMNC87xx_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long PMNC87xx_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PMNC87xx_cards));
}


/* start building a transaction */
STATIC long PMNC87xx_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, PMNC87xx_cards));
}


/* end building a transaction */
STATIC RTN_STATUS PMNC87xx_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC RTN_STATUS PMNC87xx_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct mess_info *motor_info;
    struct controller *brdptr;
    struct PMNCcontroller *cntrl;
    struct PMD_axis *paxisDef;
    PMD_model dType;

    char buff[110];
    int axis, card;
    int drive, motor;
    unsigned int size;
    int intval, velval;
    RTN_STATUS rtnval;
    bool sendMsg;

    rtnval = OK;
    buff[0] = '\0';
    sendMsg = true;

    motor_start_trans_com(mr, PMNC87xx_cards);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal;

    motor_call->type = PMNC87xx_table[command];
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);
    cntrl = (struct PMNCcontroller *) brdptr->DevicePrivate;


    paxisDef = &cntrl->axisDef[axis];
    drive = paxisDef->driverNum;
    motor = paxisDef->motorNum;
    dType = paxisDef->driverType;

    if (PMNC87xx_table[command] > motor_call->type)
	motor_call->type = PMNC87xx_table[command];

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
                rtnval = motor_end_trans_com(mr, drvtabptr);
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, PMNC87xx_cards);
		motor_call->type = PMNC87xx_table[command];

	    }
	    if (strlen(mr->post) != 0)
		motor_call->postmsgptr = (char *) &mr->post;
        default:
	    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
	    intval = (parms == NULL) ? 0 : NINT(parms[0]);
	    break;
    }

    motor_info = &((*drvtabptr->card_array)[card]->motor_info[axis]);

    switch (command)
    {
	case MOVE_ABS:
	    if (dType == PMD8753)
	      sprintf(buff, "REL A%d=%d", drive, intval-motor_info->position);
	    else
	      sprintf(buff, "ABS A%d=%d", drive, intval);	      
	    break;
	case MOVE_REL:
	    sprintf(buff, "REL A%d=%d", drive, intval);
	    break;
	case HOME_FOR:
	  // if (dType == PMD8751)
	  // sprintf(buff, "FIN A%d", drive);
	  //  else
	  //    trans->state = IDLE_STATE;	/* No command sent to the controller. */
	      sendMsg = false;
	    break;
	case HOME_REV:
	  //  if (dType == PMD8751)
	  //  sprintf(buff, "RIN A%d", drive);
	  //  else
	  //  trans->state = IDLE_STATE;	/* No command sent to the controller. */
	      sendMsg = false;
	    break;
	case LOAD_POS:

	    if (dType == PMD8753)
	      {
		/* Setting local driver position because the controller cannot be set */
		sprintf(buff, "CHL A%d=%d", drive, motor);
		motor_info->position = intval;
		motor_info->encoder_position = intval;
		cntrl->last_position[axis] = 0;  // Used to accumulate position history 
	      }
	    else if (dType == PMD8751)
	      {
		/* Closed loop driver - able to set position */
		sprintf(buff, "POS %d=%d", drive, intval);
	      }
	    else
	      {
	      trans->state = IDLE_STATE;	/* No command sent to the controller. */
	      sendMsg = false;
	      }

	    break;
	case SET_VEL_BASE:
	    if (dType == PMD8753)
	      {
		if (abs(intval) >= MAX_VELOCITY)
		  intval = MAX_VELOCITY-1;
		/* Set VEL to maximum to eliminate MPV out-of-range error */
		sprintf(buff, "VEL A%d %d=%d", drive, motor, MAX_VELOCITY);
		strcpy(motor_call->message, buff);

		rtnval = motor_end_trans_com(mr, drvtabptr);
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, PMNC87xx_cards);
		motor_call->type = PMNC87xx_table[command];


		sprintf(buff, "MPV A%d %d=%d", drive, motor, abs(intval));
	      }
	    else // if (dType == PMD8751)
	      {
	      trans->state = IDLE_STATE;	/* No command sent to the controller. */
	      sendMsg = false;
	      }

	    break;
	case SET_VELOCITY:
	    if (abs(intval) > MAX_VELOCITY)
   	        intval = MAX_VELOCITY;
	    sprintf(buff, "VEL A%d %d=%d", drive, motor, abs(intval));
	    break;
	case SET_ACCEL:
	    /* 
	     * The value passed is in steps/sec/sec.  
	     */
	    if (intval < MIN_ACCEL)
	      intval = MIN_ACCEL;
	    else if (intval > MAX_ACCEL)
	      intval = MAX_ACCEL;

	    sprintf(buff, "ACC A%d %d=%d", drive, motor, intval);
	    break;
	case GO:
	    sprintf(buff, "GO A%d", drive); 
	    break;
	case SET_ENC_RATIO:
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */
	    sendMsg = false;
	    break;
        case PRIMITIVE:
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	case STOP_AXIS:
	    sprintf(buff, "HAL A%d", drive);  /* Using the smooth stop command */
	    break;
	case JOG:
	     if (abs(intval) > MAX_VELOCITY)
	       velval = MAX_VELOCITY;
	     else
	       velval = abs(intval);

	    if (dType == PMD8753)
	      {
		if (intval >= 0)
		  sprintf(buff, "FOR A%d=%d", drive, velval);
		else
		  sprintf(buff, "REV A%d=%d", drive, velval);
	      }
	    else if (dType == PMD8751)
	      {
		sprintf(buff, "VEL A%d %d=%d", drive, motor, velval);

		strcpy(motor_call->message, buff);
		rtnval = motor_end_trans_com(mr, drvtabptr);
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, PMNC87xx_cards);
		motor_call->type = PMNC87xx_table[command];
		
		// Use Relative Move command for jogging 
		// because the 8751 does not indicate motion with (FOR and REV)
		// ** BUG? **
		if (intval >= 0)
		  sprintf(buff, "REL A%d=1000000",drive);  
		else
		  sprintf(buff, "REL A%d=-1000000",drive);
              }
	    else
              {
	      sendMsg = false;
              break;
              }

	    strcpy(motor_call->message, buff);
	    rtnval = motor_end_trans_com(mr, drvtabptr);
	    rtnval = (RTN_STATUS) motor_start_trans_com(mr, PMNC87xx_cards);
 	    motor_call->type = PMNC87xx_table[command];

	    sprintf(buff, "GO A%d", drive); 
	    break;
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */
 	    sendMsg = false;
	    break;
	case ENABLE_TORQUE:
	  if (dType == PMD8751)
	    sprintf(buff, "SER A%d", drive);  
	  else if (dType == PMD8753)
	    sprintf(buff, "MON A%d", drive);  
	  else
	    {
	      trans->state = IDLE_STATE;	/* No command sent to the controller. */
	      sendMsg = false;
	    }

	    break;
	case DISABL_TORQUE:
	  if (dType == PMD8751)
	    sprintf(buff, "NOS A%d", drive);  
	  else if (dType == PMD8753)
	    sprintf(buff, "MOF A%d", drive);  
	  else
	    {
	      trans->state = IDLE_STATE;	/* No command sent to the controller. */
	      sendMsg = false;
	    }
	    break;
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */
 	    sendMsg = false;
	    break;
	default:
 	    sendMsg = false;
	    rtnval = ERROR;
    }


    if (sendMsg == true)
      {
	size = strlen(buff);

        if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	  errlogMessage("PMNC87xx_build_trans(): buffer overflow.\n");
	else
	  {
	    strcat(motor_call->message, buff);
	    motor_end_trans_com(mr, drvtabptr);
	  }
      }

    return(rtnval);
}
