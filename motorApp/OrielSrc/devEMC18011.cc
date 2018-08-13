/*
FILENAME...	devEMC18011.cc
USAGE...	Motor record device level support for Parker Compumotor drivers

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
#include "drvEMC18011.h"
#include "epicsExport.h"


#define STATIC static

extern struct driver_table EMC18011_access;

/* ----------------Create the dsets for devEMC18011----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long EMC18011_init(int);
STATIC long EMC18011_init_record(void *);
STATIC long EMC18011_start_trans(struct motorRecord *);
STATIC RTN_STATUS EMC18011_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS EMC18011_end_trans(struct motorRecord *);

struct motor_dset devEMC18011 =
{
    {8, NULL, (DEVSUPFUN) EMC18011_init, (DEVSUPFUN) EMC18011_init_record, NULL},
    motor_update_values,
    EMC18011_start_trans,
    EMC18011_build_trans,
    EMC18011_end_trans
};

extern "C" {epicsExportAddress(dset,devEMC18011);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types EMC18011_table[] = {
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


static struct board_stat **EMC18011_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for EMC18011 stepper motor */
STATIC long EMC18011_init(int after)
{
    long rtnval;

    if (!after)
    {
	drvtabptr = &EMC18011_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &EMC18011_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long EMC18011_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    /* Disable change of direction testing in record support */
    /* This device does it's own backlash correction */
    mr->ntm = menuYesNoNO;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, EMC18011_cards));
}


/* start building a transaction */
STATIC long EMC18011_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, EMC18011_cards));
}


/* end building a transaction */
STATIC RTN_STATUS EMC18011_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC RTN_STATUS EMC18011_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    // struct mess_info *motor_info;
    struct EMC18011Controller *cntrl;
    char buff[110];
    int signal, axis, card, intval;
    double dval, cntrl_units;
    unsigned int size;
    bool sendMsg;
    RTN_STATUS rtnval;

    rtnval = OK;
    buff[0] = '\0';
    sendMsg = true;

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    intval = (parms == NULL) ? 0 : NINT(parms[0]);
    dval = (parms == NULL) ? 0 : *parms;

    motor_start_trans_com(mr, EMC18011_cards);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    signal = motor_call->signal;
    axis = signal+1; /* Motors start at 1 */

    motor_call->type = EMC18011_table[command];

    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
      return(rtnval = ERROR);

    cntrl = (struct EMC18011Controller *) brdptr->DevicePrivate;

    /* 6K Controllers expect Velocity and Acceleration settings in Revs/sec/sec */
    cntrl_units = dval * cntrl->drive_resolution;

    
    if (EMC18011_table[command] > motor_call->type)
	motor_call->type = EMC18011_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    /* SPECIAL: This controller can only address one motor at a time 
     * Other motion requests have to wait until the current motion 
     * is complete. Depend on motorRecord retries to complete pending 
     * requests. Allow record to stop current motion. 
     * Switch message type to INFO to get done flag 
     */

    switch (command)
      {
      case JOG:
      case LOAD_POS:
      case SET_VELOCITY: 
      case GET_INFO:

	/* Try to get motorLock then switch active motor */
	if (cntrl->motorLock->tryWait())
	  {
	    if (cntrl->motorSelect != signal)
	      {
		cntrl->motorSelect = signal;

		sprintf(buff, "M%d", axis);
     		strcat(motor_call->message, buff);
		motor_call->type = IMMEDIATE;  /* Assure message gets sent */
		rtnval = motor_end_trans_com(mr, drvtabptr);
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, EMC18011_cards);
		motor_call->type = EMC18011_table[command];
	      }
	  }
	break;

      default:
	break;
      }
	
    /* Only communicate to selected motor */
    if (cntrl->motorSelect != signal)
      {
	/* Assure continuous retries until other motion complete */
	motor_call->type = INFO;
	if (mr->rcnt > 0)
	  mr->rcnt--;
	return(rtnval = OK);
      }

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
	strcat(motor_call->message, mr->init);
	rtnval = motor_end_trans_com(mr, drvtabptr);
	rtnval = (RTN_STATUS) motor_start_trans_com(mr, EMC18011_cards);
	motor_call->type = EMC18011_table[command];
    }

    switch (command)
    {
	case MOVE_ABS:
	case MOVE_REL:
	case JOG:
	    if (strlen(mr->prem) != 0)
	    {
		strcat(motor_call->message, mr->prem);
		rtnval = motor_end_trans_com(mr, drvtabptr);
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, EMC18011_cards);
		motor_call->type = EMC18011_table[command];
	    }
	    if (strlen(mr->post) != 0)
		motor_call->postmsgptr = (char *) &mr->post;
	    break;
        
	default:
	    break;
    }

    /* Parker 6K controllers do not support multiple commands per line */
    switch (command)
    {
        case MOVE_ABS:
        case MOVE_REL:
	  {
	    char *xstart = &buff[1];

	    buff[0] = (command == MOVE_ABS) ? 'G' : 'T';
	    
	    /* Maximum length for distance is 7 characters. */
	    sprintf(xstart, "%.1f", cntrl_units);
	    if (strlen(xstart) > 7)
	      /* Sacrifice precision for quantity */
	      sprintf(xstart, "%.0f", cntrl_units);
	    
	    if (strlen(xstart) > 7)
	      {
		/* Put out maximum distance string */
		if (cntrl_units < 0)
		  strcpy(xstart, "-999999");
		else
		  strcpy(xstart, "9999999");
	      }
	  }
	  break;
	
	case HOME_FOR:
	case HOME_REV:
	    rtnval = ERROR;
	    sendMsg = false;
	    break;
	
	case LOAD_POS:
	  /* Only Zero setting is allowed */
	  if (intval == 0)
            strcpy(buff, "CA");
	  else
	    sendMsg = false;

	    break;
	
	case SET_VEL_BASE:
	    sendMsg = false;
	    break;	    /* EMC18011 does not use base velocity */
	
	case SET_VELOCITY:
	  /* Velocity resolution decrease at greater speeds.  */
	  if (cntrl_units < 5.0)
	    sprintf(buff, "V%.2f", cntrl_units);
	  else if (cntrl_units < 50.0)
	    sprintf(buff, "V%.1f", cntrl_units);	    
	  else if (cntrl_units < 200.0)
	    sprintf(buff, "V%.0f", cntrl_units);	    
	  else
	    /* Set speed to maximum */
	    strcpy(buff, "V200");
	  break;
	
	case SET_ACCEL:
	  sendMsg = false;
	  break;
	
	case GO:
	  /* Send command because generic device support will expect a 
	   * a response */
	  strcpy(buff, "A");
	  break;
	
	case SET_ENC_RATIO:
	  rtnval = ERROR;
	  sendMsg = false;
	  break;
	
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	  sendMsg = false;
	  break;
	
	case STOP_AXIS:
	  strcpy(buff, "S");
	  break;
	
	case JOG:

	  /* Velocity resolution decrease at greater speeds. */
	  if (cntrl_units < 5.0)
	    sprintf(buff, "V%.2f", cntrl_units);
	  else if (cntrl_units < 50.0)
	    sprintf(buff, "V%.1f", cntrl_units);	    
	  else if (cntrl_units < 200.0)
	    sprintf(buff, "V%.0f", cntrl_units);	    
	  else
	    /* Set speed to maximum */
	    strcpy(buff, "V200");

	  strcat(motor_call->message, buff);
	    rtnval = motor_end_trans_com(mr, drvtabptr);
	    rtnval = (RTN_STATUS) motor_start_trans_com(mr, EMC18011_cards);
	    motor_call->type = EMC18011_table[command];

   	    if (intval >= 0)
	      strcpy(buff, ">");
            else
	      strcpy(buff, "<");

	    break;
	
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    rtnval = ERROR;
	    sendMsg = false;
	    break;
	
	case ENABLE_TORQUE:
	case DISABL_TORQUE:
	    rtnval = ERROR;
	    sendMsg = false;
	    break;
	
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	    rtnval = ERROR;
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
	  errlogMessage("EMC18011_build_trans(): buffer overflow.\n");
	else
	  {
	  strcat(motor_call->message, buff);
	  motor_end_trans_com(mr, drvtabptr);
	  }
      }

    return(rtnval);
}
