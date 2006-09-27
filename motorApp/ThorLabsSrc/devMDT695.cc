/*
FILENAME...	devMDT695.cc
USAGE...	Motor record device level support for ThorLabs Piezo Control
                Module (MDT695)

Version:	$Revision: 1.1 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2006-09-27 20:32:37 $
*/

/*
 *      Original Author: Joe Sullivan
 *      Date: 9/27/2006
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
 * .01  09-27-06    jps     initialized from devEMC18011.cc
 */


#include <string.h>
#include <math.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvMDT695.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table MDT695_access;

/* ----------------Create the dsets for devMDT695----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long MDT695_init(void *);
STATIC long MDT695_init_record(void *);
STATIC long MDT695_start_trans(struct motorRecord *);
STATIC RTN_STATUS MDT695_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS MDT695_end_trans(struct motorRecord *);

struct motor_dset devMDT695 =
{
    {8, NULL, (DEVSUPFUN) MDT695_init, (DEVSUPFUN) MDT695_init_record, NULL},
    motor_update_values,
    MDT695_start_trans,
    MDT695_build_trans,
    MDT695_end_trans
};

extern "C" {epicsExportAddress(dset,devMDT695);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types MDT695_table[] = {
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


static struct board_stat **MDT695_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for MDT695 stepper motor */
STATIC long MDT695_init(void *arg)
{
    long rtnval;
    int after = (int) arg;

    if (after == 0)
    {
	drvtabptr = &MDT695_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MDT695_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long MDT695_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    /* Disable change of direction testing in record support */
    /* This device does it's own backlash correction */
    mr->ntm = menuYesNoNO;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, MDT695_cards));
}


/* start building a transaction */
STATIC long MDT695_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, MDT695_cards));
}


/* end building a transaction */
STATIC RTN_STATUS MDT695_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC RTN_STATUS MDT695_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    // struct mess_info *motor_info;
    struct MDT695Controller *cntrl;
    char buff[110];
    int signal, card, intval;
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

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    signal = motor_call->signal;

    brdptr = (*trans->tabptr->card_array)[card];
      if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct MDT695Controller *) brdptr->DevicePrivate;
    /* 6K Controllers expect Velocity and Acceleration settings in Revs/sec/sec */
    cntrl_units = dval * cntrl->drive_resolution;

    
    if (MDT695_table[command] > motor_call->type)
	motor_call->type = MDT695_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
	strcat(motor_call->message, mr->init);
	strcat(motor_call->message, MDT695_OUT_EOS);
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
		strcat(motor_call->message, MDT695_OUT_EOS);

	    }
	    if (strlen(mr->post) != 0)
		motor_call->postmsgptr = (char *) &mr->post;
	    break;
        
	default:
	    break;
    }

    /* ThorLabs Piezo driver - voltage set command only */
    switch (command)
    {
        case MOVE_REL:
	  // No relative move available
	  break;
        case MOVE_ABS:
	  sprintf(buff, "#V%.*f", 1, cntrl_units);
	  break;
	
	case HOME_FOR:
	case HOME_REV:
	    break;
	
	case LOAD_POS:
	    break;
	
	case SET_VEL_BASE:
	    break;	    /* MDT695 does not use base velocity */
	
	case SET_VELOCITY:
	  break;
	
	case SET_ACCEL:
	  break;
	
	case GO:
	  break;
	
	case SET_ENC_RATIO:
	    rtnval = ERROR;
	  break;
	
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	  break;
	
	case STOP_AXIS:
	  break;
	
	case JOG:
	  break;
	
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    rtnval = ERROR;
	    break;
	
	case ENABLE_TORQUE:
	case DISABL_TORQUE:
	    rtnval = ERROR;
	    break;
	
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	    rtnval = ERROR;
	    break;
	
	default:
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
      errlogMessage("MDT695_build_trans(): buffer overflow.\n");
    else
      {
	strcat(motor_call->message, buff);
      }

    return(rtnval);
}
