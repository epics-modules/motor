/*
FILENAME...	devPIE816.cc
USAGE...	Motor record device level support for Physik Instrumente (PI)
		GmbH & Co. E-816 motor controller.

Version:	1.1
Modified By:	sullivan
Last Modified:	2007/03/30 20:01:05
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 12/17/03
 *      Current Author: Joe Sullivan
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
 * .01 01/29/06 jps - copied from devPIE710.cc
 */							      


#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPIE816.h"
#include "epicsExport.h"

extern struct driver_table PIE816_access;

/* ----------------Create the dsets for devPIE816----------------- */
static struct driver_table *drvtabptr;
static long PIE816_init(void *);
static long PIE816_init_record(void *);
static long PIE816_start_trans(struct motorRecord *);
static RTN_STATUS PIE816_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS PIE816_end_trans(struct motorRecord *);

struct motor_dset devPIE816 =
{
    {8, NULL, (DEVSUPFUN) PIE816_init, (DEVSUPFUN) PIE816_init_record, NULL},
    motor_update_values,
    PIE816_start_trans,
    PIE816_build_trans,
    PIE816_end_trans
};

extern "C" {epicsExportAddress(dset,devPIE816);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types PIE816_table[] = {
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


static struct board_stat **PIE816_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PIE816 stepper motor */
static long PIE816_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
	drvtabptr = &PIE816_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PIE816_cards);
    return(rtnval);
}


/* initialize a record instance */
static long PIE816_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    /* Disable change of direction testing in record support */
    /* This is a closed-loop device */
    mr->ntm = menuYesNoNO;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PIE816_cards));
}


/* start building a transaction */
static long PIE816_start_trans(struct motorRecord *mr)
{
    motor_start_trans_com(mr, PIE816_cards);
    return(OK);
}


/* end building a transaction */
static RTN_STATUS PIE816_end_trans(struct motorRecord *mr)
{
    motor_end_trans_com(mr, drvtabptr);
    return(OK);
}


/* add a part to the transaction */
static RTN_STATUS PIE816_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct PIE816controller *cntrl;
    char buff[110];
    int card, maxdigits;
    unsigned int size;
    double dval, cntrl_units, res;
    RTN_STATUS rtnval;
    bool send;

    send = true;		/* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';
    
    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;
    
    rtnval = (RTN_STATUS) motor_start_trans_com(mr, PIE816_cards); 

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct PIE816controller *) brdptr->DevicePrivate;
    res = cntrl->drive_resolution[motor_call->signal];
    cntrl_units = dval;
    maxdigits = 3;
    
    if (PIE816_table[command] > motor_call->type)
	motor_call->type = PIE816_table[command];

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
		strcat(motor_call->message, EOL_E816);
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
	    sprintf(buff, "MOV #%.*f", maxdigits, (cntrl_units * res));
	    strcat(buff, EOL_E816);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "MVR #%.*f", maxdigits, (cntrl_units * res));
	    strcat(buff, EOL_E816);
	    break;
	
	case HOME_FOR:
	case HOME_REV:
	    rtnval = ERROR;
	    break;
	
	case LOAD_POS:
	    rtnval = ERROR;
	    break;
	
	case SET_VEL_BASE:
	    send = false;	/* DC motor; not base velocity. */
	    break;
	
	case SET_VELOCITY:
	    sprintf(buff, "VEL #%.*f", maxdigits, (cntrl_units * res));
	    strcat(buff, EOL_E816);
	    break;
	
	case ENABLE_TORQUE:
	    strcpy(buff, "SVO #1");
	    strcat(buff, EOL_E816);
	    break;

	case DISABL_TORQUE:
	    strcpy(buff, "SVO #0");
	    strcat(buff, EOL_E816);
	    break;

	case SET_ACCEL:
	    /* The PIE816 does not support acceleration commands. */
	case GO:
	    /* The PIE816 starts moving immediately on move commands, GO command
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
	  /* No stop command available - use move relative 0 */
	    sprintf(buff, "MVR #0");
	    strcat(buff, EOL_E816);
	    break;
	
	case JOG_VELOCITY:
	case JOG:
 	    sprintf(buff, "VEL #%.*f", maxdigits, cntrl_units);
	    strcat(buff, EOL_E816);
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
	errlogMessage("PIE816_build_trans(): buffer overflow.\n");
    else
    {
	strcat(motor_call->message, buff);
        rtnval = motor_end_trans_com(mr, drvtabptr);
    }
    return(rtnval);
}
