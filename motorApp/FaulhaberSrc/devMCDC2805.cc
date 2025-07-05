/*
FILENAME...	devMCDC2805.cc
USAGE...	Motor record device level support for Intelligent Motion
		Systems, Inc. MCDC2805 series of controllers.

*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/05
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
 * .01  10/20/05 mlr Initialize from ImsSrc/devMDrive.cc
 */

#include <string.h>
#include <errlog.h>

#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvMCDC2805.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table MCDC2805_access;

/* ----------------Create the dsets for devMCDC2805----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long MCDC2805_init(int);
STATIC long MCDC2805_init_record(void *);
STATIC long MCDC2805_start_trans(struct motorRecord *);
STATIC RTN_STATUS MCDC2805_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS MCDC2805_end_trans(struct motorRecord *);

struct motor_dset devMCDC2805 =
{
    {8, NULL, (DEVSUPFUN) MCDC2805_init, (DEVSUPFUN) MCDC2805_init_record, NULL},
    motor_update_values,
    MCDC2805_start_trans,
    MCDC2805_build_trans,
    MCDC2805_end_trans
};

extern "C" {epicsExportAddress(dset,devMCDC2805);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types MCDC2805_table[] = {
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


static struct board_stat **MCDC2805_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for MCDC2805 stepper motor */
STATIC long MCDC2805_init(int after)
{
    long rtnval;

    if (!after)
    {
	drvtabptr = &MCDC2805_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MCDC2805_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long MCDC2805_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, MCDC2805_cards));
}


/* start building a transaction */
STATIC long MCDC2805_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
STATIC RTN_STATUS MCDC2805_end_trans(struct motorRecord *mr)
{
    return(OK);
}


/* add a part to the transaction */
STATIC RTN_STATUS MCDC2805_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct MCDC2805controller *cntrl;
    char buff[110];
    int axis, card, intval;
    int rpm;
    unsigned int size;
    RTN_STATUS rtnval;
    bool send;
    msta_field msta;

    send = true;		/* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    intval = (parms == NULL) ? 0 : NINT(parms[0]);

    msta.All = mr->msta;

    motor_start_trans_com(mr, MCDC2805_cards);
    
    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct MCDC2805controller *) brdptr->DevicePrivate;
    
    if (MCDC2805_table[command] > motor_call->type)
	motor_call->type = MCDC2805_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
	strcat(motor_call->message, " ");
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
	    sprintf(buff, "LA %d", intval);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "LR %d", intval);
	    break;
	
	case HOME_FOR:
	case HOME_REV:
            rpm = NINT(mr->velo / mr->srev * 60.);
            if (command == HOME_REV) rpm = -rpm;
            /* Need to send multiple messages */
	    sprintf(buff, "ENCRES%d", mr->srev);
            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCDC2805_cards);
	    sprintf(buff, "HOSP%d", rpm);
            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCDC2805_cards);
	    sprintf(buff, "GOHOSEQ");
            motor_call->type = MCDC2805_table[command];
	    break;

	case LOAD_POS:
	    sprintf(buff, "HO %d", intval);
	    break;
	
	case SET_VEL_BASE:
            /* Not supported */
            send = false;
	    break;
	
	case SET_VELOCITY:
            /* The input speed is in steps/second.  The controller wants RPM */
            rpm = NINT(parms[0] / mr->srev * 60.);
            /* Need to send multiple messages */
	    sprintf(buff, "ENCRES%d", mr->srev);
            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCDC2805_cards);
	    sprintf(buff, "SP %d", rpm);
            motor_call->type = MCDC2805_table[command];
	    break;
	
	case SET_ACCEL:
            /* The input speed is in steps/s/s.  The controller wants Revs/s/s */
            rpm = NINT(parms[0] / mr->srev);
            /* Need to send multiple messages */
	    sprintf(buff, "ENCRES%d", mr->srev);
            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCDC2805_cards);
	    sprintf(buff, "AC %d", rpm);
            motor_call->type = MCDC2805_table[command];
	    break;
	
	case GO:
	    sprintf(buff, "M");
	    break;
	
	case PRIMITIVE:
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	    sprintf(buff, "V 0");
	    break;
	
	case JOG_VELOCITY:
	case JOG:
            rpm = NINT(parms[0] / mr->srev * 60.);
            /* Need to send multiple messages */
	    sprintf(buff, "ENCRES%d", mr->srev);
            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCDC2805_cards);
	    sprintf(buff, "SP %d", rpm);
            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, MCDC2805_cards);
	    sprintf(buff, "V %d", rpm);
            motor_call->type = MCDC2805_table[command];
	    break;
	
	case SET_PGAIN:
	    sprintf(buff, "POR %ld", NINT(parms[0]*255.));
	    break;

	case SET_IGAIN:
	    sprintf(buff, "I %ld", NINT(parms[0]*255.));
	    break;

	case SET_DGAIN:
	    send = false;
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "EN");
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "DI");
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
	errlogMessage("MCDC2805_build_trans(): buffer overflow.\n");
    else
    {
	strcat(motor_call->message, buff);
	motor_end_trans_com(mr, drvtabptr);
    }
    return(rtnval);
}
