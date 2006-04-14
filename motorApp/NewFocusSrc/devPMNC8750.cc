/*
FILENAME...	devPMNC8750.cc
USAGE...	Motor record device level support for NewFocus PMNC8750.

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
 * .04  01-10-06 jps initialized from drvMM3000
 */


#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPMNCCom.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table PMNC8750_access;

/* ----------------Create the dsets for devPMNC8750----------------- */
/* static long report(); */
STATIC struct driver_table *drvtabptr;
STATIC long PMNC8750_init(void *);
STATIC long PMNC8750_init_record(void *);
STATIC long PMNC8750_start_trans(struct motorRecord *);
STATIC RTN_STATUS PMNC8750_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS PMNC8750_end_trans(struct motorRecord *);

struct motor_dset devPMNC8750 =
{
    {8, NULL, (DEVSUPFUN) PMNC8750_init, (DEVSUPFUN) PMNC8750_init_record, NULL},
    motor_update_values,
    PMNC8750_start_trans,
    PMNC8750_build_trans,
    PMNC8750_end_trans
};

extern "C" {epicsExportAddress(dset,devPMNC8750);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types PMNC8750_table[] = {
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


static struct board_stat **PMNC8750_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PMNC8750 stepper motor */
STATIC long PMNC8750_init(void *arg)
{
    long rtnval;
    int after = (int) arg;

    if (after == 0)
    {
	drvtabptr = &PMNC8750_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PMNC8750_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long PMNC8750_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PMNC8750_cards));
}


/* start building a transaction */
STATIC long PMNC8750_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, PMNC8750_cards));
}


/* end building a transaction */
STATIC RTN_STATUS PMNC8750_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC RTN_STATUS PMNC8750_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct mess_info *motor_info;
    struct controller *brdptr;
    struct PMNCcontroller *cntrl;
    char buff[110];
    int axis, card;
    int drive, motor;
    unsigned int size;
    int intval;
    RTN_STATUS rtnval;
    bool sendMsg;

    rtnval = OK;
    buff[0] = '\0';
    sendMsg = true;

    motor_start_trans_com(mr, PMNC8750_cards);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal;

    motor_call->type = PMNC8750_table[command];

    drive = (axis / PMNC8750_NUM_MOTORS) + 1; /* First drive = A1 */
    motor = axis % PMNC8750_NUM_MOTORS;

    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);
    cntrl = (struct PMNCcontroller *) brdptr->DevicePrivate;

    if (PMNC8750_table[command] > motor_call->type)
	motor_call->type = PMNC8750_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
	strcat(motor_call->message, mr->init);

    switch (command)
    {
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	               break;
	case MOVE_ABS:
	case MOVE_REL:
	case HOME_FOR:
	case HOME_REV:
	case JOG:
	    if (strlen(mr->prem) != 0)
	    {
		strcat(motor_call->message, mr->prem);
                rtnval = motor_end_trans_com(mr, drvtabptr);
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, PMNC8750_cards);
		motor_call->type = PMNC8750_table[command];

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
   	    sprintf(buff, "REL A%d=%d", drive, intval-motor_info->position);
	    break;
	case MOVE_REL:
	    sprintf(buff, "REL A%d=%d", drive, intval);
	    break;
	case HOME_FOR:
	case HOME_REV:
	    sendMsg = false;
	    break;
	case LOAD_POS:
	    /* Setting driver position because the controller cannot be set */
            motor_info->position = intval;
	    sprintf(buff, "CHL A%d=%d", drive, motor);
	    break;
	case SET_VEL_BASE:
     	    if (abs(intval) > 1999)
		 intval = 1999;
            /* Set VEL to maximum to eliminate MPV out-of-range error */
            sprintf(buff, "VEL A%d %d=2000", drive, motor);
            strcpy(motor_call->message, buff);

            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, PMNC8750_cards);
	    motor_call->type = PMNC8750_table[command];


            sprintf(buff, "MPV A%d %d=%d", drive, motor, abs(intval));

	    break;
	case SET_VELOCITY:
     	    if (abs(intval) > 2000)
   	        intval = 2000;
	    sprintf(buff, "VEL A%d %d=%d", drive, motor, abs(intval));
	    break;
	case SET_ACCEL:
	    /* 
	     * The value passed is in steps/sec/sec.  
	     */
	    sprintf(buff, "ACC A%d %d=%d", drive, motor, intval);
	    break;
	case GO:
	    sprintf(buff, "GO A%d", drive);
	    break;
	case SET_ENC_RATIO:
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
  	    if (intval >= 0)
	      sprintf(buff, "FOR A%d=%d", drive, intval);
            else
	      sprintf(buff, "REV A%d=%d", drive, abs(intval));

            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);
            rtnval = (RTN_STATUS) motor_start_trans_com(mr, PMNC8750_cards);
            motor_call->type = PMNC8750_table[command];

	    sprintf(buff, "GO A%d",drive);

	    break;
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
 	    sendMsg = false;
	    break;
	case ENABLE_TORQUE:
	    sprintf(buff, "MON A%d", drive);  
	    break;
	case DISABL_TORQUE:
	    sprintf(buff, "HOF A%d", drive);  
	    break;
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
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
	  errlogMessage("PMNC8750_build_trans(): buffer overflow.\n");
	else
	  {
	    strcat(motor_call->message, buff);
	    motor_end_trans_com(mr, drvtabptr);
	  }
      }

    return(rtnval);
}
