/*
FILENAME...	devMXmotor.cc
USAGE...	Motor record device level support for MX device driver.

Version:	$Revision: 1.2 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-03-04 15:26:47 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 06/15/99
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 */

#include <registryDriverSupport.h>

#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"

#include "MXmotor.h"

extern int MXmotor_num_cards;
extern struct driver_table MXmotor_access;

/* ----------------Create the dsets for devMXmotor----------------- */
static long MXmotor_init(void *);
static long MXmotor_init_record(void *);
static long MXmotor_start_trans(struct motorRecord *);
static RTN_STATUS MXmotor_build(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS MXmotor_end_trans(struct motorRecord *);

struct motor_dset devMXmotor =
{
    {8, NULL, MXmotor_init, MXmotor_init_record, NULL},
    motor_update_values,
    MXmotor_start_trans,
    MXmotor_build,
    MXmotor_end_trans
};


/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types MXmotor_table[] = {
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

static struct board_stat **MXmotor_cards;
static const char errmsg[] = {"\n\n!!!ERROR!!! - MX driver uninitialized.\n"};

/* initialize device support for MX motor */
static long MXmotor_init(void *after)
{
    int before_after = (int) after;

    if (*(MXmotor_access.init_indicator) == NO)
    {
	errlogSevPrintf(errlogMinor, "%s", errmsg);
    	return(ERROR);
    }
    else
	return(motor_init_com(before_after, MXmotor_num_cards, &MXmotor_access,
			      &MXmotor_cards));
}


static long MXmotor_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    struct motor_trans *trans;
    struct controller *brdptr;
    struct MXcontroller *cntrl;
    MX_RECORD *motor_record;
    long rtnval;

    motor_record = mx_get_record(MXmotor_record_list, mr->name);
    if (motor_record == NULL)
    {
	printf("Motor '%s' does not exist.\n", mr->name);
	return(ERROR);
    }

    rtnval = motor_init_record_com(mr, MXmotor_num_cards, &MXmotor_access, MXmotor_cards);
    
    trans = (struct motor_trans *) mr->dpvt;
    brdptr = (*trans->tabptr->card_array)[mr->card];
    if (brdptr == NULL)
	return(rtnval = ERROR);
    cntrl = (struct MXcontroller *) brdptr->DevicePrivate;
    cntrl->MXmotor_record = motor_record;

    return(rtnval);
}


static long MXmotor_start_trans(struct motorRecord *mr)
{
    long rtnval;
    return(rtnval);
}


static RTN_STATUS MXmotor_build(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    unsigned int size;
    char buff[110];
    RTN_STATUS rtnval = OK;
    bool send = true;	/* Default to send motor command. */
    
    struct controller *brdptr;
    struct MXcontroller *cntrl;

    brdptr = (*trans->tabptr->card_array)[mr->card];
    if (brdptr == NULL)
	return(rtnval = ERROR);
    cntrl = (struct MXcontroller *) brdptr->DevicePrivate;

    buff[0] = '\0';

    motor_start_trans_com(mr, MXmotor_cards);

    motor_call = &(trans->motor_call);

    if (MXmotor_table[command] > motor_call->type)
	motor_call->type = MXmotor_table[command];

    switch (command)
    {	
	case MOVE_ABS:
	    sprintf(buff, "%d %f", command, *parms);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "%d %f", command, *parms);
	    break;
	
	case HOME_FOR:
	    sprintf(buff, "%d %d", command, 1);
	    break;
	
	case HOME_REV:
	    sprintf(buff, "%d %d", command, -1);
	    break;

	case LOAD_POS:
	    sprintf(buff, "%d %f", command, *parms);
	    break;

	case SET_VEL_BASE:
	case SET_VELOCITY:
	case SET_ACCEL:
	    sprintf(buff, "%d %f", command, *parms);
	    break;
		
	case GO:
	    send = false;
	    break;

	case GET_INFO:
	    /* This command is not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;

	case STOP_AXIS:
	    sprintf(buff, "%d", command);
	    break;
	
	default:
	    send = false;
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (send == false)
	return(rtnval);
    else if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	printf("MXmotor_build(): buffer overflow.\n");
    else
    {
	strcat(motor_call->message, buff);
	motor_end_trans_com(mr, &MXmotor_access);
    }    
    return(rtnval);
}


static RTN_STATUS MXmotor_end_trans(struct motorRecord *mr)
{
    RTN_STATUS rtnval = OK;
    return(rtnval);
}

