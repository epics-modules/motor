/*
FILENAME...	devPM500.c
USAGE...	Motor record device level support for the Newport PM500 motor
		controller.

Version:	$Revision: 1.5 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2002-07-05 19:25:38 $
*/

/*
 *      Original Author: Mark Rivers
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
 * .00  10-25-98	mlr     initialized from devPM500
 * .01  06-02-00 rls	integrated into standard motor record
 * .05  04-21-01	rls	Added jog velocity motor command. 
 */


#include	<vxWorks.h>
#include	<stdio.h>
#include	<string.h>
#include        <semLib.h>	/* jps: include for init_record wait */
#include	<logLib.h>

#ifdef __cplusplus
extern "C" {
#include	<epicsDynLink.h>
}
#else
#include	<epicsDynLink.h>
#endif
#include	<sysSymTbl.h>	/* for sysSymTbl*/

#include	<alarm.h>
#include	<callback.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<fast_lock.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<drvSup.h>

#include	"motorRecord.h"
#include	"motor.h"
#include	"motordevCom.h"
#include	"drvMMCom.h"

#define STATIC static

/* ----------------Create the dsets for devPM500----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long PM500_init(int);
STATIC long PM500_init_record(struct motorRecord *);
STATIC long PM500_start_trans(struct motorRecord *);
STATIC long PM500_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC long PM500_end_trans(struct motorRecord *);

static char PM500_axis_names[] = {'X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E', 'F',
    'G', 'H', 'I'};

struct motor_dset devPM500 =
{
    {8, NULL, PM500_init, PM500_init_record, NULL},
    motor_update_values,
    PM500_start_trans,
    PM500_build_trans,
    PM500_end_trans
};


/* --------------------------- program data --------------------- */

/* This table is used to define the command types */

static int PM500_table[] = {
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


static struct board_stat **PM500_cards;

/* --------------------------- program data --------------------- */


/* Initialize device support for PM500 controller. */
STATIC long PM500_init(int after)
{
    SYM_TYPE type;
    long rtnval;

    if (after == 0)
    {
	rtnval = symFindByNameEPICS(sysSymTbl, "_PM500_access",
		    (char **) &drvtabptr, &type);
	if (rtnval != OK)
	    return(rtnval);
    /*
	IF before DB initialization.
	    Initialize PM500 driver (i.e., call init()). See comment in
		drvPM500.c init().
	ENDIF
    */
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PM500_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long PM500_init_record(struct motorRecord *mr)
{
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PM500_cards));
}


/* start building a transaction */
STATIC long PM500_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, PM500_cards));
}


/* end building a transaction */
STATIC long PM500_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC long PM500_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct MMcontroller *cntrl;
    char axis_name;
    char buff[110];
    int axis, card, maxdigits, size;
    double dval, cntrl_units;
    long rtnval;

    rtnval = OK;
    buff[0] = '\0';
    dval = *parms;

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);

    axis_name = PM500_axis_names[axis - 1];

    cntrl = (struct MMcontroller *) brdptr->DevicePrivate;
    cntrl_units = dval * cntrl->drive_resolution[axis - 1];
    maxdigits = cntrl->res_decpts[axis - 1];
    
    if (PM500_table[command] > motor_call->type)
	motor_call->type = PM500_table[command];

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
	    sprintf(buff, "%cG%.*f;", axis_name, maxdigits, cntrl_units);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "%cR%.*f;", axis_name, maxdigits, cntrl_units);
	    break;
	
	case HOME_FOR:
	case HOME_REV:
	    sprintf(buff, "%cF0;", axis_name);
	    break;
	
	case LOAD_POS:
	    break;
	
	case SET_VEL_BASE:
	    break;	    /* PM500 does not use base velocity */
	
	case SET_VELOCITY:
	    /* PM500 uses mm/sec and karc-sec/sec for velocity, but microns and
	     * arc-sec for position.  Divide by 1000 here.
	     */
	    cntrl_units /= 1000.0;
	    sprintf(buff, "%cV%.*f;", axis_name, maxdigits, cntrl_units);
	    break;
	
	case SET_ACCEL:
	    /* PM500 uses mm/sec^2 and karc-sec/sec^2 for acceleration, but
	     * microns and arc-sec for position.  Divide by 1000 here.
	     */
	    cntrl_units /= 1000.0;
	    sprintf(buff, "%cACCEL%.*f;", axis_name, maxdigits, cntrl_units);
	    break;
	
	case GO:
	    /* 
	     * The PM500 starts moving immediately on move commands, GO command
	     * does nothing
	     */
	    break;
	
	case SET_ENC_RATIO:
	    /*
	     * The PM500 does not have the concept of encoder ratio, ignore this
	     * command
	     */
	    break;
	
	case GET_INFO:
	    sprintf(buff, "%cR;", axis_name);
	    break;
	
	case STOP_AXIS:
	    sprintf(buff, "%cT;", axis_name);
	    break;
	
	case JOG:
	    /* PM500 uses mm/sec and karc-sec/sec for velocity, but microns and
	     * arc-sec for position.  Divide by 1000 here.
	     */
	    cntrl_units /= 1000.0;
	    sprintf(buff, "%cS%f;", axis_name, cntrl_units);
           break;
	
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    break;

	case ENABLE_TORQUE:
	    sprintf(buff, "%cT;", axis_name);
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "%cM;", axis_name);
	    break;
	
	case SET_HIGH_LIMIT:
	    sprintf(buff, "%cPSLIM%.*f;", axis_name, maxdigits, cntrl_units);
	    break;
	
	case SET_LOW_LIMIT:
	    sprintf(buff, "%cNSLIM%.*f;", axis_name, maxdigits, cntrl_units);
	    break;
	
	default:
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	logMsg((char *) "devMM4000.c:MM4000_build_trans(): buffer overflow.\n",
	   0, 0, 0, 0, 0, 0);
    else
	strcat(motor_call->message, buff);

    return(rtnval);
}
