/*
FILENAME...	devMM4000.c
USAGE...	Motor record device level support for Newport MM4000.

Version:	$Revision: 1.5 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2002-04-01 22:44:16 $
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
 * .01  10-20-97	mlr     initialized from drvOms58
 * .02  10-30-97        mlr     Replaced driver calls with gpipIO functions
 * .03  10-30-98        mlr     Minor code cleanup, improved formatting
 * .04  02-01-99        mlr     Added temporary fix to delay reading motor
 *                              positions at the end of a move.
 * .05  04-21-01	rls	Added jog velocity motor command.
 */


#include	<vxWorks.h>
#include	<stdio.h>
#include	<string.h>
#include	<math.h>
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

/* ----------------Create the dsets for devMM4000----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long MM4000_init(int);
STATIC long MM4000_init_record(struct motorRecord *);
STATIC long MM4000_start_trans(struct motorRecord *);
STATIC long MM4000_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC long MM4000_end_trans(struct motorRecord *);

struct motor_dset devMM4000 =
{
    {8, NULL, MM4000_init, MM4000_init_record, NULL},
    motor_update_values,
    MM4000_start_trans,
    MM4000_build_trans,
    MM4000_end_trans
};


/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static int MM4000_table[] = {
    MOTION, 	/* MOVE_ABS */
    MOTION, 	/* MOVE_REL */
    MOTION, 	/* HOME_FOR */
    MOTION, 	/* HOME_REV */
    IMMEDIATE,	/* LOAD_POS */
    IMMEDIATE,	/* SET_VEL_BASE */
    IMMEDIATE,	/* SET_VELOCITY */
    IMMEDIATE,	/* SET_ACCEL */
    IMMEDIATE,	/* GO */
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


static struct board_stat **MM4000_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for MM4000 stepper motor */
STATIC long MM4000_init(int after)
{
    SYM_TYPE type;
    long rtnval;

    if (after == 0)
    {
	rtnval = symFindByNameEPICS(sysSymTbl, "_MM4000_access",
		    (char **) &drvtabptr, &type);
	if (rtnval != OK)
	    return(rtnval);
    /*
	IF before DB initialization.
	    Initialize MM4000 driver (i.e., call init()). See comment in
		drvMM4000.c init().
	ENDIF
    */
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MM4000_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long MM4000_init_record(struct motorRecord *mr)
{
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, MM4000_cards));
}


/* start building a transaction */
STATIC long MM4000_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, MM4000_cards));
}


/* end building a transaction */
STATIC long MM4000_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC long MM4000_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct mess_info *motor_info;
    struct MMcontroller *cntrl;
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

    cntrl = (struct MMcontroller *) brdptr->DevicePrivate;
    cntrl_units = dval * cntrl->drive_resolution[axis - 1];
    maxdigits = cntrl->res_decpts[axis - 1];
    
    if (MM4000_table[command] > motor_call->type)
	motor_call->type = MM4000_table[command];

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
	    sprintf(buff, "%dPA%.*f;", axis, maxdigits, cntrl_units);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "%dPR%.*f;", axis, maxdigits, cntrl_units);
	    break;
	
	case HOME_FOR:
	case HOME_REV:
	    sprintf(buff, "%dOR;", axis);
	    break;
	
	case LOAD_POS:
	    if (cntrl->model == MM4000)
		sprintf(buff, "%dSH%.*f;%dDH;%dSH%.*f", axis, maxdigits, cntrl_units,
		    axis, axis, maxdigits, cntrl->home_preset[axis - 1]);
	    break;
	
	case SET_VEL_BASE:
	    break;	    /* MM4000 does not use base velocity */
	
	case SET_VELOCITY:
	    sprintf(buff, "%dVA%.*f;", axis, maxdigits, cntrl_units);
	    break;
	
	case SET_ACCEL:
	    /* 
	     * The value passed is in steps/sec/sec.  
	     * Convert to user units/sec/sec
	     */
	    sprintf(buff, "%dAC%.*f;", axis, maxdigits, cntrl_units);
	    break;
	
	case GO:
	    /* 
	     * The MM4000 starts moving immediately on move commands, GO command
	     * does nothing
	     */
	    break;
	
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	    sprintf(buff, "%dST;", axis);
	    break;
	
	case JOG:
	    /* 
	    * MM4000 does not have a jog command.  Simulate with move absolute
	    * to the appropriate software limit. We can move to MM4000 soft limits.
	    * If the record soft limits are set tighter than the MM4000 limits
	    * the record will prevent JOG motion beyond its soft limits 
	    */
	    sprintf(buff, "%dVA%.*f;", axis, maxdigits, fabs(cntrl_units));
	    strcat(motor_call->message, buff);
	    if (dval > 0.)
		sprintf(buff, "%dPA%.*f;", axis, maxdigits, mr->dhlm);
	    else
		sprintf(buff, "%dPA%.*f;", axis, maxdigits, mr->dllm);
	    break;
	
	case SET_PGAIN:
	    sprintf(buff, "%dKP%f;%dUF;", axis, dval, axis);
	    break;
	
	case SET_IGAIN:
	    sprintf(buff, "%dKI%f;%dUF;", axis, dval, axis);
	    break;
	
	case SET_DGAIN:
	    sprintf(buff, "%dKD%f;%dUF;", axis, dval, axis);
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "MO;");
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "MF;");
	    break;
	
	case SET_HIGH_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis - 1];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    if (cntrl_units > motor_info->high_limit)
	    {
		mr->dhlm = motor_info->high_limit;
		rtnval = ERROR;
	    }
	    break;
	
	case SET_LOW_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis - 1];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    if (cntrl_units < motor_info->low_limit)
	    {
		mr->dllm = motor_info->low_limit;
		rtnval = ERROR;
	    }
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
