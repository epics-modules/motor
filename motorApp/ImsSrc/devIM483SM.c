/*
FILENAME...	devIM483SM.c
USAGE...	Motor record device level support for Intelligent Motion
		Systems, Inc. IM483(I/IE).

Version:	$Revision: 1.2 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2001-04-21 18:15:43 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 02/10/2000
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
 * .01	02/10/2000	rls	copied from devMM4000.c
 */


#include	<vxWorks.h>
#include	<stdioLib.h>
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
#include	"drvIM483.h"

#define STATIC static

/* ----------------Create the dsets for devIM483SM----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long IM483SM_init(int);
STATIC long IM483SM_init_record(struct motorRecord *);
STATIC long IM483SM_start_trans(struct motorRecord *);
STATIC long IM483SM_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC long IM483SM_end_trans(struct motorRecord *);

struct motor_dset devIM483SM =
{
    {8, NULL, IM483SM_init, IM483SM_init_record, NULL},
    motor_update_values,
    IM483SM_start_trans,
    IM483SM_build_trans,
    IM483SM_end_trans
};


/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static int IM483SM_table[] = {
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


static struct board_stat **IM483SM_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for IM483SM stepper motor */
STATIC long IM483SM_init(int after)
{
    SYM_TYPE type;
    long rtnval;

    if (after == 0)
    {
	rtnval = symFindByNameEPICS(sysSymTbl, "_IM483SM_access",
		    (char **) &drvtabptr, &type);
	if (rtnval != OK)
	    return(rtnval);
    /*
	IF before DB initialization.
	    Initialize IM483SM driver (i.e., call init()). See comment in
		drvIM483SM.c init().
	ENDIF
    */
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &IM483SM_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long IM483SM_init_record(struct motorRecord *mr)
{
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, IM483SM_cards));
}


/* start building a transaction */
STATIC long IM483SM_start_trans(struct motorRecord *mr)
{
    return(motor_start_trans_com(mr, IM483SM_cards));
}


/* end building a transaction */
STATIC long IM483SM_end_trans(struct motorRecord *mr)
{
    return(motor_end_trans_com(mr, drvtabptr));
}


/* add a part to the transaction */
STATIC long IM483SM_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct IM483controller *cntrl;
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

    cntrl = (struct IM483controller *) brdptr->DevicePrivate;
    cntrl_units = dval;
    maxdigits = 2;
    
    if (IM483SM_table[command] > motor_call->type)
	motor_call->type = IM483SM_table[command];

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
	    /* Using ":" to separate target from backlash move command. */
	    sprintf(buff, "R%.*f\x1E", maxdigits, cntrl_units);
	    break;
	
	case MOVE_REL:
	    /* Using ":" to separate target from backlash move command. */
	    sprintf(buff, "%.*f\x1E", maxdigits, cntrl_units);
	    break;
	
	case HOME_FOR:
	    sprintf(buff, "F1000 0");
	    break;

	case HOME_REV:
	    sprintf(buff, "F1000 1");
	    break;
	
	case LOAD_POS:
	    if (cntrl_units == 0.0)
		sprintf(buff, "O");
	    else
		rtnval = ERROR;
	    break;
	
	case SET_VEL_BASE:
	    sprintf(buff, "I%.*f;", maxdigits, cntrl_units);
	    break;
	
	case SET_VELOCITY:
	    sprintf(buff, "V%.*f;", maxdigits, cntrl_units);
	    break;
	
	case SET_ACCEL:
	    /* ????? MAKE SENSE OF ACCELERATION PARAMETER ??????*/
	    break;
	
	case GO:
	    /* The IM483 starts moving immediately on move commands, GO command
	     * does nothing. */
	    break;
	
	case SET_ENC_RATIO:
	    /*
	     * The IM483 does not have the concept of encoder ratio, ignore this
	     * command
	     */
	    break;
	
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	    sprintf(buff, "@ 0");
	    break;
	
	case JOG:
	    /* Can't get jogging to work without communication errors.
	    sprintf(buff, "M%.*f;", maxdigits, cntrl_units);
	    */
	    break;
	
	case SET_PGAIN:
	case SET_IGAIN:
	case SET_DGAIN:
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "MO;");
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "MF;");
	    break;
	
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */
	    break;
	
	default:
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	logMsg((char *) "devIM483SM.c:IM483SM_build_trans(): buffer overflow.\n",
	   0, 0, 0, 0, 0, 0);
    else
	strcat(motor_call->message, buff);

    return(rtnval);
}
