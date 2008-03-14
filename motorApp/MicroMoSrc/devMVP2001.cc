/*
FILENAME...	devMVP2001.cc
USAGE...	Motor record device level support for MicroMo
		MVP 2001 B02 (Linear, RS-485).

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2008-03-14 20:13:20 $
*/

/*
 *      Original Author: Kevin Peterson
 *      Date: 08/27/2002
 *
 *
 *  Illinois Open Source License
 *  University of Illinois
 *  Open Source License
 *
 *
 *  Copyright (c) 2004,  UNICAT.  All rights reserved.
 *
 *
 *  Developed by:
 *
 *  UNICAT, Advanced Photon Source, Argonne National Laboratory
 *
 *  Frederick Seitz Materials Research Laboratory,
 *  University of Illinois at Urbana-Champaign
 *
 *  http://www.uni.aps.anl.gov
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the
 *  "Software"), to deal with the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *
 *  Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimers.
 *
 *
 *  Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimers in the
 *  documentation and/or other materials provided with the distribution.
 *
 *
 *  Neither the names of UNICAT, Frederick Seitz Materials Research 
 *  Laboratory, University of Illinois at Urbana-Champaign,
 *  nor the names of its contributors may be used to endorse or promote
 *  products derived from this Software without specific prior written
 *  permission.
 *
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 *  ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.
 *
 *
 * Modification Log:
 * -----------------
 * .01	08/27/02  kmp  	copied from devIM483PL.c (rev 1.9, mod .04) and
 *			customized for the MVP2001.
 * .02  08/27/02  kmp	changed message construction to allow for addresses
 * 			larger than 9.
 * .03  08/29/02  kmp   fixed the sending of the ANO command.  Previously had
 *			attempted to send it during "case GO:" but now properly
 *			handled using the mr->prem.  Also implemented JOG.
 * .04  08/30/02  kmp   fixed problem with HOMF & HOMR.  The MVP2001 does not
 *			home commands.  The correct way to disable the HOMF &
 *			HOMR is to simply break from the switch statement.
 *			Doing anything else (i.e. "send = OFF;" or 
 *			"trans->state = IDLE_STATE;") causes the MEDM window
 *			to get stuck in the "Moving" state even though the motor
 *			is not moving.  Use a current version of MEDM.
 * .05	10/02/02  kmp	changed default current limit (in case where desired
 *			current limit is out of range) from 500mA to 100mA.
 * .06	02/13/04  rls	port to R3.14.x
 *
 */

#include <string.h>
#include <math.h>
#include <ctype.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvMVP2001.h"
#include "epicsExport.h"

extern struct driver_table MVP2001_access;

#define BUFF_SIZE 20		/* Maximum length of string to/from MVP2001 */

/* ----------------Create the dsets for devMVP2001----------------- */
static struct driver_table *drvtabptr;
static long MVP2001_init(void *);
static long MVP2001_init_record(void *);
static long MVP2001_start_trans(struct motorRecord *);
static RTN_STATUS MVP2001_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS MVP2001_end_trans(struct motorRecord *);

struct motor_dset devMVP2001 =
{
    {8, NULL, (DEVSUPFUN) MVP2001_init, (DEVSUPFUN) MVP2001_init_record, NULL},
    motor_update_values,
    MVP2001_start_trans,
    MVP2001_build_trans,
    MVP2001_end_trans
};

extern "C" {epicsExportAddress(dset,devMVP2001);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types MVP2001_table[] = {
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


static struct board_stat **MVP2001_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for MVP2001 DC motor */
static long MVP2001_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
	drvtabptr = &MVP2001_access;
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MVP2001_cards);
    return(rtnval);
}


/* initialize a record instance */
static long MVP2001_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, MVP2001_cards));
}


/* start building a transaction */
static long MVP2001_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
static RTN_STATUS MVP2001_end_trans(struct motorRecord *mr)
{
    return(OK);
}


/* add a part to the transaction */
static RTN_STATUS MVP2001_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct mess_info *motor_info;
    struct MVPcontroller *cntrl;
    char buff[BUFF_SIZE], enc_cpr[10], max_mA[6], parm[31];
    char prem[3] = {'A', 'N', 'O'};
    int card, axis;
    unsigned int size;
    int sp, ac, i, j, ano;    
    RTN_STATUS rtnval;
    epicsInt32 cntrl_units = 0;
    double dval;
    bool send;

    send = true;	/* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';
    dval = *parms;

    motor_start_trans_com(mr, MVP2001_cards);
    
    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct MVPcontroller *) brdptr->DevicePrivate;
    cntrl_units = (epicsInt32) NINT(dval);
    
    if (MVP2001_table[command] > motor_call->type)
	motor_call->type = MVP2001_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
	sprintf(buff, "%d %s", axis, mr->init);
        strcpy(motor_call->message, buff);
	buff[0] = '\0';
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, MVP2001_cards);
        motor_call->type = MVP2001_table[command];
    }

    switch (command)
    {
	case MOVE_ABS:
	case MOVE_REL:
	case JOG:
	    if (strlen(mr->prem) != 0)
	    {
		if (strncmp(mr->prem, prem, 3) == 0)
		{		
		    /* 
		     * If the current limit has not already been calculated, the
		     * calculation is performed using the current limit specified
		     * in the parm string of the out field (in mA).
		     * The parm string has the form:  "ENC_CPR,MAX_CURRENT_in_mA"
		     */
	    	    if (cntrl->maxCurrent[axis - 1] == NULL)
	    	    {
	    	    	i = 0;
	    	    	strcpy(parm, mr->out.value.vmeio.parm);

		    	/* get the max current from the parm string */
		    	while (parm[i] != ',')
		    	{
		    	    i++;
		    	}
		    	i++;
		    	j = i;
		    	while (isdigit(parm[i]))
		    	{
		    	    max_mA[i-j] = parm[i];
		    	    i++;
		    	}
		    	max_mA[i-j] = '\0';
		    	cntrl->maxCurrent[axis - 1] = atoi(max_mA);
	    	    }

	    	    /* The MVP2001 manual implies that the range 0.1-2.3 Amps is ok */
	    	    if (cntrl->maxCurrent[axis - 1] < 100 || cntrl->maxCurrent[axis - 1] > 2300)
	    	    	cntrl->maxCurrent[axis - 1] = 100;
	    
	    	    /* The values in the ano calc are determined from data in the MVP manual */
	    	    ano = NINT(cntrl->maxCurrent[axis - 1] * 0.865909 + 2103.431);
	    	    sprintf(buff, "%d ANO %d", axis, ano);				
		}
		else
		{		
		    sprintf(buff, "%d %s", axis, mr->prem);
		}

        	strcpy(motor_call->message, buff);    
		buff[0] = '\0'; 	    	      
		/* end ANO command trans because MVP can't handle multiple commands */
        	rtnval = motor_end_trans_com(mr, drvtabptr);	      
        	/* begin the original transaction again (command) */
		rtnval = (RTN_STATUS) motor_start_trans_com(mr, MVP2001_cards);    
        	motor_call->type = MVP2001_table[command];	      
	    }
	    /* 
	     * The following probably will not work for the MVP2001 as the
	     * mr->post field is not ready to be sent in it's primitive form
	     */
	    if (strlen(mr->post) != 0)
		motor_call->postmsgptr = (char *) &mr->post;
	    break;

	case HOME_FOR:
	case HOME_REV:
	default:
	    break;
    }


    switch (command)
    {
	case MOVE_ABS:
	    sprintf(buff, "%d LA %d", axis, cntrl_units);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "%d LR %d", axis, cntrl_units);
	    break;
	
	case HOME_FOR:
	case HOME_REV:
	    break;
	
	case LOAD_POS:
	    sprintf(buff, "%d HO %d", axis, cntrl_units);
	    break;
	
	case SET_VEL_BASE:
	    send = false;
	    break;
	
	case SET_VELOCITY:
	    cntrl_units = NINT(dval * 0.03);
	    sprintf(buff, "%d SP %d", axis, cntrl_units);
	    break;
	
	/* 
	 * The calculation of the acceleration requires the number of 
	 * encoder counts per revolution.  This value is passed in
	 * through the parm string of the out field.  the parm string
	 * has the form:  "ENC_CPR,MAX_CURRENT_in_mA"
	 */
	case SET_ACCEL:
	    /* 
	     * If the Encoder counts per revolution has already been calculated
	     * and stored into the device private structure, then it will not
	     * be recalculated.
	     */
	    if (cntrl->encoderCpr[axis - 1] == NULL)
	    {    
		i = 0;	    	
	    	strcpy(parm, mr->out.value.vmeio.parm);

		/* get the encoder cpr from the parm string */
	    	while(isdigit(parm[i]))
	    	{
	    	    enc_cpr[i] = parm[i];
	    	    i++;
	    	}
	    	enc_cpr[i] = '\0';
	    	cntrl->encoderCpr[axis - 1] = atoi(enc_cpr);      
	    }
	    
	    sp = NINT(mr->velo / fabs(mr->mres) * 0.03);
	    ac = NINT(dval * 0.03 * cntrl->encoderCpr[axis - 1] * 0.0000625);
	    if (ac < sp)
	    {
		if (ac > 0)
		    cntrl_units = ac;
		else
	            cntrl_units = 1;
	    }
	    else
	    {
	        cntrl_units = sp;
	    }
	    sprintf(buff, "%d AC %d", axis, cntrl_units);
	    break;
	
	case GO:
	    sprintf(buff, "%d M", axis);
	    break;
	
	case PRIMITIVE:
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	    sprintf(buff, "%d AB", axis);
	    break;
	
	case JOG_VELOCITY:
	case JOG:
	    cntrl_units = NINT(dval * 0.03);
	    sprintf(buff, "%d V %d", axis, cntrl_units);
	    break;
	
	case SET_PGAIN:
	    sprintf(buff, "%d POR %ld", axis, (NINT(dval * 28000 + 4000)));
	    break;
	case SET_IGAIN:
	    sprintf(buff, "%d I %ld", axis, (NINT(dval * 31999 + 1)));
	    break;
	case SET_DGAIN:
	    sprintf(buff, "%d DER %ld", axis, (NINT(dval * 31000 + 1000)));
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "%d EN", axis);
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "%d DI", axis);
	    break;
	
	case SET_HIGH_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis - 1];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    if (cntrl_units > motor_info->high_limit)
	    {
		mr->dhlm = motor_info->high_limit * fabs(mr->mres);
		rtnval = ERROR;
	    }
	    break;

	case SET_LOW_LIMIT:
	    motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis - 1];
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */

	    if (cntrl_units < motor_info->low_limit)
	    {
		mr->dllm = motor_info->low_limit * fabs(mr->mres);
		rtnval = ERROR;
	    }
	    break;

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
	errlogMessage("MVP2001_build_trans(): buffer overflow.\n");
    else
    {
	strcat(motor_call->message, buff);
	motor_end_trans_com(mr, drvtabptr);
    }
    
    return(rtnval);
}
