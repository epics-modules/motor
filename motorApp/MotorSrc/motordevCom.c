/*
FILENAME: motordevCom.c
USAGE... This file contains device functions that are common to all motor
    record device support modules.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-02-08 22:18:44 $
*/

/*
 *      Original Author: Jim Kowalkowski
 *      Current Author: Joe Sullivan
 *      Date: 01/18/93
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
 * .01  01-18-93	jbk     initialized
 *      ...
 * .03  03-19-96	tmm     v1.10: modified encoder-ratio calculation
 * .04  11-26-96	jps     allow for bumpless-reboot on position
 * .04a 02-19-97    tmm     fixed for EPICS 3.13
 */


#include	<vxWorks.h>
#include	<stdlib.h>
#include	<string.h>
#include	<math.h>
#include	<callback.h>
#include	<fast_lock.h>
#ifdef __cplusplus
extern "C" {
#include	<dbAccess.h>
#include	<recSup.h>
}
#else
#include	<dbAccess.h>
#include	<recSup.h>
#endif
#include	<devSup.h>

#include	"motorRecord.h"
#include	"motor.h"
#include	"motordevCom.h"
#include	"motordrvCom.h"

static void motor_callback(struct mess_node * motor_return);
static void motor_init_callback(struct mess_node * motor_return);

/* Command set used by record support.  WARNING! this must match
   "motor_cmnds" in motor.h .
*/

long motor_init_com(int after, int brdcnt, struct driver_table *tabptr,
	      struct board_stat ***sptr)	/* Initialize motor record device support. */
{
    MOTOR_CARD_QUERY card_query;
    int card, motor;

    if (after == 0)
    {
	/* allocate space for maximum possible cards in system */
	*sptr = (struct board_stat **) malloc(brdcnt * sizeof(struct board_stat *));

	/* for each card ask driver for num of axis supported and name */
	for (card = 0; card < brdcnt; card++)
	{
	    struct board_stat *brdptr;

	    brdptr = (*sptr)[card] = (struct board_stat *) malloc(brdcnt * sizeof(struct board_stat));
	    (tabptr->get_card_info) (card, &card_query, tabptr);
	    if (card_query.total_axis == 0)
		brdptr->exists = NO;
	    else
	    {
		brdptr->exists = YES;
		brdptr->name = card_query.card_name;
		brdptr->total_axis = card_query.total_axis;
		brdptr->axis_stat = (struct axis_stat *) malloc(
				 card_query.total_axis * sizeof(struct axis_stat));
    
		for (motor = 0; motor < card_query.total_axis; motor++)
		{
		   brdptr->axis_stat[motor].in_use = OFF;
		   brdptr->axis_stat[motor].name = card_query.axis_names[motor];
		}
	    }
	}
    }
    return (0);
}


/*
FUNCTION... motor_init_record_com - initialize a record instance.

SYNOPSIS... motor_init_record_com(	mr	- motor record pointer,
					brdcnt	- controller board count,
					tabptr	- driver table pointer,
					sptr	- controller board status pointer)
LOGIC...
    Allocate memory for motor command transaction data structure (struct motor_trans)
	and point the Device Private record field to it.
    Initialize the motor command transaction data structure.
    ...
    ...
    ...
    Error check "card" index, "signal" index and motor "in_use" indicator.
    IF error detected.
	Set the MSTA PROBLEM bit ON.
	Set RES <- MRES.
	Set RMP <- REP <- 0.
	ERROR RETURN.
    ENDIF
    
    Get motor information - call get_axis_info() via driver table.
    Update MSTA.
    Set axis assigned to a motor record indicator ON.

    Set Initialize encoder indicator based on (MSTA indicates an encoder is
	present, AND, UEIP set to YES).

    IF Initialize encoder indicator is ON.
	...
	...
    ELSE
	Set local encoder ratio to unity.
	Set RES <- MRES.
    ENDIF

    Set Initialize position indicator based on (DVEL != 0, AND, RES != 0,
	AND, the above "get_axis_info()" position == 0) [NOTE: non-zero controller
	position takes precedence over autorestore position].
    Set Command Primitive Initialization string indicator based on (non-NULL "init"
	pointer, AND, non-zero string length.

    IF (Initialize position, OR, encoder, OR, string indicators are ON)
	Create initialization semaphore.
	...
	...
	IF Initialize encoder indicator is ON.
	    Send Set Encoder Ratio command to controller.
	ENDIF
	IF Initialize position indicator is ON.
	    Send Load Position command to controller.
	ENDIF
	IF Command Primitive Initialization string present.
	    Send Initialization string to controller.
	ENDIF
	
	Send Get Info command to controller.
	...
	...
    ENDIF

    Get motor information - call get_axis_info() via driver table.
    Set RMP, REP and MSTA based on updated motor information.
    NORMAL RETURN.
*/

long motor_init_record_com(struct motorRecord *mr, int brdcnt, struct driver_table *tabptr,
	      struct board_stat *sptr[])
{
    struct motor_dset *pdset = (struct motor_dset *) (mr->dset);
    struct board_stat *brdptr;
    int card, signal;
    BOOLEAN initEncoder, initPos, initString;
    struct motor_trans *ptrans;
    MOTOR_AXIS_QUERY axis_query;
    struct mess_node *motor_call;
    double ep_mp[2];		/* encoder pulses, motor pulses */
    int rtnStat;

    /* allocate space for private field - an motor_trans structure */
    mr->dpvt = (struct motor_trans *) malloc(sizeof(struct motor_trans));

    /* Initialize the private field. */
    ptrans = (struct motor_trans *) mr->dpvt;
    ptrans->state = IDLE_STATE;
    ptrans->callback_changed = NO;
    ptrans->tabptr = tabptr;

    FASTLOCKINIT(&ptrans->lock);
    motor_call = &(ptrans->motor_call);

    callbackSetCallback((void (*)(struct callbackPvt *)) motor_callback,
			&(motor_call->callback));
    callbackSetPriority(priorityMedium, &(motor_call->callback));

    /* check if axis already in use, then mark card, axis as in use */
    if (mr->out.type != VME_IO)	/* out must be VME_IO. */
    {
	recGblRecordError(S_dev_badBus, (void *) mr,
	    (char *) "motor_init_record_com(): Illegal OUT Bus Type");
	return(S_dev_badBus);
    }

    card = mr->out.value.vmeio.card;
    brdptr = sptr[card];
    signal = mr->out.value.vmeio.signal;
    rtnStat = 0;

    if (card < 0 || card >= brdcnt || brdptr->exists == NO)
    {
	recGblRecordError(S_db_badField, (void *) mr,
	    (char *) "motor_init_record_com(): card does not exist!");
	rtnStat = S_db_badField;
    }
    else if (signal < 0 || signal >= brdptr->total_axis)
    {
	recGblRecordError(S_db_badField, (void *) mr,
	    (char *) "devOMS (init_record_com) signal does not exist!");
	rtnStat = S_db_badField;
    }
    else if (brdptr->axis_stat[signal].in_use == YES)
    {
	recGblRecordError(S_db_badField, (void *) mr,
	    (char *) "devOmsCom (init_record_com) motor already in use!");
	rtnStat = S_db_badField;
    }

    if (rtnStat)
    {
	/* Initialize readback fields for simulation */
	mr->msta = RA_PROBLEM;
	mr->res = mr->mres;
	mr->rmp = 0;		/* raw motor pulse count */
	mr->rep = 0;		/* raw encoder pulse count */
	return(rtnStat);
    }
	
    /* query motor for all info to fill into record */
    (tabptr->get_axis_info) (card, signal, &axis_query, tabptr);
    mr->msta = axis_query.status;	/* status info */
    brdptr->axis_stat[signal].in_use = ON;

/*jps: setting the encoder ratio was moved from init_record() remove  callbacks during iocInit */
    /*
     * Set the encoder ratio.  Note this is blatantly device dependent.
     * Determine the number of encoder pulses and the number of motor pulses
     * per engineering unit (EGU).  Send an array containing this information
     * to device support.
     */
    initEncoder = ((mr->msta & EA_PRESENT) && mr->ueip) ? ON : OFF;
    if (initEncoder == ON)
    {
	if (fabs(mr->mres) < 1.e-9)
	    mr->mres = 1.;
	if (fabs(mr->eres) < 1.e-9)
	    mr->eres = mr->mres;
	{
	    int m;
	    for (m = 10000000; (m > 1) && (fabs(m / mr->eres) > 1.e6 ||
			fabs(m / mr->mres) > 1.e6); m /= 10);
	    ep_mp[0] = m / mr->eres;	/* encoder pulses per ... */
	    ep_mp[1] = m / mr->mres;	/* motor pulses */
	}
	mr->res = mr->eres;
    }
    else
    {
	ep_mp[0] = 1.;
	ep_mp[1] = 1.;
	mr->res = mr->mres;
    }

    initPos = (mr->dval != 0 && mr->res != 0 && axis_query.position == 0) ? ON : OFF;
    /* Test for command primitive initialization string. */
    initString = (mr->init != NULL && strlen(mr->init)) ? ON : OFF;
    
    /* Program the device if an encoder is present */
    if (initPos == ON || initEncoder == ON || initString == ON)
    {
	/* Semaphore used to hold initialization until device is programmed - cleared by callback. */
	ptrans->initSem = semBCreate(SEM_Q_FIFO, SEM_EMPTY);

	/* Switch to special init callback so that record will not be processed during iocInit. */
	callbackSetCallback((void (*)(struct callbackPvt *)) motor_init_callback,
			    &(motor_call->callback));
	if (initEncoder == ON)
	{
	    (*pdset->start_trans)(mr);
	    (*pdset->build_trans)(SET_ENC_RATIO, ep_mp, mr);
	    (*pdset->end_trans)(mr);
	}

	if (initPos == ON)
	{
	    double setPos = mr->dval / mr->res;

	    (*pdset->start_trans)(mr);
	    (*pdset->build_trans)(LOAD_POS, &setPos, mr);
	    (*pdset->end_trans)(mr);
	}

	if (initString == ON)
	{
	    (*pdset->start_trans)(mr);
	    (*pdset->build_trans)(PRIMITIVE, NULL, mr);
	    (*pdset->end_trans)(mr);
	}

	/* Changing encoder ratio may have changed the readback value. */
	(*pdset->start_trans)(mr);
	(*pdset->build_trans)(GET_INFO, NULL, mr);
	(*pdset->end_trans)(mr);

	/* Wait for callback w/timeout */
	if ((rtnStat = semTake(ptrans->initSem, SEM_TIMEOUT)) == ERROR)
	    recGblRecordError(S_dev_NoInit, (void *) mr,
		(char *) "dev_NoInit (init_record_com: callback2 timeout");
	semDelete(ptrans->initSem);

	/* Restore regular record callback */
	callbackSetCallback((void (*)(struct callbackPvt *)) motor_callback,
			    &(motor_call->callback));
    }

    /* query motor for all info to fill into record */

    (tabptr->get_axis_info) (card, signal, &axis_query, tabptr);

    mr->rmp = axis_query.position;	/* raw motor pulse count */
    mr->rep = axis_query.encoder_position;	/* raw encoder pulse count */
    mr->msta = axis_query.status;	/* status info */
    return(OK);
}

/*
FUNCTION... long motor_update_values(struct motorRecord *)
USAGE... Update the following motor record fields with the latest driver data:
	RMP  - Raw Motor Position.
	REP  - Raw Encoder Position.
	RVEL - Raw Velocity.
	MSTA - Motor Status.
NOTES... This function MUST BE reentrant.
*/

long motor_update_values(struct motorRecord * mr)
{
    struct motor_trans *ptrans;
    long rc;

    rc = NOTHING_DONE;
    ptrans = (struct motor_trans *) mr->dpvt;

    FASTLOCK(&ptrans->lock);

    /* raw motor pulse count */
    if (ptrans->callback_changed == YES)
    {
	mr->rmp = ptrans->motor_pos;
	mr->rep = ptrans->encoder_pos;
	mr->rvel = ptrans->vel;
	mr->msta = ptrans->status;
	ptrans->callback_changed = NO;
	rc = CALLBACK_DATA;
    }
    FASTUNLOCK(&ptrans->lock);
    return (rc);
}


/*
FUNCTION... long motor_start_trans_com(struct motorRecord *, struct board_stat **, const char *)
USAGE... Start building a transaction.
NOTES... This function MUST BE reentrant.
*/

long motor_start_trans_com(struct motorRecord *mr, struct board_stat **sptr,
			 const char *init_str)
{
    struct board_stat *brdptr;
    int card = mr->out.value.vmeio.card;
    int axis = mr->out.value.vmeio.signal;
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;

    if (!mr->dpvt)
	return (S_dev_NoInit);

    motor_call = &(trans->motor_call);

    /* initialize area to device private field to store command that is to be
     * built and mark as command build in progress. */

    trans->state = BUILD_STATE;
    motor_call->card = card;
    motor_call->signal = axis;
    motor_call->type = UNDEFINED;
    motor_call->mrecord = (struct dbCommon *) mr;
    motor_call->message[0] = (char) NULL;
    motor_call->postmsgptr = (char) NULL;
    motor_call->termstring = (char) NULL;

    if (init_str != NULL)
	strcat(motor_call->message, init_str);

    /* Check for card/signal before building command - allow callback to
     * continue if no card/signal for simulation mode. */
    brdptr = sptr[card];
    if (brdptr->exists == YES && axis < brdptr->total_axis)
	motor_call->message[1] = brdptr->axis_stat[axis].name;
    else
	motor_call->message[1] = '*';
    return (0);
}


/*
FUNCTION... long motor_end_trans_com(struct motorRecord *, struct driver_table *, char *)
USAGE... Finish building a transaction.
NOTES... This function MUST BE reentrant.
*/

long motor_end_trans_com(struct motorRecord *mr, struct driver_table *tabptr,
			 char *cmnd_line_terminator)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    long rc;

    rc = OK;
    motor_call = &(trans->motor_call);
    if ((*trans->tabptr->card_array)[motor_call->card] == NULL)
	return(rc = ERROR);

    switch (trans->state)
    {
	case BUILD_STATE:
	    /* shut off command build in process thing */
	    trans->state = IDLE_STATE;
	    rc = (*tabptr->send) (motor_call, tabptr, cmnd_line_terminator);
	    break;

	case IDLE_STATE:
	default:
	    rc = ERROR;
    }
    return (rc);
}


/*
FUNCTION... static void motor_callback(struct mess_node * motor_return)
USAGE... Callback from  driver for a motor in motion.
NOTES... This function MUST BE reentrant.
*/

static void motor_callback(struct mess_node * motor_return)
{
    struct motorRecord *mr = (struct motorRecord *) motor_return->mrecord;
    struct rset *prset = (struct rset *) (motor_return->mrecord->rset);
    struct motor_trans *ptrans;

    ptrans = (struct motor_trans *) mr->dpvt;
    FASTLOCK(&ptrans->lock);

    ptrans->callback_changed = YES;
    ptrans->motor_pos = motor_return->position;
    ptrans->encoder_pos = motor_return->encoder_position;
    ptrans->vel = motor_return->velocity;
    ptrans->status = motor_return->status;

    FASTUNLOCK(&ptrans->lock);

    /* free the return data buffer */
    (ptrans->tabptr->free) (motor_return, ptrans->tabptr);

    dbScanLock((struct dbCommon *) mr);
    (*prset->process) ((struct dbCommon *) mr);
    dbScanUnlock((struct dbCommon *) mr);
    return;
}

/* callback from OMS driver for init_record  - duplicate of
 * the motor_callback without the call to process the record */
static void motor_init_callback(struct mess_node * motor_return)
{
    struct motorRecord *mr = (struct motorRecord *) motor_return->mrecord;
    struct motor_trans *ptrans;

    ptrans = (struct motor_trans *) mr->dpvt;
    FASTLOCK(&ptrans->lock);

    ptrans->callback_changed = YES;
    ptrans->motor_pos = motor_return->position;
    ptrans->encoder_pos = motor_return->encoder_position;
    ptrans->vel = motor_return->velocity;
    ptrans->status = motor_return->status;

    /* free the return data buffer */
    (ptrans->tabptr->free) (motor_return, ptrans->tabptr);
    semGive(ptrans->initSem);

    FASTUNLOCK(&ptrans->lock);
    return;
}

