/*
FILENAME...	devSoft.c
USAGE...	Motor record device level support for Soft channel.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-02-14 15:00:21 $
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
 *
 * .00 02-06-02 rls - Don't process from events unless interruptAccept is TRUE.
 *		    - When done transitions from false to true it is not
 *		      communicated to the motor record until after the last
 *		      readback update.
 *		    - In soft_process(), call dbProcess() instead of directly
 *		      calling motor record's process().
 *		    - In soft_rdbl_func(), reset motor record's target to actual
 *		      position after last readback if motion was not initiated
 *		      by this record.
 * .01 10-29-02 rls - LOCK field added to prevent synchronization due to
 *		      changing readback.
 */


/*
NOTES...
- Can't call CA functions until after dbLockInitRecords() has
    been called and initialized lock sets.   
*/


#include	<dbDefs.h>
#include	<dbFldTypes.h>
#include	<dbAccess.h>
#include	<dbEvent.h>
#include	<recSup.h>

#include	"motorRecord.h"
#include	"motor.h"
#include	"devSoft.h"

#define	STATIC	static

STATIC CALLBACK_VALUE update(struct motorRecord *);
STATIC long start(struct motorRecord *);
STATIC RTN_STATUS build(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS end(struct motorRecord *);
STATIC void soft_process(struct motorRecord *);


struct motor_dset devMotorSoft =
{
    {8, NULL, soft_init, soft_init_record, NULL},
    update,
    start,
    build,
    end
};


STATIC CALLBACK_VALUE update(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

#ifdef DMR_SOFTMOTOR_MODS
    if (ptr->load_position)
    {
	mr->rmp = ptr->new_position;
	mr->rep = ptr->new_position;
	ptr->load_position = FALSE;
    }
#endif
    if (ptr->dinp_value == SOFTMOVE || ptr->dinp_value == HARDMOVE)
	mr->msta &= ~RA_DONE;
    else
	mr->msta |= RA_DONE;
    return(ptr->callback_flag);
}


STATIC long start(struct motorRecord *mr)
{
    return((long) OK);
}


STATIC RTN_STATUS end(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    if (ptr->default_done_behavior == YES)
	mr->msta = RA_DONE;
    return(OK);
}


STATIC RTN_STATUS build(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    const short int stop = 1;
    long int status;

    switch (command)
    {
	case MOVE_ABS:
	case MOVE_REL:
	    status = dbPutLink(&mr->out, DBR_DOUBLE, &mr->dval, 1);
	    break;

	case STOP_AXIS:
	    status = dbPutLink(&mr->stoo, DBR_SHORT, &stop, 1);
	    break;

	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	    status = OK;
	    break;

	case LOAD_POS:
	    {
		struct soft_private *ptr = (struct soft_private *) mr->dpvt;
    
#ifdef DMR_SOFTMOTOR_MODS
		ptr->load_position = TRUE;
		ptr->new_position = *parms;
#endif
		mr->msta = RA_DONE;
		callbackRequest(&ptr->callback);
	    }
	    break;

	default:
	    status = ERROR;
    }
    return(status == 0 ? OK : ERROR);
}


/*
FUNCTION... void soft_dinp_func(struct motorRecord *, short)
USAGE... Update soft channel device input links and
    process soft channel motor record when done moving.
LOGIC...
    IF TRUE to FALSE transition.
	IF this soft motor's DMOV is FALSE.
	    This is a soft motor initiated move.
	    Set SOFTMOVE indicator.
	ELSE
	    This is NOT a soft motor initiated move.
	    Set HARDMOVE indicator.
	    Set soft motor's DMOV FALSE.
	    Set PP TRUE.
	ENDIF
    ENDIF    
    Get DINP_VALUE via DINP link.
    IF Get() succeeds and DINP_VALUE is true
        Process soft channel record
    ENDIF
*/
void soft_dinp_func(struct motorRecord *mr, short newdinp)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    if (interruptAccept != TRUE)
	return;
    
    /* Test for hard motor started moving or initialization. */
    if (newdinp == 0)
    {
	if (mr->dmov == FALSE)
	    ptr->dinp_value = SOFTMOVE;
	else if (mr->lock == menuYesNoNO)
	{	/* Hard motor is moving independent of soft motor. */
	    unsigned short mask = (DBE_VALUE | DBE_LOG);

	    ptr->dinp_value = HARDMOVE;
	    mr->dmov = FALSE;
	    db_post_events(mr, &mr->dmov, mask);
	    mr->pp = TRUE;
	    db_post_events(mr, &mr->pp, mask);
	}
    }
    else		/* Hard motor is done moving. */
    {
	if (ptr->dinp_value == HARDMOVE)
	    mr->pp = TRUE;
	ptr->dinp_value = DONE;
	soft_process(mr);		/* Process in case there is no readback callback. */
    }
}


void soft_rinp_func(struct motorRecord *mr, long newrinp)
{
    if (interruptAccept != TRUE)
	return;
    
    mr->rmp = newrinp;
    soft_process(mr);
}


void soft_rdbl_func(struct motorRecord *mr, double newrdbl)
{       
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    if (interruptAccept != TRUE)
	return;

    newrdbl = newrdbl / mr->mres;
    mr->rmp = NINT(newrdbl);

    if (ptr->initialized == false)
    {
	/* Reset Target to Actual position. */
	unsigned short mask = (DBE_VALUE | DBE_LOG);

	mr->dmov = FALSE;
	db_post_events(mr, &mr->dmov, mask);
	mr->pp = TRUE;
	db_post_events(mr, &mr->pp, mask);

	ptr->dinp_value = DONE;
	ptr->initialized = true;
    }    
    soft_process(mr);
}


/*
FUNCTION... STATIC void soft_process(struct motorRecord *)
USAGE...	Process the soft channel motor record.
LOGIC...
	Lock soft channel record - call dbScanLock().
	Set call back flag to CALLBACK_DATA so readback will get updated.
	Process soft channel record - call process().
	Unlock soft channel record - call dbScanUnlock().
*/

STATIC void soft_process(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    dbScanLock((struct dbCommon *) mr);
    ptr->callback_flag = CALLBACK_DATA;
    dbProcess((struct dbCommon *) mr);	/* Process the soft channel record. */
    ptr->callback_flag = NOTHING_DONE;
    dbScanUnlock((struct dbCommon *) mr);
}


/*
FUNCTION... void soft_motor_callback(CALLBACK *)
USAGE...	Process motor record after the following events:
		    - LOAD_POS motor command.
LOGIC...
*/

void soft_motor_callback(CALLBACK *cbptr)
{
#ifdef __GNUG__
    struct motorRecord *mr;

    callbackGetUser((void *) mr, cbptr);
    soft_process(mr);

#else
    void *mr;

    callbackGetUser(mr, cbptr);
    soft_process((motorRecord *) mr);
#endif
}

