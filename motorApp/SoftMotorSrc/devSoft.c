/*
FILENAME...	devSoft.c
USAGE...	Motor record device level support for Soft channel.

Version:	$Revision: 1.2 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-03-03 22:35:49 $
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


/*
NOTES...
- Can't call CA functions until after dbLockInitRecords() has
    been called and initialized lock sets.   
*/


#include	<vxWorks.h>
#include	<dbDefs.h>
#include	<dbFldTypes.h>
#ifdef __cplusplus
extern "C" {
#include	<dbEvent.h>
#include	<recSup.h>
}
#else
#include	<dbEvent.h>
#include	<recSup.h>
#endif

#include	"motorRecord.h"
#include	"motor.h"
#include	"devSoft.h"

#define	STATIC	static

STATIC long update(struct motorRecord *);
STATIC long start(struct motorRecord *);
STATIC long build(motor_cmnd, double *, struct motorRecord *);
STATIC long end(struct motorRecord *);
STATIC void soft_process(struct motorRecord *);


struct motor_dset devMotorSoft =
{
    {8, NULL, soft_init, soft_init_record, NULL},
    update,
    start,
    build,
    end
};


STATIC long update(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    mr->msta = ptr->dinp_value ? RA_DONE : 0;
    return(ptr->callback_flag);
}


STATIC long start(struct motorRecord *mr)
{
    return((long) OK);
}


STATIC long end(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    if (ptr->default_done_behavior == YES)
	mr->msta = RA_DONE;
    return((long) OK);
}


STATIC long build(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    const short int stop = 1;
    long status;

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
    
		mr->msta = RA_DONE;
		callbackRequest(&ptr->callback);
	    }
	    break;

	default:
	    status = ERROR;
    }
    return(status);
}


/*
FUNCTION... void soft_dinp_func(struct motorRecord *, short)
USAGE... Update soft channel device input links and
    process soft channel motor record when done moving.
LOGIC...
    Get DINP_VALUE via DINP link.
    IF Get() succeeds and DINP_VALUE is true
        Process soft channel record
    ENDIF
*/
void soft_dinp_func(struct motorRecord *mr, short newdinp)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    ptr->dinp_value = newdinp;
    if (ptr->dinp_value)
	soft_process(mr);
}


void soft_rinp_func(struct motorRecord *mr, long newrinp)
{
    mr->rmp = newrinp;
    soft_process(mr);
}


void soft_rdbl_func(struct motorRecord *mr, double newrdbl)
{       
    newrdbl = newrdbl / mr->res;
    if ((mr->msta & EA_PRESENT) && mr->ueip)
	mr->rep = NINT(newrdbl);
    else
	mr->rmp	= NINT(newrdbl);
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
    struct rset *prset = (struct rset *) (mr->rset);

    dbScanLock((struct dbCommon *) mr);
    ptr->callback_flag = CALLBACK_DATA;
    (*prset->process) (mr);	/* Process the soft channel record. */
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
    struct motorRecord *mr;

    callbackGetUser(mr, cbptr);
    soft_process(mr);
}

