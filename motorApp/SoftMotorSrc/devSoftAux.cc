/*
FILENAME...	devSoftAux.cc
USAGE...	Motor record device level support for Soft channel.

Version:	$Revision: 1.9 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2005-03-30 19:18:16 $
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
 * .01 06-16-03 rls Convert to R3.14.x.
 * .02 12-14-04 rls With EPICS R3.14.7 changes to epicsThread.h, need explicit
 *		    #include <stdlib.h>
 */


/*
Notes:
- A Channel Access (CA) oriented file had to be created separate from the
primary devSoft.c file because CA and record access code cannot both reside 
in the same file; each defines (redefines) the DBR's.
*/

#include <stdlib.h>
#include <cadef.h>
#include <errlog.h>
#include <epicsEvent.h>
#include <epicsRingPointer.h>
#include <callback.h>
#include <epicsThread.h>

#include "motorRecord.h"
#include "motor.h"
#include "devSoft.h"

#define	STATIC	static

STATIC void soft_dinp(struct event_handler_args);
STATIC void soft_rdbl(struct event_handler_args);
STATIC void soft_rinp(struct event_handler_args);
STATIC EPICSTHREADFUNC soft_motor_task(void *);
STATIC epicsThreadId soft_motor_id;
STATIC epicsEventId soft_motor_sem;
STATIC epicsRingPointerId soft_motor_msgQ;

STATIC void soft_dinp(struct event_handler_args args)
{
    soft_dinp_func((struct motorRecord *) args.usr, *((short *) args.dbr));
}


STATIC void soft_rdbl(struct event_handler_args args)
{
    soft_rdbl_func((struct motorRecord *) args.usr, *((double *) args.dbr));
}

STATIC void soft_rinp(struct event_handler_args args)
{
    soft_rinp_func((struct motorRecord *) args.usr, *((long *) args.dbr));
}

long soft_init(void *after)
{
    int before_after = (int) after;
    if (before_after == 0)
    {
	epicsThreadId dbCaTask_tid;
	unsigned int soft_motor_priority;
	int retry = 0;

        soft_motor_sem = epicsEventCreate(epicsEventEmpty);
	soft_motor_msgQ = epicsRingPointerCreate(MAXMSGS);

	/* 
	 * Fix for DMOV processing before the last DRBV update; i.e., lower
	 * the priority of the "soft_motor" task below the priority of the
	 * "dbCaLink" task.
	 */
	while((dbCaTask_tid = epicsThreadGetId("dbCaLink")) == 0 && retry < 10)
	{
	    epicsThreadSleep(0.1);
	    retry++;
	}

	if (dbCaTask_tid == 0)
	{
	    errMessage(0, "cannot find dbCaLink task.");
	    return(ERROR);
	}
	soft_motor_priority = epicsThreadGetPriority(dbCaTask_tid);
	soft_motor_priority -= 1;

	soft_motor_id = epicsThreadCreate((char *) "soft_motor", soft_motor_priority, 
                                          epicsThreadGetStackSize(epicsThreadStackBig),
					  (EPICSTHREADFUNC) soft_motor_task, NULL);
    }
    else
	epicsEventSignal(soft_motor_sem);	/* Start soft_motor_task(). */
    return(OK);
}


long soft_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    struct soft_private *ptr;
    CALLBACK *cbptr;
    int status = 0;
    static int count = 0;

    if (++count > MAXMSGS)
	return(ERROR);

    if (!epicsRingPointerPush(soft_motor_msgQ, (void *) mr))
        status = ERROR;

    /* Allocate space for private field. */
    mr->dpvt = (struct soft_private *) malloc(sizeof(struct soft_private));
    ptr = (struct soft_private *) mr->dpvt;
    ptr->dinp_value = (mr->dmov == 0) ? SOFTMOVE : DONE; /* Must match after initialzation. */
    ptr->initialized = false;

    cbptr = &ptr->callback;
    callbackSetCallback((void (*)(struct callbackPvt *)) soft_motor_callback,
			cbptr);
    callbackSetPriority(priorityMedium, cbptr);
    callbackSetUser(mr, cbptr);
    return ((long) status);
}


STATIC EPICSTHREADFUNC soft_motor_task(void *parm)
{
    struct motorRecord *mr;
    chid dinp, rdbl, rinp;

    epicsEventWait(soft_motor_sem);	/* Wait for dbLockInitRecords() to execute. */
    SEVCHK(ca_context_create(ca_enable_preemptive_callback),
	   "soft_motor_task: ca_context_create() error");

    while ((mr = (struct motorRecord *) epicsRingPointerPop(soft_motor_msgQ)))
    {
	struct soft_private *ptr = (struct soft_private *) mr->dpvt;
	if (mr->dinp.value.constantStr == NULL)
	{
	    ptr->default_done_behavior = true;
	}
	else
	{
	    ptr->default_done_behavior = false;
	    SEVCHK(ca_search(mr->dinp.value.pv_link.pvname, &dinp),
		   "ca_search() failure");
	    SEVCHK(ca_add_event(DBR_SHORT, dinp, soft_dinp, mr, NULL),"ca_add_event() failure");
	    SEVCHK(ca_pend_io((float) 5.0), "DINP link failure");
	}
    
	if (mr->urip != 0)
	{
	    SEVCHK(ca_search(mr->rdbl.value.pv_link.pvname, &rdbl),"ca_search() failure");
	    SEVCHK(ca_add_event(DBR_DOUBLE, rdbl, soft_rdbl, mr, NULL),"ca_add_event() failure");
	    SEVCHK(ca_pend_io((float) 5.0), "RDBL link failure");
	}

	if (mr->rinp.value.constantStr != NULL)
	{
	    SEVCHK(ca_search(mr->rinp.value.pv_link.pvname, &rinp),"ca_search() failure");
	    SEVCHK(ca_add_event(DBR_LONG, rinp, soft_rinp, mr, NULL),"ca_add_event() failure");
	    SEVCHK(ca_pend_io((float) 5.0), "RINP link failure");
	}
    }

    epicsRingPointerDelete(soft_motor_msgQ);
    epicsThreadSuspendSelf();		/* Wait Forever. */
    return(NULL);
}

