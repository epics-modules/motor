/*
FILENAME...	devSoftAux.c
USAGE...	Motor record device level support for Soft channel.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-02-08 22:19:26 $
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
Notes:
- A Channel Access (CA) oriented file had to be created separate from the
primary devSoft.c file because CA and record access code cannot both reside 
in the same file; each defines (redefines) the DBR's.
*/


#include	<vxWorks.h>
#include	<taskLib.h>
#include	<msgQLib.h>
#include	<cadef.h>

#include	"motorRecord.h"

typedef enum BOOLEAN_VALUES {OFF = 0, ON = 1} BOOLEAN;
#include	"devSoft.h"

#define	STATIC	static

STATIC void soft_dinp(struct event_handler_args);
STATIC void soft_rdbl(struct event_handler_args);
STATIC void soft_rinp(struct event_handler_args);
STATIC int soft_motor_task(int, int, int, int, int, int, int, int, int, int);
STATIC SEM_ID soft_motor_sem;
STATIC MSG_Q_ID soft_motor_msgQ;

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

long soft_init(int after)
{
    if (after == 0)
    {
	soft_motor_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
	soft_motor_msgQ = msgQCreate(MAXMSGS, 4, MSG_Q_FIFO);
	taskSpawn((char *) "soft_motor", 64, VX_FP_TASK | VX_STDIO, 5000,
		  soft_motor_task, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
    else
	semGive(soft_motor_sem);	/* Start soft_motor_task(). */
    return((long) OK);
}


long soft_init_record(struct motorRecord *mr)
{
    struct soft_private *ptr;
    STATUS status;
    static int count = 0;

    if (++count > MAXMSGS)
	return(ERROR);
    status = msgQSend(soft_motor_msgQ, (char *) &mr, 4, NO_WAIT,
		      MSG_PRI_NORMAL);

    /* Allocate space for private field. */
    mr->dpvt = (struct soft_private *) malloc(sizeof(struct soft_private));
    ptr = (struct soft_private *) mr->dpvt;
    ptr->dinp_value = mr->dmov;	/* Must match after initialzation. */

    return ((long) status);
}


STATIC int soft_motor_task(int a1, int a2, int a3, int a4, int a5, int a6,
			   int a7, int a8, int a9, int a10)
{
    struct motorRecord *mr;
    chid dinp, rdbl, rinp;

    semTake(soft_motor_sem, WAIT_FOREVER);	/* Wait for dbLockInitRecords() to execute. */
    SEVCHK(ca_task_initialize(),"ca_task_initialize");

    while (msgQReceive(soft_motor_msgQ, (char *) &mr, 4, NO_WAIT) != ERROR)
    {
	if (mr->dinp.value.constantStr == NULL)
	{
	    struct soft_private *ptr = (struct soft_private *) mr->dpvt;
	    ptr->default_done_behavior = ON;
	}
	else
	{
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

    msgQDelete(soft_motor_msgQ);
    taskSuspend(NULL);		/* Wait Forever. */
    return(ERROR);
}

