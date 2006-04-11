/*
FILENAME..	devSoft.h

USAGE... 	This file contains information that is common to
		all Soft channel device support modules.

Version:	$Revision: 1.11 $
Modified By:	$Author: peterd $
Last Modified:	$Date: 2006-04-11 10:11:24 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 03/25/99
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
 * .01 06-16-03 rls Converted to R3.14.x.
 * .02 09-23-04 rls Increase the maximum number of Soft Channel motor records
 *                  from 20 to 50.
 * .03 2006-04-10 pnd Convert to linked lists to remove arbitrary maximum
 */

#ifndef	INCdevSofth
#define	INCdevSofth 1

#include <callback.h>
#include <ellLib.h>

typedef enum DONE_STATES {SOFTMOVE = 0, HARDMOVE = 1, DONE = 2} DONE_STATES;

struct soft_private
{
    CALLBACK callback;
    CALLBACK_VALUE callback_flag;
    DONE_STATES dinp_value;		/* Value from DINP link. */
    bool default_done_behavior;		/* If the DINP is not programmed, exhibit
					 * "immediate done" default behavior. */
    bool initialized;			/* 1st RDBL call after interruptAccept is TRUE
					 * sets this ON. */
};

struct motor_node {
    ELLNODE	node;
    struct motorRecord *pmr;
};

extern long soft_init(void *);
extern long soft_init_record(void *);
extern void soft_dinp_func(struct motorRecord *, short);
extern void soft_rdbl_func(struct motorRecord *, double);
extern void soft_rinp_func(struct motorRecord *, long);
extern void soft_motor_callback(CALLBACK *);

#endif	/* INCdevSofth */
