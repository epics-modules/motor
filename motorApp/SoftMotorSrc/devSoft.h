/*
FILENAME..	devSoft.h

USAGE... 	This file contains information that is common to
		all Soft channel device support modules.

Version:	$Revision: 1.9 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-06-16 15:03:12 $
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
 */

#ifndef	INCdevSofth
#define	INCdevSofth 1

#include <callback.h>

typedef enum DONE_STATES {SOFTMOVE = 0, HARDMOVE = 1, DONE = 2} DONE_STATES;

#define MAXMSGS 20

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

extern long soft_init(void *);
extern long soft_init_record(void *);
extern void soft_dinp_func(struct motorRecord *, short);
extern void soft_rdbl_func(struct motorRecord *, double);
extern void soft_rinp_func(struct motorRecord *, long);
extern void soft_motor_callback(CALLBACK *);

#endif	/* INCdevSofth */
