/*
FILENAME..	devSoft.h

USAGE... 	This file contains information that is common to
		all Soft channel device support modules.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2001-05-14 20:13:48 $
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
 */

#ifndef	INCdevSofth
#define	INCdevSofth 1

extern long soft_init(int);
extern long soft_init_record(struct motorRecord *);
extern void soft_dinp_func(struct motorRecord *, short);
extern void soft_rdbl_func(struct motorRecord *, double);
extern void soft_rinp_func(struct motorRecord *, long);
extern void soft_motor_callback(CALLBACK *);

#define MAXMSGS 20

struct soft_private
{
    CALLBACK callback;
    long callback_flag;
#ifdef DMR_SOFTMOTOR_MODS
    BOOLEAN load_position;
    long new_position;
#endif
    short dinp_value;			/* Value from DINP link. */
    BOOLEAN default_done_behavior;	/* If the DINP is not programmed, exhibit
					 * "immediate done" default behavior. */
};

#endif	/* INCdevSofth */
