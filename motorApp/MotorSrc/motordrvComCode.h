/*
FILENAME: motordrvComCode.h
USAGE... This file contains local variables that are allocated
	in each motor record driver.  The variables are allocated
	in each driver by including this file.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-02-08 22:18:45 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 08/20/99
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

#ifndef	INCmotordrvComCode
#define	INCmotordrvComCode 1

/* --- Local data common to each driver. --- */
STATIC volatile int motor_scan_rate = SCAN_RATE;
STATIC struct controller **motor_state;
STATIC int total_cards;
STATIC int any_motor_in_motion;
STATIC struct circ_queue mess_queue;	/* in message queue head */
STATIC FAST_LOCK queue_lock;
STATIC struct circ_queue free_list;
STATIC FAST_LOCK freelist_lock;
STATIC SEM_ID motor_sem;
STATIC BOOLEAN initialized = OFF;	/* Driver initialized indicator. */

#endif	/* INCmotordrvComCode */
