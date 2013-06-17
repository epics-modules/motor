/*
FILENAME: motordrvComCode.h
USAGE... This file contains local variables that are allocated
	in each motor record driver.  The variables are allocated
	in each driver by including this file.

Version:	$Revision$
Modified By:	$Author$
Last Modified:	$Date$
HeadURL:        $URL$
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

#include <epicsTypes.h>
#include <epicsEvent.h>

/* --- Local data common to each driver. --- */
static struct controller **motor_state;
static int total_cards;
static int any_motor_in_motion;
static struct circ_queue mess_queue;	/* in message queue head */
static epicsEvent queue_lock(epicsEventFull);
static struct circ_queue free_list;
static epicsEvent freelist_lock(epicsEventFull);
static epicsEvent motor_sem(epicsEventEmpty);
static bool initialized = false;	/* Driver initialized indicator. */

#endif	/* INCmotordrvComCode */
