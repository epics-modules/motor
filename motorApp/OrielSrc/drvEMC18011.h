/*
FILENAME...	drvEMC18011.h
USAGE...    This file contains Parker Compumotor driver "include"
	    information that is specific to the 6K series serial controller

Version:	$Revision: 1.2 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2006-09-07 21:19:43 $
*/

/*
 *      Original Author: Mark Rivers
 *      Current Author: J. Sullivan
 *      Date: 10/16/97
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
 * .01  09-01-06 jps initialized from drvPC6K.h
 */

#ifndef	INCdrvEMC18011h
#define	INCdrvEMC18011h 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define EMC18011_MAX_MOTORS  3
#define EMC18011_MSG_SIZE 120
#define EMC18011_STATUS_RETRY 10

/* End-of-string defines */
#define EMC18011_OUT_EOS   "\r" /* Command */
#define EMC18011_IN_EOS    "\n"  /* Reply */

/* Although the positition resolution is fixed at 0.1um the
 * velocity resolution is 0.01um/s if programmed less then 5.0 um/s */
#define EMC18011_RESOLUTION   0.01

/* Responses to 'Z' report status command - used by set_status() */
#define Z_STOPPED    'a'
#define Z_RUNDOWN    'b'      /* Pos motion */
#define Z_RUNUP      'c'      /* Neg. motion */
#define Z_LSDOWN     'd'      /* Neg. limit switch (hard stop) */
#define Z_LSUP       'e'      /* Pos  limit switch (hard stop) */


/* Motion Master specific data is stored in this structure. */
struct EMC18011Controller
{
    asynUser *pasynUser;  	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    char asyn_port[80];     	/* asyn port name */
    char recv_string[2][EMC18011_MSG_SIZE]; /* Position Query result strings */
    int motorSelect;            /* Keep track of currently selected motor - this   */
				/* is the only one that can get status information */
    double drive_resolution;    /* This controller has a fixed drive resolution */
    CommStatus status;		/* Controller communication status. */
};


#endif	/* INCdrvEMC18011h */

