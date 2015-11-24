/*
FILENAME...	drvMDT695.h
USAGE...    This file contains ThorLabs Piezo Control Module (MDT695)
	    motorRecord driver information . 

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
 * .01  09-27-06 jps initialized from drvEMC18011.h
 */

#ifndef	INCdrvMDT695h
#define	INCdrvMDT695h 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define MDT695_MAX_MOTORS  3
#define MDT695_MSG_SIZE 120
#define MDT695_STATUS_RETRY 10

/* End-of-string defines */
#define MDT695_OUT_EOS   "\r" /* Command */
#define MDT695_IN_EOS    "\r"  /* Reply */


/* Positition resolution is fixed at 0.1 volts */
#define MDT695_RESOLUTION   0.1

#
/* Motion Master specific data is stored in this structure. */
struct MDT695Controller
{
    asynUser *pasynUser;  	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    char asyn_port[80];     	/* asyn port name */
    char recv_string[MDT695_MSG_SIZE]; /* Position Query result strings */
    double drive_resolution;    /* This controller has a fixed drive resolution */
    CommStatus status;		/* Controller communication status. */
};


#endif	/* INCdrvMDT695h */

