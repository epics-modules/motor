/*
FILENAME...	drvSC800.h
USAGE...    This file contains Kohzu SC800 motorRecord driver information.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2007-11-27 18:01:17 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Current Author:  Ron Sluiter
 *      Date: 11/08/07
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
 * .01 11-08-07 rls copied from drvMDT695.h
 */

#ifndef	INCdrvSC800h
#define	INCdrvSC800h 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define SC800_MAX_MOTORS  8
#define SC800_MSG_SIZE 80
#define SC800_STATUS_RETRY 10

/* End-of-string defines */
#define SC800_OUT_EOS   "\r\n" /* Command */
#define SC800_IN_EOS    "\r\n"  /* Reply */

/* Motion Master specific data is stored in this structure. */
struct SC800Controller
{
    asynUser *pasynUser;  	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    char asyn_port[80];     	/* asyn port name */
    char recv_string[SC800_MSG_SIZE]; /* Position Query result strings */
    CommStatus status;		/* Controller communication status. */
    int base_speed[SC800_MAX_MOTORS]; /* steps/sec. */
    int slew_speed[SC800_MAX_MOTORS]; /* steps/sec. */
    int accl_rate[SC800_MAX_MOTORS];  /* steps/(sec^2) */
};


#endif	/* INCdrvSC800h */

