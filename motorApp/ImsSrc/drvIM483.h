/*
FILENAME...	drvIM483.h
USAGE... This file contains driver "include" information that is specific to
	Intelligent Motion Systems, Inc. IM483(I/IE).

Version:	$Revision: 1.6 $
Modified By:	$Author: rivers $
Last Modified:	$Date: 2004-08-17 21:28:30 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 02/10/2000
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
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 *
 *
 * Modification Log:
 * -----------------
 * .01	02/10/2000 rls copied from drvMM4000.h
 * .02  07/01/2004 rls Converted from MPF to asyn.
 */

#ifndef	INCdrvIM483h
#define	INCdrvIM483h 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2 /* Timeout in seconds */


/* IM483 specific data is stored in this structure. */
struct IM483controller
{
    asynUser *pasynUser;  	/* For RS-232 */
    char asyn_port[80];     	/* asyn port name */
    CommStatus status;		/* Controller communication status. */
};

/* Function prototypes. */
extern RTN_STATUS IM483SMSetup(int, int);
extern RTN_STATUS IM483PLSetup(int, int);
extern RTN_STATUS  MDriveSetup(int, int);
extern RTN_STATUS IM483SMConfig(int, const char *);
extern RTN_STATUS IM483PLConfig(int, const char *);
extern RTN_STATUS  MDriveConfig(int, const char *);

#endif	/* INCdrvIM483h */

