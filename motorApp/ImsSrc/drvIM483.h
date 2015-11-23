/*
FILENAME...	drvIM483.h
USAGE... This file contains driver "include" information that is specific to
	Intelligent Motion Systems, Inc. IM483(I/IE) and MDrive controllers.

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
 * .03  03/18/2005 rls Added MDrive input configuration structure.
 */

#ifndef	INCdrvIM483h
#define	INCdrvIM483h 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2 /* Timeout in seconds */

/* MDrive input configuration - 0 indicates unassigned. */
typedef struct inputc
{
    epicsUInt8 plusLS;	/* Input # of + limit switch. */
    epicsUInt8 minusLS;	/* Input # of - limit switch. */
    epicsUInt8 homeLS;	/* Input # of home switch. */
} input_config;


/* IMS specific data is stored in this structure. */
struct IM483controller
{
    asynUser *pasynUser;  	/* For RS-232 */
    char asyn_port[80];     	/* asyn port name */
    CommStatus status;		/* Controller communication status. */
    input_config *inconfig; 	/* Discrete input configuration - MDrive only. */
};

/* Function prototypes. */
extern RTN_STATUS IM483SMSetup(int, int);
extern RTN_STATUS IM483PLSetup(int, int);
extern RTN_STATUS  MDriveSetup(int, int);
extern RTN_STATUS IM483SMConfig(int, const char *);
extern RTN_STATUS IM483PLConfig(int, const char *);
extern RTN_STATUS  MDriveConfig(int, const char *);

#endif	/* INCdrvIM483h */

