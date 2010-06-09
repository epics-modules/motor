/*
FILENAME...	drvSoloist.h
USAGE... This file contains Aerotech Soloist driver "include" information.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/16/97
 *      Current Author: Aerotech, Inc.
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
 * .01	04-29-09	hcf		initialized from drvEnsemble.h (Aerotech)
 * .02	07-07-09	cjb		merged fixes from Ensemble code (drvEnsemble.h) in R6-4-4
 */

#ifndef	INCdrvSoloisth
#define	INCdrvSploisth 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

// The following should be defined to have the same value as
// the Soloist parameters specified
#define ASCII_EOS_CHAR		'\n'	// AsciiCmdEOSChar
#define ASCII_EOS_STR		"\n"
#define ASCII_ACK_CHAR		'%'		// AsciiCmdAckChar
#define ASCII_NAK_CHAR		'!'		// AsciiCmdNakChar
#define ASCII_FAULT_CHAR	'#'		// AsciiCmdFaultChar

#define BUFF_SIZE 100       /* Maximum length of string to/from Soloist */


/* Soloist specific data is stored in this structure. */
struct Soloistcontroller
{
	asynUser *pasynUser;				/* For RS-232/Ethernet */
	int asyn_address;					/* Use for GPIB or other address with asyn */
	char asyn_port[80];					/* asyn port name */
	int axes[MAX_AXIS];
	double drive_resolution[MAX_AXIS];
	int res_decpts[MAX_AXIS];			/* Drive resolution significant dec. pts. */
	double home_preset[MAX_AXIS];		/* Controller's home preset position (XF command). */
	epicsUInt32 home_dparam[MAX_AXIS];    /* Controller's HomeDirection parameter. */
	CommStatus status;					/* Controller communication status. */
};

/* Function prototypes. */
extern RTN_STATUS SoloistSetup(int, int);
extern RTN_STATUS SoloistConfig(int, const char *, int);


#endif	/* INCdrvSoloisth */

