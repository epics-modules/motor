/*
FILENAME...     drvEnsemble.h
USAGE... This file contains Aerotech Ensemble driver "include" information.

Version:        $Revision: 1.3 $
Modified By:    $Author: sluiter $
Last Modified:  $Date: 2009-05-01 18:13:42 $
*/

/*
 *      Original Author: Chad Weimer
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
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .00  04-01-08 caw initialized from drvMM4000.h (Newport)
 */

#ifndef INCdrvEnsembleh
#define INCdrvEnsembleh 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"


// The following should be defined to have the same value as
// the Ensemble parameters specified
#define ASCII_EOS_CHAR      '\n'  // AsciiCmdEOSChar
#define ASCII_EOS_STR       "\n"
#define ASCII_ACK_CHAR      '%'   // AsciiCmdAckChar
#define ASCII_NAK_CHAR      '!'   // AsciiCmdNakChar
#define ASCII_FAULT_CHAR    '#'   // AsciiCmdFaultChar
#define ASCII_TIMEOUT_CHAR  '$'   // AsciiCmdTimeoutChar

#define BUFF_SIZE 100       /* Maximum length of string to/from Ensemble */


/* Ensemble specific data is stored in this structure. */
struct Ensemblecontroller
{
    asynUser *pasynUser;                  /* For RS-232/Ethernet */
    int asyn_address;                     /* Use for GPIB or other address with asyn */
    char asyn_port[80];                   /* asyn port name */
    int axes[MAX_AXIS];
    double drive_resolution[MAX_AXIS];
    int res_decpts[MAX_AXIS];             /* Drive resolution significant dec. pts. */
    double home_preset[MAX_AXIS];         /* Controller's home preset position (XF command). */
    epicsUInt32 home_dparam[MAX_AXIS];    /* Controller's HomeDirection parameter. */
    CommStatus status;                    /* Controller communication status. */
};

/* Function prototypes. */
extern RTN_STATUS EnsembleSetup(int, int);
extern RTN_STATUS EnsembleConfig(int, const char *, int);


#endif  /* INCdrvEnsembleh */

