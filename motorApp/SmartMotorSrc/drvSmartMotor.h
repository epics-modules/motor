/*
FILENAME...     drvSmartMotor.h
USAGE... This file contains driver "include" information that is specific to
             Animatics Corporation SmartMotors.

Version:        1.0
Modified By:    shoaf
Last Modified:  2007/02/20
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
 * .01  02/20/2007 ses copied from drvMM4000.h
 */

#ifndef INCdrvSmartMotorh
#define INCdrvSmartMotorh 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2 /* Timeout in seconds */

/* Animatics SmartMotor specific data is stored in this structure. */
struct SmartMotorcontroller
{
    asynUser *pasynUser;        /* For RS-232 */
    char asyn_port[80];         /* asyn port name */
    int num_motors;             /* # daisy chained motors on link. */
    CommStatus status;          /* Controller communication status. */
};

/* Function prototypes. */

extern RTN_STATUS  SmartMotorSetup(int, int);
extern RTN_STATUS  SmartMotorConfig(int, const char *);

#endif  /* INCdrvSmartMotor.h */

