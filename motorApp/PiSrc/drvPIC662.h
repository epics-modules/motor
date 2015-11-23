/*
FILENAME...	drvPIC662.h
USAGE... This file contains driver "include" information that is specific to
Physik Instrumente (PI) GmbH & Co. motor controller driver support.

*/

/*
 *      Original Author: Joe Sullivan
 *      Date: 12/17/03
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
 * .01 03/08/06 jps - copied from drvPI.h
 */

#ifndef	INCdrvPIh
#define	INCdrvPIh 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT	2 /* Timeout in seconds. */

// E662 Command: switch to remote control mode
//    This is done before each positioning move because
//    the controller defaults to 'Local' mode on power up.
//    Status can be read while in 'Local' mode.
#define REMOTE_MODE "DEV:CONT REM"

/* PIC662 specific data is stored in this structure. */
struct PIC662controller
{
    asynUser *pasynUser;	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    char asyn_port[80];		/* asyn port name */
    CommStatus status;		/* Controller communication status. */

    bool stop_status;               /* Signal to set_status() - stop polling */
    double drive_resolution;    
    int res_decpts;              /* Max decimal points */
};

/* Standard Event Status register */


typedef union
{
    epicsUInt8 All;
    struct
    {
#ifdef MSB_First
      unsigned int PowerOn	:1;	// (Bit 7) Power On
      unsigned int UserReq	:1;	// User Request
      unsigned int CmdError     :1;	// Command Error
      unsigned int ExeError	:1;	// Execution Error
      unsigned int DevError	:1;	// Device Dependant Error
      unsigned int QueryError	:1;	// Query Error
      unsigned int ReqControl	:1;	// Request Control
      unsigned int OpComplete	:1;	// (Bit 0) Operation Complete 
#else
      unsigned int OpComplete	:1;	// (Bit 0) Operation Complete 
      unsigned int ReqControl	:1;	// Request Control
      unsigned int QueryError	:1;	// Query Error
      unsigned int DevError	:1;	// Device Dependant Error
      unsigned int ExeError	:1;	// Execution Error
      unsigned int CmdError     :1;	// Command Error
      unsigned int UserReq	:1;	// User Request
      unsigned int PowerOn	:1;	// (Bit 7) Power On
#endif
    } Bits;
} E662_ESR_REG;


/* Function prototypes. */
extern RTN_STATUS PIC662Setup(int, int);
extern RTN_STATUS PIC662Config(int, const char *);

#endif	/* INCdrvPIh */

