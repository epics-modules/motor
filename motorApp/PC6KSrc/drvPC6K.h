/*
FILENAME...	drvPC6K.h
USAGE...    This file contains Parker Compumotor driver "include"
	    information that is specific to the 6K series serial controller

Version:	$Revision: 1.2 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2006-08-31 15:42:31 $
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
 * .01  06-20-05 jps initialized from drvSPiiPlus.h
 */

#ifndef	INCdrvPC6Kh
#define	INCdrvPC6Kh 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define PC6K_MSG_SIZE 120

#define PC6K_STATUS_RETRY 10

/* End-of-string defines */
#define PC6K_OUT_EOS   "\n" /* Command */
// #define PC6K_IN_EOS    "\n"  /* Reply */
#define PC6K_IN_EOS    ">"  /* Reply */

#define PC6K_QUERY_CNT 5
enum PC6K_query_types {QSTATUS, QPOS, QEA_POS, QVEL, QDRIVE};


enum PC6K_model
{
    PC6K8,
    PCGem6K
};

enum PC6K_motor_type
{
    UNUSED,
    STEPPER,
    DC
};

#ifndef __cplusplus
typedef enum PC6K_model PC6K_model;
typedef enum PC6K_motor_type PC6K_motor_type;
#endif


/* Motion Master specific data is stored in this structure. */
struct PC6KController
{
    asynUser *pasynUser;  	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    char asyn_port[80];     	/* asyn port name */
    char recv_string[PC6K_QUERY_CNT][PC6K_MSG_SIZE]; /* Query result strings */
    double home_preset[MAX_AXIS]; /* Controller's home preset position (XF command). */
    PC6K_model model;		/* Motion Master Model. */
    PC6K_motor_type type[8];	/* For 6K8 only; Motor type array. */
    /* Controller resolution array 
     * Units are in [Controller EGU's / Record RAW units].
     */
    double drive_resolution[MAX_AXIS];
    int res_decpts[MAX_AXIS];	/* Drive resolution significant dec. pts. */

    CommStatus status;		/* Controller communication status. */
    int status_retry;           /* This controller needs multiple retry after GO */
};


/* Motor status bits for PC6K - TAS command. 
* 
* cmd: <#>TAS
* rsp: *<#>TAS0000_0000_0000_0000_0000_0000_0000_0000 (32 bits)
*             ^ bit 1                               ^bit 32
*/

/* Adjusted bit number (-1) from manual - start counting from 0) */
#define TAS_INMOTION   0    
#define TAS_NEG        1
#define TAS_HOME       4+1     /* Home Sucessfull */
#define TAS_DRIVEDOWN  12+3    /* Drive Shutdown */
#define TAS_DRIVEFAULT 13+3    /* Drive Fault Occurred */
#define TAS_HPLUSTL    14+3    /* Hardware Plus Travel Limit */
#define TAS_HMINUSTL   15+3    /* Hardware Minus Travel Limit */
#define TAS_SPLUSTL    16+4    /* Software Plus Travel Limit */
#define TAS_SMINUSTL   17+4    /* Software Minus Travel Limit */
#define TAS_POSERROR   22+5    /* Position Error */
#define TAS_INTARGET   23+5    /* In Target Zone */


#endif	/* INCdrvPC6Kh */

