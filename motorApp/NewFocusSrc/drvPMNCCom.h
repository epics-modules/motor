/*
FILENAME...	drvPMNCCom.h
USAGE... This file contains Newport Motion Master (MM) driver "include"
	    information that is specific to Motion Master models 3000/4000.

Version:	1.10
Modified By:	rivers
Last Modified:	2004/08/17 21:28:22
*/

/*
 *      Original Author: Mark Rivers
 *      Current Author: Mark Rivers
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
 * .01  01-18-93 mlr initialized from drvOms58
 * .02  06-16-03 rls Converted to R3.14.x.
 */

#ifndef	INCdrvPMNCComh
#define	INCdrvPMNCComh 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

enum PMNC_model
{
    PMNC8750
};

#ifndef __cplusplus
typedef enum PMNC_model PMNC_model;
#endif

#define PMNC8750_NUM_CARDS	3  /* Maximum number of controllers */
#define PMNC8750_NUM_DRIVERS	3  /* Maximum number of drivers per controller */
#define PMNC8750_NUM_MOTORS	3  /* Maximum number of motors per driver  */

/* Motion Master specific data is stored in this structure. */
struct PMNCcontroller
{
    asynUser *pasynUser;  	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    char asyn_port[80];     	/* asyn port name */

/* String containing status of selected motors */
    char status_string[PMNC8750_NUM_DRIVERS+2][25];     
/* String containing position of motors */
    char position_string[PMNC8750_NUM_DRIVERS][25];   
 /* String containing selected channel */
    char chan_select_string[PMNC8750_NUM_DRIVERS][25];  
    PMNC_model model;		/* New Focus Model Info */
    long last_position[MAX_AXIS]; /* Track last incremtal position to noMotion status */
    int total_drivers;               /* Count number of drivers connected to controller */
    CommStatus status;		/* Controller communication status. */
};


/* Motor status response for PMNC. */
typedef union
{
    epicsUInt8 All;
    struct
    {
#ifdef MSB_First
	bool bit7		:1;	/* Bit #7 N/A. */
        bool positionMode	:1;	/* Set if moving in position mode */
        bool velocityMode	:1;	/* Set if moving in velocity mode */
        bool atVelocity	        :1;	/* Set if commanded velocity is reached */
        bool motorSelected	:1;	/* Set if valid motor selected */
	bool powerOn	        :1;	/* Motor power 1 - ON; 0 - OFF. */
        bool checksumError	:1;	/* CheckSum Error */
        bool inMotion	        :1;	/* In-motion indicator. */

#else
	bool inMotion	        :1;	/* In-motion indicator. */
        bool checksumError	:1;	/* CheckSum Error */
	bool powerOn	        :1;	/* Motor power 1 - ON; 0 - OFF. */
        bool motorSelected	:1;	/* Set if valid motor selected */
        bool atVelocity	        :1;	/* Set if commanded velocity is reached */
        bool velocityMode	:1;	/* Set if moving in velocity mode */
        bool positionMode	:1;	/* Set if moving in position mode */
	bool bit7		:1;	/* Bit #7 N/A. */
#endif
    } Bits;
} MOTOR_STATUS;


#endif	/* INCdrvPMNCComh */

