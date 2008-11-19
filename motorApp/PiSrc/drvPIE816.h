/* File: drvPIE816.h             */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Ron Sluiter
 *      Current Author: Joe Sullivan
 *      Date: 09/20/2005
 *
 * Modification Log:
 * -----------------
 * .00  09/13/2006  jps  copied from drvPIC838.h
 */

#ifndef	INCdrvPIE816h
#define	INCdrvPIE816h 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2		/* Timeout in seconds. */
#define POS_RES 0.0001		/* Position resolution. */

#define EOL_E816    "\n"        /* Command End-Of-Line = LF (0x10) */

struct PIE816controller
{
    asynUser *pasynUser;	/* asynUser structure */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    CommStatus status;		/* Controller communication status. */
    double drive_resolution[4];
    bool versionSupport;        /* Track supported Versions - include in Report */
    char asyn_port[80];		/* asyn port name */
};


typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
        unsigned int cmnd_err		:1; /* 15 - Command Error */
        unsigned int na6		:1; /* 14 - */
        unsigned int autozero	        :1; /* 13 - AutoZero function is running */
	unsigned int plus_ls		:1; /* 12 - Positive limit switch flag. */
	unsigned int minus_ls		:1; /* 11 - Negative limit switch flag. */
        unsigned int moving		:1; /* 10 - Moving indicator - position error outside tolerance */
        unsigned int volt_limit	        :1; /* 9 - piezo voltage limit reached */
        unsigned int torque		:1; /* 8 - Servo-control status */
        unsigned int nabyte             :8;     

#else
        unsigned int nabyte             :8;     
        unsigned int torque		:1; /* 8 - Servo-control status */
        unsigned int volt_limit	        :1; /* 9 - piezo voltage limit reached */
        unsigned int moving		:1; /* 10 - Moving indicator - position error outside tolerance */
	unsigned int minus_ls		:1; /* 11 - Negative limit switch flag. */
	unsigned int plus_ls		:1; /* 12 - Positive limit switch flag. */
        unsigned int autozero	        :1; /* 13 - AutoZero function is running */
        unsigned int na6		:1; /* 14 - */
        unsigned int cmnd_err		:1; /* 15 - Command Error */
#endif
    } Bits;                                
} E816_Status_Reg;


/* Function prototypes. */
extern RTN_STATUS PIE816Setup(int, int);
extern RTN_STATUS PIE816Config(int, const char *, int);

#endif	/* INCdrvPIE816h */
