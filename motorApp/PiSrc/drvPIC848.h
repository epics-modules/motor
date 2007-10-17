/* File: drvPIC848.h             */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Ron Sluiter
 *      Current Author: Ron Sluiter
 *      Date: 09/20/2005
 *
 * Modification Log:
 * -----------------
 * .00  09/20/2005  rls  copied from drvPIC630.h
 * .01  10/17/2007  rls  - Added "reference" home switch indicator.
                         - increased position resolution scaler.
 */

#ifndef	INCdrvPIC848h
#define	INCdrvPIC848h 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2          /* Timeout in seconds. */
#define POS_RES 0.000001        /* Position resolution. */

struct PIC848controller
{
    bool reference[4];          /* reference sensor; if true, axis position can
                                 * only be set to zero; if false, axis position
                                 * must be set before axis can be moved. */
    asynUser *pasynUser;	/* asynUser structure */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    CommStatus status;		/* Controller communication status. */
    double drive_resolution[4];
    char asyn_port[80];		/* asyn port name */
};


typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int na15		:1;
	unsigned int na14		:1;
	unsigned int axisID		:2; /*12-13 - Axis ID; 0/A, 1/B, 2/C, 3/D */
	unsigned int na11	        :1;
	unsigned int in_motion		:1; /*10 - Axis in motion flag. */
	unsigned int axis_on		:1; /* 9 - Axis on; always on (1). */
	unsigned int torque		:1; /* 8 - Servo-control status. */
	unsigned int cmnd_err		:1; /* 7 - Command error flag. */
	unsigned int minus_ls		:1; /* 6 - Negative limit switch flag. */
	unsigned int plus_ls		:1; /* 5 - Positive limit switch flag. */
	unsigned int max_pos_err	:1; /* 4 - Maximum position error is exceeded. */
	unsigned int index		:1; /* 3 - Index pulse received flag. */
	unsigned int breakpt		:1; /* 2 - Breakpoint reached flag. */
	unsigned int wrap_around	:1; /* 1 - Position wrap-around flag. */
	unsigned int Done		:1; /* 0 - Axis trajectory complete. */
#else
	unsigned int Done		:1; /* 0 - Axis trajectory complete. */
	unsigned int wrap_around	:1; /* 1 - Position wrap-around flag. */
	unsigned int breakpt		:1; /* 2 - Breakpoint reached flag. */
	unsigned int index		:1; /* 3 - Index pulse received flag. */
	unsigned int max_pos_err	:1; /* 4 - Maximum position error is exceeded. */
	unsigned int plus_ls		:1; /* 5 - Positive limit switch flag. */
	unsigned int minus_ls		:1; /* 6 - Negative limit switch flag. */
	unsigned int cmnd_err		:1; /* 7 - Command error flag. */
	unsigned int torque		:1; /* 8 - Servo-control status. */
	unsigned int axis_on		:1; /* 9 - Axis on; always on (1). */
	unsigned int in_motion		:1; /*10 - Axis in motion flag. */
	unsigned int na11	        :1;
	unsigned int axisID		:2; /*12-13 - Axis ID; 0/A, 1/B, 2/C, 3/D */
	unsigned int na14		:1;
	unsigned int na15		:1;
#endif
    } Bits;                                
} C848_Status_Reg;


/* Function prototypes. */
extern RTN_STATUS PIC848Setup(int, int);
extern RTN_STATUS PIC848Config(int, const char *, int);

#endif	/* INCdrvPIC848h */
