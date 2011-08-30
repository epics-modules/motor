/*
FILENAME...	motor.h
USAGE...	Definitions and structures common to all levels of motorRecord
		support (i.e., record, device and driver).

Version:	$Revision: 1.21 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2009-04-27 14:28:42 $
*/

/*
 *      Current Author: Ron Sluiter
 *      Date: 12/22/98
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
 *
 * .01 10-21-02 rls - Convert to R3.14.x and C++.
 * .02 12-12-03 rls - Added MAX_SCAN_RATE.
 *		    - Converted MSTA #define's to bit field.
 * .03 08-30-04 rls - Added osiUnistd.h for RTEMS.
 * .04 09-20-04 rls - Increase max. axis / board to 32 for Delta Tau PMAC.
 * .05 12-21-04 rls - Changed pre-compiler instructions for LSB/MSB_First
 *		      to support MS Visual C.
 * .06 01-27-06 rls - Added LT_EPICSBASE macro for test EPICS base versions.
 * .07 11-19-08 rls - More extensive bit field tests.
 * .08 00-09-10 rls - GNU preprocessor assertions are deprecated with VxWorks
 *                    6.x.  Added test for CPU macros.
 * .09 04-07-11 kmp - Added __APPLE__ to define bit order
 *                    tested by jph with simMotorDriver.
 * .10 08-25-11 rls - Replaced deprecated #cpu preprocessor assertions with
 *                    the defined() operator.
 */

#ifndef	INCmotorh
#define	INCmotorh 1

#include <stdio.h>
#include <dbScan.h>
#include <devSup.h>
#include <osiUnistd.h> 
#include <epicsVersion.h>

/* Less than EPICS base version test.*/
#define LT_EPICSBASE(v,r,l) ((EPICS_VERSION<=(v)) && (EPICS_REVISION<=(r)) && (EPICS_MODIFICATION<(l)))

/* Maximum message size of all supported devices; see drv[device].h for maximum
message size for each device. */
#define MAX_MSG_SIZE	300

#if defined(OK)
#undef OK
#endif

#if defined(ERROR)
#undef ERROR
#endif

typedef enum RTN_VALUES {OK = 0, ERROR = 1} RTN_STATUS;

typedef enum CALLBACK_VALUES {NOTHING_DONE = 0, CALLBACK_DATA = 1} CALLBACK_VALUE;

#define NINT(f)	(long)((f)>0 ? (f)+0.5 : (f)-0.5)	/* Nearest integer. */

/* Motor Record Command Set. !WARNING! this enumeration must match ALL of the
   following (up to and including JOG_VELOCITY);
    - "oms_table" in devOmsCom.c
    - "MM4000_table" in devMM4000.c
*/

typedef enum  {
	MOVE_ABS,	/* Absolute Move. */
	MOVE_REL,	/* Relative Move. */
	HOME_FOR,	/* Home Forward. */
	HOME_REV,	/* Home Reverse. */
	LOAD_POS,	/* Load Position. */
	SET_VEL_BASE,	/* Set Minimum Velocity. */
	SET_VELOCITY,	/* Set Jog and Trajectory Velocity. */
	SET_ACCEL,	/* Set Acceleration. */
	GO,		/* Start previously programmed move. */
	SET_ENC_RATIO,	/* Set Encoder Ratio. */
	GET_INFO,	/* Update Motor Status. */
	STOP_AXIS,	/* Stop Axis Motion. */
	JOG,		/* Momentary Jog. */
	SET_PGAIN,	/* Set Proportional Gain. */
	SET_IGAIN,	/* Set Integral Gain. */
	SET_DGAIN,	/* Set Derivative Gain. */
	ENABLE_TORQUE,	/* Enable Servo Closed-Loop Control. */
	DISABL_TORQUE,	/* Disable Servo Closed-Loop Control. */
	PRIMITIVE,	/* Primitive Controller command. */
	SET_HIGH_LIMIT,	/* Set High Travel Limit. */
	SET_LOW_LIMIT,	/* Set Low Travel Limit. */
	JOG_VELOCITY,	/* Change Jog velocity. */
	SET_RESOLUTION	/* Set resolution */
} motor_cmnd;


/* -------------------------------------------------- */

/* driver and device support parameters */
#define MAX_SCAN_RATE	60	/* Maximum polling rate in HZ. */
#define SCAN_RATE	6	/* Default polling rate in HZ. */
#define MAX_COUNT	50000	/*19000*/	/* timeout value */
#define MAX_AXIS	32	/* max number of axis per board */

#define NO		0
#define YES		1

/* Define, from top to bottom, how bit fields are packed. */
/* This works for gnu, SunPro, MS Visual C. */
#if defined(_WIN32) || defined(_M_IX86) || defined(_X86_)
    #define LSB_First (TRUE)  /* LSB is packed first. */
#elif defined(__i386__) || defined(_armv4l_) || defined(_X86_64_) || defined(__APPLE__)
    #define LSB_First (TRUE)  /* LSB is packed first. */
#elif defined(i386)
    #define LSB_First (TRUE)  /* LSB is packed first. */    
#elif defined(sparc) || defined(m68k) || defined(powerpc)
    #define MSB_First (TRUE)  /* MSB is packed first. */
#elif (CPU == PPC604) || (CPU == PPC603) || (CPU==PPC85XX) || (CPU == MC68040)
    #define MSB_First (TRUE)  /* MSB is packed first. */
#else
    #error: unknown bit order!
#endif

/* -------------------------------------------------- */
/* axis and encoder status for return to requester */

typedef union
{
    unsigned long All;
    struct
    {
#ifdef MSB_First
	unsigned int na		    :17;/* N/A bits  */
        unsigned int RA_HOMED       :1; /* Axis has been homed.*/
	unsigned int RA_MINUS_LS    :1;	/* minus limit switch has been hit */
	unsigned int CNTRL_COMM_ERR :1;	/* Controller communication error. */
	unsigned int GAIN_SUPPORT   :1;	/* Motor supports closed-loop position control. */
	unsigned int RA_MOVING      :1;	/* non-zero velocity present */
	unsigned int RA_PROBLEM     :1; /* driver stopped polling */
	unsigned int EA_PRESENT     :1; /* encoder is present */
	unsigned int EA_HOME        :1; /* encoder home signal on */
	unsigned int EA_SLIP_STALL  :1; /* slip/stall detected */
	unsigned int EA_POSITION    :1; /* position maintenence enabled */
	unsigned int EA_SLIP        :1; /* encoder slip enabled */
	unsigned int RA_HOME        :1; /* The home signal is on */
	unsigned int RA_PLUS_LS     :1; /* plus limit switch has been hit */
	unsigned int RA_DONE        :1;	/* a motion is complete */
	unsigned int RA_DIRECTION   :1;	/* (last) 0=Negative, 1=Positive */
#else
	unsigned int RA_DIRECTION   :1;	/* (last) 0=Negative, 1=Positive */
	unsigned int RA_DONE        :1;	/* a motion is complete */
	unsigned int RA_PLUS_LS     :1; /* plus limit switch has been hit */
	unsigned int RA_HOME        :1; /* The home signal is on */
	unsigned int EA_SLIP        :1; /* encoder slip enabled */
	unsigned int EA_POSITION    :1; /* position maintenence enabled */
	unsigned int EA_SLIP_STALL  :1; /* slip/stall detected */
	unsigned int EA_HOME        :1; /* encoder home signal on */
	unsigned int EA_PRESENT     :1; /* encoder is present */
	unsigned int RA_PROBLEM     :1; /* driver stopped polling */
	unsigned int RA_MOVING      :1;	/* non-zero velocity present */
	unsigned int GAIN_SUPPORT   :1;	/* Motor supports closed-loop position control. */
	unsigned int CNTRL_COMM_ERR :1;	/* Controller communication error. */
	unsigned int RA_MINUS_LS    :1;	/* minus limit switch has been hit */
        unsigned int RA_HOMED       :1; /* Axis has been homed.*/
	unsigned int na		    :17;/* N/A bits  */
#endif
    } Bits;                                
} msta_field;

/*
   The RA_PROBLEM status bit indicates that the driver has stopped the polling
   because it believes that the motor is not really moving.  Under normal
   operation, the controller will tell the driver when the motor is done
   moving.  This is a safety precaution and should normally be treated like a
   RA_DONE.
*/


/* device support entry table */
struct motor_dset
{
    struct dset base;
    CALLBACK_VALUE (*update_values) (struct motorRecord *);
    long (*start_trans) (struct motorRecord *);
    RTN_STATUS (*build_trans) (motor_cmnd, double *, struct motorRecord *);
    RTN_STATUS (*end_trans) (struct motorRecord *);
};


/* All db_post_events() calls set both VALUE and LOG bits. */
#define DBE_VAL_LOG (unsigned int) (DBE_VALUE | DBE_LOG)

#endif	/* INCmotorh */

