/*
FILENAME...	motor.h
USAGE...	Definitions and structures common to all levels of motorRecord
		support (i.e., record, device and driver).

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2001-12-14 20:45:23 $
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
 */

#ifndef	INCmotorh
#define	INCmotorh 1

#include <stdio.h>
#include <dbScan.h>
#include <devSup.h>

/* Maximum message size of all supported devices; see drv[device].h for maximum
message size for each device. */
#define MAX_MSG_SIZE	300

typedef enum BOOLEAN_VALUES {OFF = 0, ON = 1} BOOLEAN;

#define NINT(f)	(long)((f)>0 ? (f)+0.5 : (f)-0.5)	/* Nearest integer. */

/* Motor Record Command Set. !WARNING! this enumeration must match ALL of the
   following;
    - "oms_table" in devOmsCom.c
    - "MM4000_table" in devMM4000.c
*/

enum motor_cmnd {
	MOVE_ABS,	/* Absolute Move. */
	MOVE_REL,	/* Relative Move. */
	HOME_FOR,	/* Home Forward. */
	HOME_REV,	/* Home Reverse. */
	LOAD_POS,	/* Load Zero Position. */
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
	JOG_VELOCITY	/* Change Jog velocity. */
};

#ifndef __cplusplus
typedef enum motor_cmnd motor_cmnd;
#endif

/* -------------------------------------------------- */

/* driver and device support parameters */
#define SCAN_RATE	6	/* 60=once a second */
#define MAX_COUNT	50000 /*19000*/	/* timeout value */
#define MAX_AXIS	10	/* max number of axis per board */

#define NOTHING_DONE	0
#define CALLBACK_DATA 	1

#define NO		0
#define YES		1

/* -------------------------------------------------- */
/* axis and encoder status for return to requester */
#define RA_DIRECTION		0x01	/* (last) 0=Negative, 1=Positive */
#define RA_DONE			0x02	/* a motion is complete */
#define RA_OVERTRAVEL		0x04	/* a limit switch has been hit */
#define RA_HOME			0x08	/* The home signal is on */
#define EA_SLIP			0x10	/* encoder slip enabled */
#define EA_POSITION		0x20	/* position maintenence enabled */
#define EA_SLIP_STALL		0x40	/* slip/stall detected */
#define EA_HOME			0x80	/* encoder home signal on */
#define EA_PRESENT		0x100	/* encoder is present */
#define RA_PROBLEM		0x200	/* driver stopped polling */
#define RA_MOVING		0x400	/* non-zero velocity present */
#define GAIN_SUPPORT		0x800	/* Motor supports closed-loop position
					    control. */
#define CNTRL_COMM_ERR		0x1000	/* Controller communication error. */

/*
   The RA_PROBLEM status bit indicates that the driver has stopped the polling
   because it believes that the motor is not really moving.  Under normal
   operation, the controller will tell the driver when the motor is done
   moving.  This is a safety precaution and should normally be treated like a
   RA_DONE.
*/

#ifdef __cplusplus
struct local_dset
{
    long number;			/*number of support routines*/
    long (*report) (FILE, int);		/*print report*/
    long (*init) (int);			/*init support*/
    long (*init_record) (void *);	/*init support for particular record*/
    long (*get_ioint_info)
	(int, struct dbCommon *, IOSCANPVT *); /* get io interrupt information*/
};
#endif

/* device support entry table */
struct motor_dset
{
#ifdef __cplusplus
    struct local_dset base;
    long (*update_values) (struct motorRecord *);
    long (*start_trans) (struct motorRecord *);
    long (*build_trans) (motor_cmnd, double *, struct motorRecord *);
    long (*end_trans) (struct motorRecord *);
#else
    struct dset base;
    DEVSUPFUN update_values;
    DEVSUPFUN start_trans;
    DEVSUPFUN build_trans;
    DEVSUPFUN end_trans;
#endif
};

#endif	/* INCmotorh */

