/*
FILENAME...	drvSPiiPlus.h
USAGE...    This file contains ACS Tech80 driver "include"
	    information that is specific to the SPiiPlus serial controller

Version:	$Revision: 1.2 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2008-05-21 21:18:53 $

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
 * .01  04-07-05 jps initialized from drvMM4000.cc
 * .02  10-31-07 jps added command mode switch (BUFFER/DIRECT)
 */

#ifndef	INCdrvSPiiPlush
#define	INCdrvSPiiPlush 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define ACS_MSG_SIZE 120

// ACS Motion Commands - 
//     These definitions must match the native ACS programming
#define OP_ABS_MOVE 1
#define OP_REL_MOVE 2
#define OP_JOG_MOVE 3
#define OP_HOME_F   4
#define OP_HOME_R   5

#define QUERY_CNT  7
enum QUERY_TYPES {QSTATUS, QFAULT, QPOS, QEA_POS, QVEL, QHOME, QDONE};

// Query Command Strings 
#define QSTATUS_CMND     "?D/MST(%d)"
#define QFAULT_CMND      "?D/FAULT(%d)"
#define QPOS_CMND        "?APOS(%d)"
#define QEA_POS_CMND     "?FPOS(%d)"
#define QEA_POS_KIN_CMND "?FPOS%s"    // Read reverse kinimatic position ('CONNECT' mode)
#define QVEL_CMND        "?FVEL(%d)"
#define QHOME_CMND       "?opReq(%d)"
#define QDONE_CMND       "?Done(%d)"


/* Controller command interface modes 
 *      BUFFER - motion initiated by executing ACSPL buffers  (ie: Nanomotion stages)
 *      CONNECT - ACSPL 'CONNECT' command is being used (kinematics) (ie: Alio Hexapod)
 *      DIRECT - direct access to physical motors via command interpreter
 */ 
#define MODE_CNT  3
enum CMND_MODES {BUFFER, CONNECT, DIRECT};
#define BUFFER_STR "BUF"
#define CONNECT_STR "CON"
#define DIRECT_STR "DIR"


/* Motion Master specific data is stored in this structure. */
struct SPiiPlusController
{
    asynUser *pasynUser;  	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    enum CMND_MODES cmndMode;     /* Controller command interface mode */
    char asyn_port[80];     	/* asyn port name */
    char recv_string[QUERY_CNT][ACS_MSG_SIZE];  /* Query result strings */
    double home_preset[MAX_AXIS]; /* Controller's home preset position (XF command). */
    
    CommStatus status;		 /* Controller communication status. */
};


/* Motor status response for SPiiPlus . */
typedef union
{
    epicsUInt8 All;
    struct
    {
#ifdef MSB_First
      bool bit7	        :1;	/* Bit #7 N/A. */
      bool inaccel	:1;	/* Motor is accelerating */
      bool inmotion	:1;	/* Motor in-motion */
      bool inposition	:1;	/* Motor in position */
      bool bit3	        :1;	/* Bit #3 N/A */
      bool bit2	        :1;	/* Bit #2 N/A */
      bool openloop	:1;	/* Motor in open-loop (torque control) mode */
      bool enabled	:1;	/* Motor Enabled */
#else
      bool enabled	:1;	/* Motor Enabled */
      bool openloop	:1;	/* Motor in open-loop (torque control) mode */
      bool bit2	        :1;	/* Bit #2 N/A */
      bool bit3	        :1;	/* Bit #3 N/A */
      bool inposition	:1;	/* Motor in position */
      bool inmotion	:1;	/* Motor in-motion */
      bool inaccel	:1;	/* Motor is accelerating */
      bool bit7	        :1;	/* Bit #7 N/A. */
#endif
    } Bits;
} MOTOR_STATUS;


/*
 *  Motor fault flags for SPiiPlus 
 * (See page 8-4 in SPiiPlus ACSPL Programmers Guide)
 */
typedef union
{
    epicsUInt32 All;
    struct
    {
#ifdef MSB_First
      bool bit31        :1;     /* Bit #31 N/A */
      bool misc_faults  :6;	/* Misc. Faults */
      bool bits21_24    :4;	/* Bits #21 - #24 N/A */
      bool hssinc       :1;	/* HSSI Not Connected */
      bool bits19       :1;	/* Bits #9  N/A */
      bool bits18       :1;	/* Bits #18 N/A */
      bool sp           :1;	/* Servo Processor Alarm */
      bool limit_error	:3;	/* Limit Error (Vel, Accel, Current) */
      bool pos_error	:2;	/* Position Error */
      bool enc_error	:5;	/* Encoder or Drive error */
      bool sll          :1;	/* Software Left Limit */
      bool srl	        :1;	/* Software Right Limit */
      bool bits2_4      :3;	/* Bits #2 - #4  N/A */
      bool rl	        :1;	/* Left Limit */
      bool ll	        :1;	/* Right Limit */
#else
      bool ll	        :1;	/* Right Limit */
      bool rl	        :1;	/* Left Limit */
      bool bits2_4      :3;	/* Bits #2 - #4  N/A */
      bool srl	        :1;	/* Software Right Limit */
      bool sll          :1;	/* Software Left Limit */
      bool enc_error	:5;	/* Encoder or Drive error */
      bool pos_error	:2;	/* Position Error */
      bool limit_error	:3;	/* Limit Error (Vel, Accel, Current) */
      bool sp           :1;	/* Servo Processor Alarm */
      bool bits18       :1;	/* Bits #18 N/A */
      bool bits19       :1;	/* Bits #9  N/A */
      bool hssinc       :1;	/* HSSI Not Connected */
      bool bits21_24    :4;	/* Bits #21 - #24 N/A */
      bool misc_faults  :6;	/* Misc. Faults */
      bool bit31        :1;     /* Bit #31 N/A */
#endif
    } Bits;
} MOTOR_FAULTS;


#endif	/* INCdrvSPiiPlush */

