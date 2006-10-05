/*
FILENAME...	drvPMNCCom.h
USAGE... This file contains Newport Motion Master (MM) driver "include"
	    information that is specific to Motion Master models 3000/4000.

Version:	1.10
Modified By:	sullivan
Last Modified:	2004/08/17 21:28:22
*/

/*
 *      Original Author: Mark Rivers
 *      Current Author: Joe Sullivan
 *      Date: 06/28/06
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

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

/* Motor Characteristics - in steps */
#define MAX_VELOCITY    2000 
#define MIN_ACCEL       16
#define MAX_ACCEL       32000

/* Picomotor Network Controllers */
enum PMNC_model
{
  PMNC8750,
  PMNC8752
};


/* Picomotor Drivers - use DRT Command to determine */
enum PMD_model
{
  PMD8753=1,        // 3 channel open-loop
  PMD8751=2        // 1 channel closed-loop
};


/* Picomotor Driver 8751 Fault Codes (POWER OFF) */
enum PMD8751_FAULTS
{
  ENCODER=2,        
  MOTORSHORT=1,     
  NOMOTOR=0,        
  OVERHEAT=3
};



#ifndef __cplusplus
typedef enum PMNC_model PMNC_model;
typedef enum PMD_model PMD_model;
#endif

#define PMNC87xx_NUM_CARDS	10  /* Maximum number of controllers */
#define PMNC87xx_NUM_DRIVERS	10  /* Maximum number of drivers per controller */
#define PMNC8753_NUM_MOTORS	3   /* Maximum number of motors per driver  */
#define PMNC8751_NUM_MOTORS	1   /* Maximum number of motors per driver  */

/* Per axis definition */
struct PMD_axis
{
  PMD_model driverType;
  int driverNum;
  int motorNum;
};

/* Motion Master specific data is stored in this structure. */
struct PMNCcontroller
{
    asynUser *pasynUser;  	/* For RS-232 */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    char asyn_port[80];     	/* asyn port name */

    char aux_status_string[PMNC87xx_NUM_DRIVERS+1][25];     
    char status_string[PMNC87xx_NUM_DRIVERS+1][25];     
    char position_string[PMNC87xx_NUM_DRIVERS+1][25];   
    char chan_select_string[PMNC87xx_NUM_DRIVERS+1][25];  

    PMNC_model pmnc;                    /* Controller Type */ 
    bool changeEOS;                    /* Flag for controller version that does not terminate the 
                                        * STA command correctly */
    struct PMD_axis axisDef[MAX_AXIS];	/* Axis definition - New Focus Driver Model and Number */
    long last_position[MAX_AXIS]; /* Track last incremtal position to noMotion status */
    int total_drivers;               /* Count number of drivers connected to controller */
    CommStatus status;		/* Controller communication status. */
};


/* Motor status response for PMNC. */
typedef union
{
    epicsUInt8 All;
  /*  8753 Driver Status */
    struct
    {
#ifdef MSB_First
	bool bit7		:1;	/* Bit #7 N/A. */
        bool positionMode	:1;	/* Set if moving in position mode */
        bool velocityMode	:1;	/* Set if moving in velocity mode */
        bool atVelocity	        :1;	/* Set if commanded velocity is reached */
        bool motorSelected	:1;	/* Set if valid motor selected */
	bool powerOn	        :1;	/* Motor power 1 - ON; 0 - OFF. */
        bool chksumError	:1;	/* CheckSum Error */
        bool inMotion	        :1;	/* In-motion indicator. */

#else
	bool inMotion	        :1;	/* In-motion indicator. */
        bool cksumError	:1;	/* CheckSum Error */
	bool powerOn	        :1;	/* Motor power 1 - ON; 0 - OFF. */
        bool motorSelected	:1;	/* Set if valid motor selected */
        bool atVelocity	        :1;	/* Set if commanded velocity is reached */
        bool velocityMode	:1;	/* Set if moving in velocity mode */
        bool positionMode	:1;	/* Set if moving in position mode */
	bool bit7		:1;	/* Bit #7 N/A. */
#endif
    } Bits_8753;
  /*  8751 Driver Status - SERVO ON and POWER ON */
    struct
    {
#ifdef MSB_First
        bool homing	        :1;	/* Set ON if homing in progress (FIN, RIN) */
        bool forLimit	        :1;	/* Set OFF if forward limit reached */
        bool revLimit	        :1;	/* Set OFF if reverse limit reached */
        bool posError	        :1;	/* Set ON if position error limit exceeded */
        bool powerOn	        :1;	/* Motor Power Indicator */
	bool noMotor	        :1;	/* Set ON if Missing Motor. */
        bool cksumError	        :1;	/* CheckSum Error */
        bool moveDone	        :1;	/* Set ON if Done. */

#else
	bool moveDone	        :1;	/* In-motion indicator. */
        bool chksumError	:1;	/* CheckSum Error */
	bool noMotor	        :1;	/* Set ON if Missing Motor. */
	bool powerOn	        :1;	/* Motor power indicator 1 - ON; 0 - OFF. */
        bool posError	        :1;	/* Set ON if position error limit exceeded */
        bool revLimit	        :1;	/* Set OFF if reverse limit reached */
        bool forLimit	        :1;	/* Set OFF if forward limit reached */
        bool homing	        :1;	/* Set ON if homing in progress (FIN, RIN) */
#endif
    } Bits_8751;
  /*  8751 Driver Status - SERVO OFF */
    struct
    {
#ifdef MSB_First
        bool homing	        :1;	/* Set ON if homing in progress (FIN, RIN) */
        bool overheat	        :1;	/* Set OFF if motor overheated */
        bool stopIN	        :1;	/* Set OFF if External STOP input ACTIVE */
        bool posError	        :1;	/* Set ON if position error limit exceeded */
        bool powerOn	        :1;	/* Set ON if valid motor selected */
	bool noMotor	        :1;	/* Set ON if Missing Motor. */
        bool cksumError	        :1;	/* CheckSum Error */
        bool moveDone	        :1;	/* Set ON if Done. */

#else
	bool moveDone	        :1;	/* In-motion indicator. */
        bool chksumError	:1;	/* CheckSum Error */
	bool noMotor	        :1;	/* Set ON if Missing Motor. */
	bool powerOn	        :1;	/* Motor power indicator */
        bool posError	        :1;	/* Set ON if position error limit exceeded */
        bool stopIN	        :1;	/* Set OFF if External STOP input ACTIVE */
        bool overheat	        :1;	/* Set OFF if motor overheated */
        bool homing	        :1;	/* Set ON if homing in progress (FIN, RIN) */
#endif
    } Bits_8751_NOS;
  /*  8751 Driver Status - SERVO ON and POWER OFF */
    struct
    {
#ifdef MSB_First
        bool homing	        :1;	/* Set ON if homing in progress (FIN, RIN) */
        bool fault	        :2;	/* Driver Power Faults  - See PMD8751_FAULTS */
        bool posError	        :1;	/* Set ON if position error limit exceeded */
        bool powerOn	        :1;	/* Set ON if valid motor selected */
	bool noMotor	        :1;	/* Set ON if Missing Motor. */
        bool cksumError	        :1;	/* CheckSum Error */
        bool moveDone	        :1;	/* Set ON if Done. */

#else
	bool moveDone	        :1;	/* In-motion indicator. */
        bool chksumError	:1;	/* CheckSum Error */
	bool noMotor	        :1;	/* Set ON if Missing Motor. */
	bool powerOn	        :1;	/* Motor power indicator */
        bool posError	        :1;	/* Set ON if position error limit exceeded */
        bool fault	        :2;	/* Driver Power Faults - See PMD8751_FAULTS */
        bool homing	        :1;	/* Set ON if homing in progress (FIN, RIN) */
#endif
    } Bits_8751_FAULT;

} MOTOR_STATUS;


/* Driver Aux Status response for PMNC. */
typedef union
{
    epicsUInt8 All;
  /*  8751 Driver Aux Status - DIAG Command */
    struct
    {
#ifdef MSB_First
	bool bit7		:1;	/* Bit #7 N/A. */
        bool bit6	        :1;	/* Bit #6 N/A. */
        bool servoOverRun	:1;	/* Set if servo process time overrun (>.512msec) */
        bool slewDone	        :1;	/* Set when trapesoidal slew time over */
        bool accelDone 	        :1;	/* Set when trapesoidal accell time over */
        bool servoOn	        :1;	/* Set if servo mode ON - else OpenLoop */
        bool posWrap	        :1;	/* Set on 32 bit overflow (position) */
        bool diagBit	        :1;	/* Used to encode error diagnostics */

#else
        bool diagBit	        :1;	/* Used to encode error diagnostics */
        bool posWrap	        :1;	/* Set on 32 bit overflow (position) */
        bool servoOn	        :1;	/* Set if servo mode ON - else OpenLoop */
        bool accelDone 	        :1;	/* Set when trapesoidal accell time over */
        bool slewDone	        :1;	/* Set when trapesoidal slew time over */
        bool servoOverRun	:1;	/* Set if servo process time overrun (>.512msec) */
        bool bit6	        :1;	/* Bit #6 N/A. */
	bool bit7		:1;	/* Bit #7 N/A. */
#endif
    } Bits_8751;
} MOTOR_AUX_STATUS;




#endif	/* INCdrvPMNCComh */

