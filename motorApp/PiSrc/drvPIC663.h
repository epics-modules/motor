/* File: drvPIC663.h             */


/* Device Driver Support definitions for motor */
/*
 *      Copied from devPIC862.cc by Jonathan Thompson, Jan 2011
 *
 * Modification Log:
 * -----------------
 * Jan 2011 - All references to 862 changed to 663
 *            Status register definitions changed to match 663
 */

#ifndef	INCdrvPIC663h
#define	INCdrvPIC663h 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2.0	/* Timeout in seconds. */

struct PIC663controller
{
    asynUser *pasynUser;	/* asynUser structure */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    CommStatus status;		/* Controller communication status. */
    char asyn_port[80];		/* asyn port name */
};


/*  Operation status */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int na8		:8; /* NA8. */
	unsigned int drv_cur_act	:1; /* 7 - Drive current active. */
	unsigned int brake_on		:1; /* 6 - Brake on. */
	unsigned int brake_off		:1; /* 5 - Brake off. */
	unsigned int macro_act		:1; /* 4 - Macro running. */
	unsigned int joy_on		:1; /* 3 - Joystick on. */
	unsigned int ref_drv_act	:1; /* 2 - Reference drive active. */
	unsigned int on_target		:1; /* 1 - On target. */
	unsigned int ready		:1; /* 0 - Ready. */
#else
	unsigned int ready		:1; /* 0 - Ready. */
	unsigned int on_target		:1; /* 1 - On target. */
	unsigned int ref_drv_act	:1; /* 2 - Reference drive active. */
	unsigned int joy_on		:1; /* 3 - Joystick on. */
	unsigned int macro_act		:1; /* 4 - Macro running. */
	unsigned int brake_off		:1; /* 5 - Brake off. */
	unsigned int brake_on		:1; /* 6 - Brake on. */
	unsigned int drv_cur_act	:1; /* 7 - Drive current active. */
	unsigned int na8		:8; /* NA8. */
#endif
    } Bits;                                
} C663_Status_Reg1;

/*  Hardware flags */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int na8		:8; /* NA8. */
	unsigned int in_4		:1; /* 7 - Digital input 4. */
	unsigned int in_3		:1; /* 6 - Digital input 3. */
	unsigned int in_2		:1; /* 3 - Digital input 2. */
	unsigned int in_1		:1; /* 4 - Digital input 1. */
	unsigned int unused		:1; /* 3 - No function. */
	unsigned int hi_limit		:1; /* 2 - Positive limit. */
	unsigned int ref_signal		:1; /* 1 - Reference signal. */
	unsigned int lo_limit		:1; /* 0 - Negative limit. */
#else
	unsigned int lo_limit		:1; /* 0 - Negative limit. */
	unsigned int ref_signal		:1; /* 1 - Reference signal. */
	unsigned int hi_limit		:1; /* 2 - Positive limit. */
	unsigned int unused		:1; /* 3 - No function. */
	unsigned int in_1		:1; /* 4 - Digital input 1. */
	unsigned int in_2		:1; /* 3 - Digital input 2. */
	unsigned int in_3		:1; /* 6 - Digital input 3. */
	unsigned int in_4		:1; /* 7 - Digital input 4. */
	unsigned int na8		:8; /* NA8. */
#endif
    } Bits;                                
} C663_Status_Reg2;

/* Error reporting */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int na8		:8; /* NA8. */
	unsigned int error		:8; /* Error code. */
#else
	unsigned int error		:8; /* Error code. */
	unsigned int na8		:8; /* NA8. */
#endif
    } Bits;                                
} C663_Status_Reg3;

#define C663_Status_Reg3_NO_ERROR 0x00
#define C663_Status_Reg3_RS232_TIMEOUT 0x01
#define C663_Status_Reg3_RS232_OVERFLOW 0x02
#define C663_Status_Reg3_MACRO_STORAGE_FULL 0x03
#define C663_Status_Reg3_MACRO_OUT_OF_RANGE 0x04
#define C663_Status_Reg3_WRONG_MACRO_CMD 0x05
#define C663_Status_Reg3_COMMAND_ERROR 0x06


/* Function prototypes. */
extern RTN_STATUS PIC663Setup(int, int);
extern RTN_STATUS PIC663Config(int, const char *, int);

#endif	/* INCdrvPIC663h */
