/* File: drvPIC862.h             */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Ron Sluiter
 *      Current Author: Mohan Ramanathan
 *      Date: 09/04/2006
 *
 * Modification Log:
 * -----------------
 * .00  09/05/2006  mr  copied from drvPIC848.h
 * .01  09/25/2006 rls  COMM_TIMEOUT must be a float.
 */

#ifndef	INCdrvPIC862h
#define	INCdrvPIC862h 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2.0	/* Timeout in seconds. */

struct PIC862controller
{
    asynUser *pasynUser;	/* asynUser structure */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    CommStatus status;		/* Controller communication status. */
    char asyn_port[80];		/* asyn port name */
};


/*  LM629 Status bits */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int motor_off		:1; /* 7 - Motor loop OFF. */
	unsigned int breakpt		:1; /* 6 - Breakpoint reached flag. */
	unsigned int max_pos_err	:1; /* 5 - Maximum position error is exceeded. */
	unsigned int max_pos_lmt	:1; /* 4 - Maximum position Limit exceeded. */
	unsigned int index		:1; /* 3 - Index pulse received flag. */
	unsigned int trty_done		:1; /* 2 - Trajectory Complete. */
	unsigned int cmnd_err		:1; /* 1 - Command error flag. */
	unsigned int busy		:1; /* 0 - Busy. */
#else
	unsigned int busy		:1; /* 0 - Busy. */
	unsigned int cmnd_err		:1; /* 1 - Command error flag. */
	unsigned int trty_done		:1; /* 2 - Trajectory Complete. */
	unsigned int index		:1; /* 3 - Index pulse received flag. */
	unsigned int max_pos_lmt	:1; /* 4 - Maximum position Limit exceeded. */
	unsigned int max_pos_err	:1; /* 5 - Maximum position error is exceeded. */
	unsigned int breakpt		:1; /* 6 - Breakpoint reached flag. */
	unsigned int motor_off		:1; /* 7 - Motor loop OFF. */
#endif
    } Bits;                                
} C862_Status_Reg1;

/*  Internal operation flags */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int brd_addr		:1; /* 7 - Board Addressed. */
	unsigned int num_mode		:1; /* 6 - Number mode in Effect. */
	unsigned int ldzero_dis		:1; /* 3 - Leading Zero Suppression disabled. */
	unsigned int macro_act		:1; /* 4 - Macro Command called. */
	unsigned int ldzero_act		:1; /* 3 - Leading Zero Suppression active. */
	unsigned int cmnd_err		:1; /* 2 - Command error flag. */
	unsigned int wait_prgs		:1; /* 1 - Wait in Progress. */
	unsigned int echo_on		:1; /* 0 - Echo ON. */
#else
	unsigned int echo_on		:1; /* 0 - Echo ON. */
	unsigned int wait_prgs		:1; /* 1 - Wait in Progress. */
	unsigned int cmnd_err		:1; /* 2 - Command error flag. */
	unsigned int ldzero_act		:1; /* 3 - Leading Zero Suppression active. */
	unsigned int macro_act		:1; /* 4 - Macro Command called. */
	unsigned int ldzero_dis		:1; /* 3 - Leading Zero Suppression disabled. */
	unsigned int num_mode		:1; /* 6 - Number mode in Effect. */
	unsigned int brd_addr		:1; /* 7 - Board Addressed. */
#endif
    } Bits;                                
} C862_Status_Reg2;

/* Motor loop flags */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int int_comm		:1; /* 7 - Command error flag. */
	unsigned int move_err_ex	:1; /* 6 - Move Error Excess flag. */
	unsigned int na34	        :2;
	unsigned int move_err		:1; /* 3 - Move Error */
	unsigned int mvdir_pol		:1; /* 2 - Move driection polarity. */
	unsigned int na30	        :2;
#else
	unsigned int na30	        :2;
	unsigned int mvdir_pol		:1; /* 2 - Move driection polarity. */
	unsigned int move_err		:1; /* 3 - Move Error */
	unsigned int na34	        :2;
	unsigned int move_err_ex	:1; /* 6 - Move Error Excess flag. */
	unsigned int int_comm		:1; /* 7 - Command error flag. */
#endif
    } Bits;                                
} C862_Status_Reg3;

/* Signal lines status */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int na44	        :4;
	unsigned int brake_on		:1; /* 3 - Brake ON. */
	unsigned int find_prgs		:1; /* 2 - Find Edge Operation in Progress. */
	unsigned int lmt_high		:1; /* 1 - Limit Switch Active Stet HIGH */
	unsigned int lmt_on		:1; /* 0 - Limit Swicth ON flag. */
#else
	unsigned int lmt_on		:1; /* 0 - Limit Swicth ON flag. */
	unsigned int lmt_high		:1; /* 1 - Limit Switch Active Stet HIGH */
	unsigned int find_prgs		:1; /* 2 - Find Edge Operation in Progress. */
	unsigned int brake_on		:1; /* 3 - Brake ON. */
	unsigned int na44	        :4;
#endif
    } Bits;                                
} C862_Status_Reg4;

/* Signal lines inputs */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int na54	        :4;
	unsigned int minus_ls		:1; /* 3 - Negative limit signal flag. */
	unsigned int plus_ls		:1; /* 2 - Positive limit signal flag. */
	unsigned int index		:1; /* 1 - Reference signal flag */
	unsigned int na50	        :1;
#else
	unsigned int na50	        :1;
	unsigned int index		:1; /* 1 - Reference signal flag */
	unsigned int plus_ls		:1; /* 2 - Positive limit signal flag. */
	unsigned int minus_ls		:1; /* 3 - Negative limit signal flag. */
	unsigned int na54	        :4;
#endif
    } Bits;                                
} C862_Status_Reg5;


/* Function prototypes. */
extern RTN_STATUS PIC862Setup(int, int);
extern RTN_STATUS PIC862Config(int, const char *, int);

#endif	/* INCdrvPIC862h */
