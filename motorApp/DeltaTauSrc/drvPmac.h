/*
FILENAME...	drvPmac.h
USAGE... This file contains Delta Tau PMAC driver "include" information.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-06-07 19:27:05 $
*/

/*
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
 * 00 04/16/04 rls - Copied from drvOms.h
 */

#ifndef	INCdrvPmach
#define	INCdrvPmach 1

#include "motor.h"
#include "motordrvCom.h"

/* Define for return test on devNoResponseProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

#define BUFF_SIZE 100       /* Maximum length of string to/from PMAC */

/* Default profile. */

#define Pmac_NUM_CARDS		1	/* Maximum number of cards. */
#define Pmac_BRD_SIZE		0x4000	/* card address boundary */
#define Pmac_MAX_AXES		32
#define Pmac_INTERRUPT_TYPE	intVME
#define Pmac_INT_VECTOR		180	/* default interrupt vector (64-255) */
#define Pmac_INT_LEVEL		5	/* default interrupt level (1-6) */

/* PMAC Commands. */

#define AXIS_STOP "\\"

/* !!!!! DELETE THIS !!!PMAC specific data is stored in this structure. */
struct PMACcontroller
{
    int status;
};

typedef struct
{
    union
    {
	epicsUInt16 All;
	struct
	{
#ifdef MSB_First
	    unsigned int motor_on       :1; /* Motor Activated (23). */
	    unsigned int neg_limit_set  :1; /* Negative End Limit Set (22). */
	    unsigned int pos_limit_set  :1; /* Positive End Limit Set (21). */
	    unsigned int x_servo_on     :1; /* Extended Servo Algorithm Enabled (20). */
	    unsigned int amp_enabled    :1; /* Amplifier Enabled (19). */
	    unsigned int open_loop      :1; /* Open Loop Mode (18). */
	    unsigned int move_time_on   :1; /* Move Timer Active (17). */
	    unsigned int integrate_mode :1; /* Integration Mode (16). */
	    unsigned int dwell		:1; /* Dwell in Progress (15). */
	    unsigned int DataBlkErr	:1; /* Data Block Error (14). */
	    unsigned int CmndVelZero	:1; /* Desired Velocity Zero (13). */
	    unsigned int decel_abort    :1; /* Abort Deceleration (12). */
	    unsigned int block_request  :1; /* Block Request (11). */
	    unsigned int homeing        :1; /* Home Search in Progress (10). */
	    unsigned int user_phase     :1; /* User-Written Phase Enable (9). */
	    unsigned int user_servo	:1; /* User-Written Servo Enable (8). */
#else
	    unsigned int follow_enable  :1; /* . */
	    unsigned int follow_offset  :1; /* . */
	    unsigned int phased_motor   :1; /* . */
	    unsigned int alt_src_dest   :1; /* . */
	    unsigned int user_enable    :1; /* . */
	    unsigned int user_phase     :1; /* . */
	    unsigned int homeing        :1; /* . */
	    unsigned int block_request  :1; /* . */
	    unsigned int decel_abort    :1; /* . */
	    unsigned int move_time_on   :1; /* . */
	    unsigned int open_loop      :1; /* . */
	    unsigned int amp_enabled    :1; /* . */
	    unsigned int x_servo_on     :1; /* . */
	    unsigned int pos_limit_set  :1; /* . */
	    unsigned int neg_limit_set  :1; /* . */
	    unsigned int motor_on       :1; /* . */
#endif
	} Bits;
    } word1;

    union
    {
	epicsUInt16 All;
	struct
	{
#ifdef MSB_First
	    unsigned int alt_src_dest   :1; /* Alternate Source/Destination (7). */
	    unsigned int phased_motor   :1; /* Phased Motor (6). */
	    unsigned int follow_offset  :1; /* Following Offset Mode (5). */
	    unsigned int follow_enable  :1; /* Following Enabled (4). */
	    unsigned int error_trigger  :1; /* . */
	    unsigned int soft_pos_capture  :1;	/* . */
	    unsigned int alt_cmndout_mode  :1;	/* . */
	    unsigned int maxrapid_speed  :1; /* . */
	    unsigned int CS_assignment  :4; /* Coordinate System Number. */
	    unsigned int CD_assignment  :4; /* Coordinate Definition. */
#else
	    unsigned int desired_stop   :1; /* . */
	    unsigned int fore_in_pos    :1; /* Foreground In-Position. */
	    unsigned int na14           :1; /* . */
	    unsigned int assigned_CS    :1; /* Assigned to C.S. */
	    unsigned int CD_assignment  :4; /* Coordinate Definition. */
	    unsigned int CS_assignment  :4; /* Coordinate System Number. */
	    unsigned int maxrapid_speed  :1; /* . */
	    unsigned int alt_cmndout_mode  :1;	/* . */
	    unsigned int soft_pos_capture  :1;	/* . */
	    unsigned int error_trigger  :1; /* . */
#endif
	} Bits;
    } word2;

    union
    {
	epicsUInt16 All;
	struct
	{
#ifdef MSB_First
	    unsigned int assigned_CS    :1; /* Assigned to C.S. (15).*/
	    unsigned int na14           :1; /* N/A (14). */
	    unsigned int fore_in_pos    :1; /* Foreground In-Position (13). */
	    unsigned int desired_stop   :1; /* Stopped on Desired Position Limit (12) . */
	    unsigned int pos_limit_stop :1; /* Stopped on Position Limit (11). */
	    unsigned int home_complete  :1; /* Home Complete (10). */
	    unsigned int phase_search   :1; /* Phase Search/Read Active (9). */
	    unsigned int phase_ref_err  :1; /* Phase Reference Error (8). */
	    unsigned int trigger_move   :1; /* Trigger Move (7). */
	    unsigned int i2_follow_err  :1; /* Integrated Fatal Following Error (6). */
	    unsigned int i2t_amp_fault  :1; /* I2T Aplifier Fault (5). */
	    unsigned int neg_backlash   :1; /* Negative Backlash Direction Flag (4). */
	    unsigned int amp_fault      :1; /* Amplifier Fault (3). */
	    unsigned int err_follow_err :1; /* Fatal Following Error (2). */
	    unsigned int warn_follow_err:1; /* Warning Following Error (1). */
	    unsigned int in_position    :1; /* In position (0). */
#else
	    unsigned int in_position    :1; /* In position. */
	    unsigned int warn_follow_err:1; /* Following error warning. */
	    unsigned int err_follow_err :1; /* Fatal Following error. */
	    unsigned int amp_fault      :1; /* Amplifier Fault. */
	    unsigned int neg_backlash   :1; /* Negative Backlash Direction Flag. */
	    unsigned int i2t_amp_fault  :1; /* I2T Aplifier Fault. */
	    unsigned int i2_follow_err  :1; /* Integrated Fatal Following Error. */
	    unsigned int trigger_move   :1; /* Trigger Move. */
	    unsigned int phase_ref_err  :1; /* Phase Reference Error. */
	    unsigned int home_complete  :1; /* Home Complete. */
	    unsigned int pos_limit_stop :1; /* Stopped on Position Limit. */
#endif
	} Bits;
    } word3;
} MOTOR_STATUS;


typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
	unsigned int cntrl_char     :8;	/* Response control charcter. */
	unsigned int type	    :2;	/* Response type. */
	unsigned int na1	    :5;	/* n/a bit #1-4. */
	unsigned int error	    :1;	/* Error indicator. */
#else
	unsigned int error	    :1;	/* Error indicator. */
	unsigned int na1	    :5;	/* n/a bit #1-4. */
	unsigned int type	    :2;	/* Response type. */
	unsigned int cntrl_char     :8;	/* Response control charcter. */
#endif
    } Bits;                                
} REPLY_STATUS;


/* PMAC DPRAM structure. */
struct pmac_dpram
{
    epicsUInt8 na0[0xE9C];
    epicsUInt8 out_cntrl_wd;	/* Control Word at 0x0E9C. */
    epicsUInt8 na1;
    epicsUInt16 out_cntrl_char;	/* Control Character at 0x0E9E. */
    epicsUInt8 cmndbuff[160];	/* Command Buffer at 0x0EA0. */
    REPLY_STATUS reply_status;	/* Response Buffer Control Characters. */
    epicsUInt8 reply_count;	/* Response Character count - 1. */
    epicsUInt8 na2;
    epicsUInt8 response[256];	/* Response Buffer at 0x0F44. */
};

#endif	/* INCdrvPmach */
