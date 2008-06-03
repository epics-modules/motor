/*
FILENAME...	drvMAXv.h
USAGE...	OMS driver level "include" information that is specific to OMS
		model MAXv.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2008-06-03 18:58:25 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 04/05/04
 *      Current Author: Ron Sluiter
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
 * 01  04-05-04 rls Copied for drvOms58.h
 *  
 */

#ifndef INCdrvMAXvh
#define INCdrvMAXvh 1

#include "drvOmsCom.h"

/*
 * MAXv default profile
 */

#define MAXv_NUM_CARDS           15

#define BUFFER_SIZE	1024

/* MAXv DUAL-PORT MEMORY MAP */


/* Limit Switch Status - Offset = 0x40 */
typedef union
{
    epicsUInt32 All;
    struct
    {
	unsigned int s_minus  :1;
	unsigned int r_minus  :1;
	unsigned int v_minus  :1;
	unsigned int u_minus  :1;
	unsigned int t_minus  :1;
	unsigned int z_minus  :1;
	unsigned int y_minus  :1;
	unsigned int x_minus  :1;
	unsigned int s_plus   :1;
	unsigned int r_plus   :1;
	unsigned int v_plus   :1;
	unsigned int u_plus   :1;
	unsigned int t_plus   :1;
	unsigned int z_plus   :1;
	unsigned int y_plus   :1;
	unsigned int x_plus   :1;
    } Bits;
} LIMIT_SWITCH;

/* Home Switch Status - Offset = 0x44 */
typedef union
{
    epicsUInt32 All;
    struct
    {
	unsigned int home_s	:1;	/* status of S axis */
	unsigned int home_r	:1;	/* status of R axis */
	unsigned int home_v	:1;	/* status of V axis */
	unsigned int home_u	:1;	/* status of U axis */
	unsigned int home_t	:1;	/* status of T axis */
	unsigned int home_z	:1;	/* status of Z axis */
	unsigned int home_y	:1;	/* status of Y axis */
	unsigned int home_x	:1;	/* status of X axis */
    } Bits;
} HOME_SWITCH;

/* Firmware status - Offset = 0x48 */
typedef union
{
    epicsUInt32 All;
    struct
    {
	unsigned int na19		:13;	/* N/A bits 19-31 */
	unsigned int factoryparm_loaded	:1;
	unsigned int altparm_loaded	:1;
	unsigned int defaultparm_loaded	:1;
	unsigned int altprgm_err	:1;
	unsigned int altparm_chksum_err	:1;
	unsigned int prgm_err		:1;
	unsigned int parm_chksum_err	:1;
	unsigned int na10		:2;	/* N/A bits 10-11 */
	unsigned int program_error	:1;
	unsigned int flash_chksum_err	:1;
	unsigned int na3		:5;	/* N/A bits 3-7 */
	unsigned int running		:1;
	unsigned int initalizing	:1;
	unsigned int not_downloaded	:1;
    } Bits;
} FIRMWARE_STATUS;


/* Status#1 - Offset = 0xFC0 */
typedef union
{
    epicsUInt32 All;
    struct
    {
	unsigned int na3		:5;	/* N/A bits 27-31. */
	unsigned int data_avail		:1;
	unsigned int text_response	:1;
	unsigned int cmndError		:1;	/* Command error dectect */
	unsigned int slip		:8;
	unsigned int overtravel 	:8;
	unsigned int done		:8;
    } Bits;
} STATUS1;


/* OMS VME dual port memory map */
struct MAXv_motor
{
    epicsUInt32 cmndPos[8];
    epicsUInt32 encPos[8];
    LIMIT_SWITCH limit_switch;
    HOME_SWITCH  home_switch;
    FIRMWARE_STATUS firmware_status;
    epicsUInt32 direct_cmnd_mbox;
    epicsUInt32 position_req_mbox;
    epicsUInt32 coherent_cmndPos[8];
    epicsUInt32 coherent_encPos[8];
    epicsUInt32 msg_semaphore;
    epicsUInt32 queue_flush_mbox;
    epicsUInt32 gpio;
    epicsUInt32 naA0[19];		/* N/A byte offset 0xA0 - 0xEB. */
    epicsUInt32 flash_pgm_ptr;
    epicsUInt32 outPutIndex;
    epicsUInt32 outGetIndex;
    epicsUInt32 inPutIndex;
    epicsUInt32 inGetIndex;
    epicsUInt8 outBuffer[BUFFER_SIZE];
    epicsUInt8 inBuffer[BUFFER_SIZE];
    epicsUInt8 utility[BUFFER_SIZE];
    epicsUInt32 naD00[176];	/* N/A byte offset 0xD00 - 0xFBF. */
    STATUS1	status1_flag;
    STATUS1	status1_irq_enable;
    epicsUInt32 status2_flag;
    epicsUInt32 status2_irq_enable;
    epicsUInt32 IACK_vector;
    epicsUInt32 config_switch;
    epicsUInt32 AM_register;
    epicsUInt32 naFDC[7];
    epicsUInt32 FIFO_status_cntrl;
    epicsUInt32 FIFO_date;

};

#endif	/* INCdrvMAXvh */
