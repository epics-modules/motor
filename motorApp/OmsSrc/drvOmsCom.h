/*
FILENAME...	drvOmsCom.h
USAGE... 	This file contains OMS driver "include" information
		that is common to all OMS models.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-02-08 22:19:03 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 12/18/98
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

#ifndef	INCdrvOmsComh
#define	INCdrvOmsComh 1

#include "motordrvCom.h"

/* Default profile. */

#define OMS_INTERRUPT_TYPE      intVME
#define OMS_ADDRS_TYPE          atVMEA16
#define OMS_INT_VECTOR          180	/* default interrupt vector (64-255) */
#define OMS_INT_LEVEL           5	/* default interrupt level (1-6) */

/* OMS Command strings. */
#define AXIS_STOP       "A? ST\n"
#define GET_IDENT       "WY\n"
#define ERROR_CLEAR     "IC\n"
#define STOP_ALL        "AA SA\n"
#define ALL_POS         "AA RP\n"

/* Global data. */
extern char oms_trans_axis[];

#endif	/* INCdrvOmsComh */
