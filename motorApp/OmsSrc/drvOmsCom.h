/*
FILENAME...	drvOmsCom.h
USAGE... 	This file contains OMS driver "include" information
		that is common to all OMS models.

Version:	$Revision: 1.4 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2008-03-14 20:38:02 $
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

#ifdef __LP64__
typedef long long motorUInt64;
#endif

/* Default profile. */

#define OMS_INTERRUPT_TYPE      intVME
#define OMS_ADDRS_TYPE          atVMEA16
#define OMS_INT_VECTOR          180	/* default interrupt vector (64-255) */
#define OMS_INT_LEVEL           5	/* default interrupt level (1-6) */

/* OMS Command strings. */
#define AXIS_STOP       "ST"
#define GET_IDENT       "WY"
#define ERROR_CLEAR     "IC"
#define STOP_ALL        "AA SA"
#define ALL_POS         "AA RP"

struct encoder_status
{
    char slip_enable;
    char pos_enable;
    char slip_detect;
    char pos_dead;
    char axis_home;
    char unused;
};

#endif	/* INCdrvOmsComh */
