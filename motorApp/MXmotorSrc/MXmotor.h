/*
FILENAME...	MXmotor.h
USAGE... This file contains "include" information that is specific to
	MX motor device driver support.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-02-14 15:15:27 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 11/12/02
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
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 *
 *
 * Modification Log:
 * -----------------
 */

#ifndef	INCMXmotorh
#define	INCMXmotorh 1

#include "motor.h"
extern "C"
{
#include <mx_record.h>
#include <mx_motor.h>
}

struct MXcontroller
{
    CALLBACK_VALUE callback_flag;
    MX_RECORD *MXmotor_record;
};

RTN_STATUS MXmotorSetup(int, char const *, int);
extern MX_RECORD *MXmotor_record_list;

#endif	/* INCMXmotorh */

