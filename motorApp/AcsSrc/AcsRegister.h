/*
FILENAME...	AcsRegister.h
USAGE... This file contains function prototypes for ACS IOC shell commands.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-07-16 19:06:58 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 05/19/03
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

#include "motor.h"
#include "motordrvCom.h"

/* Function prototypes. */
extern RTN_STATUS MCB4BSetup(int, int);
extern RTN_STATUS MCB4BConfig(int, const char *);

