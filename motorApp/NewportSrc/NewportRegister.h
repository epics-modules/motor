/*
FILENAME...	NewportRegister.h
USAGE... This file contains function prototypes for Newport IOC shell commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-05-22 19:56:13 $
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
extern RTN_STATUS MM3000Setup(int, int, int);
extern RTN_STATUS MM4000Setup(int, int, int);
extern RTN_STATUS  PM500Setup(int, int, int);
extern RTN_STATUS ESP300Setup(int, int, int);
extern RTN_STATUS MM3000Config(int, PortType, int, const char *);
extern RTN_STATUS MM4000Config(int, PortType, int, const char *);
extern RTN_STATUS  PM500Config(int, PortType, int, const char *);
extern RTN_STATUS ESP300Config(int, PortType, int, const char *);

