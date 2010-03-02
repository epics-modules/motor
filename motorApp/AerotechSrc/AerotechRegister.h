/*
FILENAME...	AerotechRegister.h
USAGE... This file contains function prototypes for ACS IOC shell commands.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
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
 * .01	2008/04/10 caw Initial support for Ensemble
 * .02	2009/04/29 hcf Added support for Soloist
 * .03	2009/07/28 cjb Added support for Ensemble asynMotor
 */

#include "motor.h"
#include "motordrvCom.h"
#include "drvEnsembleAsyn.h"
#include "drvEnsemble.h"
#include "drvSoloist.h"


