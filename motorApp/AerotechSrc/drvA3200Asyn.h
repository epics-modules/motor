/*
FILENAME...     drvA3200Asyn.h
USAGE... This file contains Aerotech A3200 Asyn driver "include" information.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Corey Bonnell
 *      Date: 11/15/2013
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
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .00  11-15-13 cjb initialized from drvEnsembleAsync.h (Aerotech)
 */
 
#ifndef DRV_MOTOR_A3200_ASYN_H
#define DRV_MOTOR_A3200_ASYN_H

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

        int A3200AsynSetup(int numControllers); /* number of A3200 controllers in system.  */

        int A3200AsynConfig(int card,             /* Controller number */
                const char *portName, /* asyn port name of serial or GPIB port */
                int asynAddress,      /* asyn subaddress for GPIB */
                int numAxes,          /* The number of axes that the driver controls */
                int movingPollPeriod, /* Time to poll (msec) when an axis is in motion */
                int idlePollPeriod);  /* Time to poll (msec) when an axis is idle. 0 for no polling */

#ifdef __cplusplus
}
#endif

#endif

