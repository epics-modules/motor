/*
FILENAME...	PIC663Register.cc
USAGE...	Register PI motor device driver shell commands.
*/
/*
 *      Copied from PIC862Register.cc by Jonathan Thompson, Jan 2011
 *
 * Modification Log:
 * -----------------
 * Jan 2011 - All references to 862 changed to 663
 *            Status register definitions changed to match 663
 */

/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <iocsh.h>
#include "motor.h"
#include "drvPIC663.h"
#include "epicsExport.h"

extern "C"
{

// Pi Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Pi Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address (GPIB)", iocshArgInt};

static const iocshArg * const PIC663SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIC663ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIC663 = {"PIC663Setup",  2, PIC663SetupArgs};
static const iocshFuncDef configPIC663 = {"PIC663Config", 3, PIC663ConfigArgs};

static void setupPIC663CallFunc(const iocshArgBuf *args)
{
    PIC663Setup(args[0].ival, args[1].ival);
}


static void configPIC663CallFunc(const iocshArgBuf *args)
{
    PIC663Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PIC663motorRegister(void)
{
    iocshRegister(&setupPIC663,  setupPIC663CallFunc);
    iocshRegister(&configPIC663, configPIC663CallFunc);
}

epicsExportRegistrar(PIC663motorRegister);

} // extern "C"
