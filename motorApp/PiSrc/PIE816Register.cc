/*
FILENAME...	PIE816Register.cc
USAGE...	Register PI motor device driver shell commands.

Version:	1.1
Modified By:	sullivan
Last Modified:	2007/03/30 20:01:05
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
#include "drvPIE816.h"
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

static const iocshArg * const PIE816SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIE816ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIE816 = {"PIE816Setup",  2, PIE816SetupArgs};
static const iocshFuncDef configPIE816 = {"PIE816Config", 3, PIE816ConfigArgs};

static void setupPIE816CallFunc(const iocshArgBuf *args)
{
    PIE816Setup(args[0].ival, args[1].ival);
}


static void configPIE816CallFunc(const iocshArgBuf *args)
{
    PIE816Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PIE816motorRegister(void)
{
    iocshRegister(&setupPIE816,  setupPIE816CallFunc);
    iocshRegister(&configPIE816, configPIE816CallFunc);
}

epicsExportRegistrar(PIE816motorRegister);

} // extern "C"
