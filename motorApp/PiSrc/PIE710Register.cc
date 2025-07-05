/*
FILENAME...	PIE710Register.cc
USAGE...	Register PI motor device driver shell commands.

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
#include "drvPIE710.h"
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

static const iocshArg * const PIE710SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIE710ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIE710 = {"PIE710Setup",  2, PIE710SetupArgs};
static const iocshFuncDef configPIE710 = {"PIE710Config", 3, PIE710ConfigArgs};

static void setupPIE710CallFunc(const iocshArgBuf *args)
{
    PIE710Setup(args[0].ival, args[1].ival);
}


static void configPIE710CallFunc(const iocshArgBuf *args)
{
    PIE710Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PIE710motorRegister(void)
{
    iocshRegister(&setupPIE710,  setupPIE710CallFunc);
    iocshRegister(&configPIE710, configPIE710CallFunc);
}

epicsExportRegistrar(PIE710motorRegister);

} // extern "C"
