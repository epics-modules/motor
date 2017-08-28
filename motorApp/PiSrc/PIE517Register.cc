/*
FILENAME...	PIE517Register.cc
USAGE...	Register PI motor device driver shell commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2007-03-30 20:01:05 $
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
#include "drvPIE517.h"
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

static const iocshArg * const PIE517SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIE517ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIE517 = {"PIE517Setup",  2, PIE517SetupArgs};
static const iocshFuncDef configPIE517 = {"PIE517Config", 3, PIE517ConfigArgs};

static void setupPIE517CallFunc(const iocshArgBuf *args)
{
    PIE517Setup(args[0].ival, args[1].ival);
}


static void configPIE517CallFunc(const iocshArgBuf *args)
{
    PIE517Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PIE517motorRegister(void)
{
    iocshRegister(&setupPIE517,  setupPIE517CallFunc);
    iocshRegister(&configPIE517, configPIE517CallFunc);
}

epicsExportRegistrar(PIE517motorRegister);

} // extern "C"
