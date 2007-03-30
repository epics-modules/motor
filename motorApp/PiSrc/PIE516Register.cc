/*
FILENAME...	PIE516Register.cc
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
#include "drvPIE516.h"
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

static const iocshArg * const PIE516SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIE516ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIE516 = {"PIE516Setup",  2, PIE516SetupArgs};
static const iocshFuncDef configPIE516 = {"PIE516Config", 3, PIE516ConfigArgs};

static void setupPIE516CallFunc(const iocshArgBuf *args)
{
    PIE516Setup(args[0].ival, args[1].ival);
}


static void configPIE516CallFunc(const iocshArgBuf *args)
{
    PIE516Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PIE516motorRegister(void)
{
    iocshRegister(&setupPIE516,  setupPIE516CallFunc);
    iocshRegister(&configPIE516, configPIE516CallFunc);
}

epicsExportRegistrar(PIE516motorRegister);

} // extern "C"
