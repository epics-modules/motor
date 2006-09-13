/*
FILENAME...	PIC862Register.cc
USAGE...	Register PI motor device driver shell commands.

Version:	1.1
Modified By:	Ramanathan
Last Modified:	2006/09/04 19:45:52
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
#include "drvPIC862.h"
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

static const iocshArg * const PIC862SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIC862ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIC862 = {"PIC862Setup",  2, PIC862SetupArgs};
static const iocshFuncDef configPIC862 = {"PIC862Config", 3, PIC862ConfigArgs};

static void setupPIC862CallFunc(const iocshArgBuf *args)
{
    PIC862Setup(args[0].ival, args[1].ival);
}


static void configPIC862CallFunc(const iocshArgBuf *args)
{
    PIC862Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PIC862motorRegister(void)
{
    iocshRegister(&setupPIC862,  setupPIC862CallFunc);
    iocshRegister(&configPIC862, configPIC862CallFunc);
}

epicsExportRegistrar(PIC862motorRegister);

} // extern "C"
