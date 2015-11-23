/*
FILENAME...	PIC848Register.cc
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
#include "drvPIC848.h"
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

static const iocshArg * const PIC848SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIC848ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIC848 = {"PIC848Setup",  2, PIC848SetupArgs};
static const iocshFuncDef configPIC848 = {"PIC848Config", 3, PIC848ConfigArgs};

static void setupPIC848CallFunc(const iocshArgBuf *args)
{
    PIC848Setup(args[0].ival, args[1].ival);
}


static void configPIC848CallFunc(const iocshArgBuf *args)
{
    PIC848Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PIC848motorRegister(void)
{
    iocshRegister(&setupPIC848,  setupPIC848CallFunc);
    iocshRegister(&configPIC848, configPIC848CallFunc);
}

epicsExportRegistrar(PIC848motorRegister);

} // extern "C"
