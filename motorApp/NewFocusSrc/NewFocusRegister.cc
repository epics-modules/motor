/*
FILENAME...	NewFocusRegister.cc
USAGE...	Register NewFocus motor device driver shell commands.

Version:	1.6
Modified By:	rivers
Last Modified:	2004/07/28 20:24:01
*/

/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

#include <iocsh.h>
#include "NewFocusRegister.h"
#include "epicsExport.h"

extern "C"
{

// NewFocus Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Max. drivers per controller count", iocshArgInt};
static const iocshArg setupArg2 = {"Polling rate", iocshArgInt};
// NewFocus Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address (GPIB)", iocshArgInt};

static const iocshArg * const NewFocusSetupArgs[3] = {&setupArg0, 
						      &setupArg1,
						      &setupArg2};
static const iocshArg * const NewFocusConfigArgs[3] = {&configArg0, 
                                                      &configArg1,
                                                      &configArg2};

static const iocshFuncDef setupPMNC8750 = {"PMNC8750Setup", 3, NewFocusSetupArgs};

static const iocshFuncDef configPMNC8750 = {"PMNC8750Config", 3, NewFocusConfigArgs};


static void setupPMNC8750CallFunc(const iocshArgBuf *args)
{
  PMNC8750Setup(args[0].ival, args[1].ival, args[2].ival);
}

static void configPMNC8750CallFunc(const iocshArgBuf *args)
{
    PMNC8750Config(args[0].ival, args[1].sval, args[2].ival);
}

static void NewFocusRegister(void)
{
    iocshRegister(&setupPMNC8750, setupPMNC8750CallFunc);

    iocshRegister(&configPMNC8750, configPMNC8750CallFunc);
}

epicsExportRegistrar(NewFocusRegister);

} // extern "C"
