/*
FILENAME...	NewportRegister.cc
USAGE...	Register Newport motor device driver shell commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-05-22 19:56:13 $
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
#include "NewportRegister.h"
#include "epicsExport.h"

extern "C"
{

// Newport Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"N/A", iocshArgInt};
static const iocshArg setupArg2 = {"Polling rate", iocshArgInt};
// Newport Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"PortType: 0 = GPIB, 1 = RS232", iocshArgInt};
static const iocshArg configArg2 = {"MPF server location", iocshArgInt};
static const iocshArg configArg3 = {"MPF server task name", iocshArgString};


static const iocshArg * const NewportSetupArgs[3] = {&setupArg0, &setupArg1,
    &setupArg2};
static const iocshArg * const NewportConfigArgs[4] = {&configArg0, &configArg1,
    &configArg2, &configArg3};

static const iocshFuncDef setupMM3000 = {"MM300Setup", 3, NewportSetupArgs};
static const iocshFuncDef setupMM4000 = {"MM4000Setup",3, NewportSetupArgs};
static const iocshFuncDef setupPM500  = {"PM500Setup", 3, NewportSetupArgs};
static const iocshFuncDef setupESP300 = {"ESP300Setup",3, NewportSetupArgs};

static const iocshFuncDef configMM3000 = {"MM3000Config", 4, NewportConfigArgs};
static const iocshFuncDef configMM4000 = {"MM4000Config", 4, NewportConfigArgs};
static const iocshFuncDef configPM500  = {"PM500Config",  4, NewportConfigArgs};
static const iocshFuncDef configESP300 = {"ESP300Config", 4, NewportConfigArgs};

static void setupMM3000CallFunc(const iocshArgBuf *args)
{
    MM3000Setup(args[0].ival, args[1].ival, args[2].ival);
}

static void setupMM4000CallFunc(const iocshArgBuf *args)
{
    MM4000Setup(args[0].ival, args[1].ival, args[2].ival);
}

/*
static void setupPM500CallFunc(const iocshArgBuf *args)
{
    PM500Setup(args[0].ival, args[1].ival, args[2].ival);
}
static void setupESP300CallFunc(const iocshArgBuf *args)
{
    ESP300Setup(args[0].ival, args[1].ival, args[2].ival);
}
*/

static void configMM3000CallFunc(const iocshArgBuf *args)
{
    MM3000Config(args[0].ival, (PortType) args[1].ival, args[2].ival, args[3].sval);
}

static void configMM4000CallFunc(const iocshArgBuf *args)
{
    MM4000Config(args[0].ival, (PortType) args[1].ival, args[2].ival, args[3].sval);
}

/*
static void configPM500CallFunc(const iocshArgBuf *args)
{
    PM500Config(args[0].ival, (PortType) args[1].ival, args[2].ival, args[3].sval);
}
static void configESP300CallFunc(const iocshArgBuf *args)
{
    ESP300Config(args[0].ival, (PortType) args[1].ival, args[2].ival, args[3].sval);
}
*/

static void NewportRegister(void)
{
    iocshRegister(&setupMM3000, setupMM3000CallFunc);
    iocshRegister(&setupMM4000, setupMM4000CallFunc);
//    iocshRegister(&setupPM500,  setupPM500CallFunc);
//    iocshRegister(&setupESP300, setupESP300CallFunc);

    iocshRegister(&configMM3000, configMM3000CallFunc);
    iocshRegister(&configMM4000, configMM4000CallFunc);
//    iocshRegister(&configPM500,  configPM500CallFunc);
//    iocshRegister(&configESP300, configESP300CallFunc);
}

epicsExportRegistrar(NewportRegister);

} // extern "C"
