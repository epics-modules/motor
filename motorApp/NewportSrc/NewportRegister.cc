/*
FILENAME...	NewportRegister.cc
USAGE...	Register Newport motor device driver shell commands.

Version:	$Revision: 1.13 $
Modified By:	$Author: rivers $
Last Modified:	$Date: 2006-06-15 19:02:59 $
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
#include "tclCall.h"
#include "epicsExport.h"

extern "C"
{

// Newport Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Newport Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address (GPIB)", iocshArgInt};
// Newport Asyn Setup arguments
static const iocshArg asynSetupArg0 = {"Max. controller count", iocshArgInt};
// Newport Asyn Config arguments
static const iocshArg asynConfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg asynConfigArg1 = {"asyn port name", iocshArgString};
static const iocshArg asynConfigArg2 = {"asyn address (GPIB)", iocshArgInt};
static const iocshArg asynConfigArg3 = {"Number of Axes", iocshArgInt};
static const iocshArg asynConfigArg4 = {"Moving poll rate", iocshArgInt};
static const iocshArg asynConfigArg5 = {"Idle poll rate", iocshArgInt};

static const iocshArg * const NewportSetupArgs[2] = {&setupArg0, 
                                                     &setupArg1};
static const iocshArg * const NewportAsynSetupArgs[2] = {&asynSetupArg0};
static const iocshArg * const NewportConfigArgs[3] = {&configArg0, 
                                                      &configArg1,
                                                      &configArg2};
static const iocshArg * const NewportAsynConfigArgs[6] = {&asynConfigArg0, 
                                                          &asynConfigArg1,
                                                          &asynConfigArg2,
                                                          &asynConfigArg3,
                                                          &asynConfigArg4,
                                                          &asynConfigArg5};

static const iocshFuncDef setupMM3000 = {"MM300Setup", 2, NewportSetupArgs};
static const iocshFuncDef setupMM4000 = {"MM4000Setup",2, NewportSetupArgs};
static const iocshFuncDef setupMM4000Asyn = {"MM4000AsynSetup",1, NewportAsynSetupArgs};
static const iocshFuncDef setupPM500  = {"PM500Setup", 2, NewportSetupArgs};
static const iocshFuncDef setupESP300 = {"ESP300Setup",2, NewportSetupArgs};

static const iocshFuncDef configMM3000 = {"MM3000Config", 3, NewportConfigArgs};
static const iocshFuncDef configMM4000 = {"MM4000Config", 3, NewportConfigArgs};
static const iocshFuncDef configMM4000Asyn = {"MM4000AsynConfig", 6, NewportAsynConfigArgs};
static const iocshFuncDef configPM500  = {"PM500Config",  3, NewportConfigArgs};
static const iocshFuncDef configESP300 = {"ESP300Config", 3, NewportConfigArgs};

static void setupMM3000CallFunc(const iocshArgBuf *args)
{
    MM3000Setup(args[0].ival, args[1].ival);
}

static void setupMM4000CallFunc(const iocshArgBuf *args)
{
    MM4000Setup(args[0].ival, args[1].ival);
}

static void setupMM4000AsynCallFunc(const iocshArgBuf *args)
{
    MM4000AsynSetup(args[0].ival);
}

static void setupPM500CallFunc(const iocshArgBuf *args)
{
    PM500Setup(args[0].ival, args[1].ival);
}

static void setupESP300CallFunc(const iocshArgBuf *args)
{
    ESP300Setup(args[0].ival, args[1].ival);
}

static void configMM3000CallFunc(const iocshArgBuf *args)
{
    MM3000Config(args[0].ival, args[1].sval, args[2].ival);
}

static void configMM4000CallFunc(const iocshArgBuf *args)
{
    MM4000Config(args[0].ival, args[1].sval, args[2].ival);
}

static void configMM4000AsynCallFunc(const iocshArgBuf *args)
{
    MM4000AsynConfig(args[0].ival, args[1].sval, args[2].ival, 
                 args[3].ival, args[4].ival, args[5].ival);
}

static void configPM500CallFunc(const iocshArgBuf *args)
{
    PM500Config(args[0].ival,  args[1].sval, args[2].ival);
}

static void configESP300CallFunc(const iocshArgBuf *args)
{
    ESP300Config(args[0].ival, args[1].sval, args[2].ival);
}


static void NewportRegister(void)
{
    iocshRegister(&setupMM3000, setupMM3000CallFunc);
    iocshRegister(&setupMM4000, setupMM4000CallFunc);
    iocshRegister(&setupMM4000Asyn, setupMM4000AsynCallFunc);
    iocshRegister(&setupPM500,  setupPM500CallFunc);
    iocshRegister(&setupESP300, setupESP300CallFunc);

    iocshRegister(&configMM3000, configMM3000CallFunc);
    iocshRegister(&configMM4000, configMM4000CallFunc);
    iocshRegister(&configMM4000Asyn, configMM4000AsynCallFunc);
    iocshRegister(&configPM500,  configPM500CallFunc);
    iocshRegister(&configESP300, configESP300CallFunc);
}

epicsExportRegistrar(NewportRegister);

} // extern "C"
