/*
FILENAME...	NewportRegister.cc
USAGE...	Register Newport motor device driver shell commands.

Version:	$Revision: 1.6 $
Modified By:	$Author: rivers $
Last Modified:	$Date: 2004-07-28 20:24:01 $
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
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Newport Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address (GPIB)", iocshArgInt};
// NewportXPSC8 Config arguments
static const iocshArg XPSconfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg XPSconfigArg1 = {"IP", iocshArgString};
static const iocshArg XPSconfigArg2 = {"Port", iocshArgInt};
static const iocshArg XPSconfigArg3 = {"Number of Axes", iocshArgInt};
// NewportXPSC8 NameConfig arguments
static const iocshArg XPSNameconfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg XPSNameconfigArg1 = {"Axis being configured", iocshArgInt};
static const iocshArg XPSNameconfigArg2 = {"Group Name", iocshArgString};
static const iocshArg XPSNameconfigArg3 = {"Positioner Name", iocshArgString};

static const iocshArg * const NewportSetupArgs[2] = {&setupArg0, 
                                                     &setupArg1};
static const iocshArg * const NewportXPSC8SetupArgs[2] = {&setupArg0, 
                                                          &setupArg1};
static const iocshArg * const NewportConfigArgs[3] = {&configArg0, 
                                                      &configArg1,
                                                      &configArg2};
static const iocshArg * const NewportXPSC8ConfigArgs[4] = {&XPSconfigArg0, 
                                                           &XPSconfigArg1,
                                                           &XPSconfigArg2, 
                                                           &XPSconfigArg3};

static const iocshArg * const NewportXPSC8NameArgs[4] = {&XPSNameconfigArg0, 
                                                         &XPSNameconfigArg1,
                                                         &XPSNameconfigArg2,
                                                         &XPSNameconfigArg3};
static const iocshFuncDef setupMM3000 = {"MM300Setup", 2, NewportSetupArgs};
static const iocshFuncDef setupMM4000 = {"MM4000Setup",2, NewportSetupArgs};
static const iocshFuncDef setupPM500  = {"PM500Setup", 2, NewportSetupArgs};
static const iocshFuncDef setupESP300 = {"ESP300Setup",2, NewportSetupArgs};
static const iocshFuncDef setupXPSC8  = {"XPSC8Setup", 2, NewportXPSC8SetupArgs};

static const iocshFuncDef configMM3000 = {"MM3000Config", 3, NewportConfigArgs};
static const iocshFuncDef configMM4000 = {"MM4000Config", 3, NewportConfigArgs};
static const iocshFuncDef configPM500  = {"PM500Config",  3, NewportConfigArgs};
static const iocshFuncDef configESP300 = {"ESP300Config", 3, NewportConfigArgs};
static const iocshFuncDef configXPSC8  = {"XPSC8Config",  4, NewportXPSC8ConfigArgs};
static const iocshFuncDef nameXPSC8    = {"XPSC8NameConfig", 4, NewportXPSC8NameArgs};


static void setupMM3000CallFunc(const iocshArgBuf *args)
{
    MM3000Setup(args[0].ival, args[1].ival);
}

static void setupMM4000CallFunc(const iocshArgBuf *args)
{
    MM4000Setup(args[0].ival, args[1].ival);
}

static void setupPM500CallFunc(const iocshArgBuf *args)
{
    PM500Setup(args[0].ival, args[1].ival);
}

static void setupESP300CallFunc(const iocshArgBuf *args)
{
    ESP300Setup(args[0].ival, args[1].ival);
}

static void setupXPSC8CallFunc(const iocshArgBuf *args)
{
    XPSC8Setup(args[0].ival, args[1].ival);
}

static void configMM3000CallFunc(const iocshArgBuf *args)
{
    MM3000Config(args[0].ival, args[1].sval, args[2].ival);
}

static void configMM4000CallFunc(const iocshArgBuf *args)
{
    MM4000Config(args[0].ival, args[1].sval, args[2].ival);
}

static void configPM500CallFunc(const iocshArgBuf *args)
{
    PM500Config(args[0].ival,  args[1].sval, args[2].ival);
}

static void configESP300CallFunc(const iocshArgBuf *args)
{
    ESP300Config(args[0].ival, args[1].sval, args[2].ival);
}

static void configXPSC8CallFunc(const iocshArgBuf *args)
{
    XPSC8Config(args[0].ival, args[1].sval, args[2].ival, args[3].ival);
}

static void nameXPSC8CallFunc(const  iocshArgBuf *args)
{
    XPSC8NameConfig(args[0].ival, args[1].ival, args[2].sval, args[3].sval);
}

static void NewportRegister(void)
{
    iocshRegister(&setupMM3000, setupMM3000CallFunc);
    iocshRegister(&setupMM4000, setupMM4000CallFunc);
    iocshRegister(&setupPM500,  setupPM500CallFunc);
    iocshRegister(&setupESP300, setupESP300CallFunc);
    iocshRegister(&setupXPSC8,  setupXPSC8CallFunc);

    iocshRegister(&configMM3000, configMM3000CallFunc);
    iocshRegister(&configMM4000, configMM4000CallFunc);
    iocshRegister(&configPM500,  configPM500CallFunc);
    iocshRegister(&configESP300, configESP300CallFunc);
    iocshRegister(&configXPSC8,  configXPSC8CallFunc);
    iocshRegister(&nameXPSC8,    nameXPSC8CallFunc);
}

epicsExportRegistrar(NewportRegister);

} // extern "C"
