/*
FILENAME...	NewportRegister.cc
USAGE...	Register Newport motor device driver shell commands.

Version:	$Revision: 1.11 $
Modified By:	$Author: rivers $
Last Modified:	$Date: 2006-04-19 21:49:48 $
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
// NewportXPSC8 Config arguments
static const iocshArg XPSconfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg XPSconfigArg1 = {"IP", iocshArgString};
static const iocshArg XPSconfigArg2 = {"Port", iocshArgInt};
static const iocshArg XPSconfigArg3 = {"Number of Axes", iocshArgInt};
// NewportXPSC8 NameConfig arguments
static const iocshArg XPSNameconfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg XPSNameconfigArg1 = {"Axis being configured", iocshArgInt};
static const iocshArg XPSNameconfigArg2 = {"Group Number", iocshArgInt};
static const iocshArg XPSNameconfigArg3 = {"Group size", iocshArgInt};
static const iocshArg XPSNameconfigArg4 = {"Axis in group number", iocshArgInt};
static const iocshArg XPSNameconfigArg5 = {"Group Name", iocshArgString};
static const iocshArg XPSNameconfigArg6 = {"Positioner Name", iocshArgString};
// Newport XPS Gathering Test args
static const iocshArg XPSArg0 = {"Element Period*10^4", iocshArgInt};
// XPS tcl execute function
static const iocshArg tclcallArg0 = {"tcl name", iocshArgString};
static const iocshArg tclcallArg1 = {"Task name", iocshArgString};
static const iocshArg tclcallArg2 = {"Function args", iocshArgString};
// XPSSetup arguments
static const iocshArg XPSSetupArg0 = {"Number of XPS controllers", iocshArgInt};
// XPSConfig arguments
static const iocshArg XPSConfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg XPSConfigArg1 = {"IP", iocshArgString};
static const iocshArg XPSConfigArg2 = {"Port", iocshArgInt};
static const iocshArg XPSConfigArg3 = {"Number of Axes", iocshArgInt};
static const iocshArg XPSConfigArg4 = {"Moving poll rate", iocshArgInt};
static const iocshArg XPSConfigArg5 = {"Idle poll rate", iocshArgInt};
// XPSConfigAxis arguments
static const iocshArg XPSConfigAxisArg0 = {"Card number", iocshArgInt};
static const iocshArg XPSConfigAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg XPSConfigAxisArg2 = {"Axis name", iocshArgString};
static const iocshArg XPSConfigAxisArg3 = {"Steps per unit", iocshArgInt};

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

static const iocshArg * const NewportXPSC8NameArgs[7] = {&XPSNameconfigArg0, 
                                                         &XPSNameconfigArg1,
                                                         &XPSNameconfigArg2,
							 &XPSNameconfigArg3,
                                                         &XPSNameconfigArg4,
							 &XPSNameconfigArg5,		
                                                         &XPSNameconfigArg6};
static const iocshArg * const XPSArgs[1] = {&XPSArg0};

static const iocshArg * const tclcallArgs[3] = {&tclcallArg0,
						&tclcallArg1,
						&tclcallArg2};
static const iocshArg * const XPSSetupArgs[1] =  {&XPSSetupArg0};
static const iocshArg * const XPSConfigArgs[6] = {&XPSConfigArg0, 
                                                  &XPSConfigArg1,
                                                  &XPSConfigArg2, 
                                                  &XPSConfigArg2, 
                                                  &XPSConfigArg4, 
                                                  &XPSConfigArg5};
static const iocshArg * const XPSConfigAxisArgs[4] = {&XPSConfigAxisArg0, 
                                                      &XPSConfigAxisArg1,
                                                      &XPSConfigAxisArg2,
                                                      &XPSConfigAxisArg3}; 

static const iocshFuncDef setupMM3000 = {"MM300Setup", 2, NewportSetupArgs};
static const iocshFuncDef setupMM4000 = {"MM4000Setup",2, NewportSetupArgs};
static const iocshFuncDef setupPM500  = {"PM500Setup", 2, NewportSetupArgs};
static const iocshFuncDef setupESP300 = {"ESP300Setup",2, NewportSetupArgs};
static const iocshFuncDef setupXPSC8  = {"XPSC8Setup", 2, NewportXPSC8SetupArgs};
static const iocshFuncDef setupXPS  =   {"XPSSetup",   1, XPSSetupArgs};

static const iocshFuncDef configMM3000 = {"MM3000Config", 3, NewportConfigArgs};
static const iocshFuncDef configMM4000 = {"MM4000Config", 3, NewportConfigArgs};
static const iocshFuncDef configPM500  = {"PM500Config",  3, NewportConfigArgs};
static const iocshFuncDef configESP300 = {"ESP300Config", 3, NewportConfigArgs};
static const iocshFuncDef configXPSC8  = {"XPSC8Config",  4, NewportXPSC8ConfigArgs};
static const iocshFuncDef configXPS    = {"XPSConfig",    6, XPSConfigArgs};
static const iocshFuncDef configXPSAxis= {"XPSConfigAxis",4, XPSConfigAxisArgs};
static const iocshFuncDef nameXPSC8    = {"XPSC8NameConfig",7, NewportXPSC8NameArgs};

static const iocshFuncDef XPSC8GatheringTest = {"xpsgathering",1, XPSArgs};

static const iocshFuncDef TCLRun = {"tclcall",3, tclcallArgs};

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

static void setupXPSCallFunc(const iocshArgBuf *args)
{
    XPSSetup(args[0].ival);
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

static void configXPSCallFunc(const iocshArgBuf *args)
{
    XPSConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival,
              args[4].ival, args[5].ival);
}

static void configXPSAxisCallFunc(const iocshArgBuf *args)
{
    XPSConfigAxis(args[0].ival, args[1].ival, args[2].sval, args[3].ival);
}

static void nameXPSC8CallFunc(const  iocshArgBuf *args)
{
    XPSC8NameConfig(args[0].ival, args[1].ival, args[2].ival, args[3].ival,\
     			args[4].ival, args[5].sval, args[6].sval);
}

static void XPSC8GatheringTestCallFunc(const  iocshArgBuf *args)
{
    xpsgathering(args[0].ival);
}

static void TCLRunCallFunc(const  iocshArgBuf *args)
{
    tclcall(args[0].sval, args[1].sval, args[2].sval);
}


static void NewportRegister(void)
{
    iocshRegister(&setupMM3000, setupMM3000CallFunc);
    iocshRegister(&setupMM4000, setupMM4000CallFunc);
    iocshRegister(&setupPM500,  setupPM500CallFunc);
    iocshRegister(&setupESP300, setupESP300CallFunc);
    iocshRegister(&setupXPSC8,  setupXPSC8CallFunc);
    iocshRegister(&setupXPS,    setupXPSCallFunc);

    iocshRegister(&configMM3000, configMM3000CallFunc);
    iocshRegister(&configMM4000, configMM4000CallFunc);
    iocshRegister(&configPM500,  configPM500CallFunc);
    iocshRegister(&configESP300, configESP300CallFunc);
    iocshRegister(&configXPSC8,  configXPSC8CallFunc);
    iocshRegister(&configXPS,    configXPSCallFunc);
    iocshRegister(&configXPSAxis,configXPSAxisCallFunc);
    iocshRegister(&nameXPSC8,    nameXPSC8CallFunc);
    iocshRegister(&TCLRun,       TCLRunCallFunc);
#ifdef vxWorks
    iocshRegister(&XPSC8GatheringTest, XPSC8GatheringTestCallFunc);

#endif


}

epicsExportRegistrar(NewportRegister);

} // extern "C"
