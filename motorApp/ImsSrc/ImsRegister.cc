/*
FILENAME...	ImsRegister.cc
USAGE...	Register IMS motor device driver shell commands.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-07-16 19:10:15 $
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
#include "drvIM483.h"
#include "epicsExport.h"

extern "C"
{

// Ims Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Ims Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};


static const iocshArg * const IM483SetupArgs[2]  = {&setupArg0,  &setupArg1};
static const iocshArg * const IM483ConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef setupIM483SM = {"IM483SMSetup", 2, IM483SetupArgs};
static const iocshFuncDef setupIM483PL = {"IM483PLSetup", 2, IM483SetupArgs};
static const iocshFuncDef setupMDrive  = {"MDriveSetup",  2, IM483SetupArgs};

static const iocshFuncDef configIM483SM = {"IM483SMConfig", 2, IM483ConfigArgs};
static const iocshFuncDef configIM483PL = {"IM483PLConfig", 2, IM483ConfigArgs};
static const iocshFuncDef configMDrive  = {"MDriveConfig",  2, IM483ConfigArgs};

static void setupSMCallFunc(const iocshArgBuf *args)
{
    IM483SMSetup(args[0].ival, args[1].ival);
}
static void setupPLCallFunc(const iocshArgBuf *args)
{
    IM483PLSetup(args[0].ival, args[1].ival);
}
static void setupMDriveCallFunc(const iocshArgBuf *args)
{
    MDriveSetup(args[0].ival, args[1].ival);
}


static void configSMCallFunc(const iocshArgBuf *args)
{
    IM483SMConfig(args[0].ival, args[1].sval);
}
static void configPLCallFunc(const iocshArgBuf *args)
{
    IM483PLConfig(args[0].ival, args[1].sval);
}
static void configMDriveCallFunc(const iocshArgBuf *args)
{
    MDriveConfig(args[0].ival, args[1].sval);
}


static void IMSmotorRegister(void)
{
    iocshRegister(&setupIM483SM, setupSMCallFunc);
    iocshRegister(&setupIM483PL, setupPLCallFunc);
    iocshRegister(&setupMDrive,  setupMDriveCallFunc);

    iocshRegister(&configIM483SM, configSMCallFunc);
    iocshRegister(&configIM483PL, configPLCallFunc);
    iocshRegister(&configMDrive,  configMDriveCallFunc);
}

epicsExportRegistrar(IMSmotorRegister);

} // extern "C"
