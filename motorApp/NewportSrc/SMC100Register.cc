/*
FILENAME...	SMC100Register.cc
USAGE...	Register SMC100 motor device driver shell commands.

Version:	$Revision: 1.4 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-07-16 19:06:58 $
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
#include "SMC100Register.h"
#include "epicsExport.h"

extern "C"
{

// SMC100 Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// SMC100 Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

static const iocshArg * const SMC100SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const SMC100ConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef setupSMC100  = {"SMC100Setup",  2, SMC100SetupArgs};
static const iocshFuncDef configSMC100 = {"SMC100Config", 2, SMC100ConfigArgs};

static void setupSMC100CallFunc(const iocshArgBuf *args)
{
    SMC100Setup(args[0].ival, args[1].ival);
}
static void configSMC100CallFunc(const iocshArgBuf *args)
{
    SMC100Config(args[0].ival, args[1].sval);
}

static void SMC100Register(void)
{
    iocshRegister(&setupSMC100, setupSMC100CallFunc);
    iocshRegister(&configSMC100, configSMC100CallFunc);
}

epicsExportRegistrar(SMC100Register);

} // extern "C"
