/*
FILENAME...	AcsRegister.cc
USAGE...	Register ACS motor device driver shell commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-05-27 13:39:40 $
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
#include "AcsRegister.h"
#include "epicsExport.h"

extern "C"
{

// ACS Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"N/A", iocshArgInt};
static const iocshArg setupArg2 = {"Polling rate", iocshArgInt};
// ACS Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"MPF server location", iocshArgInt};
static const iocshArg configArg2 = {"MPF server task name", iocshArgString};

static const iocshArg * const MCB4BSetupArgs[3]  = {&setupArg0, &setupArg1,
    &setupArg2};
static const iocshArg * const MCB4BConfigArgs[3] = {&configArg0, &configArg1,
    &configArg2};

static const iocshFuncDef setupMCB4B  = {"MCB4BSetup",  3, MCB4BSetupArgs};
static const iocshFuncDef configMCB4B = {"MCB4BConfig", 3, MCB4BConfigArgs};

static void setupMCB4BCallFunc(const iocshArgBuf *args)
{
    MCB4BSetup(args[0].ival, args[1].ival, args[2].ival);
}
static void configMCB4BCallFunc(const iocshArgBuf *args)
{
    MCB4BConfig(args[0].ival, args[1].ival, args[2].sval);
}

static void AcsRegister(void)
{
    iocshRegister(&setupMCB4B, setupMCB4BCallFunc);
    iocshRegister(&configMCB4B, configMCB4BCallFunc);
}

epicsExportRegistrar(AcsRegister);

} // extern "C"
