/*
FILENAME...	MclennanRegister.cc
USAGE...	Register Mclennan motor device driver shell commands.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-07-16 19:17:22 $
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
#include "drvPM304.h"
#include "epicsExport.h"

extern "C"
{

// PM304Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};

// PM304Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"Number of axes", iocshArgInt};

static const iocshArg * const PM304SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PM304ConfigArgs[3] = {&configArg0, &configArg1,
    &configArg2};

static const iocshFuncDef setupPM304  = {"PM304Setup",  2, PM304SetupArgs};
static const iocshFuncDef configPM304 = {"PM304Config", 3, PM304ConfigArgs};

static void setupPM304CallFunc(const iocshArgBuf *args)
{
    PM304Setup(args[0].ival, args[1].ival);
}
static void configPM304CallFunc(const iocshArgBuf *args)
{
    PM304Config(args[0].ival, args[1].sval, args[2].ival);
}

static void MclennanRegister(void)
{
    iocshRegister(&setupPM304, setupPM304CallFunc);
    iocshRegister(&configPM304, configPM304CallFunc);
}

epicsExportRegistrar(MclennanRegister);

} // extern "C"
