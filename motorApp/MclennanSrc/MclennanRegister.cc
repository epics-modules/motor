/*
FILENAME...	MclennanRegister.cc
USAGE...	Register Mclennan motor device driver shell commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-05-28 14:49:52 $
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
#include "MclennanRegister.h"
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
static const iocshArg configArg3 = {"Number of axes", iocshArgInt};

static const iocshArg * const PM304SetupArgs[3]  = {&setupArg0, &setupArg1,
    &setupArg2};
static const iocshArg * const PM304ConfigArgs[4] = {&configArg0, &configArg1,
    &configArg2};

static const iocshFuncDef setupPM304  = {"PM304Setup",  3, PM304SetupArgs};
static const iocshFuncDef configPM304 = {"PM304Config", 4, PM304ConfigArgs};

static void setupPM304CallFunc(const iocshArgBuf *args)
{
    PM304Setup(args[0].ival, args[1].ival, args[2].ival);
}
static void configPM304CallFunc(const iocshArgBuf *args)
{
    PM304Config(args[0].ival, args[1].ival, args[2].sval, args[3].ival);
}

static void MclennanRegister(void)
{
    iocshRegister(&setupPM304, setupPM304CallFunc);
    iocshRegister(&configPM304, configPM304CallFunc);
}

epicsExportRegistrar(MclennanRegister);

} // extern "C"
