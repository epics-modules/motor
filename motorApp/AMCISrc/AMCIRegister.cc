/*
FILENAME...	AMCIRegister.cc
USAGE...	Register AMCI motor device driver shell commands.

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
#include "AMCIRegister.h"
#include "epicsExport.h"

extern "C"
{

// AMCI ANG1 Setup arguments
static const iocshArg setupArg0 = {"asyn port name", iocshArgString};
static const iocshArg setupArg1 = {"Input port name", iocshArgString};
static const iocshArg setupArg2 = {"Output port name", iocshArgString};
static const iocshArg setupArg3 = {"Number of axes", iocshArgInt};
static const iocshArg setupArg4 = {"Polling rate moving", iocshArgInt};
static const iocshArg setupArg5 = {"Polling rate idle", iocshArgInt};

static const iocshArg * const ANG1SetupArgs[6]  = {&setupArg0, &setupArg1, &setupArg2, &setupArg3, &setupArg4, &setupArg5};

static const iocshFuncDef setupANG1  = {"ANG1Setup",  6, ANG1SetupArgs};

static void setupANG1CallFunc(const iocshArgBuf *args)
{
    ANG1Setup(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival, args[5].ival);
}

static void AMCIRegister(void)
{
    iocshRegister(&setupANG1, setupANG1CallFunc);
}

epicsExportRegistrar(AMCIRegister);

} // extern "C"
