/*
FILENAME...	ACSTech80Register.cc
USAGE...	Register ACS Tech80 motor device driver shell commands.

Version:	$Revision: 1.2 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2008-05-21 21:18:52 $
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
#include "ACSTech80Register.h"
#include "epicsExport.h"

extern "C"
{

// ACSTech80 Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};

// ACSTech80 Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"Command Mode [BUFfer/CONnect/DIRect]", iocshArgString};

static const iocshArg * const ACSTech80SetupArgs[2] = {&setupArg0, &setupArg1};

static const iocshArg * const ACSTech80ConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

static const iocshFuncDef setupSPiiPlus = {"SPiiPlusSetup",2, ACSTech80SetupArgs};
static const iocshFuncDef configSPiiPlus = {"SPiiPlusConfig", 3, ACSTech80ConfigArgs};

static void setupSPiiPlusCallFunc(const iocshArgBuf *args)
{
    SPiiPlusSetup(args[0].ival, args[1].ival);
}

static void configSPiiPlusCallFunc(const iocshArgBuf *args)
{
    SPiiPlusConfig(args[0].ival, args[1].sval, args[2].sval);
}


static void ACSTech80Register(void)
{

    iocshRegister(&setupSPiiPlus, setupSPiiPlusCallFunc);

    iocshRegister(&configSPiiPlus, configSPiiPlusCallFunc);
}

epicsExportRegistrar(ACSTech80Register);

} // extern "C"
