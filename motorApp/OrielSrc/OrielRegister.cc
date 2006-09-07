/*
FILENAME...	OrielRegister.cc
USAGE...	Register Oriel Encoder/Motor Mike motor device driver shell commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2006-09-07 20:18:09 $
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
#include "OrielRegister.h"
#include "epicsExport.h"

extern "C"
{

// Oriel Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};

// Oriel Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

static const iocshArg * const OrielSetupArgs[2] = {&setupArg0, &setupArg1};

static const iocshArg * const OrielConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef setupEMC18011 = {"EMC18011Setup",2, OrielSetupArgs};

static const iocshFuncDef configEMC18011 = {"EMC18011Config", 2, OrielConfigArgs};



static void setupEMC18011CallFunc(const iocshArgBuf *args)
{
    EMC18011Setup(args[0].ival, args[1].ival);
}

static void configEMC18011CallFunc(const iocshArgBuf *args)
{
    EMC18011Config(args[0].ival, args[1].sval);
}


static void OrielRegister(void)
{

    iocshRegister(&setupEMC18011, setupEMC18011CallFunc);

    iocshRegister(&configEMC18011, configEMC18011CallFunc);

}

epicsExportRegistrar(OrielRegister);

} // extern "C"
