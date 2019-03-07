/*
FILENAME... MicroMoRegister.cc
USAGE...    Register MicroMo MVP 2001 B02 motor controller device driver shell
	    commands.

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
#include "drvMVP2001.h"
#include "epicsExport.h"

extern "C"
{

// MicroMo Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// MicroMo Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};


static const iocshArg * const MicroMoSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const MicroMoConfigArgs[3] = {&configArg0, &configArg1};

static const iocshFuncDef  setupMicroMo = {"MicroMoSetup",  2, MicroMoSetupArgs};
static const iocshFuncDef configMicroMo = {"MicroMoConfig", 2, MicroMoConfigArgs};

static void setupMicroMoCallFunc(const iocshArgBuf *args)
{
    MVP2001Setup(args[0].ival, args[1].ival);
}


static void configMicroMoCallFunc(const iocshArgBuf *args)
{
    MVP2001Config(args[0].ival, args[1].sval);
}


static void MicroMomotorRegister(void)
{
    iocshRegister(&setupMicroMo, setupMicroMoCallFunc);
    iocshRegister(&configMicroMo, configMicroMoCallFunc);
}

epicsExportRegistrar(MicroMomotorRegister);

} // extern "C"
