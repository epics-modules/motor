/*
FILENAME... MicroMoRegister.cc
USAGE...    Register MicroMo MVP 2001 B02 motor controller device driver shell
	    commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-03-03 20:02:56 $
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
static const iocshArg setupArg1 = {"Max. motor count", iocshArgInt};
static const iocshArg setupArg2 = {"Polling rate", iocshArgInt};
// MicroMo Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"MPF server location", iocshArgInt};
static const iocshArg configArg2 = {"MPF server task name", iocshArgString};


static const iocshArg * const MicroMoSetupArgs[3]  = {&setupArg0, &setupArg1,
						     &setupArg2};
static const iocshArg * const MicroMoConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupMicroMo = {"MicroMoSetup",  3, MicroMoSetupArgs};
static const iocshFuncDef configMicroMo = {"MicroMoConfig", 3, MicroMoConfigArgs};

static void setupMicroMoCallFunc(const iocshArgBuf *args)
{
    MVP2001Setup(args[0].ival, args[1].ival, args[2].ival);
}


static void configMicroMoCallFunc(const iocshArgBuf *args)
{
    MVP2001Config(args[0].ival, args[1].ival, args[2].sval);
}


static void MicroMomotorRegister(void)
{
    iocshRegister(&setupMicroMo, setupMicroMoCallFunc);
}

epicsExportRegistrar(MicroMomotorRegister);

} // extern "C"
