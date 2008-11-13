/*
FILENAME...	AerotechRegister.cc
USAGE...	Register Aerotech motor device driver shell commands.

Version:	1.0
Modified By:	weimer
Last Modified:	2008/04/10 05:52:37 PM
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
#include "AerotechRegister.h"
#include "epicsExport.h"

extern "C"
{

// Aerotech Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Aerotech Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address", iocshArgInt};

static const iocshArg * const EnsembleSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const EnsembleConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

static const iocshFuncDef setupEnsemble  = {"EnsembleSetup",  2, EnsembleSetupArgs};
static const iocshFuncDef configEnsemble = {"EnsembleConfig", 2, EnsembleConfigArgs};

static void setupEnsembleCallFunc(const iocshArgBuf *args)
{
    EnsembleSetup(args[0].ival, args[1].ival);
}
static void configEnsembleCallFunc(const iocshArgBuf *args)
{
    EnsembleConfig(args[0].ival, args[1].sval, args[2].ival);
}

static void AerotechRegister(void)
{
    iocshRegister(&setupEnsemble, setupEnsembleCallFunc);
    iocshRegister(&configEnsemble, configEnsembleCallFunc);
}

epicsExportRegistrar(AerotechRegister);

} // extern "C"
