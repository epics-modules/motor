/*
FILENAME...	AerotechRegister.cc
USAGE...	Register Aerotech motor device driver shell commands.

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

// Aerotech non-asyn Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Aerotech non-asyn Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address", iocshArgInt};

static const iocshArg * const SoloistSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const SoloistConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

static const iocshFuncDef setupSoloist  = {"SoloistSetup",  2, SoloistSetupArgs};
static const iocshFuncDef configSoloist = {"SoloistConfig", 2, SoloistConfigArgs};

// Ensemble Asyn Setup arguments
static const iocshArg ensembleAsynSetupArg0 = {"Max. controller count", iocshArgInt};
// Ensemble Asyn Config arguments
static const iocshArg ensembleAsynConfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg ensembleAsynConfigArg1 = {"asyn port name", iocshArgString};
static const iocshArg ensembleAsynConfigArg2 = {"asyn address (GPIB)", iocshArgInt};
static const iocshArg ensembleAsynConfigArg3 = {"Number of Axes", iocshArgInt};
static const iocshArg ensembleAsynConfigArg4 = {"Moving poll rate", iocshArgInt};
static const iocshArg ensembleAsynConfigArg5 = {"Idle poll rate", iocshArgInt};
static const iocshArg a3200AsynConfigArg6    = {"Task number", iocshArgInt};
static const iocshArg a3200AsynConfigArg7    = {"Linear move commands", iocshArgInt};

static const iocshArg * const EnsembleAsynSetupArgs[1] = {&ensembleAsynSetupArg0};
static const iocshArg * const EnsembleAsynConfigArgs[8] = {&ensembleAsynConfigArg0, 
    &ensembleAsynConfigArg1,
    &ensembleAsynConfigArg2,
    &ensembleAsynConfigArg3,
    &ensembleAsynConfigArg4,
    &ensembleAsynConfigArg5,
    &a3200AsynConfigArg6,
    &a3200AsynConfigArg7};

static const iocshFuncDef setupEnsembleAsyn = {"EnsembleAsynSetup", 1, EnsembleAsynSetupArgs};
static const iocshFuncDef configEnsembleAsyn = {"EnsembleAsynConfig", 6, EnsembleAsynConfigArgs};

static const iocshFuncDef setupA3200Asyn = {"A3200AsynSetup", 1, EnsembleAsynSetupArgs};
static const iocshFuncDef configA3200Asyn = {"A3200AsynConfig", 8, EnsembleAsynConfigArgs};

static void setupSoloistCallFunc(const iocshArgBuf *args)
{
    SoloistSetup(args[0].ival, args[1].ival);
}
static void configSoloistCallFunc(const iocshArgBuf *args)
{
    SoloistConfig(args[0].ival, args[1].sval, args[2].ival);
}

static void setupEnsembleAsynCallFunc(const iocshArgBuf *args)
{
    EnsembleAsynSetup(args[0].ival);
}
static void configEnsembleAsynCallFunc(const iocshArgBuf *args)
{
    EnsembleAsynConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void setupA3200AsynCallFunc(const iocshArgBuf *args)
{
    A3200AsynSetup(args[0].ival);
}
static void configA3200AsynCallFunc(const iocshArgBuf *args)
{
    A3200AsynConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival, args[7].ival);
}

static void AerotechRegister(void)
{
    iocshRegister(&setupSoloist, setupSoloistCallFunc);
    iocshRegister(&configSoloist, configSoloistCallFunc);
    iocshRegister(&setupEnsembleAsyn, setupEnsembleAsynCallFunc);
    iocshRegister(&configEnsembleAsyn, configEnsembleAsynCallFunc);
    iocshRegister(&setupA3200Asyn, setupA3200AsynCallFunc);
    iocshRegister(&configA3200Asyn, configA3200AsynCallFunc);
}

epicsExportRegistrar(AerotechRegister);

} // extern "C"

