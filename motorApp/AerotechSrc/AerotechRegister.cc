/*
FILENAME...	AerotechRegister.cc
USAGE...	Register Aerotech motor device driver shell commands.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
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

static const iocshArg * const EnsembleSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const EnsembleConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

static const iocshFuncDef setupEnsemble  = {"EnsembleSetup",  2, EnsembleSetupArgs};
static const iocshFuncDef configEnsemble = {"EnsembleConfig", 2, EnsembleConfigArgs};

static const iocshArg * const SoloistSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const SoloistConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

static const iocshFuncDef setupSoloist  = {"SoloistSetup",  2, SoloistSetupArgs};
static const iocshFuncDef configSoloist = {"SoloistConfig", 2, SoloistConfigArgs};

// Ensemble Asyn Setup arguments
static const iocshArg asynSetupArg0 = {"Max. controller count", iocshArgInt};
// Ensemble Asyn Config arguments
static const iocshArg asynConfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg asynConfigArg1 = {"asyn port name", iocshArgString};
static const iocshArg asynConfigArg2 = {"asyn address (GPIB)", iocshArgInt};
static const iocshArg asynConfigArg3 = {"Number of Axes", iocshArgInt};
static const iocshArg asynConfigArg4 = {"Moving poll rate", iocshArgInt};
static const iocshArg asynConfigArg5 = {"Idle poll rate", iocshArgInt};

static const iocshArg * const EnsembleAsynSetupArgs[2] = {&asynSetupArg0};
static const iocshArg * const EnsembleAsynConfigArgs[6] = {&asynConfigArg0, 
    &asynConfigArg1,
    &asynConfigArg2,
    &asynConfigArg3,
    &asynConfigArg4,
    &asynConfigArg5};

static const iocshFuncDef setupEnsembleAsyn = {"EnsembleAsynSetup",1, EnsembleAsynSetupArgs};
static const iocshFuncDef configEnsembleAsyn = {"EnsembleAsynConfig", 6, EnsembleAsynConfigArgs};

static void setupEnsembleCallFunc(const iocshArgBuf *args)
{
    EnsembleSetup(args[0].ival, args[1].ival);
}
static void configEnsembleCallFunc(const iocshArgBuf *args)
{
    EnsembleConfig(args[0].ival, args[1].sval, args[2].ival);
}

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
    EnsembleAsynConfig(args[0].ival, args[1].sval, args[2].ival, 
                       args[3].ival, args[4].ival, args[5].ival);
}

static void AerotechRegister(void)
{
    iocshRegister(&setupEnsemble, setupEnsembleCallFunc);
    iocshRegister(&configEnsemble, configEnsembleCallFunc);
    iocshRegister(&setupSoloist, setupSoloistCallFunc);
    iocshRegister(&configSoloist, configSoloistCallFunc);
    iocshRegister(&setupEnsembleAsyn, setupEnsembleAsynCallFunc);
    iocshRegister(&configEnsembleAsyn, configEnsembleAsynCallFunc);
}

epicsExportRegistrar(AerotechRegister);

} // extern "C"
