/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

#include <iocsh.h>
#include "drvLinMot.h"
#include "epicsExport.h"

extern "C"
{

// LinMotSetup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};

// LinMotConfig arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"Number of axes", iocshArgInt};

static const iocshArg * const LinMotSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const LinMotConfigArgs[3] = {&configArg0, &configArg1,
    &configArg2};

static const iocshFuncDef setupLinMot  = {"LinMotSetup",  2, LinMotSetupArgs};
static const iocshFuncDef configLinMot = {"LinMotConfig", 3, LinMotConfigArgs};

static void setupLinMotCallFunc(const iocshArgBuf *args)
{
    LinMotSetup(args[0].ival, args[1].ival);
}
static void configLinMotCallFunc(const iocshArgBuf *args)
{
    LinMotConfig(args[0].ival, args[1].sval, args[2].ival);
}

static void LinMotRegister(void)
{
    iocshRegister(&setupLinMot, setupLinMotCallFunc);
    iocshRegister(&configLinMot, configLinMotCallFunc);
}

epicsExportRegistrar(LinMotRegister);

} // extern "C"
