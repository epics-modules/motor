/*
FILENAME...	ThorLabslRegister.cc
USAGE...	Register ThorLabs  motor device driver shell commands.

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
#include "ThorLabsRegister.h"
#include "epicsExport.h"

extern "C"
{

// ThorLabs Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};

// ThorLabs Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

static const iocshArg * const ThorLabsSetupArgs[2] = {&setupArg0, &setupArg1};

static const iocshArg * const ThorLabsConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef setupMDT695 = {"MDT695Setup",2, ThorLabsSetupArgs};

static const iocshFuncDef configMDT695 = {"MDT695Config", 2, ThorLabsConfigArgs};



static void setupMDT695CallFunc(const iocshArgBuf *args)
{
    MDT695Setup(args[0].ival, args[1].ival);
}

static void configMDT695CallFunc(const iocshArgBuf *args)
{
    MDT695Config(args[0].ival, args[1].sval);
}


static void ThorLabsRegister(void)
{

    iocshRegister(&setupMDT695, setupMDT695CallFunc);

    iocshRegister(&configMDT695, configMDT695CallFunc);

}

epicsExportRegistrar(ThorLabsRegister);

} // extern "C"
