/*
FILENAME...	PIC630Register.cc
USAGE...	Register PIC630 motor device driver shell commands.

Version:	1.4
Modified By:	sluiter
Last Modified:	2004/07/16 19:06:58
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
#include "PIC630Register.h"
#include "epicsExport.h"

extern "C"
{

// PIC630 Setup arguments
static const iocshArg setupArg0 = {"Max. controller groups", iocshArgInt};
static const iocshArg setupArg1 = {"Max. axes per group", iocshArgInt};
static const iocshArg setupArg2 = {"Polling rate", iocshArgInt};
// PIC630 Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"Ch 1 current setting", iocshArgInt};
static const iocshArg configArg3 = {"Ch 2 current setting", iocshArgInt};
static const iocshArg configArg4 = {"Ch 3 current setting", iocshArgInt};
static const iocshArg configArg5 = {"Ch 4 current setting", iocshArgInt};
static const iocshArg configArg6 = {"Ch 5 current setting", iocshArgInt};
static const iocshArg configArg7 = {"Ch 6 current setting", iocshArgInt};
static const iocshArg configArg8 = {"Ch 7 current setting", iocshArgInt};
static const iocshArg configArg9 = {"Ch 8 current setting", iocshArgInt};
static const iocshArg configArg10 = {"Ch 9 current setting", iocshArgInt};

static const iocshArg * const PIC630SetupArgs[3]  = {&setupArg0, &setupArg1, &setupArg2};
static const iocshArg * const PIC630ConfigArgs[11] = {&configArg0, &configArg1, &configArg2, &configArg3, &configArg4, &configArg5, &configArg6, &configArg7, &configArg8, &configArg9, &configArg10};

static const iocshFuncDef setupPIC630  = {"PIC630Setup",  3, PIC630SetupArgs};
static const iocshFuncDef configPIC630 = {"PIC630Config", 11, PIC630ConfigArgs};

static void setupPIC630CallFunc(const iocshArgBuf *args)
{
    PIC630Setup(args[0].ival, args[1].ival, args[2].ival);
}
static void configPIC630CallFunc(const iocshArgBuf *args)
{
    PIC630Config(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival, args[9].ival, args[10].ival);
}

static void PIC630Register(void)
{
    iocshRegister(&setupPIC630, setupPIC630CallFunc);
    iocshRegister(&configPIC630, configPIC630CallFunc);
}

epicsExportRegistrar(PIC630Register);

} // extern "C"
