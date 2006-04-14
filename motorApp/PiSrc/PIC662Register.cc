/*
FILENAME...	PiRegister.cc
USAGE...	Register IMS motor device driver shell commands.

Version:	$Revision: 1.1 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2006-04-14 20:34:42 $
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
#include "drvPIC662.h"
#include "epicsExport.h"

extern "C"
{

// Pi 662 Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Pi 662 Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

static const iocshArg * const PIC662SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIC662ConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef  setupPIC662 = {"PIC662Setup",  2, PIC662SetupArgs};
static const iocshFuncDef configPIC662 = {"PIC662Config", 2, PIC662ConfigArgs};

static void setupPIC662CallFunc(const iocshArgBuf *args)
{
    PIC662Setup(args[0].ival, args[1].ival);
}


static void configPIC662CallFunc(const iocshArgBuf *args)
{
   PIC662Config(args[0].ival, args[1].sval);
}


static void PIC662motorRegister(void)
{
    iocshRegister(&setupPIC662,  setupPIC662CallFunc);
    iocshRegister(&configPIC662, configPIC662CallFunc);
}

epicsExportRegistrar(PIC662motorRegister);

} // extern "C"
