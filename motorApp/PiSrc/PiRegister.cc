/*
FILENAME...	PiRegister.cc
USAGE...	Register IMS motor device driver shell commands.

Version:	$Revision: 1.2 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-07-16 19:38:22 $
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
#include "drvPI.h"
#include "epicsExport.h"

extern "C"
{

// Pi Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Pi Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address (GPIB)", iocshArgInt};

static const iocshArg * const PIC844SetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIC844ConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIC844 = {"PIC844Setup",  2, PIC844SetupArgs};
static const iocshFuncDef configPIC844 = {"PIC844Config", 3, PIC844ConfigArgs};

static void setupPIC844CallFunc(const iocshArgBuf *args)
{
    PIC844Setup(args[0].ival, args[1].ival);
}


static void configPIC844CallFunc(const iocshArgBuf *args)
{
    PIC844Config(args[0].ival, args[1].sval, args[2].ival);
}


static void PImotorRegister(void)
{
    iocshRegister(&setupPIC844, setupPIC844CallFunc);
}

epicsExportRegistrar(PImotorRegister);

} // extern "C"
