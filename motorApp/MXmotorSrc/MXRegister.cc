/*
FILENAME...	MXRegister.cc
USAGE...	Register MX motor device driver shell commands.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-05-28 14:33:57 $
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
#include "MXmotor.h"
#include "epicsExport.h"

extern "C"
{

// MXmotorSetup
static const iocshArg setupArg0 = {"Max. motor #", iocshArgInt};
static const iocshArg setupArg1 = {"MX data file", iocshArgString};
static const iocshArg setupArg2 = {"Polling rate", iocshArgInt};
static const iocshArg * const mxmotorArgs[3] = {&setupArg0, &setupArg1, &setupArg2};
static const iocshFuncDef setupFuncDef = {"MXmotorSetup", 3, mxmotorArgs};
static void setupCallFunc(const iocshArgBuf *args)
{
    MXmotorSetup(args[0].ival, args[1].sval, args[2].ival);
}


static void MXmotorRegister(void)
{
    iocshRegister(&setupFuncDef, setupCallFunc);
}

epicsExportRegistrar(MXmotorRegister);

} // extern "C"
