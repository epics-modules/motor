/*
FILENAME...	ParkerRegister.cc
USAGE...	Register Parker/Compumotor motor device driver shell commands.

Version:	$Revision: 1.2 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2006-08-31 15:42:31 $
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
#include "ParkerRegister.h"
#include "epicsExport.h"

extern "C"
{

// Parker Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};

// Parker Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

  // Parker File Upload Argument */
static const iocshArg upLoadArg0 = {"Controller Card#", iocshArgInt};
static const iocshArg upLoadArg1 = {"Upload File Path", iocshArgString};
static const iocshArg upLoadArg2 = {"Program Name <NULL=Immediate>", iocshArgString};


static const iocshArg * const ParkerSetupArgs[2] = {&setupArg0, &setupArg1};

static const iocshArg * const ParkerConfigArgs[2] = {&configArg0, &configArg1};

  static const iocshArg * const ParkerUpLoadArgs[3] = {&upLoadArg0, &upLoadArg1, &upLoadArg2};

static const iocshFuncDef setupPC6K = {"PC6KSetup",2, ParkerSetupArgs};

static const iocshFuncDef configPC6K = {"PC6KConfig", 2, ParkerConfigArgs};

static const iocshFuncDef upLoadPC6K = {"PC6KUpLoad", 3, ParkerUpLoadArgs};


static void setupPC6KCallFunc(const iocshArgBuf *args)
{
    PC6KSetup(args[0].ival, args[1].ival);
}

static void configPC6KCallFunc(const iocshArgBuf *args)
{
    PC6KConfig(args[0].ival, args[1].sval);
}

static void upLoadPC6KCallFunc(const iocshArgBuf *args)
{
    PC6KUpLoad(args[0].ival, args[1].sval, args[2].sval);
}


static void ParkerRegister(void)
{

    iocshRegister(&setupPC6K, setupPC6KCallFunc);

    iocshRegister(&configPC6K, configPC6KCallFunc);

    iocshRegister(&upLoadPC6K, upLoadPC6KCallFunc);
}

epicsExportRegistrar(ParkerRegister);

} // extern "C"
