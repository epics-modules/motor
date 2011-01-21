/*  Version:        $Revision::                                              $  */
/*  Modified By:    $Author::                                                $  */
/*  Last Modified:  $Date::                                                  $  */
/*  HeadURL:        $URL::                                                   $  */

/* This is the EPICS dependent code for the Hytec 8601 Step motor driver.
 * By making this separate file for the EPICS dependent code the driver itself
 * only needs libCom from EPICS for OS-independence (i.e. it is then possible to create
 * module tests without needing to load the entire EPICS libraries!)
 */
 
#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

#include "drvHy8601Asyn.h"

/* Code for iocsh registration */

/* Hy8601Config */


static const iocshArg Hy8601ConfigArg0 = {"cardnum", iocshArgInt};
static const iocshArg Hy8601ConfigArg1 = {"carrier", iocshArgInt};
static const iocshArg Hy8601ConfigArg2 = {"ipslot", iocshArgInt};
static const iocshArg Hy8601ConfigArg3 = {"vector", iocshArgInt};
static const iocshArg Hy8601ConfigArg4 = {"useencoder", iocshArgInt};
static const iocshArg Hy8601ConfigArg5 = {"limitmode", iocshArgInt};

static const iocshArg * const Hy8601ConfigArgs[] =  {&Hy8601ConfigArg0,
                                                       &Hy8601ConfigArg1,
                                                       &Hy8601ConfigArg2,
                                                       &Hy8601ConfigArg3,
                                                       &Hy8601ConfigArg4,
                                                       &Hy8601ConfigArg5};

static const iocshFuncDef configHy8601 = {"Hy8601AsynConfig", 6, Hy8601ConfigArgs};
static void configHy8601CallFunc(const iocshArgBuf *args)
{
    Hy8601Configure(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void Hy8601AsynRegister(void)
{
    iocshRegister(&configHy8601, configHy8601CallFunc);
}

epicsExportRegistrar(Hy8601AsynRegister);


