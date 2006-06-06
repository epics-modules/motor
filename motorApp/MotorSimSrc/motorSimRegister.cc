#include <iocsh.h>
#include "drvMotorSim.h"
#include "epicsExport.h"

extern "C" {

static const iocshArg motorSimCreateArg0 = { "Card",          iocshArgInt};
static const iocshArg motorSimCreateArg1 = { "Signal",        iocshArgInt};
static const iocshArg motorSimCreateArg2 = { "High limit",    iocshArgInt};
static const iocshArg motorSimCreateArg3 = { "Low limit",     iocshArgInt};
static const iocshArg motorSimCreateArg4 = { "Home position", iocshArgInt};
static const iocshArg motorSimCreateArg5 = { "Num cards",     iocshArgInt};
static const iocshArg motorSimCreateArg6 = { "Num signals",   iocshArgInt};

static const iocshArg *const motorSimCreateArgs[] = {
  &motorSimCreateArg0,
  &motorSimCreateArg1,
  &motorSimCreateArg2,
  &motorSimCreateArg3,
  &motorSimCreateArg4,
  &motorSimCreateArg5,
  &motorSimCreateArg6,
};
static const iocshFuncDef motorSimCreateDef ={"motorSimCreate",7,motorSimCreateArgs};

static void motorSimCreateCallFunc(const iocshArgBuf *args)
{
  motorSimCreate(args[0].ival, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}

void motorSimRegister(void)
{
  iocshRegister(&motorSimCreateDef, motorSimCreateCallFunc);
}
epicsExportRegistrar(motorSimRegister);

} // extern "C"

