#include <iocsh.h>
#include <epicsExport.h>

void xps_gathering();

static const iocshArg XPSGatheringArg0 = {"Interelement period", iocshArgInt};
static const iocshArg * const XPSGatheringArgs[1] = {&XPSGatheringArg0};
static const iocshFuncDef XPSGathering = {"XPSGathering", 1, XPSGatheringArgs};
static void XPSGatheringCallFunc(const iocshArgBuf *args)
{
    xps_gathering(args[0].ival);
}
static void XPSGatheringRegister(void)
{
    iocshRegister(&XPSGathering, XPSGatheringCallFunc);
}

epicsExportRegistrar(XPSGatheringRegister);


