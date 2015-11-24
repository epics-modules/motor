/*
FILENAME...     omsMAXvEncFunc.cpp
USAGE...        Pro-Dex OMS MAXv encoder asyn motor support

*/

/*
 *  Created on: 10/2011
 *      Author: eden
 */

#include <string.h>
#include <stdio.h>

#include "omsMAXvEncFunc.h"

static const char *driverName = "omsMAXvEncFuncDriver";

#ifdef __GNUG__
    #ifdef      DEBUG
        #define Debug(l, f, args...) {if (l <= motorMAXvEncFuncdebug) \
                                  errlogPrintf(f, ## args);}
    #else
        #define Debug(l, f, args...)
    #endif
#else
    #define Debug
#endif
volatile int motorMAXvEncFuncdebug = 0;
extern "C" {epicsExportAddress(int, motorMAXvEncFuncdebug);}

/* define additional Parameters to use special encoder functions and auxiliary encoders of the MAXv */
#define NUM_ADDITIONALPARAMS 18
#define motorEncoderFunctionString          "ENCODER_FUNCTION"
#define motorAuxEncoderPositionString       "AUX_ENC_POSITION"
#define motorEncoderRawPosString            "RAW_ENC_POSITION"

omsMAXvEncFunc::omsMAXvEncFunc(const char* portName, int numAxes, int cardNo,
                const char* initString, int priority, int stackSize)
    : omsMAXv(portName, numAxes, cardNo, initString, priority, stackSize, NUM_ADDITIONALPARAMS)
{
    initialize();
}

omsMAXvEncFunc::omsMAXvEncFunc(const char* portName, int numAxes, int cardNo, const char* initString, int priority,
                int stackSize, unsigned int vmeAddr, int vector, int intlevel, const char* addressType)
    : omsMAXv(portName, numAxes, cardNo, initString, priority, stackSize, vmeAddr,
            vector, intlevel, addressType, NUM_ADDITIONALPARAMS)
{
    initialize();
}

void omsMAXvEncFunc::initialize()
{
    const char* functionName = "initialize";

    Debug(5, "omsMAXvEncFunc::initialize: start initialize\n" );

    int encIndex = numAxes;
    if (encIndex > MAXENCFUNC) encIndex = MAXENCFUNC;

    /* auxiliary encoders */
    for (int i=0; i < encIndex; ++i){
        if (createParam(i, motorEncoderFunctionString, asynParamInt32, &encFuncIndex[i]) != asynSuccess)
                errlogPrintf("%s:%s:%s: unable to create param motorEncoderFunctionString, index %d\n",
                              driverName, functionName, portName, i);
        if (createParam(i, motorEncoderRawPosString, asynParamFloat64, &encRawPosIndex[i]) != asynSuccess)
                errlogPrintf("%s:%s:%s: unable to create param motorEncoderRawPosString, index %d\n",
                              driverName, functionName, portName, i);
    }
    createParam(0, motorAuxEncoderPositionString, asynParamFloat64, &encPosIndex[0]);
    createParam(1, motorAuxEncoderPositionString, asynParamFloat64, &encPosIndex[1]);
    Debug(3, "omsMAXvEncFunc::initialize: auxiliary encoder 0 position index %d\n", encPosIndex[0] );
    Debug(3, "omsMAXvEncFunc::initialize: auxiliary encoder 1 position index %d\n", encPosIndex[1] );
    for (int i=0; i < encIndex; ++i) Debug(3, "omsMAXvEncFunc::initialize: encFuncIndex %d => %d\n", i, encFuncIndex[i] );
    lock();
    for (int i=0; i < encIndex; ++i){
        setIntegerParam(i, encFuncIndex[i], 0);
        setDoubleParam(i, encRawPosIndex[i], 0.0);
    }
    setDoubleParam(0, encPosIndex[0], 0.0);
    setDoubleParam(1, encPosIndex[1], 0.0);
    for (int i=0; i < encIndex; ++i) {
        callParamCallbacks(i, i);
    }
    unlock();
}

asynStatus omsMAXvEncFunc::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    omsBaseAxis *pAxis = (omsBaseAxis *)this->getAxis(pasynUser);
    static const char *functionName = "writeInt32";

    if ((pAxis->getAxis() < MAXENCFUNC) && (function == encFuncIndex[pAxis->getAxis()])) {
        Debug(5, "omsMAXvEncFunc::writeInt32: set average axis %d with axis: %d\n", pAxis->getAxis(), value );
        if ((value >= 0) && (value < MAXENCFUNC)) averageChannel [pAxis->getAxis()] = value;
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);
    }
    else {
        status=omsMAXv::writeInt32(pasynUser, value);
    }
    return status;
}

/**
 * overrides the base class to add encoder functions and auxiliary encoders
 * for now we assume the function is average and averageChannel contains the
 * number of the other encoder to compute the average.
 */
asynStatus omsMAXvEncFunc::getEncoderPositions(epicsInt32 encPosArr[OMS_MAX_AXES])
{
//    const char* functionName = "getEncoderPositions";
    asynStatus status = asynSuccess;
    double position;

    omsBaseController::getEncoderPositions(encPosArr);

    for (int i=0; i < OMS_MAX_AXES; ++i) {
        if ((i < MAXENCFUNC) && (averageChannel[i] != i) && (averageChannel[i] > 0) && (averageChannel[i] < OMS_MAX_AXES)){
            position = (double) encPosArr[i];
            encPosArr[i] = (int) (((double)(encPosArr[averageChannel[i]]) + position)/2.0);
            lock();
            setDoubleParam(i, encRawPosIndex[i], position);
            setDoubleParam(averageChannel[i], encRawPosIndex[averageChannel[i]], (double)encPosArr[averageChannel[i]]);
            unlock();
            Debug(9, "omsMAXvEncFunc::getEncPos: axis %d other: %d, old: %g, new: %d, other: %d \n", i, averageChannel[i], position, encPosArr[i], encPosArr[averageChannel[i]] );
        }
    }
    return status;
}

extern "C" int omsMAXvEncFuncConfig(
           int cardNo,                /* card no, starts with 0*/
           const char *portName,      /* MAXv Motor Asyn Port name */
           int numAxes,               /* Number of axes this controller supports */
           int movingPollPeriod,      /* Time to poll (msec) when an axis is in motion */
           int idlePollPeriod,        /* Time to poll (msec) when an axis is idle. 0 for no polling */
           const char *initString)    /* Init String sent to card */
{
    omsMAXvEncFunc *pController = new omsMAXvEncFunc(portName, numAxes, cardNo, initString, 0, 0);
    pController->startPoller((double)movingPollPeriod, (double)idlePollPeriod, 10);
    return 0;
}

/*
 * extended MAXv configuration, which may be used instead of omsMAXvEncFuncConfig,
 * if more details need to be specified.
 * omsMAXvEncFuncConfig2 does not need and ignores omsMAXvSetup
 */
extern "C" int omsMAXvEncFuncConfig2(
           int slotNo,                /* VME slot no of MAXv card*/
           const char* addr_type,     /* VME address type; "A16", "A24" or "A32" */
           unsigned int addrs,        /* Board Address */
           unsigned int vector,       /* Interrupt Vector: noninterrupting(0), (64-255) */
           int int_level,             /* interrupt level (1-6) */
           const char *portName,      /* MAXv Motor Asyn Port name */
           int numAxes,               /* Number of axes this controller supports */
           int priority,              /* priority of PollerTask (0 => epicsThreadPriorityMedium)*/
           int stackSize,             /* stackSize of PollerTask (0 => epicsThreadStackMedium)  */
           int movingPollPeriod,      /* Time to poll (msec) when an axis is in motion */
           int idlePollPeriod,        /* Time to poll (msec) when an axis is idle. 0 for no polling */
           const char *initString)    /* Init String sent to card */
{
    omsMAXvEncFunc *pController = new omsMAXvEncFunc(portName, numAxes, slotNo, initString, priority,
                                                           stackSize, addrs, vector, int_level, addr_type);
    pController->startPoller((double)movingPollPeriod, (double)idlePollPeriod, 10);
    return 0;
}

/* Code for iocsh registration */
/* omsMAXvEncFuncConfig */
static const iocshArg configArg0 = {"number of card", iocshArgInt};
static const iocshArg configArg1 = {"asyn motor port name", iocshArgString};
static const iocshArg configArg2 = {"number of axes", iocshArgInt};
static const iocshArg configArg3 = {"moving poll rate", iocshArgInt};
static const iocshArg configArg4 = {"idle poll rate", iocshArgInt};
static const iocshArg configArg5 = {"initstring", iocshArgString};
static const iocshArg * const configArgs[6] = {&configArg0, &configArg1, &configArg2,
                                               &configArg3, &configArg4, &configArg5 };
static const iocshFuncDef configMAXv = {"omsMAXvEncFuncConfig", 6, configArgs};
static void configMAXvCallFunc(const iocshArgBuf *args)
{
    omsMAXvEncFuncConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval);
}

/* omsMAXvEncFuncConfig2 */
static const iocshArg config2Arg0 = {"Slot number", iocshArgInt};
static const iocshArg config2Arg1 = {"Address type: A16,A24,A32", iocshArgString};
static const iocshArg config2Arg2 = {"Board Address on 4K (0x1000) boundary", iocshArgInt};
static const iocshArg config2Arg3 = {"Interrupt Vector: noninterrupting(0), (64-255)", iocshArgInt};
static const iocshArg config2Arg4 = {"Interrupt level (1-6)", iocshArgInt};
static const iocshArg config2Arg5 = {"Asyn motor port name", iocshArgString};
static const iocshArg config2Arg6 = {"Number of axes", iocshArgInt};
static const iocshArg config2Arg7 = {"Task priority: 0 => medium", iocshArgInt};
static const iocshArg config2Arg8 = {"Stack size: 0 => medium", iocshArgInt};
static const iocshArg config2Arg9 = {"Moving poll rate", iocshArgInt};
static const iocshArg config2Arg10 = {"Idle poll rate", iocshArgInt};
static const iocshArg config2Arg11 = {"Initstring", iocshArgString};
static const iocshArg * const config2Args[12] = {&config2Arg0, &config2Arg1, &config2Arg2, &config2Arg3, &config2Arg4,
        &config2Arg5, &config2Arg6, &config2Arg7, &config2Arg8, &config2Arg9, &config2Arg10, &config2Arg11};
static const iocshFuncDef config2MAXv = {"omsMAXvEncFuncConfig2", 12, config2Args};
static void config2MAXvCallFunc(const iocshArgBuf *args)
{
    omsMAXvEncFuncConfig2(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval,
                   args[6].ival, args[7].ival, args[8].ival, args[9].ival, args[10].ival, args[11].sval);
}

static void omsMAXvEncFuncAsynRegister(void)
{
    iocshRegister(&configMAXv, configMAXvCallFunc);
    iocshRegister(&config2MAXv, config2MAXvCallFunc);
}

epicsExportRegistrar(omsMAXvEncFuncAsynRegister);

