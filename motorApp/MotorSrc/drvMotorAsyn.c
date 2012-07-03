/*
 * drvMotorAsyn.c
 * 
 * Motor record common Asyn driver support layer
 *
 * Copyright (C) 2005-6 Peter Denison, Diamond Light Source
 *
 * This software is distributed subject to the EPICS Open Licence, which can
 * be found at http://www.aps.anl.gov/epics/licence/open.php
 * 
 * Notwithstanding the above, explicit permission is granted for APS to 
 * redistribute this software.
 *
 * Derived from ip330 driver from GSE-CARS which was:
 *     Original Authors: Jim Kowalkowski, Mark Rivers, Joe Sullivan, and Marty Kraimer
 *     ********************COPYRIGHT NOTIFICATION******************************
 *     This software was developed under a United States Government license
 *     described on the COPYRIGHT_UniversityOfChicago file included as part
 *     of this distribution.
 *     ************************************************************************
 *
 * Version: $Revision: 1.22 $
 * Modified by: $Author: rivers $
 * Last Modified: $Date: 2009-09-01 14:05:07 $
 *
 * Original Author: Peter Denison
 * Current Author: Peter Denison
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef vxWorks
#include <taskLib.h>
#include <intLib.h>
#endif

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <errlog.h>
#include <epicsMessageQueue.h>
#include <errlog.h>
#include <cantProceed.h> /* !! for callocMustSucceed() */

#include <asynDriver.h>
#include <asynInt32.h>
#include <asynUInt32Digital.h>
#include <asynFloat64.h>
#include <asynFloat64Array.h>
#include <asynGenericPointer.h>
#include <asynDrvUser.h>

#include <drvSup.h>
#include <registryDriverSupport.h>

#include "asynMotorController.h"
#include "motor_interface.h"
#include <limits.h>
#include <epicsExport.h>

/* Message queue size */
#define MAX_MESSAGES 100

#define NMASKBITS (sizeof(int) * CHAR_BIT)
#define BIT_ISSET(bit, n, mask) ( ((bit)/NMASKBITS < (n)) && \
            ((mask)[(bit)/NMASKBITS] & (1 << ((bit) % NMASKBITS))) )
#define BIT_SET(bit, mask, value) do { \
                (mask)[(bit)/NMASKBITS] &= ~(1 << ((bit) % NMASKBITS)); \
                if (value) { \
                    (mask)[(bit)/NMASKBITS] |= (1 << ((bit) % NMASKBITS)); \
                } \
            } while (0);

/* Note, these values must not be used for pasynUser->reason in device support  */
typedef enum {
    /* Parameters - these must match the definitions in motor_interface.h */
    motorPosition = motorAxisPosition,
    motorEncRatio = motorAxisEncoderRatio,
    motorResolution = motorAxisResolution,
    motorPGain = motorAxisPGain,
    motorIGain = motorAxisIGain,
    motorDGain = motorAxisDGain,
    motorHighLimit = motorAxisHighLimit,
    motorLowLimit = motorAxisLowLimit,
    motorSetClosedLoop = motorAxisClosedLoop,
    motorEncoderPosition = motorAxisEncoderPosn,
    motorDeferMoves = motorAxisDeferMoves,
    motorMoveToHome = motorAxisMoveToHome,
    /* Status bits split out */
    motorStatusDirection=motorAxisDirection,
    motorStatusDone, motorStatusHighLimit, motorStatusAtHome,
    motorStatusSlip, motorStatusPowerOn, motorStatusFollowingError,
    motorStatusHome, motorStatusHasEncoder, motorStatusProblem,
    motorStatusMoving, motorStatusGainSupport, motorStatusCommsError,
    motorStatusLowLimit, motorStatusHomed, motorStatusLast,
    /* Not exposed by the driver */
    motorVelocity=100, motorVelBase, motorAccel, 
    /* Commands */
    motorMoveRel, motorMoveAbs, motorMoveVel, motorHome, motorStop,
    /* Status readback */
    motorStatus, motorUpdateStatus
} motorCommand;

typedef struct {
    motorCommand command;
    char *commandString;
} motorCommandStruct;

static motorCommandStruct motorCommands[] = {
    {motorMoveRel,              motorMoveRelString},
    {motorMoveAbs,              motorMoveAbsString},
    {motorMoveVel,              motorMoveVelString},
    {motorHome,                 motorHomeString},
    {motorStop,                 motorStopString},
    {motorVelocity,             motorVelocityString},
    {motorVelBase,              motorVelBaseString},
    {motorAccel,                motorAccelString},
    {motorPosition,             motorPositionString},
    {motorEncoderPosition,      motorEncoderPositionString},
    {motorDeferMoves,           motorDeferMovesString},
    {motorResolution,           motorResolutionString},
    {motorEncRatio,             motorEncoderRatioString},
    {motorPGain,                motorPGainString},
    {motorIGain,                motorIGainString},
    {motorDGain,                motorDGainString},
    {motorHighLimit,            motorHighLimitString},
    {motorLowLimit,             motorLowLimitString},
    {motorSetClosedLoop,        motorClosedLoopString},
    {motorDeferMoves,           motorDeferMovesString},
    {motorMoveToHome,           motorMoveToHomeString},
    {motorStatus,               motorStatusString},
    {motorUpdateStatus,         motorUpdateStatusString},
    {motorStatusDirection,      motorStatusDirectionString}, 
    {motorStatusDone,           motorStatusDoneString},
    {motorStatusHighLimit,      motorStatusHighLimitString},
    {motorStatusAtHome,         motorStatusAtHomeString},
    {motorStatusSlip,           motorStatusSlipString},
    {motorStatusPowerOn,        motorStatusPowerOnString},
    {motorStatusFollowingError, motorStatusFollowingErrorString},
    {motorStatusHome,           motorStatusHomeString},
    {motorStatusHasEncoder,     motorStatusHasEncoderString},
    {motorStatusProblem,        motorStatusProblemString},
    {motorStatusMoving,         motorStatusMovingString},
    {motorStatusGainSupport,    motorStatusGainSupportString},
    {motorStatusCommsError,     motorStatusCommsErrorString},
    {motorStatusLowLimit,       motorStatusLowLimitString},
    {motorStatusHomed,          motorStatusHomedString},
};

typedef enum{typeInt32, typeFloat64, typeFloat64Array} dataType;

struct drvmotorPvt;

typedef struct drvmotorAxisPvt {
    AXIS_HDL axis;
    int num;
    /* Current parameters for move commands */
    double max_velocity;
    double min_velocity;
    double accel;
    /* Received status */
    MotorStatus status;
    struct drvmotorPvt *pPvt;
    asynUser *pasynUser;
} drvmotorAxisPvt;

typedef struct drvmotorPvt {
    char *portName;
    motorAxisDrvSET_t *drvset;
    int card;
    int numAxes;
    drvmotorAxisPvt *axisData;
    /* Housekeeping */
    epicsMutexId lock;
    int rebooting;
    epicsMessageQueueId intMsgQId;
    int messagesSent;
    int messagesFailed;
    /* Asyn interfaces */
    asynInterface common;
    asynInterface int32;
    void *int32InterruptPvt;
    asynInterface uint32digital;
    void *uint32digitalInterruptPvt;
    asynInterface float64;
    void *float64InterruptPvt;
    asynInterface float64Array;
    asynInterface genericPointer;
    void *genericPointerInterruptPvt;
    asynInterface drvUser;
    asynUser *pasynUser;
} drvmotorPvt;

/* These functions are used by the interfaces */
static asynStatus readInt32         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *value);
static asynStatus writeInt32        (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 value);
static asynStatus readUInt32Digital    (void *drvPvt, asynUser *pasynUser,
                                     epicsUInt32 *value, epicsUInt32 mask);
static asynStatus writeUInt32Digital (void *drvPvt, asynUser *pasynUser,
                                     epicsUInt32 value, epicsUInt32 mask);
static asynStatus getBounds         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *low, epicsInt32 *high);
static asynStatus readFloat64       (void *drvPvt, asynUser *pasynUser,
                                     epicsFloat64 *value);
static asynStatus writeFloat64      (void *drvPvt, asynUser *pasynUser,
                                     epicsFloat64 value);
static asynStatus readGenericPointer (void *drvPvt, asynUser *pasynUser,
                                      void *value);
static asynStatus drvUserCreate     (void *drvPvt, asynUser *pasynUser,
                                     const char *drvInfo, 
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType    (void *drvPvt, asynUser *pasynUser,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy    (void *drvPvt, asynUser *pasynUser);

static void report                  (void *drvPvt, FILE *fp, int details);
static asynStatus connect           (void *drvPvt, asynUser *pasynUser);
static asynStatus disconnect        (void *drvPvt, asynUser *pasynUser);


/* These are private functions, not used in any interfaces */
static void intCallback(void *drvPvt, unsigned int num, unsigned int *changed);
static int config      (drvmotorPvt *pPvt);
static int logFunc     (void *userParam,
                        const motorAxisLogMask_t logMask,
                        const char *pFormat, ...);

static asynCommon drvMotorCommon = {
    report,
    connect,
    disconnect
};

static asynInt32 drvMotorInt32 = {
    writeInt32,
    readInt32,
    getBounds
};

static asynUInt32Digital drvMotorUInt32Digital = {
    writeUInt32Digital,
    readUInt32Digital,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

static asynFloat64 drvMotorFloat64 = {
    writeFloat64,
    readFloat64
};

static asynFloat64Array drvMotorFloat64Array = {
    NULL,
    NULL,
    NULL,
    NULL
};

static asynGenericPointer drvMotorGenericPointer = {
    NULL,
    readGenericPointer,
};

static asynDrvUser drvMotorDrvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};

static asynUser *defaultAsynUser;


int drvAsynMotorConfigure(const char *portName, const char *driverName,
              int card, int num_axes)
{
    drvmotorPvt *pPvt;
    drvmotorAxisPvt *pAxis;
    asynStatus status;
    int i;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "drvAsynMotorConfigure");
    pPvt->portName = epicsStrDup(portName);
    pPvt->drvset = (motorAxisDrvSET_t *) registryDriverSupportFind( driverName );
    if (pPvt->drvset == NULL) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't find driver: %s\n", driverName);
    return -1;
    }

    /* Link with higher level routines */
    pPvt->common.interfaceType = asynCommonType;
    pPvt->common.pinterface  = (void *)&drvMotorCommon;
    pPvt->common.drvPvt = pPvt;
    pPvt->int32.interfaceType = asynInt32Type;
    pPvt->int32.pinterface  = (void *)&drvMotorInt32;
    pPvt->int32.drvPvt = pPvt;
    pPvt->uint32digital.interfaceType = asynUInt32DigitalType;
    pPvt->uint32digital.pinterface  = (void *)&drvMotorUInt32Digital;
    pPvt->uint32digital.drvPvt = pPvt;
    pPvt->float64.interfaceType = asynFloat64Type;
    pPvt->float64.pinterface  = (void *)&drvMotorFloat64;
    pPvt->float64.drvPvt = pPvt;
    pPvt->float64Array.interfaceType = asynFloat64ArrayType;
    pPvt->float64Array.pinterface  = (void *)&drvMotorFloat64Array;
    pPvt->float64Array.drvPvt = pPvt;
    pPvt->genericPointer.interfaceType = asynGenericPointerType;
    pPvt->genericPointer.pinterface  = (void *)&drvMotorGenericPointer;
    pPvt->genericPointer.drvPvt = pPvt;
    pPvt->drvUser.interfaceType = asynDrvUserType;
    pPvt->drvUser.pinterface  = (void *)&drvMotorDrvUser;
    pPvt->drvUser.drvPvt = pPvt;

    status = pasynManager->registerPort(portName,
                                        ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                                        1,  /*  autoconnect */
                                        0,  /* medium priority */
                                        0); /* default stack size */
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register port\n");
        return -1;
    }
    status = pasynManager->registerInterface(portName,&pPvt->common);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register common.\n");
        return -1;
    }

    status = pasynInt32Base->initialize(pPvt->portName,&pPvt->int32);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register int32\n");
        return -1;
    }
    pasynManager->registerInterruptSource(portName, &pPvt->int32,
                                          &pPvt->int32InterruptPvt);

    status = pasynUInt32DigitalBase->initialize(pPvt->portName,&pPvt->uint32digital);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register uint32digital\n");
        return -1;
    }
    pasynManager->registerInterruptSource(portName, &pPvt->uint32digital,
                                          &pPvt->uint32digitalInterruptPvt);
    
    status = pasynFloat64Base->initialize(pPvt->portName,&pPvt->float64);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register float64\n");
        return -1;
    }
    pasynManager->registerInterruptSource(portName, &pPvt->float64,
                                          &pPvt->float64InterruptPvt);

    status = pasynFloat64ArrayBase->initialize(pPvt->portName,&pPvt->float64Array);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register float64Array\n");
        return -1;
    }

    status = pasynGenericPointerBase->initialize(pPvt->portName,&pPvt->genericPointer);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register genericPointer\n");
        return -1;
    }
    pasynManager->registerInterruptSource(portName, &pPvt->genericPointer,
                                          &pPvt->genericPointerInterruptPvt);

    status = pasynManager->registerInterface(pPvt->portName,&pPvt->drvUser);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register drvUser\n");
        return -1;
    }
    /* Create asynUser for debugging */
    pPvt->pasynUser = pasynManager->createAsynUser(0, 0);

    /* Connect to device */
    status = pasynManager->connectDevice(pPvt->pasynUser, portName, -1);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure, connectDevice failed\n");
        return -1;
    }

    pPvt->lock = epicsMutexCreate();
    pPvt->card = card;
    config(pPvt);

    pPvt->numAxes = num_axes;

    pPvt->axisData = callocMustSucceed(num_axes, sizeof(drvmotorAxisPvt), "drvAsynMotorConfigure");
    for ( i = 0; i < num_axes; i++) {
        pAxis = &pPvt->axisData[i];
        pAxis->axis = (*pPvt->drvset->open)(card, i, "");
        if (!pAxis->axis) {
            asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR, 
                  "drvAsynMotorConfigure: Failed to open axis %d\n", i);
        }
        pAxis->num = i;
        pAxis->pPvt = pPvt;
            /* Create asynUser for debugging */
            pAxis->pasynUser = pasynManager->createAsynUser(0, 0);
            /* Connect to device */
            status = pasynManager->connectDevice(pAxis->pasynUser, portName, i);
            if (status != asynSuccess) {
                errlogPrintf("drvAsynMotorConfigure, connectDevice failed\n");
                return -1;
            }
        if (pAxis->axis) {
            (*pPvt->drvset->setCallback)(pAxis->axis, intCallback, (void *)pAxis);
                (*pPvt->drvset->setLog)(pAxis->axis, logFunc, pAxis->pasynUser);
        }
        /* All other axis parameters are initialised to zero at allocation */
    }
    /* Create a fallback asynUser for logging, but only the first time */
    if (!defaultAsynUser) {
        defaultAsynUser = pasynManager->createAsynUser(0,0);
    }
    (*pPvt->drvset->setLog)(NULL, logFunc, defaultAsynUser );

    return 0;
}


static int config(drvmotorPvt *pPvt)
{
    /* XXX call the driver config? with pPvt->card */
    return(0);
}

static asynStatus readInt32(void *drvPvt, asynUser *pasynUser, 
                            epicsInt32 *value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;

    pasynManager->getAddr(pasynUser, &channel);
    if (channel >= pPvt->numAxes) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::readInt32 Invalid axis %d", channel);
    return(asynError);
    }

    pAxis = &pPvt->axisData[channel];
    if (!pAxis->axis) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::readInt32 Uninitialised axis %d", pAxis->num);
    return(asynError);
    }

    switch(command) {
        case -1:   /* This is commands we know about but don't want int32 interface for */
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvMotorAsyn::readInt32 invalid command=%d",
                          command);
            return(asynError);
        case motorStatus:
        *value = pAxis->status.status;
        break;
        case motorPosition:
        case motorEncoderPosition:
        default:
        (*pPvt->drvset->getInteger)(pAxis->axis, command, value);
        break;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::readInt32, reason=%d, value=%d\n", 
          command, *value);
    return(asynSuccess);
}

static asynStatus readUInt32Digital(void *drvPvt, asynUser *pasynUser,
                                    epicsUInt32 *value, epicsUInt32 mask)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;

    pasynManager->getAddr(pasynUser, &channel);
    if (channel >= pPvt->numAxes) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "drvMotorAsyn::readInt32 Invalid axis %d", channel);
        return(asynError);
    }

    pAxis = &pPvt->axisData[channel];
    if (!pAxis->axis) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::readInt32 Uninitialised axis %d", pAxis->num);
        return(asynError);
    }

    switch(command) {
        case -1:   /* This is commands we know about but don't want uint32digital interface for */
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvMotorAsyn::readUInt32Digital invalid command=%d",
                          command);
            return(asynError);
        default:
            (*pPvt->drvset->getInteger)(pAxis->axis, command, (epicsInt32*)value);
            break;
    }
    
    *value = *value & mask;
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::readUInt32Digital, reason=%d, value=%d\n", 
          command, *value);
    return(asynSuccess);    
}

static asynStatus readFloat64(void *drvPvt, asynUser *pasynUser,
                              epicsFloat64 *value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;
    asynStatus status = asynSuccess;

    pasynManager->getAddr(pasynUser, &channel);
    if (channel >= pPvt->numAxes) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::readFloat64 Invalid axis %d", channel);
    return(asynError);
    }

    pAxis = &pPvt->axisData[channel];
    if (!pAxis->axis) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::readFloat64 Uninitialised axis %d", pAxis->num);
    return(asynError);
    }

    switch(command) {
         case -1:
             epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                           "drvMotorAsyn::readFloat64 invalid command=%d",
                           command);
             return(asynError);
         case motorVelocity:
         *value = pAxis->max_velocity;
         break;
         case motorVelBase:
         *value = pAxis->min_velocity;
         break;
         case motorAccel:
         *value = pAxis->accel;
         break;
         case motorPosition:
         case motorEncoderPosition:
         default:
         (*pPvt->drvset->getDouble)(pAxis->axis, command, value);
         break;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::readFloat64, reason=%d, value=%f\n", 
          command, *value);
    return(status);
}

static asynStatus getBounds(void *drvPvt, asynUser *pasynUser,
                            epicsInt32 *low, epicsInt32 *high)
{
    *low = 0;
    *high = 65535;
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::getBounds,low=%d, high=%d\n", *low, *high);
    return(asynSuccess);
}

static asynStatus writeInt32(void *drvPvt, asynUser *pasynUser, 
                             epicsInt32 value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;
    asynStatus status=asynSuccess;

    pasynManager->getAddr(pasynUser, &channel);
    if (channel >= pPvt->numAxes) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::writeInt32 Invalid axis %d", channel);
    return(asynError);
    }

    pAxis = &pPvt->axisData[channel];
    if (!pAxis->axis) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::writeInt32 Uninitialised axis %d", pAxis->num);
    return(asynError);
    }

    switch(command) {
        case -1:
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvMotorAsyn::writeInt32 invalid command=%d",
                          command);
            return(asynError);
        case motorStop:
        status = (*pPvt->drvset->stop)(pAxis->axis, pAxis->accel);
        break;
        case motorHome:
        status = (*pPvt->drvset->home)(pAxis->axis, pAxis->min_velocity,
                           pAxis->max_velocity, pAxis->accel,
                           (value == 0) ? 0 : 1);
        break;
        case motorSetClosedLoop:
        status = (*pPvt->drvset->setInteger)(pAxis->axis, motorAxisClosedLoop,
                             value);
        break;
        case motorUpdateStatus:
            if (pPvt->drvset->forceCallback != NULL)
            status = (*pPvt->drvset->forceCallback)(pAxis->axis);
        break;
        default:
        status = (*pPvt->drvset->setInteger)(pAxis->axis, command, value);
        break;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
          "drvMotorAsyn::writeInt32, reason=%d, value=%d\n",
          command, value);
    return(status);
}

static asynStatus writeUInt32Digital(void *drvPvt, asynUser *pasynUser, 
                                     epicsUInt32 value, epicsUInt32 mask)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;
    asynStatus status;
    epicsUInt32 temp = 0;

    pasynManager->getAddr(pasynUser, &channel);
    if (channel >= pPvt->numAxes) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvMotorAsyn::writeInt32 Invalid axis %d", channel);
        return(asynError);
    }

    pAxis = &pPvt->axisData[channel];
    if (!pAxis->axis) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvMotorAsyn::writeInt32 Uninitialised axis %d", pAxis->num);
        return(asynError);
    }

    switch(command) {
        case -1:
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvMotorAsyn::writeUInt32Digital invalid command=%d",
                          command);
            return(asynError);
        default:
            (*pPvt->drvset->getInteger)(pAxis->axis, command, (epicsInt32*)&temp);
            temp &= ~mask;
            temp |= (value & mask);
            status = (*pPvt->drvset->setInteger)(pAxis->axis, command, temp);
            break;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
          "drvMotorAsyn::writeUInt32Digital, reason=%d, value=%d\n",
          command, temp);
    return(status);
}
        
static asynStatus writeFloat64(void *drvPvt, asynUser *pasynUser, 
                               epicsFloat64 value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;
    asynStatus status = asynError;

    pasynManager->getAddr(pasynUser, &channel);
    if (channel >= pPvt->numAxes) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::writeFloat64 Invalid axis %d", channel);
    return(asynError);
    }

    pAxis = &pPvt->axisData[channel];
    if (!pAxis->axis) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::writeFloat64 Uninitialised axis %d", pAxis->num);
    return(asynError);
    }

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
          "drvMotorAsyn::writeFloat64, reason=%d, pasynUser=%p pAxis=%p\n",
          command, pasynUser, pAxis);

    switch(command) {
        case -1:
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvMotorAsyn::writeFloat64 invalid command=%d",
                          command);
            return(asynError);
        case motorMoveRel:
        status = (*pPvt->drvset->move)(pAxis->axis, value, 1, pAxis->min_velocity,
                       pAxis->max_velocity, pAxis->accel);
        break;
        case motorMoveAbs:
        status = (*pPvt->drvset->move)(pAxis->axis, value, 0, pAxis->min_velocity,
                       pAxis->max_velocity, pAxis->accel);
        break;
        case motorMoveVel:
        status = (*pPvt->drvset->velocityMove)(pAxis->axis, pAxis->min_velocity,
                           value, pAxis->accel);
        break;
        case motorHome:
        status = (*pPvt->drvset->home)(pAxis->axis, pAxis->min_velocity,
                           pAxis->max_velocity, pAxis->accel,
                           (value == 0) ? 0 : 1);
        break;
        case motorVelocity:
        pAxis->max_velocity = value;
        status = asynSuccess;
        break;
        case motorVelBase:
        pAxis->min_velocity = value;
        status = asynSuccess;
        break;
        case motorAccel:
        pAxis->accel = value;
        status = asynSuccess;
        break;
        case motorPosition:
        case motorResolution:
        case motorEncRatio:
        case motorPGain:
        case motorIGain:
        case motorDGain:
        case motorHighLimit:
        case motorLowLimit:
        default:
        status = (*pPvt->drvset->setDouble)(pAxis->axis, command, value);
        break;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::writeFloat64, reason=%d, value=%f\n",
          command, value);
    return(status);
}

static asynStatus readGenericPointer(void *drvPvt, asynUser *pasynUser, 
                  void *pValue)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    MotorStatus *value = (MotorStatus *)pValue;

    pasynManager->getAddr(pasynUser, &channel);
    if (channel >= pPvt->numAxes) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::readGenericPointer Invalid axis %d", channel);
    return(asynError);
    }

    pAxis = &pPvt->axisData[channel];
    if (!pAxis->axis) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
              "drvMotorAsyn::readGenericPointer Uninitialised axis %d", pAxis->num);
    return(asynError);
    }

    if (value) {
        value->status = pAxis->status.status;
        (*pPvt->drvset->getDouble)(pAxis->axis, motorPosition,
                       &(value->position));
        (*pPvt->drvset->getDouble)(pAxis->axis, motorEncoderPosition,
                       &(value->encoderPosition));
        (*pPvt->drvset->getDouble)(pAxis->axis, motorVelocity,
                       &(value->velocity));
    }
                             
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::readMotorStatus, [%08x,%f,%f,%f]\n",
          value->status, value->position, value->encoderPosition,
          value->velocity);
    return(asynSuccess);
}

static int logFunc(void *userParam,
                   const motorAxisLogMask_t logMask,
                   const char *pFormat, ...)
{
    va_list     pvar;
    asynUser    *pasynUser = (asynUser *)userParam;

    if (!pasynUser) {
        pasynUser = defaultAsynUser;
    }

    va_start(pvar, pFormat);
    switch(logMask) {
        case motorAxisTraceError:
            pasynTrace->vprint(pasynUser, ASYN_TRACE_ERROR, pFormat, pvar);
            break;
        case motorAxisTraceIODevice:
            pasynTrace->vprint(pasynUser, ASYN_TRACEIO_DEVICE, pFormat, pvar);
            break;
        case motorAxisTraceIOFilter:
            pasynTrace->vprint(pasynUser, ASYN_TRACEIO_FILTER, pFormat, pvar);
            break;
        case motorAxisTraceIODriver:
            pasynTrace->vprint(pasynUser, ASYN_TRACEIO_DRIVER, pFormat, pvar);
            break;
        case motorAxisTraceFlow:
            pasynTrace->vprint(pasynUser, ASYN_TRACE_FLOW, pFormat, pvar);
            break;
        default:
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "drvMotorAsyn:logFunc unknown logMask %d\n", logMask);
    }
    va_end (pvar);
    return(MOTOR_AXIS_OK);
}

static void intCallback(void *axisPvt, unsigned int nChanged,
            unsigned int *changed)
{
    drvmotorAxisPvt *pAxis = (drvmotorAxisPvt *)axisPvt;
    drvmotorPvt *pPvt = pAxis->pPvt;
    int addr;
    unsigned int reason;
    ELLLIST *pclientList;
    interruptNode *pnode;
    int ivalue;
    double dvalue;
    unsigned int i, bit_num;
    epicsInt32 changedmask = 0;

    /* We are called back with an array of things that have changed.
       First put these into a single int32 word for passing up to higher layers
       Note that for now this relies on the order of the changed flags being
       correct */
    for (i = 0; i < nChanged; i++) {
        if (changed[i] >= motorAxisDirection && 
            changed[i] <= motorAxisHomed) {
            bit_num = changed[i] - motorAxisDirection;
            changedmask |= (1 << bit_num);
            (*pPvt->drvset->getInteger)(pAxis->axis, changed[i], &ivalue);
            BIT_SET(bit_num, &(pAxis->status.status), ivalue);
        }
        if (changed[i] == motorPosition) {
            (*pPvt->drvset->getDouble)(pAxis->axis, changed[i], 
                           &(pAxis->status.position));
        }
        if (changed[i] == motorEncoderPosition) {
            (*pPvt->drvset->getDouble)(pAxis->axis, changed[i], 
                           &(pAxis->status.encoderPosition));
        }
        /*    if (changed[i] == motorVelocity) {
            (*pPvt->drvset->getDouble)(pAxis->axis, changed[i], 
                           &(pAxis->status.velocity));
                           }*/
    }

    /* Pass float64 interrupts */
    pasynManager->interruptStart(pPvt->float64InterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
        asynFloat64Interrupt *pfloat64Interrupt = pnode->drvPvt;
        addr = pfloat64Interrupt->addr;
        reason = pfloat64Interrupt->pasynUser->reason;
        if (addr == pAxis->num) {
            for (i = 0; i < nChanged; i++) {
                if (changed[i] == reason) {
                    (*pPvt->drvset->getDouble)(pAxis->axis, changed[i], &dvalue);
                    pfloat64Interrupt->callback(pfloat64Interrupt->userPvt, 
                                pfloat64Interrupt->pasynUser,
                                dvalue);
                }
            }
        }
        pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(pPvt->float64InterruptPvt);

    /* Pass motorStatus interrupts */
    pasynManager->interruptStart(pPvt->genericPointerInterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
        asynGenericPointerInterrupt *pInterrupt = pnode->drvPvt;
        addr = pInterrupt->addr;
        reason = pInterrupt->pasynUser->reason;
        if (addr == pAxis->num) {
            pInterrupt->callback(pInterrupt->userPvt, 
                            pInterrupt->pasynUser,
                            (void *)&pAxis->status);
        }
        pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(pPvt->genericPointerInterruptPvt);

    /* Pass int32 interrupts */
    pasynManager->interruptStart(pPvt->int32InterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
        asynInt32Interrupt *pint32Interrupt = pnode->drvPvt;
        addr = pint32Interrupt->addr;
        reason = pint32Interrupt->pasynUser->reason;
        if (addr == pAxis->num) {
            if ( reason >= motorStatusDirection && 
             reason < motorStatusLast ) {
                if (BIT_ISSET(reason - motorStatusDirection, 1, &changedmask))
                {
                    (*pPvt->drvset->getInteger)(pAxis->axis, reason, &ivalue);
                    pint32Interrupt->callback(pint32Interrupt->userPvt, 
                                  pint32Interrupt->pasynUser,
                                  ivalue);
                }
            }
            /* If we've subscribed to the aggregate status */
            else if (reason == motorStatus) {
                pint32Interrupt->callback(pint32Interrupt->userPvt,
                          pint32Interrupt->pasynUser,
                          pAxis->status.status);
            }
            else {
                (*pPvt->drvset->getInteger)(pAxis->axis, reason, &ivalue);
                          pint32Interrupt->callback(pint32Interrupt->userPvt, 
                          pint32Interrupt->pasynUser,
                          ivalue);
            }
        }
        pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(pPvt->int32InterruptPvt);
}


/*static void rebootCallback(void *drvPvt)*/
/*{*/
/*   drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;*/
    /* Anything special we have to do on reboot */
/*    pPvt->rebooting = 1;*/
/*}*/

/* asynDrvUser routines */
static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
                                const char *drvInfo, 
                                const char **pptypeName, size_t *psize)
{
    int i;
    char *pstring;
    int ncommands = sizeof(motorCommands)/sizeof(motorCommands[0]);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvMotorAsyn::drvUserCreate, drvInfo=%s, pptypeName=%p, psize=%p, pasynUser=%p\n", drvInfo, pptypeName, psize, pasynUser);

    for (i=0; i < ncommands; i++) {
        pstring = motorCommands[i].commandString;
        if (epicsStrCaseCmp(drvInfo, pstring) == 0) {
            break;
        }
    }
    if (i < ncommands) {
    pasynUser->reason = motorCommands[i].command;
    if (pptypeName) {
        *pptypeName = epicsStrDup(pstring);
    }
    if (psize) {
        *psize = sizeof(motorCommands[i].command);
    }
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvMotorAsyn::drvUserCreate, command=%s\n", pstring);
    return(asynSuccess);
    } else {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "drvmotorAsyn::drvUserCreate, unknown command=%s", drvInfo);
    return(asynError);
    }
}
    
static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
                                 const char **pptypeName, size_t *psize)
{
    motorCommand command = pasynUser->reason;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvMotorAsyn::drvUserGetType entered");

    *pptypeName = NULL;
    *psize = 0;
    if (pptypeName)
        *pptypeName = epicsStrDup(motorCommands[command].commandString);
    if (psize) *psize = sizeof(command);
    return(asynSuccess);
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvMotorAsyn::drvUserDestroy, drvPvt=%p, pasynUser=%p\n",
              drvPvt, pasynUser);

    return(asynSuccess);
}

/* asynCommon routines */

/* Report  parameters */
static void report(void *drvPvt, FILE *fp, int details)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    interruptNode *pnode;
    ELLLIST *pclientList;

    fprintf(fp, "Port: %s\n", pPvt->portName);
    if (details >= 1) {
        fprintf(fp, "    messages sent OK=%d; send failed (queue full)=%d\n",
                pPvt->messagesSent, pPvt->messagesFailed);
        /* Report int32 interrupts */
        pasynManager->interruptStart(pPvt->int32InterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynInt32Interrupt *pint32Interrupt = pnode->drvPvt;
            fprintf(fp, "    int32 callback client address=%p, addr=%d, reason=%d\n",
                    pint32Interrupt->callback, pint32Interrupt->addr, 
                    pint32Interrupt->pasynUser->reason);
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->int32InterruptPvt);

        /* Report float64 interrupts */
        pasynManager->interruptStart(pPvt->float64InterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynFloat64Interrupt *pfloat64Interrupt = pnode->drvPvt;
            fprintf(fp, "    float64 callback client address=%p, addr=%d, reason=%d\n",
                    pfloat64Interrupt->callback, pfloat64Interrupt->addr, 
                    pfloat64Interrupt->pasynUser->reason);
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->float64InterruptPvt);

        /* Report motorStatus interrupts */
        pasynManager->interruptStart(pPvt->genericPointerInterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynGenericPointerInterrupt *pInterrupt = pnode->drvPvt;
            fprintf(fp, "    motorStatus callback client address=%p, reason=%d\n",
                    pInterrupt->callback, 
                    pInterrupt->pasynUser->reason); 
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->genericPointerInterruptPvt);
    }
}

/* Connect */
static asynStatus connect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionConnect(pasynUser);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
          "drvMotorAsyn::connect, pasynUser=%p\n", pasynUser);
    return(asynSuccess);
}

/* Disconnect */
static asynStatus disconnect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionDisconnect(pasynUser);
    return(asynSuccess);
}

static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "driverName",iocshArgString};
static const iocshArg initArg2 = { "cardNum",iocshArgInt};
static const iocshArg initArg3 = { "numAxes",iocshArgInt};
static const iocshArg * const initArgs[4] = {&initArg0,
                                             &initArg1,
                                             &initArg2,
                                             &initArg3};
static const iocshFuncDef initFuncDef = {"drvAsynMotorConfigure",4,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    drvAsynMotorConfigure(args[0].sval, args[1].sval, args[2].ival,
                          args[3].ival);
}

void motorRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(motorRegister);
