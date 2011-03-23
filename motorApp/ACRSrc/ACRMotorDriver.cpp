/*
FILENAME...   ACRMotorDriver.cpp
USAGE...      Motor driver support for the Parker ACR series of controllers, including the Aries.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include "asynMotorDriver.h"
#include <epicsExport.h>

// drvInfo strings for extra parameters that the ACR controller supports
#define ACRJerkString           "ACR_JERK"
#define ACRBinaryInString       "ACR_BINARY_IN"
#define ACRBinaryOutString      "ACR_BINARY_OUT"
#define ACRBinaryOutRBVString   "ACR_BINARY_OUT_RBV"

#define CtlY 25

#define ACR_TIMEOUT 1.0
#define MAX_ACR_STRING_SIZE 80

static const char *driverName = "ACRMotorDriver";

static void ACRMotorPollerC(void *drvPvt);

class ACRMotorAxis 
{
public:
    ACRMotorAxis(int axis);
    int axisNumber;
    char axisName[10];
    double pulsesPerUnit;
    int flagsReg;
    int limitsReg;
    int encoderPositionReg;
    int theoryPositionReg;
    double encoderPosition;
    double theoryPosition;
    int currentFlags;
    int currentLimits;
};

class ACRMotorController : asynMotorDriver {
public:
    ACRMotorController(const char *portName, const char *ACRPortName, int numAxes, int movingPollPeriod, int idlePollPeriod);
    
    /* These are the methods that we override from asynMotorDriver */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
    void report(FILE *fp, int level);
    asynStatus moveAxis(asynUser *pasynUser, double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocityAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration);
    asynStatus homeAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stopAxis(asynUser *pasynUser, double acceleration);
    asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
    asynStatus triggerProfile(asynUser *pasynUser);
    
    /* These are the methods that are new to this class */
    asynStatus writeACR();
    asynStatus writeACR(const char *output, double timeout);
    asynStatus writeReadACR();
    asynStatus writeReadACR(const char *output, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);
    void ACRMotorPoller();  // Should be private, but called from non-member function
    
protected:
    int ACRJerk;
#define FIRST_ACR_PARAM ACRJerk
    int ACRBinaryIn;
    int ACRBinaryOut;
    int ACRBinaryOutRBV;
#define LAST_ACR_PARAM ACRBinaryOutRBV

#define NUM_ACR_PARAMS (&LAST_ACR_PARAM - &FIRST_ACR_PARAM + 1)

private:
    ACRMotorAxis* getAxis(asynUser *pasynUser);
    int numAxes;
    ACRMotorAxis** pAxes;
    epicsEventId pollEventId;
    double idlePollPeriod;
    double movingPollPeriod;
    asynUser *pasynUserACR;
    char outString[MAX_ACR_STRING_SIZE];
    char inString[MAX_ACR_STRING_SIZE];
    int binaryIn;
    int binaryOutRBV;
    int binaryInReg;
    int binaryOutReg;
};

ACRMotorController::ACRMotorController(const char *portName, const char *ACRPortName, int numAxes, int movingPollPeriod, int idlePollPeriod)
    :   asynMotorDriver(portName, numAxes, NUM_ACR_PARAMS, 
            asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask, 
            asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
            1, // autoconnect
            0, 0)  // Default priority and stack size
{
    int axis;
    asynStatus status;
    ACRMotorAxis *pAxis;
    static const char *functionName = "ACRMotorController";

    if (numAxes < 1 ) numAxes = 1;
    this->numAxes = numAxes;
    this->idlePollPeriod = idlePollPeriod/1000.;
    this->movingPollPeriod = movingPollPeriod/1000.;
    this->pAxes = (ACRMotorAxis**) calloc(numAxes, sizeof(ACRMotorAxis*));
    this->pollEventId = epicsEventMustCreate(epicsEventEmpty);
    this->binaryInReg  = 4096;
    this->binaryOutReg = 4097;
    
    // Create controller-specific parameters
    createParam(ACRJerkString,         asynParamFloat64,        &this->ACRJerk);
    createParam(ACRBinaryInString,     asynParamUInt32Digital,  &this->ACRBinaryIn);
    createParam(ACRBinaryOutString,    asynParamUInt32Digital,  &this->ACRBinaryOut);
    createParam(ACRBinaryOutRBVString, asynParamUInt32Digital,  &this->ACRBinaryOutRBV);

    /* Connect to ACR controller */
    status = pasynOctetSyncIO->connect(ACRPortName, 0, &this->pasynUserACR, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: cannot connect to ACR controller\n",
            driverName, functionName);
    }
    // Turn off command echoing
    sprintf(this->outString, "BIT1792=0");
    writeACR();
     // Turn off command prompt
    sprintf(this->outString, "BIT1794=1");
    writeACR();
    // Wait a short while so that any responses to the above commands have time to arrive so we can flush
    // them in the next writeReadACR()
    epicsThreadSleep(0.5);
    for (axis=0; axis<numAxes; axis++) {
        pAxis  = new ACRMotorAxis(axis);
        this->pAxes[axis] = pAxis;
        // Get the number of pulses per unit on this axis
        sprintf(this->outString, "%s PPU", pAxis->axisName);
        status = writeReadACR();
        if (status) {
            setIntegerParam(axis, motorStatusProblem, 1);
        } else {
            pAxis->pulsesPerUnit = atof(this->inString);
            // We assume servo motor with encoder for now
            setIntegerParam(axis, motorStatusGainSupport, 1);
            setIntegerParam(axis, motorStatusHasEncoder, 1);
        }
        callParamCallbacks(axis);
    }

    epicsThreadCreate("ACRMotorThread", 
        epicsThreadPriorityLow,
        epicsThreadGetStackSize(epicsThreadStackMedium),
        (EPICSTHREADFUNC)ACRMotorPollerC, (void *)this);
}


/** Configuration command, called directly or from iocsh */
extern "C" int ACRMotorCreateController(const char *portName, const char *ACRPortName, int numAxes, int movingPollPeriod, int idlePollPeriod)
{
    ACRMotorController *pACRMotorController
        = new ACRMotorController(portName, ACRPortName, numAxes, movingPollPeriod, idlePollPeriod);
    pACRMotorController = NULL;
    return(asynSuccess);
}

ACRMotorAxis::ACRMotorAxis(int axisNumber)
    : axisNumber(axisNumber)
{
    sprintf(this->axisName, "AXIS%d", axisNumber);
    this->encoderPositionReg = 12290 + 256*axisNumber;
    this->theoryPositionReg  = 12294 + 256*axisNumber;
    this->limitsReg          = 4600  + axisNumber;
    this->flagsReg           = 4120  + axisNumber;
}

void ACRMotorController::report(FILE *fp, int level)
{
    int axis;
    ACRMotorAxis *pAxis;

    fprintf(fp, "ACR motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
        this->portName, this->numAxes, this->movingPollPeriod, this->idlePollPeriod);

    if (level > 0) {
        fprintf(fp, "  binary input = 0x%x\n", this->binaryIn);
        fprintf(fp, "  binary output readback = 0x%x\n", this->binaryOutRBV);
        for (axis=0; axis<this->numAxes; axis++) {
            pAxis = this->pAxes[axis];
            fprintf(fp, "  axis %d\n"
                        "    pulsesPerUnit = %f\n"
                        "    encoder position=%f\n"
                        "    theory position=%f\n"
                        "    limits=0x%x\n"
                        "    flags=0x%x\n", 
                pAxis->axisNumber, pAxis->pulsesPerUnit, 
                pAxis->encoderPosition, pAxis->theoryPosition,
                pAxis->currentLimits, pAxis->currentFlags);

        }
    }

    // Call the base class method
    asynMotorDriver::report(fp, level);
}

ACRMotorAxis * ACRMotorController::getAxis(asynUser *pasynUser)
{
    int axis;
    ACRMotorAxis *pAxis;
    
    getAddress(pasynUser, &axis);
    pAxis = this->pAxes[axis];
    return(pAxis);
}
    

asynStatus ACRMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    ACRMotorAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "writeInt32";
    
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(pAxis->axisNumber, function, value);
    
    if (function == motorDeferMoves)
    {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: %sing Deferred Move flag on driver %s\n",
            value != 0.0?"Sett":"Clear",
            driverName, functionName, this->portName);
    } 
    else if (function == motorSetClosedLoop)
    {
        sprintf(this->outString, "DRIVE %s %s", value ? "ON":"OFF", pAxis->axisName);
        writeACR();
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: %s setting closed loop=%d on %s\n",
            driverName, functionName, this->portName, value, pAxis->axisName);
    } 
    else 
    {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorDriver::writeInt32(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axisNumber);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}

asynStatus ACRMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    ACRMotorAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "writeFloat64";
    
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(pAxis->axisNumber, function, value);
    
    if (function == motorPosition) 
    {
        sprintf(this->outString, "%s RES %f", pAxis->axisName, value/pAxis->pulsesPerUnit);
        status = writeACR();
        sprintf(this->outString, "%s JOG REN", pAxis->axisName);
        status = writeACR();
        asynPrint(pasynUser, ASYN_TRACE_FLOW, 
            "%s:%s: Set axis %d to position %d\n", 
            driverName, functionName, pAxis->axisNumber, value);
    }
    else if (function == ACRJerk)
    {
        sprintf(this->outString, "%s JOG JRK %f", pAxis->axisName, value);
        status = writeACR();
        
    } else {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorDriver::writeFloat64(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axisNumber);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%f\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%f\n", 
              driverName, functionName, function, value);
    return status;
}

asynStatus ACRMotorController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
    // This function is limited to writing a single bit, because we use the BIT command.
    // It writes to least significant bit that is set in the mask
    int bit, tmask=0x1;
    asynStatus status;
    //static const char *functionName = "writeUInt32Digital";
    
    for (bit=0; bit<32; bit++) {
        if (mask & tmask) break;
        tmask = tmask << 1;
    }
    sprintf(this->outString, "BIT %d=%d", 32+bit, value);
    status = writeACR();

    return(status);
}

asynStatus ACRMotorController::moveAxis(asynUser*pasynUser, double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
    ACRMotorAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveAxis";

    sprintf(this->outString, "%s JOG ACC %f", pAxis->axisName, acceleration/pAxis->pulsesPerUnit);
    writeACR();
    sprintf(this->outString, "%s JOG VEL %f", pAxis->axisName, max_velocity/pAxis->pulsesPerUnit);
    writeACR();
    // Note, the CtlY being send below clears the kill for all axes, in case they had hit a limit, etc.
    if (relative) {
        sprintf(this->outString, "%c:%s JOG INC %f", CtlY, pAxis->axisName, position/pAxis->pulsesPerUnit);
        writeACR();
    } else {
        sprintf(this->outString, "%c:%s JOG ABS %f", CtlY, pAxis->axisName, position/pAxis->pulsesPerUnit);
        writeACR();
    }
    // Wake up the poller task to begin polling quickly
    epicsEventSignal(this->pollEventId);

    setIntegerParam(pAxis->axisNumber, motorStatusDone, 0);
    callParamCallbacks(pAxis->axisNumber);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f\n",
        driverName, functionName, this->portName, pAxis->axisNumber, position, min_velocity, max_velocity, acceleration );
    return asynSuccess;
}

asynStatus ACRMotorController::homeAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration, int forwards)
{
    asynStatus status = asynError;
    ACRMotorAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "homeAxis";

    sprintf(this->outString, "%s JOG ACC %f", pAxis->axisName, acceleration/pAxis->pulsesPerUnit);
    status = writeACR();
    sprintf(this->outString, "%s JOG VEL %f", pAxis->axisName, max_velocity/pAxis->pulsesPerUnit);
    status = writeACR();
    sprintf(this->outString, "%c:%s JOG HOME %d", CtlY, pAxis->axisName, forwards ? 1 : -1);
    status = writeACR();
    // Wake up the poller task to begin polling quickly
    epicsEventSignal(this->pollEventId);

    setIntegerParam(pAxis->axisNumber, motorStatusDone, 0);
    callParamCallbacks(pAxis->axisNumber);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set driver %s, axis %d to home %s, min vel=%f, max_vel=%f, accel=%f\n",
        driverName, functionName, this->portName, pAxis->axisNumber, (forwards?"FORWARDS":"REVERSE"), min_velocity, max_velocity, acceleration);
    return status;
}


asynStatus ACRMotorController::moveVelocityAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration)
{
    asynStatus status = asynError;
    double speed=max_velocity;
    int forwards=1;
    ACRMotorAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveVelocityAxis";

    if (speed < 0) {
        speed = -speed;
        forwards = 0;
    }
    sprintf(this->outString, "%s JOG ACC %f", pAxis->axisName, acceleration/pAxis->pulsesPerUnit);
    status = writeACR();
    sprintf(this->outString, "%s JOG VEL %f", pAxis->axisName, speed/pAxis->pulsesPerUnit);
    status = writeACR();
    sprintf(this->outString, "%c:%s JOG %s", CtlY, pAxis->axisName, forwards ? "FWD" : "REV");
    status = writeACR();
    // Wake up the poller task to begin polling quickly
    epicsEventSignal(this->pollEventId);

    setIntegerParam(pAxis->axisNumber, motorStatusDone, 0);
    callParamCallbacks(pAxis->axisNumber);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set port %s, axis %d move with velocity of %f, accel=%f, forwards=%d\n",
        driverName, functionName, this->portName, pAxis->axisNumber, speed, acceleration, forwards);
    return status;
}

asynStatus ACRMotorController::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger)
{
  return asynError;
}

asynStatus ACRMotorController::triggerProfile(asynUser *pasynUser)
{
  return asynError;
}

asynStatus ACRMotorController::stopAxis(asynUser *pasynUser, double acceleration )
{
    ACRMotorAxis *pAxis = this->getAxis(pasynUser);
    asynStatus status;
    static const char *functionName = "stopAxis";

    sprintf(this->outString, "%s JOG OFF", pAxis->axisName);
    status = writeReadACR();

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set axis %d to stop, status=%d\n",
        driverName, functionName, pAxis->axisNumber, status);
    return status;
}

static void ACRMotorPollerC(void *drvPvt)
{
    ACRMotorController *pController = (ACRMotorController*)drvPvt;
    pController->ACRMotorPoller();
}
    

void ACRMotorController::ACRMotorPoller()
{
    double timeout;
    int i;
    int forcedFastPolls=0;
    int anyMoving;
    int done;
    int driveOn;
    int limit;
    ACRMotorAxis *pAxis;
    int status;
    epicsUInt32 newBits, changedBits;
    asynStatus comStatus;

    timeout = this->idlePollPeriod;
    epicsEventSignal(this->pollEventId);  /* Force on poll at startup */

    while(1) {
        if (timeout != 0.) status = epicsEventWaitWithTimeout(this->pollEventId, timeout);
        else               status = epicsEventWait(this->pollEventId);
        if (status == epicsEventWaitOK) {
            /* We got an event, rather than a timeout.  This is because other software
             * knows that an axis should have changed state (started moving, etc.).
             * Force a minimum number of fast polls, because the controller status
             * might not have changed the first few polls
             */
            forcedFastPolls = 10;
        }
        anyMoving = 0;
        this->lock();
        for (i=0; i<this->numAxes; i++) {
            pAxis = this->pAxes[i];

            // Read the current encoder position
            sprintf(this->outString, "?P%d", pAxis->encoderPositionReg);
            comStatus = this->writeReadACR();
            if (comStatus) goto done;
            pAxis->encoderPosition = atof(this->inString);
            setDoubleParam(i, motorEncoderPosition, pAxis->encoderPosition);

            // Read the current theoretical position
            sprintf(this->outString, "?P%d", pAxis->theoryPositionReg);
            comStatus = this->writeReadACR();
            if (comStatus) goto done;
            pAxis->theoryPosition = atof(this->inString);
            setDoubleParam(i, motorPosition, pAxis->theoryPosition);

            // Read the current flags
            sprintf(this->outString, "?P%d", pAxis->flagsReg);
            comStatus = this->writeReadACR();
            if (comStatus) goto done;
            pAxis->currentFlags = atoi(this->inString);
            done = (pAxis->currentFlags & 0x1000000)?0:1;
            setIntegerParam(i, motorStatusDone, done);
            if (!done) anyMoving = 1;
            
            // Read the current limit status
            sprintf(this->outString, "?P%d", pAxis->limitsReg);
            comStatus = this->writeReadACR();
            if (comStatus) goto done;
            pAxis->currentLimits = atoi(this->inString);
            limit = (pAxis->currentLimits & 0x1)?1:0;
            setIntegerParam(i, motorStatusHighLimit, limit);
            limit = (pAxis->currentLimits & 0x2)?1:0;
            setIntegerParam(i, motorStatusLowLimit, limit);
            limit = (pAxis->currentLimits & 0x4)?1:0;
            setIntegerParam(i, motorStatusAtHome, limit);

            // Read the drive power on status
            sprintf(this->outString, "DRIVE %s", pAxis->axisName);
            comStatus = this->writeReadACR();
            if (comStatus) goto done;
            driveOn = strstr(this->inString, "ON") ? 1:0;
            setIntegerParam(i, motorStatusPowerOn, driveOn);
            setIntegerParam(i, motorStatusProblem, 0);

            done:
            setIntegerParam(i, motorStatusProblem, comStatus?1:0);
            callParamCallbacks(i);
        }
        // Read the binary inputs
        sprintf(this->outString, "?P%d", this->binaryInReg);
        comStatus = this->writeReadACR();
        if (!comStatus) {
            newBits = atoi(this->inString);
            changedBits = newBits ^ this->binaryIn;
            this->binaryIn = newBits;
            setUIntDigitalParam(0, ACRBinaryIn, this->binaryIn, 0xFFFFFFFF);
            setUInt32DigitalInterrupt(0, ACRBinaryIn, changedBits, interruptOnBoth);
        }

        // Read the binary outputs
        sprintf(this->outString, "?P%d", this->binaryOutReg);
        comStatus = this->writeReadACR();
        if (!comStatus) {
            newBits = atoi(this->inString);
            changedBits = newBits ^ this->binaryOutRBV;
            this->binaryOutRBV = newBits;
            setUIntDigitalParam(0, ACRBinaryOutRBV, this->binaryOutRBV, 0xFFFFFFFF);
            setUInt32DigitalInterrupt(0, ACRBinaryOut, changedBits, interruptOnBoth);
        }
        
        callParamCallbacks(0);

        this->unlock();
        if (forcedFastPolls > 0) {
            timeout = this->movingPollPeriod;
            forcedFastPolls--;
        } else if (anyMoving) {
            timeout = this->movingPollPeriod;
        } else {
            timeout = this->idlePollPeriod;
        }
    }
}

asynStatus ACRMotorController::writeACR()
{
    return(this->writeACR(this->outString, ACR_TIMEOUT));
}

asynStatus ACRMotorController::writeACR(const char *output, double timeout)
{
    size_t nwrite;
    asynStatus status;
    asynUser *pasynUser = this->pasynUserACR;
    // const char *functionName="writeACR";
    
    status = pasynOctetSyncIO->write(pasynUser, output,
                strlen(output), timeout, &nwrite);
                                        
    return(status);
}

asynStatus ACRMotorController::writeReadACR()
{
    size_t nread;
    return(this->writeReadACR(this->outString, this->inString, sizeof(this->inString), &nread, ACR_TIMEOUT));
}

asynStatus ACRMotorController::writeReadACR(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
    size_t nwrite;
    asynStatus status;
    int eomReason;
    asynUser *pasynUser = this->pasynUserACR;
    const char *functionName="writeReadACR";
    
    status = pasynOctetSyncIO->writeRead(pasynUser, output,
                strlen(output), input, maxChars, timeout,
                &nwrite, nread, &eomReason);
                                        
    if (status) asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s, status=%d, sent '%s', received '%s'\n",
                    driverName, functionName, status, output, input);

    return(status);
}


/** Code for iocsh registration */
static const iocshArg ACRMotorCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ACRMotorCreateControllerArg1 = {"ACR port name", iocshArgString};
static const iocshArg ACRMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg ACRMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ACRMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const ACRMotorCreateControllerArgs[] =  {&ACRMotorCreateControllerArg0,
                                                          &ACRMotorCreateControllerArg1,
                                                          &ACRMotorCreateControllerArg2,
                                                          &ACRMotorCreateControllerArg3,
                                                          &ACRMotorCreateControllerArg4};
static const iocshFuncDef ACRMotorCreateControllerDef = {"ACRMotorCreateController", 5, ACRMotorCreateControllerArgs};
static void ACRMotorCreateContollerCallFunc(const iocshArgBuf *args)
{
    ACRMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void ACRMotorRegister(void)
{

    iocshRegister(&ACRMotorCreateControllerDef, ACRMotorCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(ACRMotorRegister);
}
