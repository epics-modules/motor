#include <stdlib.h>

#include "asynMotorDriver.h"


/** All of the arguments are simply passed to
  * the constructor for the asynPortDriver base class. After calling the base class
  * constructor this method sets reasonable default values for all of the parameters
  * defined in asynMotorDriver.h.
  */

static const char *driverName = "asynMotorDriver";

asynMotorDriver::asynMotorDriver(const char *portName, int maxAxes, int numParams,
                                 int interfaceMask, int interruptMask,
                                 int asynFlags, int autoConnect, int priority, int stackSize)

    : asynPortDriver(portName, maxAxes, NUM_MOTOR_DRIVER_PARAMS+numParams,
          interfaceMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask | asynDrvUserMask,
          interruptMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask,
          asynFlags, autoConnect, priority, stackSize)

{
    static const char *functionName = "asynMotorDriver";

    /* Create the base set of motor parameters */
    createParam(motorMoveRelString,                asynParamFloat64,    &motorMoveRel);
    createParam(motorMoveAbsString,                asynParamFloat64,    &motorMoveAbs);
    createParam(motorMoveVelString,                asynParamFloat64,    &motorMoveVel);
    createParam(motorHomeString,                   asynParamFloat64,    &motorHome);
    createParam(motorStopString,                   asynParamInt32,      &motorStop);
    createParam(motorVelocityString,               asynParamFloat64,    &motorVelocity);
    createParam(motorVelBaseString,                asynParamFloat64,    &motorVelBase);
    createParam(motorAccelString,                  asynParamFloat64,    &motorAccel);
    createParam(motorPositionString,               asynParamFloat64,    &motorPosition);
    createParam(motorEncoderPositionString,        asynParamFloat64,    &motorEncoderPosition);
    createParam(motorDeferMovesString,             asynParamInt32,      &motorDeferMoves);
    createParam(motorResolutionString,             asynParamFloat64,    &motorResolution);
    createParam(motorEncRatioString,               asynParamFloat64,    &motorEncRatio);
    createParam(motorPgainString,                  asynParamFloat64,    &motorPgain);
    createParam(motorIgainString,                  asynParamFloat64,    &motorIgain);
    createParam(motorDgainString,                  asynParamFloat64,    &motorDgain);
    createParam(motorHighLimString,                asynParamFloat64,    &motorHighLim);
    createParam(motorLowLimString,                 asynParamFloat64,    &motorLowLim);
    createParam(motorSetClosedLoopString,          asynParamInt32,      &motorSetClosedLoop);
    createParam(motorStatusString,                 asynParamInt32,      &motorStatus);
    createParam(motorUpdateStatusString,           asynParamInt32,      &motorUpdateStatus);
    createParam(motorStatusDirectionString,        asynParamInt32,      &motorStatusDirection);
    createParam(motorStatusDoneString,             asynParamInt32,      &motorStatusDone);
    createParam(motorStatusHighLimitString,        asynParamInt32,      &motorStatusHighLimit);
    createParam(motorStatusAtHomeString,           asynParamInt32,      &motorStatusAtHome);
    createParam(motorStatusSlipString,             asynParamInt32,      &motorStatusSlip);
    createParam(motorStatusPowerOnString,          asynParamInt32,      &motorStatusPowerOn);
    createParam(motorStatusFollowingErrorString,   asynParamInt32,      &motorStatusFollowingError);
    createParam(motorStatusHomeString,             asynParamInt32,      &motorStatusHome);
    createParam(motorStatusHasEncoderString,       asynParamInt32,      &motorStatusHasEncoder);
    createParam(motorStatusProblemString,          asynParamInt32,      &motorStatusProblem);
    createParam(motorStatusMovingString,           asynParamInt32,      &motorStatusMoving);
    createParam(motorStatusGainSupportString,      asynParamInt32,      &motorStatusGainSupport);
    createParam(motorStatusCommsErrorString,       asynParamInt32,      &motorStatusCommsError);
    createParam(motorStatusLowLimitString,         asynParamInt32,      &motorStatusLowLimit);
    createParam(motorStatusHomedString,            asynParamInt32,      &motorStatusHomed);
    
    this->axisStatus = (MotorStatus *)calloc(maxAxes, sizeof(MotorStatus));
    this->axisStatusChanged = (int *)calloc(maxAxes, sizeof(int));
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: constructor complete\n",
        driverName, functionName);

}
// We override the setIntegerParam, setDoubleParam, and callParamCallbacks methods so we can construct 
// the aggregate status structure and do callbacks on it

asynStatus asynMotorDriver::setIntegerParam(int axis, int function, int value)
{
    int mask;
    
    // This assumes the parameters defined above are in the same order as the bits the motor record expects!
	if (function >= motorStatusDirection && 
	    function <= motorStatusHomed) {
	    mask = 1 << (function - motorStatusDirection);
        if (value) this->axisStatus[axis].status |= mask;
        else       this->axisStatus[axis].status &= ~mask;
        this->axisStatusChanged[axis] = 1;
	}
    // Call the base class method
    return asynPortDriver::setIntegerParam(axis, function, value);
}

asynStatus asynMotorDriver::setDoubleParam(int axis, int function, double value)
{
    if (function == motorPosition) {
        axisStatusChanged[axis] = 1;
        this->axisStatus[axis].position = value;
    } else if (function == motorEncoderPosition) {
        axisStatusChanged[axis] = 1;
        this->axisStatus[axis].encoder_posn = value;
    }    
    // Call the base class method
    return asynPortDriver::setDoubleParam(axis, function, value);
}   

asynStatus asynMotorDriver::callParamCallbacks(int axis, int addr)
{
    if (this->axisStatusChanged[axis]) {
        this->axisStatusChanged[axis] = 0;
        doCallbacksGenericPointer((void *)&this->axisStatus[axis], motorStatus, axis);
    }
    return asynPortDriver::callParamCallbacks(axis, addr);
}

asynStatus asynMotorDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int axis;
    int function = pasynUser->reason;
    asynStatus status=asynSuccess;
    double accel;
    static const char *functionName = "writeFloat64";

    status = this->getAddress(pasynUser, &axis);

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(axis, function, value);

    if (function == motorStop) {
        getDoubleParam(axis, motorAccel, &accel);
	    status = this->stopAxis(pasynUser, accel);
    
    } else if (function == motorUpdateStatus) {
        // We don't implement this yet.  Is it needed?
   	    //status = this->forceCallback)(pasynUser);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(axis, axis);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s error, status=%d axis=%d, function=%d, value=%d\n", 
              driverName, functionName, status, axis, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s:: axis=%d, function=%d, value=%d\n", 
              driverName, functionName, axis, function, value);
    return status;
}        
    

asynStatus asynMotorDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int axis;
    int function = pasynUser->reason;
    double baseVelocity, velocity, accel;
    asynStatus status = asynError;
    static const char *functionName = "writeFloat64";

    status = this->getAddress(pasynUser, &axis);

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(axis, function, value);

    getDoubleParam(axis, motorVelBase, &baseVelocity);
    getDoubleParam(axis, motorVelocity, &velocity);
    getDoubleParam(axis, motorAccel, &accel);


    if (function == motorMoveRel) {
	    status = this->moveAxis(pasynUser, value, 1, baseVelocity, velocity, accel);
	
    } else if (function == motorMoveAbs) {
	    status = this->moveAxis(pasynUser, value, 0, baseVelocity, velocity, accel);

    } else if (function == motorMoveVel) {
	    status = this->moveVelocityAxis(pasynUser, baseVelocity, value, accel);

    // Note, the motorHome command happens on the asynFloat64 interface, even though the value (direction) is really integer 
    } else if (function == motorHome) {
	    status = this->homeAxis(pasynUser, baseVelocity, velocity, accel, (value == 0) ? 0 : 1);

    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(axis, axis);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s error, status=%d axis=%d, function=%d, value=%f\n", 
              driverName, functionName, status, axis, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s:: axis=%d, function=%d, value=%f\n", 
              driverName, functionName, axis, function, value);
    return status;
        
}

asynStatus asynMotorDriver::readGenericPointer(asynUser *pasynUser, void *pointer)
{
    static const char *functionName = "readGenericPointer";
    MotorStatus *pStatus = (MotorStatus *)pointer;
    int axis;
    
    getAddress(pasynUser, &axis);
    getIntegerParam(axis, motorStatus, (int *)&pStatus->status);
    getDoubleParam(axis, motorPosition, &pStatus->position);
    getDoubleParam(axis, motorEncoderPosition, &pStatus->encoder_posn);
    getDoubleParam(axis, motorVelocity, &pStatus->velocity);
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
        "%s:%s: MotorStatus = status%d, position=%f, encoder position=%f, velocity=%f\n", 
        driverName, functionName, pStatus->status, pStatus->position, pStatus->encoder_posn, pStatus->velocity);
    return(asynSuccess);
}    

asynStatus asynMotorDriver::moveAxis(asynUser *pasynUser, double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
    static const char *functionName = "moveAxis";

    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: not implemented in this driver\n", 
        driverName, functionName);
    return(asynError);
}

asynStatus asynMotorDriver::moveVelocityAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration)
{
    static const char *functionName = "moveVelocityAxis";

    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: not implemented in this driver\n", 
        driverName, functionName);
    return(asynError);
}

asynStatus asynMotorDriver::homeAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration, int forwards)
{
    static const char *functionName = "homeAxis";

    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: not implemented in this driver\n", 
        driverName, functionName);
    return(asynError);
}


asynStatus asynMotorDriver::stopAxis(asynUser *pasynUser, double acceleration)
{
    static const char *functionName = "stopAxis";

    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: not implemented in this driver\n", 
        driverName, functionName);
    return(asynError);
}

asynStatus asynMotorDriver::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger)
{
    static const char *functionName = "profileMove";

    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: not implemented in this driver\n", 
        driverName, functionName);
    return(asynError);
}

asynStatus asynMotorDriver::triggerProfile(asynUser *pasynUser)
{
    static const char *functionName = "triggerProfile";

    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: not implemented in this driver\n", 
        driverName, functionName);
    return(asynError);
}

