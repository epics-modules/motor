#include "asynMotorDriver.h"

/** All of the arguments are simply passed to
  * the constructor for the asynPortDriver base class. After calling the base class
  * constructor this method sets reasonable default values for all of the parameters
  * defined in asynMotorDriver.h.
  */

static const char *driverName = "asynMotorDriver";

asynMotorDriver::asynMotorDriver(const char *portName, int maxAddr, int numParams,
                                 int interfaceMask, int interruptMask,
                                 int asynFlags, int autoConnect, int priority, int stackSize)

    : asynPortDriver(portName, maxAddr, NUM_MOTOR_DRIVER_PARAMS+numParams,
          interfaceMask | asynInt32Mask | asynFloat64Mask,
          interruptMask | asynInt32Mask | asynFloat64Mask,
          asynFlags, autoConnect, priority, stackSize)

{
    static const char *functionName = "asynMotorDriver";

    /* Create the base set of motor parameters */
    addParam(motorMoveRelString,                &motorMoveRel);
    addParam(motorMoveAbsString,                &motorMoveAbs);
    addParam(motorMoveVelString,                &motorMoveVel);
    addParam(motorHomeString,                   &motorHome);
    addParam(motorStopString,                   &motorStop);
    addParam(motorVelocityString,               &motorVelocity);
    addParam(motorVelBaseString,                &motorVelBase);
    addParam(motorAccelString,                  &motorAccel);
    addParam(motorPositionString,               &motorPosition);
    addParam(motorEncoderPositionString,        &motorEncoderPosition);
    addParam(motorDeferMovesString,             &motorDeferMoves);
    addParam(motorResolutionString,             &motorResolution);
    addParam(motorEncRatioString,               &motorEncRatio);
    addParam(motorPgainString,                  &motorPgain);
    addParam(motorIgainString,                  &motorIgain);
    addParam(motorDgainString,                  &motorDgain);
    addParam(motorHighLimString,                &motorHighLim);
    addParam(motorLowLimString,                 &motorLowLim);
    addParam(motorSetClosedLoopString,          &motorSetClosedLoop);
    addParam(motorStatusString,                 &motorStatus);
    addParam(motorUpdateStatusString,           &motorUpdateStatus);
    addParam(motorStatusDirectionString,        &motorStatusDirection);
    addParam(motorStatusDoneString,             &motorStatusDone);
    addParam(motorStatusHighLimitString,        &motorStatusHighLimit);
    addParam(motorStatusAtHomeString,           &motorStatusAtHome);
    addParam(motorStatusSlipString,             &motorStatusSlip);
    addParam(motorStatusPowerOnString,          &motorStatusPowerOn);
    addParam(motorStatusFollowingErrorString,   &motorStatusFollowingError);
    addParam(motorStatusHomeString,             &motorStatusHome);
    addParam(motorStatusHasEncoderString,       &motorStatusHasEncoder);
    addParam(motorStatusProblemString,          &motorStatusProblem);
    addParam(motorStatusMovingString,           &motorStatusMoving);
    addParam(motorStatusGainSupportString,      &motorStatusGainSupport);
    addParam(motorStatusCommsErrorString,       &motorStatusCommsError);
    addParam(motorStatusLowLimitString,         &motorStatusLowLimit);
    addParam(motorStatusHomedString,            &motorStatusHomed);
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: constructor complete\n",
        driverName, functionName);

}

asynStatus asynMotorDriver::readGenericPointer(asynUser *pasynUser, void *pointer)
{
    static const char *functionName = "readGenericPointer";

    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: not implemented in this driver\n", 
        driverName, functionName);
    return(asynError);
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

