#include <stdlib.h>

#include <asynPortDriver.h>
#define epicsExportSharedSymbols
#include <shareLib.h>
#include "asynMotorDriver.h"

/** All of the arguments are simply passed to the constructor for the asynPortDriver base class. 
  * After calling the base class constructor this method creates the motor parameters
  * defined in asynMotorDriver.h.
  */

static const char *driverName = "asynMotorDriver";

asynMotorController::asynMotorController(const char *portName, int maxAxes, int numParams,
                                         int interfaceMask, int interruptMask,
                                         int asynFlags, int autoConnect, int priority, int stackSize)

  : asynPortDriver(portName, maxAxes, NUM_MOTOR_DRIVER_PARAMS+numParams,
      interfaceMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask | asynDrvUserMask,
      interruptMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask,
      asynFlags, autoConnect, priority, stackSize)

{
  static const char *functionName = "asynMotorDriver";

  /* Create the base set of motor parameters */
  createParam(motorMoveRelString,                asynParamFloat64,    &motorMoveRel_);
  createParam(motorMoveAbsString,                asynParamFloat64,    &motorMoveAbs_);
  createParam(motorMoveVelString,                asynParamFloat64,    &motorMoveVel_);
  createParam(motorHomeString,                   asynParamFloat64,    &motorHome_);
  createParam(motorStopString,                   asynParamInt32,      &motorStop_);
  createParam(motorVelocityString,               asynParamFloat64,    &motorVelocity_);
  createParam(motorVelBaseString,                asynParamFloat64,    &motorVelBase_);
  createParam(motorAccelString,                  asynParamFloat64,    &motorAccel_);
  createParam(motorPositionString,               asynParamFloat64,    &motorPosition_);
  createParam(motorEncoderPositionString,        asynParamFloat64,    &motorEncoderPosition_);
  createParam(motorDeferMovesString,             asynParamInt32,      &motorDeferMoves_);
  createParam(motorResolutionString,             asynParamFloat64,    &motorResolution_);
  createParam(motorEncRatioString,               asynParamFloat64,    &motorEncRatio_);
  createParam(motorPgainString,                  asynParamFloat64,    &motorPgain_);
  createParam(motorIgainString,                  asynParamFloat64,    &motorIgain_);
  createParam(motorDgainString,                  asynParamFloat64,    &motorDgain_);
  createParam(motorHighLimitString,              asynParamFloat64,    &motorHighLimit_);
  createParam(motorLowLimitString,               asynParamFloat64,    &motorLowLimit_);
  createParam(motorSetClosedLoopString,          asynParamInt32,      &motorSetClosedLoop_);
  createParam(motorStatusString,                 asynParamInt32,      &motorStatus_);
  createParam(motorUpdateStatusString,           asynParamInt32,      &motorUpdateStatus_);
  createParam(motorStatusDirectionString,        asynParamInt32,      &motorStatusDirection_);
  createParam(motorStatusDoneString,             asynParamInt32,      &motorStatusDone_);
  createParam(motorStatusHighLimitString,        asynParamInt32,      &motorStatusHighLimit_);
  createParam(motorStatusAtHomeString,           asynParamInt32,      &motorStatusAtHome_);
  createParam(motorStatusSlipString,             asynParamInt32,      &motorStatusSlip_);
  createParam(motorStatusPowerOnString,          asynParamInt32,      &motorStatusPowerOn_);
  createParam(motorStatusFollowingErrorString,   asynParamInt32,      &motorStatusFollowingError_);
  createParam(motorStatusHomeString,             asynParamInt32,      &motorStatusHome_);
  createParam(motorStatusHasEncoderString,       asynParamInt32,      &motorStatusHasEncoder_);
  createParam(motorStatusProblemString,          asynParamInt32,      &motorStatusProblem_);
  createParam(motorStatusMovingString,           asynParamInt32,      &motorStatusMoving_);
  createParam(motorStatusGainSupportString,      asynParamInt32,      &motorStatusGainSupport_);
  createParam(motorStatusCommsErrorString,       asynParamInt32,      &motorStatusCommsError_);
  createParam(motorStatusLowLimitString,         asynParamInt32,      &motorStatusLowLimit_);
  createParam(motorStatusHomedString,            asynParamInt32,      &motorStatusHomed_);

  axisStatus_ = (MotorStatus *)calloc(maxAxes, sizeof(MotorStatus));
  axisStatusChanged_ = (int *)calloc(maxAxes, sizeof(int));

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: constructor complete\n",
    driverName, functionName);

}
// We override the setIntegerParam, setDoubleParam, and callParamCallbacks methods so we can construct 
// the aggregate status structure and do callbacks on it

asynStatus asynMotorController::setIntegerParam(int axis, int function, int value)
{
  int mask;
  
  // This assumes the parameters defined above are in the same order as the bits the motor record expects!
  if (function >= motorStatusDirection_ && 
  function <= motorStatusHomed_) {
  mask = 1 << (function - motorStatusDirection_);
  if (value) axisStatus_[axis].status |= mask;
  else       axisStatus_[axis].status &= ~mask;
  axisStatusChanged_[axis] = 1;
  }
  // Call the base class method
  return asynPortDriver::setIntegerParam(axis, function, value);
}

asynStatus asynMotorController::setDoubleParam(int axis, int function, double value)
{
  if (function == motorPosition_) {
    axisStatusChanged_[axis] = 1;
    axisStatus_[axis].position = value;
  } else if (function == motorEncoderPosition_) {
    axisStatusChanged_[axis] = 1;
    axisStatus_[axis].encoderPosition = value;
  }  
  // Call the base class method
  return asynPortDriver::setDoubleParam(axis, function, value);
}   

asynStatus asynMotorController::callParamCallbacks(int axis)
{
  if (axisStatusChanged_[axis]) {
    axisStatusChanged_[axis] = 0;
    doCallbacksGenericPointer((void *)&axisStatus_[axis], motorStatus_, axis);
  }
  return asynPortDriver::callParamCallbacks(axis);
}

asynStatus asynMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int axis;
  int function = pasynUser->reason;
  asynStatus status=asynSuccess;
  double accel;
  static const char *functionName = "writeFloat64";

  status = getAddress(pasynUser, &axis);

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(axis, function, value);

  if (function == motorStop_) {
    getDoubleParam(axis, motorAccel_, &accel);
    status = stopAxis(pasynUser, accel);
  
  } else if (function == motorUpdateStatus_) {
    // We don't implement this yet.  Is it needed?
    //status = this->forceCallback)(pasynUser);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(axis);
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
  

asynStatus asynMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int axis;
  int function = pasynUser->reason;
  double baseVelocity, velocity, accel;
  asynStatus status = asynError;
  static const char *functionName = "writeFloat64";

  status = getAddress(pasynUser, &axis);

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setDoubleParam(axis, function, value);

  getDoubleParam(axis, motorVelBase_, &baseVelocity);
  getDoubleParam(axis, motorVelocity_, &velocity);
  getDoubleParam(axis, motorAccel_, &accel);

  if (function == motorMoveRel_) {
    status = moveAxis(pasynUser, value, 1, baseVelocity, velocity, accel);
  
  } else if (function == motorMoveAbs_) {
    status = moveAxis(pasynUser, value, 0, baseVelocity, velocity, accel);

  } else if (function == motorMoveVel_) {
    status = moveVelocityAxis(pasynUser, baseVelocity, value, accel);

  // Note, the motorHome command happens on the asynFloat64 interface, even though the value (direction) is really integer 
  } else if (function == motorHome_) {
    status = homeAxis(pasynUser, baseVelocity, velocity, accel, (value == 0) ? 0 : 1);

  }
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(axis);
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

asynStatus asynMotorController::readGenericPointer(asynUser *pasynUser, void *pointer)
{
  static const char *functionName = "readGenericPointer";
  MotorStatus *pStatus = (MotorStatus *)pointer;
  int axis;
  
  getAddress(pasynUser, &axis);
  getIntegerParam(axis, motorStatus_, (int *)&pStatus->status);
  getDoubleParam(axis, motorPosition_, &pStatus->position);
  getDoubleParam(axis, motorEncoderPosition_, &pStatus->encoderPosition);
  getDoubleParam(axis, motorVelocity_, &pStatus->velocity);
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
    "%s:%s: MotorStatus = status%d, position=%f, encoder position=%f, velocity=%f\n", 
    driverName, functionName, pStatus->status, pStatus->position, pStatus->encoderPosition, pStatus->velocity);
  return(asynSuccess);
}  

asynStatus asynMotorController::moveAxis(asynUser *pasynUser, double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  static const char *functionName = "moveAxis";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}

asynStatus asynMotorController::moveVelocityAxis(asynUser *pasynUser, double minVelocity, double maxVelocity, double acceleration)
{
  static const char *functionName = "moveVelocityAxis";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}

asynStatus asynMotorController::homeAxis(asynUser *pasynUser, double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  static const char *functionName = "homeAxis";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}


asynStatus asynMotorController::stopAxis(asynUser *pasynUser, double acceleration)
{
  static const char *functionName = "stopAxis";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}

asynStatus asynMotorController::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger)
{
  static const char *functionName = "profileMove";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}

asynStatus asynMotorController::triggerProfile(asynUser *pasynUser)
{
  static const char *functionName = "triggerProfile";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}

