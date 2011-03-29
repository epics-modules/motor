#include <stdlib.h>

#include <epicsThread.h>

#include <asynPortDriver.h>
#define epicsExportSharedSymbols
#include <shareLib.h>
#include "asynMotorDriver.h"

/* Number of fast polls when poller first wakes up.  This allows for fact
 * that the status may not be moving on the first few polls */
 #define FORCED_FAST_POLLS 10

/** All of the arguments are simply passed to the constructor for the asynPortDriver base class. 
  * After calling the base class constructor this method creates the motor parameters
  * defined in asynMotorDriver.h.
  */

static const char *driverName = "asynMotorDriver";
static void asynMotorPollerC(void *drvPvt);

  asynMotorAxis::asynMotorAxis(class asynMotorController *pController, int axisNo)
    : pController_(pController), axisNo_(axisNo), statusChanged_(1)
{
  if (!pController) return;
  if ((axisNo < 0) || (axisNo >= pController->numAxes_)) return;
  pController->pAxes_[axisNo] = this;
  status_.status = 0;
  
  // Create the asynUser, connect to this axis
  pasynUser_ = pasynManager->createAsynUser(NULL, NULL);
  pasynManager->connectDevice(pasynUser_, pController->portName, axisNo);
}

asynMotorController::asynMotorController(const char *portName, int numAxes, int numParams,
                                         int interfaceMask, int interruptMask,
                                         int asynFlags, int autoConnect, int priority, int stackSize)

  : asynPortDriver(portName, numAxes, NUM_MOTOR_DRIVER_PARAMS+numParams,
      interfaceMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask | asynDrvUserMask,
      interruptMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask,
      asynFlags, autoConnect, priority, stackSize),
    numAxes_(numAxes)

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

  pAxes_ = (asynMotorAxis**) calloc(numAxes, sizeof(asynMotorAxis*));
  pollEventId_ = epicsEventMustCreate(epicsEventEmpty);
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: constructor complete\n",
    driverName, functionName);
}

asynStatus asynMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int axis;
  int function = pasynUser->reason;
  asynStatus status=asynSuccess;
  asynMotorAxis *pAxis;
  double accel;
  static const char *functionName = "writeInt32";

  status = getAddress(pasynUser, &axis);
  pAxis = pAxes_[axis];

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  pAxis->setIntegerParam(function, value);

  if (function == motorStop_) {
    getDoubleParam(axis, motorAccel_, &accel);
    status = pAxis->stop(accel);
  
  } else if (function == motorUpdateStatus_) {
    // We don't implement this yet.  Is it needed?
    //status = this->forceCallback)(pasynUser);
  }

  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
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
  int function = pasynUser->reason;
  double baseVelocity, velocity, acceleration;
  asynMotorAxis *pAxis;
  int axis;
  int forwards;
  asynStatus status = asynError;
  static const char *functionName = "writeFloat64";

  pAxis = getAxis(pasynUser);
  axis = pAxis->axisNo_;

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = pAxis->setDoubleParam(function, value);

  getDoubleParam(axis, motorVelBase_, &baseVelocity);
  getDoubleParam(axis, motorVelocity_, &velocity);
  getDoubleParam(axis, motorAccel_, &acceleration);

  if (function == motorMoveRel_) {
    status = pAxis->move(value, 1, baseVelocity, velocity, acceleration);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    pAxis->callParamCallbacks();
    wakeupPoller();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d move relative by %f, base velocity=%f, velocity=%f, acceleration=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value, baseVelocity, velocity, acceleration );
  
  } else if (function == motorMoveAbs_) {
    status = pAxis->move(value, 0, baseVelocity, velocity, acceleration);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    pAxis->callParamCallbacks();
    wakeupPoller();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d move absolute to %f, base velocity=%f, velocity=%f, acceleration=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value, baseVelocity, velocity, acceleration );

  } else if (function == motorMoveVel_) {
    status = pAxis->moveVelocity(baseVelocity, value, acceleration);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    pAxis->callParamCallbacks();
    wakeupPoller();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set port %s, axis %d move with velocity of %f, acceleration=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value, acceleration);

  // Note, the motorHome command happens on the asynFloat64 interface, even though the value (direction) is really integer 
  } else if (function == motorHome_) {
    forwards = (value == 0) ? 0 : 1;
    status = pAxis->home(baseVelocity, velocity, acceleration, forwards);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    pAxis->callParamCallbacks();
    wakeupPoller();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d to home %s, base velocity=%f, velocity=%f, acceleration=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, (forwards?"FORWARDS":"REVERSE"), baseVelocity, velocity, acceleration);

  } else if (function == motorPosition_) {
    status = pAxis->setPosition(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d to position=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  }
  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  
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

asynMotorAxis* asynMotorController::getAxis(asynUser *pasynUser)
{
    int axisNo;
    
    getAddress(pasynUser, &axisNo);
    return getAxis(axisNo);
}

asynMotorAxis* asynMotorController::getAxis(int axisNo)
{
    if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
    return pAxes_[axisNo];
}

asynStatus asynMotorController::startPoller(double movingPollPeriod, double idlePollPeriod)
{
  movingPollPeriod_ = movingPollPeriod;
  idlePollPeriod_   = idlePollPeriod;
  epicsThreadCreate("motorPoller", 
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)asynMotorPollerC, (void *)this);
  return asynSuccess;
}

asynStatus asynMotorController::wakeupPoller()
{
  epicsEventSignal(pollEventId_);
  return asynSuccess;
}

asynStatus asynMotorController::poll()
{
  return asynSuccess;
}

static void asynMotorPollerC(void *drvPvt)
{
  asynMotorController *pController = (asynMotorController*)drvPvt;
  pController->asynMotorPoller();
}
  

void asynMotorController::asynMotorPoller()
{
  double timeout;
  int i;
  int forcedFastPolls=0;
  int anyMoving;
  int moving;
  asynMotorAxis *pAxis;
  int status;

  timeout = idlePollPeriod_;
  wakeupPoller();  /* Force on poll at startup */

  while(1) {
    if (timeout != 0.) status = epicsEventWaitWithTimeout(pollEventId_, timeout);
    else               status = epicsEventWait(pollEventId_);
    if (status == epicsEventWaitOK) {
      /* We got an event, rather than a timeout.  This is because other software
       * knows that an axis should have changed state (started moving, etc.).
       * Force a minimum number of fast polls, because the controller status
       * might not have changed the first few polls
       */
      forcedFastPolls = FORCED_FAST_POLLS;
    }
    anyMoving = 0;
    lock();
    this->poll();
    for (i=0; i<numAxes_; i++) {
        pAxis=getAxis(i);
        pAxis->poll(&moving);
        if (moving) anyMoving=1;
    }
    unlock();
    if (forcedFastPolls > 0) {
      timeout = movingPollPeriod_;
      forcedFastPolls--;
    } else if (anyMoving) {
      timeout = movingPollPeriod_;
    } else {
      timeout = idlePollPeriod_;
    }
  }
}


// We implement the setIntegerParam, setDoubleParam, and callParamCallbacks methods so we can construct 
// the aggregate status structure and do callbacks on it

asynStatus asynMotorAxis::setIntegerParam(int function, int value)
{
  int mask;
  asynMotorController *pC = getController();
  
  // This assumes the parameters defined above are in the same order as the bits the motor record expects!
  if (function >= pC->motorStatusDirection_ && 
      function <= pC->motorStatusHomed_) {
    mask = 1 << (function - pC->motorStatusDirection_);
    if (value) status_.status |= mask;
    else       status_.status &= ~mask;
    statusChanged_ = 1;
  }
  // Call the base class method
  return pC->setIntegerParam(axisNo_, function, value);
}

asynStatus asynMotorAxis::setDoubleParam(int function, double value)
{
  asynMotorController *pC = getController();

  if (function == pC->motorPosition_) {
    statusChanged_ = 1;
    status_.position = value;
  } else if (function == pC->motorEncoderPosition_) {
    statusChanged_ = 1;
    status_.encoderPosition = value;
  }  
  // Call the base class method
  return pC->setDoubleParam(axisNo_, function, value);
}   

asynStatus asynMotorAxis::callParamCallbacks()
{
  asynMotorController *pC = getController();
  
  if (statusChanged_) {
    statusChanged_ = 0;
    pC->doCallbacksGenericPointer((void *)&status_, pC->motorStatus_, axisNo_);
  }
  return pC->callParamCallbacks(axisNo_);
}

asynMotorController* asynMotorAxis::getController()
{
  return pController_;
}
