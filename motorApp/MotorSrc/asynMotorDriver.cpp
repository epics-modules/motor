#include <stdlib.h>

#include <epicsThread.h>

#include <asynPortDriver.h>
#define epicsExportSharedSymbols
#include <shareLib.h>
#include "asynMotorDriver.h"

/** Number of fast polls when poller first wakes up.  This allows for fact
 * that the status may not be moving on the first few polls */


static const char *driverName = "asynMotorDriver";
static void asynMotorPollerC(void *drvPvt);

/** Creates a new asynMotorController object.
  * All of the arguments are simply passed to the constructor for the asynPortDriver base class. 
  * After calling the base class constructor this method creates the motor parameters
  * defined in asynMotorDriver.h.
  */
asynMotorController::asynMotorController(const char *portName, int numAxes, int numParams,
                                         int interfaceMask, int interruptMask,
                                         int asynFlags, int autoConnect, int priority, int stackSize)

  : asynPortDriver(portName, numAxes, NUM_MOTOR_DRIVER_PARAMS+numParams,
      interfaceMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask | asynDrvUserMask,
      interruptMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask,
      asynFlags, autoConnect, priority, stackSize),
    numAxes_(numAxes)

{
  static const char *functionName = "asynMotorController::asynMotorController";

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

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorStop_ then it calls pAxis->stop().
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * Motor drivers will reimplement this function if they support 
  * controller-specific parameters on the asynInt32 interface. They should call this
  * base class method for any parameters that are not controller-specific.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus asynMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int axis;
  int function = pasynUser->reason;
  asynStatus status=asynSuccess;
  asynMotorAxis *pAxis;
  double accel;
  static const char *functionName = "asynMotorController::writeInt32";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;
  axis = pAxis->axisNo_;

  /* Set the parameter and readback in the parameter library. */
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
  
/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorMoveRel_, motorMoveAbs_, motorMoveVel_, motorHome_, or motorPosition_,
  * then it calls pAxis->move(), pAxis->moveVelocity(), pAxis->home(), or pAxis->setPosition().
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * Motor drivers will reimplement this function if they support 
  * controller-specific parameters on the asynFloat64 interface.  They should call this
  * base class method for any parameters that are not controller-specific.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus asynMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  double baseVelocity, velocity, acceleration;
  asynMotorAxis *pAxis;
  int axis;
  int forwards;
  asynStatus status = asynError;
  static const char *functionName = "asynMotorController::writeFloat64";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;
  axis = pAxis->axisNo_;

  /* Set the parameter and readback in the parameter library. */
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

/** Called when asyn clients call pasynGenericPointer->read().
  * Builds an aggregate MotorStatus structure at the memory location of the
  * input pointer.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] pointer Pointer to the MotorStatus object to return. */
asynStatus asynMotorController::readGenericPointer(asynUser *pasynUser, void *pointer)
{
  static const char *functionName = "asynMotorController::readGenericPointer";
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

/** Function to define a coordinated move of multiple axes.
  * This is not currently implemented, as the API still needs work! */
asynStatus asynMotorController::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger)
{
  static const char *functionName = "asynMotorController::profileMove";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}

/** Function to trigger a coordinated move of multiple axes.
  * This is not currently implemented, as the API still needs work! */
asynStatus asynMotorController::triggerProfile(asynUser *pasynUser)
{
  static const char *functionName = "asynMotorController::triggerProfile";

  asynPrint(pasynUser, ASYN_TRACE_ERROR,
    "%s:%s: not implemented in this driver\n", 
    driverName, functionName);
  return(asynError);
}

/** Returns a pointer to an asynMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * Derived classes will reimplement this function to return a pointer to the derived
  * axis type.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
asynMotorAxis* asynMotorController::getAxis(asynUser *pasynUser)
{
    int axisNo;
    
    getAddress(pasynUser, &axisNo);
    return getAxis(axisNo);
}

/** Returns a pointer to an asynMotorAxis object.
  * Returns NULL if the axis number is invalid.
  * Derived classes will reimplement this function to return a pointer to the derived
  * axis type.
  * \param[in] axisNo Axis index number. */
asynMotorAxis* asynMotorController::getAxis(int axisNo)
{
    if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
    return pAxes_[axisNo];
}

/** Starts the motor poller thread.
  * Derived classes will typically call this at near the end of their constructor.
  * Derived classes can typically use the base class implementation of the poller thread,
  * but are free to reimplement it if necessary.
  * \param[in] movingPollPeriod The time between polls when any axis is moving.
  * \param[in] idlePollPeriod The time between polls when no axis is moving.
  * \param[in] forcedFastPolls The number of times to force the movingPollPeriod after waking up the poller.  
  * This can need to be non-zero for controllers that do not immediately
  * report that an axis is moving after it has been told to start. */
asynStatus asynMotorController::startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls)
{
  movingPollPeriod_ = movingPollPeriod;
  idlePollPeriod_   = idlePollPeriod;
  forcedFastPolls_  = forcedFastPolls;
  epicsThreadCreate("motorPoller", 
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)asynMotorPollerC, (void *)this);
  return asynSuccess;
}

/** Wakes up the poller thread to make it start polling at the movingPollingPeriod_.
  * This is typically called after an axis has been told to move, so the poller immediately
  * starts polling quickly. */
asynStatus asynMotorController::wakeupPoller()
{
  epicsEventSignal(pollEventId_);
  return asynSuccess;
}

/** Polls the asynMotorController (not a specific asynMotorAxis).
  * The base class asynMotorPoller thread calls this method once just before it calls asynMotorAxis::poll
  * for each axis.
  * This base class implementation does nothing.  Derived classes can implement this method if there
  * are controller-wide parameters that need to be polled.  It can also be used for efficiency in some
  * cases. For example some controllers can return the status or positions for all axes in a single
  * command.  In that case asynMotorController::poll() could read that information, and then 
  * asynMotorAxis::poll() might just extract the axis-specific information from the result. */
asynStatus asynMotorController::poll()
{
  return asynSuccess;
}

static void asynMotorPollerC(void *drvPvt)
{
  asynMotorController *pController = (asynMotorController*)drvPvt;
  pController->asynMotorPoller();
}
  

/** Default poller function that runs in the thread created by asynMotorController::startPoller().
  * This base class implementation can be used by most derived classes. 
  * It polls at the idlePollPeriod_ when no axes are moving, and at the movingPollPeriod_ when
  * any axis is moving.  It will immediately do a poll when asynMotorController::wakeupPoller() is
  * called, and will then do forcedFastPolls_ loops at the movingPollPeriod, before reverting back
  * to the idlePollPeriod_ if no axes are moving. It takes the lock on the port driver when it is polling.
  */
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
      forcedFastPolls = forcedFastPolls_;
    }
    anyMoving = 0;
    lock();
    this->poll();
    for (i=0; i<numAxes_; i++) {
        pAxis=getAxis(i);
        if (!pAxis) continue;
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


// asynMotorAxis methods

/** Creates a new asynMotorAxis object.
  * \param[in] pC Pointer to the asynMotorController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Checks that pC is not null, and that axisNo is in the valid range.
  * Sets a pointer to itself in pC->pAxes[axisNo_].
  * Connects pasynUser_ to this asyn port and axisNo.
  */
asynMotorAxis::asynMotorAxis(class asynMotorController *pC, int axisNo)
  : pC_(pC), axisNo_(axisNo), statusChanged_(1)
{
  static const char *functionName = "asynMotorAxis::asynMotorAxis";

  if (!pC) {
    printf("%s:%s: Error, controller is NULL\n",
    driverName, functionName);
    return;
  }
  if ((axisNo < 0) || (axisNo >= pC->numAxes_)) {
    printf("%s:%s: Error, axis=%d is not in range 0 to %d\n",
    driverName, functionName, axisNo, pC->numAxes_-1);
    return;
  }
  pC->pAxes_[axisNo] = this;
  status_.status = 0;
  
  // Create the asynUser, connect to this axis
  pasynUser_ = pasynManager->createAsynUser(NULL, NULL);
  pasynManager->connectDevice(pasynUser_, pC->portName, axisNo);
}


// We implement the setIntegerParam, setDoubleParam, and callParamCallbacks methods so we can construct 
// the aggregate status structure and do callbacks on it

/** Sets the value for an integer for this axis in the parameter library.
  * This function takes special action if the parameter is one of the motorStatus parameters
  * (motorStatusDirection_, motorStatusHomed_, etc.).  In that case it sets or clears the appropriate
  * bit in its private MotorStatus.status structure and if that status has changed sets a flag to
  * do callbacks to devMotorAsyn when callParamCallbacks() is called.
  * \param[in] function The function (parameter) number 
  * \param[in] value Value to set */
asynStatus asynMotorAxis::setIntegerParam(int function, int value)
{
  int mask;
  epicsUInt32 status;
  // This assumes the parameters defined above are in the same order as the bits the motor record expects!
  if (function >= pC_->motorStatusDirection_ && 
      function <= pC_->motorStatusHomed_) {
    status = status_.status;
    mask = 1 << (function - pC_->motorStatusDirection_);
    if (value) status |= mask;
    else       status &= ~mask;
    if (status != status_.status) {
      status_.status = status;
      statusChanged_ = 1;
    }
  }
  // Call the base class method
  return pC_->setIntegerParam(axisNo_, function, value);
}

/** Sets the value for a double for this axis in the parameter library.
  * This function takes special action if the parameter is motorPosition_ or motorEncoderPosition_.  
  * In that case it sets the value in the private MotorStatus structure and if the value has changed
  * then sets a flag to do callbacks to devMotorAsyn when callParamCallbacks() is called.
  * \param[in] function The function (parameter) number 
  * \param[in] value Value to set */
asynStatus asynMotorAxis::setDoubleParam(int function, double value)
{
  if (function == pC_->motorPosition_) {
    if (value != status_.position) {
        statusChanged_ = 1;
        status_.position = value;
    }
  } else if (function == pC_->motorEncoderPosition_) {
    if (value != status_.encoderPosition) {
        statusChanged_ = 1;
        status_.encoderPosition = value;
    }
  }  
  // Call the base class method
  return pC_->setDoubleParam(axisNo_, function, value);
}   

/** Calls the callbacks for any parameters that have changed for this axis in the parameter library.
  * This function takes special action if the aggregate MotorStatus structure has changed.
  * In that case it does callbacks on the asynGenericPointer interface, typically to devMotorAsyn. */  
asynStatus asynMotorAxis::callParamCallbacks()
{
  if (statusChanged_) {
    statusChanged_ = 0;
    pC_->doCallbacksGenericPointer((void *)&status_, pC_->motorStatus_, axisNo_);
  }
  return pC_->callParamCallbacks(axisNo_);
}
