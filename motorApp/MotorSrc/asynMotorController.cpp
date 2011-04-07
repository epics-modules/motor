/* asynMotorController.cpp
 * 
 * Mark Rivers
 *
 * This file defines the base class for an asynMotorController.  It is the class
 * from which real motor controllers are derived.  It derives from asynPortDriver.
 */
#include <stdlib.h>
#include <string.h>

#include <epicsThread.h>

#include <asynPortDriver.h>
#define epicsExportSharedSymbols
#include <shareLib.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

static const char *driverName = "asynMotorController";
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
      interfaceMask | asynOctetMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask | asynDrvUserMask,
      interruptMask | asynOctetMask | asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynGenericPointerMask,
      asynFlags, autoConnect, priority, stackSize),
    shuttingDown_(0), numAxes_(numAxes)

{
  static const char *functionName = "asynMotorController";

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

  // These are the per-controller parameters for profile moves
  createParam(profileNumAxesString,              asynParamInt32,      &profileNumAxes_);
  createParam(profileNumPointsString,            asynParamInt32,      &profileNumPoints_);
  createParam(profileCurrentPointString,         asynParamInt32,      &profileCurrentPoint_);
  createParam(profileNumPulsesString,            asynParamInt32,      &profileNumPulses_);
  createParam(profileStartPulsesString,          asynParamInt32,      &profileStartPulses_);
  createParam(profileEndPulsesString,            asynParamInt32,      &profileEndPulses_);
  createParam(profileActualPulsesString,         asynParamInt32,      &profileActualPulses_);
  createParam(profileNumReadbacksString,         asynParamInt32,      &profileNumReadbacks_);
  createParam(profileTimeModeString,             asynParamInt32,      &profileTimeMode_);
  createParam(profileFixedTimeString,          asynParamFloat64,      &profileFixedTime_);
  createParam(profileTimeArrayString,     asynParamFloat64Array,      &profileTimeArray_);
  createParam(profileAccelerationString,       asynParamFloat64,      &profileAcceleration_);
  createParam(profileBuildString,                asynParamInt32,      &profileBuild_);
  createParam(profileBuildStateString,           asynParamInt32,      &profileBuildState_);
  createParam(profileBuildStatusString,          asynParamInt32,      &profileBuildStatus_);
  createParam(profileBuildMessageString,         asynParamOctet,      &profileBuildMessage_);
  createParam(profileExecuteString,              asynParamInt32,      &profileExecute_);
  createParam(profileExecuteStateString,         asynParamInt32,      &profileExecuteState_);
  createParam(profileExecuteStatusString,        asynParamInt32,      &profileExecuteStatus_);
  createParam(profileExecuteMessageString,       asynParamOctet,      &profileExecuteMessage_);
  createParam(profileAbortString,                asynParamInt32,      &profileAbort_);
  createParam(profileReadbackString,             asynParamInt32,      &profileReadback_);
  createParam(profileReadbackStateString,        asynParamInt32,      &profileReadbackState_);
  createParam(profileReadbackStatusString,       asynParamInt32,      &profileReadbackStatus_);
  createParam(profileReadbackMessageString,      asynParamOctet,      &profileReadbackMessage_);

  // These are the per-axis parameters for profile moves
  createParam(profileUseAxisString,              asynParamInt32,      &profileUseAxis_);
  createParam(profilePositionsString,     asynParamFloat64Array,      &profilePositions_);
  createParam(profileReadbacksString,     asynParamFloat64Array,      &profileReadbacks_);
  createParam(profileFollowingErrorsString, asynParamFloat64Array,    &profileFollowingErrors_);
  createParam(profileMotorResolutionString,    asynParamFloat64,      &profileMotorResolution_);
  createParam(profileMotorDirectionString,       asynParamInt32,      &profileMotorDirection_);
  createParam(profileMotorOffsetString,        asynParamFloat64,      &profileMotorOffset_);

  pAxes_ = (asynMotorAxis**) calloc(numAxes, sizeof(asynMotorAxis*));
  pollEventId_ = epicsEventMustCreate(epicsEventEmpty);

  maxProfilePoints_ = 0;
  profileTimes_ = NULL;

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: constructor complete\n",
    driverName, functionName);
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorStop_ then it calls pAxis->stop().
  * If the function is motorUpdateStatus_ then it does a poll and forces a callback.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * Motor drivers will reimplement this function if they support 
  * controller-specific parameters on the asynInt32 interface. They should call this
  * base class method for any parameters that are not controller-specific.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus asynMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status=asynSuccess;
  asynMotorAxis *pAxis;
  int axis;
  static const char *functionName = "writeInt32";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;
  axis = pAxis->axisNo_;

  /* Set the parameter and readback in the parameter library. */
  pAxis->setIntegerParam(function, value);

  if (function == motorStop_) {
    double accel;
    getDoubleParam(axis, motorAccel_, &accel);
    status = pAxis->stop(accel);
  
  } else if (function == motorUpdateStatus_) {
    bool moving;
    /* Do a poll, and then force a callback */
    poll();
    status = pAxis->poll(&moving);
    pAxis->statusChanged_ = 1;
  } else if (function == profileBuild_) {
    status = buildProfile();
  } else if (function == profileExecute_) {
    status = executeProfile();
  } else if (function == profileAbort_) {
    status = abortProfile();
  } else if (function == profileReadback_) {
    status = readbackProfile();
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
  static const char *functionName = "writeFloat64";

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

/** Called when asyn clients call pasynFloat64Array->write().
  * The base class implementation simply prints an error message.  
  * Derived classes may reimplement this function if required.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to write.
  * \param[in] nElements Number of elements to write. */
asynStatus asynMotorController::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                                  size_t nElements)
{
  int function = pasynUser->reason;
  asynMotorAxis *pAxis;
  static const char *functionName = "writeFloat64Array";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;
  
  if (nElements > maxProfilePoints_) nElements = maxProfilePoints_;
   
  if (function == profileTimeArray_) {
    memcpy(profileTimes_, value, nElements*sizeof(double));
  } 
  else if (function == profilePositions_) {
    pAxis->defineProfile(value, nElements);
  } 
  else {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: unknown parameter number %d\n", 
      driverName, functionName, function);
    return asynError ;
  }
  return asynSuccess;
}

/** Called when asyn clients call pasynFloat64Array->read().
  * Returns the readbacks or following error arrays from profile moves.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to read.
  * \param[in] nElements Maximum number of elements to read. 
  * \param[in] Number of values actually returned */
asynStatus asynMotorController::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                                 size_t nElements, size_t *nRead)
{
  int function = pasynUser->reason;
  asynMotorAxis *pAxis;
  int numReadbacks;
  static const char *functionName = "readFloat64Array";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;
  
  getIntegerParam(profileNumReadbacks_, &numReadbacks);
  *nRead = numReadbacks;
  if (*nRead > nElements) *nRead = nElements;

  if (function == profileReadbacks_) {
    memcpy(value, pAxis->profileReadbacks_, *nRead*sizeof(double));
  } 
  else if (function == profileFollowingErrors_) {
    memcpy(value, pAxis->profileFollowingErrors_, *nRead*sizeof(double));
  } 
  else {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: unknown parameter number %d\n", 
      driverName, functionName, function);
    return asynError ;
  }
  return asynSuccess;
}


/** Called when asyn clients call pasynGenericPointer->read().
  * Builds an aggregate MotorStatus structure at the memory location of the
  * input pointer.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] pointer Pointer to the MotorStatus object to return. */
asynStatus asynMotorController::readGenericPointer(asynUser *pasynUser, void *pointer)
{
  MotorStatus *pStatus = (MotorStatus *)pointer;
  int axis;
  asynMotorAxis *pAxis;
  static const char *functionName = "readGenericPointer";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;
  axis = pAxis->axisNo_;
 
  getAddress(pasynUser, &axis);
  getIntegerParam(axis, motorStatus_, (int *)&pStatus->status);
  getDoubleParam(axis, motorPosition_, &pStatus->position);
  getDoubleParam(axis, motorEncoderPosition_, &pStatus->encoderPosition);
  getDoubleParam(axis, motorVelocity_, &pStatus->velocity);
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
    "%s:%s: MotorStatus = status%d, position=%f, encoder position=%f, velocity=%f\n", 
    driverName, functionName, pStatus->status, pStatus->position, pStatus->encoderPosition, pStatus->velocity);
  return asynSuccess;
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
  bool anyMoving;
  bool moving;
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
    anyMoving = false;
    lock();
    if (shuttingDown_) {
      unlock();
      break;
    }
    poll();
    for (i=0; i<numAxes_; i++) {
        pAxis=getAxis(i);
        if (!pAxis) continue;
        pAxis->poll(&moving);
        if (moving) anyMoving = true;;
    }
    if (forcedFastPolls > 0) {
      timeout = movingPollPeriod_;
      forcedFastPolls--;
    } else if (anyMoving) {
      timeout = movingPollPeriod_;
    } else {
      timeout = idlePollPeriod_;
    }
    unlock();
  }
}

/* These are the functions for profile moves */
/** Initialize a profile move of multiple axes. */
asynStatus asynMotorController::initializeProfile(size_t maxProfilePoints)
{
  int axis;
  asynMotorAxis *pAxis;
  // static const char *functionName = "initializeProfile";
  
  maxProfilePoints_ = maxProfilePoints;
  if (profileTimes_) free(profileTimes_);
  profileTimes_ = (double *)calloc(maxProfilePoints, sizeof(double));
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    pAxis->initializeProfile(maxProfilePoints);
  }
  return asynSuccess;
}
  
/** Build a profile move of multiple axes. */
asynStatus asynMotorController::buildProfile()
{
  //static const char *functionName = "buildProfile";
  asynMotorAxis *pAxis;
  int i;
  int status=0;
  double time;
  int timeMode;
  int numPoints;

  status |= getIntegerParam(profileTimeMode_, &timeMode);
  status |= getDoubleParam(profileFixedTime_, &time);
  status |= getIntegerParam(profileNumPoints_, &numPoints);
  if (status) return asynError;
  if (timeMode == PROFILE_TIME_MODE_FIXED) {
    memset(profileTimes_, 0, maxProfilePoints_*sizeof(double));
    for (i=0; i<numPoints; i++) {
      profileTimes_[i] = time;
    }
  }
  for (i=0; i<numAxes_; i++) {
    pAxis = getAxis(i);
    pAxis->buildProfile();
  }
  return asynSuccess;
}

/** Execute a profile move of multiple axes. */
asynStatus asynMotorController::executeProfile()
{
  // static const char *functionName = "executeProfile";
  int axis;
  asynMotorAxis *pAxis;
  
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    pAxis->executeProfile();
  }
  return asynSuccess;
}

/** Aborts a profile move. */
asynStatus asynMotorController::abortProfile()
{
  // static const char *functionName = "abortProfile";
  int axis;
  asynMotorAxis *pAxis;
  
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    pAxis->abortProfile();
  }
  return asynSuccess;
}

/** Readback the actual motor positions from a profile move of multiple axes. */
asynStatus asynMotorController::readbackProfile()
{
  // static const char *functionName = "readbackProfile";
  int axis;
  asynMotorAxis *pAxis;
  
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    pAxis->readbackProfile();
  }
  return asynSuccess;
}
