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
#include <iocsh.h>

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#define epicsExportSharedSymbols
#include <shareLib.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#ifndef VERSION_INT
#  define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#endif

#define MOTOR_ASYN_VERSION_INT VERSION_INT(ASYN_VERSION,ASYN_REVISION,ASYN_MODIFICATION,0)

#define VERSION_INT_4_32 VERSION_INT(4,32,0,0)

static const char *driverName = "asynMotorController";
static void asynMotorPollerC(void *drvPvt);
static void asynMotorMoveToHomeC(void *drvPvt);


extern "C" {
  double getNowTimeSecs(void)
  {
    epicsTimeStamp nowTime;
    epicsTimeGetCurrent(&nowTime);
    return nowTime.secPastEpoch + (nowTime.nsec * 0.000000001);
  }
}

/** Creates a new asynMotorController object.
  * All of the arguments are simply passed to the constructor for the asynPortDriver base class. 
  * After calling the base class constructor this method creates the motor parameters
  * defined in asynMotorDriver.h.
  */
asynMotorController::asynMotorController(const char *portName, int numAxes, int numParams,
                                         int interfaceMask, int interruptMask,
                                         int asynFlags, int autoConnect, int priority, int stackSize)

  : asynPortDriver(portName, numAxes,
#if MOTOR_ASYN_VERSION_INT < VERSION_INT_4_32
                   NUM_MOTOR_DRIVER_PARAMS+numParams,
#endif
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
  createParam(motorMoveToHomeString,             asynParamInt32,      &motorMoveToHome_);
  createParam(motorResolutionString,             asynParamFloat64,    &motorResolution_);
  createParam(motorEncoderRatioString,           asynParamFloat64,    &motorEncoderRatio_);
  createParam(motorPGainString,                  asynParamFloat64,    &motorPGain_);
  createParam(motorIGainString,                  asynParamFloat64,    &motorIGain_);
  createParam(motorDGainString,                  asynParamFloat64,    &motorDGain_);
  createParam(motorHighLimitString,              asynParamFloat64,    &motorHighLimit_);
  createParam(motorLowLimitString,               asynParamFloat64,    &motorLowLimit_);
  createParam(motorClosedLoopString,             asynParamInt32,      &motorClosedLoop_);
  createParam(motorPowerAutoOnOffString,         asynParamInt32,      &motorPowerAutoOnOff_);
  createParam(motorPowerOnDelayString,           asynParamFloat64,    &motorPowerOnDelay_);
  createParam(motorPowerOffDelayString,          asynParamFloat64,    &motorPowerOffDelay_);
  createParam(motorPowerOffFractionString,       asynParamInt32,      &motorPowerOffFraction_);
  createParam(motorPostMoveDelayString,          asynParamFloat64,    &motorPostMoveDelay_);
  createParam(motorStatusString,                 asynParamInt32,      &motorStatus_);
  createParam(motorUpdateStatusString,           asynParamInt32,      &motorUpdateStatus_);
  createParam(motorLatestCommandString,          asynParamInt32,      &motorLatestCommand_);
  createParam(motorMessageIsFromDriverString,    asynParamInt32,      &motorMessageIsFromDriver_);
  createParam(motorMessageTextString,            asynParamOctet,      &motorMessageText_);
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

  /* Addition flags which can be set by the specific driver */
  createParam(motorFlagsHomeOnLsString,          asynParamInt32,      &motorFlagsHomeOnLs_);
  createParam(motorFlagsLSrampDownString,        asynParamInt32,      &motorFlagsLSrampDown_);
  createParam(motorFlagsNoStopOnLsString,        asynParamInt32,      &motorFlagsNoStopOnLS_);
  createParam(motorFlagsDriverUsesEGUString,     asynParamInt32,      &motorFlagsDriverUsesEGU_);
  createParam(motorFlagsAdjAfterHomedString,     asynParamInt32,      &motorFlagsAdjAfterHomed_);

  createParam(motorWaitPollsBeforeReadyString,   asynParamInt32,      &motorWaitPollsBeforeReady_);

  createParam(motorNotHomedProblemString,        asynParamInt32,      &motorNotHomedProblem_);
  createParam(motorShowPowerOffString,           asynParamInt32,      &motorShowPowerOff_);

  // These are per-axis parameters for passing additional motor record information to the driver
  createParam(motorRecResolutionString,        asynParamFloat64,      &motorRecResolution_);
  createParam(motorRecDirectionString,           asynParamInt32,      &motorRecDirection_);
  createParam(motorRecOffsetString,            asynParamFloat64,      &motorRecOffset_);

  /* Parameters from the controller to the driver and record */
  createParam(motorHighLimitROString,            asynParamFloat64,    &motorHighLimitRO_);
  createParam(motorLowLimitROString,             asynParamFloat64,    &motorLowLimitRO_);

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
  createParam(profileMoveModeString,             asynParamInt32,      &profileMoveMode_);
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

  pAxes_ = (asynMotorAxis**) calloc(numAxes, sizeof(asynMotorAxis*));
  pollEventId_ = epicsEventMustCreate(epicsEventEmpty);
  moveToHomeId_ = epicsEventMustCreate(epicsEventEmpty);

  maxProfilePoints_ = 0;
  profileTimes_ = NULL;
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);

  moveToHomeAxis_ = 0;

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: constructor complete\n",
    driverName, functionName);
}

asynMotorController::~asynMotorController()
{
}

/** Called when asyn clients call pasynManager->report().
  * This calls the report method for each axis, and then the base class
  * asynPortDriver report method.
  * \param[in] fp FILE pointer.
  * \param[in] level Level of detail to print. */
void asynMotorController::report(FILE *fp, int level)
{
  int axis;
  asynMotorAxis *pAxis;

  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (!pAxis) continue; 
    pAxis->report(fp, level);
  }

  // Call the base class method
  asynPortDriver::report(fp, level);
}

asynStatus asynMotorController::autoPowerOn(asynMotorAxis *pAxis)
{
  asynStatus status = asynSuccess;
  int axis = pAxis->axisNo_;
  int autoPower = 0;

  getIntegerParam(axis, motorPowerAutoOnOff_, &autoPower);
  if (!autoPower) return asynSuccess;

  double autoPowerOnDelay = 0.0;
  getDoubleParam(axis, motorPowerOnDelay_, &autoPowerOnDelay);
  if (autoPower == 1) {
    status = pAxis->setClosedLoop(true);
    pAxis->setWasMovingFlag(1);
    epicsThreadSleep(autoPowerOnDelay);
    return status;
  }
  else if (autoPower == 2) {
    int motorStatusPowerOn;
    pAxis->setDisableFlag(0);
    getIntegerParam(axis, motorStatusPowerOn_, &motorStatusPowerOn);
    if (motorStatusPowerOn || pAxis->pollPowerIsOn()) {
      /* Amplifier is on, we are done */
      return asynSuccess;
    }
    status = pAxis->setClosedLoop(true);
    pAxis->setWasMovingFlag(1);
    while (autoPowerOnDelay > 0.0) {
      if (pAxis->pollPowerIsOn())
        return asynSuccess;
      epicsThreadSleep(movingPollPeriod_);
      autoPowerOnDelay -= movingPollPeriod_;
    }
  }
  return status;
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
  status = pAxis->setIntegerParam(function, value);

  if (function == motorStop_) {
    double accel;
    getDoubleParam(axis, motorAccel_, &accel);
    pAxis->setIntegerParam(motorLatestCommand_, LATEST_COMMAND_STOP);
    status = pAxis->stop(accel);
  
  } else if (function == motorDeferMoves_) {
    status = setDeferredMoves(value);
  
  } else if (function == motorClosedLoop_) {
    status = pAxis->setClosedLoop(value);

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

  } else if (function == motorMoveToHome_) {
    if (value == 1) {
      pAxis->setIntegerParam(motorLatestCommand_, LATEST_COMMAND_MOVE_TO_HOME);
      asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s:: Starting a move to home for axis %d\n",  driverName, functionName, axis);
      moveToHomeAxis_ = axis;
      epicsEventSignal(moveToHomeId_);
    }
  }

  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  {
    const char *paramName = NULL;
    if (getParamName(function, &paramName)) paramName = "";
    if (status)
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d axis=%d, function=%s (%d), value=%d\n",
                driverName, functionName, status, axis, paramName, function, value);
    else
      asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s:: axis=%d, function=%s (%d), value=%d\n",
                driverName, functionName, axis, paramName, function, value);
  }
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

  if (function == motorMoveRel_) {
    autoPowerOn(pAxis);
    getDoubleParam(axis, motorVelBase_, &baseVelocity);
    getDoubleParam(axis, motorVelocity_, &velocity);
    getDoubleParam(axis, motorAccel_, &acceleration);
    pAxis->setIntegerParam(motorLatestCommand_, LATEST_COMMAND_MOVE_REL);
    status = pAxis->move(value, 1, baseVelocity, velocity, acceleration);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    getIntegerParam(axis, motorWaitPollsBeforeReady_, &pAxis->waitNumPollsBeforeReady_);
    pAxis->callParamCallbacks();
    wakeupPoller();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d move relative by %f, base velocity=%f, velocity=%f, acceleration=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value, baseVelocity, velocity, acceleration );
  
  } else if (function == motorMoveAbs_) {
    autoPowerOn(pAxis);
    getDoubleParam(axis, motorVelBase_, &baseVelocity);
    getDoubleParam(axis, motorVelocity_, &velocity);
    getDoubleParam(axis, motorAccel_, &acceleration);
    pAxis->setIntegerParam(motorLatestCommand_, LATEST_COMMAND_MOVE_ABS);
    status = pAxis->move(value, 0, baseVelocity, velocity, acceleration);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    getIntegerParam(axis, motorWaitPollsBeforeReady_, &pAxis->waitNumPollsBeforeReady_);
    pAxis->callParamCallbacks();
    wakeupPoller();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d move absolute to %f, base velocity=%f, velocity=%f, acceleration=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value, baseVelocity, velocity, acceleration );

  } else if (function == motorMoveVel_) {
    autoPowerOn(pAxis);
    getDoubleParam(axis, motorVelBase_, &baseVelocity);
    getDoubleParam(axis, motorAccel_, &acceleration);
    pAxis->setIntegerParam(motorLatestCommand_, LATEST_COMMAND_MOVE_VEL);
    status = pAxis->moveVelocity(baseVelocity, value, acceleration);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    getIntegerParam(axis, motorWaitPollsBeforeReady_, &pAxis->waitNumPollsBeforeReady_);
    pAxis->callParamCallbacks();
    wakeupPoller();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set port %s, axis %d move with velocity of %f, acceleration=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value, acceleration);

  // Note, the motorHome command happens on the asynFloat64 interface, even though the value (direction) is really integer 
  } else if (function == motorHome_) {
    autoPowerOn(pAxis);
    getDoubleParam(axis, motorVelBase_, &baseVelocity);
    getDoubleParam(axis, motorVelocity_, &velocity);
    getDoubleParam(axis, motorAccel_, &acceleration);
    forwards = (value == 0) ? 0 : 1;
    pAxis->setIntegerParam(motorLatestCommand_, LATEST_COMMAND_HOMING);
    status = pAxis->home(baseVelocity, velocity, acceleration, forwards);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    getIntegerParam(axis, motorWaitPollsBeforeReady_, &pAxis->waitNumPollsBeforeReady_);
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

  } else if (function == motorEncoderPosition_) {
    status = pAxis->setEncoderPosition(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d to encoder position=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  } else if (function == motorHighLimit_) {
    status = pAxis->setHighLimit(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d high limit=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  } else if (function == motorLowLimit_) {
    status = pAxis->setLowLimit(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d low limit=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  } else if (function == motorPGain_) {
    status = pAxis->setPGain(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d proportional gain=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  } else if (function == motorIGain_) {
    status = pAxis->setIGain(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d integral gain=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  } else if (function == motorDGain_) {
    status = pAxis->setDGain(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d derivative gain=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  } else if (function == motorEncoderRatio_) {
    status = pAxis->setEncoderRatio(value);
    pAxis->callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
      "%s:%s: Set driver %s, axis %d encoder ratio=%f\n",
      driverName, functionName, portName, pAxis->axisNo_, value);

  }
  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  
  {
    const char *paramName = NULL;
    if (getParamName(function, &paramName)) paramName = "";
    if (status)
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d axis=%d, function=%s (%d), value=%f\n",
                driverName, functionName, status, axis, paramName, function, value);
    else
      asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s:: axis=%d, function=%s (%d), value=%f\n",
                driverName, functionName, axis, paramName, function, value);
  }
  return status;
    
}

/** Called when asyn clients call pasynFloat64Array->write().
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
  * \param[in] nRead Number of values actually returned */
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
  int axisNo;
  asynStatus status = asynSuccess;
  asynMotorAxis *pAxis;
  static const char *functionName = "readGenericPointer";

  pAxis = getAxis(pasynUser);
  if (!pAxis) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s: no axis found\n",
                  functionName);
    return asynError;
  }
  axisNo = pAxis->axisNo_;
  getAddress(pasynUser, &axisNo);
  /*  We need to make sure that the most important members had been retrieved
      from the controller */
  status = getIntegerParam(axisNo, motorStatus_, (int *)&pStatus->status);
  if (status != asynSuccess) {
    if (status == asynParamUndefined) {
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                    "Error: axisNo=%d motorStatus is undefined", axisNo);
    }
    return status;
  }
  status = getDoubleParam(axisNo, motorPosition_, &pStatus->position);
  if (status != asynSuccess) {
    if (status == asynParamUndefined) {
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                    "Error: axisNo=%d motorPosition is undefined", axisNo);
    }
    return status;
  }
  memcpy(pStatus, &pAxis->status_, sizeof(*pStatus));
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: axisNo=%d status=0x%04x, position=%f, encoder position=%f, velocity=%f, "
            "highLimit=%f lowLimit=%f\n",
            driverName, functionName,  axisNo, pStatus->status, pStatus->position,
            pStatus->encoderPosition, pStatus->velocity,
            pStatus->MotorConfigRO.motorHighLimitRaw,
            pStatus->MotorConfigRO.motorLowLimitRaw);
  return status;
}

asynStatus asynMotorController::writeGenericPointer(asynUser *pasynUser, void *genericPointer)
{
  genericMessage_type *pMotorExtMessage = (genericMessage_type *)genericPointer;
  asynStatus status = asynSuccess;
  asynMotorAxis *pAxis;
  int axis;
  static const char *functionName = "GenericPointer";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;
  axis = pAxis->axisNo_;

  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: axis=%d  pos=%f mres=%f accEGU=%f vbas=%f vel=%f moveType=%d\n",
            driverName, functionName, axis,
            pMotorExtMessage->pos,
            pMotorExtMessage->mres,
            pMotorExtMessage->accEGU,
            pMotorExtMessage->vbas,
            pMotorExtMessage->vel,
            pMotorExtMessage->moveType);
  autoPowerOn(pAxis);
  switch (pMotorExtMessage->moveType) {
    case MOVE_TYPE_ABS:
    case MOVE_TYPE_REL:
      /* fall through */
      {
        int relative = (pMotorExtMessage->moveType == MOVE_TYPE_REL);
        pAxis->setIntegerParam(motorLatestCommand_,
                               relative ? LATEST_COMMAND_MOVE_REL : LATEST_COMMAND_MOVE_ABS);

        status = pAxis->moveEGU(pMotorExtMessage->pos, pMotorExtMessage->mres, relative,
                                pMotorExtMessage->vbas, pMotorExtMessage->vel,
                                pMotorExtMessage->accEGU);
        break;
      }
      case MOVE_TYPE_VELO:
        pAxis->setIntegerParam(motorLatestCommand_,LATEST_COMMAND_MOVE_VEL);
        status = pAxis->moveVeloEGU(pMotorExtMessage->mres,
                                    pMotorExtMessage->vbas, pMotorExtMessage->vel,
                                    pMotorExtMessage->accEGU);
      break;
  }
  pAxis->setIntegerParam(motorStatusDone_, 0);
  getIntegerParam(axis, motorWaitPollsBeforeReady_, &pAxis->waitNumPollsBeforeReady_);
  pAxis->callParamCallbacks();
  wakeupPoller();

  return status;
}


/** Called when asyn clients call pasynOctetSyncIO->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write.
  * \param[in] nChars len (but we only support strings ?!).
  * \param[out] nActual. number of octets that had been written */
asynStatus asynMotorController::writeOctet(asynUser *pasynUser, const char *value,
                                          size_t nChars, size_t *nActual)
{
  asynStatus status = asynSuccess;
  asynMotorAxis *pAxis;
  int function = pasynUser->reason;

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;

  status = pAxis->setStringParam(function, value);
  if (status == asynSuccess) *nActual = strlen(value);
  return status;
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

/** Processes deferred moves.
  * \param[in] deferMoves defer moves till later (true) or process moves now (false) */
asynStatus asynMotorController::setDeferredMoves(bool deferMoves)
{
  return asynSuccess;
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
  



/** Poller helper function
  */
double asynMotorController::pollAll(void)
{
  double timeout = idlePollPeriod_;
  asynMotorAxis *pAxis;
  asynStatus asynstatus;
  bool anyMoving = false;
  int axisNo;
  asynstatus = poll();
  if (asynstatus) {
    asynStatusConnected_ = asynstatus;
    return timeout;
  }

  for (axisNo=0; axisNo<numAxes_; axisNo++) {
    double autoPowerOffDelay = 0.0;
    double nowTimeSecs = 0.0;
    int autoPower = 0;
    bool moving;
    pAxis=getAxis(axisNo);
    if (!pAxis) continue;

    getIntegerParam(axisNo, motorPowerAutoOnOff_, &autoPower);
    getDoubleParam(axisNo, motorPowerOffDelay_, &autoPowerOffDelay);

    asynstatus = pAxis->poll(&moving);
    if (asynstatus) {
      asynStatusConnected_ = asynstatus;
      return timeout;
    }
    if (autoPower) {
      int wasMovingFlag = pAxis->getWasMovingFlag();
      int disableFlag = pAxis->getDisableFlag();
      if ((moving && !wasMovingFlag) || (!moving && wasMovingFlag))  {
        asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
                  "%s:%s: axis=%d autoPower=%d moving=%d getWasMovingFlag=%d getDisableFlag()=%d\n",
                  driverName, __FUNCTION__, axisNo,
                  autoPower, (int)moving, wasMovingFlag, disableFlag);
      }
    }
    if (moving) {
      anyMoving = true;
      pAxis->setWasMovingFlag(1);
    } else {
      /* autoPower mode 2 does allow to keep power on
         forever, if autoPowerOffDelay < 0.0 */
      if ((autoPower == 2) && (autoPowerOffDelay >= 0.0))
        autoPower = 1;
      if ((pAxis->getWasMovingFlag() == 1) && (autoPower == 1)) {
        pAxis->setDisableFlag(1);
        pAxis->setWasMovingFlag(0);
        pAxis->setLastEndOfMoveTime(getNowTimeSecs());
      }
    }

    //Auto power off drive, if:
    //  We have detected an end of move
    //  We are not moving again
    //  Auto power off is enabled
    //  Auto power off delay timer has expired
    if ((!moving) && (autoPower == 1) && (pAxis->getDisableFlag() == 1)) {
      double lastMoveSecs = pAxis->getLastEndOfMoveTime();
      nowTimeSecs = getNowTimeSecs();
      double diffTime = nowTimeSecs - lastMoveSecs;
      asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
                "%s:%s: axis=%d moving=%d lastMoveSecs=%f nowTimeSecs=%f diffTime=%f\n",
                driverName, __FUNCTION__, axisNo,
                (int)moving,
                lastMoveSecs, nowTimeSecs, diffTime);
      if (diffTime >= autoPowerOffDelay) {
        pAxis->setClosedLoop(0);
        pAxis->setDisableFlag(0);
      }
    }

  }
  if (forcedFastPolls_ > 0) {
    timeout = movingPollPeriod_;
    forcedFastPolls_--;
  } else if (anyMoving) {
    timeout = movingPollPeriod_;
  }
  return timeout;
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
    }
    if (shuttingDown_) {
      return; /* Terminate while(1) loop */
    }

    /*
     * A poller does may not use an pasynUserController_, because e.g. it
     * can access the hardware directly), then we dont have to wait for Connect
     * But if the poller uses pasynUserController_, then it must be connected.
     */
    while (pasynUserController_ && (asynStatusConnected_ != asynSuccess)) {
      asynStatus asynstatus = pasynManager->waitConnect(pasynUserController_,
							idlePollPeriod_);
      if (asynStatusConnected_ != asynstatus) {
	asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
		  "%s:%s: waitConnect asynstatus=%d\n",
		  driverName, "asynMotorPoller", (int)asynstatus);
	asynStatusConnected_ = asynstatus;
      }
      if (shuttingDown_) {
	return; /* Terminate while(1) loop */
      }
    }
    lock();
    timeout = pollAll();
    unlock();
  }
}

/**
 * Start the thread which deals with moving axes to their home position.
 * This is called by the derived concrete controller class at object instatiation, so
 * that drivers that don't need this functionality don't have the overhead of the thread.
 */
asynStatus asynMotorController::startMoveToHomeThread()
{
  epicsThreadCreate("motorMoveToHome", 
                    epicsThreadPriorityMedium,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)asynMotorMoveToHomeC, (void *)this);
  return asynSuccess;
}

static void asynMotorMoveToHomeC(void *drvPvt)
{
  asynMotorController *pController = (asynMotorController*)drvPvt;
  pController->asynMotorMoveToHome();
}



/**
 * Default move to home thread. Not normally overridden.
 */
void asynMotorController::asynMotorMoveToHome()
{
  
  asynMotorAxis *pAxis;
  int status = 0;
  static const char *functionName = "asynMotorMoveToHome";

  while(1) {
    status = epicsEventWait(moveToHomeId_);
    if (status == epicsEventWaitOK) { 
      pAxis = getAxis(this->moveToHomeAxis_);
      if (!pAxis) continue;
      status = pAxis->doMoveToHome();
      if (status) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: move to home failed in asynMotorController::asynMotorMoveToHome. Axis number=%d\n", 
        driverName, functionName, this->moveToHomeAxis_);
      }
    } 
  } 
}


/** Writes a string to the controller.
  * Calls writeController() with a default location of the string to write and a default timeout. */ 
asynStatus asynMotorController::writeController()
{
  return writeController(outString_, DEFAULT_CONTROLLER_TIMEOUT);
}

/** Writes a string to the controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus asynMotorController::writeController(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeController";
  
  status = pasynOctetSyncIO->write(pasynUserController_, output,
                                   strlen(output), timeout, &nwrite);
                                  
  return status ;
}

/** Writes a string to the controller and reads the response.
  * Calls writeReadController() with default locations of the input and output strings
  * and default timeout. */ 
asynStatus asynMotorController::writeReadController()
{
  size_t nread;
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus asynMotorController::writeReadController(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  const char *functionName="writeReadController";

  status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
                                       strlen(output), input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);
  if (status == asynTimeout) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
      "%s:%s Timeout\n",
      driverName, functionName);
    asynStatusConnected_ = status;
  } else if ((status != asynSuccess) ||
	     ((*nread == 0) && (eomReason & ASYN_EOM_END))) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
	      "%s:%s nread=%u out=%s status=%s (%d)\n",
	      driverName, functionName,
	      (unsigned)*nread, output ? output : "NULL",
              pasynManager->strStatus(status), (int)status);
    asynStatusConnected_ = asynDisconnected;
  }
  return status;
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
    if (!pAxis) continue;
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
    if (!pAxis) continue;
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
    if (!pAxis) continue;
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
    if (!pAxis) continue;
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
    if (!pAxis) continue;
    pAxis->readbackProfile();
  }
  return asynSuccess;
}

/** Set the moving poll period (in secs) at runtime.*/
asynStatus asynMotorController::setMovingPollPeriod(double movingPollPeriod)
{
  static const char *functionName = "setMovingPollPeriod";

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Setting moving poll period to %f\n", 
    driverName, functionName, movingPollPeriod);

  lock();
  movingPollPeriod_ = movingPollPeriod;
  wakeupPoller();
  unlock();
  return asynSuccess;
}

/** Set the idle poll period (in secs) at runtime.*/
asynStatus asynMotorController::setIdlePollPeriod(double idlePollPeriod)
{
  static const char *functionName = "setIdlePollPeriod";

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s:%s: Setting idle poll period to %f\n", 
    driverName, functionName, idlePollPeriod);

  lock();
  idlePollPeriod_ = idlePollPeriod;
  wakeupPoller();
  unlock();
  return asynSuccess;
}

/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

asynStatus setMovingPollPeriod(const char *portName, double movingPollPeriod)
{
  asynMotorController *pC;
  static const char *functionName = "setMovingPollPeriod";

  pC = (asynMotorController*) findAsynPortDriver(portName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, portName);
    return asynError;
  }
    
  return pC->setMovingPollPeriod(movingPollPeriod);
}

asynStatus setIdlePollPeriod(const char *portName, double idlePollPeriod)
{
  asynMotorController *pC;
  static const char *functionName = "setIdlePollPeriod";

  pC = (asynMotorController*) findAsynPortDriver(portName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, portName);
    return asynError;
  }
    
  return pC->setIdlePollPeriod(idlePollPeriod);
}



asynStatus asynMotorEnableMoveToHome(const char *portName, int axis, int distance)
{
  asynMotorController *pC = NULL;
  asynMotorAxis *pA = NULL;
  static const char *functionName = "asynMotorEnableMoveToHome";

  pC = (asynMotorController*) findAsynPortDriver(portName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, portName);
    return asynError;
  }
  
  pA = pC->getAxis(axis);
  if (!pA) {
    printf("%s:%s: Error axis %d not found\n", driverName, functionName, axis);;
    return asynError;
  }

  if (distance<=0) {
    printf("%s:%s: Error distance must be positive integer axis=%d\n", driverName, functionName, axis);
  } else {
    pA->setReferencingModeMove(distance);
  }

  return asynSuccess;
}


/* setMovingPollPeriod */
static const iocshArg setMovingPollPeriodArg0 = {"Controller port name", iocshArgString};
static const iocshArg setMovingPollPeriodArg1 = {"Axis number", iocshArgDouble};
static const iocshArg * const setMovingPollPeriodArgs[] = {&setMovingPollPeriodArg0,
                                                           &setMovingPollPeriodArg1};
static const iocshFuncDef setMovingPollPeriodDef = {"setMovingPollPeriod", 2, setMovingPollPeriodArgs};

static void setMovingPollPeriodCallFunc(const iocshArgBuf *args)
{
  setMovingPollPeriod(args[0].sval, args[1].dval);
}

/* setIdlePollPeriod */
static const iocshArg setIdlePollPeriodArg0 = {"Controller port name", iocshArgString};
static const iocshArg setIdlePollPeriodArg1 = {"Axis number", iocshArgDouble};
static const iocshArg * const setIdlePollPeriodArgs[] = {&setIdlePollPeriodArg0,
                                                         &setIdlePollPeriodArg1};
static const iocshFuncDef setIdlePollPeriodDef = {"setIdlePollPeriod", 2, setIdlePollPeriodArgs};

static void setIdlePollPeriodCallFunc(const iocshArgBuf *args)
{
  setIdlePollPeriod(args[0].sval, args[1].dval);
}


/* asynMotorEnableMoveToHome */
static const iocshArg asynMotorEnableMoveToHomeArg0 = {"Controller port name", iocshArgString};
static const iocshArg asynMotorEnableMoveToHomeArg1 = {"Axis number", iocshArgInt};
static const iocshArg asynMotorEnableMoveToHomeArg2 = {"Distance", iocshArgInt};
static const iocshArg * const asynMotorEnableMoveToHomeArgs[] = {&asynMotorEnableMoveToHomeArg0,
                                                                 &asynMotorEnableMoveToHomeArg1,
                                                                 &asynMotorEnableMoveToHomeArg2};
static const iocshFuncDef enableMoveToHome = {"asynMotorEnableMoveToHome", 3, asynMotorEnableMoveToHomeArgs};

static void enableMoveToHomeCallFunc(const iocshArgBuf *args)
{
  asynMotorEnableMoveToHome(args[0].sval, args[1].ival, args[2].ival);
}


static void asynMotorControllerRegister(void)
{
  iocshRegister(&setMovingPollPeriodDef, setMovingPollPeriodCallFunc);
  iocshRegister(&setIdlePollPeriodDef, setIdlePollPeriodCallFunc);
  iocshRegister(&enableMoveToHome, enableMoveToHomeCallFunc);
}
epicsExportRegistrar(asynMotorControllerRegister);

} //extern C
