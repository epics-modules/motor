/*
FILENAME...	asynMotorStatus.cpp
USAGE...	

Version:	$Revision$
Modified By:	$Author$
Last Modified:	$Date$
*/

#include "asynMotorStatus.h"

asynMotorStatus::asynMotorStatus(asynMotorController *pC, int axisNo) :
  asynPortDriverExt(pC, axisNo){
    //empty
  }
/** return the number of parameters associated with the status object
  */
int asynMotorStatus::getNumParams()
{
  return(NUM_STATUS_PARAMS);
}

/** Create the parameters associated with the status object
  */
asynStatus asynMotorStatus::createParams()
{
  asynStatus status = asynSuccess;
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
  return(status);
}

/** Set the axis is done moving bit
  */
asynStatus asynMotorStatus::setDoneMoving(bool done){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusDone_, done);
  return retStat;
}

/** Set the axis moving bit
  */
asynStatus asynMotorStatus::setMoving(bool moving){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusMoving_, moving);
  return retStat;
}

/** Get the value stored in the done bit indicator
  */
//asynStatus asynMotorStatus::getDone(int *done){
//  asynStatus retStat = asynSuccess;
//  return retStat;
//}

/** Set problem bit indicator
  */
asynStatus asynMotorStatus::setProblem(bool on){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusProblem_, on);
  return retStat;
}

/** Set the - Does the axis have closed-loop gain support? indicator
  */
asynStatus asynMotorStatus::setHasGainSupport(bool on){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusGainSupport_, on);
  return retStat;
}

/** Set the - Does the axis have position feedback (encoder)? indicator
  */
asynStatus asynMotorStatus::setHasEncoder(bool on){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusHasEncoder_, on);
  return retStat;
}

/** Set the - Is the axis at the home position? indicator
  */
asynStatus asynMotorStatus::setAtHome(bool on){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusAtHome_, on);
  return retStat;
}

/** Set the - Is the high limit switch on? indicator
  */
asynStatus asynMotorStatus::setHighLimitOn(bool on){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusHighLimit_, on);
  return retStat;
}

/** Set the - Is the low limit switch on? indicator
  */
asynStatus asynMotorStatus::setLowLimitOn(bool on){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusLowLimit_, on);
  return retStat;
}

/** Set the current/last axis move direction indicator
  */
asynStatus asynMotorStatus::setDirection(Direction way){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusDirection_, way);
  return retStat;
}

/** Set the - Is axis power (torque) on? - indicator
  */
asynStatus asynMotorStatus::setPowerOn(bool on){
  asynStatus retStat = asynSuccess;
  retStat = setIntegerParam(motorStatusPowerOn_, on);
  return retStat;
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
asynStatus asynMotorStatus::setIntegerParam(int function, int value)
{
  int mask;
  epicsUInt32 status;
  // This assumes the parameters defined above are in the same order as the bits the motor record expects!
  if (function >= motorStatusDirection_ && 
      function <= motorStatusHomed_) {
    status = status_;
    mask = 1 << (function - motorStatusDirection_);
    if (value) status |= mask;
    else       status &= ~mask;
    if (status != status_) {
      status_ = status;
      statusChanged_ = 1;
    }
  }
  // Call the base class method
  return pC_->setIntegerParam(axisNo_, function, value);
}


