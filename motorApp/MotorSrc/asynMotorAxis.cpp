/* asynMotorAxis.cpp 
 * 
 * Mark Rivers
 *
 * This file defines the base class for an asynMotorAxis.  It is the class
 * from which real motor axes are derived.
 */
#include <stdlib.h>
#include <string.h>

#include <epicsThread.h>

#include <asynPortDriver.h>
#define epicsExportSharedSymbols
#include <shareLib.h>
#include "asynMotorAxis.h"
#include "asynMotorController.h"

static const char *driverName = "asynMotorAxis";



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
  static const char *functionName = "asynMotorAxis";

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
  profilePositions_       = NULL;
  profileReadbacks_       = NULL;
  profileFollowingErrors_ = NULL;
  
  // Create the asynUser, connect to this axis
  pasynUser_ = pasynManager->createAsynUser(NULL, NULL);
  pasynManager->connectDevice(pasynUser_, pC->portName, axisNo);
}



/** Move the motor to an absolute location or by a relative amount.
  * \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move 
  * by (if relative=1). Units=steps.
  * \param[in] relative  Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus asynMotorAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  return asynError;
}



/** Move the motor at a fixed velocity until told to stop.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus asynMotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  return asynError;
}



/** Move the motor to the home position.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards  Flag indicating to move the motor in the forward direction(1) or reverse direction(0).
  *                      Some controllers need to be told the direction, others know which way to go to home. */
asynStatus asynMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  return asynError;
}



/** Stop the motor.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus asynMotorAxis::stop(double acceleration)
{
  return asynError;
}



/** Poll the axis.
  * This function should read the controller position, encoder position, and as many of the motorStatus flags
  * as the hardware supports.  It should call setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then call callParamCallbacks() at the end.
  * \param[out] moving A flag that the function must set indicating that the axis is moving (1) or done (0). */
asynStatus asynMotorAxis::poll(bool *moving)
{
  return asynError;
}



/** Set the current position of the motor.
  * \param[in] position The new absolute motor position that should be set in the hardware. Units=steps.*/
asynStatus asynMotorAxis::setPosition(double position)
{
  return asynError;
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
  pC_->setIntegerParam(axisNo_, pC_->motorStatus_, status);
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

/* These are the functions for profile moves */
asynStatus asynMotorAxis::initializeProfile(size_t maxProfilePoints)
{
  if (profilePositions_)       free(profilePositions_);
  profilePositions_ =         (double *)calloc(maxProfilePoints, sizeof(double));
  if (profileReadbacks_)    free(profileReadbacks_);
  profileReadbacks_ =         (double *)calloc(maxProfilePoints, sizeof(double));
  if (profileFollowingErrors_) free(profileFollowingErrors_);
  profileFollowingErrors_ =   (double *)calloc(maxProfilePoints, sizeof(double));
  return asynSuccess;
}
  


/** Function to define the motor positions for a profile move. 
  * This base class function converts the positions from user units
  * to controller units, using the profileMotorOffset_, profileMotorDirection_,
  * and profileMotorResolution_ parameters. 
  * \param[in] positions Array of profile positions for this axis in user units.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus asynMotorAxis::defineProfile(double *positions, size_t numPoints)
{
  size_t i;
  double resolution;
  double offset;
  int direction;
  double scale;
  int status=0;
  //static const char *functionName = "defineProfile";

  if (numPoints > pC_->maxProfilePoints_) return asynError;

  status |= pC_->getDoubleParam(axisNo_, pC_->profileMotorResolution_, &resolution);
  status |= pC_->getDoubleParam(axisNo_, pC_->profileMotorOffset_, &offset);
  status |= pC_->getIntegerParam(axisNo_, pC_->profileMotorDirection_, &direction);
  if (status) return asynError;
  if (resolution == 0.0) return asynError;
  
  // Convert to controller units
  scale = 1.0/resolution;
  if (direction != 0) scale = -scale;
  for (i=0; i<numPoints; i++) {
    profilePositions_[i] = (positions[i] - offset)*scale;
  }
  return asynSuccess;
}



/** Function to build a coordinated move of multiple axes. */
asynStatus asynMotorAxis::buildProfile()
{
  // static const char *functionName = "buildProfile";

  return asynSuccess;
}



/** Function to execute a coordinated move of multiple axes. */
asynStatus asynMotorAxis::executeProfile()
{
  // static const char *functionName = "executeProfile";

  return asynSuccess;
}



/** Function to abort a profile. */
asynStatus asynMotorAxis::abortProfile()
{
  // static const char *functionName = "abortProfile";

  return asynSuccess;
}



/** Function to readback the actual motor positions from a coordinated move of multiple axes.
  * This base class function converts the readbacks and following errors from controller units 
  * to user units and does callbacks on the arrays.
  * Caution: this function modifies the readbacks in place, so it must only be called
  * once per readback operation.
 */
asynStatus asynMotorAxis::readbackProfile()
{
  int i;
  double resolution;
  double offset;
  int direction;
  int numReadbacks;
  int status=0;
  static const char *functionName = "readbackProfile";

  status |= pC_->getDoubleParam(axisNo_, pC_->profileMotorResolution_, &resolution);
  status |= pC_->getDoubleParam(axisNo_, pC_->profileMotorOffset_, &offset);
  status |= pC_->getIntegerParam(axisNo_, pC_->profileMotorDirection_, &direction);
  status |= pC_->getIntegerParam(0, pC_->profileNumReadbacks_, &numReadbacks);
  if (status) return asynError;
  
  // Convert to user units
  if (direction != 0) resolution = -resolution;
  for (i=0; i<numReadbacks; i++) {
    profileReadbacks_[i] = profileReadbacks_[i] * resolution + offset;
    profileFollowingErrors_[i] = profileFollowingErrors_[i] * resolution;
  }
  status  = pC_->doCallbacksFloat64Array(profileReadbacks_,       numReadbacks, pC_->profileReadbacks_, axisNo_);
  status |= pC_->doCallbacksFloat64Array(profileFollowingErrors_, numReadbacks, pC_->profileFollowingErrors_, axisNo_);
  return asynSuccess;
}
