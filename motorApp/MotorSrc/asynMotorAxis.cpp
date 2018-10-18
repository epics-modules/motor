/* asynMotorAxis.cpp 
 * 
 * Mark Rivers
 *
 * This file defines the base class for an asynMotorAxis.  It is the class
 * from which real motor axes are derived.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
  
  /* Used to keep track of referencing mode in the driver.*/
  referencingMode_ = 0;
  /* Used to enable/disable move to home, and to tell driver how far to move.*/
  referencingModeMove_ = 0;

  waitNumPollsBeforeReady_  = 0;
  wasMovingFlag_ = 0;
  disableFlag_ = 0;
  initialPollDone_ = 0;
  lastEndOfMoveTime_ = 0;

  // Create the asynUser, connect to this axis
  pasynUser_ = pasynManager->createAsynUser(NULL, NULL);
  pasynManager->connectDevice(pasynUser_, pC->portName, axisNo);
}

asynMotorAxis::~asynMotorAxis()
{
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
  return asynSuccess;
}

/** Move the motor to an absolute location or by a relative amount.
  * \param[in] posEGU  The absolute position to move to (if relative=0) or the relative distance to move
  * by (if relative=1). Units=steps.
  * \param[in] relative  Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVeloEGU The initial velocity, often called the base velocity. Units=EQU/sec.
  * \param[in] maxVeloEGU The maximum velocity, often called the slew velocity. Units=EGU/sec.
  * \param[in] accEGU  The acceleration value. Units=EGU/sec/sec. */
asynStatus asynMotorAxis::moveEGU(double posEGU, double mres, int relative,
                                  double minVeloEGU, double maxVeloEGU, double accEGU)
{
  double amres = fabs(mres);

  pC_->setDoubleParam(axisNo_, pC_->motorVelBase_, minVeloEGU/amres);
  pC_->setDoubleParam(axisNo_, pC_->motorVelocity_, maxVeloEGU/amres);
  pC_->setDoubleParam(axisNo_, pC_->motorAccel_, accEGU/amres);

  return move(posEGU/mres, relative, minVeloEGU/amres, maxVeloEGU/amres, accEGU/amres);
}


/** Move the motor at a fixed velocity until told to stop.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus asynMotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  return asynSuccess;
}

/** Move the motor at a fixed velocity until told to stop.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=EGU/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=EGU/sec.
  * \param[in] acceleration The acceleration value. Units=EGU/sec/sec. */
asynStatus asynMotorAxis::moveVeloEGU(double mres, double minVeloEGU, double maxVeloEGU, double accEGU)
{
  double amres = fabs(mres);
  pC_->setDoubleParam(axisNo_, pC_->motorVelBase_, minVeloEGU/amres);
  pC_->setDoubleParam(axisNo_, pC_->motorVelocity_, maxVeloEGU/amres);
  pC_->setDoubleParam(axisNo_, pC_->motorAccel_, accEGU/amres);
  return moveVelocity(minVeloEGU/mres, maxVeloEGU/mres, accEGU/amres);
}


/** Move the motor to the home position.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards  Flag indicating to move the motor in the forward direction(1) or reverse direction(0).
  *                      Some controllers need to be told the direction, others know which way to go to home. */
asynStatus asynMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  return asynSuccess;
}


/** Stop the motor.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus asynMotorAxis::stop(double acceleration)
{
  return asynSuccess;
}


/** initial poll of the axis.
  * This function is only called once and should read the configuration of the controller,
  * soft limits and other variables that can be used to initiate the record. */
asynStatus asynMotorAxis::initialPoll(void)
{
  return asynSuccess;
}


/** poll of power on status of the axis.
  * This function is called when the power is turned on before a movement.
  * Typically there is a "blind" timeout, e.g. 3 seconds.
  * If the drive can be polled and reports that the power is on,
  * the timeout will be shortened. */

bool asynMotorAxis::pollPowerIsOn(void)
{
  return false;
}

/** Indicate that the controller is disconnected
  * This function does noy need to do anything.
  * It may set initialPollDone_ to 0. */
void asynMotorAxis::handleDisconnect(asynStatus status)
{
  (void)status;
}

/** Poll the axis.
  * This function should read the controller position, encoder position, and as many of the motorStatus flags
  * as the hardware supports.  It should call setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then call callParamCallbacks() at the end.
  * \param[out] moving A flag that the function must set indicating that the axis is moving (1) or done (0). */
asynStatus asynMotorAxis::poll(bool *moving)
{
  return asynSuccess;
}


/** Set the current position of the motor.
  * \param[in] position The new absolute motor position that should be set in the hardware. Units=steps.*/
asynStatus asynMotorAxis::setPosition(double position)
{
  return asynSuccess;
}


/** Set the current encoder position of the motor.
  * \param[in] position The new absolute encoder position that should be set in the hardware. Units=steps.*/
asynStatus asynMotorAxis::setEncoderPosition(double position)
{
  return asynSuccess;
}


/** Set the high limit position of the motor.
  * \param[in] highLimit The new high limit position that should be set in the hardware. Units=steps.*/
asynStatus asynMotorAxis::setHighLimit(double highLimit)
{
  return asynSuccess;
}


/** Set the low limit position of the motor.
  * \param[in] lowLimit The new low limit position that should be set in the hardware. Units=steps.*/
asynStatus asynMotorAxis::setLowLimit(double lowLimit)
{
  return asynSuccess;
}


/** Set the proportional gain of the motor.
  * \param[in] pGain The new proportional gain. */
asynStatus asynMotorAxis::setPGain(double pGain)
{
  return asynSuccess;
}


/** Set the integral gain of the motor.
  * \param[in] iGain The new integral gain. */
asynStatus asynMotorAxis::setIGain(double iGain)
{
  return asynSuccess;
}


/** Set the derivative gain of the motor.
  * \param[in] dGain The new derivative gain. */
asynStatus asynMotorAxis::setDGain(double dGain)
{
  return asynSuccess;
}


/** Set the motor closed loop status. 
  * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus asynMotorAxis::setClosedLoop(bool closedLoop)
{
  return asynSuccess;
}


/** Set the motor encoder ratio. 
  * \param[in] ratio The new encoder ratio */
asynStatus asynMotorAxis::setEncoderRatio(double ratio)
{
  return asynSuccess;
}

void asynMotorAxis::report(FILE *fp, int details)
{
}


/**
 * Default implementation of doMoveToHome. 
 * Derived classes need to implement this to actually perform the 
 * axis move to the home position.
 */
asynStatus asynMotorAxis::doMoveToHome()
{
  static const char *functionName="doMoveToHome";
  
  asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
    "%s:%s: Axis=%d no implementation\n",
    driverName, functionName, pC_->moveToHomeAxis_);
  return asynSuccess;
}


/**
 * Set method for referencingModeMove_
 */
void  asynMotorAxis::setReferencingModeMove(int distance)
{
  referencingModeMove_ = distance; 
}


/**
 * Get method for referencingModeMove_
 */
int  asynMotorAxis::getReferencingModeMove() 
{
  return referencingModeMove_;
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
  epicsUInt32 status=0, flags=0;
  // This assumes the parameters defined above are in the same order as the bits the motor record expects!
  if (function >= pC_->motorStatusDirection_ && 
      function <= pC_->motorStatusHomed_) {
    if ((function == pC_->motorStatusDone_) &&
        waitNumPollsBeforeReady_) {
      /* Work around the ready before started problem */
      if (value) {
        waitNumPollsBeforeReady_--;
        value = 0;
      } else {
        waitNumPollsBeforeReady_ = 0;
      }
      statusChanged_ = 1;
    }

    status = status_.status;
    mask = 1 << (function - pC_->motorStatusDirection_);
    if (value) status |= mask;
    else       status &= ~mask;
    if (status != status_.status) {
      status_.status = status;
      statusChanged_ = 1;
    }
  } else  if (function >= pC_->motorFlagsHomeOnLs_ &&
              function <= pC_->motorFlagsNoStopOnLS_) {
    flags = status_.flags;
    mask = 1 << (function - pC_->motorFlagsHomeOnLs_);
    if (value) flags |= mask;
    else       flags &= ~mask;
    if (flags != status_.flags) {
      status_.flags = flags;
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
  } else if (function == pC_->motorHighLimitRO_) {
    if (value != status_.MotorConfigRO.motorHighLimitRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorHighLimitRaw = value;
    }
  } else if (function == pC_->motorLowLimitRO_) {
    if (value != status_.MotorConfigRO.motorLowLimitRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorLowLimitRaw = value;
    }
  } else if (function == pC_->motorDefVelocityRO_) {
    if (value != status_.MotorConfigRO.motorDefVelocityRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorDefVelocityRaw = value;
    }
  } else if (function == pC_->motorMaxVelocityRO_) {
    if (value != status_.MotorConfigRO.motorMaxVelocityRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorMaxVelocityRaw = value;
    }
  } else if (function == pC_->motorDefJogVeloRO_) {
    if (value != status_.MotorConfigRO.motorDefJogVeloRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorDefJogVeloRaw = value;
    }
  } else if (function == pC_->motorDefJogAccRO_) {
    if (value != status_.MotorConfigRO.motorDefJogAccRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorDefJogAccRaw = value;
    }
  } else if (function == pC_->motorSDBDRO_) {
    if (value != status_.MotorConfigRO.motorSDBDRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorSDBDRaw = value;
    }
  } else if (function == pC_->motorRDBDRO_) {
    if (value != status_.MotorConfigRO.motorRDBDRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorRDBDRaw = value;
    }
  } else if (function == pC_->motorERESRO_) {
    if (value != status_.MotorConfigRO.motorERESRaw) {
      statusChanged_ = 1;
      status_.MotorConfigRO.motorERESRaw = value;
    }
  }
  // Call the base class method
  return pC_->setDoubleParam(axisNo_, function, value);
}   

/**
  * Sets the value for a string for this axis in the parameter library.
  * \param[in] function The function (parameter) number 
  * \param[in] value Value to set */
asynStatus asynMotorAxis::setStringParam(int function, const char *value)
{
  // Call the base class method
  return pC_->setStringParam(axisNo_, function, value);
}


void asynMotorAxis::updateMsgTxtFromDriver(const char *value)
{
  if (value && value[0]) {
    pC_->setIntegerParam(axisNo_,pC_->motorMessageIsFromDriver_, 1);
    setStringParam(pC_->motorMessageText_,value);
  } else {
    pC_->setIntegerParam(axisNo_,pC_->motorMessageIsFromDriver_, 0);
  }
}

/** Update the MsgTxt field*/
void asynMotorAxis::updateMsgTxtField()
{
  int motorMessageIsFromDriver;
  pC_->getIntegerParam(axisNo_,pC_->motorMessageIsFromDriver_, &motorMessageIsFromDriver);
  /* The driver has put in a message, keep it */
  if (motorMessageIsFromDriver) return;

  int motorStatusDone;
  int motorStatusCommsError;
  pC_->getIntegerParam(axisNo_,pC_->motorStatusCommsError_, &motorStatusCommsError);
  if (motorStatusCommsError) {
    setStringParam(pC_->motorMessageText_,"E: Communication");
    return;
  }
  pC_->getIntegerParam(axisNo_,pC_->motorStatusDone_, &motorStatusDone);

  if (motorStatusDone) {
    int motorStatusHighLimit;
    int motorStatusLowLimit;
    pC_->getIntegerParam(axisNo_,pC_->motorStatusHighLimit_, &motorStatusHighLimit);
    pC_->getIntegerParam(axisNo_,pC_->motorStatusLowLimit_, &motorStatusLowLimit);
    if (motorStatusHighLimit && motorStatusLowLimit)
      setStringParam(pC_->motorMessageText_,"I: Limit switches");
    else if (motorStatusHighLimit || motorStatusLowLimit)
      setStringParam(pC_->motorMessageText_,"I: Limit switch");
    else {
      int motorStatusProblem;
      int motorStatusHomed;
      int motorNotHomedProblem;
      double motorHighLimit;
      double motorLowLimit;
      pC_->getIntegerParam(axisNo_,pC_->motorStatusProblem_, &motorStatusProblem);
      pC_->getIntegerParam(axisNo_,pC_->motorNotHomedProblem_, &motorNotHomedProblem);
      pC_->getIntegerParam(axisNo_,pC_->motorStatusHomed_, &motorStatusHomed);
      if (motorStatusProblem) {
	if (!motorStatusHomed && (motorNotHomedProblem & MOTORNOTHOMEDPROBLEM_ERROR)) {
	  setStringParam(pC_->motorMessageText_,"E: Axis not homed");
	  return;
	}
        setStringParam(pC_->motorMessageText_,"E: Problem");
        return;
      }

      if (!motorStatusHomed && motorNotHomedProblem) {
        /* the "E: prefix should only be shown if the problem bit
           is set. Otherwise it is an info */
        setStringParam(pC_->motorMessageText_,"I: Axis not homed");
        return;
      }
      /* If both soft limits are defined, and both are != 0,
         check if the axis is below or above the range.
         motorLowLimit == motorHighLimit == 0 means "no limits"
         (e.g. a rotary axis) */
      if (!pC_->getDoubleParam(axisNo_, pC_->motorLowLimit_, &motorLowLimit) &&
          !pC_->getDoubleParam(axisNo_, pC_->motorHighLimit_, &motorHighLimit) &&
          (motorLowLimit != 0.0 || motorHighLimit != 0.0)) {
        double motorPosition;
        pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &motorPosition);
        if (motorPosition < motorLowLimit) {
          setStringParam(pC_->motorMessageText_,"I: Below soft limit");
          return;
        } else if (motorPosition > motorHighLimit) {
          setStringParam(pC_->motorMessageText_,"I: Above soft limit");
          return;
        }
      }
      int motorLatestCommand;
      int motorShowPowerOff;
      pC_->getIntegerParam(axisNo_,pC_->motorLatestCommand_, &motorLatestCommand);
      pC_->getIntegerParam(axisNo_,pC_->motorShowPowerOff_,  &motorShowPowerOff);
      if (motorShowPowerOff) {
        int motorStatusPowerOn;
        pC_->getIntegerParam(axisNo_,pC_->motorStatusPowerOn_,  &motorStatusPowerOn);
        if (motorStatusPowerOn) motorShowPowerOff = 0;
      }
      if (motorShowPowerOff)
        setStringParam(pC_->motorMessageText_,"I: PowerOff");
      else if (motorLatestCommand == LATEST_COMMAND_STOP)
        setStringParam(pC_->motorMessageText_,"I: Stop");
      else
        setStringParam(pC_->motorMessageText_," ");
    }
    return;
  }
  int motorStatusMoving;
  pC_->getIntegerParam(axisNo_,pC_->motorStatusMoving_, &motorStatusMoving);
  int motorLatestCommand;
  pC_->getIntegerParam(axisNo_,pC_->motorLatestCommand_, &motorLatestCommand);
  if (motorStatusMoving) {
    switch (motorLatestCommand) {
    case LATEST_COMMAND_HOMING:
        setStringParam(pC_->motorMessageText_,"I: Homing");
        break;
    case LATEST_COMMAND_MOVE_TO_HOME:
        setStringParam(pC_->motorMessageText_,"I: Moving home");
        break;
    case LATEST_COMMAND_MOVE_ABS:
        setStringParam(pC_->motorMessageText_,"I: Moving abs");
        break;
    case LATEST_COMMAND_MOVE_REL:
        setStringParam(pC_->motorMessageText_,"I: Moving rel");
        break;
    case LATEST_COMMAND_MOVE_VEL:
        setStringParam(pC_->motorMessageText_,"I: Moving vel");
        break;
    case LATEST_COMMAND_STOP:
        /* Stopped, but moving */
         pC_->setIntegerParam(axisNo_, pC_->motorLatestCommand_,
                              LATEST_COMMAND_UNDEFINED);
         /* fall through */
    default:
        setStringParam(pC_->motorMessageText_,"I: Moving");
    }
  } else {
    /* Not done, not moving. Show the latest command */
    switch (motorLatestCommand) {
    case LATEST_COMMAND_STOP:
        setStringParam(pC_->motorMessageText_,"I: Stop");
        break;
    case LATEST_COMMAND_HOMING:
        setStringParam(pC_->motorMessageText_,"I: Home");
        break;
    case LATEST_COMMAND_MOVE_TO_HOME:
        setStringParam(pC_->motorMessageText_,"I: Move home");
        break;
    case LATEST_COMMAND_MOVE_ABS:
        setStringParam(pC_->motorMessageText_,"I: Move abs");
        break;
    case LATEST_COMMAND_MOVE_REL:
        setStringParam(pC_->motorMessageText_,"I: Move rel");
        break;
    case LATEST_COMMAND_MOVE_VEL:
        setStringParam(pC_->motorMessageText_,"I: Move vel");
        break;
    case LATEST_COMMAND_UNDEFINED:
        setStringParam(pC_->motorMessageText_," ");
        break;
    default:
        setStringParam(pC_->motorMessageText_,"I: Unkown");
    }
  }
}

/** Calls the callbacks for any parameters that have changed for this axis in the parameter library.
  * This function takes special action if the aggregate MotorStatus structure has changed.
  * In that case it does callbacks on the asynGenericPointer interface, typically to devMotorAsyn. */  
asynStatus asynMotorAxis::callParamCallbacks()
{
  if (statusChanged_) {
    statusChanged_ = 0;
    updateMsgTxtField();
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
  static const char *functionName = "defineProfile";
  
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: axis=%d, numPoints=%d, positions[0]=%f\n",
            driverName, functionName, axisNo_, (int)numPoints, positions[0]);

  if (numPoints > pC_->maxProfilePoints_) return asynError;

  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &offset);
  status |= pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &direction);
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: axis=%d, status=%d, offset=%f direction=%d, resolution=%f\n",
            driverName, functionName, axisNo_, status, offset, direction, resolution);
  if (status) return asynError;
  if (resolution == 0.0) return asynError;
  
  // Convert to controller units
  scale = 1.0/resolution;
  if (direction != 0) scale = -scale;
  for (i=0; i<numPoints; i++) {
    profilePositions_[i] = (positions[i] - offset)*scale;
  }
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: axis=%d, scale=%f, offset=%f positions[0]=%f, profilePositions_[0]=%f\n",
            driverName, functionName, axisNo_, scale, offset, positions[0], profilePositions_[0]);
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
  //static const char *functionName = "readbackProfile";

  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &offset);
  status |= pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &direction);
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

/****************************************************************************/
/* The following functions are used by the automatic drive power control in the 
   base class poller in the asynMotorController class.*/

/**
 * Read the flag that indicates if the last poll was moving.
 */
int asynMotorAxis::getWasMovingFlag(void)
{
  return wasMovingFlag_;
}

/**
 * Set this to 1 if the previous poll indicated moving state 
 */
void asynMotorAxis::setWasMovingFlag(int wasMovingFlag)
{
  wasMovingFlag_ = wasMovingFlag;
}

/**
 * Read the flag that indicates if the drive should be automatically
 * disabled.
 */
int asynMotorAxis::getDisableFlag(void)
{
  return disableFlag_;
}

/**
 * Set this to 1 if the drive should be automatically disabled.
 */
void asynMotorAxis::setDisableFlag(int disableFlag)
{
  disableFlag_ = disableFlag;
}

/**
 * Read the time in seconds of the last end of move.
 */
double asynMotorAxis::getLastEndOfMoveTime(void)
{
  return lastEndOfMoveTime_;
}

/**
 * Set this to the current time at the end of a move.
 */
void asynMotorAxis::setLastEndOfMoveTime(double time)
{
  lastEndOfMoveTime_ = time;
}


/********************************************************************/

