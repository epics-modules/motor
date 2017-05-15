/*
FILENAME... TCPsimMotorAxis.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <epicsThread.h>

#include "TCPsimMotor.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

/***************
      1 MV <position>   # Absolute move (counts)
      1 MR <displacement> # Relative move (counts)
      1 JOG <velocity>    # Jog (counts/s, signed)
      //1 POS <position>    # Set position (counts)
      //1 ACC <acceleration>  # Set acceleration (counts/s/s)
      1 VEL <velocity>    # Set velocity (counts/s)
      //1 BAS <base_velocity> # Set base velocity (counts/s)
      1 AB      # Abort motion
      1 POS?      # Query position (returns: counts)
      1 ST?     # Query status (returns: integer)
      //1 ACC?      # Query accel (returns: counts/s/s)
      //1 VEL?      # Query velocity (returns: counts/s)

      //1 LL <position>   # Set low limit (counts)
      //1 HL <position>   # Set high limit (counts)
      //1 LL?       # Query low limit (returns: counts)
      //1 HL?       # Query high limit (returns: counts)

      Status bit definitions:
      See below, which ones are used.

***************/
//#define STATUS_BIT_DIRECTION       0x1
#define STATUS_BIT_DONE            0x2
#define STATUS_BIT_MOVING          0x4
#define STATUS_BIT_LIMIT_POS       0x8
#define STATUS_BIT_LIMIT_NEG       0x10
//#define STATUS_BIT_HOMING         0x20
#define STATUS_BIT_HSIGNAL        0x40
#define STATUS_BIT_HOMED          0x80
#define STATUS_BIT_ERROR         0x100

//
// These are the TCPsimMotorAxis methods
//

/** Creates a new TCPsimMotorAxis object.
  * \param[in] pC Pointer to the TCPsimMotorController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
  *
  *
  * Initializes register numbers, etc.
  */
TCPsimMotorAxis::TCPsimMotorAxis(TCPsimMotorController *pC, int axisNo,
           int axisFlags, const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  memset(&drvlocal, 0, sizeof(drvlocal));
  drvlocal.cfg.axisFlags = axisFlags;
  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str = "encoder=";

    char *pOptions = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, encoder_is_str, strlen(encoder_is_str))) {
        pThisOption += strlen(encoder_is_str);
        drvlocal.cfg.externalEncoderStr = strdup(pThisOption);
        setIntegerParam(pC->motorStatusHasEncoder_, 1);
      }
    }
    free(pOptions);
  }

  pC_->wakeupPoller();
}


extern "C" int TCPsimMotorCreateAxis(const char *TCPsimMotorName, int axisNo,
              int axisFlags, const char *axisOptionsStr)
{
  TCPsimMotorController *pC;

  pC = (TCPsimMotorController*) findAsynPortDriver(TCPsimMotorName);
  if (!pC)
  {
    printf("Error port %s not found\n", TCPsimMotorName);
    return asynError;
  }
  pC->lock();
  new TCPsimMotorAxis(pC, axisNo, axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
  * \param[in] AsynStatus status
  *
  * Sets the dirty bits
  */
void TCPsimMotorAxis::handleStatusChange(asynStatus newStatus)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
            "TCPsimMotorAxis::handleStatusChange status=%s (%d)\n",
            pasynManager->strStatus(newStatus), (int)newStatus);
  if (newStatus == asynSuccess) {
    if (drvlocal.cfg.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS) {
  /* Enable the amplifier when the axis is created,
     but wait until we have a connection to the controller.
     After we lost the connection, Re-enable the amplifier
     See AMPLIFIER_ON_FLAG */
      amplifierPercentage(100);
    }
  }
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void TCPsimMotorAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
 }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


/** Writes a command to the axis, and expects a logical ack from the controller
  * Outdata is in pC_->outString_
  * Indata is in pC_->inString_
  * The communiction is logged ASYN_TRACE_INFO
  *
  * When the communictaion fails ot times out, writeReadOnErrorDisconnect() is called
  */
asynStatus TCPsimMotorAxis::writeReadACK(void)
{
  asynStatus status = pC_->writeReadOnErrorDisconnect();
  switch (status) {
    case asynError:
      return status;
    case asynSuccess:
      if (!strstr(pC_->inString_, "OK")) {
        status = asynError;
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s in=%s return=%s (%d)\n",
                  pC_->outString_, pC_->inString_,
                  pasynManager->strStatus(status), (int)status);
        return status;
      }
    default:
      break;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "out=%s in=%s status=%s (%d) oldPosition=%d\n",
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(status), (int)status,
            drvlocal.lastpoll.motorPosition);
  return status;
}


/** Sets an integer or boolean value on an axis
  * the values in the controller must be updated
  * \param[in] name of the variable to be updated
  * \param[in] value the (integer) variable to be updated
  *
  */
asynStatus TCPsimMotorAxis::setValueOnAxis(const char* var)
{
  sprintf(pC_->outString_, "%d %s", axisNo_, var);
  return writeReadACK();
}

/** Sets an integer or boolean value on an axis
  * the values in the controller must be updated
  * \param[in] name of the variable to be updated
  * \param[in] value the (integer) variable to be updated
  *
  */
asynStatus TCPsimMotorAxis::setValueOnAxis(const char* var, int value)
{
  sprintf(pC_->outString_, "%d %s %d", axisNo_, var, value);
  return writeReadACK();
}

/** Gets an integer or boolean value from an axis
  * \param[in] name of the variable to be retrieved
  * \param[in] pointer to the integer result
  *
  */
asynStatus TCPsimMotorAxis::getValueFromAxis(const char* var, int *value)
{
  asynStatus comStatus;
  int res;

  sprintf(pC_->outString_, "%d %s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus) {
    return comStatus;
  } else {
    int nvals = sscanf(pC_->inString_, "%d", &res);
    if (nvals != 1) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "nvals=%d command=\"%s\" response=\"%s\"\n",
                nvals, pC_->outString_, pC_->inString_);
      return asynError;
    }
  }
  *value = res;
  return asynSuccess;
}

/** Move the axis to a position, either absolute or relative
  * \param[in] position in mm
  * \param[in] relative (0=absolute, otherwise relative)
  * \param[in] minimum velocity, mm/sec
  * \param[in] maximum velocity, mm/sec
  * \param[in] acceleration, seconds to maximum velocity
  *
  */
asynStatus TCPsimMotorAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "move() position=%f relative=%d minVelocity=%f maxVelocity=%f acceleration=%f\n",
            position, relative, minVelocity, maxVelocity, acceleration);

  if (status == asynSuccess) status = setValueOnAxis("VEL", NINT(maxVelocity));
  if (status == asynSuccess) status = setValueOnAxis(relative ? "MR" : "MA", NINT(position));

  return status;
}


/** Home the motor, search the home position
  * \param[in] minimum velocity, mm/sec
  * \param[in] maximum velocity, mm/sec
  * \param[in] acceleration, seconds to maximum velocity
  * \param[in] forwards (0=backwards, otherwise forwards)
  *
  */
asynStatus TCPsimMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;

  if ((drvlocal.cfg.axisFlags & AMPLIFIER_ON_FLAG_WHEN_HOMING) &&
      (status == asynSuccess)) status = amplifierPercentage(100);
  if (status == asynSuccess) status = setValueOnAxis("VEL", NINT(maxVelocity));
  if (status == asynSuccess) status = setValueOnAxis("HOM", forwards);
  return status;
}


/** jog the the motor, search the home position
  * \param[in] minimum velocity, mm/sec (not used)
  * \param[in] maximum velocity, mm/sec (positive or negative)
  * \param[in] acceleration, seconds to maximum velocity
  *
  */
asynStatus TCPsimMotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  if (status == asynSuccess) setValueOnAxis("JOG", NINT(maxVelocity));
  return status;
}


/** Enable the amplifier on an axis
  *
  */
asynStatus TCPsimMotorAxis::amplifierPercentage(int percent)
{
  return setValueOnAxis("POW", percent);
}

/** Stop the axis, called by motor Record
  *
  */
asynStatus TCPsimMotorAxis::stop(double acceleration )
{
  asynStatus status = setValueOnAxis("AB");
  if (status == asynSuccess) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
        "stop(%d)\n",  axisNo_);
  }
  return status;
}


/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status,
  * and the drive power-on status.
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus TCPsimMotorAxis::poll(bool *moving)
{
  int        nowMoving = 0;
  int        axis_status;
  int        motorPosition;
  asynStatus comStatus;
  comStatus = getValueFromAxis("ST", &axis_status);
  if (comStatus) goto badpollall;
  if (drvlocal.lastpoll.axis_status != axis_status) {
    int status = axis_status;

    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) line=%d status=0x%x %s%s%s%s%s%s%s\n",
              axisNo_, __LINE__, status,
              status & STATUS_BIT_DONE ?      "DONE  " : "      ",
              status & STATUS_BIT_MOVING ?    "MOVIN " : "      ",
              status & STATUS_BIT_LIMIT_POS ? "LIM-P " : "      ",
              status & STATUS_BIT_LIMIT_NEG ? "LIM-N " : "      ",
              status & STATUS_BIT_HSIGNAL ?   "HOME  " : "      ",
              status & STATUS_BIT_HOMED   ?   "HOMED " : "      ",
              status & STATUS_BIT_ERROR   ?   "ERROR " : "      ");
    drvlocal.lastpoll.axis_status = axis_status;
  }
  setIntegerParam(pC_->motorStatusProblem_, 0); //st_axis_status.status & STATUS_BITS_DISABLE);
  setIntegerParam(pC_->motorStatusAtHome_, axis_status & STATUS_BIT_HSIGNAL);
  setIntegerParam(pC_->motorStatusLowLimit_, axis_status & STATUS_BIT_LIMIT_NEG);
  setIntegerParam(pC_->motorStatusHighLimit_, axis_status & STATUS_BIT_LIMIT_POS);
  setIntegerParam(pC_->motorStatusHomed_, axis_status & STATUS_BIT_HOMED);
  //setIntegerParam(pC_->motorStatusPowerOn_, axis_status & STATUS_BIT_POWERON);

  /* Phase 2: read the Axis (readback) position */
  comStatus = getValueFromAxis("POS", &motorPosition);
  if (comStatus) goto badpollall;
  /* Use previous motorPosition and current motorPosition to calculate direction.*/
  if (motorPosition > drvlocal.lastpoll.motorPosition) {
    setIntegerParam(pC_->motorStatusDirection_, 1);
  } else if (motorPosition < drvlocal.lastpoll.motorPosition) {
    setIntegerParam(pC_->motorStatusDirection_, 0);
  }
  drvlocal.lastpoll.motorPosition = motorPosition;
  setDoubleParam(pC_->motorPosition_, (double)motorPosition);

  /* Phase 3: is motor moving */
  if (axis_status & STATUS_BIT_MOVING) {
    nowMoving = 1;
  }

  setIntegerParam(pC_->motorStatusMoving_, nowMoving);
  setIntegerParam(pC_->motorStatusDone_, !nowMoving);
  *moving = nowMoving ? true : false;

  callParamCallbacks();
  return asynSuccess;

badpollall:
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
            "out=%s in=%s return=%s (%d)\n",
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(comStatus), (int)comStatus);

  setIntegerParam(pC_->motorStatusProblem_, 1);
  callParamCallbacks();
  return asynError;
}

asynStatus TCPsimMotorAxis::setIntegerParam(int function, int value)
{
  asynStatus status;
  if (function == pC_->motorClosedLoop_) {
    if (drvlocal.cfg.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
      (void)amplifierPercentage(value ? 100 : 0);
    }
  }

  //Call base class method
  status = asynMotorAxis::setIntegerParam(function, value);
  return status;
}

/** Set a floating point parameter on the axis
  * \param[in] function, which parameter is updated
  * \param[in] value, the new value
  *
  * When the IOC starts, we will send the soft limits to the controller.
  * When a soft limit is changed, and update is send them to the controller.
  */
asynStatus TCPsimMotorAxis::setDoubleParam(int function, double value)
{
  asynStatus status;
  if (function == pC_->motorMoveRel_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorMoveRel_)=%f\n", value);
  } else if (function == pC_->motorMoveAbs_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorMoveAbs_)=%f\n", value);
  } else if (function == pC_->motorMoveVel_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorMoveVel_)=%f\n", value);
  } else if (function == pC_->motorHome_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParmotorHome__)=%f\n", value);
  } else if (function == pC_->motorStop_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParmotorStop_)=%f\n", value);
  } else if (function == pC_->motorVelocity_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(mmotorVelocity_=%f\n", value);
  } else if (function == pC_->motorVelBase_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorVelBase_)=%f\n", value);
  } else if (function == pC_->motorAccel_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParamotorAccel_l_)=%f\n", value);
#ifdef PRINT_POLLED_MOTORPOSITION
  } else if (function == pC_->motorPosition_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(mmotorPosition_=%f\n", value);
  } else if (function == pC_->motorEncoderPosition_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorMovmotorEncoderPosition_=%f\n", value);
#endif
  } else if (function == pC_->motorDeferMoves_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motmotorDeferMoves_=%f\n", value);
  } else if (function == pC_->motorMoveToHome_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motmotorMoveToHome_=%f\n", value);
  } else if (function == pC_->motorResolution_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorResolution_=%f\n", value);
  } else if (function == pC_->motorEncoderRatio_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorEncoderRatio_)=%f\n", value);
  } else if (function == pC_->motorPGain_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoublmotorPGain_oveRel_)=%f\n", value);
  } else if (function == pC_->motorIGain_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoublmotorIGain_oveRel_)=%f\n", value);
  } else if (function == pC_->motorDGain_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoublmotorDGain_oveRel_)=%f\n", value);
      /* Limits handled above */
  } else if (function == pC_->motorClosedLoop_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParamotorClosedLoop_l_)=%f\n", value);

#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(momotorPowerAutoOnOff_%f\n", value);
#endif
#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorPowerOnDelay_)=%f\n", value);
#endif
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(mmotorPowerOffDelay_=%f\n", value);
#endif
#ifdef motorPowerOffFractionString
  } else if (function == pC_->motorPowerOffFraction_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motomotorPowerOffFraction_=%f\n", value);
#endif
#ifdef motorPostMoveDelayString
  } else if (function == pC_->motorPostMoveDelay_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(mmotorPostMoveDelay_=%f\n", value);
#endif
  } else if (function == pC_->motorStatus_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoublemotorStatus_veRel_)=%f\n", value);
  } else if (function == pC_->motorUpdateStatus_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorUpdateStatus_)=%f\n", value);
#ifdef motorRecResolutionString
  } else if (function == pC_->motorRecResolution_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorRecResolution_)=%f\n", value);
#endif
#ifdef motorRecOffsetString
  } else if (function == pC_->motorRecOffset_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorRecOffset_)=%f\n", value);
#endif
#ifdef motorRecDirectionString
  } else if (function == pC_->motorRecDirection_) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "setDoubleParam(motorRecDirection_)=%f\n", value);
#endif
  }

  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}
