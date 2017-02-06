/*
FILENAME... 874xMotorDriver.cpp
USAGE...    Motor driver support for the NewFocus 874x series motor controller

Based on ACRMotorDriver.cpp by:
Mark Rivers
March 4, 2011

== Modifications ==
2015-12-01 - Wayne Lewis - Modify for NewFocus 874x series
*/

/*
 * TODO: Extend for 8743 closed loop functionality
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "874xMotorDriver.h"

static const char *driverName = "nf874xMotorDriver";

/** Creates a new nf874xController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] nf874xPortName      The name of the drvAsynIPPPort that was created previously to connect to the nf874x controller 
  * \param[in] numAxes           The number of axes that this controller supports. Create one extra axis to allow for base 1 indexing of NewFocus controllers. 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
nf874xController::nf874xController(const char *portName, const char *nf874xPortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  //:  asynMotorController(portName, numAxes, NUM_nf874x_PARAMS, 
  :  asynMotorController(portName, numAxes+1, 0,
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  static const char *functionName = "nf874xController";

  /* Connect to nf874x controller */
  status = pasynOctetSyncIO->connect(nf874xPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to nf874x controller\n",
      driverName, functionName);
  }
  epicsThreadSleep(0.5);
  // Create the axis objects
  // Axis 0 will remain unused. This allows consistent axis numbering with 
  // NewFocus convention.
  for (axis=1; axis<=numAxes; axis++) {
    new nf874xAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new nf874xController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] nf874xPortName       The name of the drvAsynIPPPort that was created previously to connect to the nf874x controller 
  * \param[in] numAxes           The number of axes that this controller supports.
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int nf874xCreateController(const char *portName, const char *nf874xPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  new nf874xController(portName, nf874xPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void nf874xController::report(FILE *fp, int level)
{
  fprintf(fp, "nf874x motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an nf874xMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
nf874xAxis* nf874xController::getAxis(asynUser *pasynUser)
{
  return static_cast<nf874xAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an nf874xMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
nf874xAxis* nf874xController::getAxis(int axisNo)
{
  return static_cast<nf874xAxis*>(asynMotorController::getAxis(axisNo));
}


/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus nf874xController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  nf874xAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  
  status = asynMotorController::writeInt32(pasynUser, value);
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%d\n", 
        driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%d\n", 
        driverName, functionName, function, value);
  return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * For all other functions it calls asynMotorController::writeFloat64.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus nf874xController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  nf874xAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeFloat64";
  
  
  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam(pAxis->axisNo_, function, value);
  
  status = asynMotorController::writeFloat64(pasynUser, value);
  
  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%f\n", 
        driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%f\n", 
        driverName, functionName, function, value);
  return status;
}

// These are the nf874xAxis methods

/** Creates a new nf874xAxis object.
  * \param[in] pC Pointer to the nf874xController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
nf874xAxis::nf874xAxis(nf874xController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  sprintf(axisName_, "%d", axisNo);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void nf874xAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_ );
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


asynStatus nf874xAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;

  sprintf(pC_->outString_, "%s AC %f", axisName_, acceleration);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s VA %f", axisName_, maxVelocity);
  status = pC_->writeController();
  if (relative) {
    sprintf(pC_->outString_, "%s PR %f", axisName_, position);
    status = pC_->writeController();
  } else {
    sprintf(pC_->outString_, "%s PA %f", axisName_, position);
    status = pC_->writeController();
  }
  return status;
}

asynStatus nf874xAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;

  sprintf(pC_->outString_, "%s AC %f", axisName_, acceleration);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s VA %f", axisName_, acceleration);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s DH", axisName_);
  status = pC_->writeController();
  return status;
}

asynStatus nf874xAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  double speed=maxVelocity;
  int forwards=1;

  if (speed < 0) {
    speed = -speed;
    forwards = 0;
  }
  sprintf(pC_->outString_, "%s AC %f", axisName_, acceleration);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s VA %f", axisName_, speed);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s MV %s", axisName_, forwards ? "+" : "-");
  status = pC_->writeController();
  return status;
}

asynStatus nf874xAxis::stop(double acceleration )
{
  asynStatus status;

  sprintf(pC_->outString_, "%s ST", axisName_);
  status = pC_->writeController();
  return status;
}

asynStatus nf874xAxis::setPosition(double position)
{
  asynStatus status;

  sprintf(pC_->outString_, "%s DH %f", axisName_, position);
  status = pC_->writeController();
  return status;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus nf874xAxis::poll(bool *moving)
{ 
  int done;
  asynStatus comStatus;

  // Read the current encoder position
  sprintf(pC_->outString_, "%s TP?", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  encoderPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_,encoderPosition_);

  // Read the current theoretical position
  setDoubleParam(pC_->motorPosition_, encoderPosition_);

  // Read the current moving status
  sprintf(pC_->outString_, "%s MD?", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  done = atoi(pC_->inString_);
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg nf874xCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg nf874xCreateControllerArg1 = {"nf874x port name", iocshArgString};
static const iocshArg nf874xCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg nf874xCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg nf874xCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const nf874xCreateControllerArgs[] = {&nf874xCreateControllerArg0,
                                                           &nf874xCreateControllerArg1,
                                                           &nf874xCreateControllerArg2,
                                                           &nf874xCreateControllerArg3,
                                                           &nf874xCreateControllerArg4};
static const iocshFuncDef nf874xCreateControllerDef = {"nf874xCreateController", 5, nf874xCreateControllerArgs};
static void nf874xCreateContollerCallFunc(const iocshArgBuf *args)
{
  nf874xCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void nf874xMotorRegister(void)
{
  iocshRegister(&nf874xCreateControllerDef, nf874xCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(nf874xMotorRegister);
}
