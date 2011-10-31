/*
FILENAME... ACRMotorDriver.cpp
USAGE...    Motor driver support for the Parker ACR series of controllers, including the Aries.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "ACRMotorDriver.h"
#include <epicsExport.h>

#define CtlY 25

#define ACR_TIMEOUT 1.0

static const char *driverName = "ACRMotorDriver";

/** Creates a new ACRController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ACRPortName       The name of the drvAsynIPPPort that was created previously to connect to the ACR controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
ACRController::ACRController(const char *portName, const char *ACRPortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_ACR_PARAMS, 
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask, 
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  ACRAxis *pAxis;
  static const char *functionName = "ACRController";

  binaryInReg_  = 4096;
  binaryOutReg_ = 4097;
  
  // Create controller-specific parameters
  createParam(ACRJerkString,         asynParamFloat64,       &ACRJerk_);
  createParam(ACRReadBinaryIOString, asynParamInt32,         &ACRReadBinaryIO_);
  createParam(ACRBinaryInString,     asynParamUInt32Digital, &ACRBinaryIn_);
  createParam(ACRBinaryOutString,    asynParamUInt32Digital, &ACRBinaryOut_);
  createParam(ACRBinaryOutRBVString, asynParamUInt32Digital, &ACRBinaryOutRBV_);

  /* Connect to ACR controller */
  status = pasynOctetSyncIO->connect(ACRPortName, 0, &pasynUserACR_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to ACR controller\n",
      driverName, functionName);
  }
  // Turn off command echoing
  sprintf(outString_, "BIT1792=0");
  writeController();
   // Turn off command prompt
  sprintf(outString_, "BIT1794=1");
  writeController();
  // Wait a short while so that any responses to the above commands have time to arrive so we can flush
  // them in the next writeReadController()
  epicsThreadSleep(0.5);
  // Read the binary I/O registers once
  readBinaryIO();
  // Set the output=output readback so bi records reflect current state
  setUIntDigitalParam(0, ACRBinaryOut_, binaryOutRBV_, 0xFFFFFFFF);
  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new ACRAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new ACRController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ACRPortName       The name of the drvAsynIPPPort that was created previously to connect to the ACR controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int ACRCreateController(const char *portName, const char *ACRPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  ACRController *pACRController
    = new ACRController(portName, ACRPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pACRController = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void ACRController::report(FILE *fp, int level)
{
  int axis;
  ACRAxis *pAxis;

  fprintf(fp, "ACR motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    fprintf(fp, "  binary input = 0x%x\n", binaryIn_);
    fprintf(fp, "  binary output readback = 0x%x\n", binaryOutRBV_);
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      fprintf(fp, "  axis %d\n"
              "  pulsesPerUnit_ = %f\n"
              "    encoder position=%f\n"
              "    theory position=%f\n"
              "    limits=0x%x\n"
              "    flags=0x%x\n", 
              pAxis->axisNo_, pAxis->pulsesPerUnit_, 
              pAxis->encoderPosition_, pAxis->theoryPosition_,
              pAxis->currentLimits_, pAxis->currentFlags_);
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an ACRMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ACRAxis* ACRController::getAxis(asynUser *pasynUser)
{
  return static_cast<ACRAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an ACRMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
ACRAxis* ACRController::getAxis(int axisNo)
{
  return static_cast<ACRAxis*>(asynMotorController::getAxis(axisNo));
}


/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorSetClosedLoop_ then it turns the drive power on or off.
  * If the function is ACRReadBinaryIO_ then it reads the binary I/O registers on the controller.
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus ACRController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ACRAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  
  if (function == motorSetClosedLoop_)
  {
    sprintf(outString_, "DRIVE %s %s", value ? "ON":"OFF", pAxis->axisName_);
    writeController();
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
      "%s:%s: %s setting closed loop=%d on %s\n",
      driverName, functionName, this->portName, value, pAxis->axisName_);
  } 
  else if (function == ACRReadBinaryIO_)
  {
    readBinaryIO();
  }
  else 
  {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }
  
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
  * If the function is ACRJerk_ it sets the jerk value in the controller.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * For all other functions it calls asynMotorController::writeFloat64.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ACRController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ACRAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeFloat64";
  
  
  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam(pAxis->axisNo_, function, value);
  
  if (function == ACRJerk_)
  {
    sprintf(outString_, "%s JOG JRK %f", pAxis->axisName_, value);
    status = writeController();
    
  } else {
    /* Call base class method */
    status = asynMotorController::writeFloat64(pasynUser, value);
  }
  
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

/** Called when asyn clients call pasynUInt32Digital->write().
  * Writes a single bit to one of the ACR binary output registers. 
  * This function is limited to writing a single bit, because we use the BIT command.
  * It writes to least significant bit that is set in the mask.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write.
  * \param[in] mask Mask value to use when writinging the value. */
asynStatus ACRController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  int bit, tmask=0x1;
  asynStatus status;
  //static const char *functionName = "writeUInt32Digital";
  
  for (bit=0; bit<32; bit++) {
    if (mask & tmask) break;
    tmask = tmask << 1;
  }
  sprintf(outString_, "BIT %d=%d", 32+bit, value);
  status = writeController();
  // Read the I/O back
  readBinaryIO();

  return(status);
}

/** Reads the binary input and binary output registers on the ACR.
  * Sets the values in the parameter library.
  * Keeps track of which bits have changed.
  * Calls any registered callbacks for this pasynUser->reason and address. */ 
asynStatus ACRController::readBinaryIO()
{
  asynStatus status;

  // Read the binary inputs
  sprintf(outString_, "?P%d", binaryInReg_);
  status = writeReadController();
  if (!status) {
    binaryIn_ = atoi(inString_);
    setUIntDigitalParam(0, ACRBinaryIn_, binaryIn_, 0xFFFFFFFF);
  }

  // Read the binary outputs
  sprintf(outString_, "?P%d", binaryOutReg_);
  status = writeReadController();
  if (!status) {
    binaryOutRBV_  = atoi(inString_);
    setUIntDigitalParam(0, ACRBinaryOutRBV_, binaryOutRBV_, 0xFFFFFFFF);
  }
  callParamCallbacks(0);
  return status;
}

/** Writes a string to the ACR controller.
  * Calls writeController() with a default location of the string to write and a default timeout. */ 
asynStatus ACRController::writeController()
{
  return writeController(outString_, ACR_TIMEOUT);
}

/** Writes a string to the ACR controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus ACRController::writeController(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeController";
  
  status = pasynOctetSyncIO->write(pasynUserACR_, output,
                                   strlen(output), timeout, &nwrite);
                                  
  return status ;
}

/** Writes a string to the ACR controller and reads the response.
  * Calls writeReadController() with default locations of the input and output strings
  * and default timeout. */ 
asynStatus ACRController::writeReadController()
{
  size_t nread;
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, ACR_TIMEOUT);
}

/** Writes a string to the ACR controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus ACRController::writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadController";
  
  status = pasynOctetSyncIO->writeRead(pasynUserACR_, output,
                                       strlen(output), input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);
                        
  return status;
}


// These are the ACRAxis methods

/** Creates a new ACRAxis object.
  * \param[in] pC Pointer to the ACRController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
ACRAxis::ACRAxis(ACRController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;
  
  sprintf(axisName_, "AXIS%d", axisNo);
  encoderPositionReg_ = 12290 + 256*axisNo;
  theoryPositionReg_  = 12294 + 256*axisNo;
  limitsReg_          = 4600  + axisNo;
  flagsReg_           = 4120  + axisNo;
  // Get the number of pulses per unit on this axis
  sprintf(pC->outString_, "%s PPU", axisName_);
  status = pC->writeReadController();
  if (status) {
    setIntegerParam(pC->motorStatusProblem_, 1);
  } else {
    pulsesPerUnit_ = atof(pC->inString_);
    // We assume servo motor with encoder for now
    setIntegerParam(pC->motorStatusGainSupport_, 1);
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
  }
  callParamCallbacks();
}

asynStatus ACRAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveAxis";

  sprintf(pC_->outString_, "%s JOG ACC %f", axisName_, acceleration/pulsesPerUnit_);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s JOG VEL %f", axisName_, maxVelocity/pulsesPerUnit_);
  status = pC_->writeController();
  // Note, the CtlY being send below clears the kill for all axes, in case they had hit a limit, etc.
  if (relative) {
    sprintf(pC_->outString_, "%c:%s JOG INC %f", CtlY, axisName_, position/pulsesPerUnit_);
    status = pC_->writeController();
  } else {
    sprintf(pC_->outString_, "%c:%s JOG ABS %f", CtlY, axisName_, position/pulsesPerUnit_);
    status = pC_->writeController();
  }
  return status;
}

asynStatus ACRAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "homeAxis";

  sprintf(pC_->outString_, "%s JOG ACC %f", axisName_, acceleration/pulsesPerUnit_);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s JOG VEL %f", axisName_, maxVelocity/pulsesPerUnit_);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%c:%s JOG HOME %d", CtlY, axisName_, forwards ? 1 : -1);
  status = pC_->writeController();
  return status;
}

asynStatus ACRAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  double speed=maxVelocity;
  int forwards=1;
  // static const char *functionName = "moveVelocityAxis";

  if (speed < 0) {
    speed = -speed;
    forwards = 0;
  }
  sprintf(pC_->outString_, "%s JOG ACC %f", axisName_, acceleration/pulsesPerUnit_);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s JOG VEL %f", axisName_, speed/pulsesPerUnit_);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%c:%s JOG %s", CtlY, axisName_, forwards ? "FWD" : "REV");
  status = pC_->writeController();
  return status;
}

asynStatus ACRAxis::stop(double acceleration )
{
  asynStatus status;
  static const char *functionName = "stopAxis";

  sprintf(pC_->outString_, "%s JOG OFF", axisName_);
  status = pC_->writeController();

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
    "%s:%s: Set axis %d to stop, status=%d\n",
    driverName, functionName, axisNo_, status);
  return status;
}

asynStatus ACRAxis::setPosition(double position)
{
  asynStatus status;

  sprintf(pC_->outString_, "%s RES %f", axisName_, position/pulsesPerUnit_);
  status = pC_->writeController();
  sprintf(pC_->outString_, "%s JOG REN", axisName_);
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
asynStatus ACRAxis::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int limit;
  asynStatus comStatus;

  // Read the current encoder position
  sprintf(pC_->outString_, "?P%d", encoderPositionReg_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  encoderPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_,encoderPosition_);

  // Read the current theoretical position
  sprintf(pC_->outString_, "?P%d", theoryPositionReg_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  theoryPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, theoryPosition_);

  // Read the current flags
  sprintf(pC_->outString_, "?P%d", flagsReg_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  currentFlags_ = atoi(pC_->inString_);
  done = (currentFlags_ & 0x1000000)?0:1;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // Read the current limit status
  sprintf(pC_->outString_, "?P%d", limitsReg_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  currentLimits_ = atoi(pC_->inString_);
  limit = (currentLimits_ & 0x1)?1:0;
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (currentLimits_ & 0x2)?1:0;
  setIntegerParam(pC_->motorStatusLowLimit_, limit);
  limit = (currentLimits_ & 0x4)?1:0;
  setIntegerParam(pC_->motorStatusAtHome_, limit);

  // Read the drive power on status
  sprintf(pC_->outString_, "DRIVE %s", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  driveOn = strstr(pC_->inString_, "ON") ? 1:0;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg ACRCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ACRCreateControllerArg1 = {"ACR port name", iocshArgString};
static const iocshArg ACRCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg ACRCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ACRCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const ACRCreateControllerArgs[] = {&ACRCreateControllerArg0,
                                                           &ACRCreateControllerArg1,
                                                           &ACRCreateControllerArg2,
                                                           &ACRCreateControllerArg3,
                                                           &ACRCreateControllerArg4};
static const iocshFuncDef ACRCreateControllerDef = {"ACRCreateController", 5, ACRCreateControllerArgs};
static void ACRCreateContollerCallFunc(const iocshArgBuf *args)
{
  ACRCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void ACRMotorRegister(void)
{
  iocshRegister(&ACRCreateControllerDef, ACRCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(ACRMotorRegister);
}
