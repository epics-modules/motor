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

#include "asynMotorDriver.h"
#include "ACRMotorDriver.h"
#include <epicsExport.h>

#define CtlY 25

#define ACR_TIMEOUT 1.0

static const char *driverName = "ACRMotorDriver";

ACRController::ACRController(const char *portName, const char *ACRPortName, int numAxes, int movingPollPeriod, int idlePollPeriod)
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

  idlePollPeriod_ = idlePollPeriod/1000.;
  movingPollPeriod_ = movingPollPeriod/1000.;
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
  for (axis=0; axis<numAxes; axis++) {
    pAxis  = new ACRAxis(this, axis);
    // Get the number of pulses per unit on this axis
    sprintf(outString_, "%s PPU", pAxis->axisName_);
    status = writeReadController();
    if (status) {
      setIntegerParam(axis, motorStatusProblem_, 1);
    } else {
      pAxis->pulsesPerUnit_ = atof(inString_);
      // We assume servo motor with encoder for now
      setIntegerParam(axis, motorStatusGainSupport_, 1);
      setIntegerParam(axis, motorStatusHasEncoder_, 1);
    }
    callParamCallbacks(axis);
  }

  startPoller(movingPollPeriod/1000., idlePollPeriod/1000.);
}


/** Configuration command, called directly or from iocsh */
extern "C" int ACRCreateController(const char *portName, const char *ACRPortName, int numAxes, int movingPollPeriod, int idlePollPeriod)
{
  ACRController *pACRController
    = new ACRController(portName, ACRPortName, numAxes, movingPollPeriod, idlePollPeriod);
  pACRController = NULL;
  return(asynSuccess);
}

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

ACRAxis* ACRController::getAxis(asynUser *pasynUser)
{
  return static_cast<ACRAxis*>(asynMotorController::getAxis(pasynUser));
}

ACRAxis* ACRController::getAxis(int axisNo)
{
  return static_cast<ACRAxis*>(asynMotorController::getAxis(axisNo));
}


asynStatus ACRController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ACRAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  
  if (function == motorDeferMoves_)
  {
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
      "%s:%s: %sing Deferred Move flag on driver %s\n",
      value != 0.0?"Sett":"Clear",
      driverName, functionName, this->portName);
  } 
  else if (function == motorSetClosedLoop_)
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
  callParamCallbacks(pAxis->axisNo_);
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

asynStatus ACRController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  // This function is limited to writing a single bit, because we use the BIT command.
  // It writes to least significant bit that is set in the mask
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

asynStatus ACRController::readBinaryIO()
{
  asynStatus status;
  epicsUInt32 newBits, changedBits;

  // Read the binary inputs
  sprintf(outString_, "?P%d", binaryInReg_);
  status = writeReadController();
  if (!status) {
    newBits = atoi(inString_);
    changedBits = newBits ^ binaryIn_;
    binaryIn_ = newBits;
    setUIntDigitalParam(0, ACRBinaryIn_, binaryIn_, 0xFFFFFFFF);
    setUInt32DigitalInterrupt(0, ACRBinaryIn_, changedBits, interruptOnBoth);
  }

  // Read the binary outputs
  sprintf(outString_, "?P%d", binaryOutReg_);
  status = writeReadController();
  if (!status) {
    newBits = atoi(inString_);
    changedBits = newBits ^ binaryOutRBV_;
    binaryOutRBV_ = newBits;
    setUIntDigitalParam(0, ACRBinaryOutRBV_, binaryOutRBV_, 0xFFFFFFFF);
    setUInt32DigitalInterrupt(0, ACRBinaryOutRBV_, changedBits, interruptOnBoth);
  }
  callParamCallbacks(0);
  return status;
}

asynStatus ACRController::triggerProfile(asynUser *pasynUser)
{
  return asynError;
}

asynStatus ACRController::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger)
{
  return asynError;
}

asynStatus ACRController::writeController()
{
  return writeController(outString_, ACR_TIMEOUT);
}

asynStatus ACRController::writeController(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeController";
  
  status = pasynOctetSyncIO->write(pasynUserACR_, output,
                                   strlen(output), timeout, &nwrite);
                                  
  return status ;
}

asynStatus ACRController::writeReadController()
{
  size_t nread;
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, ACR_TIMEOUT);
}

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

ACRAxis::ACRAxis(ACRController *pController, int axisNo)
  : asynMotorAxis(pController, axisNo)
{
  sprintf(axisName_, "AXIS%d", axisNo);
  encoderPositionReg_ = 12290 + 256*axisNo;
  theoryPositionReg_  = 12294 + 256*axisNo;
  limitsReg_          = 4600  + axisNo;
  flagsReg_           = 4120  + axisNo;
}

asynStatus ACRAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  ACRController *pC = getController();
  asynStatus status;
  // static const char *functionName = "moveAxis";

  sprintf(pC->outString_, "%s JOG ACC %f", axisName_, acceleration/pulsesPerUnit_);
  status = pC->writeController();
  sprintf(pC->outString_, "%s JOG VEL %f", axisName_, maxVelocity/pulsesPerUnit_);
  status = pC->writeController();
  // Note, the CtlY being send below clears the kill for all axes, in case they had hit a limit, etc.
  if (relative) {
    sprintf(pC->outString_, "%c:%s JOG INC %f", CtlY, axisName_, position/pulsesPerUnit_);
    status = pC->writeController();
  } else {
    sprintf(pC->outString_, "%c:%s JOG ABS %f", CtlY, axisName_, position/pulsesPerUnit_);
    status = pC->writeController();
  }
  return status;
}

asynStatus ACRAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  ACRController *pC = getController();
  asynStatus status;
  // static const char *functionName = "homeAxis";

  sprintf(pC->outString_, "%s JOG ACC %f", axisName_, acceleration/pulsesPerUnit_);
  status = pC->writeController();
  sprintf(pC->outString_, "%s JOG VEL %f", axisName_, maxVelocity/pulsesPerUnit_);
  status = pC->writeController();
  sprintf(pC->outString_, "%c:%s JOG HOME %d", CtlY, axisName_, forwards ? 1 : -1);
  status = pC->writeController();
  return status;
}

asynStatus ACRAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  ACRController *pC = getController();
  asynStatus status;
  double speed=maxVelocity;
  int forwards=1;
  // static const char *functionName = "moveVelocityAxis";

  if (speed < 0) {
    speed = -speed;
    forwards = 0;
  }
  sprintf(pC->outString_, "%s JOG ACC %f", axisName_, acceleration/pulsesPerUnit_);
  status = pC->writeController();
  sprintf(pC->outString_, "%s JOG VEL %f", axisName_, speed/pulsesPerUnit_);
  status = pC->writeController();
  sprintf(pC->outString_, "%c:%s JOG %s", CtlY, axisName_, forwards ? "FWD" : "REV");
  status = pC->writeController();
  return status;
}

asynStatus ACRAxis::stop(double acceleration )
{
  ACRController *pC = getController();
  asynStatus status;
  static const char *functionName = "stopAxis";

  sprintf(pC->outString_, "%s JOG OFF", axisName_);
  status = pC->writeController();

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
    "%s:%s: Set axis %d to stop, status=%d\n",
    driverName, functionName, axisNo_, status);
  return status;
}

asynStatus ACRAxis::setPosition(double position)
{
  ACRController *pC = getController();
  asynStatus status;

  sprintf(pC->outString_, "%s RES %f", axisName_, position/pulsesPerUnit_);
  status = pC->writeController();
  sprintf(pC->outString_, "%s JOG REN", axisName_);
  status = pC->writeController();
  return status;
}

ACRController* ACRAxis::getController()
{
  return static_cast<ACRController*>(asynMotorAxis::getController());
}

asynStatus ACRAxis::poll(int *moving)
{ 
  ACRController *pC = getController();
  int done;
  int driveOn;
  int limit;
  asynStatus comStatus;

  // Read the current encoder position
  sprintf(pC->outString_, "?P%d", encoderPositionReg_);
  comStatus = pC->writeReadController();
  if (comStatus) goto skip;
  encoderPosition_ = atof(pC->inString_);
  setDoubleParam(pC->motorEncoderPosition_,encoderPosition_);

  // Read the current theoretical position
  sprintf(pC->outString_, "?P%d", theoryPositionReg_);
  comStatus = pC->writeReadController();
  if (comStatus) goto skip;
  theoryPosition_ = atof(pC->inString_);
  setDoubleParam(pC->motorPosition_, theoryPosition_);

  // Read the current flags
  sprintf(pC->outString_, "?P%d", flagsReg_);
  comStatus = pC->writeReadController();
  if (comStatus) goto skip;
  currentFlags_ = atoi(pC->inString_);
  done = (currentFlags_ & 0x1000000)?0:1;
  setIntegerParam(pC->motorStatusDone_, done);
  *moving = done ? 0:1;

  // Read the current limit status
  sprintf(pC->outString_, "?P%d", limitsReg_);
  comStatus = pC->writeReadController();
  if (comStatus) goto skip;
  currentLimits_ = atoi(pC->inString_);
  limit = (currentLimits_ & 0x1)?1:0;
  setIntegerParam(pC->motorStatusHighLimit_, limit);
  limit = (currentLimits_ & 0x2)?1:0;
  setIntegerParam(pC->motorStatusLowLimit_, limit);
  limit = (currentLimits_ & 0x4)?1:0;
  setIntegerParam(pC->motorStatusAtHome_, limit);

  // Read the drive power on status
  sprintf(pC->outString_, "DRIVE %s", axisName_);
  comStatus = pC->writeReadController();
  if (comStatus) goto skip;
  driveOn = strstr(pC->inString_, "ON") ? 1:0;
  setIntegerParam(pC->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC->motorStatusProblem_, comStatus ? 1:0);
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
