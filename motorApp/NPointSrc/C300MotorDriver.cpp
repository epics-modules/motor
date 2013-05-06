/*
FILENAME... C300MotorDriver.cpp
USAGE...    Motor driver support for the nPoint C300 series of controllers

Kevin Peterson
March 27, 2012

KNOWN PROBLEMS:

1. The C300 behaves more like a temperature controller than a motor controller.
   The position is the set point.  There is no command to change the speed or 
   acceleration.  There is no done-moving indicator.  Attempts to change the speed
   or acceleration from EPICS are ignored.  The done-moving indicator is simulated.

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "C300MotorDriver.h"
#include <epicsExport.h>

#define C300_TIMEOUT 1.0
#define C300_DATA_OFFSET 112
#define C300_CORR_OFFSET 28

static const char *driverName = "C300MotorDriver";

int C300Tolerance = 256;
extern "C" {epicsExportAddress(int, C300Tolerance);}

/** Creates a new C300Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] C300PortName       The name of the drvAsynIPPPort that was created previously to connect to the C300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
C300Controller::C300Controller(const char *portName, const char *C300PortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_C300_PARAMS, 
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  C300Axis *pAxis;
  static const char *functionName = "C300Controller";

  /* Connect to C300 controller */
  status = pasynOctetSyncIO->connect(C300PortName, 0, &pasynUserC300_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to C300 controller\n",
      driverName, functionName);
  }
  // Unlock the controller
  sprintf(outString_, "SYS:PASS:CEN \"nPoint\"");
  writeController();

  // get controller id, firmware version and range?

  // Wait a short while so that any responses to the above commands have time to arrive so we can flush
  // them in the next writeReadController()
  //epicsThreadSleep(0.5);

  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new C300Axis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new C300Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] C300PortName       The name of the drvAsynIPPPort that was created previously to connect to the C300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int C300CreateController(const char *portName, const char *C300PortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  C300Controller *pC300Controller
    = new C300Controller(portName, C300PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pC300Controller = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void C300Controller::report(FILE *fp, int level)
{
  int axis;
  C300Axis *pAxis;

  fprintf(fp, "C300 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      fprintf(fp, "  axis %d\n"
              "  bitsPerUnit_ = %f\n"
              "    encoder position=%f\n"
              "    theory position=%f\n"
              "    limits=0x%x\n",
              pAxis->axisNo_, pAxis->bitsPerUnit_, 
              pAxis->encoderPosition_, pAxis->theoryPosition_,
              pAxis->currentLimits_);
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}


/** Returns a pointer to an C300MotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
C300Axis* C300Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<C300Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an C300MotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
C300Axis* C300Controller::getAxis(int axisNo)
{
  return static_cast<C300Axis*>(asynMotorController::getAxis(axisNo));
}


/** Writes a string to the C300 controller.
  * Calls writeController() with a default location of the string to write and a default timeout. */ 
asynStatus C300Controller::writeController()
{
  return writeController(outString_, C300_TIMEOUT);
}

/** Writes a string to the C300 controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus C300Controller::writeController(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  const char *functionName="writeController";
  
  status = pasynOctetSyncIO->write(pasynUserC300_, output,
                                   strlen(output), timeout, &nwrite);

  asynPrint(pasynUserC300_, ASYN_TRACEIO_DRIVER,
      "%s:%s: %s\n", driverName, functionName, output);
                                  
  return status ;
}

/** Writes a string to the C300 controller and reads the response.
  * Calls writeReadController() with default locations of the input and output strings
  * and default timeout. */ 
asynStatus C300Controller::writeReadController()
{
  size_t nread;
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, C300_TIMEOUT);
}

/** Writes a string to the C300 controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus C300Controller::writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  const char *functionName="writeReadController";
  
  status = pasynOctetSyncIO->writeRead(pasynUserC300_, output,
                                       strlen(output), input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);
                        
  asynPrint(pasynUserC300_, ASYN_TRACEIO_DRIVER,
      "%s:%s: output: %s  input: %s\n", driverName, functionName, output, input);
                                  
  return status;
}


// These are the C300Axis methods

/** Creates a new C300Axis object.
  * \param[in] pC Pointer to the C300Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
C300Axis::C300Axis(C300Controller *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;
  
  sprintf(axisName_, "CHAN%d", axisNo+1);

  // Get the range for this axis
  sprintf(pC->outString_, "SYST:%s:STAG:RANG?", axisName_);
  status = pC->writeReadController();
  if (status) {
    setIntegerParam(pC->motorStatusProblem_, 1);
  } else {
    // 2^20 bits per total range
    bitsPerUnit_ = (1048576) / atof(pC->inString_);
    // We assume servo motor with encoder for now
    setIntegerParam(pC->motorStatusGainSupport_, 1);
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
  }
  
  // Get the DI Factor for this axis
  sprintf(pC->outString_, "SYST:%s:DI:FACT?", axisName_);
  status = pC->writeReadController();
  if (status) {
    setIntegerParam(pC->motorStatusProblem_, 1);
  } else {
    //
    diScaleFactor_ = atof(pC->inString_);
    diScaleFactorInv_ = 1.0 / diScaleFactor_;
  }
  
  callParamCallbacks();
}

asynStatus C300Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveAxis";

  if (relative) {
    /*sprintf(pC_->outString_, "%s:POS %f", axisName_, (theoryPosition_ + position)/bitsPerUnit_);*/
    sprintf(pC_->outString_, "%s:POS %f", axisName_, (theoryPosition_ + position));
    status = pC_->writeController();
  } else {
    /*sprintf(pC_->outString_, "%s:POS %f", axisName_, position/bitsPerUnit_);*/
    sprintf(pC_->outString_, "%s:POS %f", axisName_, position);
    status = pC_->writeController();
  }
  
  
  return status;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus C300Axis::poll(bool *moving)
{ 
  int done;
  int num;
  asynStatus comStatus;

  // Read the current encoder position
  sprintf(pC_->outString_, "%s:DATA?", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // encoder position is the 9th floating point value in a comma separated list
  
  //printf("\naxisName_ = %s\n", axisName_);
  //printf("data string = %s\n", pC_->inString_);

  num = sscanf(&pC_->inString_[C300_DATA_OFFSET], "%lG", &posMonCorrectedVal_); 
  //printf("data string = %s\n", pC_->inString_);
  num = sscanf(&pC_->inString_[C300_CORR_OFFSET], "%lG", &analogInScaledVal_);
  //printf("data string = %s\n", pC_->inString_);
  
  //printf("posMonCorrectedVal_ = %lG\n", posMonCorrectedVal_); 
  //printf("analogInScaledVal_ = %lG\n", analogInScaledVal_);
  //printf("diScaleFactor_ = %f\n", diScaleFactor_); 
  //printf("diScaleFactorInv_ = %f\n", diScaleFactorInv_);
  //printf("bitsPerUnit_ = %f\n", bitsPerUnit_);
  /*encoderPosition_ = (posMonCorrectedVal_ - analogInScaledVal_) * -1.0 * diScaleFactorInv_;*/
  encoderPosition_ = (posMonCorrectedVal_ - analogInScaledVal_) * diScaleFactorInv_;
  //printf("encoderPosition_ = %f\n", encoderPosition_);

  setDoubleParam(pC_->motorEncoderPosition_,encoderPosition_);

  // Read the current theoretical position
  sprintf(pC_->outString_, "%s:POS?", axisName_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  theoryPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, theoryPosition_);
  //printf("theoryPosition_ = %f\n", theoryPosition_);

  // Read error status

  // Simulate dmov functionality here
  done = (abs(theoryPosition_ - encoderPosition_) > C300Tolerance) ? 0 : 1;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // The limits will be fixed.

  // Motor doesn't have limits or home switch (maybe move this to a constructor)
  setIntegerParam(pC_->motorStatusHighLimit_, 0);
  setIntegerParam(pC_->motorStatusLowLimit_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg C300CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg C300CreateControllerArg1 = {"C300 port name", iocshArgString};
static const iocshArg C300CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg C300CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg C300CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const C300CreateControllerArgs[] = {&C300CreateControllerArg0,
                                                           &C300CreateControllerArg1,
                                                           &C300CreateControllerArg2,
                                                           &C300CreateControllerArg3,
                                                           &C300CreateControllerArg4};
static const iocshFuncDef C300CreateControllerDef = {"C300CreateController", 5, C300CreateControllerArgs};
static void C300CreateContollerCallFunc(const iocshArgBuf *args)
{
  C300CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void C300MotorRegister(void)
{
  iocshRegister(&C300CreateControllerDef, C300CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(C300MotorRegister);
}
