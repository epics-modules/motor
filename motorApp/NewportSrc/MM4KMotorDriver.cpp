/*
FILENAME... MM4KMotorDriver.cpp
USAGE...    Motor driver support for the Newport MM4K series of controllers, including the Aries.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsString.h>

#include <asynOctetSyncIO.h>

#include "MM4KMotorDriver.h"
#include <epicsExport.h>

#define COMM_TIMEOUT 1.0

static const char *driverName = "MM4KMotorDriver";

MM4KController::MM4KController(const char *asynMotorPort, const char *asynCommPort, int numAxes, double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(asynMotorPort, numAxes,  
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask, 
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  static const char *functionName = "MM4KController";
  int modelNum, totalAxes;
  char *pmodel, *ptr, *tokSave;

  binaryInReg_  = 4096;
  binaryOutReg_ = 4097;
  

  /* Connect to MM4K controller */
  status = pasynOctetSyncIO->connect(asynCommPort, 0, &pasynUser_, NULL);

  if (status)
  {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: cannot connect to MM4K controller\n", driverName, functionName);
    goto errexit;
  }
  else
  {
    int retry = 0;

    pasynOctetSyncIO->flush(this->pasynUser_);
    strcpy(outString_, "VE;");

    do
    {
      status = writeReadController();
      retry++;
    } while (status != asynSuccess && retry < 3);
    if (status != asynSuccess)
      goto errexit;
  }

  strcpy(firmwareVersion_, &inString_[2]);  /* Skip "VE" */

  /* Set Motion Master model indicator. */
  pmodel = strstr(firmwareVersion_, "MM");
  if (pmodel == NULL)
  {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: invalid model = %s\n", driverName, functionName, firmwareVersion_);
    goto errexit;
  }
  modelNum = atoi(pmodel+2);
  if (modelNum == 4000)
    this->model = MM4000;
  else if (modelNum == 4005 || modelNum == 4006)
    this->model = MM4005;
  else
  {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: invalid model = %s\n", driverName, functionName, firmwareVersion_);
    goto errexit;
  }

  // Read the binary I/O registers once
  //  readBinaryIO();

  // Set the output=output readback so bi records reflect current state
  //setUIntDigitalParam(0, MM4KBinaryOut_, binaryOutRBV_, 0xFFFFFFFF);

  writeReadController("TP;");

  /* The return string will tell us how many axes this controller has */
  for (totalAxes = 0, tokSave = NULL, ptr = epicsStrtok_r(inString_, ",", &tokSave); ptr != 0; ptr = epicsStrtok_r(NULL, ",", &tokSave), totalAxes++);

  if (totalAxes > numAxes)
  {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: available axes (%d) > maximum allowed (%d)\n", driverName, functionName, totalAxes, numAxes);
    totalAxes = numAxes;
  }
  else
    numAxes = totalAxes;

  startPoller(movingPollPeriod/1000., idlePollPeriod/1000., 0);

errexit:;
}


/** Configuration command, called directly or from iocsh */
extern "C" int MM4KCreateController(const char *asynMotorPort, const char *asynCommPort, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  MM4KController *pMM4KController = new MM4KController(asynMotorPort, asynCommPort, numAxes, movingPollPeriod, idlePollPeriod);
  pMM4KController->initializePortDriver();
  pMM4KController = NULL;
  return(asynSuccess);
}

asynStatus MM4KController::createDriverParams()
{
  asynStatus status = asynSuccess;
  status = asynMotorController::createDriverParams();
  createParam(MM4KJerkString,         asynParamFloat64,       &MM4KJerk_);
  createParam(MM4KReadBinaryIOString, asynParamInt32,         &MM4KReadBinaryIO_);
  createParam(MM4KBinaryInString,     asynParamUInt32Digital, &MM4KBinaryIn_);
  createParam(MM4KBinaryOutString,    asynParamUInt32Digital, &MM4KBinaryOut_);
  createParam(MM4KBinaryOutRBVString, asynParamUInt32Digital, &MM4KBinaryOutRBV_);

  return asynSuccess;
}

void MM4KController::report(FILE *fp, int level)
{
  int axis;
  MM4KAxis *pAxis;

  fprintf(fp, "MM4K motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f, ID=%s\n", 
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_, firmwareVersion_);

  if (level > 0)
  {
    for (axis=0; axis < numAxes_; axis++)
    {
        pAxis = getAxis(axis);
        pAxis->axisReport(fp);
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

MM4KAxis* MM4KController::getAxis(asynUser *pasynUser)
{
  return static_cast<MM4KAxis*>(asynMotorController::getAxis(pasynUser));
}

MM4KAxis* MM4KController::getAxis(int axisNo)
{
  return static_cast<MM4KAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus MM4KController::setOutString(const char *ptr)
{
  asynStatus status = asynSuccess;
  if (strlen(ptr) < sizeof(outString_))
      strcpy(outString_, ptr);
  else
      status = asynOverflow;
  return(status);
}

asynStatus MM4KController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  MM4KAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  int axisindex = pAxis->getAxisIndex();
  
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(axisindex, function, value);
  
  if (function == motorDeferMoves_)
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: %sing Deferred Move flag on driver %s\n", value != 0.0?"Sett":"Clear", driverName, functionName, this->portName);
  else if (function == motorSetClosedLoop_)
  {
    /* The MM4000 only allows turning on and off ALL motors (MO and MF commands), */
    /* not individual axes. Don't implement */        
    if (model != MM4000)
    {
        if (value == 0)
            writeController("%sMF", axisindex);
        else
            writeController("%sMO", axisindex);
    }
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: %s setting closed loop=%d on axis #%d\n", driverName, functionName, this->portName, value, axisindex);
  } 
  else if (function == MM4KReadBinaryIO_)
    readBinaryIO();
  else 
    status = asynMotorController::writeInt32(pasynUser, value); /* Call base class method */
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: error, status=%d function=%d, value=%d\n", driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: function=%d, value=%d\n", driverName, functionName, function, value);
  return status;
}

asynStatus MM4KController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  MM4KAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeFloat64";
  int axisindex = pAxis->getAxisIndex();
  
  
  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam(axisindex, function, value);
  
  if (function == MM4KJerk_)
  {
//    sprintf(outString_, "%s JOG JRK %f", pAxis->axisName_, value);
//    status = writeController();
    
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

asynStatus MM4KController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
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

asynStatus MM4KController::readBinaryIO()
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
    setUIntDigitalParam(0, MM4KBinaryIn_, binaryIn_, 0xFFFFFFFF);
    setUInt32DigitalInterrupt(0, MM4KBinaryIn_, changedBits, interruptOnBoth);
  }

  // Read the binary outputs
  sprintf(outString_, "?P%d", binaryOutReg_);
  status = writeReadController();
  if (!status) {
    newBits = atoi(inString_);
    changedBits = newBits ^ binaryOutRBV_;
    binaryOutRBV_ = newBits;
    setUIntDigitalParam(0, MM4KBinaryOutRBV_, binaryOutRBV_, 0xFFFFFFFF);
    setUInt32DigitalInterrupt(0, MM4KBinaryOutRBV_, changedBits, interruptOnBoth);
  }
  callParamCallbacks(0);
  return status;
}

asynStatus MM4KController::writeController()
{
  return writeController(outString_, COMM_TIMEOUT);
}

asynStatus MM4KController::writeController(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeController";
  
  status = pasynOctetSyncIO->write(pasynUser_, output, strlen(output), timeout, &nwrite);
                                  
  return status ;
}

asynStatus MM4KController::writeController(const char *format, int axisnum)
{
  sprintf(outString_, format, axisnum + 1);
  return writeController(outString_, COMM_TIMEOUT);
}

asynStatus MM4KController::writeReadController()
{
  size_t nread;
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, COMM_TIMEOUT);
}

asynStatus MM4KController::writeReadController(const char *output)
{
  size_t nread;
  strcpy(outString_, output);
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, COMM_TIMEOUT);
}


asynStatus MM4KController::writeReadController(const char *format, int axisnum)
{
  size_t nread;
  sprintf(outString_, format, axisnum + 1);
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, COMM_TIMEOUT);
}

asynStatus MM4KController::writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadController";
  
  status = pasynOctetSyncIO->writeRead(pasynUser_, output, strlen(output), input, maxChars, timeout, &nwrite, nread, &eomReason);
                        
  return status;
}

asynStatus MM4KController::writeReadConvertDouble(const char *format, int axisnum, double *output)
{
  asynStatus status;
  
  status = writeReadController(format, axisnum);
  if (status == asynSuccess)
      *output = atof(&inString_[3]);                       
  return status;
}

asynStatus MM4KController::writeReadConvertInt(const char *format, int axisnum, int *output)
{
  asynStatus status;
  
  status = writeReadController(format, axisnum);
  if (status == asynSuccess)
      *output = atoi(&inString_[3]);                       
  return status;
}

char MM4KController::writeReadRtnResponse(const char *format, int axisnum, asynStatus *status)
{
  *status = writeReadController(format, axisnum);
  return(inString_[3]);
}

char MM4KController::writeReadRtnResponse(const char *format, asynStatus *status)
{
  *status = writeReadController(format);
  return(inString_[2]);
}

int MM4KController::getNumParams()
{
  return NUM_MM4K_PARAMS + asynMotorController::getNumParams() + asynMotorStatus::getNumParams();
}

asynStatus MM4KController::postInitDriver()
{
  MM4KAxis *pAxis;
  int axis;
  for (axis=0; axis < numAxes_; axis++)
  {
    pAxis  = new MM4KAxis(this, axis);
    pAxis->getStatus()->createParams();
    callParamCallbacks();
  }
  return asynSuccess;


}

MM4KAxis::MM4KAxis(MM4KController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;

  sprintf(axisName_, "%d", axisNo+1);
  
  // Get the number of pulses per unit on this axis
  status = pC->writeReadConvertDouble("%dTU", axisNo, &pulsesPerUnit_);
  if (status)
    getStatus()->setProblem(true);
  else
  {
    int loopState;

    int digits = (int) -log10(pulsesPerUnit_) + 2;
    if (digits < 1)
        digits = 1;
    maxDigits = digits;

    getStatus()->setHasGainSupport(true); /* Assume gain support. */

    pC->writeReadConvertInt("%dTC", axisNo, &loopState);
    getStatus()->setHasEncoder(loopState);

    /* Save home preset position. */
    pC->writeReadConvertDouble("%dXH", axisNo, &homePreset);

    /* Determine low limit */
    pC->writeReadConvertDouble("%dTL", axisNo, &lowLimit);

    /* Determine high limit */
    pC->writeReadConvertDouble("%dTR", axisNo, &highLimit);
  }
}

void MM4KAxis::axisReport(FILE *fp)
{
  fprintf(fp, "  axis %d\n"
          "  pulsesPerUnit_ = %f\n"
          "    encoder position=%f\n"
          "    theory position=%f\n"
          "    limits=0x%x\n"
          "    flags=0x%x\n", 
          axisNo_,          pulsesPerUnit_, 
          encoderPosition_, theoryPosition_,
          currentLimits_,   currentFlags_);
}

asynStatus MM4KAxis::move(double position, bool relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  char *moveCommand;
  char buffer[MAX_BUF_SIZE];

  if (relative == true)
      moveCommand = (char *) "PR";
  else
      moveCommand = (char *) "PA";

  sprintf(buffer, "%sAC%.*f;%sVA%.*f;%s%s%.*f;",
          axisName_,              maxDigits, acceleration * pulsesPerUnit_,
          axisName_,              maxDigits, maxVelocity  * pulsesPerUnit_,
          axisName_, moveCommand, maxDigits, position     * pulsesPerUnit_);
  status = pC_->setOutString((const char*) &buffer);
  if (status == asynSuccess)
    status = pC_->writeController();
  return status;
}

asynStatus MM4KAxis::home(double minVelocity, double maxVelocity, double acceleration, bool forwards)
{
  asynStatus status;
  char buffer[MAX_BUF_SIZE];
  // static const char *functionName = "homeAxis";

  sprintf(buffer, "%sAC%.*f;%sVA%.*f;%sOR;",
          axisName_,              maxDigits, acceleration * pulsesPerUnit_,
          axisName_,              maxDigits, maxVelocity  * pulsesPerUnit_,
          axisName_);
  status = pC_->setOutString((const char*) &buffer);
  if (status == asynSuccess)
    status = pC_->writeController();
  return status;
}

asynStatus MM4KAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  double speed=maxVelocity, abspos;
  int forwards=1;
  // static const char *functionName = "moveVelocityAxis";

  if (speed < 0)
  {
    speed = -speed;
    forwards = 0;
    abspos = lowLimit / pulsesPerUnit_;
  }
  else
    abspos = highLimit / pulsesPerUnit_;
  
  status = move(abspos, false, minVelocity, speed, acceleration);

  return status;
}

asynStatus MM4KAxis::stop(double acceleration )
{
  asynStatus status;
  static const char *functionName = "stopAxis";

  status = pC_->writeController("%dST;", axisNo_);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
    "%s:%s: Set axis %d to stop, status=%d\n",
    driverName, functionName, axisNo_, status);
  return status;
}

asynStatus MM4KAxis::setPosition(double position)
{
  char buffer[MAX_BUF_SIZE];
  asynStatus status;

  sprintf(buffer, "%d RES %f", axisNo_, position/pulsesPerUnit_);
  status = pC_->setOutString((const char*) &buffer);
  if (status == asynSuccess)
    status = pC_->writeController();
  sprintf(buffer, "%d JOG REN", axisNo_);
  status = pC_->setOutString((const char*) &buffer);
  if (status == asynSuccess)
    status = pC_->writeController();
  return status;
}

asynStatus MM4KAxis::poll(bool *moving)
{ 
  bool done;
  asynStatus comStatus;
  bool errInd;
  char rtnchar;

  // Read the current encoder position
  comStatus = pC_->writeReadConvertDouble("%dTP;", axisNo_, &encoderPosition_);
  if (comStatus)
    goto skip;
  setDoubleParam(pC_->getMotorEncoderPositionIndex(), encoderPosition_ / pulsesPerUnit_);

  // Read the current theoretical position
  comStatus = pC_->writeReadConvertDouble("%dTH;", axisNo_, &theoryPosition_);
  if (comStatus)
    goto skip;
  setDoubleParam(pC_->getMotorPositionIndex(), theoryPosition_ / pulsesPerUnit_);

  // Read the current status
  rtnchar = pC_->writeReadRtnResponse("%dMS", axisNo_, &comStatus);
  currentFlags_ = (int) rtnchar;
  getStatus()->setAtHome(currentFlags_ & MM4000_HOME);
  getStatus()->setHighLimitOn(currentFlags_ & MM4000_HIGH_LIMIT);
  getStatus()->setLowLimitOn(currentFlags_ & MM4000_LOW_LIMIT);
  getStatus()->setDirection(currentFlags_ & MM4000_DIRECTION ? PLUS : MINUS);
  getStatus()->setPowerOn(!(currentFlags_ & MM4000_POWER_OFF));
  done = (currentFlags_ & MM4000_MOVING) ? false : true;
  getStatus()->setDoneMoving(done);
  *moving = done ? false : true;

  /* Check for controller error. */
  rtnchar = pC_->writeReadRtnResponse("TE;", &comStatus);
  if (rtnchar == '@')
      errInd = false;
  else
  {
      errInd = true;
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, "controller error on card/addr = %d:%d\n", axisNo_);
  }
  getStatus()->setProblem(errInd);

skip:
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg CreateControllerArg0 = {"Asyn Motor port name", iocshArgString};
static const iocshArg CreateControllerArg1 = {"Asyn Comm. port name", iocshArgString};
static const iocshArg CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const CreateControllerArgs[] =  {&CreateControllerArg0,
                                                         &CreateControllerArg1,
                                                         &CreateControllerArg2,
                                                         &CreateControllerArg3,
                                                         &CreateControllerArg4};
static const iocshFuncDef CreateControllerDef = {"MM4KCreateController", 5, CreateControllerArgs};
static void CreateContollerCallFunc(const iocshArgBuf *args)
{
    MM4KCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void MM4KMotorRegister(void)
{
  iocshRegister(&CreateControllerDef, CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(MM4KMotorRegister);
}
