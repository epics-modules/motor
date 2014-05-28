/*
FILENAME... MVP2001Driver.cpp
USAGE...    Motor driver support for the MicroMo MVP 2001 controller.

Kevin Peterson
August 14, 2013

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "MVP2001Driver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new MVP2001Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MVP2001PortName     The name of the drvAsynSerialPort that was created previously to connect to the MVP2001 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
MVP2001Controller::MVP2001Controller(const char *portName, const char *MVP2001PortName, int numAxes, 
                                 double movingPollPeriod,double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_MVP2001_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  static const char *functionName = "MVP2001Controller::MVP2001Controller";

  /* Connect to MVP2001 controller */
  status = pasynOctetSyncIO->connect(MVP2001PortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to MVP 2001 controller\n",
      functionName);
  }

  // Don't create axes here; make the user configure them at boot time since additional information about the axes needs to be specified

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new MVP2001Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MVP2001PortName       The name of the drvAsynIPPPort that was created previously to connect to the MVP2001 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int MVP2001CreateController(const char *portName, const char *MVP2001PortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  MVP2001Controller *pMVP2001Controller
    = new MVP2001Controller(portName, MVP2001PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pMVP2001Controller = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void MVP2001Controller::report(FILE *fp, int level)
{
  fprintf(fp, "MVP 2001 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an MVP2001Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
MVP2001Axis* MVP2001Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<MVP2001Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an MVP2001Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
MVP2001Axis* MVP2001Controller::getAxis(int axisNo)
{
  return static_cast<MVP2001Axis*>(asynMotorController::getAxis(axisNo));
}

/** Writes a string to the controller and reads the response.
  * Calls writeRead2xController() with default locations of the input and output strings
  * and default timeout. */ 
asynStatus MVP2001Controller::writeRead2xController()
{
  size_t nread;
  return writeRead2xController(outString_, inString_, sizeof(inString_), &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

/* A custom read method is needed to handle the prepended terminator for commands
   MVP2001 commands that generate a response. */
asynStatus MVP2001Controller::writeRead2xController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeRead2xController";
  
  // Write the command; read the prepended terminator
  status = pasynOctetSyncIO->writeRead(pasynUserController_, output, strlen(output), input, maxChars, timeout, &nwrite, nread, &eomReason);
  if (status) goto skip;
  
  // A small delay is needed to avoid timeouts in the following read
  epicsThreadSleep(0.033);
  
  // Read the response
  status = pasynOctetSyncIO->read(pasynUserController_, input, maxChars, timeout, nread, &eomReason);
  
  skip:
  return status;
}

void MVP2001Controller::parseReply(char *inString, int *val, int nchars)
{
  buff_[0] = '\0';

  // Controller responses are of the form "0001 FFFF"
  strncat(buff_, &inString[5], nchars);
  sscanf(buff_, "%x", val);
  //*val = strtoul(buff_, NULL, 16);

}

// These are the MVP2001Axis methods

/** Creates a new MVP2001Axis object.
  * \param[in] pC Pointer to the MVP2001Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
MVP2001Axis::MVP2001Axis(MVP2001Controller *pC, int axisNo, int encLPR, int maxCurr, int limPol)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;
  
  axisIndex_ = axisNo + 1;
  encoderLinesPerRev_ = encLPR;

  // The MVP2001 manual implies that the range is 0.1-2.3A is ok
  if ( (maxCurr < 100) || maxCurr > 2300 )
    maxCurrent_ = 100;
  else 
    maxCurrent_ = maxCurr;

  // Default to NO if a bad value is given (1=NO, 0=NC)
  if ( limPol != 0 )
    limitPolarity_ = 1;
  else
    limitPolarity_ = 0;
  
  // Set the limit & estop polarity
  sprintf(pC_->outString_, "%d LP %d", axisIndex_, limitPolarity_);
  status = pC_->writeController();

  // Home the motor (this zeroes the position, which will be restored by autosave at iocInit)
  sprintf(pC_->outString_, "%d HO", axisIndex_);
  status = pC_->writeController();

  /* The old driver enabled the motor here. Doing so causes problems.
     Specifically, if enabling the motor causes the readback position to be 
     non-zero, then the autosaved position is NOT restored at iocInit */
  //sprintf(pC_->outString_, "%d EN", axisIndex_);
  //status = pC_->writeController();

  // Query the MVP Loop sample period
  sprintf(pC_->outString_, "%d SR", axisIndex_);
  status = pC_->writeRead2xController();
  if (status == asynSuccess)
    pC_->parseReply(pC_->inString_, &samplePeriod_, 4);
  else
    samplePeriod_ = 500;
  
  // Allow CNEN to turn motor power on/off
  setIntegerParam(pC->motorStatusGainSupport_, 1);
  setIntegerParam(pC->motorStatusHasEncoder_, 1);

}


extern "C" int MVP2001CreateAxis(const char *MVP2001Name, int axisNo, int encLPR, int maxCurr, int limPol)
{
  MVP2001Controller *pC;
  
  pC = (MVP2001Controller*) findAsynPortDriver(MVP2001Name);
  if (!pC) 
  {
    printf("Error port %s not found\n", MVP2001Name);
    return asynError;
  }

  pC->lock();
  new MVP2001Axis(pC, axisNo, encLPR, maxCurr, limPol);
  pC->unlock();
  return asynSuccess;
}


/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void MVP2001Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
    fprintf(fp, "  axis index %d\n", axisIndex_);
    fprintf(fp, "  encoderLinesPerRev %d\n", encoderLinesPerRev_);
    fprintf(fp, "  maxCurrent %d (mA)\n", maxCurrent_);
    fprintf(fp, "  samplePeriod %d (us)\n", samplePeriod_);
    fprintf(fp, "  limitPolarity %d\n", limitPolarity_);
 }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus MVP2001Axis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  int ano;
  int sp;
  int ac;
  // static const char *functionName = "MVP2001::sendAccelAndVelocity";

  // Send the maximum current (equation determined from data in a graph in the MVP manual)
  ano = NINT(maxCurrent_ * 0.865909 + 2103.431);
  sprintf(pC_->outString_, "%d ANO %d", axisIndex_, ano);
  status = pC_->writeController();

  // Send the velocity
  // TODO: explain the velocity calc
  sp = NINT(samplePeriod_ * 6e-5 * velocity);
  sprintf(pC_->outString_, "%d SP %d", axisIndex_, sp);
  status = pC_->writeController();

  // Send the acceleration
  // TODO: explain thie acceleration calc
  ac = NINT(7.5e-12 * samplePeriod_ * samplePeriod_ * encoderLinesPerRev_ * acceleration);
  if (ac < sp)
  {
    // Don't allow negative accelerations
    if (ac <= 0)
      ac = 1;
  }
  else
  {
    // Don't let the acceleration exceed the speed
    ac = sp;
  }  
  sprintf(pC_->outString_, "%d AC %d", axisIndex_, ac);
  status = pC_->writeController();
  // Maybe put a goto earlier to go to here if not asynSuccess, but then need to do the same in methods that call this one
  return status;
}


asynStatus MVP2001Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "MVP2001Axis::move";

  status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  // Set the target position
  if (relative) {
    sprintf(pC_->outString_, "%d LR %d", axisIndex_, NINT(position));
  } else {
    sprintf(pC_->outString_, "%d LA %d", axisIndex_, NINT(position));
  }
  status = pC_->writeController();
  
  // Send the 'move' command
  sprintf(pC_->outString_, "%d M", axisIndex_);
  status = pC_->writeController();
  
  return status;
}

asynStatus MVP2001Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  // static const char *functionName = "MVP2001Axis::home";

  // Homing wasn't implemented in the original driver.  
  // It could be implemented using the HA & HP commands.

  return asynSuccess;
}

asynStatus MVP2001Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int sp;
  static const char *functionName = "MVP2001Axis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);
    
  // Call this to set the max current and acceleration
  status = sendAccelAndVelocity(acceleration, maxVelocity);

  // Calculate velocity
  sp = NINT(samplePeriod_ * 6e-5 * maxVelocity);

  sprintf(pC_->outString_, "%d V %d", axisIndex_, sp);
  status = pC_->writeController();
  return status;
}

asynStatus MVP2001Axis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "MVP2001Axis::stop";

  sprintf(pC_->outString_, "%d AB", axisIndex_);
  status = pC_->writeController();
  return status;
}

asynStatus MVP2001Axis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "MVP2001Axis::setPosition";

  sprintf(pC_->outString_, "%d HO %d", axisIndex_, NINT(position));
  status = pC_->writeController();
  return status;
}


/** Set the proportional gain of the motor.
  * \param[in] pGain The new proportional gain. */
asynStatus MVP2001Axis::setPGain(double pGain)
{
  int ival;
  asynStatus status;

  ival = NINT(pGain * 28000 + 4000);
  sprintf(pC_->outString_, "%d POR %d", axisIndex_, ival);
  status = pC_->writeController();

  return status;
}


/** Set the integral gain of the motor.
  * \param[in] iGain The new integral gain. */
asynStatus MVP2001Axis::setIGain(double iGain)
{
  int ival;
  asynStatus status;

  ival = NINT(iGain * 31999 + 1);
  sprintf(pC_->outString_, "%d I %d", axisIndex_, ival);
  status = pC_->writeController();

  return status;
}


/** Set the derivative gain of the motor.
  * \param[in] dGain The new derivative gain. */
asynStatus MVP2001Axis::setDGain(double dGain)
{
  int ival;
  asynStatus status;

  ival = NINT(dGain * 31000 + 1000);
  sprintf(pC_->outString_, "%d DER %d", axisIndex_, ival);
  status = pC_->writeController();

  return status;
}

asynStatus MVP2001Axis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  //static const char *functionName = "MVP2001Axis::setClosedLoop";
  
  if (closedLoop)
  {
    // Send an AB here for EN to work (EN fails if status ends in 8, rather than E)
    sprintf(pC_->outString_, "%d AB", axisIndex_);
    status = pC_->writeController();
    epicsThreadSleep(0.033);
    
    sprintf(pC_->outString_, "%d EN", axisIndex_);
  }
  else
  {
    sprintf(pC_->outString_, "%d DI", axisIndex_);
  }

  status = pC_->writeController();
  return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus MVP2001Axis::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int limit;
  int position;
  int status;
  asynStatus comStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "%d POS", axisIndex_);
  comStatus = pC_->writeRead2xController();
  if (comStatus) 
    goto skip;
  // The response string is of the form "0001 000001F4"
  pC_->parseReply(pC_->inString_, &position, 8);
  setDoubleParam(pC_->motorPosition_, position);

  // Read the moving status of this motor
  sprintf(pC_->outString_, "%d ST", axisIndex_);
  comStatus = pC_->writeRead2xController();
  if (comStatus) 
    goto skip;
  // The response string is of the form "0001 0008"
  pC_->parseReply(pC_->inString_, &status, 4);

  // Set the direction bit in the move method instead of here since there isn't a direction bit, requires private readback position var
  // Or set the direction bit here, requires a private target position var

  done = (status & 0x1) ? 0 : 1;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // Read the limit status
  limit = (status & 0x2000) ? 1 : 0;
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (status & 0x8000) ? 1 : 0;
  setIntegerParam(pC_->motorStatusLowLimit_, limit);
  // MVP2001 doesn't have a home status bit
  
  // Read the drive power on status
  driveOn = (status & 0x100) ? 0 : 1;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg MVP2001CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg MVP2001CreateControllerArg1 = {"MVP 2001 port name", iocshArgString};
static const iocshArg MVP2001CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg MVP2001CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg MVP2001CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const MVP2001CreateControllerArgs[] = {&MVP2001CreateControllerArg0,
                                                             &MVP2001CreateControllerArg1,
                                                             &MVP2001CreateControllerArg2,
                                                             &MVP2001CreateControllerArg3,
                                                             &MVP2001CreateControllerArg4};
static const iocshFuncDef MVP2001CreateControllerDef = {"MVP2001CreateController", 5, MVP2001CreateControllerArgs};
static void MVP2001CreateContollerCallFunc(const iocshArgBuf *args)
{
  MVP2001CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


/* MVP2001CreateAxis */
static const iocshArg MVP2001CreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg MVP2001CreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg MVP2001CreateAxisArg2 = {"Encoder lines per rev", iocshArgInt};
static const iocshArg MVP2001CreateAxisArg3 = {"Max current (ma)", iocshArgInt};
static const iocshArg MVP2001CreateAxisArg4 = {"Limit polarity", iocshArgInt};
static const iocshArg * const MVP2001CreateAxisArgs[] = {&MVP2001CreateAxisArg0,
                                                     &MVP2001CreateAxisArg1,
                                                     &MVP2001CreateAxisArg2,
                                                     &MVP2001CreateAxisArg3,
                                                     &MVP2001CreateAxisArg4};
static const iocshFuncDef MVP2001CreateAxisDef = {"MVP2001CreateAxis", 5, MVP2001CreateAxisArgs};
static void MVP2001CreateAxisCallFunc(const iocshArgBuf *args)
{
  MVP2001CreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}

static void MVP2001Register(void)
{
  iocshRegister(&MVP2001CreateControllerDef, MVP2001CreateContollerCallFunc);
  iocshRegister(&MVP2001CreateAxisDef,       MVP2001CreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(MVP2001Register);
}
