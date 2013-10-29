/*
FILENAME... MMC200Driver.cpp
USAGE...    Motor driver support for the Micronix MMC-200 controller.

Kevin Peterson
July 10, 2013

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "MMC200Driver.h"
#include <epicsExport.h>

/** Creates a new MMC200Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MMC200PortName     The name of the drvAsynSerialPort that was created previously to connect to the MMC200 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
MMC200Controller::MMC200Controller(const char *portName, const char *MMC200PortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod, int ignoreLimits)
  :  asynMotorController(portName, numAxes, NUM_MMC200_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  MMC200Axis *pAxis;
  static const char *functionName = "MMC200Controller::MMC200Controller";

  /* Set flag to ignore limits */
  if (ignoreLimits == 0)
    ignoreLimits_ = 0;
  else
    ignoreLimits_ = 1;

  /* Connect to MMC200 controller */
  status = pasynOctetSyncIO->connect(MMC200PortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to MMC-200 controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new MMC200Axis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new MMC200Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MMC200PortName       The name of the drvAsynIPPPort that was created previously to connect to the MMC200 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int MMC200CreateController(const char *portName, const char *MMC200PortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod, int ignoreLimits)
{
  MMC200Controller *pMMC200Controller
    = new MMC200Controller(portName, MMC200PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., ignoreLimits);
  pMMC200Controller = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void MMC200Controller::report(FILE *fp, int level)
{
  fprintf(fp, "MMC-200 motor driver\n");
  fprintf(fp, "  port name=%s\n", this->portName);
  fprintf(fp, "  num axes=%d\n", numAxes_);
  fprintf(fp, "  moving poll period=%f\n", movingPollPeriod_);
  fprintf(fp, "  idle poll period=%f\n", idlePollPeriod_);
  fprintf(fp, "  ignore limits=%d\n", ignoreLimits_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an MMC200Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
MMC200Axis* MMC200Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<MMC200Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an MMC200Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
MMC200Axis* MMC200Controller::getAxis(int axisNo)
{
  return static_cast<MMC200Axis*>(asynMotorController::getAxis(axisNo));
}


// These are the MMC200Axis methods

/** Creates a new MMC200Axis object.
  * \param[in] pC Pointer to the MMC200Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
MMC200Axis::MMC200Axis(MMC200Controller *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  int errorFlag = 0;
  asynStatus status;
  
  // controller axes are numbered from 1
  axisIndex_ = axisNo + 1;
  
  // Read the axis resolution (units = tens of picometers per full step)
  sprintf(pC_->outString_, "%dREZ?", axisIndex_);
  status = pC_->writeReadController();
  if (status != asynSuccess)
    errorFlag = 1;
  rez_ = atoi(&pC_->inString_[1]);

  // Read the number of microsteps
  sprintf(pC_->outString_, "%dUST?", axisIndex_);
  status = pC_->writeReadController();
  if (status != asynSuccess)
    errorFlag = 1;
  microSteps_ = atoi(&pC_->inString_[1]);
  
  // Calculate motor resolution (mm / microstep)
  resolution_ = rez_ * 1e-8 / microSteps_;

  // Read max velocity (needed for jog speed calculation)
  sprintf(pC_->outString_, "%dVMX?", axisIndex_);
  status = pC_->writeReadController();
  if (status != asynSuccess)
    errorFlag = 1;
  maxVelocity_ = atof(&pC_->inString_[1]);

  // Allow CNEN to turn motor power on/off
  setIntegerParam(pC->motorStatusGainSupport_, 1);
  setIntegerParam(pC->motorStatusHasEncoder_, 1);

  // What should happen if the controller doesn't respond?
  // For now put the controller in an error state
  
  if (errorFlag == 1)
  {
    setIntegerParam(pC_->motorStatusProblem_, 1);
  }
  
  callParamCallbacks();

}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void MMC200Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
    fprintf(fp, "  axis index %d\n", axisIndex_);
    fprintf(fp, "  rez %d\n", rez_);
    fprintf(fp, "  micro steps %d\n", microSteps_);
    fprintf(fp, "  resolution %f\n", resolution_);
    fprintf(fp, "  max velocity %f\n", maxVelocity_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus MMC200Axis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "MMC200::sendAccelAndVelocity";

  // get max velocity or do this in status?

  // Send the velocity
  sprintf(pC_->outString_, "%dVEL%.3f", axisIndex_, velocity * resolution_);
  status = pC_->writeController();

  // Send the acceleration
  sprintf(pC_->outString_, "%dACC%.3f", axisIndex_, acceleration * resolution_);
  status = pC_->writeController();
  
  // Send the deceleration
  sprintf(pC_->outString_, "%dDEC%.3f", axisIndex_, acceleration * resolution_);
  status = pC_->writeController();
  
  return status;
}


asynStatus MMC200Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "MMC200Axis::move";

  status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  if (relative) {
    sprintf(pC_->outString_, "%dMVR%.6f", axisIndex_, position * resolution_);
  } else {
    sprintf(pC_->outString_, "%dMVA%.6f", axisIndex_, position * resolution_);
  }
  status = pC_->writeController();
  return status;
}

asynStatus MMC200Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "MMC200Axis::home";

  status = sendAccelAndVelocity(acceleration, maxVelocity);

  if (forwards) {
    sprintf(pC_->outString_, "%dHCG1", axisIndex_);
  } else {
    sprintf(pC_->outString_, "%dHCG0", axisIndex_);
  }
  status = pC_->writeController();
  
  sprintf(pC_->outString_, "%dHOM", axisIndex_);
  status = pC_->writeController();
  return status;
}

asynStatus MMC200Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  double jogPercent;
  static const char *functionName = "MMC200Axis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);
    
  // the JOG command accepts a speed as a % of max speed
  // How to move in other direction if only specifying a % 
  jogPercent = (maxVelocity * resolution_) / maxVelocity_ * 100.0;
  
  // Cap at 100 percent
  if (jogPercent < -100.0)
    jogPercent = -100.0;
  if (jogPercent > 100.0)
    jogPercent = 100.0;

  // Set jog acceleration
  sprintf(pC_->outString_, "%dJAC%.3f", axisIndex_, acceleration * resolution_);
  status = pC_->writeController();

  // Start jogging
  sprintf(pC_->outString_, "%dJOG%.3f", axisIndex_, jogPercent);
  status = pC_->writeController();

  return status;
}

asynStatus MMC200Axis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "MMC200Axis::stop";

  sprintf(pC_->outString_, "%dSTP", axisIndex_);
  status = pC_->writeController();
  return status;
}

asynStatus MMC200Axis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "MMC200Axis::setPosition";

  // The MMC-200 only allows setting the position to zero
  if (position == 0.0)
  {
    sprintf(pC_->outString_, "%dZRO", axisIndex_);
    status = pC_->writeController();
  }
  else
  {
    // Should this be asynError instead?
    status = asynSuccess;
  }
  return status;
}

asynStatus MMC200Axis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  //static const char *functionName = "MMC200Axis::setClosedLoop";

  // closed-loop command has more than two states. Implement it in a parameter.

  // turn motor power on/off
  sprintf(pC_->outString_, "%dMOT%d", axisIndex_, (closedLoop) ? 1:0);
  status = pC_->writeController();
  return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus MMC200Axis::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int lowLimit;
  int highLimit;
  int status;
  double pos;
  double enc;
  asynStatus comStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "%dPOS?", axisIndex_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "#0.000000,0.000000"
  // #<theoretical position>,<encoder position>
  sscanf(pC_->inString_, "#%lf,%lf", &pos, &enc);
  
  setDoubleParam(pC_->motorPosition_, pos / resolution_);
  setDoubleParam(pC_->motorEncoderPosition_, enc / resolution_
  );

  // Read the status of this axis
  sprintf(pC_->outString_, "%dSTA?", axisIndex_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "#8"
  status = atoi(&pC_->inString_[1]);

  // Interpret the bits
  // 0x1 = Negative Limit Active
  // 0x2 = Positive Limit Active
  // 0x4 = A program is running
  // 0x8 = Done Moving
  // 0x10 = In deceleration phase
  // 0x20 = In constant velocity phase
  // 0x40 = In acceleration phase
  // 0x80 = One or more errors have occurred

  done = status & 0x8;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  /* Only check limit bits if ignore flag is zero */
  if (pC_->ignoreLimits_ == 0)
  {
    highLimit = status & 0x2;
    lowLimit = status & 0x1;
  }
  else
  {
    highLimit = 0;
    lowLimit = 0;
  }
  setIntegerParam(pC_->motorStatusHighLimit_, highLimit);
  setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);
  
  //setIntegerParam(pC_->motorStatusAtHome_, limit);

  // Clear error buffer 
  if (status & 0x80)
  {
    sprintf(pC_->outString_, "%dCER", axisIndex_);
    status = pC_->writeController();
  }

  // Read the drive power on status
  sprintf(pC_->outString_, "%dMOT?", axisIndex_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;

  // The response string is of the form "#1"
  driveOn = atoi(&pC_->inString_[1]);
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  // should this line be here?
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg MMC200CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg MMC200CreateControllerArg1 = {"MMC-200 port name", iocshArgString};
static const iocshArg MMC200CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg MMC200CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg MMC200CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg MMC200CreateControllerArg5 = {"Ignore limit flag", iocshArgInt};
static const iocshArg * const MMC200CreateControllerArgs[] = {&MMC200CreateControllerArg0,
                                                             &MMC200CreateControllerArg1,
                                                             &MMC200CreateControllerArg2,
                                                             &MMC200CreateControllerArg3,
                                                             &MMC200CreateControllerArg4,
                                                             &MMC200CreateControllerArg5};
static const iocshFuncDef MMC200CreateControllerDef = {"MMC200CreateController", 6, MMC200CreateControllerArgs};
static void MMC200CreateContollerCallFunc(const iocshArgBuf *args)
{
  MMC200CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void MMC200Register(void)
{
  iocshRegister(&MMC200CreateControllerDef, MMC200CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(MMC200Register);
}
