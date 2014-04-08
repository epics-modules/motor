/*
FILENAME... AG_UC.cpp
USAGE...    Motor driver support for the Newport Agilis UC series controllers.

Mark Rivers
April 11, 2013

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "AG_UC.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

#define AGILIS_TIMEOUT 2.0
#define WRITE_DELAY 0.01

/** Creates a new AG_UCController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName    The name of the drvAsynSerialPort that was created previously to connect to the Agilis controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
AG_UCController::AG_UCController(const char *portName, const char *serialPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_AG_UC_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  static const char *functionName = "AG_UCController::AG_UCController";

  /* Connect to Agilis controller */
  status = pasynOctetSyncIO->connect(serialPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to Agilis controller\n",
      functionName);
  }
  
  // Reset the controller
  sprintf(outString_, "RS");
  status = writeAgilis(0);
  // Reset takes some time?
  epicsThreadSleep(0.5);

  // Put the controller in remote mode
  sprintf(outString_, "MR");
  status = writeAgilis(0);
  
  // Flush any characters that controller has, read firmware version
  sprintf(outString_, "VE");
  status = writeReadController();
  // Seems to be necessary to delay a short time between writes
  epicsThreadSleep(WRITE_DELAY);  
  printf("Agilis controller firmware version = %s\n", inString_);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot read version information from Agilis controller\n",
      functionName);
    return;
  }
  strcpy(controllerVersion_, inString_);
  // Figure out what model this is
  if (strstr(controllerVersion_, "AG-UC2")) {
    AG_UCModel_ = ModelAG_UC2;
  } 
  else if (strstr(controllerVersion_, "AG-UC8PC")) {
    AG_UCModel_ = ModelAG_UC8PC;
  } 
  else if (strstr(controllerVersion_, "AG-UC8")) {
    AG_UCModel_ = ModelAG_UC8;
  } 
  else {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s: unknown model, firmware string=%s\n",
      functionName, controllerVersion_);
    return;
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new AG_UCController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName       The name of the drvAsynIPPPort that was created previously to connect to the Agilis controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" {

int AG_UCCreateController(const char *portName, const char *serialPortName, int numAxes, 
                                      int movingPollPeriod, int idlePollPeriod)
{
  new AG_UCController(portName, serialPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

asynStatus AG_UCCreateAxis(const char *AG_UCName,  /* specify which controller by port name */
                           int axis,                /* axis number 0-7 */
                           int hasLimits,           /* Actuator has limits 0 or 1 */
                           int forwardAmplitude,    /* Step amplitude in forward direction */
                           int reverseAmplitude)    /* Step amplitude in reverse direction */
{
  AG_UCController *pC;
  static const char *functionName = "AG_UCCreateAxis";

  pC = (AG_UCController*) findAsynPortDriver(AG_UCName);
  if (!pC) {
    printf("%s: Error port %s not found\n",
           functionName, AG_UCName);
    return asynError;
  }  
  pC->lock();
  new AG_UCAxis(pC, axis, hasLimits ? true:false, forwardAmplitude, reverseAmplitude);
  pC->unlock();
  return asynSuccess;
}
} // extern "C" 


/** Writes a string to the controller.
  * Calls writeAgilis() with a default location of the string to write and a default timeout. */ 
asynStatus AG_UCController::writeAgilis(int channelID)
{
  return writeAgilis(channelID, outString_, AGILIS_TIMEOUT);
}


/** Writes a string to the controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus AG_UCController::writeAgilis(int channelID, const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeAgilis";
  
  // If channelID is non-zero then we need to send the CC command first
  
  if (channelID != 0) setChannel(channelID);

  status = pasynOctetSyncIO->write(pasynUserController_, output,
                                   strlen(output), timeout, &nwrite);
                                   
  // Seems to be necessary to delay a short time between writes
  epicsThreadSleep(WRITE_DELAY);
                                  
  return status ;
}

asynStatus AG_UCController::setChannel(int channelID)
{
  char tempString[40];
  asynStatus status;
  
  sprintf(tempString, "CC%d", channelID);
  status = writeController(tempString, AGILIS_TIMEOUT);
  // Seems to be necessary to delay a short time between writes
  epicsThreadSleep(WRITE_DELAY);
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void AG_UCController::report(FILE *fp, int level)
{
  fprintf(fp, "Agilis UC motor driver %s, model=%d, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, AG_UCModel_, numAxes_, movingPollPeriod_, idlePollPeriod_);
  fprintf(fp, "controller version %s\n", 
    controllerVersion_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an AG_UCAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
AG_UCAxis* AG_UCController::getAxis(asynUser *pasynUser)
{
  return static_cast<AG_UCAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an AG_UCAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
AG_UCAxis* AG_UCController::getAxis(int axisNo)
{
  return static_cast<AG_UCAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the AG_UCAxis methods

/** Creates a new AG_UCAxis object.
  * \param[in] pC Pointer to the AG_UCController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
AG_UCAxis::AG_UCAxis(AG_UCController *pC, int axisNo, bool hasLimits, 
                       int forwardAmplitude, int reverseAmplitude)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), hasLimits_(hasLimits), 
    forwardAmplitude_(forwardAmplitude), reverseAmplitude_(reverseAmplitude),
    currentPosition_(0), positionOffset_(0)
{
  axisID_ = (axisNo % 2) + 1;  // Either 1 or 2
  if (pC_->AG_UCModel_ == ModelAG_UC2) {
    channelID_ = 0;
  } else {
    channelID_ = axisNo/2 + 1;
  }
  if (forwardAmplitude_ <= 0) forwardAmplitude_ = 50;
  if (reverseAmplitude_ >= 0) forwardAmplitude_ = -50;
  sprintf(pC_->outString_, "%dSU%d", axisID_, forwardAmplitude_);
  writeAgilis();
  sprintf(pC_->outString_, "%dSU%d", axisID_, reverseAmplitude_);
  writeAgilis();
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void AG_UCAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axisID=%d, channelID=%d, hasLimits=%d\n"
                "  forwardAmplitude=%d, reverseAmplitude=%d\n"
                "  currentPosition=%d, positionOffset=%d\n",
            axisID_, channelID_, hasLimits_, 
            forwardAmplitude_, reverseAmplitude_,
            currentPosition_, positionOffset_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus AG_UCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int steps = NINT(position);
  // static const char *functionName = "AG_UCAxis::move";

  if (relative) {
    sprintf(pC_->outString_, "%dPR%d", axisID_, steps);
  } else {
    steps = NINT(position - currentPosition_);
    sprintf(pC_->outString_, "%dPR%d", axisID_, steps);
  }
  status = writeAgilis();
  return status;
}

int AG_UCAxis::velocityToSpeedCode(double velocity)
{
  int speed;
  if      (fabs(velocity) <= 5)   speed = 1;
  else if (fabs(velocity) <= 100) speed = 2;
  else if (fabs(velocity) <= 666) speed = 4;
  else                            speed = 3;
  if (velocity < 0) speed = -speed;
  return speed;
}

asynStatus AG_UCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  //static const char *functionName = "AG_UCAxis::home";

  if (!hasLimits_) return asynError;
  sprintf(pC_->outString_, "%dMV%d", axisID_, velocityToSpeedCode(maxVelocity));
  status = writeAgilis();
  return status;
}

asynStatus AG_UCAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  //static const char *functionName = "AG_UCAxis::moveVelocity";

  sprintf(pC_->outString_, "%dJA%d", axisID_, velocityToSpeedCode(maxVelocity));
  status = writeAgilis();
  return status;
}

asynStatus AG_UCAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "AG_UCAxis::stop";

  sprintf(pC_->outString_, "%dST", axisID_);
  status = writeAgilis();
  return status;
}

asynStatus AG_UCAxis::setPosition(double position)
{
  //static const char *functionName = "AG_UCAxis::setPosition";

  positionOffset_ = NINT(position) - currentPosition_;
  return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus AG_UCAxis::poll(bool *moving)
{ 
  int done;
  int lim, limit=0;
  int position;
  asynStatus comStatus;

  //  Select this channel
  pC_->setChannel(channelID_);
  
  // Read the current motor position
  sprintf(pC_->outString_, "%dTP", axisID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // Seems to be necessary to delay a short time between writes
  epicsThreadSleep(WRITE_DELAY);  
  // The response string is of the form "1TPxxx"
  position = atoi(&pC_->inString_[3]);
  currentPosition_ = position + positionOffset_;
  setDoubleParam(pC_->motorPosition_, double(currentPosition_));

  // Read the moving status of this motor
  sprintf(pC_->outString_, "%dTS", axisID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // Seems to be necessary to delay a short time between writes
  epicsThreadSleep(WRITE_DELAY);  
  // The response string is of the form "1TSn"
  done = (pC_->inString_[3] == '0') ? 1:0;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // Read the limit status
  sprintf(pC_->outString_, "PH");
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // Seems to be necessary to delay a short time between writes
  epicsThreadSleep(WRITE_DELAY);  
  // The response string is of the form "PHn"
  lim = atoi(&pC_->inString_[2]);
  if ((axisID_ == 1) && (lim == 1 || lim == 3)) limit = 1;
  if ((axisID_ == 2) && (lim == 3 || lim == 3)) limit = 1;
  setIntegerParam(pC_->motorStatusLowLimit_, limit);
  setIntegerParam(pC_->motorStatusHighLimit_, limit);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Writes a string to the controller.
  * Calls writeAgilis() with a default location of the string to write and a default timeout. */ 
asynStatus AG_UCAxis::writeAgilis()
{
  return pC_->writeAgilis(channelID_, pC_->outString_, AGILIS_TIMEOUT);
}


asynStatus AG_UCAxis::writeAgilis(const char *output, double timeout)
{
  return pC_->writeAgilis(channelID_, output, timeout);
}

/** Code for iocsh registration */
static const iocshArg AG_UCCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg AG_UCCreateControllerArg1 = {"Serial port name", iocshArgString};
static const iocshArg AG_UCCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg AG_UCCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg AG_UCCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const AG_UCCreateControllerArgs[] = {&AG_UCCreateControllerArg0,
                                                             &AG_UCCreateControllerArg1,
                                                             &AG_UCCreateControllerArg2,
                                                             &AG_UCCreateControllerArg3,
                                                             &AG_UCCreateControllerArg4};
static const iocshFuncDef AG_UCCreateControllerDef = {"AG_UCCreateController", 5, AG_UCCreateControllerArgs};
static void AG_UCCreateContollerCallFunc(const iocshArgBuf *args)
{
  AG_UCCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

/* AG_UCCreateAxis */
static const iocshArg AG_UCCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg AG_UCCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg AG_UCCreateAxisArg2 = {"Has Limits", iocshArgInt};
static const iocshArg AG_UCCreateAxisArg3 = {"Forward amplitude", iocshArgInt};
static const iocshArg AG_UCCreateAxisArg4 = {"Reverse amplitude", iocshArgInt};
static const iocshArg * const AG_UCCreateAxisArgs[] = {&AG_UCCreateAxisArg0,
                                                        &AG_UCCreateAxisArg1,
                                                        &AG_UCCreateAxisArg2,
                                                        &AG_UCCreateAxisArg3,
                                                        &AG_UCCreateAxisArg4};
static const iocshFuncDef AG_UCCreateAxisDef = {"AG_UCCreateAxis", 5, AG_UCCreateAxisArgs};

static void AG_UCCreateAxisCallFunc(const iocshArgBuf *args)
{
  AG_UCCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}

static void AG_UCRegister(void)
{
  iocshRegister(&AG_UCCreateControllerDef, AG_UCCreateContollerCallFunc);
  iocshRegister(&AG_UCCreateAxisDef, AG_UCCreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(AG_UCRegister);
}
