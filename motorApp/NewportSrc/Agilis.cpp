/*
FILENAME... AgilisDriver.cpp
USAGE...    Motor driver support for the Newport MCB-4B controller.

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

#include "Agilis.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new AgilisController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] AgilisPortName     The name of the drvAsynSerialPort that was created previously to connect to the Agilis controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
AgilisController::AgilisController(const char *portName, const char *AgilisPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_AGILIS_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  static const char *functionName = "AgilisController::AgilisController";

  /* Connect to Agilis controller */
  status = pasynOctetSyncIO->connect(AgilisPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to Agilis controller\n",
      functionName);
  }
  
  // Put the controller in remote mode
  sprintf(outString_, "MR");
  status = writeController();

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new AgilisController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] AgilisPortName       The name of the drvAsynIPPPort that was created previously to connect to the Agilis controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" {

int AgilisCreateController(const char *portName, const char *AgilisPortName, int numAxes, 
                                      int movingPollPeriod, int idlePollPeriod)
{
  new AgilisController(portName, AgilisPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

asynStatus AgilisCreateAxis(const char *AgilisName,  /* specify which controller by port name */
                            int axis,                /* axis number 0-7 */
                            int hasLimits,           /* Actuator has limits 0 or 1 */
                            int forwardAmplitude,    /* Step amplitude in forward direction */
                            int reverseAmplitude)    /* Step amplitude in reverse direction */
{
  AgilisController *pC;
  static const char *functionName = "Agilis::AgilisCreateAxis";

  pC = (AgilisController*) findAsynPortDriver(AgilisName);
  if (!pC) {
    printf("%s: Error port %s not found\n",
           functionName, AgilisName);
    return asynError;
  }  
  pC->lock();
  new AgilisAxis(pC, axis, hasLimits ? true:false, forwardAmplitude, reverseAmplitude);
  pC->unlock();
  return asynSuccess;
}
} // extern "C" 


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void AgilisController::report(FILE *fp, int level)
{
  fprintf(fp, "Agilis motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an AgilisAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
AgilisAxis* AgilisController::getAxis(asynUser *pasynUser)
{
  return static_cast<AgilisAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an AgilisAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
AgilisAxis* AgilisController::getAxis(int axisNo)
{
  return static_cast<AgilisAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the AgilisAxis methods

/** Creates a new AgilisAxis object.
  * \param[in] pC Pointer to the AgilisController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
AgilisAxis::AgilisAxis(AgilisController *pC, int axisNo, bool hasLimits, 
                       int forwardAmplitude, int reverseAmplitude)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), hasLimits_(hasLimits), 
    forwardAmplitude_(forwardAmplitude), reverseAmplitude_(reverseAmplitude),
    currentPosition_(0), positionOffset_(0), axisID_(axisNo+1)
{
  if (forwardAmplitude_ <= 0) forwardAmplitude_ = 50;
  if (reverseAmplitude_ >= 0) forwardAmplitude_ = -50;
  sprintf(pC_->outString_, "%dSU%d", axisID_, forwardAmplitude_);
  pC_->writeController();
  sprintf(pC_->outString_, "%dSU%d", axisID_, reverseAmplitude_);
  pC_->writeController();
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void AgilisAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d, hasLimits=%d, forwardAmplitude=%d, reverseAmplitude=%d\n",
            axisID_, hasLimits_, forwardAmplitude_, reverseAmplitude_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus AgilisAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int steps = NINT(position);
  // static const char *functionName = "AgilisAxis::move";

  if (relative) {
    sprintf(pC_->outString_, "%dPR%d", axisID_, steps);
  } else {
    if (!hasLimits_) {
      steps = NINT(position - currentPosition_);
      sprintf(pC_->outString_, "%dPR%d", axisID_, steps);
    } else {
      sprintf(pC_->outString_, "%dPA%d", axisID_, steps);
    }
  }
  status = pC_->writeController();
  return status;
}

asynStatus AgilisAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "AgilisAxis::home";

  sprintf(pC_->outString_, "%dML4", axisID_);
  status = pC_->writeController();
  return status;
}

asynStatus AgilisAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int speed;
  //static const char *functionName = "AgilisAxis::moveVelocity";

  if      (abs(maxVelocity) <= 5)   speed = 1;
  else if (abs(maxVelocity) <= 100) speed = 2;
  else if (abs(maxVelocity) <= 666) speed = 4;
  else                              speed = 3;
  if (maxVelocity < 0) speed = -speed;
  sprintf(pC_->outString_, "%dJA%d", axisID_, speed);
  status = pC_->writeController();
  return status;
}

asynStatus AgilisAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "AgilisAxis::stop";

  sprintf(pC_->outString_, "%dST", axisID_);
  status = pC_->writeController();
  return status;
}

asynStatus AgilisAxis::setPosition(double position)
{
  //static const char *functionName = "AgilisAxis::setPosition";

  positionOffset_ = NINT(position) - currentPosition_;
  return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus AgilisAxis::poll(bool *moving)
{ 
  int done;
  int lim, limit=0;
  int position;
  asynStatus comStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "%dTP", axisID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1TPxxx"
  position = atoi(&pC_->inString_[3]);
  currentPosition_ = position + positionOffset_;
  setDoubleParam(pC_->motorPosition_, double(currentPosition_));

  // Read the moving status of this motor
  sprintf(pC_->outString_, "%dTS", axisID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1TSn"
  done = (pC_->inString_[3] == '0') ? 1:0;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // Read the limit status
  sprintf(pC_->outString_, "PH");
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
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

/** Code for iocsh registration */
static const iocshArg AgilisCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg AgilisCreateControllerArg1 = {"MCB-4B port name", iocshArgString};
static const iocshArg AgilisCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg AgilisCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg AgilisCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const AgilisCreateControllerArgs[] = {&AgilisCreateControllerArg0,
                                                             &AgilisCreateControllerArg1,
                                                             &AgilisCreateControllerArg2,
                                                             &AgilisCreateControllerArg3,
                                                             &AgilisCreateControllerArg4};
static const iocshFuncDef AgilisCreateControllerDef = {"AgilisCreateController", 5, AgilisCreateControllerArgs};
static void AgilisCreateContollerCallFunc(const iocshArgBuf *args)
{
  AgilisCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

/* AgilisCreateAxis */
static const iocshArg AgilisCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg AgilisCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg AgilisCreateAxisArg2 = {"Has Limits", iocshArgInt};
static const iocshArg AgilisCreateAxisArg3 = {"Forward amplitude", iocshArgInt};
static const iocshArg AgilisCreateAxisArg4 = {"Reverse amplitude", iocshArgInt};
static const iocshArg * const AgilisCreateAxisArgs[] = {&AgilisCreateAxisArg0,
                                                        &AgilisCreateAxisArg1,
                                                        &AgilisCreateAxisArg2,
                                                        &AgilisCreateAxisArg3,
                                                        &AgilisCreateAxisArg4};
static const iocshFuncDef AgilisCreateAxisDef = {"AgilisCreateAxis", 5, AgilisCreateAxisArgs};

static void AgilisCreateAxisCallFunc(const iocshArgBuf *args)
{
  AgilisCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}

static void AgilisRegister(void)
{
  iocshRegister(&AgilisCreateControllerDef, AgilisCreateContollerCallFunc);
  iocshRegister(&AgilisCreateAxisDef, AgilisCreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(AgilisRegister);
}
