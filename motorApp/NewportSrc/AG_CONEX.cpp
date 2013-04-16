/*
FILENAME... AG_CONEX.cpp
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

#include "AG_CONEX.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

#define AGILIS_TIMEOUT 2.0
#define LINUX_WRITE_DELAY 0.1

/** Creates a new AG_CONEXController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName    The name of the drvAsynSerialPort that was created previously to connect to the Agilis controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
AG_CONEXController::AG_CONEXController(const char *portName, const char *serialPortName, int controllerID, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, 1, NUM_AG_CONEX_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0),  // Default priority and stack size
   controllerID_(controllerID)
                    
{
  asynStatus status;
  static const char *functionName = "AG_CONEXController::AG_CONEXController";

  /* Connect to Agilis controller */
  status = pasynOctetSyncIO->connect(serialPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to Agilis controller\n",
      functionName);
  }
  
  // Flush any characters that controller has, read firmware version
  sprintf(outString_, "%dVE", controllerID_);
  status = writeReadController();
  printf("Agilis controller firmware version = %s\n", inString_);
  
  // Create the axis object
  new AG_CONEXAxis(this);

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new AG_CONEXController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName       The name of the drvAsynIPPPort that was created previously to connect to the Agilis controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" {

int AG_CONEXCreateController(const char *portName, const char *serialPortName, int controllerID, 
                             int movingPollPeriod, int idlePollPeriod)
{
  new AG_CONEXController(portName, serialPortName, controllerID, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

} // extern "C" 


/** Writes a string to the controller.
  * Calls writeAgilis() with a default location of the string to write and a default timeout. */ 
asynStatus AG_CONEXController::writeAgilis()
{
  return writeAgilis(outString_, AGILIS_TIMEOUT);
}

/** Writes a string to the controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus AG_CONEXController::writeAgilis(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeAgilis";
  
  status = pasynOctetSyncIO->write(pasynUserController_, output,
                                   strlen(output), timeout, &nwrite);
                                   
  // On Linux it seems to be necessary to delay a short time between writes
  #ifdef linux
  epicsThreadSleep(LINUX_WRITE_DELAY);
  #endif
                                  
  return status ;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void AG_CONEXController::report(FILE *fp, int level)
{
  fprintf(fp, "Agilis CONEX motor driver %s, controllerID=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, controllerID_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an AG_CONEXAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
AG_CONEXAxis* AG_CONEXController::getAxis(asynUser *pasynUser)
{
  return static_cast<AG_CONEXAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an AG_CONEXAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
AG_CONEXAxis* AG_CONEXController::getAxis(int axisNo)
{
  return static_cast<AG_CONEXAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the AG_CONEXAxis methods

/** Creates a new AG_CONEXAxis object.
  * \param[in] pC Pointer to the AG_CONEXController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
AG_CONEXAxis::AG_CONEXAxis(AG_CONEXController *pC)
  : asynMotorAxis(pC, 0),
    pC_(pC), 
    currentPosition_(0.), positionOffset_(0.)
{
  // Read the encoder increment
  sprintf(pC_->outString_, "%dSU?", pC->controllerID_);
  pC_->writeReadController();
  encoderIncrement_ = atof(&pC_->inString_[3]);
  
  // Read the low and high software limits
  sprintf(pC_->outString_, "%dSL?", pC->controllerID_);
  pC_->writeReadController();
  lowLimit_ = atof(&pC_->inString_[3]);
  sprintf(pC_->outString_, "%dSR?", pC->controllerID_);
  pC_->writeReadController();
  highLimit_ = atof(&pC_->inString_[3]);
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void AG_CONEXAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  currentPosition=%f, positionOffset=%f, encoderIncrement=%f, lowLimit=%f, highLimit=%f\n",
            currentPosition_, positionOffset_, encoderIncrement_, lowLimit_, highLimit_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus AG_CONEXAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "AG_CONEXAxis::move";

  if (relative) {
    sprintf(pC_->outString_, "%dPR%f", pC_->controllerID_, position*encoderIncrement_);
  } else {
    sprintf(pC_->outString_, "%dPA%f", pC_->controllerID_, (position-positionOffset_)*encoderIncrement_);
  }
  status = pC_->writeAgilis();
  return status;
}

asynStatus AG_CONEXAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  //static const char *functionName = "AG_CONEXAxis::home";

  sprintf(pC_->outString_, "%dOR", pC_->controllerID_);
  status = pC_->writeAgilis();
  return status;
}

asynStatus AG_CONEXAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  double position;
  //static const char *functionName = "AG_CONEXAxis::moveVelocity";

  // The CONEX does not have a jog command.  Move almost to soft limit.
  if (maxVelocity > 0) position = lowLimit_  + 0.999*(highLimit_ - lowLimit_);
  else                 position = highLimit_ - 0.999*(highLimit_ - lowLimit_);
  
  sprintf(pC_->outString_, "%dPA%f", pC_->controllerID_, position);
  status = pC_->writeAgilis();
  return status;
}

asynStatus AG_CONEXAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "AG_CONEXAxis::stop";

  sprintf(pC_->outString_, "%dST", pC_->controllerID_);
  status = pC_->writeAgilis();
  return status;
}

asynStatus AG_CONEXAxis::setPosition(double position)
{
  //static const char *functionName = "AG_CONEXAxis::setPosition";

  positionOffset_ = position - currentPosition_;
  return asynSuccess;
}

asynStatus AG_CONEXAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  //static const char *functionName = "AG_CONEXAxis::setClosedLoop";

  sprintf(pC_->outString_, "%dMM%d", pC_->controllerID_, closedLoop ? 1 : 0);
  status = pC_->writeAgilis();
  return status;
}



/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus AG_CONEXAxis::poll(bool *moving)
{ 
  int done=1;
  double position;
  unsigned int status;
  int count;
  asynStatus comStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "%dTP", pC_->controllerID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1TPxxx"
  position = atof(&pC_->inString_[3]);
  currentPosition_ = (position + positionOffset_)/encoderIncrement_;
  setDoubleParam(pC_->motorPosition_, currentPosition_);

  // Read the moving status of this motor
  sprintf(pC_->outString_, "%dTS", pC_->controllerID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1TSabcdef"
  count = sscanf(pC_->inString_, "%*dTS%*4c%x", &status);
  if (count != 1) goto skip;
  if ((status == 0x1e) || (status == 0x28)) done = 0;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  setIntegerParam(pC_->motorStatusLowLimit_, 0);
  setIntegerParam(pC_->motorStatusHighLimit_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg AG_CONEXCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg AG_CONEXCreateControllerArg1 = {"Serial port name", iocshArgString};
static const iocshArg AG_CONEXCreateControllerArg2 = {"Controller ID", iocshArgInt};
static const iocshArg AG_CONEXCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg AG_CONEXCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const AG_CONEXCreateControllerArgs[] = {&AG_CONEXCreateControllerArg0,
                                                                &AG_CONEXCreateControllerArg1,
                                                                &AG_CONEXCreateControllerArg2,
                                                                &AG_CONEXCreateControllerArg3,
                                                                &AG_CONEXCreateControllerArg4};
static const iocshFuncDef AG_CONEXCreateControllerDef = {"AG_CONEXCreateController", 5, AG_CONEXCreateControllerArgs};
static void AG_CONEXCreateContollerCallFunc(const iocshArgBuf *args)
{
  AG_CONEXCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void AG_CONEXRegister(void)
{
  iocshRegister(&AG_CONEXCreateControllerDef, AG_CONEXCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(AG_CONEXRegister);
}
