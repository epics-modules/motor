/*
FILENAME... HXPDriver.cpp
USAGE...    Motor driver support for the Newport Hexapod controller.

Note: This driver was tested with the v1.3.x of the firmware

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsString.h>

#include <asynOctetSyncIO.h>

#include "HXPDriver.h"
#include "hxp_drivers.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)
#define NUM_AXES 6
#define GROUP "HEXAPOD"
#define CS "Work"
#define MRES 0.00001

static const char *driverName = "HXPDriver";

/** Creates a new HXPController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] IPAddress         The Newport hexapod controller's ip address
  * \param[in] IPPort            TCP/IP port used to communicate with the hexapod controller 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
HXPController::HXPController(const char *portName, const char *IPAddress, int IPPort,
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, NUM_AXES, NUM_HXP_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  HXPAxis *pAxis;
  static const char *functionName = "HXPController::HXPController";

  axisNames_ = epicsStrDup("XYZUVW");

  IPAddress_ = epicsStrDup(IPAddress);
  IPPort_ = IPPort;

  // This socket is used for polling by the controller and all axes
  pollSocket_ = HXPTCP_ConnectToServer((char *)IPAddress, IPPort, HXP_POLL_TIMEOUT);
  if (pollSocket_ < 0) {
    printf("%s:%s: error calling TCP_ConnectToServer for pollSocket\n",
           driverName, functionName);
  }
  
  HXPFirmwareVersionGet(pollSocket_, firmwareVersion_);

  for (axis=0; axis<NUM_AXES; axis++) {
    pAxis = new HXPAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);

}


/** Creates a new HXPController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] HXPPortName  The name of the drvAsynIPPPort that was created previously to connect to the Newport hexapod controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int HXPCreateController(const char *portName, const char *IPAddress, int IPPort,
                                   int movingPollPeriod, int idlePollPeriod)
{
  HXPController *pHXPController
    = new HXPController(portName, IPAddress, IPPort, movingPollPeriod/1000., idlePollPeriod/1000.);
  pHXPController = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void HXPController::report(FILE *fp, int level)
{
  fprintf(fp, "Newport hexapod motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, NUM_AXES, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an HXPAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
HXPAxis* HXPController::getAxis(asynUser *pasynUser)
{
  return static_cast<HXPAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an HXPAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
HXPAxis* HXPController::getAxis(int axisNo)
{
  return static_cast<HXPAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the HXPAxis methods

/** Creates a new HXPAxis object.
  * \param[in] pC Pointer to the HXPController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
HXPAxis::HXPAxis(HXPController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{  
  axisName_ = pC_->axisNames_[axisNo];
  sprintf(positionerName_, "%s.%c", GROUP, (char) axisName_);

  // Couldn't a negative timeout be used here instead?
  moveSocket_ = HXPTCP_ConnectToServer(pC_->IPAddress_, pC->IPPort_, HXP_POLL_TIMEOUT);

 /* Set the poll rate on the moveSocket to a negative number, which means that SendAndReceive should do only a write, no read */
  HXPTCP_SetTimeout(moveSocket_, -0.1);
  pollSocket_ = pC_->pollSocket_;

  /* Enable gain support so that the CNEN field can be used to send
     the init command to clear a motor fault for stepper motors, even
     though they lack closed-loop support. */
  setIntegerParam(pC_->motorStatusGainSupport_, 1);

}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void HXPAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
    fprintf(fp, "    axisName %c\n", axisName_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus HXPAxis::move(double position, int relative, double baseVelocity, double slewVelocity, double acceleration)
{
  int status;
  double cur_pos[NUM_AXES];
  double rel_pos[NUM_AXES] = {};
  double *pos;
  static const char *functionName = "HXPAxis::move";

  
  if (relative) {
    rel_pos[axisNo_] = position * MRES;
    pos = rel_pos;
    status = HXPHexapodMoveIncremental(moveSocket_, GROUP, CS, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    if (status != 0 && status != -27) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: Error performing HexapodMoveIncremental[%s,%d] %d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move! */
        return asynError;
    }
  } else {
    // get current positions before moving (could an error overflow the array?)
    status = HXPGroupPositionCurrentGet(pollSocket_, GROUP, 6, (double *) &cur_pos);

    cur_pos[axisNo_] = position * MRES;
    pos = cur_pos;
    status = HXPHexapodMoveAbsolute(moveSocket_, GROUP, CS, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    if (status != 0 && status != -27) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: Error performing HexapodMoveAbsolute[%s,%d] %d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move! */
        return asynError;
    }
  }
  
  return asynSuccess;
}

asynStatus HXPAxis::home(double baseVelocity, double slewVelocity, double acceleration, int forwards)
{
  // static const char *functionName = "HXPAxis::home";

  // kill all
  HXPGroupKill(moveSocket_, GROUP);
  // initialize
  HXPGroupInitialize(moveSocket_, GROUP);
  // home
  HXPGroupHomeSearch(moveSocket_, GROUP);

  return asynSuccess;
}

asynStatus HXPAxis::stop(double acceleration )
{
  int status;
  //static const char *functionName = "HXPAxis::stop";

  status = HXPGroupMoveAbort(moveSocket_, GROUP);

  return asynSuccess;
}

asynStatus HXPAxis::setClosedLoop(bool closedLoop)
{
  int status;
  static const char *functionName = "HXPAxis::setClosedLoop";

  if (closedLoop) {
    status = HXPGroupMotionEnable(pollSocket_, GROUP);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: [%s,%d]: error calling GroupMotionEnable status=%d\n",
                 driverName, functionName, pC_->portName, axisNo_, status);
    } else {
      asynPrint(pasynUser_, ASYN_TRACE_FLOW,
                "%s:%s: set XPS %s, axis %d closed loop enable\n",
                 driverName, functionName, pC_->portName, axisNo_);
    }
  } else {
    status = HXPGroupMotionDisable(pollSocket_, GROUP);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: [%s,%d]: error calling GroupMotionDisable status=%d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
    } else {
      asynPrint(pasynUser_, ASYN_TRACE_FLOW,
                "%s:%s: motorAxisSetInteger set XPS %s, axis %d closed loop disable\n",
                driverName, functionName, pC_->portName, axisNo_);
    }
  }
  
  return (asynStatus)status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus HXPAxis::poll(bool *moving)
{ 
  int status;
  //char readResponse[25];

  static const char *functionName = "HXPAxis::poll";

  status = HXPGroupStatusGet(pollSocket_, 
                          GROUP, 
                          &axisStatus_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupStatusGet status=%d; pollSocket=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status, pollSocket_);
    goto done;
  }

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: [%s,%d]: %s axisStatus=%d\n",
            driverName, functionName, pC_->portName, axisNo_, positionerName_, axisStatus_);
  /* Set the status */
  //setIntegerParam(pC_->HXPStatus_, axisStatus_);

  /* If the group is not moving then the axis is not moving */
  if ((axisStatus_ < 43) || (axisStatus_ > 48))
    moving_ = false;
  else
    moving_ = true;

  /* Set the axis done parameter */
  *moving = moving_;
  //if (deferredMove_) *moving = true;
  setIntegerParam(pC_->motorStatusDone_, *moving?0:1);

  /*Read the controller software limits in case these have been changed by a TCL script.*/

  /*Test for states that mean we cannot move an axis (disabled, uninitialised, etc.) 
    and set problem bit in MSTA.*/
  if ((axisStatus_ < 10) || ((axisStatus_ >= 20) && (axisStatus_ <= 42)) ||
      (axisStatus_ == 50) || (axisStatus_ == 64)) {
    if ( (pC_->noDisableError_ > 0) && (axisStatus_==20) ) {
      setIntegerParam(pC_->motorStatusProblem_, 0);             
    } else {
      asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
          "%s:%s: [%s,%d]: in unintialised/disabled/not referenced. XPS State Code: %d\n",
          driverName, functionName, pC_->portName, axisNo_, axisStatus_);
      setIntegerParam(pC_->motorStatusProblem_, 1);
    }
  } else {
    setIntegerParam(pC_->motorStatusProblem_, 0);
  }

  status = HXPGroupPositionCurrentGet(pollSocket_,
                                   positionerName_,
                                   1,
                                   &encoderPosition_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupPositionCurrentGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  //setDoubleParam(pC_->motorEncoderPosition_, (encoderPosition_/stepSize_));
  setDoubleParam(pC_->motorEncoderPosition_, encoderPosition_ / MRES);

  status = HXPGroupPositionSetpointGet(pollSocket_,
                                   positionerName_,
                                   1,
                                   &setpointPosition_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupPositionSetpointGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  //setDoubleParam(pC_->motorPosition_, (setpointPosition_/stepSize_));
  setDoubleParam(pC_->motorPosition_, setpointPosition_ / MRES);

  // limit check?
  
  // dir flag?

  done:
  setIntegerParam(pC_->motorStatusProblem_, status ? 1:0);
  callParamCallbacks();
  return status ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg HXPCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg HXPCreateControllerArg1 = {"IP address", iocshArgString};
static const iocshArg HXPCreateControllerArg2 = {"Port", iocshArgInt};
static const iocshArg HXPCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg HXPCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const HXPCreateControllerArgs[] = {&HXPCreateControllerArg0,
                                                             &HXPCreateControllerArg1,
                                                             &HXPCreateControllerArg2,
                                                             &HXPCreateControllerArg3,
                                                             &HXPCreateControllerArg4};
static const iocshFuncDef HXPCreateControllerDef = {"HXPCreateController", 5, HXPCreateControllerArgs};
static void HXPCreateControllerCallFunc(const iocshArgBuf *args)
{
  HXPCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void HXPRegister(void)
{
  iocshRegister(&HXPCreateControllerDef, HXPCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(HXPRegister);
}
