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

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "hxp_drivers.h"
#include "HXPDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)
#define NUM_AXES 6
#define GROUP "HEXAPOD"
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

  createParam(HXPMoveCoordSysString,          asynParamInt32,   &HXPMoveCoordSys_);
  createParam(HXPStatusString,                asynParamInt32,   &HXPStatus_);
  createParam(HXPErrorString,                 asynParamInt32,   &HXPError_);
  createParam(HXPErrorDescString,             asynParamOctet,   &HXPErrorDesc_);
  createParam(HXPMoveAllString,               asynParamInt32,   &HXPMoveAll_);
  createParam(HXPMoveAllTargetXString,        asynParamFloat64, &HXPMoveAllTargetX_);
  createParam(HXPMoveAllTargetYString,        asynParamFloat64, &HXPMoveAllTargetY_);
  createParam(HXPMoveAllTargetZString,        asynParamFloat64, &HXPMoveAllTargetZ_);
  createParam(HXPMoveAllTargetUString,        asynParamFloat64, &HXPMoveAllTargetU_);
  createParam(HXPMoveAllTargetVString,        asynParamFloat64, &HXPMoveAllTargetV_);
  createParam(HXPMoveAllTargetWString,        asynParamFloat64, &HXPMoveAllTargetW_);
  createParam(HXPCoordSysReadAllString,       asynParamInt32,   &HXPCoordSysReadAll_);
  createParam(HXPCoordSysToolXString,         asynParamFloat64, &HXPCoordSysToolX_);
  createParam(HXPCoordSysToolYString,         asynParamFloat64, &HXPCoordSysToolY_); 
  createParam(HXPCoordSysToolZString,         asynParamFloat64, &HXPCoordSysToolZ_); 
  createParam(HXPCoordSysToolUString,         asynParamFloat64, &HXPCoordSysToolU_); 
  createParam(HXPCoordSysToolVString,         asynParamFloat64, &HXPCoordSysToolV_); 
  createParam(HXPCoordSysToolWString,         asynParamFloat64, &HXPCoordSysToolW_); 
  createParam(HXPCoordSysWorkXString,         asynParamFloat64, &HXPCoordSysWorkX_); 
  createParam(HXPCoordSysWorkYString,         asynParamFloat64, &HXPCoordSysWorkY_);
  createParam(HXPCoordSysWorkZString,         asynParamFloat64, &HXPCoordSysWorkZ_);
  createParam(HXPCoordSysWorkUString,         asynParamFloat64, &HXPCoordSysWorkU_);
  createParam(HXPCoordSysWorkVString,         asynParamFloat64, &HXPCoordSysWorkV_);
  createParam(HXPCoordSysWorkWString,         asynParamFloat64, &HXPCoordSysWorkW_);
  createParam(HXPCoordSysBaseXString,         asynParamFloat64, &HXPCoordSysBaseX_);
  createParam(HXPCoordSysBaseYString,         asynParamFloat64, &HXPCoordSysBaseY_);
  createParam(HXPCoordSysBaseZString,         asynParamFloat64, &HXPCoordSysBaseZ_);
  createParam(HXPCoordSysBaseUString,         asynParamFloat64, &HXPCoordSysBaseU_);
  createParam(HXPCoordSysBaseVString,         asynParamFloat64, &HXPCoordSysBaseV_);
  createParam(HXPCoordSysBaseWString,         asynParamFloat64, &HXPCoordSysBaseW_);
  createParam(HXPCoordSysSetString,           asynParamInt32,   &HXPCoordSysSet_);
  createParam(HXPCoordSysToSetString,         asynParamInt32,   &HXPCoordSysToSet_);
  createParam(HXPCoordSysSetXString,          asynParamFloat64, &HXPCoordSysSetX_);
  createParam(HXPCoordSysSetYString,          asynParamFloat64, &HXPCoordSysSetY_);
  createParam(HXPCoordSysSetZString,          asynParamFloat64, &HXPCoordSysSetZ_);
  createParam(HXPCoordSysSetUString,          asynParamFloat64, &HXPCoordSysSetU_);
  createParam(HXPCoordSysSetVString,          asynParamFloat64, &HXPCoordSysSetV_);
  createParam(HXPCoordSysSetWString,          asynParamFloat64, &HXPCoordSysSetW_);

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
  int coordSys;
  
  getIntegerParam(HXPMoveCoordSys_, &coordSys);
  
  fprintf(fp, "Newport hexapod motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f, coordSys=%d\n", 
    this->portName, NUM_AXES, movingPollPeriod_, idlePollPeriod_, coordSys);

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


/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is HXPMoveAll_ then it calls moves all motors with a single command
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus HXPController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  HXPAxis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  
  if (function == HXPMoveAll_)
  {
    /* if value == 1: motors are moved
       if value == 0: nothing is done and parameter is simply reset */
    if (value == 1)
    {
      moveAll(pAxis);
    }
  }
  else if (function == HXPCoordSysReadAll_)
  {
    if (value == 1)
    {
      readAllCS(pAxis);
    }
  }
  else if (function == HXPCoordSysSet_)
  {
    if (value == 1)
    {
      setCS(pAxis);
    }
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


/** 
  * Moves all hexpod axes to new target positions
  */
int HXPController::moveAll(HXPAxis *pAxis)
{
  int status;
  double x, y, z, u, v, w;
  
  getDoubleParam(0, HXPMoveAllTargetX_, &x);
  getDoubleParam(0, HXPMoveAllTargetY_, &y);
  getDoubleParam(0, HXPMoveAllTargetZ_, &z);
  getDoubleParam(0, HXPMoveAllTargetU_, &u);
  getDoubleParam(0, HXPMoveAllTargetV_, &v);
  getDoubleParam(0, HXPMoveAllTargetW_, &w);

  status = HXPHexapodMoveAbsolute(pAxis->moveSocket_, GROUP, "Work", x, y, z, u, v, w);

  postError(pAxis, status);

  /* callParamCallback() is called from postError, so it isn't needed here */

  return status;
}

/**
  * Reads the Tool, Work, and Base coordinate-system definitions
  */
int HXPController::readAllCS(HXPAxis *pAxis)
{
  int status;
  double x, y, z, u, v, w;

  status = HXPHexapodCoordinateSystemGet(pAxis->moveSocket_, GROUP, "Tool", &x, &y , &z, &u, &v, &w);

  setDoubleParam(0, HXPCoordSysToolX_, x);
  setDoubleParam(0, HXPCoordSysToolY_, y);
  setDoubleParam(0, HXPCoordSysToolZ_, z);
  setDoubleParam(0, HXPCoordSysToolU_, u);
  setDoubleParam(0, HXPCoordSysToolV_, v);
  setDoubleParam(0, HXPCoordSysToolW_, w);

  postError(pAxis, status);

  status = HXPHexapodCoordinateSystemGet(pAxis->moveSocket_, GROUP, "Work", &x, &y , &z, &u, &v, &w);

  setDoubleParam(0, HXPCoordSysWorkX_, x);
  setDoubleParam(0, HXPCoordSysWorkY_, y);
  setDoubleParam(0, HXPCoordSysWorkZ_, z);
  setDoubleParam(0, HXPCoordSysWorkU_, u);
  setDoubleParam(0, HXPCoordSysWorkV_, v);
  setDoubleParam(0, HXPCoordSysWorkW_, w);

  postError(pAxis, status);
  
  status = HXPHexapodCoordinateSystemGet(pAxis->moveSocket_, GROUP, "Base", &x, &y , &z, &u, &v, &w);

  setDoubleParam(0, HXPCoordSysBaseX_, x);
  setDoubleParam(0, HXPCoordSysBaseY_, y);
  setDoubleParam(0, HXPCoordSysBaseZ_, z);
  setDoubleParam(0, HXPCoordSysBaseU_, u);
  setDoubleParam(0, HXPCoordSysBaseV_, v);
  setDoubleParam(0, HXPCoordSysBaseW_, w);

  postError(pAxis, status);

  /* callParamCallback() is called from postError, so it isn't needed here */
  
  return status;
}

/** 
  * Redefine the origins of the hexapod coordinate-systems
  */
int HXPController::setCS(HXPAxis *pAxis)
{
  int status = 0;
  int cs;     // 0=None,1=Work,2=Tool,3=Base
  double x, y, z, u, v, w;
  
  getIntegerParam(0, HXPCoordSysToSet_, &cs);
  getDoubleParam(0, HXPCoordSysSetX_, &x);
  getDoubleParam(0, HXPCoordSysSetY_, &y);
  getDoubleParam(0, HXPCoordSysSetZ_, &z);
  getDoubleParam(0, HXPCoordSysSetU_, &u);
  getDoubleParam(0, HXPCoordSysSetV_, &v);
  getDoubleParam(0, HXPCoordSysSetW_, &w);
  
  if (cs == 1)
  {
    status = HXPHexapodCoordinateSystemSet(pAxis->moveSocket_, GROUP, "Work", x, y, z, u, v, w);
  }
  else if (cs == 2)
  {
    status = HXPHexapodCoordinateSystemSet(pAxis->moveSocket_, GROUP, "Tool", x, y, z, u, v, w);
  }
  else if (cs == 3)
  {
    status = HXPHexapodCoordinateSystemSet(pAxis->moveSocket_, GROUP, "Base", x, y, z, u, v, w);
  }
  
  postError(pAxis, status);

  return status;
}

void HXPController::postError(HXPAxis *pAxis, int status)
{
  /* This is similar to what is done in HXPAxis::move() */
  if (status < 0)
  {
    /* Set the error */
    setIntegerParam(HXPError_, status);
    
    /* Get the error string */
    HXPErrorStringGet(pAxis->moveSocket_, status, pAxis->errorDescFull_);
    
    /* Trim the error string */
    strncpy(pAxis->errorDesc_, pAxis->errorDescFull_, 39);
    pAxis->errorDesc_[39] = 0;
    
    /* Set the error description */
    setStringParam(HXPErrorDesc_, pAxis->errorDesc_);
  }
  else
  {
    /* Clear the error */
    setIntegerParam(HXPError_, 0);
    setStringParam(HXPErrorDesc_, "");
  }
  callParamCallbacks();

  return;
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

  /* Enable gain support so that the CNEN field to enable/disable the hexapod */
  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  // does the hexapod read encoders from the stages? leave in for now to test relative moves
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);

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
  double start_pos;
  double end_pos;
  double diff_pos;
  double cur_pos[NUM_AXES];
  double rel_pos[NUM_AXES] = {};
  double *pos;
  int coordSys; // 0 = work, 1 = tool
  static const char *functionName = "HXPAxis::move";
  asynStatus retval = asynSuccess;

  pC_->getIntegerParam(pC_->HXPMoveCoordSys_, &coordSys); 
  
  if (relative) {
    rel_pos[axisNo_] = position * MRES;
    pos = rel_pos;
    
    if (coordSys == 0)
    {
      status = HXPHexapodMoveIncremental(moveSocket_, GROUP, "Work", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    }
    else
    {
      status = HXPHexapodMoveIncremental(moveSocket_, GROUP, "Tool", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    }
    
    if (status != 0 && status != -27) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: Error performing HexapodMoveIncremental[%s,%d] %d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move! */
        retval = asynError;
    }
  } else {
    // get current positions before moving (could an error overflow the array?)
    status = HXPGroupPositionCurrentGet(pollSocket_, GROUP, 6, (double *) &cur_pos);

    // Capture current position for relative move calc
    start_pos = cur_pos[axisNo_];
    
    end_pos = position * MRES;
    
    if (coordSys == 0)
    {
      // Update position of axis to be moved
      cur_pos[axisNo_] = end_pos;
      pos = cur_pos;

      status = HXPHexapodMoveAbsolute(moveSocket_, GROUP, "Work", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    }
    else
    {
      // Calculate relative move amount (needed for Tool coord-sys moves)
      diff_pos = end_pos - start_pos;
      
      // Update position of axis to be moved
      rel_pos[axisNo_] = diff_pos;
      pos = rel_pos;
    
      status = HXPHexapodMoveIncremental(moveSocket_, GROUP, "Tool", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

    }

    if (status != 0 && status != -27) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: Error performing HexapodMoveAbsolute[%s,%d] %d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move! */
        retval = asynError;
    }
  }
  
  if (status < 0)
  {
    /* Set the error */
    pC_->setIntegerParam(pC_->HXPError_, status);
    
    /* Get the error string */
    HXPErrorStringGet(moveSocket_, status, errorDescFull_);
    
    /* Trim the error string */
    strncpy(errorDesc_, errorDescFull_, 39);
    errorDesc_[39] = 0;
    
    /* Set the error description */
    pC_->setStringParam(pC_->HXPErrorDesc_, errorDesc_);
    
  }
  else
  {
    /* Clear the error */
    pC_->setIntegerParam(pC_->HXPError_, 0);
    pC_->setStringParam(pC_->HXPErrorDesc_, "");
  }
  callParamCallbacks();

  return retval;

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
  /* Note: there is only one status PV for the controller. Currently it reflects the status of axis 0.
           It might be better to call pC_->setIntegerParam(pC_->HXPStatus_, axisStatus_); */
  setIntegerParam(pC_->HXPStatus_, axisStatus_);

  /* If the group is not moving then the axis is not moving */
  if ((axisStatus_ < 43) || (axisStatus_ > 48))
    moving_ = false;
  else
    moving_ = true;

  /* Set the axis done parameter */
  *moving = moving_;
  setIntegerParam(pC_->motorStatusDone_, *moving?0:1);

  /*Test for states that mean we cannot move an axis (disabled, uninitialised, etc.) 
    and set problem bit in MSTA.*/
  if ((axisStatus_ < 10) || ((axisStatus_ >= 20) && (axisStatus_ <= 42)) ||
      (axisStatus_ == 50) || (axisStatus_ == 64)) 
  {
    /* Don't consider a normal disabled status to be a problem */
    if ( axisStatus_==20 ) 
    {
      setIntegerParam(pC_->motorStatusProblem_, 0);             
    }
    else 
    {
      asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
          "%s:%s: [%s,%d]: in unintialised/disabled/not referenced. XPS State Code: %d\n",
          driverName, functionName, pC_->portName, axisNo_, axisStatus_);
      setIntegerParam(pC_->motorStatusProblem_, 1);
    }
    
    /* Group status indicates power is off */
    setIntegerParam(pC_->motorStatusPowerOn_, 0);
  }
  else 
  {
    setIntegerParam(pC_->motorStatusProblem_, 0);
    setIntegerParam(pC_->motorStatusPowerOn_, 1);
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
