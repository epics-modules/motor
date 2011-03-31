/*
FILENAME...     XPSMotorDriver.cpp
USAGE...        Newport XPS EPICS asyn motor device driver

Version:        $Revision: 19717 $
Modified By:    $Author: sluiter $
Last Modified:  $Date: 2009-12-09 10:21:24 -0600 (Wed, 09 Dec 2009) $
HeadURL:        $URL: https://subversion.xor.aps.anl.gov/synApps/trunk/support/motor/vstub/motorApp/NewportSrc/XPSMotorDriver.cpp $
*/

/*
Original Author: Mark Rivers
*/

/*
Copyright (c) 2005 University of Chicago and the Regents of the University of 
California. All rights reserved.

synApps is distributed subject to the following license conditions:
SOFTWARE LICENSE AGREEMENT
Software: synApps 
Versions: Release 4-5 and higher.

   1. The "Software", below, refers to synApps (in either source code, or 
      binary form and accompanying documentation). Each licensee is addressed 
          as "you" or "Licensee."

   2. The copyright holders shown above and their third-party licensors hereby 
      grant Licensee a royalty-free nonexclusive license, subject to the 
          limitations stated herein and U.S. Government license rights.

   3. You may modify and make a copy or copies of the Software for use within 
      your organization, if you meet the following conditions:
         1. Copies in source code must include the copyright notice and this 
                    Software License Agreement.
         2. Copies in binary form must include the copyright notice and this 
                    Software License Agreement in the documentation and/or other 
                        materials provided with the copy.

   4. You may modify a copy or copies of the Software or any portion of it, thus
      forming a work based on the Software, and distribute copies of such work
      outside your organization, if you meet all of the following conditions:
         1. Copies in source code must include the copyright notice and this 
                    Software License Agreement;
         2. Copies in binary form must include the copyright notice and this 
                    Software License Agreement in the documentation and/or other 
                        materials provided with the copy;
         3. Modified copies and works based on the Software must carry 
                    prominent notices stating that you changed specified portions of 
                        the Software.

   5. Portions of the Software resulted from work developed under a 
      U.S. Government contract and are subject to the following license: 
          the Government is granted for itself and others acting on its behalf a 
          paid-up, nonexclusive, irrevocable worldwide license in this computer 
          software to reproduce, prepare derivative works, and perform publicly and 
          display publicly.

   6. WARRANTY DISCLAIMER. THE SOFTWARE IS SUPPLIED "AS IS" WITHOUT WARRANTY OF 
      ANY KIND. THE COPYRIGHT HOLDERS, THEIR THIRD PARTY LICENSORS, THE UNITED 
          STATES, THE UNITED STATES DEPARTMENT OF ENERGY, AND THEIR EMPLOYEES: (1) 
          DISCLAIM ANY WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
          ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
          PURPOSE, TITLE OR NON-INFRINGEMENT, (2) DO NOT ASSUME ANY LEGAL LIABILITY 
          OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR USEFULNESS OF THE 
          SOFTWARE, (3) DO NOT REPRESENT THAT USE OF THE SOFTWARE WOULD NOT 
          INFRINGE PRIVATELY OWNED RIGHTS, (4) DO NOT WARRANT THAT THE SOFTWARE WILL 
          FUNCTION UNINTERRUPTED, THAT IT IS ERROR-FREE OR THAT ANY ERRORS WILL BE 
          CORRECTED.

   7. LIMITATION OF LIABILITY. IN NO EVENT WILL THE COPYRIGHT HOLDERS, THEIR 
      THIRD PARTY LICENSORS, THE UNITED STATES, THE UNITED STATES DEPARTMENT OF 
          ENERGY, OR THEIR EMPLOYEES: BE LIABLE FOR ANY INDIRECT, INCIDENTAL, 
          CONSEQUENTIAL, SPECIAL OR PUNITIVE DAMAGES OF ANY KIND OR NATURE, 
          INCLUDING BUT NOT LIMITED TO LOSS OF PROFITS OR LOSS OF DATA, FOR ANY 
          REASON WHATSOEVER, WHETHER SUCH LIABILITY IS ASSERTED ON THE BASIS OF 
          CONTRACT, TORT (INCLUDING NEGLIGENCE OR STRICT LIABILITY), OR OTHERWISE, 
          EVEN IF ANY OF SAID PARTIES HAS BEEN WARNED OF THE POSSIBILITY OF SUCH 
          LOSS OR DAMAGES.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <iocsh.h>

#include "XPSMotorDriver.h"
#include "XPS_C8_drivers.h"

static const char *driverName = "XPSMotorDriver";

typedef enum { none, positionMove, velocityMove, homeReverseMove, homeForwardsMove } moveType;

/** Struct for a list of strings describing the different corrector types possible on the XPS.*/
typedef struct {
  char *PIPosition;
  char *PIDFFVelocity;
  char *PIDFFAcceleration;
  char *PIDDualFFVoltage;
  char *NoCorrector;
} CorrectorTypes_t;

const static CorrectorTypes_t CorrectorTypes = { 
  "PositionerCorrectorPIPosition",
  "PositionerCorrectorPIDFFVelocity",
  "PositionerCorrectorPIDFFAcceleration",
  "PositionerCorrectorPIDDualFFVoltage",
  "NoCorrector"
};

/** This is controlled via the XPSEnableSetPosition function (available via the IOC shell). */ 
static int doSetPosition = 1;

/**
 * Parameter to control the sleep time used when setting position. 
 * A function called XPSSetPosSleepTime(int) (millisec parameter) 
 * is available in the IOC shell to control this.
 */
static double setPosSleepTime = 0.5;

/** Deadband to use for the velocity comparison with zero. */
#define XPS_VELOCITY_DEADBAND 0.0000001

#define XPS_MAX_AXES 8
#define XPSC8_END_OF_RUN_MINUS  0x80000100
#define XPSC8_END_OF_RUN_PLUS   0x80000200

#define TCP_TIMEOUT 2.0
#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

XPSController::XPSController(const char *portName, const char *IPAddress, int IPPort, 
                                       int numAxes, double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_XPS_PARAMS, 
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask, 
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  static const char *functionName = "XPSController";
  
  IPAddress_ = epicsStrDup(IPAddress);
  IPPort_ = IPPort;

  // Create controller-specific parameters
  createParam(XPSMinJerkString, asynParamFloat64, &XPSMinJerk_);
  createParam(XPSMaxJerkString, asynParamFloat64, &XPSMaxJerk_);
  createParam(XPSStatusString,  asynParamInt32,   &XPSStatus_);

  pollSocket_ = TCP_ConnectToServer((char *)IPAddress, IPPort, TCP_TIMEOUT);
  if (pollSocket_ < 0) {
    printf("%s:%s: error calling TCP_ConnectToServer for pollSocket\n",
           driverName, functionName);
  }

  FirmwareVersionGet(pollSocket_, firmwareVersion_);
  
  /* Create the poller thread for this controller
   * NOTE: at this point the axis objects don't yet exist, but the poller tolerates this */
  startPoller(movingPollPeriod, idlePollPeriod, 10);
}

void XPSController::report(FILE *fp, int level)
{
  int axis;
  XPSAxis *pAxis;

  fprintf(fp, "XPS motor driver %s, numAxes=%d, firmware version=%s, moving poll period=%f, idle poll period=%f\n", 
          this->portName, numAxes_, firmwareVersion_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      fprintf(fp, "  axis %d\n"
                  "    name = %s\n"
                  "    step size = %g\n"
                  "    poll socket = %d, moveSocket = %d\n"
                  "    status = %d\n", 
              pAxis->axisNo_,
              pAxis->positionerName_, 
              pAxis->stepSize_, 
              pAxis->pollSocket_, pAxis->moveSocket_, 
              pAxis->axisStatus_);
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

/**
 * Perform a deferred move (a coordinated group move) on all the axes in a group.
 * @param pController Pointer to XPSController structure.
 * @param groupName Pointer to string naming the group on which to perform the group move.
 * @return motor driver status code.
 */
asynStatus XPSController::processDeferredMovesInGroup(char *groupName)
{
  double positions[XPS_MAX_AXES];
  int positions_index = 0;
  int first_loop = 1;
  int axis = 0;
  int NbPositioners = 0;
  int relativeMove = 0;
  int status = 0;
  XPSAxis *pAxis=NULL;

  /* Loop over all axes in this controller. */
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "Executing deferred move on XPS: %s, Group: %s\n", 
              this->portName, groupName);

    /* Ignore axes in other groups. */
    if (!strcmp(pAxis->groupName_, groupName)) {
      if (first_loop) {
        /* Get the number of axes in this group. */
        NbPositioners = pAxis->isInGroup();
        first_loop = 0;
      }

      /* Set relative flag for the actual move at the end of the funtion.*/
      if (pAxis->deferredRelative_) {
        relativeMove = 1;
      }

      /* Build position buffer.*/
      if (pAxis->deferredMove_) {
        positions[positions_index] = 
          pAxis->deferredRelative_ ? (pAxis->setpointPosition_ + pAxis->deferredPosition_) : pAxis->deferredPosition_;
      } else {
        positions[positions_index] = 
          pAxis->deferredRelative_ ? 0 : pAxis->setpointPosition_;
      }

      /* Reset deferred flag. */
      /* We need to do this for the XPS, because we cannot do partial group moves. Every axis
         in the group will be included the next time we do a group move. */
      pAxis->deferredMove_ = 0;

      /* Next axis in this group. */
      positions_index++;
    }
  }
  
  /* Send the group move command. */
  if (relativeMove) {
    status = GroupMoveRelative(pAxis->moveSocket_,
                               groupName,
                               NbPositioners,
                               positions);
  } else {
    status = GroupMoveAbsolute(pAxis->moveSocket_,
                               groupName,
                               NbPositioners,
                               positions);
  }
  
  if (status!=0) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "Error peforming GroupMoveAbsolute/Relative in processDeferredMovesInGroup. XPS Return code: %d\n", 
              status);
    return asynError;
  }  

  return asynSuccess;
  
}

/**
 * Process deferred moves for a controller and groups.
 * This function calculates which unique groups in the controller
 * and passes the controller pointer and group name to processDeferredMovesInGroup.
 * @return motor driver status code.
 */
asynStatus XPSController::processDeferredMoves()
{
  asynStatus status = asynError;
  int axis = 0;
  int i = 0;
  int dealWith = 0;
  /* Array to cache up to XPS_MAX_AXES group names. Don't initialise to null */
  char *groupNames[XPS_MAX_AXES];
  char *blankGroupName = " ";
  XPSAxis *pAxis;

  /* Clear group name cache. */
  for (i=0; i<XPS_MAX_AXES; i++) {
    groupNames[i] = blankGroupName;
  }

  /* Loop over axes, testing for unique groups. */
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
  
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "Processing deferred moves on XPS: %s\n", this->portName);
  
    /* Call processDeferredMovesInGroup only once for each group on this controller.
       Positioners in the same group may not be adjacent in list, so we have to test for this. */
    for (i=0; i<XPS_MAX_AXES; i++) {
      if (strcmp(pAxis->groupName_, groupNames[i])) {
        dealWith++;
        groupNames[i] = pAxis->groupName_;
      }
    }
    if (dealWith == XPS_MAX_AXES) {
      dealWith = 0;
      /* Group name was not in cache, so deal with this group. */
      status = this->processDeferredMovesInGroup(pAxis->groupName_);
    }
    /*  Next axis, and potentially next group.*/
  }
  
  return status;
}

asynStatus XPSController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  XPSAxis *pAxis = this->getAxis(pasynUser);
  double deviceValue;
  static const char *functionName = "writeFloat64";
  
  /* Set the parameter and readback in the parameter library. */
  status = pAxis->setDoubleParam(function, value);
  
  if (function == motorResolution_)
  {
    pAxis->stepSize_ = value;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s:%s: Set XPS %s, axis %d stepSize to %f\n", 
               driverName, functionName, portName, pAxis->axisNo_, value);
  }
  else if (function == motorLowLimit_)
  {
    deviceValue = value*pAxis->stepSize_;
    /* We need to read the current highLimit because otherwise we could be setting it to an invalid value */
    status = PositionerUserTravelLimitsGet(pAxis->pollSocket_,
                                           pAxis->positionerName_,
                                           &pAxis->lowLimit_, &pAxis->highLimit_);
    if (status != 0) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s:%s: motorAxisSetDouble[%s,%d]: error performing PositionerUserTravelLimitsGet for lowLim status=%d\n",
                driverName, functionName, portName, pAxis->axisNo_, status);
      goto done;
    }
    status = PositionerUserTravelLimitsSet(pAxis->pollSocket_,
                                           pAxis->positionerName_,
                                           deviceValue, pAxis->highLimit_);
    if (status != 0) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: motorAxisSetDouble[%s,%d]: error performing PositionerUserTravelLimitsSet for lowLim=%f status=%d\n",
                driverName, functionName, portName, pAxis->axisNo_, deviceValue, status);
      goto done;
    } 
    pAxis->lowLimit_ = deviceValue;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s:%s: Set XPS %s, axis %d low limit to %f\n", 
              driverName, functionName, portName, pAxis->axisNo_, deviceValue);
    status = asynSuccess;
  } else if (function == motorHighLimit_)
  {
    deviceValue = value*pAxis->stepSize_;
    /* We need to read the current highLimit because otherwise we could be setting it to an invalid value */
    status = PositionerUserTravelLimitsGet(pAxis->pollSocket_,
                                           pAxis->positionerName_,
                                           &pAxis->lowLimit_, &pAxis->highLimit_);
    if (status != 0) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s:%s: motorAxisSetDouble[%s,%d]: error performing PositionerUserTravelLimitsGet for highLim status=%d\n",
                driverName, functionName, portName, pAxis->axisNo_, status);
      goto done;
    }
    status = PositionerUserTravelLimitsSet(pAxis->pollSocket_,
                                           pAxis->positionerName_,
                                           pAxis->lowLimit_, deviceValue);
    if (status != 0) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s:%s: motorAxisSetDouble[%s,%d]: error performing PositionerUserTravelLimitsSet for highLim=%f status=%d\n",
                driverName, functionName, portName, pAxis->axisNo_, deviceValue, status);
      goto done;
    } 
    pAxis->highLimit_ = deviceValue;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s:%s: Set XPS %s, axis %d high limit to %f\n", 
              driverName, functionName, portName, pAxis->axisNo_, deviceValue);
    status = asynSuccess;
  } else if (function == motorPgain_)
  {
    status = pAxis->setPID(&value, 0);
  } else if (function == motorIgain_)
  {
    status = pAxis->setPID(&value, 1);
  } else if (function == motorDgain_)
  {
    status = pAxis->setPID(&value, 2);
  } else {
    /* Call base class method */
    status = asynMotorController::writeFloat64(pasynUser, value);
  }
  done:
  return (asynStatus)status;
}


asynStatus XPSController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  XPSAxis *pAxis = this->getAxis(pasynUser);
  static const char *functionName = "writeInt32";

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = pAxis->setIntegerParam(function, value);

  if (function == motorSetClosedLoop_)
  {
    if (value) {
      status = GroupMotionEnable(pAxis->pollSocket_, pAxis->groupName_);
      if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: [%s,%d]: error calling GroupMotionEnable status=%d\n",
                   driverName, functionName, portName, pAxis->axisNo_, status);
      } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: motorAxisSetInteger set XPS %s, axis %d closed loop enable\n",
                   driverName, functionName, portName, pAxis->axisNo_);
      }
    } else {
      status = GroupMotionDisable(pAxis->pollSocket_, pAxis->groupName_);
      if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: [%s,%d]: error calling GroupMotionDisable status=%d\n",
                  driverName, functionName, portName, pAxis->axisNo_, status);
      } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: motorAxisSetInteger set XPS %s, axis %d closed loop disable\n",
                  driverName, functionName, portName, pAxis->axisNo_);
      }
    }
  } else if (function == motorDeferMoves_)
  {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s%s: Setting deferred move mode on XPS %s to %d\n", 
               driverName, functionName, portName, value);
    if (value == 0.0 && this->movesDeferred_ != 0) {
      status = this->processDeferredMoves();
    }
    this->movesDeferred_ = value;
    if (status) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s%s: Deferred moved failed on XPS %s, status=%d\n", 
                driverName, functionName, portName, status);
      status = asynError;
    }
  } else {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }
  return (asynStatus)status;
}

XPSAxis* XPSController::getAxis(asynUser *pasynUser)
{
  return static_cast<XPSAxis*>(asynMotorController::getAxis(pasynUser));
}

XPSAxis* XPSController::getAxis(int axisNo)
{
  return static_cast<XPSAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the XPSAxis:: methods

XPSAxis::XPSAxis(XPSController *pC, int axisNo, const char *positionerName, double stepSize)
  :   asynMotorAxis(pC, axisNo),
      pC_(pC)
{
  static const char *functionName = "XPSAxis::XPSAxis";
  char *index;
  int status;
  double minJerkTime, maxJerkTime;

  moveSocket_ = TCP_ConnectToServer(pC_->IPAddress_, pC->IPPort_, TCP_TIMEOUT);
  if (moveSocket_ < 0) {
    printf("%s:%s: error calling TCP_ConnectToServer for move socket\n",
           driverName, functionName);
  }
  /* Set the poll rate on the moveSocket to a negative number, which means that SendAndReceive should do only a write, no read */
  TCP_SetTimeout(moveSocket_, -0.1);
  pollSocket_ = pC_->pollSocket_;

  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  setDoubleParam(pC_->motorPgain_, xpsCorrectorInfo_.KP);
  setDoubleParam(pC_->motorIgain_, xpsCorrectorInfo_.KI);
  setDoubleParam(pC_->motorDgain_, xpsCorrectorInfo_.KD);
  callParamCallbacks();
  /* Initialise deferred move flags. */
  deferredRelative_ = 0;
  deferredPosition_ = 0;
  /* Disable deferred move for the axis. Should not cause move of this axis
     if other axes in same group do deferred move. */
  deferredMove_ = 0; 

  index = (char *)strchr(positionerName, '.');
  if (index == NULL) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: positionerName must be of form group.positioner = %s\n",
                driverName, functionName, positionerName);
  }
  positionerName_ = epicsStrDup(positionerName);
  groupName_ = epicsStrDup(positionerName);
  index = strchr(groupName_, '.');
  if (index != NULL) *index = '\0';  /* Terminate group name at place of '.' */

  stepSize_ = stepSize;
  /* Read some information from the controller for this axis */
  status = PositionerSGammaParametersGet(pollSocket_,
                                         positionerName_,
                                         &velocity_,
                                         &accel_,
                                         &minJerkTime,
                                         &maxJerkTime);
   setDoubleParam(pC_->XPSMinJerk_, minJerkTime);
   setDoubleParam(pC_->XPSMaxJerk_, maxJerkTime);

  /* NOTE: this will require PID to be allowed to be set greater than 1 in motor record. */
  /* And we need to implement this in Asyn layer. */
  getPID();

  /* Wake up the poller task which will make it do a poll, 
   * updating values for this axis to use the new resolution (stepSize_) */   
  pC_->wakeupPoller();

}

asynStatus XPSAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  char errorString[100];
  double deviceUnits;
  int status;
  double minJerk, maxJerk;
  static const char *functionName = "XPSAxis::move";

  pC_->getDoubleParam(axisNo_, pC_->XPSMinJerk_, &minJerk); 
  pC_->getDoubleParam(axisNo_, pC_->XPSMaxJerk_, &maxJerk); 
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: Set XPS %s, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f, minJerk=%f, maxJerk=%f\n",
            driverName, functionName, pC_->portName, axisNo_, position, min_velocity, max_velocity, acceleration, minJerk, maxJerk);

  /* Look at the last poll value of the positioner status.  If it is disabled, then enable it */
  if (axisStatus_ >= 20 && axisStatus_ <= 36) {
    status = GroupMotionEnable(pollSocket_, groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: motorAxisMove[%s,%d]: error performing GroupMotionEnable %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
      return asynError;
    }
  }
  status = PositionerSGammaParametersSet(pollSocket_,
                                         positionerName_, 
                                         max_velocity*stepSize_,
                                         acceleration*stepSize_,
                                         minJerk,
                                         maxJerk);
  if (status != 0) {
    ErrorStringGet(pollSocket_, status, errorString);
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: Error performing PositionerSGammaParametersSet[%s,%d] %d: %s\n",
              driverName, functionName, pC_->portName, axisNo_, status, errorString);
    return asynError;
  }

  deviceUnits = position * stepSize_;
  if (relative) {
    if (pC_->movesDeferred_ == 0) {
      status = GroupMoveRelative(moveSocket_,
                                 positionerName_,
                                 1,
                                 &deviceUnits); 
      if (status != 0 && status != -27) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: Error performing GroupMoveRelative[%s,%d] %d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
        return asynError;
      }
    } else {
      deferredPosition_ = deviceUnits;
      deferredMove_ = 1;
      deferredRelative_ = relative;
    }
  } else {
    if (pC_->movesDeferred_ == 0) {
      status = GroupMoveAbsolute(moveSocket_,
                                 positionerName_,
                                 1,
                                 &deviceUnits); 
      if (status != 0 && status != -27) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: Error performing GroupMoveAbsolute[%s,%d] %d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
        return asynError;
      }
    } else {
      deferredPosition_ = deviceUnits;
      deferredMove_ = 1;
      deferredRelative_ = relative;
    }
  }

  return asynSuccess;
}

asynStatus XPSAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards)
{
  int status;
  int groupStatus;
  char errorBuffer[100];
  static const char *functionName = "XPSAxis::home";

  status = GroupStatusGet(pollSocket_, groupName_, &groupStatus);
  /* The XPS won't allow a home command if the group is in the Ready state
   * If the group is Ready, then make it not Ready  */
  if (groupStatus >= 10 && groupStatus <= 18) {
    status = GroupKill(pollSocket_, groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: [%s,%d]: error calling GroupKill error=%s\n",
                driverName, functionName, pC_->portName, axisNo_, getXPSError(status, errorBuffer));
      return asynError;
    }
  }
  status = GroupStatusGet(pollSocket_, groupName_, &groupStatus);
  /* If axis not initialized, then initialize it */
  if (groupStatus >= 0 && groupStatus <= 9) {
    status = GroupInitialize(pollSocket_, groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: [%s,%d]: error calling GroupInitialize error=%s\n",
                driverName, functionName, pC_->portName, axisNo_, getXPSError(status, errorBuffer));
      return asynError;
    }
  }
  status = GroupHomeSearch(moveSocket_, groupName_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupHomeSearch error=%s\n",
              driverName, functionName, pC_->portName, axisNo_, getXPSError(status, errorBuffer));
    return asynError;
  }

  return asynSuccess;
}


asynStatus XPSAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration)
{
  int status;
  double deviceVelocity, deviceAcceleration;
  static const char *functionName = "XPSAxis::moveVelocity";

  status = GroupJogModeEnable(pollSocket_, groupName_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupJogModeEnable error=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }
  deviceVelocity = max_velocity * stepSize_;
  deviceAcceleration = acceleration * stepSize_;
  status = GroupJogParametersSet(moveSocket_, positionerName_, 1, &deviceVelocity, &deviceAcceleration);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupJogParametersSet error=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }
  return asynSuccess;
}

asynStatus XPSAxis::setPosition(double position)
{
  XPSAxis *pAxis;
  int status=0;
  int axisIndex;
  int axisIndexInGrp;
  int axesInGroup;
  double positions[XPS_MAX_AXES];
  static const char *functionName = "XPSAxis::setPosition";


  /* If the user has disabled setting the controller position, skip this.*/
  if (!doSetPosition) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: XPS set position is disabled. Enable it using XPSEnableSetPosition(1).\n",
               driverName, functionName);
    return asynError;
  }
  /* Test if this axis is in a XPS group.*/
  axesInGroup = isInGroup();
  if (axesInGroup>1) {
    /* We are in a group, so we need to read the positions of all the axes in the group,
       kill the group, and set all the positions in the group using referencing mode.
       We read the positions seperately, rather than in one command, because we can't assume
       that the ordering is the same in the XPS as in the driver.*/
    for (axisIndex=0; axisIndex<pC_->numAxes_; axisIndex++) {
      pAxis = pC_->getAxis(axisIndex);
      status = GroupPositionCurrentGet(pollSocket_, 
                                       pAxis->positionerName_, 
                                       1, 
                                       &positions[axisIndex]);
    }
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: Error performing GroupPositionCurrentGet(%s,%d). Aborting set position. XPS API Error: %d.\n", 
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    } 
    status = GroupKill(pollSocket_, groupName_);
    status = GroupInitialize(pollSocket_, groupName_);
    if (status != 0) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: Error performing GroupKill/GroupInitialize(%s,%d). Aborting set position. XPS API Error: %d.\n", 
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    /* Wait after axis initialisation (we don't want to set position immediately after
     * initialisation because the stage can oscillate slightly). */
    epicsThreadSleep(setPosSleepTime);

    status = GroupReferencingStart(pollSocket_, groupName_);
    axisIndexInGrp = 0;
    /* Set positions for all axes in the group using the cached values. */
    for (axisIndex=0; axisIndex<pC_->numAxes_; axisIndex++) {
      pAxis = pC_->getAxis(axisIndex);
      if (!strcmp(groupName_, pAxis->groupName_)) {
        /* But skip the current axis, because we do this just after the loop.*/
        if (strcmp(positionerName_, pAxis->positionerName_)) {
          status = GroupReferencingActionExecute(pollSocket_, 
                                                 pAxis->positionerName_, 
                                                 "SetPosition", 
                                                 "None", 
                                                 positions[axisIndexInGrp]);
        }
        ++axisIndexInGrp;
      }
    }
    /* Now reset the position of the axis we are interested in, using the argument passed into this function.*/
    status = GroupReferencingActionExecute(pollSocket_, 
                                           positionerName_, 
                                           "SetPosition", 
                                           "None", 
                                           position*(stepSize_));
    /* Stop referencing, then we are homed on all axes in group.*/
    status = GroupReferencingStop(pollSocket_, 
                                  groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%:s%s: Error performing referencing set position (%s,%d). XPS API Error: %d.", 
                driverName, functionName, pC_->portName, axisNo_, status);
    }
  } else {
    /* We are not in a group, so we just need to use the XPS
       referencing mode to set the position.*/
    status = GroupKill(pollSocket_, groupName_);
    status = GroupInitialize(pollSocket_, groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: Error performing GroupKill/GroupInitialize(%s,%d). XPS API Error: %d. Aborting set position.\n", 
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    /* Wait after axis initialisation (we don't want to set position immediately after
       initialisation because the stage can oscillate slightly).*/
    epicsThreadSleep(setPosSleepTime);

    status = GroupReferencingStart(pollSocket_, 
                                  groupName_);
    status = GroupReferencingActionExecute(pollSocket_, 
                                           positionerName_, 
                                           "SetPosition", 
                                           "None", 
                                           position*(stepSize_));
    status = GroupReferencingStop(pollSocket_, 
                                  groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error performing referencing set position (%s,%d). XPS API Error: %d.", 
                driverName, functionName, pC_->portName, axisNo_, status);
    }
  }
  return asynSuccess;
} 

asynStatus XPSAxis::stop(double acceleration)
{
  int status;
  double deviceVelocity=0.;
  double deviceAcceleration;
  static const char *functionName = "stopAxis";

  /* We need to read the status, because a jog is stopped differently from a move */ 
  status = GroupStatusGet(pollSocket_, groupName_, &axisStatus_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupStatusGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }
  if (axisStatus_ == 47) {
    deviceAcceleration = acceleration * stepSize_;
    status = GroupJogParametersSet(moveSocket_, positionerName_, 1, &deviceVelocity, &deviceAcceleration);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: [%s,%d]: error calling GroupJogParametersSet status=%d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }
  }
  
  if (axisStatus_ == 44) {
    status = GroupMoveAbort(moveSocket_, groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: [%s,%d]: error calling GroupMoveAbort status=%d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }
  }

  /* Clear defer move flag for this axis. */
  deferredMove_ = 0;

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: XPS %s, axis %d stop with accel=%f\n",
            driverName, functionName, axisNo_, acceleration);

  return asynSuccess;
}

asynStatus XPSAxis::poll(int *moving)
{
  int status;
  int axisDone;
  double actualVelocity, theoryVelocity, acceleration;
  static const char *functionName = "XPSAxis::poll";

  status = GroupStatusGet(pollSocket_, 
                          groupName_, 
                          &axisStatus_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupStatusGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: [%s,%d]: %s axisStatus=%d\n",
            driverName, functionName, pC_->portName, axisNo_, positionerName_, axisStatus_);
  /* Set done flag by default */
  axisDone = 1;
  if (axisStatus_ >= 10 && axisStatus_ <= 18) {
    /* These states mean ready from move/home/jog etc */
  }
  if (axisStatus_ >= 43 && axisStatus_ <= 48) {
    /* These states mean it is moving/homeing/jogging etc*/
    axisDone = 0;
    if (axisStatus_ == 47) {
      /* We are jogging.  When the velocity gets back to 0 disable jogging */
      status = GroupJogParametersGet(pollSocket_, positionerName_, 1, &theoryVelocity, &acceleration);
      status = GroupJogCurrentGet(pollSocket_, positionerName_, 1, &actualVelocity, &acceleration);
      if (status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                  "%s:%s: [%s,%d]: error calling GroupJogCurrentGet status=%d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        goto done;
      }
      if (actualVelocity == 0. && theoryVelocity == 0.) {
        status = GroupJogModeDisable(pollSocket_, groupName_);
        if (status) {
          asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                    "%s:%s: [%s,%d]: error calling GroupJogModeDisable status=%d\n",
                    driverName, functionName, pC_->portName, axisNo_, status);
          /* In this mode must do a group kill? */
          status = GroupKill(pollSocket_, groupName_);
          asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                    "%s:%s: [%s,%d]: called GroupKill!\n",
                    driverName, functionName, pC_->portName, axisNo_);
          goto done;
        }
      } 
    }
  }
  /* Set the status */
  setIntegerParam(pC_->XPSStatus_, axisStatus_);
  /* Set the axis done parameter */
  /* AND the done flag with the inverse of deferred_move.*/
  axisDone &= !deferredMove_;
  *moving = axisDone ? 0 : 1;
  setIntegerParam(pC_->motorStatusDone_, axisDone);
  setIntegerParam(pC_->motorStatusHome_, (axisStatus_ == 11) ? 1 : 0);
  if ((axisStatus_ >= 0 && axisStatus_ <= 9) || 
    (axisStatus_ >= 20 && axisStatus_ <= 42)) {
    /* Not initialized, homed or disabled */
    asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
              "%s:%s: [%s,%d]: in bad state %d\n",
              driverName, functionName, pC_->portName, axisNo_, axisStatus_);
    /* setIntegerParam(axis, motorStatusHighHardLimit, 1);
     * setIntegerParam(axis, motorStatusLowHardLimit,  1);
     */
  }

  /* Test for following error, and set appropriate param. */
  if ((axisStatus_ == 21 || axisStatus_ == 22) ||
      (axisStatus_ >= 24 && axisStatus_ <= 26) ||
      (axisStatus_ == 28 || axisStatus_ == 35)) {
      asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
                "%s:%s: [%s,%d]: in following error. XPS State Code: %d\n",
                driverName, functionName, pC_->portName, axisNo_, axisStatus_);
    setIntegerParam(pC_->motorStatusFollowingError_, 1);
  } else {
    setIntegerParam(pC_->motorStatusFollowingError_, 0);
  }

  status = GroupPositionCurrentGet(pollSocket_,
                                   positionerName_,
                                   1,
                                   &encoderPosition_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupPositionCurrentGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  setDoubleParam(pC_->motorEncoderPosition_, (encoderPosition_/stepSize_));

  status = GroupPositionSetpointGet(pollSocket_,
                                   positionerName_,
                                   1,
                                   &setpointPosition_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupPositionSetpointGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  setDoubleParam(pC_->motorPosition_, (setpointPosition_/stepSize_));

  status = PositionerErrorGet(pollSocket_,
                              positionerName_,
                              &positionerError_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling PositionerErrorGet status=%d\n",
               driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  /* These are hard limits */
  if (positionerError_ & XPSC8_END_OF_RUN_PLUS) {
    setIntegerParam(pC_->motorStatusHighLimit_, 1);
  } else {
    setIntegerParam(pC_->motorStatusHighLimit_, 0);
  }
  if (positionerError_ & XPSC8_END_OF_RUN_MINUS) {
    setIntegerParam(pC_->motorStatusLowLimit_, 1);
  } else {
    setIntegerParam(pC_->motorStatusLowLimit_, 0);
  }

  /* Read the current velocity and use it set motor direction and moving flag. */
  status = GroupVelocityCurrentGet(pollSocket_,
                                   positionerName_,
                                   1,
                                   &currentVelocity_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupPositionVelocityGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_,  status);
    goto done;
  }
  setIntegerParam(pC_->motorStatusDirection_, (currentVelocity_ > XPS_VELOCITY_DEADBAND));
  setIntegerParam(pC_->motorStatusMoving_,    (fabs(currentVelocity_) > XPS_VELOCITY_DEADBAND));

  done:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status ? asynError : asynSuccess;
}

char *XPSAxis::getXPSError(int status, char *buffer)
{
    status = ErrorStringGet(this->pollSocket_, status, buffer);
    return buffer;
}

/**
 * Test if axis is configured as an XPS single axis or a group.
 * This is done by comparing cached group names.
 * @param pAxis Axis struct AXIS_HDL
 * @return 1 if in group single group, or return the number of axes in the group.
 */
int XPSAxis::isInGroup()
{
  XPSAxis *pAxis;
  int i;
  int group=0;

  for(i=0; i<pC_->numAxes_; i++) {
    pAxis = pC_->getAxis(i);
    if (!strcmp(groupName_, pAxis->groupName_)) {
      ++group;
    }
  } 
  return group;
}

/**
 * Function to set the XPS controller PID parameters.
 * @param pAxis Axis struct AXIS_HDL
 * @param value The desired value of the parameter.
 * @param pidoption Set to 0 for P, 1 for I and 2 for D.
 *
 * @return Zero if success, non-zero if error (and equal to XPS API error if error is from XPS).
 */
asynStatus XPSAxis::setPID(const double * value, int pidoption)
{
  int status = 0;
  char correctorType[250] = {'\0'};
  static const char *functionName = "XPSAxis::setPID";

  /*The XPS function that we use to set the PID parameters is dependant on the 
    type of corrector in use for that axis.*/
  status = PositionerCorrectorTypeGet(pollSocket_,
                                      positionerName_,
                                      correctorType);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: Error with PositionerCorrectorTypeGet. XPS: %s, Axis: %d, XPS API Error: %d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }

  if (!strcmp(correctorType, CorrectorTypes.PIPosition)) {
    /*Read the PID parameters first.*/
    status = PositionerCorrectorPIPositionGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIPositionGet. Aborting setting PID XPS: %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    /*Set the P, I or D parameter in the xpsCorrectorInfo struct.*/
    setPIDValue(value, pidoption); 

    /*Now set the parameters in the XPS.*/
    status = PositionerCorrectorPIPositionSetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIPositionSet: %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }
  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFVelocity)) {
    status = PositionerCorrectorPIDFFVelocityGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFVelocityGet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    setPIDValue(value, pidoption); 

    status = PositionerCorrectorPIDFFVelocitySetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFVelocitySet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFAcceleration)) {
    status = PositionerCorrectorPIDFFAccelerationGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFAccelerationGet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    setPIDValue(value, pidoption); 

    status = PositionerCorrectorPIDFFAccelerationSetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFAccelerationSet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDDualFFVoltage)) {
    status = PositionerCorrectorPIDDualFFVoltageGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDDualFFVoltageGet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    setPIDValue(value, pidoption); 

    status = PositionerCorrectorPIDDualFFVoltageSetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDDualFFVoltageSet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.NoCorrector)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s:. XPS %s axis %d corrector type is %s. Cannot set PID.\n", 
              driverName, functionName, pC_->portName, axisNo_, correctorType);
    return asynError; 
  } else {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s:. XPS %s axis %d not a valid corrector type: %s. Cannot set PID.\n", 
              driverName, functionName, pC_->portName, axisNo_, correctorType); 
    return asynError; 
  }
  return asynSuccess;
}

/**
 * Function to read the PID values from the XPS (and any other XPS corrector info that is valid for the axis). 
 * The read values are set in the AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Zero if success, non-zero if error (and equal to XPS API error if error is from XPS).
 */
asynStatus XPSAxis::getPID() 
{
  int status = 0;
  char correctorType[250] = {'\0'};
  static const char *functionName = "XPSAxis::getPID";
  
  /*The XPS function that we use to set the PID parameters is dependant on the 
    type of corrector in use for that axis.*/
  status = PositionerCorrectorTypeGet(pollSocket_,
                                      positionerName_,
                                      correctorType);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: Error with PositionerCorrectorTypeGet. XPS: %s, Axis: %d, XPS API Error: %d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }
  if (!strcmp(correctorType, CorrectorTypes.PIPosition)) {
    /*Read the PID parameters and set in pAxis.*/
    status = PositionerCorrectorPIPositionGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: XPS %s, Axis: %d, Error with PositionerCorrectorPIPositionGet. XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFVelocity)) {
    status = PositionerCorrectorPIDFFVelocityGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: XPS %s, Axis: %d, Error with PositionerCorrectorPIDFFVelocityGet. XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFAcceleration)) {
    status = PositionerCorrectorPIDFFAccelerationGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: XPS %s, Axis: %d, Error with PositionerCorrectorPIDFFAccelerationGet. XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDDualFFVoltage)) {
    status = PositionerCorrectorPIDDualFFVoltageGetWrapper();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: XPS %s, Axis: %d, Error with PositionerCorrectorPIDDualFFVoltageGet. XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.NoCorrector)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s:. XPS %s axis %d corrector type is %s. Cannot get PID.\n", 
              driverName, functionName, pC_->portName, axisNo_, correctorType);
    return asynError; 
  } else {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s:. XPS %s axis %d not a valid corrector type: %s. Cannot set PID.\n", 
              driverName, functionName, pC_->portName, axisNo_, correctorType); 
    return asynError; 
  }
  return asynSuccess;
}

/**
 * Set the P, I or D parameter in a xpsCorrectorInfo_t struct.
 * @param xpsCorrectorInfo Pointer to a xpsCorrectorInfo_t struct.
 * @param value The value to set.
 * @param pidoption Set to 0 for P, 1 for I and 2 for D.
 */
asynStatus XPSAxis::setPIDValue(const double * value, int pidoption) 
{
  static const char *functionName = "XPSAxis::setPIDValue";

  if ((pidoption < 0) || (pidoption > 2)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: XPS %s axis %d pidoption out of range=%d\n",
              driverName, functionName, pC_->portName, axisNo_, pidoption);
    return asynError;
  } else {
    switch (pidoption) {
    case 0:
      xpsCorrectorInfo_.KP = *value;
      break;
    case 1:
      xpsCorrectorInfo_.KI = *value;
      break;
    case 2:
      xpsCorrectorInfo_.KD = *value;
      break;
    default:
      /*Do nothing.*/
      break;
    }
  }
  return asynSuccess;
}



/**
 * Wrapper function for PositionerCorrectorPIPositionGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIPositionGetWrapper()
{
  int status;
  status = PositionerCorrectorPIPositionGet(pollSocket_,
                                            positionerName_,
                                            &xpsCorrectorInfo_.ClosedLoopStatus,
                                            &xpsCorrectorInfo_.KP, 
                                            &xpsCorrectorInfo_.KI, 
                                            &xpsCorrectorInfo_.IntegrationTime);
  return status ? asynError : asynSuccess;
}

/**
 * Wrapper function for PositionerCorrectorPIDFFVelocityGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDFFVelocityGetWrapper()
{
  int status;
  status = PositionerCorrectorPIDFFVelocityGet(pollSocket_,
                                               positionerName_,
                                               &xpsCorrectorInfo_.ClosedLoopStatus,
                                               &xpsCorrectorInfo_.KP, 
                                               &xpsCorrectorInfo_.KI,
                                               &xpsCorrectorInfo_.KD,
                                               &xpsCorrectorInfo_.KS,
                                               &xpsCorrectorInfo_.IntegrationTime,
                                               &xpsCorrectorInfo_.DerivativeFilterCutOffFrequency,
                                               &xpsCorrectorInfo_.GKP,
                                               &xpsCorrectorInfo_.GKI,
                                               &xpsCorrectorInfo_.GKD,
                                               &xpsCorrectorInfo_.KForm,
                                               &xpsCorrectorInfo_.FeedForwardGainVelocity);
  return status ? asynError : asynSuccess;
}


/**
 * Wrapper function for PositionerCorrectorPIDFFAccelerationGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDFFAccelerationGetWrapper()
{
  int status;
  status = PositionerCorrectorPIDFFAccelerationGet(pollSocket_,
                                                   positionerName_,
                                                   &xpsCorrectorInfo_.ClosedLoopStatus,
                                                   &xpsCorrectorInfo_.KP, 
                                                   &xpsCorrectorInfo_.KI,
                                                   &xpsCorrectorInfo_.KD,
                                                   &xpsCorrectorInfo_.KS,
                                                   &xpsCorrectorInfo_.IntegrationTime,
                                                   &xpsCorrectorInfo_.DerivativeFilterCutOffFrequency,
                                                   &xpsCorrectorInfo_.GKP,
                                                   &xpsCorrectorInfo_.GKI,
                                                   &xpsCorrectorInfo_.GKD,
                                                   &xpsCorrectorInfo_.KForm,
                                                   &xpsCorrectorInfo_.FeedForwardGainAcceleration);
  return status ? asynError : asynSuccess;
}


/**
 * Wrapper function for PositionerCorrectorPIDDualFFVoltageGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDDualFFVoltageGetWrapper()
{
  int status;
  status =  PositionerCorrectorPIDDualFFVoltageGet(pollSocket_,
                                                  positionerName_,
                                                  &xpsCorrectorInfo_.ClosedLoopStatus,
                                                  &xpsCorrectorInfo_.KP, 
                                                  &xpsCorrectorInfo_.KI,
                                                  &xpsCorrectorInfo_.KD,
                                                  &xpsCorrectorInfo_.KS,
                                                  &xpsCorrectorInfo_.IntegrationTime,
                                                  &xpsCorrectorInfo_.DerivativeFilterCutOffFrequency,
                                                  &xpsCorrectorInfo_.GKP,
                                                  &xpsCorrectorInfo_.GKI,
                                                  &xpsCorrectorInfo_.GKD,
                                                  &xpsCorrectorInfo_.KForm,
                                                  &xpsCorrectorInfo_.FeedForwardGainVelocity,
                                                  &xpsCorrectorInfo_.FeedForwardGainAcceleration,
                                                  &xpsCorrectorInfo_.Friction);
  return status ? asynError : asynSuccess;
}


/**
 * Wrapper function for PositionerCorrectorPIPositionSet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIPositionSetWrapper()
{
  int status;
  status = PositionerCorrectorPIPositionSet(pollSocket_,
                                            positionerName_,
                                            xpsCorrectorInfo_.ClosedLoopStatus,
                                            xpsCorrectorInfo_.KP, 
                                            xpsCorrectorInfo_.KI, 
                                            xpsCorrectorInfo_.IntegrationTime);
  return status ? asynError : asynSuccess;
}


/**
 * Wrapper function for PositionerCorrectorPIDFFVelocitySet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDFFVelocitySetWrapper()
{
  int status;
  status = PositionerCorrectorPIDFFVelocitySet(pollSocket_,
                                               positionerName_,
                                               xpsCorrectorInfo_.ClosedLoopStatus,
                                               xpsCorrectorInfo_.KP, 
                                               xpsCorrectorInfo_.KI,
                                               xpsCorrectorInfo_.KD,
                                               xpsCorrectorInfo_.KS,
                                               xpsCorrectorInfo_.IntegrationTime,
                                               xpsCorrectorInfo_.DerivativeFilterCutOffFrequency,
                                               xpsCorrectorInfo_.GKP,
                                               xpsCorrectorInfo_.GKI,
                                               xpsCorrectorInfo_.GKD,
                                               xpsCorrectorInfo_.KForm,
                                               xpsCorrectorInfo_.FeedForwardGainVelocity);
  return status ? asynError : asynSuccess;
}

/**
 * Wrapper function for PositionerCorrectorPIDFFAccelerationSet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis:: PositionerCorrectorPIDFFAccelerationSetWrapper()
{
  int status;
  status = PositionerCorrectorPIDFFAccelerationSet(pollSocket_,
                                                   positionerName_,
                                                   xpsCorrectorInfo_.ClosedLoopStatus,
                                                   xpsCorrectorInfo_.KP, 
                                                   xpsCorrectorInfo_.KI,
                                                   xpsCorrectorInfo_.KD,
                                                   xpsCorrectorInfo_.KS,
                                                   xpsCorrectorInfo_.IntegrationTime,
                                                   xpsCorrectorInfo_.DerivativeFilterCutOffFrequency,
                                                   xpsCorrectorInfo_.GKP,
                                                   xpsCorrectorInfo_.GKI,
                                                   xpsCorrectorInfo_.GKD,
                                                   xpsCorrectorInfo_.KForm,
                                                   xpsCorrectorInfo_.FeedForwardGainAcceleration);
  return status ? asynError : asynSuccess;
}

/**
 * Wrapper function for PositionerCorrectorPIDDualFFVoltageSet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDDualFFVoltageSetWrapper()
{
  int status;
  status = PositionerCorrectorPIDDualFFVoltageSet(pollSocket_,
                                                  positionerName_,
                                                  xpsCorrectorInfo_.ClosedLoopStatus,
                                                  xpsCorrectorInfo_.KP, 
                                                  xpsCorrectorInfo_.KI,
                                                  xpsCorrectorInfo_.KD,
                                                  xpsCorrectorInfo_.KS,
                                                  xpsCorrectorInfo_.IntegrationTime,
                                                  xpsCorrectorInfo_.DerivativeFilterCutOffFrequency,
                                                  xpsCorrectorInfo_.GKP,
                                                  xpsCorrectorInfo_.GKI,
                                                  xpsCorrectorInfo_.GKD,
                                                  xpsCorrectorInfo_.KForm,
                                                  xpsCorrectorInfo_.FeedForwardGainVelocity,
                                                  xpsCorrectorInfo_.FeedForwardGainAcceleration,
                                                  xpsCorrectorInfo_.Friction);
  return status ? asynError : asynSuccess;
}

/** The following functions have C linkage, and can be called directly or from iocsh */
extern "C" {
/**
 * Function to enable/disable the write down of position to the 
 * XPS controller. Call this function at IOC shell.
 * @param setPos 0=disable, 1=enable
 */
/* void XPSEnableSetPosition(int setPos) 
{
  doSetPosition = setPos;
}
 */
/**
 * Function to set the threadSleep time used when setting the XPS position.
 * The sleep is performed after the axes are initialised, to take account of any
 * post initialisation wobble.
 * @param posSleep The time in miliseconds to sleep.
 */
/* void XPSSetPosSleepTime(int posSleep) 
{
  setPosSleepTime = (double)posSleep / 1000.0;
}
 */
asynStatus XPSCreate(const char *portName, const char *IPAddress, int IPPort,
                         int numAxes, int movingPollPeriod, int idlePollPeriod)
{
    XPSController *pXPSController
        = new XPSController(portName, IPAddress, IPPort, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    pXPSController = NULL;
    return(asynSuccess);
}

asynStatus XPSCreateAxis(const char *XPSName,         /* specify which controller by port name */
                         int axis,                    /* axis number 0-7 */
                         const char *positionerName,  /* groupName.positionerName e.g. Diffractometer.Phi */
                         int stepsPerUnit)            /* steps per user unit */
{
  XPSController *pC;
  XPSAxis *pAxis;
  static const char *functionName = "XPSCreateAxis";

  pC = (XPSController*) findAsynPortDriver(XPSName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, XPSName);
    return asynError;
  }
  pAxis = new XPSAxis(pC, axis, positionerName, 1./stepsPerUnit);
  pAxis = NULL;
  return(asynSuccess);
}

/* Code for iocsh registration */

/* XPSCreate */
static const iocshArg XPSCreateArg0 = {"Controller port name", iocshArgString};
static const iocshArg XPSCreateArg1 = {"IP address", iocshArgString};
static const iocshArg XPSCreateArg2 = {"IP port", iocshArgInt};
static const iocshArg XPSCreateArg3 = {"Number of axes", iocshArgInt};
static const iocshArg XPSCreateArg4 = {"Moving poll rate (ms)", iocshArgInt};
static const iocshArg XPSCreateArg5 = {"Idle poll rate (ms)", iocshArgInt};
static const iocshArg * const XPSCreateArgs[6] = {&XPSCreateArg0,
                                                  &XPSCreateArg1,
                                                  &XPSCreateArg2,
                                                  &XPSCreateArg2,
                                                  &XPSCreateArg4,
                                                  &XPSCreateArg5};
static const iocshFuncDef createXPS = {"XPSCreate", 6, XPSCreateArgs};
static void createXPSCallFunc(const iocshArgBuf *args)
{
  XPSCreate(args[0].sval, args[1].sval, args[2].ival, 
            args[3].ival, args[4].ival, args[5].ival);
}


/* XPSCreateAxis */
static const iocshArg XPSCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg XPSCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg XPSCreateAxisArg2 = {"Axis name", iocshArgString};
static const iocshArg XPSCreateAxisArg3 = {"Steps per unit", iocshArgInt};
static const iocshArg * const XPSCreateAxisArgs[4] = {&XPSCreateAxisArg0,
                                                      &XPSCreateAxisArg1,
                                                      &XPSCreateAxisArg2,
                                                      &XPSCreateAxisArg3};
static const iocshFuncDef createXPSAxis = {"XPSCreateAxis", 4, XPSCreateAxisArgs};

static void createXPSAxisCallFunc(const iocshArgBuf *args)
{
  XPSCreateAxis(args[0].sval, args[1].ival, args[2].sval, args[3].ival);
}


/* void XPSEnableSetPosition(int setPos) */
static const iocshArg XPSEnableSetPositionArg0 = {"Set Position Flag", iocshArgInt};
static const iocshArg * const XPSEnableSetPositionArgs[1] = {&XPSEnableSetPositionArg0};
static const iocshFuncDef xpsEnableSetPosition = {"XPSEnableSetPosition", 1, XPSEnableSetPositionArgs};
static void xpsEnableSetPositionCallFunc(const iocshArgBuf *args)
{
//  XPSEnableSetPosition(args[0].ival);
}

/* void XPSSetPosSleepTime(int posSleep) */
static const iocshArg XPSSetPosSleepTimeArg0 = {"Set Position Sleep Time", iocshArgInt};
static const iocshArg * const XPSSetPosSleepTimeArgs[1] = {&XPSSetPosSleepTimeArg0};
static const iocshFuncDef xpsSetPosSleepTime = {"XPSSetPosSleepTime", 1, XPSSetPosSleepTimeArgs};
static void xpsSetPosSleepTimeCallFunc(const iocshArgBuf *args)
{
//  XPSSetPosSleepTime(args[0].ival);
}


static void XPSRegister3(void)
{
  iocshRegister(&createXPS,            createXPSCallFunc);
  iocshRegister(&createXPSAxis,        createXPSAxisCallFunc);
//  iocshRegister(&xpsEnableSetPosition, xpsEnableSetPositionCallFunc);
//  iocshRegister(&xpsSetPosSleepTime,   xpsSetPosSleepTimeCallFunc);
}

epicsExportRegistrar(XPSRegister3);
} // extern "C"
