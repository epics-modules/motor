/*
FILENAME...     XPSMotorDriver.cpp
USAGE...        Newport XPS EPICS asyn motor device driver

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

#include <iostream>
using std::cout;
using std::endl;

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <epicsString.h>
#include <iocsh.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <epicsExport.h>
#include "XPSController.h"
#include "XPS_C8_drivers.h"
#include "asynOctetSocket.h"
#include "XPSAxis.h"

#define XPSC8_END_OF_RUN_MINUS  0x80000100
#define XPSC8_END_OF_RUN_PLUS   0x80000200
#define XPSC8_ZM_HIGH_LEVEL     0x00000004
/** Deadband to use for the velocity comparison with zero. */
#define XPS_VELOCITY_DEADBAND 0.0000001

static const char *driverName = "XPSAxis";

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

static void shutdownCallback(void *pPvt)
{
  XPSController *pC = static_cast<XPSController *>(pPvt);

  pC->lock();
  pC->shuttingDown_ = 1;
  pC->unlock();
}

// These are the XPSAxis:: methods
XPSAxis::XPSAxis(XPSController *pC, int axisNo, const char *positionerName, double stepSize)
  :   asynMotorAxis(pC, axisNo),
      pC_(pC)
{
  static const char *functionName = "XPSAxis";
  char *index;
  double minJerkTime, maxJerkTime;

  moveSocket_ = TCP_ConnectToServer(pC_->IPAddress_, pC->IPPort_, XPS_POLL_TIMEOUT);
  if (moveSocket_ < 0) {
    printf("%s:%s: error calling TCP_ConnectToServer for move socket\n",
           driverName, functionName);
  }
  /* Set the poll rate on the moveSocket to a negative number, which means that SendAndReceive should do only a write, no read */
  TCP_SetTimeout(moveSocket_, -0.1);
  pollSocket_ = pC_->pollSocket_;
  
  /* Set an EPICS exit handler that will shut down polling before asyn kills the IP sockets */
  epicsAtExit(shutdownCallback, pC_);

  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  setDoubleParam(pC_->motorPGain_, xpsCorrectorInfo_.KP);
  setDoubleParam(pC_->motorIGain_, xpsCorrectorInfo_.KI);
  setDoubleParam(pC_->motorDGain_, xpsCorrectorInfo_.KD);
  callParamCallbacks();
  /* Initialise deferred move flags. */
  deferredRelative_ = false;
  deferredPosition_ = 0.0;
  /* Disable deferred move for the axis. Should not cause move of this axis
     if other axes in same group do deferred move. */
  deferredMove_ = false;
  
  // Assume axis is not moving
  moving_ = false;

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
  PositionerSGammaParametersGet(pollSocket_,
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


void XPSAxis::report(FILE *fp, int details)
{
  fprintf(fp, "  axis %d\n"
              "    name = %s\n"
              "    step size = %g\n"
              "    poll socket = %d, moveSocket = %d\n"
              "    status = %d\n", 
          axisNo_,
          positionerName_, 
          stepSize_, 
          pollSocket_, moveSocket_, 
          axisStatus_);
}


asynStatus XPSAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  char errorString[100];
  double deviceUnits;
  int status;
  double minJerk, maxJerk;
  static const char *functionName = "move";

  pC_->getDoubleParam(axisNo_, pC_->XPSMinJerk_, &minJerk); 
  pC_->getDoubleParam(axisNo_, pC_->XPSMaxJerk_, &maxJerk); 
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: Set XPS %s, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f, minJerk=%f, maxJerk=%f\n",
            driverName, functionName, pC_->portName, axisNo_, position, min_velocity, max_velocity, acceleration, minJerk, maxJerk);

  /* Look at the last poll value of the positioner status.  If it is disabled, then enable it */
  /* This can be disabled by calling XPSDisableAutoEnable() at the IOC shell.*/
  if (axisStatus_ >= 20 && axisStatus_ <= 36) {
    if (pC_->autoEnable_) {
      status = GroupMotionEnable(pollSocket_, groupName_);
      if (status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: motorAxisMove[%s,%d]: error performing GroupMotionEnable %d\n",
                  driverName, functionName, pC_->portName, axisNo_, status);
        return asynError;
      }
    } else {
      //Return error if a move is attempted and auto enable is turned off.
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
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move! */
        return asynError;
      }
      moving_ = true;
    } else {
      deferredPosition_ = deviceUnits;
      deferredMove_ = true;
      deferredRelative_ = (relative != 0);
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
      moving_ = true;
    } else {
      deferredPosition_ = deviceUnits;
      deferredMove_ = true;
      deferredRelative_ = (relative != 0);
    }
  }

  return asynSuccess;
}

asynStatus XPSAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards)
{
  int status;
  int groupStatus;
  char errorBuffer[100];
  static const char *functionName = "home";

  /* Find out if any axes are in the same group, and set referencing mode for them all.*/
  XPSAxis *pTempAxis = NULL;
  for (int axis=0; axis<pC_->numAxes_; axis++) {
    pTempAxis = pC_->getAxis(axis);
    if (strcmp(groupName_, pTempAxis->groupName_) == 0) {
      pTempAxis->referencingMode_ = 0;
    }  
  }

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
  if ((groupStatus >= 0 && groupStatus <= 9) || (groupStatus == 50) || (groupStatus == 63)) {
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
  moving_ = true;

  return asynSuccess;
}


asynStatus XPSAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration)
{
  int status;
  double deviceVelocity, deviceAcceleration;
  static const char *functionName = "moveVelocity";

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
  moving_ = true;

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
  static const char *functionName = "setPosition";


  /* If the user has disabled setting the controller position, skip this.*/
  if (!pC_->enableSetPosition_) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: XPS set position is disabled. Enable it by setting enableSetPosition parameter in XPSCreateController).\n",
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
    epicsThreadSleep(pC_->setPositionSettlingTime_);

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
    /*Some types of XPS axes (eg. spindle) need a sleep here, otherwise 
      the axis can be left in referencing mode.*/
    epicsThreadSleep(0.05);
    status = GroupReferencingStop(pollSocket_, 
                                  groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: Error performing referencing set position (%s,%d). XPS API Error: %d.\n", 
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
    epicsThreadSleep(pC_->setPositionSettlingTime_);

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
  static const char *functionName = "stopAxis";

  /* We need to read the status, because a jog is stopped differently from a move */ 
  status = GroupStatusGet(pollSocket_, groupName_, &axisStatus_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupStatusGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }
  
  if ((axisStatus_ == 44) || (axisStatus_ == 45) || (axisStatus_ == 47)) {
    status = GroupMoveAbort(moveSocket_, groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: [%s,%d]: error calling GroupMoveAbort status=%d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      GroupMoveAbort(moveSocket_, groupName_);
      return asynError;
    }
  }

  if (axisStatus_ == 43) {
    status = GroupKill(moveSocket_, groupName_);
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                "%s:%s: [%s,%d]: error calling GroupKill status=%d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }
  }

  /* Clear defer move flag for this axis. */
  deferredMove_ = false;

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: XPS %s, axis %d stop with accel=%f\n",
            driverName, functionName, pC_->portName, axisNo_, acceleration);

  return asynSuccess;
}

asynStatus XPSAxis::poll(bool *moving)
{
  int status;
  char readResponse[25];
  char statusString[MAX_MESSAGE_LEN] = {0};
  static const char *functionName = "poll";

  status = GroupStatusGet(pollSocket_, 
                          groupName_, 
                          &axisStatus_);
  if (!status) {
    status = GroupStatusStringGet(pollSocket_,
                                  axisStatus_,
                                  statusString); 
  }
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error calling GroupStatusGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: [%s,%d]: %s axisStatus=%d, statusString=%s\n",
            driverName, functionName, pC_->portName, axisNo_, positionerName_, axisStatus_, statusString);
  /* Set the status */
  setIntegerParam(pC_->XPSStatus_, axisStatus_);
  setStringParam(pC_->XPSStatusString_, statusString);
  
  /* Previously we set the motion done flag by seeing if axisStatus_ was >=43 && <= 48, which means moving,
   * homing, jogging, etc.  However, this information is about the group, not the axis, so if one
   * motor in the group was moving, then they all appeared to be moving.  This is not what we want, because
   * the EPICS motor record required the first motor to stop before the second motor could be moved. 
   * Instead we look for a response on the moveSocket_ to see when the motor motion was complete.
     NOTE: by default this mode is disabled. To enable it use the XPSController::enableMovingMode() function.*/
   
  /* If the group is not moving then the axis is not moving */
  if ((axisStatus_ < 43) || (axisStatus_ > 48)) {
    moving_ = false;
  } else {
    if (!pC_->enableMovingMode_) {
      moving_ = true;
    }
  }
  
  /* If the axis is moving then read from the moveSocket to see if it is done 
   * We currently assume the move is complete if we get any response, we don't
   * check the actual response. */
  if (pC_->enableMovingMode_) {
    if (moving_) {
      status = ReadXPSSocket(moveSocket_, readResponse, sizeof(readResponse), 0);
      if (status < 0) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
                  "%s:%s: [%s,%d]: error calling ReadXPSSocket status=%d\n",
                  driverName, functionName, pC_->portName, axisNo_,  status);
        goto done;
      }
      if (status > 0) {
        asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
                  "%s:%s: [%s,%d]: readXPSSocket returned nRead=%d, [%s]\n",
                  driverName, functionName, pC_->portName, axisNo_,  status, readResponse);
        status = 0;
        moving_ = false;
      }
    }
  }

  /* Set the axis done parameter */
  *moving = moving_;
  if (deferredMove_) *moving = true;
  setIntegerParam(pC_->motorStatusDone_, *moving?0:1);

  /*Read the controller software limits in case these have been changed by a TCL script.*/
  status = PositionerUserTravelLimitsGet(pollSocket_, positionerName_, &lowLimit_, &highLimit_);
  if (status == 0) {
    setDoubleParam(pC_->motorHighLimit_, (highLimit_/stepSize_));
    setDoubleParam(pC_->motorLowLimit_, (lowLimit_/stepSize_));
  }

  /* Set the ATHM signal.*/
  if (axisStatus_ == 11) {
    if (referencingMode_ == 0) {
      setIntegerParam(pC_->motorStatusHome_, 1);
    } else {
      setIntegerParam(pC_->motorStatusHome_, 0);
    }
  } else {
    setIntegerParam(pC_->motorStatusHome_, 0);
  }

  /* Set the HOMED signal.*/
  if ((axisStatus_ >= 10 && axisStatus_ <= 21) || 
      (axisStatus_ == 44) || (axisStatus_ == 45) || (axisStatus_ == 47)) {
    if (referencingMode_ == 0) {
      setIntegerParam(pC_->motorStatusHomed_, 1);
    } else {
      setIntegerParam(pC_->motorStatusHomed_, 0);
    }
  } else {
    setIntegerParam(pC_->motorStatusHomed_, 0);
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

asynStatus XPSAxis::setLowLimit(double value)
{
  double deviceValue;
  int status;
  static const char *functionName = "setLowLimit";
  
  deviceValue = value*stepSize_;
  /* We need to read the current highLimit because otherwise we could be setting it to an invalid value */
  status = PositionerUserTravelLimitsGet(pollSocket_,
                                         positionerName_,
                                         &lowLimit_, &highLimit_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error performing PositionerUserTravelLimitsGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  status = PositionerUserTravelLimitsSet(pollSocket_,
                                         positionerName_,
                                         deviceValue, highLimit_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
          "%s:%s: [%s,%d]: error performing PositionerUserTravelLimitsSet for lowLim=%f status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, deviceValue, status);
    goto done;
  } 
  lowLimit_ = deviceValue;
  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: Set XPS %s, axis %d low limit to %f\n", 
            driverName, functionName, pC_->portName, axisNo_, deviceValue);
  
  done:
  return (asynStatus)status;
}

  
asynStatus XPSAxis::setHighLimit(double value)
{
  double deviceValue;
  int status;
  static const char *functionName = "setHighLimit";
  
  deviceValue = value*stepSize_;
  /* We need to read the current highLimit because otherwise we could be setting it to an invalid value */
  status = PositionerUserTravelLimitsGet(pollSocket_,
                                         positionerName_,
                                         &lowLimit_, &highLimit_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
              "%s:%s: [%s,%d]: error performing PositionerUserTravelLimitsGet status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    goto done;
  }
  status = PositionerUserTravelLimitsSet(pollSocket_,
                                         positionerName_,
                                         lowLimit_, deviceValue);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, 
          "%s:%s: [%s,%d]: error performing PositionerUserTravelLimitsSet for highLim=%f status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, deviceValue, status);
    goto done;
  } 
  highLimit_ = deviceValue;
  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: Set XPS %s, axis %d high limit to %f\n", 
            driverName, functionName, pC_->portName, axisNo_, deviceValue);
  
  done:
  return (asynStatus)status;
}


asynStatus XPSAxis::setPGain(double value)
{
  return setPID(&value, 0);
}


asynStatus XPSAxis::setIGain(double value)
{
  return setPID(&value, 1);
}


asynStatus XPSAxis::setDGain(double value)
{
  return setPID(&value, 2);
}


asynStatus XPSAxis::setClosedLoop(bool closedLoop)
{
  int status;
  static const char *functionName = "setClosedLoop";
  
  if (closedLoop) {
    status = GroupMotionEnable(pollSocket_, groupName_);
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
    status = GroupMotionDisable(pollSocket_, groupName_);
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

double XPSAxis::motorRecPositionToXPSPosition(double motorRecPosition)
{
  int direction;
  double offset, resolution, XPSPosition;
  
  pC_->getDoubleParam (axisNo_, pC_->motorRecResolution_,   &resolution);
  pC_->getDoubleParam (axisNo_, pC_->motorRecOffset_,       &offset);
  pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_,    &direction);
  
  if (direction != 0) resolution = -resolution;
  if (resolution == 0) resolution = 1;
  XPSPosition = (motorRecPosition - offset) / resolution * stepSize_;
  return XPSPosition;
}

double XPSAxis::XPSPositionToMotorRecPosition(double XPSPosition)
{
  int direction;
  double offset, resolution, motorRecPosition;
  
  pC_->getDoubleParam (axisNo_, pC_->motorRecResolution_,   &resolution);
  pC_->getDoubleParam (axisNo_, pC_->motorRecOffset_,       &offset);
  pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_,    &direction);
  
  if (direction != 0) resolution = -resolution;
  if (stepSize_ == 0) stepSize_ = 1;
  motorRecPosition = XPSPosition * resolution / stepSize_ + offset;
  return motorRecPosition;
}

double XPSAxis::motorRecStepToXPSStep(double motorRecStep)
{
  int direction;
  double resolution, XPSStep;
  
  pC_->getDoubleParam (axisNo_, pC_->motorRecResolution_,   &resolution);
  pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_,    &direction);
  
  if (direction != 0) resolution = -resolution;
  if (resolution == 0) resolution = 1;
  XPSStep = motorRecStep / resolution * stepSize_;
  return XPSStep;
}

double XPSAxis::XPSStepToMotorRecStep(double XPSStep)
{
  int direction;
  double resolution, motorRecStep;
  
  pC_->getDoubleParam (axisNo_, pC_->motorRecResolution_,   &resolution);
  pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_,    &direction);
  
  if (direction != 0) resolution = -resolution;
  if (stepSize_ == 0) stepSize_ = 1;
  motorRecStep = XPSStep * resolution / stepSize_;
  return motorRecStep;
}

asynStatus XPSAxis::setPositionCompare()
{
  int mode;
  double minPosition, maxPosition, stepSize, pulseWidth, settlingTime;
  int itemp;
  int direction;
  int status;
  static const char *functionName = "setPositionCompare";

  pC_->getIntegerParam(axisNo_, pC_->XPSPositionCompareMode_,         &mode);
  pC_->getDoubleParam (axisNo_, pC_->XPSPositionCompareMinPosition_,  &minPosition);
  pC_->getDoubleParam (axisNo_, pC_->XPSPositionCompareMaxPosition_,  &maxPosition);
  pC_->getDoubleParam (axisNo_, pC_->XPSPositionCompareStepSize_,     &stepSize);
  pC_->getIntegerParam(axisNo_, pC_->XPSPositionComparePulseWidth_,   &itemp);
  pulseWidth = positionComparePulseWidths[itemp];
  pC_->getIntegerParam(axisNo_, pC_->XPSPositionCompareSettlingTime_, &itemp);
  settlingTime = positionCompareSettlingTimes[itemp];
  
  // minPosition and maxPosition are in motor record units. Convert to XPS units
  minPosition = motorRecPositionToXPSPosition(minPosition);
  maxPosition = motorRecPositionToXPSPosition(maxPosition);
  stepSize = fabs(motorRecStepToXPSStep(stepSize));
  
  // Swap max and min positions if needed
  pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &direction);
  if (direction != 0) {
    double temp=maxPosition;
    maxPosition = minPosition;
    minPosition = temp;
  }

  // Disable the position compare so we can set parameters
  status = PositionerPositionCompareDisable(pollSocket_, positionerName_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: [%s,%d]: error calling PositionerPositionCompareDisable status=%d\n",
              driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }
  status = PositionerPositionComparePulseParametersSet(pollSocket_, positionerName_, pulseWidth, settlingTime);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: [%s,%d]: error calling PositionerPositionComparePulseParametersSet"
              " status=%d, pulseWidth=%f, settlingTime=%f\n",
               driverName, functionName, pC_->portName, axisNo_, status, pulseWidth, settlingTime);
    return asynError;
  }
  switch (mode) {
    case XPSPositionCompareModeDisable:
      break;
  
    case XPSPositionCompareModePulse:
      status = PositionerPositionCompareSet(pollSocket_, positionerName_, minPosition, maxPosition, stepSize);
      if (status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: [%s,%d]: error calling PositionerPositionCompareSet"
                  " status=%d, minPosition=%f, maxPosition=%f, stepSize=%f\n",
                   driverName, functionName, pC_->portName, axisNo_, status, minPosition, maxPosition, stepSize);
        return asynError;
      }
      status = PositionerPositionCompareEnable(pollSocket_, positionerName_);
      if (status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: [%s,%d]: error calling PositionerPositionCompareEnable status=%d\n",
                   driverName, functionName, pC_->portName, axisNo_, status);
        return asynError;
      }
      break;

    case XPSPositionCompareModeAquadBWindowed:
      status = PositionerPositionCompareAquadBWindowedSet(pollSocket_, positionerName_, minPosition, maxPosition);
      if (status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: [%s,%d]: error calling PositionerPositionCompareAquadBWindowedSet"
                  " status=%d, minPosition=%f, maxPosition=%f\n",
                   driverName, functionName, pC_->portName, axisNo_, status, minPosition, maxPosition);
        return asynError;
      }
      status = PositionerPositionCompareEnable(pollSocket_, positionerName_);
      if (status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: [%s,%d]: error calling PositionerPositionCompareEnable status=%d\n",
                   driverName, functionName, pC_->portName, axisNo_, status);
        return asynError;
      }
      break;

    case XPSPositionCompareModeAquadBAlways:
      status = PositionerPositionCompareAquadBAlwaysEnable(pollSocket_, positionerName_);
      if (status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  "%s:%s: [%s,%d]: error calling PositionerPositionCompareAquadBAlwaysEnable status=%d\n",
                   driverName, functionName, pC_->portName, axisNo_, status);
        return asynError;
      }
      break;
  } 
  
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: set XPS %s, axis %d positionCompare set and enable\n",
             driverName, functionName, pC_->portName, axisNo_);

  return asynSuccess;
}

asynStatus XPSAxis::getPositionCompare()
{
  bool enable;
  int status;
  int i;
  int direction;
  double minPosition=0, maxPosition=0, stepSize=0, pulseWidth, settlingTime;
  static const char *functionName = "getPositionCompare";
  
  pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &direction);

  status = PositionerPositionComparePulseParametersGet(pollSocket_, positionerName_, &pulseWidth, &settlingTime);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: [%s,%d]: error calling PositionerPositionComparePulseParametersGet status=%d\n",
               driverName, functionName, pC_->portName, axisNo_, status);
    return asynError;
  }
  status = PositionerPositionCompareGet(pollSocket_, positionerName_, &minPosition, &maxPosition, &stepSize, &enable);
  if (status == 0) {
    // Swap max and min positions if needed
    if (direction != 0) {
      double temp=maxPosition;
      maxPosition = minPosition;
      minPosition = temp;
    }
//    setDoubleParam(pC_->XPSPositionCompareMinPosition_,  XPSPositionToMotorRecPosition(minPosition));
//    setDoubleParam(pC_->XPSPositionCompareMaxPosition_,  XPSPositionToMotorRecPosition(maxPosition));
    setDoubleParam(pC_->XPSPositionCompareStepSize_,     fabs(XPSStepToMotorRecStep(stepSize)));
  } 
  status = PositionerPositionCompareAquadBWindowedGet(pollSocket_, positionerName_, &minPosition, &maxPosition, &enable);
  if (status == 0) {
    // Swap max and min positions if needed
    if (direction != 0) {
      double temp=maxPosition;
      maxPosition = minPosition;
      minPosition = temp;
    }
//    setDoubleParam(pC_->XPSPositionCompareMinPosition_,  XPSPositionToMotorRecPosition(minPosition));
//    setDoubleParam(pC_->XPSPositionCompareMaxPosition_,  XPSPositionToMotorRecPosition(maxPosition));
  } 
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: set XPS %s, axis %d "
            " enable=%d, minPosition=%f, maxPosition=%f, stepSize=%f, pulseWidth=%f, settlingTime=%f\n",
             driverName, functionName, pC_->portName, axisNo_,
             enable, minPosition, maxPosition, stepSize, pulseWidth, settlingTime);

  for (i=0; i<MAX_PULSE_WIDTHS; i++) {
    if (fabs(pulseWidth - positionComparePulseWidths[i]) < 0.001) break;
  }
  setIntegerParam(pC_->XPSPositionComparePulseWidth_,  i);
  for (i=0; i<MAX_SETTLING_TIMES; i++) {
    if (fabs(settlingTime - positionCompareSettlingTimes[i]) < 0.001) break;
  }
  setIntegerParam(pC_->XPSPositionCompareSettlingTime_,  i);
  return asynSuccess;
}

char *XPSAxis::getXPSError(int status, char *buffer)
{
    status = ErrorStringGet(pollSocket_, status, buffer);
    return buffer;
}

/**
 * Test if axis is configured as an XPS single axis or a group.
 * This is done by comparing cached group names.
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
  static const char *functionName = "setPID";

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
    status = PositionerCorrectorPIPositionGet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIPositionGet. Aborting setting PID XPS: %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    /*Set the P, I or D parameter in the xpsCorrectorInfo struct.*/
    setPIDValue(value, pidoption); 

    /*Now set the parameters in the XPS.*/
    status = PositionerCorrectorPIPositionSet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIPositionSet: %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }
  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFVelocity)) {
    status = PositionerCorrectorPIDFFVelocityGet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFVelocityGet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    setPIDValue(value, pidoption); 

    status = PositionerCorrectorPIDFFVelocitySet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFVelocitySet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFAcceleration)) {
    status = PositionerCorrectorPIDFFAccelerationGet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFAccelerationGet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    setPIDValue(value, pidoption); 

    status = PositionerCorrectorPIDFFAccelerationSet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDFFAccelerationSet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDDualFFVoltage)) {
    status = PositionerCorrectorPIDDualFFVoltageGet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: Error with PositionerCorrectorPIDDualFFVoltageGet. Aborting setting PID %s, Axis: %d, XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

    setPIDValue(value, pidoption); 

    status = PositionerCorrectorPIDDualFFVoltageSet();
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
  static const char *functionName = "getPID";
  
  /* The XPS function that we use to set the PID parameters is dependant on the 
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
    status = PositionerCorrectorPIPositionGet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: XPS %s, Axis: %d, Error with PositionerCorrectorPIPositionGet. XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFVelocity)) {
    status = PositionerCorrectorPIDFFVelocityGet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: XPS %s, Axis: %d, Error with PositionerCorrectorPIDFFVelocityGet. XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDFFAcceleration)) {
    status = PositionerCorrectorPIDFFAccelerationGet();
    if (status) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                "%s:%s: XPS %s, Axis: %d, Error with PositionerCorrectorPIDFFAccelerationGet. XPS API Error: %d\n",
                driverName, functionName, pC_->portName, axisNo_, status);
      return asynError;
    }

  } else if (!strcmp(correctorType, CorrectorTypes.PIDDualFFVoltage)) {
    status = PositionerCorrectorPIDDualFFVoltageGet();
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
  static const char *functionName = "setPIDValue";

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
 * It will set parameters in this object..
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIPositionGet()
{
  int status;
  status = ::PositionerCorrectorPIPositionGet(pollSocket_,
                                              positionerName_,
                                              &xpsCorrectorInfo_.ClosedLoopStatus,
                                              &xpsCorrectorInfo_.KP, 
                                              &xpsCorrectorInfo_.KI, 
                                              &xpsCorrectorInfo_.IntegrationTime);
  return status ? asynError : asynSuccess;
}

/**
 * Wrapper function for PositionerCorrectorPIDFFVelocityGet.
 * It will set parameters in this object..
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDFFVelocityGet()
{
  int status;
  status = ::PositionerCorrectorPIDFFVelocityGet(pollSocket_,
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
 * It will set parameters in this object..
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDFFAccelerationGet()
{
  int status;
  status = ::PositionerCorrectorPIDFFAccelerationGet(pollSocket_,
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
 * It will set parameters in this object..
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDDualFFVoltageGet()
{
  int status;
  status =  ::PositionerCorrectorPIDDualFFVoltageGet(pollSocket_,
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
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIPositionSet()
{
  int status;
  status = ::PositionerCorrectorPIPositionSet(pollSocket_,
                                              positionerName_,
                                              xpsCorrectorInfo_.ClosedLoopStatus,
                                              xpsCorrectorInfo_.KP, 
                                              xpsCorrectorInfo_.KI, 
                                              xpsCorrectorInfo_.IntegrationTime);
  return status ? asynError : asynSuccess;
}


/**
 * Wrapper function for PositionerCorrectorPIDFFVelocitySet.
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDFFVelocitySet()
{
  int status;
  status = ::PositionerCorrectorPIDFFVelocitySet(pollSocket_,
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
 * @return Return value from XPS function.
 */
asynStatus XPSAxis:: PositionerCorrectorPIDFFAccelerationSet()
{
  int status;
  status = ::PositionerCorrectorPIDFFAccelerationSet(pollSocket_,
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
 * @return Return value from XPS function.
 */
asynStatus XPSAxis::PositionerCorrectorPIDDualFFVoltageSet()
{
  int status;
  status = ::PositionerCorrectorPIDDualFFVoltageSet(pollSocket_,
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

/** Function to define the motor positions for a profile move. 
  * This calls the base class function to convert to steps, but since the
  * XPS works in user-units we need to do an additional conversion by stepSize_. 
  * \param[in] positions Array of profile positions for this axis in user units.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus XPSAxis::defineProfile(double *positions, size_t numPoints)
{
  size_t i;
  asynStatus status;
  //static const char *functionName = "defineProfile";
  
  // Call the base class function
  status = asynMotorAxis::defineProfile(positions, numPoints);
  if (status) return status;
  
  // Convert to XPS units from steps
  for (i=0; i<numPoints; i++) {
    profilePositions_[i] = profilePositions_[i]*stepSize_;
  }
  return asynSuccess;
}


/** Function to readback the actual motor positions from a coordinated move of multiple axes.
  * This scales by the stepSize to get to steps, and then calls the base class method
  * that converts to user units
  * Caution: this function modifies the readbacks in place, so it must only be called
  * once per readback operation.
 */
asynStatus XPSAxis::readbackProfile()
{
  int i;
  int numReadbacks;
  int status=0;
  // static const char *functionName = "readbackProfile";
 
  status |= pC_->getIntegerParam(pC_->profileNumReadbacks_, &numReadbacks);
  if (status) return asynError;

  // Convert to steps
  for (i=0; i<numReadbacks; i++) {
    profileReadbacks_[i]       /= stepSize_;
    profileFollowingErrors_[i] /= stepSize_;
  }
  // Call the base class method
  asynMotorAxis::readbackProfile();
  return asynSuccess;
}


/**
 * XPS implementation of the move to home function
 *
 * It first does a kill, followed by a referencing start/stop 
 * sequence on an group. Then uses the hardware status to
 * determine which direction to move.
 * The distance to move is set when enabling this functionality
 * (it is disabled by default, because it only applies to
 * stages with home switches in the middle of travel).
 *
 */
asynStatus XPSAxis::doMoveToHome(void) 
{
  int status = 0;
  int axis = 0;
  int groupStatus = 0;
  int initialHardwareStatus = 0;
  int hardwareStatus = 0;
  double defaultDistance = 0;
  double vel=0;
  double accel=0;
  double minJerk=0;
  double maxJerk=0;
  static const char * functionName = "XPSAxis::doMoveToHome";

  XPSAxis *pTempAxis = NULL;

  if (getReferencingModeMove() == 0) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: This function has not been enabled for axis %d\n",
              driverName, functionName, axisNo_);
    return asynError;
  }

  defaultDistance = (double) getReferencingModeMove();
  
  /*NOTE: the XPS has some race conditions in its firmware. That's why I placed some
    epicsThreadSleep calls between the XPS functions below.*/

  /* The XPS won't allow a home command if the group is in the Ready state
     * If the group is Ready, then make it not Ready  */
  status = GroupStatusGet(pollSocket_, groupName_, &groupStatus);
  if (groupStatus >= 10 && groupStatus <= 18) {
    status = GroupKill(moveSocket_, groupName_);
  }
  epicsThreadSleep(0.05);
  status = GroupInitialize(pollSocket_, groupName_);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: error calling GroupInitialize\n", driverName, functionName);
    return asynError;
  }
  epicsThreadSleep(0.05);
  status = GroupReferencingStart(moveSocket_, groupName_);
  epicsThreadSleep(0.05);
  status = GroupReferencingStop(moveSocket_, groupName_);
  epicsThreadSleep(0.05);

  status = GroupStatusGet(pollSocket_, groupName_, &groupStatus);
  if (groupStatus != 11) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: error putting axis into referencing mode.\n", driverName, functionName);
    return asynError;
  }
  
  /* Find out if any axes are in the same group, and set referencing mode for them all.*/
  for (axis=0; axis<pC_->numAxes_; axis++) {
    pTempAxis = pC_->getAxis(axis);
    if (strcmp(groupName_, pTempAxis->groupName_) == 0) {
      pTempAxis->referencingMode_ = 1;
    }  
  }
  
  /*Set status bits correctly*/
  pC_->lock();
  setIntegerParam(pC_->motorStatusHomed_, 0);
  setIntegerParam(pC_->motorStatusHome_, 0);
  callParamCallbacks();
  pC_->unlock();
  
  /*Read which side of the home switch we are on.*/
  status = PositionerHardwareStatusGet(pollSocket_, positionerName_, &initialHardwareStatus);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: error calling PositionerHardwareStatusGet.\n", driverName, functionName);
    return asynError;
  }
  
  if (!(XPSC8_ZM_HIGH_LEVEL & initialHardwareStatus)) {
    defaultDistance = defaultDistance * -1.0;
  }

  /*I want to set a slow speed here, so as not to move at default (max) speed. The user must have chance to
    stop things if it looks like it has past the home switch and is not stopping. First I need to read what is currently
    set for velocity, and then I divide it by 2.*/
  status = PositionerSGammaParametersGet(pollSocket_,
                                         positionerName_, 
                                         &vel, &accel, &minJerk, &maxJerk);
  if (status != 0) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: Error performing PositionerSGammaParametersGet.\n", driverName, functionName);
    GroupKill(moveSocket_, groupName_);
    return asynError;

  }
  status = PositionerSGammaParametersSet(pollSocket_,
                                         positionerName_, 
                                         (vel/2), accel, minJerk, maxJerk);
  if (status != 0) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: Error performing PositionerSGammaParametersSet.\n", driverName, functionName);
    GroupKill(moveSocket_, groupName_);
    return asynError;
  }
  epicsThreadSleep(0.05);
  
  /*Move in direction of home switch.*/
  status = GroupMoveRelative(moveSocket_, positionerName_, 1,
                             &defaultDistance); 
  if (status != 0) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: Error performing GroupMoveRelative.\n", driverName, functionName);
    /*Issue a kill here if we have failed to move.*/
    status = GroupKill(moveSocket_, groupName_);
    return asynError;
  }
  
  epicsThreadSleep(0.1);

  status = GroupStatusGet(pollSocket_, groupName_, &groupStatus);
  
  if (groupStatus == 44) {
    while (1) {
      epicsThreadSleep(0.2);
      status = PositionerHardwareStatusGet(pollSocket_, positionerName_, &hardwareStatus);
      if (hardwareStatus != initialHardwareStatus) {
        break;
      }
      status = GroupStatusGet(pollSocket_, groupName_, &groupStatus);
      if (groupStatus != 44) {
        /* move finished for some other reason.*/
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: Error performing GroupMoveRelative.\n", driverName, functionName);
        /*Issue a kill here if we have failed to move.*/
        status = GroupKill(moveSocket_, groupName_);
        return asynError;
      }
    }
  } else {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: Error performing GroupMoveRelative.\n", driverName, functionName);
    /*Issue a kill here if we have failed to move.*/
    status = GroupKill(moveSocket_, groupName_);
    return asynError;
  }

  status = GroupMoveAbort(pollSocket_, groupName_);
  if (status != 0) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: Error performing GroupMoveAbort.\n", driverName, functionName);
    /*This should really have worked. Do a kill instead.*/
    status = GroupKill(moveSocket_, groupName_);
    return asynError;
  }

  return asynSuccess;

}


