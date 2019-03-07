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
#include <errno.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <iocsh.h>
#include <asynDriver.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "XPSController.h"
#include "XPS_C8_drivers.h"
#include "xps_ftp.h"
#include "XPSAxis.h"

static const char *driverName = "XPSController";

static void XPSProfileThreadC(void *pPvt);

/** Struct for a list of strings describing the different corrector types possible on the XPS.*/
typedef struct {
  const char *PIPosition;
  const char *PIDFFVelocity;
  const char *PIDFFAcceleration;
  const char *PIDDualFFVoltage;
  const char *NoCorrector;
} CorrectorTypes_t;

const static CorrectorTypes_t CorrectorTypes = { 
  "PositionerCorrectorPIPosition",
  "PositionerCorrectorPIDFFVelocity",
  "PositionerCorrectorPIDFFAcceleration",
  "PositionerCorrectorPIDDualFFVoltage",
  "NoCorrector"
};

/* The maximum size of the item names in gathering, e.g. "GROUP2.POSITIONER1.CurrentPosition" */
#define MAX_GATHERING_AXIS_STRING 60
/* Number of items per axis */
#define NUM_GATHERING_ITEMS 2
/* Total length of gathering configuration string */
#define MAX_GATHERING_STRING MAX_GATHERING_AXIS_STRING * NUM_GATHERING_ITEMS * XPS_MAX_AXES
// Maximum number of bytes that GatheringDataMultipleLinesGet() can return
#define GATHERING_MAX_READ_LEN 65536

#ifndef MAX
#define MAX(a,b) ((a)>(b)? (a): (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a)<(b)? (a): (b))
#endif


XPSController::XPSController(const char *portName, const char *IPAddress, int IPPort, 
                             int numAxes, double movingPollPeriod, double idlePollPeriod,
                             int enableSetPosition, double setPositionSettlingTime)
  :  asynMotorController(portName, numAxes, NUM_XPS_PARAMS, 
                         0, // No additional interfaces
                         0, // No addition interrupt interfaces
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0),  // Default priority and stack size
     enableSetPosition_((enableSetPosition!=0)?true:false), 
     setPositionSettlingTime_(setPositionSettlingTime), 
     ftpUsername_(NULL), ftpPassword_(NULL)
{
  static const char *functionName = "XPSController";
  
  IPAddress_ = epicsStrDup(IPAddress);
  IPPort_ = IPPort;
  pAxes_ = (XPSAxis **)(asynMotorController::pAxes_);
  movesDeferred_ = false;

  // Create controller-specific parameters
  createParam(XPSMinJerkString,                       asynParamFloat64, &XPSMinJerk_);
  createParam(XPSMaxJerkString,                       asynParamFloat64, &XPSMaxJerk_);
  createParam(XPSPositionCompareModeString,           asynParamInt32,   &XPSPositionCompareMode_);
  createParam(XPSPositionCompareMinPositionString,    asynParamFloat64, &XPSPositionCompareMinPosition_);
  createParam(XPSPositionCompareMaxPositionString,    asynParamFloat64, &XPSPositionCompareMaxPosition_);
  createParam(XPSPositionCompareStepSizeString,       asynParamFloat64, &XPSPositionCompareStepSize_);
  createParam(XPSPositionComparePulseWidthString,     asynParamInt32,   &XPSPositionComparePulseWidth_);
  createParam(XPSPositionCompareSettlingTimeString,   asynParamInt32,   &XPSPositionCompareSettlingTime_);
  createParam(XPSProfileMaxVelocityString,            asynParamFloat64, &XPSProfileMaxVelocity_);
  createParam(XPSProfileMaxAccelerationString,        asynParamFloat64, &XPSProfileMaxAcceleration_);
  createParam(XPSProfileMinPositionString,            asynParamFloat64, &XPSProfileMinPosition_);
  createParam(XPSProfileMaxPositionString,            asynParamFloat64, &XPSProfileMaxPosition_);
  createParam(XPSProfileGroupNameString,              asynParamOctet,   &XPSProfileGroupName_);
  createParam(XPSTrajectoryFileString,                asynParamOctet,   &XPSTrajectoryFile_);
  createParam(XPSStatusString,                        asynParamInt32,   &XPSStatus_);
  createParam(XPSStatusStringString,                  asynParamOctet,   &XPSStatusString_);
  createParam(XPSTclScriptString,                     asynParamOctet,   &XPSTclScript_);
  createParam(XPSTclScriptExecuteString,              asynParamInt32,   &XPSTclScriptExecute_);

  // This socket is used for polling by the controller and all axes
  pollSocket_ = TCP_ConnectToServer((char *)IPAddress, IPPort, XPS_POLL_TIMEOUT);
  if (pollSocket_ < 0) {
    printf("%s:%s: error calling TCP_ConnectToServer for pollSocket\n",
           driverName, functionName);
  }
  
  // This socket is used for moving motors during profile moves
  // Each axis also has its own moveSocket
  moveSocket_ = TCP_ConnectToServer((char *)IPAddress, IPPort, XPS_MOVE_TIMEOUT);
  if (moveSocket_ < 0) {
    printf("%s:%s: error calling TCP_ConnectToServer for moveSocket\n",
           driverName, functionName);
  }
  
  FirmwareVersionGet(pollSocket_, firmwareVersion_);
  
  /* Create the poller thread for this controller
   * NOTE: at this point the axis objects don't yet exist, but the poller tolerates this */
  startPoller(movingPollPeriod, idlePollPeriod, 10);
  
  /* Start the thread that will handle move to home commands.*/
  startMoveToHomeThread();

  // Create the event that wakes up the thread for profile moves
  profileExecuteEvent_ = epicsEventMustCreate(epicsEventEmpty);
  
  // Create the thread that will execute profile moves
  epicsThreadCreate("XPSProfile", 
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)XPSProfileThreadC, (void *)this);

  //By default, automatically enable axes that have been disabled.
  autoEnable_ = 1;

  /* Flag used to turn off setting MSTA problem bit when the axis is disabled.*/
  noDisableError_ = 0;

  /* Flag to disable a mode to change the moving state determination for an axis.*/
  /* See function XPSController::enableMovingMode().*/
  enableMovingMode_ = false;

}

void XPSController::report(FILE *fp, int level)
{
  fprintf(fp, "XPS motor driver: %s\n", this->portName);
  fprintf(fp, "                 numAxes: %d\n", numAxes_);
  fprintf(fp, "        firmware version: %s\n", firmwareVersion_);
  fprintf(fp, "      moving poll period: %f\n", movingPollPeriod_);
  fprintf(fp, "        idle poll period: %f\n", idlePollPeriod_);
  
  if (level > 0) {
    fprintf(fp, "       enableSetPosition: %d\n", enableSetPosition_);
    fprintf(fp, " setPositionSettlingTime: %f\n", setPositionSettlingTime_);
    fprintf(fp, "               IPAddress: %s\n", IPAddress_);
    fprintf(fp, "                  IPPort: %d\n", IPPort_);
    fprintf(fp, "             ftpUserName: %s\n", ftpUsername_);
    fprintf(fp, "         ftpUserPassword: %s\n", ftpPassword_);
    fprintf(fp, "           movesDeferred: %d\n", movesDeferred_);
    fprintf(fp, "              autoEnable: %d\n", autoEnable_);
    fprintf(fp, "          noDisableError: %d\n", noDisableError_);
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

asynStatus XPSController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  XPSAxis *pAxis;
  //static const char *functionName = "writeInt32";

  pAxis = this->getAxis(pasynUser);
  if (!pAxis) return asynError;
  
  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = pAxis->setIntegerParam(function, value);

  if (function == XPSTclScriptExecute_) {
    /* Execute the TCL script */
    char fileName[MAX_FILENAME_LEN];
    getStringParam(XPSTclScript_, (int)sizeof(fileName), fileName);
    if (fileName != NULL) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "Executing TCL script %s on XPS: %s\n", 
                fileName, this->portName);
      status = TCLScriptExecute(pAxis->moveSocket_,
                                fileName,"0","0");
      if (status != 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                  "TCLScriptExecute returned error %d, on XPS: %s\n", 
                  status, this->portName);
        status = asynError;
      }
    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                "TCL script name has not been set on XPS: %s\n", 
                this->portName);
      status = asynError;
    }

  } else if ((function == XPSPositionCompareMode_) ||
             (function == XPSPositionComparePulseWidth_) ||
             (function == XPSPositionCompareStepSize_)) {
    status = pAxis->setPositionCompare();
    status = pAxis->getPositionCompare();

  } else {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();

  return (asynStatus)status;

}

/** Called when asyn clients call pasynFloat64->write().
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus XPSController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  XPSAxis *pAxis;
  asynStatus status = asynError;
  //static const char *functionName = "writeFloat64";

  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;

  /* Set the parameter and readback in the parameter library. */
  status = pAxis->setDoubleParam(function, value);

  if ((function == XPSPositionCompareMinPosition_) ||
      (function == XPSPositionCompareMaxPosition_) ||
      (function == XPSPositionCompareStepSize_)) {
    status = pAxis->setPositionCompare();
    status = pAxis->getPositionCompare();

  } else {
    /* Call base class method */
    status = asynMotorController::writeFloat64(pasynUser, value);

  }
  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  
  return status;
    
}

/**
 * Perform a deferred move (a coordinated group move) on all the axes in a group.
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

  /* Clear the defer flag for all the axes in this group. */
  /* We need to do this for the XPS, because we cannot do partial group moves. Every axis
     in the group will be included the next time we do a group move. */
  /* Loop over all axes in this controller. */
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    /* Ignore axes in other groups. */
    if (!strcmp(pAxis->groupName_, groupName)) {
      pAxis->deferredMove_ = false;
    }
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
asynStatus XPSController::setDeferredMoves(bool deferMoves)
{
  asynStatus status = asynError;
  int axis = 0;
  int i = 0;
  int dealWith = 0;
  /* Array to cache up to XPS_MAX_AXES group names. Don't initialise to null */
  const char *groupNames[XPS_MAX_AXES];
  const char *blankGroupName = " ";
  XPSAxis *pAxis;
  
  // If we are not ending deferred moves then return
  if (deferMoves || !movesDeferred_) {
    movesDeferred_ = true;
    return asynSuccess;
  }
  
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


/** Returns a pointer to an XPSAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
XPSAxis* XPSController::getAxis(asynUser *pasynUser)
{
    int axisNo;
    
    getAddress(pasynUser, &axisNo);
    return getAxis(axisNo);
}



/** Returns a pointer to an XPSAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
XPSAxis* XPSController::getAxis(int axisNo)
{
    if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
    return pAxes_[axisNo];
}


asynStatus XPSController::waitMotors()
{
  int groupStatus;
  int status;
  char groupName[MAX_GROUPNAME_LEN];
  
  getStringParam(XPSProfileGroupName_, (int)sizeof(groupName), groupName);

  while (1) {
    status = GroupStatusGet(pollSocket_, groupName, &groupStatus);
    if (status) return asynError;
    if (groupStatus >= 10 && groupStatus <= 18) break;
    epicsThreadSleep(0.1);
  }
  return asynSuccess;
}


/* Function to initialize profile */ 
asynStatus XPSController::initializeProfile(size_t maxPoints, const char* ftpUsername, const char* ftpPassword)
{
  ftpUsername_ = epicsStrDup(ftpUsername);
  ftpPassword_ = epicsStrDup(ftpPassword);
  asynMotorController::initializeProfile(maxPoints);
  return asynSuccess;
}



/* Function to build, install and verify trajectory */ 
asynStatus XPSController::buildProfile()
{
  FILE *trajFile;
  char *trajectoryDirectory;
  int i, j; 
  int status;
  bool buildOK=true;
  bool verifyOK=true;
  int numPoints;
  int numElements;
  double trajVel;
  double D0, D1, T0, T1;
  SOCKET ftpSocket;
  char fileName[MAX_FILENAME_LEN];
  char groupName[MAX_GROUPNAME_LEN];
  char message[MAX_MESSAGE_LEN];
  int buildStatus;
  double distance;
  double maxVelocity;
  double maxAcceleration;
  double maxVelocityActual=0.0;
  double maxAccelerationActual=0.0;
  double minPositionActual=0.0, maxPositionActual=0.0;
  double minProfile, maxProfile;
  double lowLimit, highLimit;
  double minJerkTime, maxJerkTime;
  double preTimeMax, postTimeMax;
  double preVelocity[XPS_MAX_AXES], postVelocity[XPS_MAX_AXES];
  bool inGroup[XPS_MAX_AXES];
  double time;
  int useAxis[XPS_MAX_AXES];
  static const char *functionName = "buildProfile";
  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);
            
  // Call the base class method which will build the time array if needed
  asynMotorController::buildProfile();

  strcpy(message, "");
  setStringParam(profileBuildMessage_, message);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
  setIntegerParam(profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();

  /* We create trajectories with an extra element at the beginning and at the end.
   * The distance and time of the first element is defined so that the motors will
   * accelerate from 0 to the velocity of the first "real" element at their 
   * maximum allowed acceleration.
   * Similarly, the distance and time of last element is defined so that the 
   * motors will decelerate from the velocity of the last "real" element to 0 
   * at the maximum allowed acceleration. */

  preTimeMax = 0.;
  postTimeMax = 0.;
  getIntegerParam(profileNumPoints_, &numPoints);
  getStringParam(XPSTrajectoryFile_, (int)sizeof(fileName), fileName);
  getStringParam(XPSProfileGroupName_, (int)sizeof(groupName), groupName);

  /* Zero values since axes may not be used */
  for (j=0; j<numAxes_; j++) {
    preVelocity[j] = 0.;
    postVelocity[j] = 0.;
    getIntegerParam(j, profileUseAxis_, &useAxis[j]);
    inGroup[j] = (strcmp(pAxes_[j]->groupName_, groupName) == 0);
  }
  
  for (j=0; j<numAxes_; j++) {
    if (!useAxis[j] || !inGroup[j]) continue;
    status = PositionerSGammaParametersGet(pollSocket_, pAxes_[j]->positionerName_, 
                                           &maxVelocity, &maxAcceleration,
                                           &minJerkTime, &maxJerkTime);
    if (status) {
      buildOK = false;
      sprintf(message, "Error calling positionerSGammaParametersSet, status=%d\n", status);
      goto done;
    }

    /* The calculation using maxAcceleration read from controller below
     * is "correct" but subject to roundoff errors when sending ASCII commands
     * to XPS.  Reduce acceleration 10% to account for this. */
    maxAcceleration *= 0.9;
    distance = pAxes_[j]->profilePositions_[1] - pAxes_[j]->profilePositions_[0];
    preVelocity[j] = distance/profileTimes_[0];
    time = fabs(preVelocity[j]) / maxAcceleration;
    preTimeMax = MAX(preTimeMax, time);
    distance = pAxes_[j]->profilePositions_[numPoints-1] - 
               pAxes_[j]->profilePositions_[numPoints-2];
    postVelocity[j] = distance/profileTimes_[numPoints-1];
    time = fabs(postVelocity[j]) / maxAcceleration;
    postTimeMax = MAX(postTimeMax, time);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: axis %d profilePositions[0]=%f, profilePositions[%d]=%f, maxAcceleration=%f, preTimeMax=%f, postTimeMax=%f\n",
              driverName, functionName, j, pAxes_[j]->profilePositions_[0], numPoints-1, pAxes_[j]->profilePositions_[numPoints-1],
              maxAcceleration, preTimeMax, postTimeMax);
  }
    
  // preTimeMax and postTimeMax can be very small if the scan velocity is small, because it can accelerate to this velocity
  // almost instantly.  This leads to errors with the XPS reporting acceleration too high, due to roundoff.
  // Fix this by using a minimum time for acceleration.
  preTimeMax = MAX(preTimeMax, XPS_MIN_PROFILE_ACCEL_TIME);
  postTimeMax = MAX(postTimeMax, XPS_MIN_PROFILE_ACCEL_TIME); 
  
  for (j=0; j<numAxes_; j++) {
    pAxes_[j]->profilePreDistance_  =  0.5 * preVelocity[j]  * preTimeMax; 
    pAxes_[j]->profilePostDistance_ =  0.5 * postVelocity[j] * postTimeMax; 
  }

  /* Create the profile file */
  trajFile =  fopen(fileName, "wb");
  if (trajFile == 0) {
    buildOK = false;
    status = -1;
    sprintf(message, "Error creating trajectory file %s, error=%s\n", fileName, strerror(errno));
    goto done;
  }

  /* Create the initial acceleration element */
  fprintf(trajFile,"%f", preTimeMax);
  for (j=0; j<numAxes_; j++) {
    if (!inGroup[j]) continue;
    fprintf(trajFile,", %f, %f", pAxes_[j]->profilePreDistance_, preVelocity[j]);
  }
  fprintf(trajFile,"\n");
 
  /* The number of profile elements in the file is numPoints-1 */
  numElements = numPoints - 1;
  for (i=0; i<numElements; i++) {
    T0 = profileTimes_[i];
    if (i < numElements-1)
      T1 = profileTimes_[i+1];
    else
      T1 = T0;
    fprintf(trajFile,"%f", profileTimes_[i]);
    for (j=0; j<numAxes_; j++) {
      if (!inGroup[j]) continue;
      D0 = pAxes_[j]->profilePositions_[i+1] - 
           pAxes_[j]->profilePositions_[i];
      if (i < numElements-1) 
        D1 = pAxes_[j]->profilePositions_[i+2] - 
             pAxes_[j]->profilePositions_[i+1];
      else
        D1 = D0;
      /* Average either side of the point */
      trajVel = ((D0 + D1) / (T0 + T1));
      if (!useAxis[j]) {
        D0 = 0.0;  /* Axis turned off*/
        trajVel = 0.0;
      }
      fprintf(trajFile,", %f, %f",D0,trajVel);
    }  
    fprintf(trajFile,"\n");
  }

  /* Create the final acceleration element. Final velocity must be 0. */
  fprintf(trajFile,"%f", postTimeMax);
  for (j=0; j<numAxes_; j++) {
    if (!inGroup[j]) continue;
    fprintf(trajFile,", %f, %f", pAxes_[j]->profilePostDistance_, 0.);
  }
  fprintf(trajFile,"\n");
  fclose (trajFile);
  
  /* FTP the trajectory file from the local directory to the XPS */
  status = ftpConnect(IPAddress_, ftpUsername_, ftpPassword_, &ftpSocket);
  if (status) {
    buildOK = false;
    sprintf(message, "Error calling ftpConnect, status=%d\n", status);
    goto done;
  }
  if (strstr(firmwareVersion_, "C8")) {
    trajectoryDirectory = XPS_C8_TRAJECTORY_DIRECTORY;
  } else if (strstr(firmwareVersion_, "Q8")) {
    trajectoryDirectory = XPS_Q8_TRAJECTORY_DIRECTORY;
  } else {
    buildOK = false;
    sprintf(message, "Firmware version does not contain C8 or Q8=%s\n", firmwareVersion_);
    goto done;
  }
  status = ftpChangeDir(ftpSocket, trajectoryDirectory);
  if (status) {
    buildOK = false;
    sprintf(message, "Error calling  ftpChangeDir, status=%d\n", status);
    goto done;
  }
  status = ftpStoreFile(ftpSocket, fileName);
  if (status) {
    buildOK = false;
    sprintf(message, "Error calling  ftpStoreFile, status=%d\n", status);
    goto done;
  }
  status = ftpDisconnect(ftpSocket);
  if (status) {
    buildOK = false;
    sprintf(message, "Error calling  ftpDisconnect, status=%d\n", status);
    goto done;
  }

  /* Verify trajectory */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling MultipleAxesPVTVerification(%d, %s, %s)\n",
            driverName, functionName, pollSocket_, groupName, fileName);
  status = MultipleAxesPVTVerification(pollSocket_, groupName, fileName);
  if (status) verifyOK = false;
  switch (-status) {
    case 0:
      strcpy(message, " ");
      break;
    case 69:
      strcpy(message, "Acceleration Too High");
      break;
    case 68:
      strcpy(message, "Velocity Too High");
      break;
    case 70:
      strcpy(message, "Final Velocity Non Zero");
      break;
    case 75:    
      strcpy(message, "Negative or Null Delta Time");
      break;
    default: 
      sprintf(message, "Unknown trajectory verify error=%d", status);
      break;
  }
  if (!verifyOK) goto done;

  /* Read dynamic parameters*/
  for (j=0; j<numAxes_; j++) {
    if (!inGroup[j]) continue;
    maxVelocityActual = 0;
    maxAccelerationActual = 0;   
    status = MultipleAxesPVTVerificationResultGet(pollSocket_,
                 pAxes_[j]->positionerName_, fileName, 
                 &minPositionActual, &maxPositionActual, 
                 &maxVelocityActual, &maxAccelerationActual);
    pAxes_[j]->setDoubleParam(XPSProfileMinPosition_,     minPositionActual);
    pAxes_[j]->setDoubleParam(XPSProfileMaxPosition_,     maxPositionActual);
    pAxes_[j]->setDoubleParam(XPSProfileMaxVelocity_,     maxVelocityActual);
    pAxes_[j]->setDoubleParam(XPSProfileMaxAcceleration_, maxAccelerationActual);
    if (status) {
      verifyOK = false;
      sprintf(message, "MultipleAxesPVTVerificationResultGet error for axis %s, status=%d\n",
              pAxes_[j]->positionerName_, status);
      goto done;
    }
    // Don't do the rest if the axis is not being used
    if (!useAxis[j]) continue;
    /* Check that the trajectory won't exceed the software limits
     * The XPS does not check this because the trajectory is defined in relative moves and it does
     * not know where we will be in absolute coordinates when we execute the trajectory */
    status = PositionerUserTravelLimitsGet(pollSocket_,
                                           pAxes_[j]->positionerName_,
                                           &lowLimit, 
                                           &highLimit);
    minProfile = pAxes_[j]->profilePositions_[0] + minPositionActual;
    if (minProfile < lowLimit) {
      verifyOK = false;
      sprintf(message, "Low soft limit violation for axis %s, position=%f, limit=%f\n",
              pAxes_[j]->positionerName_, minProfile, lowLimit);
      goto done;
    }
    maxProfile = pAxes_[j]->profilePositions_[0] + maxPositionActual;
    if (maxProfile > highLimit) {
      verifyOK = false;
      sprintf(message, "High soft limit violation for axis %s, position=%f, limit=%f\n",
              pAxes_[j]->positionerName_, maxProfile, highLimit);
      goto done;
    }
  }
  done:
  buildStatus = (buildOK && verifyOK) ?  PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
  setIntegerParam(profileBuildStatus_, buildStatus);
  setStringParam(profileBuildMessage_, message);
  if (buildStatus != PROFILE_STATUS_SUCCESS) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  /* Clear build command.  This is a "busy" record, don't want to do this until build is complete. */
  setIntegerParam(profileBuild_, 0);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
  callParamCallbacks();
  return status ? asynError : asynSuccess; 
}

/* Function to execute trajectory */ 
asynStatus XPSController::executeProfile()
{
  epicsEventSignal(profileExecuteEvent_);
  return asynSuccess;
}

/* C Function which runs the profile thread */ 
static void XPSProfileThreadC(void *pPvt)
{
  XPSController *pC = (XPSController*)pPvt;
  pC->profileThread();
}


/* Function which runs in its own thread to execute profiles */ 
void XPSController::profileThread()
{
  while (true) {
    epicsEventWait(profileExecuteEvent_);
    runProfile();
  }
}

/* Function to run trajectory.  It runs in a dedicated thread, so it's OK to block.
 * It needs to lock and unlock when it accesses class data. */ 
asynStatus XPSController::runProfile()
{
  int status;
  bool executeOK=true;
  bool aborted=false;
  int j;
  int startPulses, endPulses;
  int lastTime;
  int numPoints, numElements, numPulses;
  int executeStatus;
  double pulsePeriod;
  double position;
  double time;
  int i;
  int moveMode;
  char message[MAX_MESSAGE_LEN];
  char buffer[MAX_GATHERING_STRING];
  char fileName[MAX_FILENAME_LEN];
  char groupName[MAX_GROUPNAME_LEN];
  int eventId;
  int useAxis[XPS_MAX_AXES];
  bool inGroup[XPS_MAX_AXES];
  XPSAxis *pAxis;
  static const char *functionName = "runProfile";
  
  lock();
  getStringParam(XPSTrajectoryFile_,   (int)sizeof(fileName), fileName);
  getStringParam(XPSProfileGroupName_, (int)sizeof(groupName), groupName);
  getIntegerParam(profileStartPulses_, &startPulses);
  getIntegerParam(profileEndPulses_,   &endPulses);
  getIntegerParam(profileNumPoints_,   &numPoints);
  getIntegerParam(profileNumPulses_,   &numPulses);
  for (j=0; j<numAxes_; j++) {
    getIntegerParam(j, profileUseAxis_, &useAxis[j]);
    inGroup[j] = (strcmp(pAxes_[j]->groupName_, groupName) == 0);
  }
  strcpy(message, " ");
  setStringParam(profileExecuteMessage_, message);
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_MOVE_START);
  setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();
  unlock();

  // Move the motors to the start position
  // This depends on whether we are in absolute or relative mode
  getIntegerParam(profileMoveMode_, &moveMode);
  for (j=0; j<numAxes_; j++) {
    if (!useAxis[j] || !inGroup[j]) continue;
    pAxis = getAxis(j);
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE) {
      position = pAxis->profilePositions_[0] - pAxis->profilePreDistance_;
      status = GroupMoveAbsolute(pAxis->moveSocket_,
                                 pAxis->positionerName_,
                                 1,
                                 &position);
    } else {
      position = -pAxis->profilePreDistance_;
      status = GroupMoveRelative(pAxis->moveSocket_,
                                 pAxis->positionerName_,
                                 1,
                                 &position);
    }
  }

  // Wait for the motors to get there
  wakeupPoller();
  waitMotors();

  lock();
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_EXECUTING);
  callParamCallbacks();
  unlock();
  
  /* Configure Gathering */
  /* Reset gathering.  
   * This must be done because GatheringOneData just appends to in-memory list */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling GatheringReset(%d)\n", 
            driverName, functionName, pollSocket_);
  status = GatheringReset(pollSocket_);
  if (status != 0) {
    executeOK = false;
    sprintf(message, "Error performing GatheringReset, status=%d",status);
    goto done;
  }

  /* Write list of gathering parameters.
   * Note that there must be NUM_GATHERING_ITEMS per axis in this list. */
  strcpy(buffer, "");
  for (j=0; j<numAxes_; j++) {
    strcat (buffer, pAxes_[j]->positionerName_);
    strcat (buffer, ".SetpointPosition;");
    strcat (buffer, pAxes_[j]->positionerName_);
    strcat (buffer, ".CurrentPosition;");
  }
  
  /* Define what is to be saved in the GatheringExternal.dat.  
   * 3 pieces of information per axis. */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling GatheringConfigurationSet(%d, %d, %s)\n", 
            driverName, functionName, pollSocket_, numAxes_*NUM_GATHERING_ITEMS, buffer);
  status = GatheringConfigurationSet(pollSocket_, 
                                     numAxes_*NUM_GATHERING_ITEMS, buffer);
  if (status != 0) {
    executeOK = false;
    sprintf(message, "Error performing GatheringConfigurationSet, status=%d, buffer=%s", 
            status, buffer);
    goto done;
  }
  
  /* The number of trajectory elements for the user is numPoints-1 
     The actual number is 2 greater because of the acceleration and decelleration */
  numElements = numPoints - 1;

  // Check valid range of start and end pulses;  these start at 1, not 0.
  // numElements+1 is the decelleration element
  if ((startPulses < 1)           || (startPulses > numElements+1) ||
      (endPulses   < startPulses) || (endPulses   > numElements+1)) {
    executeOK = false;
    sprintf(message, "Error: start or end pulses outside valid range");
    goto done;
  }
  // The XPS can only output pulses at a fixed period, not a fixed distance along the trajectory.  
  // The trajectory elements where pulses start and stop are defined by startPulses and endPulses.
  // Compute the time between pulses as the total time over which pulses should be output divided 
  //  by the number of pulses to be output. */
  time = 0;
  lastTime = endPulses;
  if (endPulses > numElements) lastTime = numElements;
  for (i=startPulses; i<=lastTime; i++) {
    time += profileTimes_[i-1];
  }
  // We put out pulses starting at the beginning of element startPulses and ending at the end of element
  // endPulses.
  if (numPulses != 0)
    pulsePeriod = time / numPulses;
  else
    pulsePeriod = 0;
  
  /* Define trajectory output pulses. 
   * startPulses and endPulses are defined as 1=first real element, need to add
   * 1 to each to skip the acceleration element.  
   * The XPS is told the element to stop outputting pulses, and it seems to stop
   * outputting at the end of that element.  So we need to have that element be
   * the decceleration element, which means adding another +1, or we come up 1 pulse short.
   * But this means we will almost always get too many pulses */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling MultipleAxesPVTPulseOutputSet(%d, %s, %d, %d, %f)\n", 
            driverName, functionName, pollSocket_, groupName,
            startPulses+1, endPulses+1, pulsePeriod);
  status = MultipleAxesPVTPulseOutputSet(pollSocket_, groupName,
                                         startPulses+1, 
                                         endPulses+1, 
                                         pulsePeriod);

  /* Define trigger */
  sprintf(buffer, "Always;%s.PVT.TrajectoryPulse", groupName);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling EventExtendedConfigurationTriggerSet(%d, %d, %s, %s, %s, %s. %s)\n", 
            driverName, functionName, pollSocket_, 2, buffer, "", "", "", "");
  status = EventExtendedConfigurationTriggerSet(pollSocket_, 2, buffer, 
                                                "", "", "", "");
  if (status != 0) {
    executeOK = false;
    sprintf(message, "Error performing EventExtendedConfigurationTriggerSet, status=%d, buffer=%s, strlen(buffer)=%ld", 
            status, buffer, (long)strlen(buffer));
    goto done;
  }

  /* Define action */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling EventExtendedConfigurationActionSet(%d, %d, %s, %s, %s, %s, %s)\n", 
            driverName, functionName, pollSocket_, 1, "GatheringOneData", "", "", "", "");
  status = EventExtendedConfigurationActionSet(pollSocket_, 1, 
                                               "GatheringOneData", 
                                               "", "", "", "");
  if (status != 0) {
    executeOK = false;
    sprintf(message, "Error performing EventExtendedConfigurationActionSet, status=%d", 
            status);
    goto done;
  }

  /* Start gathering */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling EventExtendedStart(%d, %p)\n", 
            driverName, functionName, pollSocket_, &eventId);
  status= EventExtendedStart(pollSocket_, &eventId);
  if (status != 0) {
    executeOK = false;
    sprintf(message, "Error performing EventExtendedStart, status=%d", 
            status);
    goto done;
  }

  wakeupPoller();
  
  /* We call the command to run the trajectory on the moveSocket which does not
   * wait for a reply.  Thus this routine returns immediately without a meaningful
   * status */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling MultipleAxesPVTExecution(%d, %s, %s, %d)\n", 
            driverName, functionName, moveSocket_, groupName, fileName, 1);
  status = MultipleAxesPVTExecution(moveSocket_, groupName,
                                    fileName, 1);
  /* status -27 means the trajectory was aborted */
  if (status == -27) {
    executeOK = false;
    aborted = true;
    sprintf(message, "MultipleAxesPVTExecution aborted");
  }
  else if (status != 0) {
    executeOK = false;
    sprintf(message, "Error performing MultipleAxesPVTExecution, status=%d", 
            status);
  }

  /* Remove the event */
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling EventExtendedRemove(%d, %d)\n", 
            driverName, functionName, pollSocket_, eventId);
  status = EventExtendedRemove(pollSocket_, eventId);
  if (status != 0) {
    executeOK = false;
    sprintf(message, "Error performing ExtendedEventRemove, status=%d", 
            status);
  }
    
  /* Stop the gathering */  
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: calling GatheringStop(%d)\n", 
            driverName, functionName, pollSocket_);
  status = GatheringStop(pollSocket_);
  /* status -30 means gathering not started i.e. aborted before the end of
     1 trajectory element */
  if ((status != 0) && (status != -30)) {
    executeOK = false;
    sprintf(message, "Error performing GatheringStop, status=%d", 
            status);
  }
  
  done:
  lock();
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_FLYBACK);
  callParamCallbacks();
  unlock();

  // Move the motors to the end position
  for (j=0; j<numAxes_; j++) {
    if (!useAxis[j] ||!inGroup[j]) continue;
    pAxis = getAxis(j);
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE) {
      position = pAxis->profilePositions_[numPoints-1];
      status = GroupMoveAbsolute(pAxis->moveSocket_,
                                 pAxis->positionerName_,
                                 1,
                                 &position); 
    } else {
      position = -pAxis->profilePostDistance_;
      status = GroupMoveRelative(pAxis->moveSocket_,
                                 pAxis->positionerName_,
                                 1,
                                 &position); 
    }  
  }
  
  // Wait for the motors to get there
  wakeupPoller();
  waitMotors();

  lock();
  if (executeOK)    executeStatus = PROFILE_STATUS_SUCCESS;
  else if (aborted) executeStatus = PROFILE_STATUS_ABORT;
  else              executeStatus = PROFILE_STATUS_FAILURE;
  setIntegerParam(profileExecuteStatus_, executeStatus);
  setStringParam(profileExecuteMessage_, message);
  if (!executeOK) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  /* Clear execute command.  This is a "busy" record, don't want to do this until build is complete. */
  setIntegerParam(profileExecute_, 0);
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);
  callParamCallbacks();
  unlock();
  return executeOK ? asynSuccess : asynError; 
}

/** Polls the controller, rather than individual axis
  * Used during profile moves */
asynStatus XPSController::poll()
{
  int executeState;
  int status;
  int number;
  char fileName[MAX_FILENAME_LEN];
  char groupName[MAX_GROUPNAME_LEN];
  
  getIntegerParam(profileExecuteState_, &executeState);
  if (executeState != PROFILE_EXECUTE_EXECUTING) return asynSuccess;

  getStringParam(XPSTrajectoryFile_, (int)sizeof(fileName), fileName);
  getStringParam(XPSProfileGroupName_, (int)sizeof(groupName), groupName);
  status = MultipleAxesPVTParametersGet(pollSocket_, groupName, fileName, &number);
  if (status) return asynError;
  setIntegerParam(profileCurrentPoint_, number);
  callParamCallbacks();
  return asynSuccess;
}



asynStatus XPSController::abortProfile()
{
  int status;
  char groupName[MAX_GROUPNAME_LEN];
  static const char *functionName = "abortProfile";
  
  getStringParam(XPSProfileGroupName_, (int)sizeof(groupName), groupName);
  status = GroupMoveAbort(pollSocket_, groupName);
  if (status != 0) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Error performing GroupMoveAbort, status=%d\n",
              driverName, functionName, status);
    return asynError;
  }
  return asynSuccess;
}
       


/* Function to readback trajectory */ 
asynStatus XPSController::readbackProfile()
{
  char message[MAX_MESSAGE_LEN];
  bool readbackOK=true;
  int numPulses;
  char* buffer=NULL;
  char* bptr, *tptr;
  int currentSamples, maxSamples;
  double setpointPosition, actualPosition;
  int readbackStatus;
  int status;
  int i, j;
  int nitems;
  int numRead=0, numInBuffer, numChars;
  static const char *functionName = "readbackProfile";
    
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);

  strcpy(message, "");
  setStringParam(profileReadbackMessage_, message);
  setIntegerParam(profileReadbackState_, PROFILE_READBACK_BUSY);
  setIntegerParam(profileReadbackStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();
  
  status = getIntegerParam(profileNumPulses_, &numPulses);

  /* Erase the readback and error arrays */
  for (j=0; j<numAxes_; j++) {
    memset(pAxes_[j]->profileReadbacks_,       0, maxProfilePoints_*sizeof(double));
    memset(pAxes_[j]->profileFollowingErrors_, 0, maxProfilePoints_*sizeof(double));
  }
  /* Read the number of lines of gathering */
  status = GatheringCurrentNumberGet(pollSocket_, &currentSamples, &maxSamples);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s: GatheringCurrentNumberGet, status=%d, currentSamples=%d, maxSamples=%d\n", 
            driverName, functionName, status, currentSamples, maxSamples);
  if (status != 0) {
    readbackOK = false;
    sprintf(message, "Error calling GatherCurrentNumberGet, status=%d", status);
    goto done;
  }
  if (currentSamples < numPulses) {
    readbackOK = false;
    sprintf(message, "Error, numPulses=%d, currentSamples=%d", numPulses, currentSamples);
    //goto done;
  } 
  if (currentSamples > (int) maxProfilePoints_) {
      currentSamples = maxProfilePoints_;
  }
  buffer = (char *)calloc(GATHERING_MAX_READ_LEN, sizeof(char));
  numInBuffer = 0;
  for (numRead=0; numRead<currentSamples;) {
    /* Read the next buffer */
    /* Try to read all the remaining points */
    status = -1;
    numInBuffer = currentSamples - numRead;
    while (status && (numInBuffer > 0)) {
      status = GatheringDataMultipleLinesGet(pollSocket_, numRead, numInBuffer, buffer);
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "%s:%s: GatheringDataMultipleLinesGet, status=%d, numInBuffer=%d\n", 
                driverName, functionName, status, numInBuffer);
      if (status) numInBuffer /= 2;
    }
    if (numInBuffer == 0) {
      readbackOK = false;
      sprintf(message, "Error reading gathering data, numInBuffer = 0");
      goto done;
    }
    bptr = buffer;
    for (i=0; i<numInBuffer; i++) {
      /* Find the next \n and replace with null */
      tptr = strstr(bptr, "\n");
      if (tptr) *tptr = 0;
      for (j=0; j<numAxes_; j++) {
        nitems = sscanf(bptr, "%lf;%lf%n", 
                        &setpointPosition, &actualPosition, &numChars);
        bptr += numChars+1;
        if (nitems != NUM_GATHERING_ITEMS) {
          readbackOK = false;
          sprintf(message, "Error reading Gathering.dat file, nitems=%d, should be %d",
                  nitems, NUM_GATHERING_ITEMS);
          goto done;
        }
        // Note, these positions are in controller units, need to be converted to user units
        pAxes_[j]->profileFollowingErrors_[numRead] = actualPosition - setpointPosition;
        pAxes_[j]->profileReadbacks_[numRead] = actualPosition;
      }
      numRead++;
      bptr = tptr + 1;
    }
  }
  
  done:
  if (buffer) free(buffer);
  setIntegerParam(profileActualPulses_, numRead);
  setIntegerParam(profileNumReadbacks_, numRead);
  /* Convert from controller to user units and post the arrays */
  for (j=0; j<numAxes_; j++) {
    pAxes_[j]->readbackProfile();
  }
  readbackStatus = readbackOK ?  PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
  setIntegerParam(profileReadbackStatus_, readbackStatus);
  setStringParam(profileReadbackMessage_, message);
  if (!readbackOK) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, message);
  }
  /* Clear readback command.  This is a "busy" record, don't want to do this until readback is complete. */
  setIntegerParam(profileReadback_, 0);
  setIntegerParam(profileReadbackState_, PROFILE_READBACK_DONE);
  callParamCallbacks();
  return status ? asynError : asynSuccess; 
}

/* Function to disable the automatic enable of axes when moving */ 
asynStatus XPSController::disableAutoEnable()
{
  autoEnable_ = 0;
  return asynSuccess; 
}

/* Function to disable the MSTA problem bit in the event of an XPS Disable state 20 */ 
asynStatus XPSController::noDisableError()
{
  noDisableError_ = 1;
  return asynSuccess; 
}

/* Function to enable a mode where the XPSAxis poller will check the moveSocket response 
   to determine motion done. It does not rely on the axis state in this case. This prevents
   axes in multipleAxis groups being in 'moving state' when another axis in the same
   group is moving. This allows the motor record to move the axis when another axis in the
   same group is moving. However, this has the consequence that a moving state is not 
   detected if the move is externally generated. So by default this mode is turned off. It
   can be enabled by calling this function. */ 
asynStatus XPSController::enableMovingMode()
{
  enableMovingMode_ = true;
  return asynSuccess; 
}



/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

asynStatus XPSCreateController(const char *portName, const char *IPAddress, int IPPort,
                               int numAxes, int movingPollPeriod, int idlePollPeriod,
                               int enableSetPosition, int setPositionSettlingTime)
{
    new XPSController(portName, IPAddress, IPPort, numAxes, 
                      movingPollPeriod/1000., idlePollPeriod/1000.,
                      enableSetPosition, setPositionSettlingTime/1000.);
    return asynSuccess;
}



asynStatus XPSCreateAxis(const char *XPSName,         /* specify which controller by port name */
                         int axis,                    /* axis number 0-7 */
                         const char *positionerName,  /* groupName.positionerName e.g. Diffractometer.Phi */
                         const char *stepsPerUnit)    /* steps per user unit */
{
  XPSController *pC;
  double stepSize;
  static const char *functionName = "XPSCreateAxis";

  pC = (XPSController*) findAsynPortDriver(XPSName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, XPSName);
    return asynError;
  }
  errno = 0;
  stepSize = strtod(stepsPerUnit, NULL);
  if (errno != 0) {
    printf("%s:%s: Error invalid steps per unit=%s\n",
           driverName, functionName, stepsPerUnit);
    return asynError;
  }
  
  pC->lock();
  new XPSAxis(pC, axis, positionerName, 1./stepSize);
  pC->unlock();
  return asynSuccess;
}


asynStatus XPSCreateProfile(const char *XPSName,         /* specify which controller by port name */
                            int maxPoints,               /* maximum number of profile points */
                            const char *ftpUsername,      /* FTP account name */
                            const char *ftpPassword)     /* FTP password */
{
  XPSController *pC;
  static const char *functionName = "XPSCreateProfile";

  pC = (XPSController*) findAsynPortDriver(XPSName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, XPSName);
    return asynError;
  }
  pC->lock();
  pC->initializeProfile(maxPoints, ftpUsername, ftpPassword);
  pC->unlock();
  return asynSuccess;
}


asynStatus XPSDisableAutoEnable(const char *XPSName)
{
  XPSController *pC;
  static const char *functionName = "XPSDisableAutoEnable";

  pC = (XPSController*) findAsynPortDriver(XPSName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, XPSName);
    return asynError;
  }

  return pC->disableAutoEnable();
}


asynStatus XPSNoDisableError(const char *XPSName)
{
  XPSController *pC;
  static const char *functionName = "XPSNoDisableError";

  pC = (XPSController*) findAsynPortDriver(XPSName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, XPSName);
    return asynError;
  }

  return pC->noDisableError();
}

asynStatus XPSEnableMovingMode(const char *XPSName)
{
  XPSController *pC;
  static const char *functionName = "XPSEnableMovingMode";

  pC = (XPSController*) findAsynPortDriver(XPSName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, XPSName);
    return asynError;
  }

  return pC->enableMovingMode();
}




/* Code for iocsh registration */

/* XPSCreateController */
static const iocshArg XPSCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg XPSCreateControllerArg1 = {"IP address", iocshArgString};
static const iocshArg XPSCreateControllerArg2 = {"IP port", iocshArgInt};
static const iocshArg XPSCreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg XPSCreateControllerArg4 = {"Moving poll rate (ms)", iocshArgInt};
static const iocshArg XPSCreateControllerArg5 = {"Idle poll rate (ms)", iocshArgInt};
static const iocshArg XPSCreateControllerArg6 = {"Enable set position", iocshArgInt};
static const iocshArg XPSCreateControllerArg7 = {"Set position settling time (ms)", iocshArgInt};
static const iocshArg * const XPSCreateControllerArgs[] = {&XPSCreateControllerArg0,
                                                           &XPSCreateControllerArg1,
                                                           &XPSCreateControllerArg2,
                                                           &XPSCreateControllerArg3,
                                                           &XPSCreateControllerArg4,
                                                           &XPSCreateControllerArg5,
                                                           &XPSCreateControllerArg6,
                                                           &XPSCreateControllerArg7};
static const iocshFuncDef configXPS = {"XPSCreateController", 8, XPSCreateControllerArgs};
static void configXPSCallFunc(const iocshArgBuf *args)
{
  XPSCreateController(args[0].sval, args[1].sval, args[2].ival, 
                      args[3].ival, args[4].ival, args[5].ival,
                      args[6].ival, args[7].ival);
}



/* XPSCreateAxis */
static const iocshArg XPSCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg XPSCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg XPSCreateAxisArg2 = {"Axis name", iocshArgString};
static const iocshArg XPSCreateAxisArg3 = {"Steps per unit", iocshArgString};
static const iocshArg * const XPSCreateAxisArgs[] = {&XPSCreateAxisArg0,
                                                     &XPSCreateAxisArg1,
                                                     &XPSCreateAxisArg2,
                                                     &XPSCreateAxisArg3};
static const iocshFuncDef configXPSAxis = {"XPSCreateAxis", 4, XPSCreateAxisArgs};

static void configXPSAxisCallFunc(const iocshArgBuf *args)
{
  XPSCreateAxis(args[0].sval, args[1].ival, args[2].sval, args[3].sval);
}



/* XPSCreateProfile */
static const iocshArg XPSCreateProfileArg0 = {"Controller port name", iocshArgString};
static const iocshArg XPSCreateProfileArg1 = {"Max points", iocshArgInt};
static const iocshArg XPSCreateProfileArg2 = {"FTP username", iocshArgString};
static const iocshArg XPSCreateProfileArg3 = {"FTP password", iocshArgString};
static const iocshArg * const XPSCreateProfileArgs[] = {&XPSCreateProfileArg0,
                                                        &XPSCreateProfileArg1,
                                                        &XPSCreateProfileArg2,
                                                        &XPSCreateProfileArg3};
static const iocshFuncDef configXPSProfile = {"XPSCreateProfile", 4, XPSCreateProfileArgs};

static void configXPSProfileCallFunc(const iocshArgBuf *args)
{
  XPSCreateProfile(args[0].sval, args[1].ival, args[2].sval, args[3].sval);
}


/* XPSDisableAutoEnable */
static const iocshArg XPSDisableAutoEnableArg0 = {"Controller port name", iocshArgString};
static const iocshArg * const XPSDisableAutoEnableArgs[] = {&XPSDisableAutoEnableArg0};
static const iocshFuncDef disableAutoEnable = {"XPSDisableAutoEnable", 1, XPSDisableAutoEnableArgs};

static void disableAutoEnableCallFunc(const iocshArgBuf *args)
{
  XPSDisableAutoEnable(args[0].sval);
}

/* XPSNoDisableError */
static const iocshArg XPSNoDisableErrorArg0 = {"Controller port name", iocshArgString};
static const iocshArg * const XPSNoDisableErrorArgs[] = {&XPSNoDisableErrorArg0};
static const iocshFuncDef noDisableError = {"XPSNoDisableError", 1, XPSNoDisableErrorArgs};

static void noDisableErrorCallFunc(const iocshArgBuf *args)
{
  XPSNoDisableError(args[0].sval);
}

/* XPSEnableMovingMode */
static const iocshArg XPSEnableMovingModeArg0 = {"Controller port name", iocshArgString};
static const iocshArg * const XPSEnableMovingModeArgs[] = {&XPSEnableMovingModeArg0};
static const iocshFuncDef enableMovingMode = {"XPSEnableMovingMode", 1, XPSEnableMovingModeArgs};

static void enableMovingModeCallFunc(const iocshArgBuf *args)
{
  XPSEnableMovingMode(args[0].sval);
}


static void XPSRegister3(void)
{
  iocshRegister(&configXPS,            configXPSCallFunc);
  iocshRegister(&configXPSAxis,        configXPSAxisCallFunc);
  iocshRegister(&configXPSProfile,     configXPSProfileCallFunc);
  iocshRegister(&disableAutoEnable,    disableAutoEnableCallFunc);
  iocshRegister(&noDisableError,       noDisableErrorCallFunc);
  iocshRegister(&enableMovingMode,     enableMovingModeCallFunc);
}
epicsExportRegistrar(XPSRegister3);

} // extern "C"
