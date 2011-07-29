/*
FILENAME...     XPSMotorDriver.cpp
USAGE...        Newport XPS EPICS asyn motor device driver
*/

#ifndef XPSController_H
#define XPSController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "XPSAxis.h"

#define XPS_MAX_AXES 8
#define XPS_POLL_TIMEOUT 2.0
#define XPS_MOVE_TIMEOUT 100000.0 // "Forever"

// drvInfo strings for extra parameters that the XPS controller supports
#define XPSMinJerkString                "XPS_MIN_JERK"
#define XPSMaxJerkString                "XPS_MAX_JERK"
#define XPSProfileMaxVelocityString     "XPS_PROFILE_MAX_VELOCITY"
#define XPSProfileMaxAccelerationString "XPS_PROFILE_MAX_ACCELERATION"
#define XPSProfileMinPositionString     "XPS_PROFILE_MIN_POSITION"
#define XPSProfileMaxPositionString     "XPS_PROFILE_MAX_POSITION"
#define XPSProfileGroupNameString       "XPS_PROFILE_GROUP_NAME"
#define XPSTrajectoryFileString         "XPS_TRAJECTORY_FILE"
#define XPSStatusString                 "XPS_STATUS"

class XPSController : public asynMotorController {

  public:
  XPSController(const char *portName, const char *IPAddress, int IPPort,
                int numAxes, double movingPollPeriod, double idlePollPeriod,
                int enableSetPosition, double setPositionSettlingTime);

  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  XPSAxis* getAxis(asynUser *pasynUser);
  XPSAxis* getAxis(int axisNo);
  asynStatus poll();

  /* These are the functions for profile moves */
  asynStatus initializeProfile(size_t maxPoints, const char* ftpUsername, const char* ftpPassword);
  asynStatus buildProfile();
  asynStatus executeProfile();
  asynStatus abortProfile();
  asynStatus readbackProfile();

  /* These are the methods that are new to this class */
  void profileThread();
  asynStatus runProfile();
  asynStatus waitMotors();
  /* Deferred moves functions.*/
  asynStatus processDeferredMoves();
  asynStatus processDeferredMovesInGroup(char * groupName);

  /*Disable automatic enable of axes.*/
  asynStatus disableAutoEnable();

  protected:
  XPSAxis **pAxes_;       /**< Array of pointers to axis objects */

  #define FIRST_XPS_PARAM XPSMinJerk_
  int XPSMinJerk_;
  int XPSMaxJerk_;
  int XPSProfileMaxVelocity_;
  int XPSProfileMaxAcceleration_;
  int XPSProfileMinPosition_;
  int XPSProfileMaxPosition_;
  int XPSProfileGroupName_;
  int XPSTrajectoryFile_;
  int XPSStatus_;
  #define LAST_XPS_PARAM XPSStatus_

  private:
  bool enableSetPosition_;          /**< Enable/disable setting the position from EPICS */ 
  double setPositionSettlingTime_;  /**< The settling (sleep) time used when setting position. */
  char *IPAddress_;
  int IPPort_;
  char *ftpUsername_;
  char *ftpPassword_;
  int pollSocket_;
  int moveSocket_;
  char firmwareVersion_[100];
  int movesDeferred_;
  epicsEventId profileExecuteEvent_;
  int autoEnable_;
  
  friend class XPSAxis;
};
#define NUM_XPS_PARAMS (&LAST_XPS_PARAM - &FIRST_XPS_PARAM + 1)
#endif /* XPSController_H */

