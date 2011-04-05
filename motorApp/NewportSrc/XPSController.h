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
#define TCP_TIMEOUT 2.0

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

  /* These are the functions for profile moves */
  asynStatus initializeProfile(size_t maxPoints, const char* ftpUsername, const char* ftpPassword);
  asynStatus buildProfile();
  asynStatus executeProfile();
  asynStatus readbackProfile();

  /* These are the methods that are new to this class */
  /* Deferred moves functions.*/
  asynStatus processDeferredMoves();
  asynStatus processDeferredMovesInGroup(char * groupName);
  asynStatus enableSetPosition(int enable);
  asynStatus setPositionSettlingTime(double sleepTime);

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
  int enableSetPosition_; /**< This is controlled via the XPSEnableSetPosition function (available via the IOC shell). */ 

  /** Parameter to control the sleep time used when setting position. 
   *  A function called XPSSetPositionSettlingTime(XPS, int) (millisec parameter) 
   *  is available in the IOC shell to control this. */
  double setPositionSettlingTime_;
  char *IPAddress_;
  int IPPort_;
  char *ftpUsername_;
  char *ftpPassword_;
  int pollSocket_;
  int moveSocket_;
  char firmwareVersion_[100];
  int movesDeferred_;
    
  friend class XPSAxis;
};
#define NUM_XPS_PARAMS (&LAST_XPS_PARAM - &FIRST_XPS_PARAM + 1)
#endif /* XPSController_H */

