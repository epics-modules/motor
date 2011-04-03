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

// drvInfo strings for extra parameters that the XPS controller supports
#define XPSMinJerkString    "XPS_MIN_JERK"
#define XPSMaxJerkString    "XPS_MAX_JERK"
#define XPSStatusString     "XPS_STATUS"

class XPSController : public asynMotorController {
public:
  XPSController(const char *portName, const char *IPAddress, int IPPort,
                int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  XPSAxis* getAxis(asynUser *pasynUser);
  XPSAxis* getAxis(int axisNo);

  /* These are the functions for profile moves */
  asynStatus buildProfile();
  asynStatus executeProfile();
  asynStatus readbackProfile();

  /* These are the methods that are new to this class */
  /* Deferred moves functions.*/
  asynStatus processDeferredMoves();
  asynStatus processDeferredMovesInGroup(char * groupName);


protected:
  XPSAxis **pAxes_;       /**< Array of pointers to axis objects */

  int XPSMinJerk_;
#define FIRST_XPS_PARAM XPSMinJerk_
  int XPSMaxJerk_;
  int XPSStatus_;
#define LAST_XPS_PARAM XPSStatus_

#define NUM_XPS_PARAMS (&LAST_XPS_PARAM - &FIRST_XPS_PARAM + 1)

private:
  char *IPAddress_;
  int IPPort_;
  char *ftpUsername_;
  char *ftpPassword_;
  char *profileGroupName_;
  int pollSocket_;
  int moveSocket_;
  char firmwareVersion_[100];
  int movesDeferred_;
    
friend class XPSAxis;
};
#endif /* XPSController_H */

