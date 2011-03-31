/*
FILENAME...     XPSMotorDriver.cpp
USAGE...        Newport XPS EPICS asyn motor device driver
*/

#include "asynMotorDriver.h"

// drvInfo strings for extra parameters that the XPS controller supports
#define XPSMinJerkString    "XPS_MIN_JERK"
#define XPSMaxJerkString    "XPS_MAX_JERK"
#define XPSStatusString     "XPS_STATUS"

/** Struct that contains information about the XPS corrector loop.*/ 
typedef struct
{
  bool ClosedLoopStatus;
  double KP; /**< Main proportional term from PID loop.*/
  double KI; /**< Main integral term from PID loop.*/
  double KD; /**< Main differential term from PID loop.*/
  double KS;
  double IntegrationTime;
  double DerivativeFilterCutOffFrequency;
  double GKP;
  double GKI;
  double GKD;
  double KForm;
  double FeedForwardGainVelocity;
  double FeedForwardGainAcceleration;
  double Friction;
} xpsCorrectorInfo_t;

class XPSAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  XPSAxis(class XPSController *pController, int axisNo, const char *positionerName, double stepSize);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(int *moving);
  asynStatus setPosition(double position);
  
private:
  XPSController *pC_;
  char *getXPSError(int status, char *buffer);
  int isInGroup();
  asynStatus setPID(const double * value, int pidoption);
  asynStatus getPID();
  asynStatus setPIDValue(const double * value, int pidoption); 

  /* Wrapper functions for the verbose PositionerCorrector functions. */
  asynStatus PositionerCorrectorPIPositionGetWrapper();
  asynStatus PositionerCorrectorPIDFFVelocityGetWrapper();
  asynStatus PositionerCorrectorPIDFFAccelerationGetWrapper();
  asynStatus PositionerCorrectorPIDDualFFVoltageGetWrapper();
  asynStatus PositionerCorrectorPIPositionSetWrapper();
  asynStatus PositionerCorrectorPIDFFVelocitySetWrapper();
  asynStatus PositionerCorrectorPIDFFAccelerationSetWrapper();
  asynStatus PositionerCorrectorPIDDualFFVoltageSetWrapper();

  int pollSocket_;
  int moveSocket_;
  double setpointPosition_;
  double encoderPosition_;
  double currentVelocity_;
  double velocity_;
  double accel_;
  double highLimit_;
  double lowLimit_;
  double stepSize_;
  char *positionerName_;
  char *groupName_;
  int positionerError_;
  int axisStatus_;
  xpsCorrectorInfo_t xpsCorrectorInfo_;
  double deferredPosition_;
  int deferredMove_;
  int deferredRelative_;

friend class XPSController;
};


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

  /* These are the methods that are new to this class */
  /* Deferred moves functions.*/
  asynStatus processDeferredMoves();
  asynStatus processDeferredMovesInGroup(char * groupName);

protected:
  int XPSMinJerk_;
#define FIRST_XPS_PARAM XPSMinJerk_
  int XPSMaxJerk_;
  int XPSStatus_;
#define LAST_XPS_PARAM XPSStatus_

#define NUM_XPS_PARAMS (&LAST_XPS_PARAM - &FIRST_XPS_PARAM + 1)

private:
  char *IPAddress_;
  int IPPort_;
  int pollSocket_;
  char firmwareVersion_[100];
  int movesDeferred_;
    
friend class XPSAxis;
};

