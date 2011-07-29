/*
FILENAME...     XPSMotorDriver.cpp
USAGE...        Newport XPS EPICS asyn motor device driver
*/
#ifndef XPSMotorAxis_H
#define XPSMotorAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"

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

class XPSController;

class XPSAxis : public asynMotorAxis
{
  public:
  /* These are the methods we override from the base class */
  XPSAxis(XPSController *pController, int axisNo, const char *positionerName, double stepSize);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus doMoveToHome();
  asynStatus setPosition(double position);

  virtual asynStatus defineProfile(double *positions, size_t numPoints);
  virtual asynStatus readbackProfile();
  
  private:
  XPSController *pC_;
  char *getXPSError(int status, char *buffer);
  int isInGroup();
  asynStatus setPID(const double * value, int pidoption);
  asynStatus getPID();
  asynStatus setPIDValue(const double * value, int pidoption); 

  /* Wrapper functions for the verbose PositionerCorrector functions. */
  asynStatus PositionerCorrectorPIPositionGet();
  asynStatus PositionerCorrectorPIDFFVelocityGet();
  asynStatus PositionerCorrectorPIDFFAccelerationGet();
  asynStatus PositionerCorrectorPIDDualFFVoltageGet();
  asynStatus PositionerCorrectorPIPositionSet();
  asynStatus PositionerCorrectorPIDFFVelocitySet();
  asynStatus PositionerCorrectorPIDFFAccelerationSet();
  asynStatus PositionerCorrectorPIDDualFFVoltageSet();

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
  double profilePreDistance_;
  double profilePostDistance_;
  xpsCorrectorInfo_t xpsCorrectorInfo_;
  double deferredPosition_;
  int deferredMove_;
  int deferredRelative_;

  friend class XPSController;
};

#endif /* XPSMotorAxis_H */
