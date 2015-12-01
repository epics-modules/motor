/*
FILENAME...   874xMotorDriver.h
USAGE...      Motor driver support for the NewFocus 874x series of controllers

Based on ACRMotorDriver.h by:
Mark Rivers
March 28, 2011

== Modifications ==
2015-12-01 - Wayne Lewis - Modify for NewFocus 874x controllers
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class epicsShareClass nf874xAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  nf874xAxis(class nf874xController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

private:
  nf874xController *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  char axisName_[10];      /**< Name of each axis, used in commands to nf874x controller */ 
  double encoderPosition_; /**< Cached copy of the encoder position */ 
  
friend class nf874xController;
};

class epicsShareClass nf874xController : public asynMotorController {
public:
  nf874xController(const char *portName, const char *nf874xPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  nf874xAxis* getAxis(asynUser *pasynUser);
  nf874xAxis* getAxis(int axisNo);

friend class nf874xAxis;
};
