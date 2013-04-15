/*
FILENAME...  AG_UC.h
USAGE...      Motor driver support for the Newport Agilis AG-UC series of controllers.

Mark Rivers
April 11, 2013

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

// No controller-specific parameters yet
#define NUM_AG_UC_PARAMS 0  

class AG_UCAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  AG_UCAxis(class AG_UCController *pC, int axis, bool hasLimits, int forwardAmplitude, int reverseAmplitude);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

private:
  int velocityToSpeedCode(double velocity);
  AG_UCController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  bool hasLimits_;
  int forwardAmplitude_;
  int reverseAmplitude_;
  int currentPosition_;
  int positionOffset_;
  int axisID_;
  
friend class AG_UCController;
};

class AG_UCController : public asynMotorController {
public:
  AG_UCController(const char *portName, const char *serialPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  AG_UCAxis* getAxis(asynUser *pasynUser);
  AG_UCAxis* getAxis(int axisNo);
  asynStatus writeAgilis();
  asynStatus writeAgilis(const char *output, double timeout);

  friend class AG_UCAxis;
};
