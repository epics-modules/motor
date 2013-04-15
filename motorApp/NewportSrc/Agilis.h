/*
FILENAME...   Agilis.h
USAGE...      Motor driver support for the Newport Agilis controller.

Mark Rivers
April 11, 2013

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_AGILIS_AXES 8

// No controller-specific parameters yet
#define NUM_AGILIS_PARAMS 0  

class AgilisAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  AgilisAxis(class AgilisController *pC, int axis, bool hasLimits, int stepSize);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

private:
  AgilisController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  bool hasLimits_;
  int stepSize_;
  int currentPosition_;
  int positionOffset_;
  int axisID_;
  
friend class AgilisController;
};

class AgilisController : public asynMotorController {
public:
  AgilisController(const char *portName, const char *AgilisPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  AgilisAxis* getAxis(asynUser *pasynUser);
  AgilisAxis* getAxis(int axisNo);

friend class AgilisAxis;
};
