/*
FILENAME...   MCB4BDriver.h
USAGE...      Motor driver support for the ACS MCB-4B controller.

Mark Rivers
March 1, 2012

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_MCB4B_AXES 4

// No controller-specific parameters yet
#define NUM_MCB4B_PARAMS 0  

class MCB4BAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MCB4BAxis(class MCB4BController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  MCB4BController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  
friend class MCB4BController;
};

class MCB4BController : public asynMotorController {
public:
  MCB4BController(const char *portName, const char *MCB4BPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  MCB4BAxis* getAxis(asynUser *pasynUser);
  MCB4BAxis* getAxis(int axisNo);

friend class MCB4BAxis;
};
