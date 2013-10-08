/*
FILENAME...   MMC200Driver.h
USAGE...      Motor driver support for the Micronix MMC-200 controller.

Kevin Peterson
July 10, 2013

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_MMC200_AXES 99

// No controller-specific parameters yet
#define NUM_MMC200_PARAMS 0  

class MMC200Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MMC200Axis(class MMC200Controller *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  MMC200Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  int axisIndex_;    /* Numbered from 1 */
  int rez_;          /* Units = picometers per full step */
  int microSteps_;   /* Units = microsteps per full step */
  double resolution_;   /* Units = mm per microstep */
  double maxVelocity_;  /* Units = mm per second */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  
friend class MMC200Controller;
};

class MMC200Controller : public asynMotorController {
public:
  MMC200Controller(const char *portName, const char *MMC200PortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int ignoreLimits);

  void report(FILE *fp, int level);
  MMC200Axis* getAxis(asynUser *pasynUser);
  MMC200Axis* getAxis(int axisNo);
  
private:
  int ignoreLimits_;  /* 1 = ignore limits */

friend class MMC200Axis;
};
