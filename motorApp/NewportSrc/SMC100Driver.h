/*
FILENAME...   SMC100Driver.h
USAGE...      Motor driver support for the Newport SMC100 controller.

Based on the ACS MCB-4B Model 3 device driver written by:
Mark Rivers
March 1, 2012

K. Goetze 2012-03-23

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_SMC100_AXES 1

// No controller-specific parameters yet
#define NUM_SMC100_PARAMS 0  

class SMC100Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  SMC100Axis(class SMC100Controller *pC, int axis, double stepSize);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  SMC100Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  double stepSize_;      /**< Encoder increment value obtained with SU? command _or_ resolution, set at boot time */
                         /*   with SMC100CreateController command */
  
friend class SMC100Controller;
};

class SMC100Controller : public asynMotorController {
public:
  SMC100Controller(const char *portName, const char *SMC100PortName, int numAxes, double movingPollPeriod, double idlePollPeriod, double stepSize);

  void report(FILE *fp, int level);
  SMC100Axis* getAxis(asynUser *pasynUser);
  SMC100Axis* getAxis(int axisNo);

friend class SMC100Axis;
};
