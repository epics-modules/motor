/*
FILENAME...   SM300Driver.h
USAGE...      Motor driver support for the SM300 controller.

Based on the Newport SMC100 device driver written by:

*/


// No controller-specific parameters yet
#define NUM_SM300_PARAMS 0  

class epicsShareClass SM300Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  SM300Axis(class SM300Controller *pC, int axis, char axisLabel);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  //asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  SM300Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  double stepSize_;      /**< Encoder increment value obtained with SU? command _or_ resolution, set at boot time */
                         /*   with SMC100CreateController command */
  char axisLabel; /** label for the axis*/
  asynStatus setMotorPosition(const char * lq_return);
friend class SM300Controller;
};

class SM300Controller : public asynMotorController {
public:
  SM300Controller(const char *portName, const char *SMC100PortName, int numAxes, double movingPollPeriod, double idlePollPeriod, double stepSize);
  void report(FILE *fp, int level);
  SM300Axis* getAxis(asynUser *pasynUser);
  SM300Axis* getAxis(int axisNo);

  virtual asynStatus poll();  

private:
	int _status_set;

friend class SM300Axis;
	
};
