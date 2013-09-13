/*
FILENAME...   MVP2001Driver.h
USAGE...      Motor driver support for the MicroMo MVP2001 controller.

Kevin Peterson
August 14, 2013

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_MVP2001_AXES 32     /* motor.h sets the maximum number of axes */
#define BUFF_SIZE 20		/* Maximum length of string to/from MVP2001 */

// No controller-specific parameters yet
#define NUM_MVP2001_PARAMS 0  

class MVP2001Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MVP2001Axis(class MVP2001Controller *pC, int axisNo, int encLPR, int maxCurr, int limPol);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setPGain(double pGain);
  asynStatus setIGain(double iGain);
  asynStatus setDGain(double dGain);
  asynStatus setClosedLoop(bool closedLoop);

private:
  MVP2001Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  int axisIndex_;
  double stepsPerRev_;
  int encoderLinesPerRev_;
  int maxCurrent_;
  int samplePeriod_;
  int limitPolarity_;
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  
friend class MVP2001Controller;
};

class MVP2001Controller : public asynMotorController {
public:
  MVP2001Controller(const char *portName, const char *MVP2001PortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  MVP2001Axis* getAxis(asynUser *pasynUser);
  MVP2001Axis* getAxis(int axisNo);

private:
  char buff_[BUFF_SIZE];
  asynStatus writeRead2xController();
  asynStatus writeRead2xController(const char *output, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);
  void parseReply(char *inString, int *val, int nchars);
  
friend class MVP2001Axis;
};
