/*
FILENAME...   C300MotorDriver.h
USAGE...      Motor driver support for the nPoint C300 series of controllers

Kevin Peterson
March 27, 2012

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_C300_STRING_SIZE 300

class C300Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  C300Axis(class C300Controller *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus poll(bool *moving);

private:
  C300Controller *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  char axisName_[6];      /**< Name of each axis, used in commands to C300 controller */ 
  double bitsPerUnit_;   /**< Bits per engineering unit, which is what C300 controller uses */ 
  double encoderPosition_; /**< Cached copy of the encoder position */ 
  double theoryPosition_;  /**< Cached copy of the theoretical position */ 
  int currentLimits_;      /**< Cached copy of the current limits */ 
  double diScaleFactor_;
  double diScaleFactorInv_;
  double posMonCorrectedVal_;
  double analogInScaledVal_;
  
friend class C300Controller;
};

class C300Controller : public asynMotorController {
public:
  C300Controller(const char *portName, const char *C300PortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
  C300Axis* getAxis(asynUser *pasynUser);
  C300Axis* getAxis(int axisNo);

  /* These are the methods that are new to this class */
  asynStatus writeController();
  asynStatus writeController(const char *output, double timeout);
  asynStatus writeReadController();
  asynStatus writeReadController(const char *output, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);

protected:

#define NUM_C300_PARAMS 0

private:
  asynUser *pasynUserC300_;
  char outString_[MAX_C300_STRING_SIZE];
  char inString_[MAX_C300_STRING_SIZE];


friend class C300Axis;
};
