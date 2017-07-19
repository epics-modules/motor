/*
FILENAME...   ANG1Driver.h
USAGE...      Motor driver support for the AMCI ANG1 controller.

Based on MCB-4B driver written by
Mark Rivers
March 1, 2012

K. Goetze 2014-03-24

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_ANG1_AXES 1

#define MAX_INPUT_REGS 10
#define MAX_OUTPUT_REGS  10

// No. of controller-specific parameters
#define NUM_ANG1_PARAMS 1 

/** drvInfo strings for extra parameters that the ACR controller supports */
#define ANG1JerkString           "ANG1_JERK"
//#define ACRReadBinaryIOString   "ACR_READ_BINARY_IO"
//#define ACRBinaryInString       "ACR_BINARY_IN"
//#define ACRBinaryOutString      "ACR_BINARY_OUT"
//#define ACRBinaryOutRBVString   "ACR_BINARY_OUT_RBV" 

class ANG1Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  ANG1Axis(class ANG1Controller *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  ANG1Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  asynUser *pasynUserForceRead_;

friend class ANG1Controller;
};

class ANG1Controller : public asynMotorController {
public:
  ANG1Controller(const char *portName, const char *ANG1InPortName, const char *ANG1OutPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  ANG1Axis* getAxis(asynUser *pasynUser);
  ANG1Axis* getAxis(int axisNo); 
  asynUser *pasynUserInReg_[MAX_INPUT_REGS];
  asynUser *pasynUserOutReg_[MAX_OUTPUT_REGS]; 
//  asynUser *pasynUserForceRead_;


  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  //asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  //void report(FILE *fp, int level);
  //ACRAxis* getAxis(asynUser *pasynUser);
  //ACRAxis* getAxis(int axisNo);


  /* These are the methods that are new to this class */
  
protected:
  int ANG1Jerk_;          /**< Jerk time parameter index */        


private:
  asynStatus writeReg16(int, int, double);
  asynStatus writeReg32(int, int, double);
  asynStatus readReg16(int, epicsInt32*, double);
  asynStatus readReg32(int, epicsInt32*, double);
  char *inputDriver_;

friend class ANG1Axis;
};
