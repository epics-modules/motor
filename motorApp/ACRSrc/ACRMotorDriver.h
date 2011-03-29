/*
FILENAME...   ACRMotorDriver.h
USAGE...      Motor driver support for the Parker ACR series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/

#include "asynMotorDriver.h"

// drvInfo strings for extra parameters that the ACR controller supports
#define ACRJerkString           "ACR_JERK"
#define ACRReadBinaryIOString   "ACR_READ_BINARY_IO"
#define ACRBinaryInString       "ACR_BINARY_IN"
#define ACRBinaryOutString      "ACR_BINARY_OUT"
#define ACRBinaryOutRBVString   "ACR_BINARY_OUT_RBV"

#define MAX_ACR_STRING_SIZE 80

class ACRAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  ACRAxis(class ACRController *pController, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(int *moving);
  asynStatus setPosition(double position);

  // These are the methods that are new to this class 
  class ACRController* getController();

private:
  char axisName_[10];
  double pulsesPerUnit_;
  int flagsReg_;
  int limitsReg_;
  int encoderPositionReg_;
  int theoryPositionReg_;
  double encoderPosition_;
  double theoryPosition_;
  int currentFlags_;
  int currentLimits_;
  
friend class ACRController;
};

class ACRController : public asynMotorController {
public:
  ACRController(const char *portName, const char *ACRPortName, int numAxes, int movingPollPeriod, int idlePollPeriod);
  
  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
  void report(FILE *fp, int level);
  ACRAxis* getAxis(asynUser *pasynUser);
  ACRAxis* getAxis(int axisNo);
  asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
  asynStatus triggerProfile(asynUser *pasynUser);
  
  /* These are the methods that are new to this class */
  asynStatus readBinaryIO();
  asynStatus writeController();
  asynStatus writeController(const char *output, double timeout);
  asynStatus writeReadController();
  asynStatus writeReadController(const char *output, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);
  
protected:
  int ACRJerk_;
#define FIRST_ACR_PARAM ACRJerk_
  int ACRReadBinaryIO_;
  int ACRBinaryIn_;
  int ACRBinaryOut_;
  int ACRBinaryOutRBV_;
#define LAST_ACR_PARAM ACRBinaryOutRBV_

#define NUM_ACR_PARAMS (&LAST_ACR_PARAM - &FIRST_ACR_PARAM + 1)

private:
  asynUser *pasynUserACR_;
  char outString_[MAX_ACR_STRING_SIZE];
  char inString_[MAX_ACR_STRING_SIZE];
  int binaryIn_;
  int binaryOutRBV_;
  int binaryInReg_;
  int binaryOutReg_;
  
friend class ACRAxis;
};
