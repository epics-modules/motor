/*
FILENAME...   ACRMotorDriver.h
USAGE...      Motor driver support for the Parker ACR series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

/** drvInfo strings for extra parameters that the ACR controller supports */
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
  ACRAxis(class ACRController *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

private:
  ACRController *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  char axisName_[10];      /**< Name of each axis, used in commands to ACR controller */ 
  double pulsesPerUnit_;   /**< Pulses per engineering unit, which is what ACR controller uses */ 
  int flagsReg_;           /**< Address of the flags register */ 
  int limitsReg_;          /**< Address of the limits register */ 
  int encoderPositionReg_; /**< Address of the encoder position register */ 
  int theoryPositionReg_;  /**< Address of the theoretical position register */ 
  double encoderPosition_; /**< Cached copy of the encoder position */ 
  double theoryPosition_;  /**< Cached copy of the theoretical position */ 
  int currentFlags_;       /**< Cached copy of the current flags */ 
  int currentLimits_;      /**< Cached copy of the current limits */ 
  
friend class ACRController;
};

class ACRController : public asynMotorController {
public:
  ACRController(const char *portName, const char *ACRPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynPortDriver */
  asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
  
  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  ACRAxis* getAxis(asynUser *pasynUser);
  ACRAxis* getAxis(int axisNo);

  
  /* These are the methods that are new to this class */
  asynStatus readBinaryIO();
  asynStatus writeController();
  asynStatus writeController(const char *output, double timeout);
  asynStatus writeReadController();
  asynStatus writeReadController(const char *output, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);
  
protected:
  int ACRJerk_;          /**< Jerk time parameter index */        
#define FIRST_ACR_PARAM ACRJerk_
  int ACRReadBinaryIO_;  /**< Read binary I/O parameter index */ 
  int ACRBinaryIn_;      /**< Binary input parameter index */
  int ACRBinaryOut_;     /**< Binary output parameter index */
  int ACRBinaryOutRBV_;  /**< Binary output readback parameter index */
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
