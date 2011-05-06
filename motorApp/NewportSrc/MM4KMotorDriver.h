/*
FILENAME...   MM4KMotorDriver.h
USAGE...      asyn motor model #3 template.

Mark Rivers
March 28, 2011

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

// drvInfo strings for extra parameters that the MM4K controller supports
#define MM4KJerkString           "MM4K_JERK"
#define MM4KReadBinaryIOString   "MM4K_READ_BINARY_IO"
#define MM4KBinaryInString       "MM4K_BINARY_IN"
#define MM4KBinaryOutString      "MM4K_BINARY_OUT"
#define MM4KBinaryOutRBVString   "MM4K_BINARY_OUT_RBV"

// Motor status response bits for MM[3000/4000/4005/4006].
#define MM4000_HOME       0x20  /* Home LS. */
#define MM4000_LOW_LIMIT  0x10  /* Minus Travel Limit. */
#define MM4000_HIGH_LIMIT 0x08  /* Plus Travel Limit. */
#define MM4000_DIRECTION  0x04  /* Motor direction: 0 - minus; 1 - plus. */
#define MM4000_POWER_OFF  0x02  /* Motor power 0 - ON; 1 - OFF. */
#define MM4000_MOVING     0x01  /* In-motion indicator. */

#define MAX_BUF_SIZE 160

enum MM_model
{
    MM4000,
    MM4005
};

// class asynMotorAxis;

class MM4KAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MM4KAxis(class MM4KController *pC, int axis);
  asynStatus move(double position, bool relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, bool forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  void       axisReport(FILE *fp);

private:
  MM4KController *pC_;     /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  char axisName_[10];      /**< Name of each axis, used in commands to controller */ 
  double pulsesPerUnit_;   /**< Pulses per engineering unit, which is what ACR controller uses */ 
  int flagsReg_;           /**< Address of the flags register */ 
  int limitsReg_;          /**< Address of the limits register */ 
  int encoderPositionReg_; /**< Address of the encoder position register */ 
  int theoryPositionReg_;  /**< Address of the theoretical position register */ 
  double encoderPosition_; /**< Cached copy of the encoder position */ 
  double theoryPosition_;  /**< Cached copy of the theoretical position */ 
  int currentFlags_;       /**< Cached copy of the current flags */ 
  int currentLimits_;      /**< Cached copy of the current limits */ 
  int maxDigits;
  double homePreset;
  double highLimit;
  double lowLimit;
};

class MM4KController : public asynMotorController {
public:
  MM4KController(const char *portName, const char *MM4KPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynPortDriver */
  asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
  
  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  MM4KAxis* getAxis(asynUser *pasynUser);
  MM4KAxis* getAxis(int axisNo);
  asynStatus setOutString(const char *);
  
  /* These are the methods that are new to this class */
  asynStatus readBinaryIO();
  asynStatus writeController();
  asynStatus writeController(const char *output, double timeout);
  asynStatus writeController(const char *format, int axisnum);
  asynStatus writeReadController();
  asynStatus writeReadController(const char *output);
  asynStatus writeReadController(const char *format, int axisnum);
  asynStatus writeReadController(const char *output, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);
  asynStatus writeReadConvertDouble(const char *format, int axisnum, double *output);
  asynStatus writeReadConvertInt(const char *format, int axisnum, int *output);
  char       writeReadRtnResponse(const char *format, int axisnum, asynStatus *);
  char       writeReadRtnResponse(const char *format, asynStatus *);
  virtual int  getNumParams();

protected:
  int MM4KJerk_;
#define FIRST_MM4K_PARAM MM4KJerk_
  int MM4KReadBinaryIO_;
  int MM4KBinaryIn_;
  int MM4KBinaryOut_;
  int MM4KBinaryOutRBV_;
#define LAST_MM4K_PARAM MM4KBinaryOutRBV_

#define NUM_MM4K_PARAMS (&LAST_MM4K_PARAM - &FIRST_MM4K_PARAM + 1)

private:
  asynUser *pasynUser_;
  char outString_[MAX_BUF_SIZE];
  char inString_[MAX_BUF_SIZE];
  int binaryIn_;
  int binaryOutRBV_;
  int binaryInReg_;
  int binaryOutReg_;
  MM_model model;
  char firmwareVersion_[100];
};
