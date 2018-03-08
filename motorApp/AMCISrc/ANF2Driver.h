/*
FILENAME...   ANF2Driver.h
USAGE...      Motor driver support for the AMCI ANF2 controller.

Based on MCB-4B driver written by
Mark Rivers
March 1, 2012

K. Goetze 2014-03-24

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_AXES 2

#define MAX_INPUT_REGS 10
#define MAX_OUTPUT_REGS 10

#define AXIS_REG_OFFSET 10

/*** Input CMD Registers ***/
#define STATUS_1   0
#define STATUS_2   1
#define POS_RD_UPR 2
#define POS_RD_LWR 3
#define EN_POS_UPR 4
#define EN_POS_LWR 5
#define EN_CAP_UPR 6
#define EN_CAP_LWR 7
// Not used must equal zero #define RESERVED 8
#define NET_CONN   9

/*** Output CMD Registers ***/
#define CMD_MSW    0  // module 0 starts at register address 1024.  This is set in drvModbusAsynConfigure.
#define CMD_LSW    1
#define POS_WR_UPR 2
#define POS_WR_LWR 3
#define SPD_UPR    4
#define SPD_LWR    5
#define ACCEL      6
#define DECEL      7
// Not used must equal zero #define RESERVED 8
// Not used must equal zero #define RESERVED 9

/*** Output Configuration Registers ***/
#define CONFIG_MSW   0
#define CONFIG_LSW   1
#define BASE_SPD_MSW 2
#define BASE_SPD_LSW 3
#define HOME_TIMEOUT 4


// No. of controller-specific parameters
#define NUM_ANF2_PARAMS 1 

/** drvInfo strings for extra parameters that the ACR controller supports */
#define ANF2JerkString           "ANF2_JERK"

class ANF2Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  ANF2Axis(class ANF2Controller *pC, int axisNo, epicsInt32 config);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  ANF2Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  asynUser *pasynUserForceRead_;
  int axisNo_;
  epicsInt32 config_;

friend class ANF2Controller;
};

class ANF2Controller : public asynMotorController {
public:
  ANF2Controller(const char *portName, const char *ANF2InPortName, const char *ANF2OutPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  ANF2Axis* getAxis(asynUser *pasynUser);
  ANF2Axis* getAxis(int axisNo); 
  asynUser *pasynUserInReg_[MAX_AXES][MAX_INPUT_REGS];
  asynUser *pasynUserOutReg_[MAX_AXES][MAX_OUTPUT_REGS]; 
//  asynUser *pasynUserForceRead_;


  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  //asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  //void report(FILE *fp, int level);

  /* These are the methods that are new to this class */
  
protected:
  int ANF2Jerk_;          /**< Jerk time parameter index */        


private:
  asynStatus writeReg16(int, int, int, double);
  asynStatus writeReg32(int, int, int, double);
  asynStatus readReg16(int, int, epicsInt32*, double);
  asynStatus readReg32(int, int, epicsInt32*, double);
  char *inputDriver_;

friend class ANF2Axis;
};
