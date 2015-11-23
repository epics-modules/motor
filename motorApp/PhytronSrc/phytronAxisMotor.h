/*
FILENAME... phytronAxisMotor.h
USAGE...    Motor record  support for Phytron Axis controller.

Tom Slejko & Bor Marolt
Cosylab d.d. 2014

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"


//Number of controller specific parameters
#define NUM_PHYTRON_PARAMS 29

#define MAX_VELOCITY      40000 //steps/s
#define MIN_VELOCITY      1     //steps/s

#define MAX_ACCELERATION  500000  // steps/s^2
#define MIN_ACCELERATION  4000    // steps/s^2

//Controller parameters
#define controllerStatusString      "CONTROLLER_STATUS"
#define controllerStatusResetString "CONTROLLER_STATUS_RESET"
#define resetControllerString       "CONTROLLER_RESET"

//Axis parameters
#define axisStatusString            "AXIS_STATUS"
#define homingProcedureString       "HOMING_PROCEDURE"
#define axisModeString              "AXIS_MODE"
#define mopOffsetPosString          "MOP_POS"
#define mopOffsetNegString          "MOP_NEG"
#define stepResolutionString        "STEP_RES"
#define stopCurrentString           "STOP_CURRENT"
#define runCurrentString            "RUN_CURRENT"
#define boostCurrentString          "BOOST_CURRENT"
#define encoderTypeString           "ENCODER_TYP"
#define initRecoveryTimeString      "INIT_TIME"
#define positionRecoveryTimeString  "POSITION_TIME"
#define boostConditionString        "BOOST"
#define encoderRateString           "ENC_RATE"
#define switchTypString             "SWITCH_TYP"
#define pwrStageModeString          "PWR_STAGE_MODE"
#define encoderResolutionString     "ENC_RESOLUTION"
#define encoderFunctionString       "ENC_FUNCTION"
#define encoderSFIWidthString       "ENC_SFI_WIDTH"
#define encoderDirectionString      "ENC_DIRECTION"
#define powerStageTempString        "PS_TEMPERATURE"
#define powerStagetMonitorString    "PS_MONITOR"
#define motorTempString             "MOTOR_TEMP"
#define currentDelayTimeString      "CURRENT_DELAY_TIME"
#define axisResetString             "AXIS_RESET"
#define axisStatusResetString       "AXIS_STATUS_RESET"

typedef enum {
  phytronSuccess,
  phytronTimeout,
  phytronOverflow,
  phytronError,
  phytronDisconnected,
  phytronDisabled,
  phytronInvalidReturn,
  phytronInvalidCommand
} phytronStatus;

enum movementType{
  stdMove,
  homeMove,
  stopMove
};

enum homingType{
  limit,
  center,
  encoder,
  limitEncoder,
  centerEncoder,
  referenceCenter,
  referenceCenterEncoder,
};


class phytronAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  phytronAxis(class phytronController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

  asynStatus setEncoderRatio(double ratio);
  asynStatus setEncoderPosition(double position);

  float axisModuleNo_; //Used by sprintf to form commands

private:
  phytronController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */

  phytronStatus setVelocity(double minVelocity, double maxVelocity, int moveType);
  phytronStatus setAcceleration(double acceleration, int movementType);

  size_t response_len;

friend class phytronController;
};

class phytronController : public asynMotorController {
public:
  phytronController(const char *portName, const char *phytronPortName, double movingPollPeriod, double idlePollPeriod, double timeout);
  asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

  void report(FILE *fp, int level);
  phytronAxis* getAxis(asynUser *pasynUser);
  phytronAxis* getAxis(int axisNo);

  phytronStatus sendPhytronCommand(const char *command, char *response_buffer, size_t response_max_len, size_t *nread);

  void resetAxisEncoderRatio();

  //casts phytronStatus to asynStatus
  asynStatus    phyToAsyn(phytronStatus phyStatus);

  char * controllerName_;
  std::vector<phytronAxis*> axes;

protected:
  //Additional parameters used by additional records
  int axisStatus_;
  int controllerStatus_;
  int homingProcedure_;
  int axisMode_;
  int mopOffsetPos_;
  int mopOffsetNeg_;
  int stepResolution_;
  int stopCurrent_;
  int runCurrent_;
  int boostCurrent_;
  int encoderType_;
  int initRecoveryTime_;
  int positionRecoveryTime_;
  int boost_;
  int encoderRate_;
  int switchTyp_;
  int pwrStageMode_;
  int encoderRes_;
  int encoderFunc_;
  int encoderSFIWidth_;
  int encoderDirection_;
  int powerStageTemp_;
  int powerStageMonitor_;
  int motorTemp_;
  int currentDelayTime_;
  int resetController_;
  int axisReset_;
  int axisStatusReset_;
  int controllerStatusReset_;

private:
  double timeout_;


friend class phytronAxis;
};
