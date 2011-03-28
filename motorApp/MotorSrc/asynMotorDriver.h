/* asynMotorDriver.h 
 * 
 * Mark Rivers
 *
 * This file defines the base class for an asynMotorDriver.  It is the class
 * from which real motor drivers are derived.  It derives from asynPortDriver.
 */
#ifndef asynMotorDriver_H
#define asynMotorDriver_H

#include <epicsTypes.h>

#define motorMoveRelString              "MOTOR_MOVE_REL"
#define motorMoveAbsString              "MOTOR_MOVE_ABS"
#define motorMoveVelString              "MOTOR_MOVE_VEL"
#define motorHomeString                 "MOTOR_HOME"
#define motorStopString                 "MOTOR_STOP_AXIS"
#define motorVelocityString             "MOTOR_VELOCITY"
#define motorVelBaseString              "MOTOR_VEL_BASE"
#define motorAccelString                "MOTOR_ACCEL"
#define motorPositionString             "MOTOR_POSITION"
#define motorEncoderPositionString      "MOTOR_ENCODER_POSITION"
#define motorDeferMovesString           "MOTOR_DEFER_MOVES"
#define motorResolutionString           "MOTOR_RESOLUTION"
#define motorEncRatioString             "MOTOR_ENC_RATIO"
#define motorPgainString                "MOTOR_PGAIN"
#define motorIgainString                "MOTOR_IGAIN"
#define motorDgainString                "MOTOR_DGAIN"
#define motorHighLimitString            "MOTOR_HIGH_LIMIT"
#define motorLowLimitString             "MOTOR_LOW_LIMIT"
#define motorSetClosedLoopString        "MOTOR_SET_CLOSED_LOOP"
#define motorStatusString               "MOTOR_STATUS"
#define motorUpdateStatusString         "MOTOR_UPDATE_STATUS"
#define motorStatusDirectionString      "MOTOR_STATUS_DIRECTION" 
#define motorStatusDoneString           "MOTOR_STATUS_DONE"
#define motorStatusHighLimitString      "MOTOR_STATUS_HIGH_LIMIT"
#define motorStatusAtHomeString         "MOTOR_STATUS_AT_HOME"
#define motorStatusSlipString           "MOTOR_STATUS_SLIP"
#define motorStatusPowerOnString        "MOTOR_STATUS_POWERED"
#define motorStatusFollowingErrorString "MOTOR_STATUS_FOLLOWING_ERROR"
#define motorStatusHomeString           "MOTOR_STATUS_HOME"
#define motorStatusHasEncoderString     "MOTOR_STATUS_HAS_ENCODER"
#define motorStatusProblemString        "MOTOR_STATUS_PROBLEM"
#define motorStatusMovingString         "MOTOR_STATUS_MOVING"
#define motorStatusGainSupportString    "MOTOR_STATUS_GAIN_SUPPORT"
#define motorStatusCommsErrorString     "MOTOR_STATUS_COMMS_ERROR"
#define motorStatusLowLimitString       "MOTOR_STATUS_LOW_LIMIT"
#define motorStatusHomedString          "MOTOR_STATUS_HOMED"

typedef struct MotorStatus {
  double position;
  double encoderPosition;
  double velocity;
  epicsUInt32 status;
} MotorStatus;

#ifdef __cplusplus
#include <asynPortDriver.h>

/** Class from which motor drivers are directly derived. */
class epicsShareFunc asynMotorAxis {
public:
  /* This is the constructor for the class. */
  asynMotorAxis(int axisNumber, class asynMotorController *pController);

  virtual asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  virtual asynStatus stop(double acceleration);

private:
  int axisNumber_;
  class asynMotorController *pController_;
};

/** Class from which motor drivers are directly derived. */
class epicsShareFunc asynMotorController : public asynPortDriver {
public:
  /* This is the constructor for the class. */
  asynMotorController(const char *portName, int maxAxes, int numParams,
                      int interfaceMask, int interruptMask,
                      int asynFlags, int autoConnect, int priority, int stackSize);

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus readGenericPointer(asynUser *pasynUser, void *pointer);
  virtual asynStatus setIntegerParam(int list, int index, int value);
  virtual asynStatus setDoubleParam(int list, int index, double value);
  virtual asynStatus callParamCallbacks(int addr);

  /* These are the methods that are new to this class */
  virtual asynStatus moveAxis(asynUser *pasynUser, double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus moveVelocityAxis(asynUser *pasynUser, double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus homeAxis(asynUser *pasynUser, double minVelocity, double maxVelocity, double acceleration, int forwards);
  virtual asynStatus stopAxis(asynUser *pasynUser, double acceleration);
  virtual asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
  virtual asynStatus triggerProfile(asynUser *pasynUser);

protected:
  int motorMoveRel_;
  #define FIRST_MOTOR_PARAM motorMoveRel_
  int motorMoveAbs_;
  int motorMoveVel_;
  int motorHome_;
  int motorStop_;
  int motorVelocity_;
  int motorVelBase_;
  int motorAccel_;
  int motorPosition_;
  int motorEncoderPosition_;
  int motorDeferMoves_;
  int motorResolution_;
  int motorEncRatio_;
  int motorPgain_;
  int motorIgain_;
  int motorDgain_;
  int motorHighLimit_;
  int motorLowLimit_;
  int motorSetClosedLoop_;
  int motorStatus_;
  int motorUpdateStatus_;
  int motorStatusDirection_;
  int motorStatusDone_;
  int motorStatusHighLimit_;
  int motorStatusAtHome_;
  int motorStatusSlip_;
  int motorStatusPowerOn_;
  int motorStatusFollowingError_;
  int motorStatusHome_;
  int motorStatusHasEncoder_;
  int motorStatusProblem_;
  int motorStatusMoving_;
  int motorStatusGainSupport_;
  int motorStatusCommsError_;
  int motorStatusLowLimit_;
  int motorStatusHomed_;
  #define LAST_MOTOR_PARAM motorStatusHomed_

private:
  MotorStatus *axisStatus_;
  int *axisStatusChanged_;
};
#define NUM_MOTOR_DRIVER_PARAMS (&LAST_MOTOR_PARAM - &FIRST_MOTOR_PARAM + 1)

#endif /* _cplusplus */
#endif /* asynMotorDriver_H */
