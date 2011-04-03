/* asynMotorController.h 
 * 
 * Mark Rivers
 *
 * This file defines the base class for an asynMotorController.  It is the class
 * from which real motor controllers are derived.  It derives from asynPortDriver.
 */
#ifndef asynMotorController_H
#define asynMotorController_H

#include <epicsEvent.h>
#include <epicsTypes.h>

/** Strings defining parameters for the driver. 
  * These are the values passed to drvUserCreate. 
  * The driver will place in pasynUser->reason an integer to be used when the
  * standard asyn interface methods are called. */
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

// These are the per-controller parameters for profile moves (coordinated motion)
#define profileNumAxesString            "PROFILE_NUM_AXES"
#define profileNumPointsString          "PROFILE_NUM_POINTS"
#define profileNumPulsesString          "PROFILE_NUM_PULSES"
#define profileStartPulsesString        "PROFILE_START_PULSES"
#define profileEndPulsesString          "PROFILE_END_PULSES"
#define profileActualPulsesString       "PROFILE_ACTUAL_PULSES"
#define profileTimeArrayString          "PROFILE_TIME_ARRAY"
#define profileAccelerationString       "PROFILE_ACCELERATION"
#define profileBuildString              "PROFILE_BUILD"
#define profileBuildStateString         "PROFILE_BUILD_STATE"
#define profileBuildStatusString        "PROFILE_BUILD_STATUS"
#define profileBuildMessageString       "PROFILE_BUILD_MESSAGE"
#define profileExecuteString            "PROFILE_EXECUTE"
#define profileExecuteStateString       "PROFILE_EXECUTE_STATE"
#define profileExecuteStatusString      "PROFILE_EXECUTE_STATUS"
#define profileExecuteMessageString     "PROFILE_EXECUTE_MESSAGE"
#define profileReadbackString           "PROFILE_READBACK"
#define profileReadbackStateString      "PROFILE_READBACK_STATE"
#define profileReadbackStatusString     "PROFILE_READBACK_STATUS"
#define profileReadbackMessageString    "PROFILE_READBACK_MESSAGE"

// These are the per-axis parameters for profile moves
#define profileUseAxisString            "PROFILE_USE_AXIS"
#define profilePositionsString          "PROFILE_POSITIONS"
#define profileReadbacksString          "PROFILE_READBACKS"
#define profileFollowingErrorsString    "PROFILE_FOLLOWING_ERRORS"
#define profileMotorResolutionString    "PROFILE_MOTOR_RESOLUTION"
#define profileMotorDirectionString     "PROFILE_MOTOR_DIRECTION"
#define profileMotorOffsetString        "PROFILE_MOTOR_OFFSET"

/** The structure that is passed back to devMotorAsyn when the status changes. */
typedef struct MotorStatus {
  double position;           /**< Commanded motor position */
  double encoderPosition;    /**< Actual encoder position */
  double velocity;           /**< Actual velocity */
  epicsUInt32 status;        /**< Word containing status bits (motion done, limits, etc.) */
} MotorStatus;

/* State codes for Build, Read and Execute. Careful, these must match the
 * corresponding MBBI records, but there is no way to check this */
enum ProfileBuildState{
  PROFILE_BUILD_DONE,
  PROFILE_BUILD_BUSY,
};

enum ProfileExecuteState{
  PROFILE_EXECUTE_DONE,
  PROFILE_EXECUTE_MOVE_START,
  PROFILE_EXECUTE_EXECUTING,
  PROFILE_EXECUTE_FLYBACK
};

enum ProfileReadbackState{
  PROFILE_READBACK_DONE,
  PROFILE_READBACK_BUSY
};

/* Status codes for Build, Execute and Read */
enum ProfileStatus {
  PROFILE_STATUS_UNDEFINED,
  PROFILE_STATUS_SUCCESS,
  PROFILE_STATUS_FAILURE,
  PROFILE_STATUS_ABORT,
  PROFILE_STATUS_TIMEOUT
};

#ifdef __cplusplus
#include <asynPortDriver.h>

class asynMotorAxis;

class epicsShareFunc asynMotorController : public asynPortDriver {
public:
  /* This is the constructor for the class. */
  asynMotorController(const char *portName, int numAxes, int numParams,
                      int interfaceMask, int interruptMask,
                      int asynFlags, int autoConnect, int priority, int stackSize);

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nEelements);
  virtual asynStatus readGenericPointer(asynUser *pasynUser, void *pointer);

  /* These are the methods that are new to this class */
  virtual asynMotorAxis* getAxis(asynUser *pasynUser);
  virtual asynMotorAxis* getAxis(int axisNo);
  virtual asynStatus startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls);
  virtual asynStatus wakeupPoller();
  virtual asynStatus poll();
  void asynMotorPoller();  // This should be private but is called from C function
  
  /* These are the functions for profile moves */
  virtual asynStatus initializeProfile(int maxPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus readbackProfile();
  
  int shuttingDown_;   /**< Flag indicating that IOC is shutting down.  Stops poller */

protected:
  /** These are the index numbers for the parameters in the parameter library.
   * They are the values of pasynUser->reason in calls from device support */
   // These are the motor commands
  #define FIRST_MOTOR_PARAM motorMoveRel_
  int motorMoveRel_;
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

  // These are the status bits
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

  // These are the per-controller parameters for profile moves
  int profileNumAxes_;
  int profileNumPoints_;
  int profileNumPulses_;
  int profileStartPulses_;
  int profileEndPulses_;
  int profileActualPulses_;
  int profileTimeArray_;
  int profileAcceleration_;
  int profileBuild_;
  int profileBuildState_;
  int profileBuildStatus_;
  int profileBuildMessage_;
  int profileExecute_;
  int profileExecuteState_;
  int profileExecuteStatus_;
  int profileExecuteMessage_;
  int profileReadback_;
  int profileReadbackState_;
  int profileReadbackStatus_;
  int profileReadbackMessage_;

  // These are the per-axis parameters for profile moves
  int profileUseAxis_;
  int profilePositions_;
  int profileReadbacks_;
  int profileFollowingErrors_;
  int profileMotorResolution_;
  int profileMotorDirection_;
  int profileMotorOffset_;
  #define LAST_MOTOR_PARAM profileMotorOffset_

  int numAxes_;                 /**< Number of axes this controller supports */
  asynMotorAxis **pAxes_;       /**< Array of pointers to axis objects */
  epicsEventId pollEventId_;    /**< Event ID to wake up poller */
  double idlePollPeriod_;       /**< The time between polls when no axes are moving */
  double movingPollPeriod_;     /**< The time between polls when any axis is moving */
  int    forcedFastPolls_;      /**< The number of forced fast polls when the poller wakes up */
 
  int maxProfilePoints_;        /**< Maximum number of profile point */
  double *profileTimes_;        /**< Array of times per profile point */

friend class asynMotorAxis;
};
#define NUM_MOTOR_DRIVER_PARAMS (&LAST_MOTOR_PARAM - &FIRST_MOTOR_PARAM + 1)

#endif /* _cplusplus */
#endif /* asynMotorController_H */
