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
#include <shareLib.h>

#define MAX_CONTROLLER_STRING_SIZE 1024
#define DEFAULT_CONTROLLER_TIMEOUT 2.0

#define  MOTORNOTHOMEDPROBLEM_IGNORE    0
#define  MOTORNOTHOMEDPROBLEM_WARNING   1
#define  MOTORNOTHOMEDPROBLEM_ERROR     2

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
#define motorMoveToHomeString           "MOTOR_MOVE_HOME"
#define motorResolutionString           "MOTOR_RESOLUTION"
#define motorEncoderRatioString         "MOTOR_ENCODER_RATIO"
#define motorPGainString                "MOTOR_PGAIN"
#define motorIGainString                "MOTOR_IGAIN"
#define motorDGainString                "MOTOR_DGAIN"
#define motorHighLimitString            "MOTOR_HIGH_LIMIT"
#define motorLowLimitString             "MOTOR_LOW_LIMIT"
#define motorClosedLoopString           "MOTOR_CLOSED_LOOP"
#define motorPowerAutoOnOffString       "MOTOR_POWER_AUTO_ONOFF"
#define motorPowerOnDelayString         "MOTOR_POWER_ON_DELAY"
#define motorPowerOffDelayString        "MOTOR_POWER_OFF_DELAY"
#define motorPowerOffFractionString     "MOTOR_POWER_OFF_FRACTION"
#define motorPostMoveDelayString        "MOTOR_POST_MOVE_DELAY"
#define motorStatusString               "MOTOR_STATUS"
#define motorLatestCommandString        "MOTOR_LATEST_COMMAND"
#define motorMessageIsFromDriverString  "MOTOR_MESSAGE_DRIVER"
#define motorMessageTextString          "MOTOR_MESSAGE_TEXT"
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

/* Addition flags which can be set by the specific driver */
#define motorFlagsHomeOnLsString        "MOTOR_FLAGS_HOME_ON_LS"
#define motorFlagsLSrampDownString      "MOTOR_FLAGS_LS_RAMP_DOWN"
#define motorFlagsNoStopOnLsString      "MOTOR_FLAGS_NO_STOP_ON_LS"
#define motorFlagsDriverUsesEGUString   "MOTOR_FLAGS_DRIVER_USES_EGU"
#define motorFlagsAdjAfterHomedString   "MOTOR_FLAGS_ADJ_AFTER_HOMED"
#define motorFlagsNtmUpdateString       "MOTOR_FLAGS_NTM_UPDATE"
#define motorFlagsNotHomedProblemString "MOTOR_FLAGS_NOT_HOMED_PROBLEM"
#define motorFlagsNoTweakOnLsString     "MOTOR_FLAGS_NO_TWEAK_ON_LS"

#define motorWaitPollsBeforeReadyString "MOTOR_WAIT_POLLS_BEFORE_READY"

#define motorShowPowerOffString       "MOTOR_SHOW_POWER_OFF"

/* These are per-axis parameters for passing additional motor record information to the driver */
#define motorRecResolutionString        "MOTOR_REC_RESOLUTION"
#define motorRecDirectionString         "MOTOR_REC_DIRECTION"
#define motorRecOffsetString            "MOTOR_REC_OFFSET"

  /* Parameters from the controller to the driver and record */
#define motorHighLimitROString          "MOTOR_HIGH_LIMIT_RO"
#define motorLowLimitROString           "MOTOR_LOW_LIMIT_RO"

/* These are the per-controller parameters for profile moves (coordinated motion) */
#define profileNumAxesString            "PROFILE_NUM_AXES"
#define profileNumPointsString          "PROFILE_NUM_POINTS"
#define profileCurrentPointString       "PROFILE_CURRENT_POINT"
#define profileNumPulsesString          "PROFILE_NUM_PULSES"
#define profileStartPulsesString        "PROFILE_START_PULSES"
#define profileEndPulsesString          "PROFILE_END_PULSES"
#define profileActualPulsesString       "PROFILE_ACTUAL_PULSES"
#define profileNumReadbacksString       "PROFILE_NUM_READBACKS"
#define profileTimeModeString           "PROFILE_TIME_MODE"
#define profileFixedTimeString          "PROFILE_FIXED_TIME"
#define profileTimeArrayString          "PROFILE_TIME_ARRAY"
#define profileAccelerationString       "PROFILE_ACCELERATION"
#define profileMoveModeString           "PROFILE_MOVE_MODE"
#define profileBuildString              "PROFILE_BUILD"
#define profileBuildStateString         "PROFILE_BUILD_STATE"
#define profileBuildStatusString        "PROFILE_BUILD_STATUS"
#define profileBuildMessageString       "PROFILE_BUILD_MESSAGE"
#define profileExecuteString            "PROFILE_EXECUTE"
#define profileExecuteStateString       "PROFILE_EXECUTE_STATE"
#define profileExecuteStatusString      "PROFILE_EXECUTE_STATUS"
#define profileExecuteMessageString     "PROFILE_EXECUTE_MESSAGE"
#define profileAbortString              "PROFILE_ABORT"
#define profileReadbackString           "PROFILE_READBACK"
#define profileReadbackStateString      "PROFILE_READBACK_STATE"
#define profileReadbackStatusString     "PROFILE_READBACK_STATUS"
#define profileReadbackMessageString    "PROFILE_READBACK_MESSAGE"

/* These are the per-axis parameters for profile moves */
#define profileUseAxisString            "PROFILE_USE_AXIS"
#define profilePositionsString          "PROFILE_POSITIONS"
#define profileReadbacksString          "PROFILE_READBACKS"
#define profileFollowingErrorsString    "PROFILE_FOLLOWING_ERRORS"

/* bits in status word */
#define STATUS_BIT_DIRECTION       (1<<0)
#define STATUS_BIT_DONE            (1<<1)
#define STATUS_BIT_HIGH_LIMIT      (1<<2)
#define STATUS_BIT_AT_HOME         (1<<3)
#define STATUS_BIT_SLIP            (1<<4)
#define STATUS_BIT_POWERED         (1<<5)
#define STATUS_BIT_FOLLOWING_ERROR (1<<6)
#define STATUS_BIT_HOME            (1<<7)
#define STATUS_BIT_HAS_ENCODER     (1<<8)
#define STATUS_BIT_PROBLEM         (1<<9)
#define STATUS_BIT_MOVING          (1<<10)
#define STATUS_BIT_GAIN_SUPPORT    (1<<11)
#define STATUS_BIT_COMMS_ERROR     (1<<12)
#define STATUS_BIT_LOW_LIMIT       (1<<13)
#define STATUS_BIT_HOMED           (1<<14)


typedef struct MotorConfigRO {
  double motorHighLimitRaw;   /**< Read only high soft limit from controller */
  double motorLowLimitRaw;    /**< Read only low soft limit from controller */
} MotorConfigRO;

/** The structure that is passed back to devMotorAsyn when the status changes. */
typedef struct MotorStatus {
  double position;           /**< Commanded motor position */
  double encoderPosition;    /**< Actual encoder position */
  double velocity;           /**< Actual velocity */
  epicsUInt32 status;        /**< Word containing status bits (motion done, limits, etc.) */
  epicsUInt32 flagsValue;    /**< Word containing flag bits: The Value  */
  epicsUInt32 flagsWritten;  /**< Word containing flag bits: Which bits had been written by the controller  */
  int         positionWritten; /**< Commanded motor position is valid == has been written by the driver */
  struct MotorConfigRO MotorConfigRO;
} MotorStatus;

#define MOVE_TYPE_ABS 1
#define MOVE_TYPE_REL 2
#define MOVE_TYPE_VELO 3

/* Same as in motor.h */
typedef struct {
    double pos;
    double mres;
    double accEGU;
    double vbas;
    double vel;
    int moveType;
} genericMessage_type;


enum ProfileTimeMode{
  PROFILE_TIME_MODE_FIXED,
  PROFILE_TIME_MODE_ARRAY
};

enum ProfileMoveMode{
  PROFILE_MOVE_MODE_ABSOLUTE,
  PROFILE_MOVE_MODE_RELATIVE
};

/* State codes for Build, Read and Execute. Careful, these must match the
 * corresponding MBBI records, but there is no way to check this */
enum ProfileBuildState{
  PROFILE_BUILD_DONE,
  PROFILE_BUILD_BUSY
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

/* Latest command, needed for MsgTxt */
enum LatestCommand {
  LATEST_COMMAND_UNDEFINED,
  LATEST_COMMAND_STOP,
  LATEST_COMMAND_HOMING,
  LATEST_COMMAND_MOVE_TO_HOME,
  LATEST_COMMAND_MOVE_ABS,
  LATEST_COMMAND_MOVE_REL,
  LATEST_COMMAND_MOVE_VEL
  /* More to be added, like profile move */
};


#ifdef __cplusplus
#include <asynPortDriver.h>

class asynMotorAxis;

class epicsShareClass asynMotorController : public asynPortDriver {

  public:
  /* This is the constructor for the class. */
  asynMotorController(const char *portName, int numAxes, int numParams,
                      int interfaceMask, int interruptMask,
                      int asynFlags, int autoConnect, int priority, int stackSize);
                      
  virtual ~asynMotorController();

  asynStatus autoPowerOn(asynMotorAxis *pAxis);
  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
  virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nRead);
  virtual asynStatus readGenericPointer(asynUser *pasynUser, void *pointer);
  virtual asynStatus writeGenericPointer(asynUser *pasynUser, void *genericPointer);
  virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);
  virtual void report(FILE *fp, int details);

  /* These are the methods that are new to this class */
  virtual asynMotorAxis* getAxis(asynUser *pasynUser);
  virtual asynMotorAxis* getAxis(int axisNo);
  virtual asynStatus startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls);
  virtual asynStatus wakeupPoller();
  virtual asynStatus poll();
  virtual asynStatus setDeferredMoves(bool defer);
  double pollAll(void);
  void asynMotorPoller();  // This should be private but is called from C function
  
  /* Functions to deal with moveToHome.*/
  virtual asynStatus startMoveToHomeThread();
  void asynMotorMoveToHome();
  
  /* These are the functions for profile moves */
  virtual asynStatus initializeProfile(size_t maxPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus abortProfile();
  virtual asynStatus readbackProfile();
  
  virtual asynStatus setMovingPollPeriod(double movingPollPeriod);
  virtual asynStatus setIdlePollPeriod(double idlePollPeriod);

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
  int motorMoveToHome_;
  int motorResolution_;
  int motorEncoderRatio_;
  int motorPGain_;
  int motorIGain_;
  int motorDGain_;
  int motorHighLimit_;
  int motorLowLimit_;
  int motorClosedLoop_;
  int motorPowerAutoOnOff_;
  int motorPowerOnDelay_;
  int motorPowerOffDelay_;
  int motorPowerOffFraction_;
  int motorPostMoveDelay_;
  int motorStatus_;
  int motorUpdateStatus_;
  int motorLatestCommand_;
  int motorMessageIsFromDriver_;
  int motorMessageText_;

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

  /*
   * Flags from driver/controller to the record
   * Don't change the order here, Add new values at the end
   * Keep in sync with motor.h: #define MF_HOME
   * Don't forget to update asynMotorMotor::setIntegerParam()
   */
  int motorFlagsHomeOnLs_;
  int motorFlagsLSrampDown_;
  int motorFlagsNoStopOnLS_;
  int motorFlagsDriverUsesEGU_;
  int motorFlagsAdjAfterHomed_;
  int motorFlagsNtmUpdate_;
  int motorFlagsNotHomedProblem_;
  int motorFlagsNoTweakOnLs_;
  // These are per-motor parameters for passing additional motor record information to the driver
  int motorRecResolution_;
  int motorRecDirection_;
  int motorRecOffset_;

  int motorWaitPollsBeforeReady_;

  // Parameters from the controller to the driver and record
  int motorShowPowerOff_;
  int motorHighLimitRO_;
  int motorLowLimitRO_;
  // These are the per-controller parameters for profile moves
  int profileNumAxes_;
  int profileNumPoints_;
  int profileCurrentPoint_;
  int profileNumPulses_;
  int profileStartPulses_;
  int profileEndPulses_;
  int profileActualPulses_;
  int profileNumReadbacks_;
  int profileTimeMode_;
  int profileFixedTime_;
  int profileTimeArray_;
  int profileAcceleration_;
  int profileMoveMode_;
  int profileBuild_;
  int profileBuildState_;
  int profileBuildStatus_;
  int profileBuildMessage_;
  int profileExecute_;
  int profileExecuteState_;
  int profileExecuteStatus_;
  int profileExecuteMessage_;
  int profileAbort_;
  int profileReadback_;
  int profileReadbackState_;
  int profileReadbackStatus_;
  int profileReadbackMessage_;

  // These are the per-axis parameters for profile moves
  int profileUseAxis_;
  int profilePositions_;
  int profileReadbacks_;
  int profileFollowingErrors_;
  #define LAST_MOTOR_PARAM profileFollowingErrors_

  int numAxes_;                 /**< Number of axes this controller supports */
  asynMotorAxis **pAxes_;       /**< Array of pointers to axis objects */
  epicsEventId pollEventId_;    /**< Event ID to wake up poller */
  epicsEventId moveToHomeId_;    /**< Event ID to wake up move to home thread */
  double idlePollPeriod_;       /**< The time between polls when no axes are moving */
  double movingPollPeriod_;     /**< The time between polls when any motor is moving */
  int    forcedFastPolls_;      /**< The number of forced fast polls when the poller wakes up */
 
  size_t maxProfilePoints_;     /**< Maximum number of profile points */
  double *profileTimes_;        /**< Array of times per profile point */

  int moveToHomeAxis_;

  /* These are convenience functions for controllers that use asynOctet interfaces to the hardware */
  asynStatus writeController();
  asynStatus writeController(const char *output, double timeout);
  asynStatus writeReadController();
  asynStatus writeReadController(const char *output, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);
  asynUser *pasynUserController_;
  asynStatus asynStatusConnected_;
  char outString_[MAX_CONTROLLER_STRING_SIZE];
  char inString_[MAX_CONTROLLER_STRING_SIZE];

  friend class asynMotorAxis;
};
#define NUM_MOTOR_DRIVER_PARAMS (&LAST_MOTOR_PARAM - &FIRST_MOTOR_PARAM + 1)

#endif /* _cplusplus */
#endif /* asynMotorController_H */
