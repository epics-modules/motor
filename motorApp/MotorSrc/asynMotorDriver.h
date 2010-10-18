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
#define motorHighLimString              "MOTOR_HIGH_LIMIT"
#define motorLowLimString               "MOTOR_LOW_LIMIT"
#define motorSetClosedLoopString        "MOTOR_SET_CLOSED_LOOP"
#define motorStatusString               "MOTOR_STATUS"
#define motorUpdateStatusString         "MOTOR_UPDATE_STATUS"
#define motorStatusDirectionString      "MOTOR_STATUS_DIRECTION" 
#define motorStatusDoneString           "MOTOR_STATUS_DONE"
#define motorStatusHighLimitString      "MOTOR_STATUS_HIGHLIMIT"
#define motorStatusAtHomeString         "MOTOR_STATUS_ATHOME"
#define motorStatusSlipString           "MOTOR_STATUS_SLIP"
#define motorStatusPowerOnString        "MOTOR_STATUS_POWERED"
#define motorStatusFollowingErrorString "MOTOR_STATUS_FOLLOWINGERROR"
#define motorStatusHomeString           "MOTOR_STATUS_HOME"
#define motorStatusHasEncoderString     "MOTOR_STATUS_HASENCODER"
#define motorStatusProblemString        "MOTOR_STATUS_PROBLEM"
#define motorStatusMovingString         "MOTOR_STATUS_MOVING"
#define motorStatusGainSupportString    "MOTOR_STATUS_GAINSUPPORT"
#define motorStatusCommsErrorString     "MOTOR_STATUS_COMMSERROR"
#define motorStatusLowLimitString       "MOTOR_STATUS_LOWLIMIT"
#define motorStatusHomedString          "MOTOR_STATUS_HOMED"

typedef struct MotorStatus {
    double position;
    double encoder_posn;
    double velocity;
    epicsUInt32 status;
} MotorStatus;

#ifdef __cplusplus
#include <asynPortDriver.h>
/** Class from which motor drivers are directly derived. */
class asynMotorDriver : public asynPortDriver {
public:
    /* This is the constructor for the class. */
    epicsShareFunc asynMotorDriver(const char *portName, int maxAxes, int numParams,
                    int interfaceMask, int interruptMask,
                    int asynFlags, int autoConnect, int priority, int stackSize);

    /* These are the methods that we override from asynPortDriver */
    epicsShareFunc virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    epicsShareFunc virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    epicsShareFunc virtual asynStatus readGenericPointer(asynUser *pasynUser, void *pointer);
    epicsShareFunc virtual asynStatus setIntegerParam(int list, int index, int value);
    epicsShareFunc virtual asynStatus setDoubleParam(int list, int index, double value);
    epicsShareFunc virtual asynStatus callParamCallbacks(int list, int addr);

    /* These are the methods that are new to this class */
    epicsShareFunc virtual asynStatus moveAxis(asynUser *pasynUser, double position, int relative, double min_velocity, double max_velocity, double acceleration);
    epicsShareFunc virtual asynStatus moveVelocityAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration);
    epicsShareFunc virtual asynStatus homeAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration, int forwards);
    epicsShareFunc virtual asynStatus stopAxis(asynUser *pasynUser, double acceleration);
    epicsShareFunc virtual asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
    epicsShareFunc virtual asynStatus triggerProfile(asynUser *pasynUser);

protected:
    int motorMoveRel;
    #define FIRST_MOTOR_PARAM motorMoveRel
    int motorMoveAbs;
    int motorMoveVel;
    int motorHome;
    int motorStop;
    int motorVelocity;
    int motorVelBase;
    int motorAccel;
    int motorPosition;
    int motorEncoderPosition;
    int motorDeferMoves;
    int motorResolution;
    int motorEncRatio;
    int motorPgain;
    int motorIgain;
    int motorDgain;
    int motorHighLim;
    int motorLowLim;
    int motorSetClosedLoop;
    int motorStatus;
    int motorUpdateStatus;
    int motorStatusDirection;
    int motorStatusDone;
    int motorStatusHighLimit;
    int motorStatusAtHome;
    int motorStatusSlip;
    int motorStatusPowerOn;
    int motorStatusFollowingError;
    int motorStatusHome;
    int motorStatusHasEncoder;
    int motorStatusProblem;
    int motorStatusMoving;
    int motorStatusGainSupport;
    int motorStatusCommsError;
    int motorStatusLowLimit;
    int motorStatusHomed;
    #define LAST_MOTOR_PARAM motorStatusHomed

private:
    MotorStatus *axisStatus;
    int *axisStatusChanged;
};
#define NUM_MOTOR_DRIVER_PARAMS (&LAST_MOTOR_PARAM - &FIRST_MOTOR_PARAM + 1)

#endif /* _cplusplus */
#endif /* asynMotorDriver_H */
