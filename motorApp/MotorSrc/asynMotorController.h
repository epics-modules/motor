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



enum ProfileTimeMode{
  PROFILE_TIME_MODE_FIXED,
  PROFILE_TIME_MODE_ARRAY
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

#ifdef __cplusplus
#include <asynPortDriver.h>

class asynMotorAxis;

class epicsShareFunc asynMotorController : public asynPortDriver {

  public:
  /* This is the constructor for the class. */
  asynMotorController(const char *portName, int numAxes,
                      int interfaceMask, int interruptMask,
                      int asynFlags, int autoConnect, int priority, int stackSize);

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
  virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nRead);
  virtual asynStatus readGenericPointer(asynUser *pasynUser, void *pointer);

  /* These are the methods that are new to this class */
  virtual asynMotorAxis* getAxis(asynUser *pasynUser);
  virtual asynMotorAxis* getAxis(int axisNo);
  virtual asynStatus startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls);
  virtual asynStatus wakeupPoller();
  virtual asynStatus poll();
  void asynMotorPoller();  // This should be private but is called from C function
  void setAxesPtr(int index, asynMotorAxis *ptr); // Allows a subclass to initialize pAxes_[index] entry to ptr.

  /* These are the functions for profile moves */
  virtual asynStatus initializeProfile(size_t maxPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus abortProfile();
  virtual asynStatus readbackProfile();
  virtual int        getNumParams();
  inline int getMotorEncoderPositionIndex()        {return motorEncoderPosition_;}
  inline int getMotorDeferMovesIndex()             {return motorDeferMoves_;}
  inline int getMotorResolutionIndex()             {return motorResolution_;}
  inline int getMotorEncRatioIndex()               {return motorEncRatio_;}
  inline int getMotorPgainIndex()                  {return motorPgain_;}
  inline int getMotorIgainIndex()                  {return motorIgain_;}
  inline int getMotorDgainIndex()                  {return motorDgain_;}
  inline int getMotorHighLimitIndex()              {return motorHighLimit_;}
  inline int getMotorLowLimitIndex()               {return motorLowLimit_;}
  inline int getMotorSetClosedLoopIndex()          {return motorSetClosedLoop_;}
  inline int getMotorStatusIndex()                 {return motorStatus_;}
  inline int getMotorUpdateStatusIndex()           {return motorUpdateStatus_;}

  inline int getProfileNumAxesIndex()              {return profileNumAxes_;}
  inline int getProfileNumPointsIndex()            {return profileNumPoints_;}
  inline int getProfileCurrentPointIndex()         {return profileCurrentPoint_;}
  inline int getProfileNumPulsesIndex()            {return profileNumPulses_;}
  inline int getProfileStartPulsesIndex()          {return profileStartPulses_;}
  inline int getProfileEndPulsesIndex()            {return profileEndPulses_;}
  inline int getProfileActualPulsesIndex()         {return profileActualPulses_;}
  inline int getProfileNumReadbacksIndex()         {return profileNumReadbacks_;}
  inline int getProfileTimeModeIndex()             {return profileTimeMode_;}
  inline int getProfileFixedTimeIndex()            {return profileFixedTime_;}
  inline int getProfileTimeArrayIndex()            {return profileTimeArray_;}
  inline int getProfileAccelerationIndex()         {return profileAcceleration_;}
  inline int getProfileBuildIndex()                {return profileBuild_;}
  inline int getProfileBuildStateIndex()           {return profileBuildState_;}
  inline int getProfileBuildStatusIndex()          {return profileBuildStatus_;}
  inline int getProfileBuildMessageIndex()         {return profileBuildMessage_;}
  inline int getProfileExecuteIndex()              {return profileExecute_;}
  inline int getProfileExecuteStateIndex()         {return profileExecuteState_;}
  inline int getProfileExecuteStatusIndex()        {return profileExecuteStatus_;}
  inline int getProfileExecuteMessageIndex()       {return profileExecuteMessage_;}
  inline int getProfileAbortIndex()                {return profileAbort_;}
  inline int getProfileReadbackIndex()             {return profileReadback_;}
  inline int getProfileReadbackStateIndex()        {return profileReadbackState_;}
  inline int getProfileReadbackStatusIndex()       {return profileReadbackStatus_;}
  inline int getprofileReadbackMessageIndex()      {return profileReadbackMessage_;}

  inline int getProfileUseAxisIndex()              {return profileUseAxis_;}
  inline int getProfilePositionsIndex()            {return profilePositions_;}
  inline int getProfileReadbacksIndex()            {return profileReadbacks_;}
  inline int getProfileFollowingErrorsIndex()      {return profileFollowingErrors_;}
  inline int getProfileMotorResolutionIndex()      {return profileMotorResolution_;}
  inline int getProfileMotorDirectionIndex()       {return profileMotorDirection_;}
  inline int getProfileMotorOffsetIndex()          {return profileMotorOffset_;}
  
  inline int getNumAxes()                          {return numAxes_;}
  inline size_t getMaxProfilePoints()              {return maxProfilePoints_;}

  int shuttingDown_;   /**< Flag indicating that IOC is shutting down.  Stops poller */

  protected:
  /** These are the index numbers for the parameters in the parameter library.
   * They are the values of pasynUser->reason in calls from device support */
   // These are the motor commands
  virtual asynStatus createDriverParams();
  #define FIRST_MOTOR_PARAM motorEncoderPosition_
//  int motorStop_;
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
 
  size_t maxProfilePoints_;     /**< Maximum number of profile points */
  double *profileTimes_;        /**< Array of times per profile point */
};
#define NUM_MOTOR_DRIVER_PARAMS (&LAST_MOTOR_PARAM - &FIRST_MOTOR_PARAM + 1)

#endif /* _cplusplus */
#endif /* asynMotorController_H */
