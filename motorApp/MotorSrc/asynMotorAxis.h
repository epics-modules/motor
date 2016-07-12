/* asynMotorAxis.h 
 * 
 * Mark Rivers
 *
 * This file defines the base class for an asynMotorAxis.  It is the class
 * from which real motor axes are derived.
 */
#ifndef asynMotorAxis_H
#define asynMotorAxis_H

#include <epicsEvent.h>
#include <epicsTypes.h>

#ifdef __cplusplus
#include <asynPortDriver.h>

#include "asynMotorController.h"

/** Class from which motor axis objects are derived. */
class epicsShareClass asynMotorAxis {

  public:
  /* This is the constructor for the class. */
  asynMotorAxis(class asynMotorController *pController, int axisNumber);
  virtual ~asynMotorAxis();

  virtual asynStatus setIntegerParam(int index, int value);
  virtual asynStatus setDoubleParam(int index, double value);
  virtual asynStatus setStringParam(int index, const char *value);
  virtual void report(FILE *fp, int details);
  virtual asynStatus callParamCallbacks();

  virtual asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  virtual asynStatus stop(double acceleration);
  virtual asynStatus poll(bool *moving);
  virtual asynStatus setPosition(double position);
  virtual asynStatus setEncoderPosition(double position);
  virtual asynStatus setHighLimit(double highLimit);
  virtual asynStatus setLowLimit(double lowLimit);
  virtual asynStatus setPGain(double pGain);
  virtual asynStatus setIGain(double iGain);
  virtual asynStatus setDGain(double dGain);
  virtual asynStatus setClosedLoop(bool closedLoop);
  virtual asynStatus setEncoderRatio(double ratio);
  virtual asynStatus doMoveToHome();

  virtual asynStatus initializeProfile(size_t maxPoints);
  virtual asynStatus defineProfile(double *positions, size_t numPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus abortProfile();
  virtual asynStatus readbackProfile();

  void setReferencingModeMove(int distance);
  int getReferencingModeMove();

  int getWasMovingFlag();
  void setWasMovingFlag(int wasMoving);
  int getDisableFlag();
  void setDisableFlag(int disableFlag);
  double getLastEndOfMoveTime();
  void setLastEndOfMoveTime(double time);

  protected:
  class asynMotorController *pC_;    /**< Pointer to the asynMotorController to which this axis belongs.
                                      *   Abbreviated because it is used very frequently */
  int axisNo_;                       /**< Index number of this axis (0 - pC_->numAxes_-1) */
  asynUser *pasynUser_;              /**< asynUser connected to this axis for asynTrace debugging */
  double *profilePositions_;         /**< Array of target positions for profile moves */
  double *profileReadbacks_;         /**< Array of readback positions for profile moves */
  double *profileFollowingErrors_;   /**< Array of following errors for profile moves */   
  int referencingMode_;
  MotorStatus status_;
  int statusChanged_;
  
  private:
  int referencingModeMove_;
  int wasMovingFlag_;
  int disableFlag_;
  double lastEndOfMoveTime_;
  
  friend class asynMotorController;
};

#endif /* _cplusplus */
#endif /* asynMotorAxis_H */
