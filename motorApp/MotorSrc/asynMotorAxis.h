/* asynMotorAxis.h 
 * 
 * Mark Rivers
 *
 * This file defines the base class for an asynMotoAxis.  It is the class
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
class epicsShareFunc asynMotorAxis {

  public:
  /* This is the constructor for the class. */
  asynMotorAxis(class asynMotorController *pController, int axisNumber);

  virtual asynStatus setIntegerParam(int index, int value);
  virtual asynStatus setDoubleParam(int index, double value);
  virtual asynStatus callParamCallbacks();

  virtual asynStatus move(double position, bool relative, double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  virtual asynStatus home(double minVelocity, double maxVelocity, double acceleration, bool forwards);
  virtual asynStatus stop(double acceleration);
  virtual asynStatus poll(bool *moving);
  virtual asynStatus setPosition(double position);

  virtual asynStatus initializeProfile(size_t maxPoints);
  virtual asynStatus defineProfile(double *positions, size_t numPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus abortProfile();
  virtual asynStatus readbackProfile();
          int        getAxisIndex()         {return axisNo_;}
  inline  void       setStatusChanged()     {statusChanged_ = true;}
  inline  void       clearStatusChanged()   {statusChanged_ = false;}
  inline  double *   getprofileReadbacks()  {return profileReadbacks_;}
  inline  double *   getprofileFollowingErrors() {return profileFollowingErrors_;}

  protected:
  class asynMotorController *pC_;    /**< Pointer to the asynMotorController to which this axis belongs.
                                      *   Abbreviated because it is used very frequently */
  int axisNo_;                       /**< Index number of this axis (0 - pC_->numAxes_-1) */
  asynUser *pasynUser_;              /**< asynUser connected to this axis for asynTrace debugging */
  double *profilePositions_;         /**< Array of target positions for profile moves */
  double *profileReadbacks_;         /**< Array of readback positions for profile moves */
  double *profileFollowingErrors_;   /**< Array of following errors for profile moves */   

  private:
  MotorStatus status_;
  bool statusChanged_;
};

#endif /* _cplusplus */
#endif /* asynMotorAxis_H */
