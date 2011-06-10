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
#include "asynMotorStatus.h"



/** Class from which motor axis objects are derived. */
class epicsShareFunc asynMotorAxis {

  public:
  /* This is the constructor for the class. */
  asynMotorAxis(class asynMotorController *pController, int axisNumber);
  asynStatus initializeAxis();
  virtual asynStatus createParams();
  static int        getNumParams();

  //TODO Make getXXXIndex protected after separating parameters
  inline int getMotorVelocityIndex()               {return motorVelocity_;}
  inline int getMotorPositionIndex()               {return motorPosition_;}

  virtual asynStatus setIntegerParam(int index, int value);
  virtual asynStatus setDoubleParam(int index, double value);
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
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
          asynMotorStatus* getStatus();

protected:
  virtual asynStatus preInitAxis();
  virtual asynStatus postInitAxis();

  inline int getMotorMoveRelIndex()                {return motorMoveRel_;}
   inline int getMotorMoveAbsIndex()                {return motorMoveAbs_;}
   inline int getMotorMoveVelIndex()                {return motorMoveVel_;}
   inline int getMotorHomeIndex()                   {return motorHome_;}

   inline int getMotorStopIndex()                   {return motorStop_;}
   inline int getMotorVelBaseIndex()                {return motorVelBase_;}
   inline int getMotorAccelIndex()                  {return motorAccel_;}

  class asynMotorController *pC_;    /**< Pointer to the asynMotorController to which this axis belongs.
                                      *   Abbreviated because it is used very frequently */
  int axisNo_;                       /**< Index number of this axis (0 - pC_->numAxes_-1) */
  asynUser *pasynUser_;              /**< asynUser connected to this axis for asynTrace debugging */
  double *profilePositions_;         /**< Array of target positions for profile moves */
  double *profileReadbacks_;         /**< Array of readback positions for profile moves */
  double *profileFollowingErrors_;   /**< Array of following errors for profile moves */   
  asynMotorStatus *aMotorStatus;

private:
  MotorStatus status_;
  bool statusChanged_;

  int motorMoveRel_;
  int motorMoveAbs_;
  int motorMoveVel_;
  int motorHome_;
  int motorStop_;
  int motorVelocity_;
  int motorVelBase_;
  int motorAccel_;
  int motorPosition_;
  static const int NUM_ASYN_AXIS_PARAMS = 9;

};

#endif /* _cplusplus */
#endif /* asynMotorAxis_H */
