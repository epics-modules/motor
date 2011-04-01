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

  // These are pure virtual functions which derived classes must implement
  /** Move the motor to an absolute location or by a relative amount.
    * \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move 
    * by (if relative=1). Units=steps.
    * \param[in] relative  Flag indicating relative move (1) or absolute move (0).
    * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
    * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
    * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
  virtual asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) = 0;

  /** Move the motor at a fixed velocity until told to stop.
    * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
    * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
    * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
  virtual asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration) = 0;

  /** Move the motor to the home position.
    * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
    * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
    * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
    * \param[in] forwards  Flag indicating to move the motor in the forward direction(1) or reverse direction(0).
    *                      Some controllers need to be told the direction, others know which way to go to home. */
  virtual asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards) = 0;

  /** Stop the motor.
    * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
  virtual asynStatus stop(double acceleration) = 0;

  /** Poll the axis.
    * This function should read the controller position, encoder position, and as many of the motorStatus flags
    * as the hardware supports.  It should call setIntegerParam() and setDoubleParam() for each item that it polls,
    * and then call callParamCallbacks() at the end.
    * \param[out] moving A flag that the function must set indicating that the axis is moving (1) or done (0). */
  virtual asynStatus poll(int *moving) = 0;

  /** Set the current position of the motor.
    * \param[in] position The new absolute motor position that should be set in the hardware. Units=steps.*/
  virtual asynStatus setPosition(double position) = 0;

  virtual asynStatus initializeProfile(int maxPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus readbackProfile();

protected:
  class asynMotorController *pC_;    /**< Pointer to the asynMotorController to which this axis belongs.
                                      *   Abbreviated because it is used very frequently */
  int axisNo_;                       /**< Index number of this axis (0 - pC_->numAxes_-1) */
  asynUser *pasynUser_;              /**< asynUser connected to this axis for asynTrace debugging */

private:
  MotorStatus status_;
  int statusChanged_;
  double *profilePositions_;         /**< Array of target positions for profile moves */
  double *profilePositionsRBV_;      /**< Array of readback positions for profile moves */
  double *profileFollowingErrors_;   /**< Array of following errors for profile moves */   
  
friend class asynMotorController;
};

#endif /* _cplusplus */
#endif /* asynMotorAxis_H */
