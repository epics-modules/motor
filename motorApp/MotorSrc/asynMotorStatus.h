/*
FILENAME...	asynMotorStatus.h
USAGE...	

Version:	$Revision$
Modified By:	$Author$
Last Modified:	$Date$
*/

#ifndef asynMotorStatus_H
#define asynMotorStatus_H

#include "asynMotorController.h"
#include <epicsTypes.h>

/** The structure that is passed back to devMotorAsyn when the status changes. */
typedef struct MotorStatus {
  double position;           /**< Commanded motor position */
  double encoderPosition;    /**< Actual encoder position */
  double velocity;           /**< Actual velocity */
  epicsUInt32 status;        /**< Word containing status bits (motion done, limits, etc.) */
} MotorStatus;



#ifdef __cplusplus
#include "asynPortDriverExt.h"

typedef enum Direction {PLUS = 0, MINUS = 1} DIRECTION;

class asynMotorStatus : public asynPortDriverExt
{
public:
  asynMotorStatus(asynMotorController *pC, int axisNo);
  static int        getNumParams();
  virtual asynStatus createParams();
          asynStatus setDoneMoving(bool done);
          asynStatus setMoving(bool moving);
          asynStatus setProblem(bool on);
          asynStatus setHasGainSupport(bool yes);
          asynStatus setHasEncoder(bool yes);
          asynStatus setAtHome(bool on);
          asynStatus setHighLimitOn(bool on);
          asynStatus setLowLimitOn(bool on);
          asynStatus setDirection(Direction way);
          asynStatus setPowerOn(bool on);
          void setStatusChanged();
          bool hasStatusChanged();
          void clearStatusChanged();
          asynStatus setIntegerParam(int function, int value);
          epicsUInt32 getStatus();

protected:
  int axisNo_;
  epicsUInt32 status_;
  bool statusChanged_;
  // These are the status bits
#define FIRST_STATUS_PARAM motorStatusDirection_
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
#define LAST_STATUS_PARAM motorStatusHomed_
  
};

#define NUM_STATUS_PARAMS 15
#endif /*__cplusplus */
#endif /*asynMotorStatus_H */

