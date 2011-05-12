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
//          asynStatus getStatusDone(int *done);
          asynStatus setProblem(bool on);
          asynStatus setHasGainSupport(bool yes);
          asynStatus setHasEncoder(bool yes);
          asynStatus setAtHome(bool on);
          asynStatus setHighLimitOn(bool on);
          asynStatus setLowLimitOn(bool on);
          asynStatus setDirection(Direction way);
          asynStatus setPowerOn(bool on);

protected:
  asynStatus setIntegerParam(int function, int value);
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

