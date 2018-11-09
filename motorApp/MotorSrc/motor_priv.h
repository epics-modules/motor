/* motor_priv.h: private fields, which could be outside the record */

#ifndef INC_motor_priv_H
#define INC_motor_priv_H

#include "epicsTypes.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

  struct motor_priv {
    struct {
      epicsFloat64 position;           /**< Commanded motor position */
      epicsFloat64 encoderPosition;    /**< Actual encoder position */
    } readBack;
    struct {
      epicsFloat64 motorDialHighLimitRO;   /**< read only high limit from controller. */
      epicsFloat64 motorDialLowLimitRO;    /**< read only low limit from controller. */
      int motorDialLimitsValid;
    } softLimitRO;
    struct {
      msta_field msta;
    } lastReadBack;
    struct {
      epicsFloat64 motorMaxVelocityDial; /**< Maximum velocity */
      epicsFloat64 motorDefVelocityDial; /**< Default velocity (moveAbs, moveRel) */
      epicsFloat64 motorDefJogVeloDial;  /**< Default velocity (for jogging) */
      epicsFloat64 motorDefJogAccDial;   /**< Default accelation (steps/sec2 or motorUnits/sec2) */
      epicsFloat64 motorDefHomeVeloDial; /**< Default velocity (for homing) */
      epicsFloat64 motorSDBDDial;        /**< Minimal movement */
      epicsFloat64 motorRDBDDial;        /**< "At target position" deadband */
      epicsFloat64 motorERESDial;        /**< ERES as configured in the controller */
      epicsFloat64 motorSREV;            /**< SREV as configured in the controller */
      epicsFloat64 motorUREV;            /**< UREV as configured in the controller */
    } configRO;
    struct {
      epicsFloat64 motorHighLimitRaw;  /* last from dev support in status */
      epicsFloat64 motorLowLimitRaw;   /* last from dev support in status */
      epicsFloat64 val;               /* last .VAL */
      epicsFloat64 dval;               /* last .DVAL */
      epicsFloat64 commandedDval;      /* Where did we tell the motor to go */
      epicsInt32   rval;               /* last .RVAL */
      epicsFloat64 rlv;                /* Last Rel Value (EGU) */
      epicsFloat64 alst;               /* Last Value Archived */
      epicsFloat64 mlst;               /* Last Val Monitored */
    } last;
  };

#ifdef __cplusplus
}
#endif
#endif /* INC_motor_priv_H */
