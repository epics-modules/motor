#ifndef DRV_MOTOR_ASYN_H
#define DRV_MOTOR_ASYN_H

#include "motor_interface.h"

#include <limits.h>

#define NMASKBITS (sizeof(int) * CHAR_BIT)
#define BIT_ISSET(bit, n, mask) ( ((bit)/NMASKBITS < (n)) && \
            ((mask)[(bit)/NMASKBITS] & (1 << ((bit) % NMASKBITS))) )
#define BIT_SET(bit, mask, value) do { \
                  (mask)[(bit)/NMASKBITS] &= ~(1 << ((bit) % NMASKBITS)); \
                  if (value) { \
                      (mask)[(bit)/NMASKBITS] |= (1 << ((bit) % NMASKBITS)); \
                  } \
              } while (0);

typedef enum {
    /* Parameters - these must match the definitions in motor_interface.h */
    motorPosition = motorAxisPosition,
    motorEncRatio = motorAxisEncoderRatio,
    motorResolution = motorAxisResolution,
    motorPgain = motorAxisPGain,
    motorIgain = motorAxisIGain,
    motorDgain = motorAxisDGain,
    motorHighLim = motorAxisHighLimit,
    motorLowLim = motorAxisLowLimit,
    motorSetClosedLoop = motorAxisClosedLoop,
    motorEncoderPosition = motorAxisEncoderPosn,
    /* Not exposed by the driver */
    motorVelocity=100, motorVelBase, motorAccel, 
    /* Commands */
    motorMoveRel, motorMoveAbs, motorMoveVel, motorHome, motorStop,
    /* Status readback */
    motorStatus,
    /* Status bits split out */
    motorStatusDirection=motorAxisDirection,
    motorStatusDone, motorStatusHighLimit, motorStatusAtHome,
    motorStatusSlip, motorStatusPowerOn, motorStatusFollowingError,
    motorStatusHome, motorStatusHasEncoder, motorStatusProblem,
    motorStatusMoving, motorStatusGainSupport, motorStatusCommsError,
    motorStatusLowLimit, motorStatusLast
} motorCommand; 

#endif
