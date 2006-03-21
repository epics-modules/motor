#ifndef DRV_MOTOR_SIM_H
#define DRV_MOTOR_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

void motorSimCreate( int card, int axis, double hiLimit, double lowLimit, double home, int nCards, int nAxes );

#ifdef __cplusplus
}
#endif
#endif
