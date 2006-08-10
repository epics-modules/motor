#ifndef DRV_MOTOR_SIM_H
#define DRV_MOTOR_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

void motorSimCreate( int card, int axis, int hiLimit, int lowLimit, int home, int nCards, int nAxes, int startPosn );

#ifdef __cplusplus
}
#endif
#endif
