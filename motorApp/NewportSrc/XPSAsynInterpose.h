#include "motor_interface.h"

typedef enum {
    minJerkTime = MOTOR_AXIS_NUM_PARAMS,
    maxJerkTime,
    XPSStatus
} XPSCommand;

#define XPS_NUM_PARAMS 3

