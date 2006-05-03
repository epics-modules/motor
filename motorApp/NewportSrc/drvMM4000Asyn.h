#ifndef DRV_MOTOR_MM4000_ASYN_H
#define DRV_MOTOR_MM4000_ASYN_H

#ifdef __cplusplus
extern "C" {
#endif

int MM4000AsynSetup(int numControllers); /* number of XPS controllers in system.  */

int MM4000AsynConfig(int card,             /* Controller number */
                     const char *portName, /* asyn port name of serial or GPIB port */
                     int asynAddress,      /* asyn subaddress for GPIB */
                     int numAxes,          /* Number of axes this controller supports */
                     int movingPollPeriod, /* Time to poll (msec) when an axis is in motion */
                     int idlePollPeriod);  /* Time to poll (msec) when an axis is idle. 0 for no polling */

#ifdef __cplusplus
}
#endif
#endif
