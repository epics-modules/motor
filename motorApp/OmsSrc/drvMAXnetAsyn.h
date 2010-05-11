#ifndef DRV_MOTOR_MAXNET_ASYN_H
#define DRV_MOTOR_MAXNET_ASYN_H


#ifdef __cplusplus
extern "C" {
#endif

int MAXnetSetup(int numControllers);    /* number of MAXnet cards in system.  */

int MAXnetConfig(int card,              /* Controller number */
              const char *portName,     /* MAXnet Device name */
              int numAxes,              /* Number of axes this controller supports */
              int movingPollPeriod,     /* Time to poll (msec) when an axis is in motion */
              int idlePollPeriod,       /* Time to poll (msec) when an axis is idle. 0 for no polling */
              const char *initString);  /* Init String sent to card */

void MAXnetSendRead(int card,               /* number of card */
                  const char *sendstr);     /* string to send*/

#ifdef __cplusplus
}
#endif
#endif
