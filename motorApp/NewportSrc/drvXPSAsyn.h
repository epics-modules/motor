#ifndef DRV_MOTOR_XPS_ASYN_H
#define DRV_MOTOR_XPS_ASYN_H

#ifdef __cplusplus
extern "C" {
#endif

int XPSSetup(int numControllers); /* number of XPS controllers in system.  */

int XPSConfig(int card,           /* Controller number */
              const char *ip,     /* XPS IP address or IP name */
              int port,           /* IP port number that XPS is listening on */
              int numAxes,        /* Number of axes this controller supports */
              int movingPollRate, /* Rate to poll (Hz) when an axis is in motion */
              int idlePollRate);  /* Rate to poll (Hz) when an axis is idle */

int XPSConfigAxis(int card,                    /* specify which controller 0-up*/
                  int axis,                    /* axis number 0-7 */
                  const char *positionerName); /* groupName.positionerName e.g. Diffractometer.Phi */

#ifdef __cplusplus
}
#endif
#endif
