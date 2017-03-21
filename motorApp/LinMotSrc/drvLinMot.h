#ifndef	INCdrvLinMoth
#define	INCdrvLinMoth 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define LinMot_MAX_MOTORS  4
#define LinMot_MSG_SIZE 80
#define LinMot_STATUS_RETRY 10

/* End-of-string defines */
#define LinMot_OUT_EOS   "\r\n" /* Command */
#define LinMot_IN_EOS    "\r\n"  /* Reply */

/* Motion Master specific data is stored in this structure. */
struct LinMotController
{
    asynUser *pasynUser;  	/* For RS-232 */
    char port[80];          /* asyn port name */
    int n_axes;     		/* Number of axes */
    int max_speed[LinMot_MAX_MOTORS]; /* steps/sec. */
};

#endif	/* INCdrvLinMoth */

