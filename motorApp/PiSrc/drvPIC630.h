/* File: drvPIC630.h             */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Kurt Goetze
 *      Current Author: Kurt Goetze
 *      Date: 02/07/2005
 *
 * Modification Log:
 * -----------------
 * .00  02/07/2005  kag  initialized from drvMicos.h
 * .01  03/30/2005  kag  port to 3.14 / asyn
 */

#ifndef	INCdrvPIC630h
#define	INCdrvPIC630h 1

#include "motordrvCom.h"
#include "asynDriver.h"

/* PIC630 default profile. */

#define PIC630_NUM_CARDS   8   /* Maximum number of controller chains */
#define PIC630_NUM_AXIS    9   /* Maximum number of axes per chain */

#define COMM_TIMEOUT    2 /* Timeout in seconds. */

struct PIC630Controller
{
    asynUser *pasynUser;   /* asynUser structure */
    char asyn_port[80];   /* asyn port name */
};

#endif	/* INCdrvPIC630h */
