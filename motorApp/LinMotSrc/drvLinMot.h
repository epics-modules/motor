/* File: drvLinMot.h             */
/* Version: 2.0                 */
/* Date Last Modified: 09-29-99 */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Mark Rivers
 *      Current Author: Mark Rivers
 *      Date: 11/20/98
 *
 * Modification Log:
 * -----------------
 * .01  11/20/98  mlr  initialized from drvMM4000.h
 * .02  09/29/99  mlr  re-wrote for new version of motor software (V4)
 * .03  02/11/03  mlr  Added support for PM600 model
 */

#ifndef	INCdrvLinMoth
#define	INCdrvLinMoth 1

#include "motordrvCom.h"
#include "asynOctetSyncIO.h"

/* LinMot default profile. */

#define LinMot_NUM_CARDS           4
#define LinMot_MAX_CHANNELS        10

#define MODEL_LinMot 0
#define MODEL_PM600 1

struct LinMotController
{
    asynUser *pasynUser;    /* asyn */
    int n_axes;             /* Number of axes on this controller */
    char port[80];          /* asyn port name */
    int speed_resolution = 190735;   /* The motor speed resolution. Default set to LinMot standard */
};

RTN_STATUS LinMotSetup(int, int);
RTN_STATUS LinMotConfig(int, const char *, int);

#endif	/* INCdrvLinMoth */
