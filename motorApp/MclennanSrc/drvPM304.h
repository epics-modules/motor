/* File: drvPM304.h             */
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

#ifndef	INCdrvPM304h
#define	INCdrvPM304h 1

#include "motordrvCom.h"
#include "asynSyncIO.h"

/* PM304 default profile. */

#define PM304_NUM_CARDS           4
#define PM304_MAX_CHANNELS        10

#define MODEL_PM304 0
#define MODEL_PM600 1

#define OUTPUT_TERMINATOR "\r"
#define INPUT_TERMINATOR  "\n"

struct PM304controller
{
    asynUser *pasynUser;    /* asyn */
    int n_axes;             /* Number of axes on this controller */
    int model;              /* Model = MODEL_PM304 or MODEL_PM600 */
    int use_encoder[PM304_MAX_CHANNELS];  /* Does axis have an encoder? */
    char port[80];          /* asyn port name */
};

RTN_STATUS PM304Setup(int, int, int);
RTN_STATUS PM304Config(int, const char *, int);

#endif	/* INCdrvPM304h */
