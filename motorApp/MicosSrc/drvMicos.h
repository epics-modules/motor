/* File: drvMicos.h             */


/* Device Driver Support definitions for Micos MoCo dc motor controller. */
/*
 *      Original Author: Kurt Goetze
 *      Current Author: Kurt Goetze
 *      Date: 11/24/2003
 *
 * Modification Log:
 * -----------------
 * .00  11/24/2003  kag  initialized from drvMCB4B.h
 * .01  07/12/2004  rls  Converted from MPF to asyn.
 */

#ifndef	INCdrvMicosh
#define	INCdrvMicosh 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynSyncIO.h"

/* Micos default profile. */

#define MICOS_NUM_CARDS   16
#define MICOS_NUM_AXIS    16
#define CTLA               1
#define OUTPUT_TERMINATOR "\r"

struct MicosController
{
    asynUser *pasynUser;  	/* For RS-232 */
    char asyn_port[80];     	/* asyn port name */
};

/* Function prototypes. */
extern RTN_STATUS MicosSetup(int, int, int);
extern RTN_STATUS MicosConfig(int, const char *);

#endif	/* INCdrvMicosh */
