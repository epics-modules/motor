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
 */

#ifndef	INCdrvPM304h
#define	INCdrvPM304h 1

#include "motordrvCom.h"

/* PM304 default profile. */

#define PM304_NUM_CARDS           4
#define PM304_NUM_CHANNELS        1

#define OUTPUT_TERMINATOR "\r"
#define INPUT_TERMINATOR  '\n'

struct PM304controller
{
    void *serialInfo;       /* For RS-232 */
    int serial_card;        /* Card on which Hideos/MPF is running */
    char serial_task[20];   /* Hideos/MPF task/server name for serial port */
};

#endif	/* INCdrvPM304h */
