/* File: drvMCB4B.h             */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Mark Rivers
 *      Current Author: Mark Rivers
 *      Date: 2/24/3002
 *
 * Modification Log:
 * -----------------
 * .01  02/24/2002  mlr  initialized from drvPM304.h
 */

#ifndef	INCdrvMCB4Bh
#define	INCdrvMCB4Bh 1

#include "motordrvCom.h"

/* MCB4B default profile. */

#define MCB4B_NUM_CARDS           4
#define MCB4B_NUM_CHANNELS        4

#define OUTPUT_TERMINATOR "\r"
#define INPUT_TERMINATOR  '\r'

struct MCB4Bcontroller
{
    void *serialInfo;       /* For RS-232 */
    int serial_card;        /* Card on which Hideos/MPF is running */
    char serial_task[20];   /* Hideos/MPF task/server name for serial port */
};

#endif	/* INCdrvMCB4Bh */
