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
 */

#ifndef	INCdrvMicosh
#define	INCdrvMicosh 1

#include "motor.h"
#include "motordrvCom.h"
#include "serialIO.h"

/* Micos default profile. */

#define MICOS_NUM_CARDS   16
#define MICOS_NUM_AXIS    16
#define CTLA               1
#define OUTPUT_TERMINATOR "\r"

struct MicosController
{
    serialIO *serialInfo;       /* For RS-232 */
    int serial_card;        /* Card on which Hideos/MPF is running */
    char serial_task[20];   /* Hideos/MPF task/server name for serial port */
};

/* Function prototypes. */
extern RTN_STATUS MicosSetup(int, int, int);
extern RTN_STATUS MicosConfig(int, int, const char *);

#endif	/* INCdrvMicosh */
