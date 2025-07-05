/*
FILENAME...     drvOmsPC68Com.h
USAGE... This file contains information common to all OMS PC68/78 controllers.

*/

/*
 *      Original Author: Brian Tieman
 *      Current Author: Ron Sluiter
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 */

#ifndef INCdrvOmsPC68Comh
#define INCdrvOmsPC68Comh 1

#include "motor.h"
#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynOctet.h"

/* status register */
#define STAT_IRQ                0x80
#define STAT_TRANS_BUF_EMPTY    0x40
#define STAT_INPUT_BUF_FULL     0x20
#define STAT_DONE               0x10
#define STAT_OVERTRAVEL         0x08
#define STAT_ENCODER_REQ        0x04
#define STAT_UNUSED             0x02
#define STAT_ERROR              0x01
#define STAT_ERROR_MSK          0x0F

/* done flag register */
#define DONE_X	0x01
#define DONE_Y	0x02
#define DONE_Z	0x04
#define DONE_T	0x08
#define DONE_U	0x10
#define DONE_V	0x20
#define DONE_R	0x40
#define DONE_S	0x80

/* OmsPC68 specific data is stored in this structure. */
struct OmsPC68controller
{
    int card;
    int errcnt;
    char asyn_port[80];         /* asyn port name */
    CommStatus status;          /* Controller communication status. */
    asynUser* pasynUser;
    asynOctet* pasynOctet;
    void* octetPvt;
    void* registrarPvt;
};


struct encoder_status
{
    char slip_enable;
    char pos_enable;
    char slip_detect;
    char pos_dead;
    char axis_home;
    char unused;
};


#define ECHO_OFF        "EF"
#define AXIS_STOP       "ST"
#define GET_IDENT       "WY"
#define ERROR_CLEAR     "IC"
#define STOP_ALL        "AA SA"
#define ALL_POS         "AA RP"
#define ALL_INFO        "QA RP RE EA"
#define AXIS_INFO       "QA RP"
#define ENCODER_QUERY   "EA"
#define DONE_QUERY      "RA"
#define	PID_QUERY       "?KP"

/* Function prototypes. */
extern RTN_STATUS OmsPC68Setup(int, int);
extern RTN_STATUS OmsPC68Config(int, const char *);

#endif  /* INCdrvOmsPC68Comh */

