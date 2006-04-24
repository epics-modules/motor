/*
FILENAME...     drvOmsPC68Com.h
USAGE... This file contains information common to all OMS PC68/78 controllers.

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2006-04-24 18:11:59 $
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
#include "asynOctetSyncIO.h"


/* OmsPC68 specific data is stored in this structure. */
struct OmsPC68controller
{
    asynUser *pasynUser;        /* For RS-232 */
    char asyn_port[80];         /* asyn port name */
    CommStatus status;          /* Controller communication status. */
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
#define	PID_QUERY	"?KP"

/* Function prototypes. */
extern RTN_STATUS OmsPC68Setup(int, int);
extern RTN_STATUS OmsPC68Config(int, const char *);

#endif  /* INCdrvOmsPC68Comh */

