/*
FILENAME...	drvMMCom.h
USAGE... This file contains Newport Motion Master (MM) driver "include"
	    information that is specific to Motion Master models 3000/4000.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-04-18 21:01:59 $
*/

/*
 *      Original Author: Mark Rivers
 *      Current Author: Mark Rivers
 *      Date: 10/16/97
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
 *	      The Controls and Automation Group (AT-8)
 *	      Ground Test Accelerator
 *	      Accelerator Technology Division
 *	      Los Alamos National Laboratory
 *
 *      Co-developed with
 *	      The Controls and Computing Group
 *	      Accelerator Systems Division
 *	      Advanced Photon Source
 *	      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  01-18-93	mlr     initialized from drvOms58
 */

#ifndef	INCdrvMMComh
#define	INCdrvMMComh 1

#include "motordrvCom.h"

enum MM_model
{
    MM4000,
    MM4005
};

enum MM_motor_type
{
    UNUSED,
    STEPPER,
    DC
};

#ifndef __cplusplus
typedef enum MM_model MM_model;
typedef enum MM_motor_type MM_motor_type;
#endif


/* Motion Master specific data is stored in this structure. */
struct MMcontroller
{
    PortType port_type;		/* GPIB_PORT or RS232_PORT */
    struct serialInfo *serialInfo;  /* For RS-232 */
    int gpib_link;
    int gpib_address;
    struct gpibInfo *gpibInfo;  /* For GPIB */
    int serial_card;            /* Card on which Hideos is running */
    char serial_task[20];       /* Hideos task name for serial port */
    char status_string[80];     /* String containing status of motors */
    char position_string[80];   /* String containing position of motors */
    MM_model model;		/* Motion Master Model. */
    MM_motor_type type[4];	/* For MM3000 only; Motor type array. */
    /* For MM4000/5 only; controller resolution array (from TU command).
     * Units are in [Controller EGU's / Record RAW units].
     */
    double drive_resolution[MAX_AXIS];
    int res_decpts[MAX_AXIS];	/* Drive resolution significant dec. pts. */
    double home_preset[MAX_AXIS]; /* Controller's home preset position (XF command). */
    CommStatus status;		/* Controller communication status. */
};

#endif	/* INCdrvMMComh */

