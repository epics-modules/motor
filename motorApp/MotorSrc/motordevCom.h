/*
FILENAME...	motordevCom.h
USAGE...	This file contains definitions and structures that
		are common to all motor record device support modules.

Version:	$Revision$
Modified By:	$Author$
Last Modified:	$Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 12/22/98
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01 12-12-03 rls - Converted MSTA #define's to bit field.
 */


#ifndef	INCmotordevCom
#define	INCmotordevCom 1

#include <epicsEvent.h>
#include "motor.h"
#include "motordrvCom.h"

#define IDLE_STATE 0
#define BUILD_STATE 1

/* Axis status. */
struct axis_stat
{
    bool in_use;	/* Indicates axis assigned to a motor record. */
};

/* Controller board status. */
struct board_stat
{
    int exists;
    char *name;
    int total_axis;
    struct axis_stat *axis_stat;
};

/* the device private data structure of the record */
struct motor_trans
{
    int state;
    epicsEvent *lock;
    struct mess_node motor_call;
    int callback_changed;
    int motor_pos;
    int encoder_pos;
    int vel;
    msta_field status;
    epicsEvent *initSem;
    struct driver_table *tabptr;
    bool dpm;		/* For OMS VME58 only, drive power monitoring. */
};

epicsShareFunc CALLBACK_VALUE motor_update_values(struct motorRecord *);
epicsShareFunc long motor_init_com(int, int, struct driver_table *,
			   struct board_stat ***);
epicsShareFunc long motor_init_record_com(struct motorRecord *, int,
			    struct driver_table *, struct board_stat *[]);
epicsShareFunc long motor_start_trans_com(struct motorRecord *, struct board_stat *[]);
epicsShareFunc RTN_STATUS motor_end_trans_com(struct motorRecord *, struct driver_table *);

#endif	/* INCmotordevComh */
