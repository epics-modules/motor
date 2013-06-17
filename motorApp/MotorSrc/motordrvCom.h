/*
FILENAME...	motordrvCom.h

USAGE...	This file contains definitions and structures that
		are common to all motor record driver support modules.

Version:	$Revision$
Modified By:	$Author$
Last Modified:	$Date$
HeadURL:        $URL$
*/

/*
 *      Current Author: Ron Sluiter
 *      Date: 12/22/98
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
 * .01 10-02-01 rls added RETRY to CommStatus enumeration.
 * .02 10-24-03 rls moved irqdatastr to OmsSrc.
 * .03 12-12-03 rls Converted MSTA #define's to bit field.
 * .04 09-20-04 rls support for 32 axes / controller, maximum.
 * .05 05/10/05 rls Added "update_delay" for "Stale data delay" bug fix.
 * .06 10/18/05 rls Added MAX_TIMEOUT for all devices drivers.
 */


#ifndef	INCmotordrvComh
#define	INCmotordrvComh 1

#include <callback.h>
#include <epicsTypes.h>
#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsMessageQueue.h>

#include "motor.h"

#define MAX_IDENT_LEN 100
#define MAX_TIMEOUT   5.0 /* Maximum communication timeout for all devices
			   drivers (in sec.). */

/* Controller communication port type, followed by status. */
enum PortType
{
    GPIB_PORT = 0,
    RS232_PORT		/* = 1 */
};

/* Controller communication status. */
enum CommStatus
{
    NORMAL,
    RETRY,		/* Communication retry. */
    COMM_ERR		/* Communication timeout error. */
};


/*
Valid message types for the driver. The order is of importance; 0 is
lowest importance.  Device support will choose the greatest one to
use as the driver transaction type.
*/

enum msg_types {
    UNDEFINED,		/* garbage type */
    IMMEDIATE,		/* execute immediate, no reply */
    MOVE_TERM,		/* terminate a previous active motion */
    MOTION,		/* move */
    VELOCITY,		/* make motion updates till MOVE_TERM */
    INFO		/* get curr motor/encoder pos and stat */
};


/* Macros used to set/clear bits in any_motor_in_motion variable. */
#define SET_MM_ON(v,a)  v|=(1<<a)
#define SET_MM_OFF(v,a) v&=~(1<<a)

/* Misc. defines. */
#define ALL_CARDS -1

#define	FLUSH -1 /* The 3rd argument of driver_table's getmsg() can indicate
		either FLUSH the buffer or the # of commands to process. */

/* message queue management - device and driver support only */
struct mess_node
{
    CALLBACK callback;	/* !!WARNING!! - MUST be 1st structure member. Calls to
		    callbackRequest(CALLBACK *) use pointer to mess_node. */
    int signal;
    int card;
    msg_types type;
    char message[MAX_MSG_SIZE];
    long position;
    long encoder_position;
    long velocity;
    msta_field status;
    struct dbCommon *mrecord;	/* "Hidden" pointer to motor record. */
    struct mess_node *next;
    char *postmsgptr;
    char const *termstring;	/* Termination string for STOP_AXIS command
				    (see process_messages()). */
};

/* initial position query to driver - device and driver support only */
typedef struct mess_card_query
{
    char *card_name;
    int total_axis;
} MOTOR_CARD_QUERY;

typedef struct mess_axis_query
{
    long position;
    long encoder_position;
    msta_field status;
} MOTOR_AXIS_QUERY;

struct axis_status
{
    char direction;
    char done;
    char overtravel;
    char home;
};

struct circ_queue	/* Circular queue structure. */
{
    struct mess_node *head;
    struct mess_node *tail;
};

/*----------------motor state info-----------------*/

struct mess_info
{
    struct mess_node *motor_motion;	/* in motion, NULL/node */
    int encoder_present;		/* one YES/NO for each axis */
    epicsInt32 position;		/* one pos for each axis */
    epicsInt32 encoder_position;	/* one pos for each axis */
    epicsInt32 velocity;		/* Raw velocity readback */
    int no_motion_count;
    epicsTime status_delay;     /* Insure 10ms delay between motion/velocity
				 * commands and status query. */
    msta_field status;		/* one pos for each axis */
    int pid_present;		/* PID control indicator for VME58 (YES/NO). */
    double high_limit;		/* MM4000 only; Controller's high travel limit. */
    double low_limit;		/* MM4000 only; Controller's low travel limit. */
};

struct controller	/* Controller board information. */
{
    int motor_in_motion;/* count of motors in motion */
    char ident[MAX_IDENT_LEN];	/* identification string for this card */
    int total_axis;	/* total axis on this card */
    char *localaddr;	/* address of this card */
    bool cmnd_response; /* Indicates controller communication response
	    * to VELOCITY, MOTION and MOVE_TERM type commands. */
    void *DevicePrivate; /* Pointer to device specific structure. */
    struct mess_info motor_info[MAX_AXIS];
};

/* The "driver_table" structure allows device level access to driver level
   data and functions; it supports "common code" for all motor record device
   drivers.
*/
struct driver_table
{
    int (*init) (void);
    RTN_STATUS (*send) (struct mess_node *, struct driver_table *);
    int (*free) (struct mess_node *, struct driver_table *);
    int (*get_card_info) (int, MOTOR_CARD_QUERY *, struct driver_table *);
    int (*get_axis_info) (int, int, MOTOR_AXIS_QUERY *, struct driver_table *);
    struct circ_queue *queptr;
    epicsEvent *quelockptr;
    struct circ_queue *freeptr;
    epicsEvent *freelockptr;
    epicsEvent *semptr;
    struct controller ***card_array;
    int *cardcnt_ptr;
    int *any_inmotion_ptr;
    RTN_STATUS (*sendmsg) (int, char const *, char *);
    int (*getmsg) (int, char *, int);
    int (*setstat) (int, int);
    void (*query_done) (int, int, struct mess_node *);
    void (*strtstat) (int);			/* Optional; start status function or NULL. */
    const bool *const init_indicator;		/* Driver initialized indicator. */
    char **axis_names;				/* Axis name array or NULL. */
};


struct thread_args
{
    int motor_scan_rate; /* Poll rate in HZ. */
    struct driver_table *table;
    double update_delay; /* Some controllers (OMS VME58) require a delay
    between a move command and a status update to prevent "stale" data.  A
    zero value disables this delay. */
};


/* Function prototypes. */

epicsShareFunc RTN_STATUS motor_send(struct mess_node *, struct driver_table *);
epicsShareFunc int motor_free(struct mess_node *, struct driver_table *);
epicsShareFunc int motor_card_info(int, MOTOR_CARD_QUERY *, struct driver_table *);
epicsShareFunc int motor_axis_info(int, int, MOTOR_AXIS_QUERY *, struct driver_table *);
epicsShareFunc int motor_task(struct thread_args *);

#endif	/* INCmotordrvComh */
