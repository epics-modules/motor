/*
FILENAME...	motordrvCom.h

USAGE...	This file contains definitions and structures that
		are common to all motor record driver support modules.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-06-14 15:09:40 $
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
 */


#ifndef	INCmotordrvComh
#define	INCmotordrvComh 1

#include <rngLib.h>


/* Controller communication port type, followed by status. */
enum PortType
{
    GPIB_PORT = 0,
    RS232_PORT		/* = 1 */
};

enum CommStatus
{
    NORMAL,
    COMM_ERR
};

#ifndef __cplusplus
typedef enum PortType PortType;
typedef enum CommStatus CommStatus;
#endif

/*
Valid command types for the driver. The order is of importance; 0 is
lowest importance.  Device support will choose the greatest one to
use as the driver transaction type.
*/

#define UNDEFINED (unsigned char)0	/* garbage type */
#define IMMEDIATE (unsigned char)1	/* 'i' an execute immediate, no reply */
#define MOVE_TERM (unsigned char)2	/* 't' terminate a previous active motion */
#define MOTION    (unsigned char)3	/* 'm' will produce motion updates */
#define VELOCITY  (unsigned char)4	/* 'v' make motion updates till MOVE_TERM */
#define INFO      (unsigned char)5	/* 'f' get curr motor/encoder pos and stat */

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
    unsigned char type;
    char message[MAX_MSG_SIZE];
    long position;
    long encoder_position;
    long velocity;
    unsigned long status;
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
    char *axis_names;
    int total_axis;
} MOTOR_CARD_QUERY;

typedef struct mess_axis_query
{
    long position;
    long encoder_position;
    unsigned long status;
} MOTOR_AXIS_QUERY;

struct axis_status
{
    char direction;
    char done;
    char overtravel;
    char home;
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

struct circ_queue	/* Circular queue structure. */
{
    struct mess_node *head;
    struct mess_node *tail;
};

/*----------------motor state info-----------------*/

struct irqdatastr	/* Used only for VME44. */
{
    /* Interrupt Handling control elements */
    int irqErrno;	/* Error indicator from isr */
    uint8_t irqEnable;
    RING_ID recv_rng;	/* message receiving control */
    SEM_ID recv_sem;
    RING_ID send_rng;	/* message transmitting control */
    SEM_ID send_sem;
};

struct mess_info
{
    struct mess_node *motor_motion;	/* in motion, NULL/node */
    int encoder_present;		/* one YES/NO for each axis */
    int32_t position;		/* one pos for each axis */
    int32_t encoder_position;	/* one pos for each axis */
    int32_t velocity;		/* Raw velocity readback(not implemented) */
    int no_motion_count;
    ULONG status_delay;		/* Insure 10ms delay between motion/velocity
				 * commands and status query. */
    unsigned long status;	/* one pos for each axis */
    int pid_present;		/* PID control indicator for VME58 (YES/NO). */
    double high_limit;		/* MM4000 only; Controller's high travel limit. */
    double low_limit;		/* MM4000 only; Controller's low travel limit. */
};

struct controller	/* Controller board information. */
{
    int motor_in_motion;/* count of motors in motion */
    char ident[50];	/* identification string for this card */
    int total_axis;	/* total axis on this card */
    char *localaddr;	/* address of this card */
    BOOLEAN cmnd_response; /* Indicates controller communication response
	    * to VELOCITY, MOTION and MOVE_TERM type commands. */
    struct irqdatastr *irqdata;	/* VME44 only; IRQ data. */
    void *DevicePrivate; /* Pointer to device specific structure.  For
		MM3000/4000 = (struct MMcontroller *); otherwise Null. */
    struct mess_info motor_info[MAX_AXIS];
};

/* The "driver_table" structure allows device level access to driver level
   data and functions; it supports "common code" for all motor record device
   drivers.
*/
struct driver_table
{
    int (*init) (void);
    int (*send) (struct mess_node *, struct driver_table *, char *);
    int (*free) (struct mess_node *, struct driver_table *);
    int (*get_card_info) (int, MOTOR_CARD_QUERY *, struct driver_table *);
    int (*get_axis_info) (int, int, MOTOR_AXIS_QUERY *, struct driver_table *);
    struct circ_queue *queptr;
    FAST_LOCK *quelockptr;
    struct circ_queue *freeptr;
    FAST_LOCK *freelockptr;
    SEM_ID *semptr;
    struct controller ***card_array;
    int *cardcnt_ptr;
    int *any_inmotion_ptr;
    int (*sendmsg) (int, char const *, char);
    int (*getmsg) (int, char *, int);
    int (*setstat) (int, int);
    void (*query_done) (int, int, struct mess_node *);
    void (*strtstat) (int);			/* Optional; start status function or NULL. */
    const BOOLEAN *const init_indicator;	/* Driver initialized indicator. */
};

/* Function prototypes. */

extern int motor_send(struct mess_node *, struct driver_table *, char *);
extern int motor_free(struct mess_node *, struct driver_table *);
extern int motor_card_info(int, MOTOR_CARD_QUERY *, struct driver_table *);
extern int motor_axis_info(int, int, MOTOR_AXIS_QUERY *, struct driver_table *);
extern int motor_task(int a1, int, int, int, int, int, int, int, int, int a10);

#endif	/* INCmotordrvComh */
