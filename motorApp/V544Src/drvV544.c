/* File: drvV544.c 		   	*/
/* Version: 1.3a		   	*/
/* Date Last Modified: 2/19/97	*/


/* Device Driver Support routines for motor */
/*
 *      Original Author: Jim Kowalkowski
 *      Current Author: Joe Sullivan
 *      Date: 11/14/94
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
 * .01  01-18-93	jbk     initialized (drvOms.c)
 * .02  11-14-94	jps     copy drvOms.c and modify to point to vme58 driver
 * .03  ??-??-95	jps     copy drvOms58.c and modify to point to Highland driver
 * .03a 02-19-97    tmm     fixed for EPICS 3.13
*/

#include	<vxWorks.h>
#include	<stdioLib.h>
#include	<sysLib.h>
#include	<tickLib.h>
#include	<string.h>
#ifdef __cplusplus
extern "C" {
#include	<callback.h>
#include	<recSup.h>
#include        <devLib.h>
}
#else
#include	<callback.h>
#include	<recSup.h>
#include        <devLib.h>
#endif
#include	<taskLib.h>
#include        <rebootLib.h>

#include	<alarm.h>
/* #include	<dbRecType.h> tmm: EPICS 3.13 */
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<fast_lock.h>
#include	<devSup.h>
#include	<drvSup.h>
#include        <errMdef.h>

#include	"motor.h"
#include	"drvV544.h"

#define PRIVATE_FUNCTIONS 1	/* normal:1, debug:0 */

#define STATIC static

/* Define for return test on locationProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

#define SET_MM_ON(v,a)  v|=(1<<a)
#define SET_MM_OFF(v,a) v&=~(1<<a)


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

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};

/*----------------debugging-----------------*/

#ifdef	DEBUG
    #define Debug(l, f, args...) { if(l<=drvV544debug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

volatile int drvV544debug = 0;
volatile int drvV544sendTest = 0;
STATIC volatile int motor_scan_rate = SCAN_RATE;
STATIC volatile unsigned v544InterruptVector = 0;
STATIC volatile int16_t v544InterruptMask = CS_IEN_OFF;
STATIC volatile uint8_t v544InterruptLevel = V544_INT_LEVEL;
STATIC volatile int max_io_tries = MAX_COUNT;
STATIC volatile uint16_t v544ParkSelect = CS_PARK_50;	/* 50% parking current -
							 * minimum */
STATIC volatile uint16_t v544BusyTimeout = CS_BUSY_TO;	/* Command ACK timeout */
STATIC volatile uint16_t v544CmndAbortTO = CS_ABORT_TO;	/* Command aborted */


/* Delay before acknowledging limit switch state - allows test for moving away from
 * a made switch */
STATIC volatile int overtravelTO = 2;
STATIC volatile int motionTO = 10;

/*----------------motor state info-----------------*/
struct mess_info
{
    struct mess_node *motor_motion;	/* in motion, NULL/node */
    int encoder_present;	/* one YES/NO for each axis */
    int32_t position;	/* one pos for each axis */
    int32_t encoder_position;	/* one pos for each axis */
    int32_t velocity;	/* Raw velocity readback */
    int no_motion_count;
    uint16_t parkCurrent;	/* Bit mapped percentage 0 to 100 */
    unsigned long status;	/* one pos for each axis */
};

struct mot_state
{
    int motor_in_motion;	/* count of motors in motion */
    char ident[50];		/* identification string for this card */
    int total_axis;		/* total axis on this card */
    char *localaddr;		/* address of this card */
    struct mess_info motor_info[MAX_AXIS];
};
STATIC struct mot_state **motor_state;



STATIC int v544_num_cards = 0;
STATIC int v544_num_channels;
STATIC char *v544_addrs;

STATIC int total_cards;
STATIC char V544_trans_axis[] = {'X', 'Y', 'Z', 'T', 'U', 'V', 'R', 'S'};
STATIC int any_motor_in_motion;
STATIC struct mess_queue mess_queue;	/* in message queue head */
STATIC struct mess_queue free_list;

STATIC FAST_LOCK motor_freelist;
STATIC FAST_LOCK motor_queue;
STATIC SEM_ID motor_sem;

/*----------------functions-----------------*/
STATIC int send_cmnd(int card, int motor, int32_t pos, uint16_t vel,
		      uint16_t accel, uint16_t cmnd);
STATIC int process_messages();
STATIC int query_axis(int card);
STATIC struct mess_node *get_head_node();
STATIC struct mess_node *motor_malloc();
STATIC void start_status();
STATIC int set_status(int card, int signal);
STATIC int motor_free(struct mess_node * node);
STATIC int motor_send(MOTOR_CALL * u_msg);
STATIC int motor_card_info(int card, MOTOR_CARD_QUERY * cq);
STATIC int motor_axis_info(int card, int signal, MOTOR_AXIS_QUERY * aq);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC int motorIsrSetup(unsigned card);
STATIC void motorIsr(int card);
STATIC void v544_reset();
/*----------------functions-----------------*/

struct v544_support v544_access = {
    motor_send,
    motor_free,
    motor_card_info,
    motor_axis_info
};

struct
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvV544 =
{
    2,
    report,
    init
};

static long report(int level)
{
    printf("no report yet \n");
    return (0);
}

static long init()
{
    (void) motor_init();
    return ((long) 0);
}



/*****************************************************/
/* send/receive messages from the stepper controller */
/* motor_task()			     */
/*****************************************************/

STATIC motor_task(int a1, int a2, int a3, int a4, int a5, int a6, int a7,
		  int a8, int a9, int a10)
{
    ULONG tick_count, tick_used, tick_curr;
    STATUS sem_ret;
    int wait_time, i;

    tick_used = tickGet();

    while (1)
    {
	tick_curr = tickGet() - tick_used;
	tick_count = (tick_curr > motor_scan_rate)
	    ? NO_WAIT : (motor_scan_rate - tick_curr);
	wait_time = any_motor_in_motion ? (int) tick_count : WAIT_FOREVER;

	sem_ret = semTake(motor_sem, wait_time);

	if (sem_ret == ERROR)
	    tick_used = tickGet();


	if (any_motor_in_motion)
	{
	    for (i = 0; i < v544_num_cards; i++)
	    {
		if (motor_state[i] &&
		    motor_state[i]->motor_in_motion)
		    query_axis(i);
	    }
	}

	if (sem_ret != ERROR)
	    process_messages();
    }
}



/*****************************************************/
/* send query to get all axis in motions position    */
/* query_axis()                                      */
/*****************************************************/
STATIC query_axis(int card)
{
    volatile struct vmex_motor *pmotor;
    MOTOR_RETURN *mess_ret;
    register struct mess_node *motor_motion;
    register struct mess_info *motor_info;
    int i;


    pmotor = (struct vmex_motor *) motor_state[card]->localaddr;

    /* get the encoder positions for axis in motion */
    for (i = 0; i < motor_state[card]->total_axis; i++)
    {
	motor_info = &(motor_state[card]->motor_info[i]);
	motor_motion = motor_info->motor_motion;

	/*
	 * Check for commanded motion and actual motion or done before
	 * issuing a callback.
	 */
	if (motor_motion && set_status(card, i))
	{
	    if (drvV544debug >= 4)
		printf("query_axis: motor %d, pos=%d, enc=%d, vel=%d, status=0x%04x\n",
		       (card * V544_NUM_CHANNELS) + i,
		       motor_info->position,
		       motor_info->encoder_position,
		       motor_info->velocity,
		       motor_info->status);

	    motor_motion->position = motor_info->position;
	    motor_motion->encoder_position = motor_info->encoder_position;
	    motor_motion->velocity = motor_info->velocity;
	    motor_motion->status = motor_info->status;

	    mess_ret = (MOTOR_RETURN *) motor_malloc();
	    mess_ret->callback = motor_motion->callback;
	    mess_ret->precord = motor_motion->precord;
	    mess_ret->position = motor_motion->position;
	    mess_ret->encoder_position = motor_motion->encoder_position;
	    mess_ret->velocity = motor_motion->velocity;
	    mess_ret->status = motor_motion->status;
	    mess_ret->type = motor_motion->type;

	    if (motor_motion->status & RA_OVERTRAVEL ||
		motor_motion->status & RA_PROBLEM)
	    {
		/* Assure motion is stopped */
		send_cmnd(card, i, 0, 0, 0, CS_IDLE);
		Debug(1, "query_axis: motion aborted - status=0x%4x\n",
		      motor_info->status);

		motor_state[card]->motor_in_motion--;

		motor_free(motor_motion);
		motor_motion = (struct mess_node *) NULL;
		motor_info->motor_motion = (struct mess_node *) NULL;
	    }
	    else if (motor_motion->status & RA_DONE)
	    {
		/* Check for additional motion in command list */
		if (motor_motion->cmndIndx < motor_motion->cmndCnt)
		{
		    CMND_DEF *pcmnd = &motor_motion->cmndList[motor_motion->cmndIndx++];

		    send_cmnd(card, i, pcmnd->pos, pcmnd->vel,
			      pcmnd->accel, pcmnd->cmnd);

		    /* Not done yet. */
		    motor_motion->status &= ~RA_DONE;
		    mess_ret->status = motor_motion->status;

		}
		else
		{
		    motor_state[card]->motor_in_motion--;

		    motor_free(motor_motion);
		    motor_motion = (struct mess_node *) NULL;
		    motor_info->motor_motion = (struct mess_node *) NULL;
		}
	    }

	    callbackRequest((CALLBACK *) mess_ret);

	    if (motor_state[card]->motor_in_motion == 0)
	    {
		SET_MM_OFF(any_motor_in_motion, card);
	    }
	}
    }

    return (0);
}


/**************************************************************
 * Read motor status from card
 * set_status()
 ************************************************************/

STATIC set_status(int card, int signal)
{
    register struct mess_info *motor_info;
    struct vmex_motor *pmotor;
    MOTOR_REG *pmotorReg;
    POS_REG *pmotorPos;
    POS_REG motorPosReg;
    int32_t motorPos;
    uint16_t motorVel;
    uint16_t motorStatusReg;
    uint16_t motorCmndReg;
    int rtn_state = 0;


    motor_info = &(motor_state[card]->motor_info[signal]);
    pmotor = (struct vmex_motor *) motor_state[card]->localaddr;

    pmotorReg = &pmotor->motorReg[signal];
    motorStatusReg = pmotorReg->status;
    motorCmndReg = pmotorReg->cmnd;

    Debug(5, "set_status: cmndReg=0x%04x\n", motorCmndReg);


    /* Test for command-in-progress */
    if (motorCmndReg & (CS_BUSY | CS_DONE))
    {
	pmotorPos = &pmotor->posReg[signal];

	/* Get motor position, test if read during update */
	do
	{
	    motorPosReg.coarse = pmotorPos->coarse;
	    motorPosReg.fine = pmotorPos->fine;
	} while (motorPosReg.coarse != pmotorPos->coarse);

	motorVel = pmotor->velReg[signal];
	motorPos = (motorPosReg.coarse << 16) | motorPosReg.fine;

	/* Set direction base on position history */
	if (motorPos > motor_info->position)
	{
	    /* Positive motion - check clockwise limit */
	    motor_info->status |= RA_DIRECTION;

	    if ((motorStatusReg & STAT_CWL) && (motorCmndReg & CS_DONE))
		motor_info->status |= RA_OVERTRAVEL;
	    else
		motor_info->status &= ~RA_OVERTRAVEL;

	    motor_info->no_motion_count = 0;
	}
	else if (motorPos < motor_info->position)
	{
	    /* Negative motion - check counterclockwise limit */
	    motor_info->status &= ~RA_DIRECTION;
	    motorVel *= -1;

	    if ((motorStatusReg & STAT_CCWL) && (motorCmndReg & CS_DONE))
		motor_info->status |= RA_OVERTRAVEL;
	    else
		motor_info->status &= ~RA_OVERTRAVEL;

	    motor_info->no_motion_count = 0;
	}
	else
	{
	    /*
	     * No motion - check both limits and set direction accordingly.
	     */
	    if (motorStatusReg & (STAT_CCWL | STAT_CWL))
	    {
		motor_info->status |= RA_OVERTRAVEL;

		if (motorStatusReg & STAT_CWL)
		    motor_info->status |= RA_DIRECTION;	        /* Positive Limit */
		else
		    motor_info->status &= ~RA_DIRECTION;	/* Negative Limit */
	    }

	    motor_info->no_motion_count++;

	}

	if (motor_info->no_motion_count > motionTO)
	{
	    motor_info->status |= RA_PROBLEM;
	    motor_info->no_motion_count = 0;
	}
	else
	    motor_info->status &= ~RA_PROBLEM;

	motor_info->encoder_position = 0;
	motor_info->position = motorPos;
	motor_info->velocity = motorVel;

	if (motorCmndReg & CS_DONE)
	    motor_info->status |= RA_DONE;
	else
	    motor_info->status &= ~RA_DONE;

	if (motorStatusReg & STAT_INDX)
	    motor_info->status |= RA_HOME;
	else
	    motor_info->status &= ~RA_HOME;

	rtn_state = (!motor_info->no_motion_count ||
		     (motor_info->status & (RA_OVERTRAVEL | RA_DONE | RA_PROBLEM))) ? 1 : 0;
    }
    else if (motor_info->no_motion_count++ > motionTO)
    {
	/* Command was never acknowledged */
	motor_info->no_motion_count = 0;

	Debug(1, "set_status: Ack Timeout - cmndReg=0x%04x\n", motorCmndReg);

	motor_info->status |= RA_PROBLEM;
	rtn_state = 1;
    }

    return (rtn_state);
}


/*****************************************************/
/* Process messages */
/* process_messages()     */
/*****************************************************/
STATIC process_messages()
{
    register struct mess_node *motor_motion;
    struct mess_node *node;
    CMND_DEF *pcmnd;
    int16_t cmndbuf;
    char qa_buf[15];

    while (node = get_head_node())
    {
	Debug(6, "Got node type (%d)\n", node->type);

	/* Check that card and signal exists - check tightened 6/11/96 (jps) */
	if ((node->card >= 0 && node->card < total_cards) &&
	    motor_state[node->card] &&
	    (node->signal >= 0 && node->signal < motor_state[node->card]->total_axis))
	{
	    switch (node->type)
	    {
	    case QUERY:
		/* Not needed */
		callbackRequest((CALLBACK *) node);
		break;
	    case VELOCITY:
		/* send a message to V544 */
		motor_motion =
		    motor_state[node->card]->motor_info[node->signal].motor_motion;

		pcmnd = &node->cmndList[node->cmndIndx++];
		send_cmnd(node->card, node->signal, pcmnd->pos, pcmnd->vel, pcmnd->accel,
			  pcmnd->cmnd);

		/*
		 * this is tricky - another motion is here there is a very
		 * large assumption being made here: that the person who sent
		 * the previous motion is the same one that is sending this
		 * one, if he weren't, the guy that sent the original would
		 * never get notified of finish motion.  This makes sense in
		 * record processing since only one record can be assigned to
		 * an axis and sent commands to it. An improvement would be
		 * to check and see if the record pointers were the same, if
		 * they were not, then send a finish message to the previous
		 * registered motion guy.
		 */

		if (!motor_motion)	/* if NULL */
		    motor_state[node->card]->motor_in_motion++;
		else
		    motor_free(motor_motion);

		SET_MM_ON(any_motor_in_motion, node->card);
		motor_state[node->card]->motor_info[node->signal].motor_motion = node;
		break;
	    case MOTION:
		/* send a message to V544 */
		motor_motion =
		    motor_state[node->card]->motor_info[node->signal].motor_motion;

		pcmnd = &node->cmndList[node->cmndIndx++];
		send_cmnd(node->card, node->signal, pcmnd->pos, pcmnd->vel, pcmnd->accel,
			  pcmnd->cmnd);

		/* this is tricky - see velocity comment */
		if (!motor_motion)	/* if NULL */
		    motor_state[node->card]->motor_in_motion++;
		else
		    motor_free(motor_motion);

		SET_MM_ON(any_motor_in_motion, node->card);
		motor_state[node->card]->motor_info[node->signal].no_motion_count = 0;
		motor_state[node->card]->motor_info[node->signal].motor_motion = node;
		break;
	    case INFO:
		set_status(node->card, node->signal);
		node->position =
		    motor_state[node->card]->motor_info[node->signal].position;
		node->encoder_position =
		    motor_state[node->card]->motor_info[node->signal].encoder_position;
		node->status =
		    motor_state[node->card]->motor_info[node->signal].status;

		/*
		 * ===========================================================
		 * node->status & RA_DONE is not a reliable indicator of
		 * anything, in this case, since set_status() didn't ask the
		 * V544 controller to set the "Done" flag on command
		 * completion. Nevertheless, recMotor:process() needs to know
		 * whether the motor has stopped, and this we can tell by
		 * looking for a struct motor_motion.
		 * ===========================================================
		 * */
		motor_motion =
		    motor_state[node->card]->motor_info[node->signal].motor_motion;
		if (motor_motion)
		    node->status &= ~RA_DONE;
		else
		    node->status |= RA_DONE;

		callbackRequest((CALLBACK *) node);
		break;

	    case MOVE_TERM:
		pcmnd = &node->cmndList[node->cmndIndx++];
		send_cmnd(node->card, node->signal, pcmnd->pos, pcmnd->vel, pcmnd->accel,
			  pcmnd->cmnd);
		motor_motion =
		    motor_state[node->card]->motor_info[node->signal].motor_motion;
		/* if motion-in-progress - cancel buffered commands  */
		if (motor_motion)
		    motor_motion->cmndIndx = motor_motion->cmndCnt;
		break;
	    default:
		/* send a message to V544 */
		pcmnd = &node->cmndList[node->cmndIndx++];
		send_cmnd(node->card, node->signal, pcmnd->pos, pcmnd->vel,
			  pcmnd->accel, pcmnd->cmnd);
		taskDelay(1);	/* Give command time to finish */
		set_status(node->card, node->signal);
		if (!motor_state[node->card]->motor_info[node->signal].status & CS_DONE)
		    Debug(1, "process(v455): motor %d Immediate command NOT-done yet.\n",
			  (node->card * V544_NUM_CHANNELS) + node->signal);
		motor_free(node);	/* free message buffer */
		break;
	    }
	}
	else
	{
	    Debug(1, "send_mess - invalid card #%d\n", node->card);
	    node->position = 0;
	    node->encoder_position = 0;
	    node->velocity = 0;
	    node->status = RA_PROBLEM;
	    callbackRequest((CALLBACK *) node);
	}
    }
    return (0);
}




/*****************************************************/
/* Get a message off the queue */
/* get_head_node()			     */
/*****************************************************/
STATIC struct mess_node *get_head_node()
{
    struct mess_node *node, *save_node;

    Debug(7, "get_head_node: entry\n", 0);
    /* get a message from queue */
    FASTLOCK(&motor_queue);

    node = mess_queue.head;
    save_node = (struct mess_node *) NULL;
    Debug(7, "get_head_node: node = %x\n", node);

    /* delete node from list */
    if (node)
    {				/* tmm added */
	if (node == mess_queue.head)
	{
	    Debug(7, "get_head_node: setting mess_queue.head=%x\n", node->next);
	    mess_queue.head = node->next;
	}
	if (node == mess_queue.tail)
	{
	    Debug(7, "get_head_node: setting mess_queue.tail=%x\n", save_node);
	    mess_queue.tail = save_node;
	}
	if (save_node)
	{
	    Debug(7, "get_head_node: setting save_node->next=%x\n", node->next);
	    save_node->next = node->next;
	}
    }
    FASTUNLOCK(&motor_queue);

    Debug(7, "get_head_node: returning %x\n", node);
    return (node);
}




/*****************************************************/
/* send command to the V544 board control register   */
/* send_cmnd()			                     */
/*      units = micro steps (256 ustep per step)     */
/*****************************************************/
STATIC int send_cmnd(int card,	/* board number */
		      int motor,/* motor number */
		      int32_t pos,	/* request position */
		      uint16_t vel,	/* request velocity */
		      uint16_t accel,	/* request acceleration */
		      uint16_t cmnd)	/* motor command */
{
    volatile struct vmex_motor *pmotor;	/* board base address */
    volatile MOTOR_REG *pmotorReg;
    int return_code, i, trys;
    int abortTO;
    uint16_t cmndSave;

    pmotor = (struct vmex_motor *) motor_state[card]->localaddr;
    pmotorReg = &pmotor->motorReg[motor];
    return_code = 0;

    cmndSave = pmotorReg->cmnd;

    if (vel != 0 && !(cmndSave & CS_DONE))
    {
	Debug(1, "send_cmnd: motor %d busy\n", (card * V544_NUM_CHANNELS) + motor);
	return_code = -1;
    }

    if (cmndSave & CS_ERROR)
    {
	Debug(1, "send_cmnd: command error - CMND=0x%04x\n", pmotorReg->cmnd);
	return_code = -1;
    }

    if ((pmotorReg->status) & STAT_MOTR)
    {
	Debug(1, "send_cmnd: motor %d sense error\n", (card * 4) + motor);
	return_code = -1;
    }

    pmotorReg->pos.coarse = pos >> 16;	/* High word */
    pmotorReg->pos.fine = pos & 0xffff;	/* Low word */
    pmotorReg->velocity = vel;
    pmotorReg->accel = pmotorReg->lsDecel = accel;

    if (cmndSave & CS_DONE)
    {
	/*
	 * The v544 sometimes indicates done after 180us without starting the
	 * command.  That time is well short of the normal latency so it is
	 * save to test for it.
	 */
	trys = 0;
	do
	{
	    pmotorReg->cmnd = (cmnd | CS_GO | v544InterruptMask | v544ParkSelect);
	    abortTO = v544CmndAbortTO;
	    while (abortTO--);	/* dwell about 200us */
	    if ((cmndSave = (pmotorReg->cmnd & CS_DONE)))
		Debug(2, "send_cmnd: motor %d command aborted - retrying.\n",
		      (card * 4) + motor);
	} while (cmndSave && trys++ < 3);

	if (drvV544sendTest)
	{
	    POS_REG *posReg = &pmotor->posReg[motor];
	    int posMoving = 0;

	    pos = posReg->fine;

	    printf("sendTest: Waiting for BUSY/DONE.");
	    while (!((cmndSave = pmotorReg->cmnd) & (CS_BUSY | CS_DONE)))
	    {
		if (pos != posReg->fine && !posMoving)
		{
		    posMoving = 1;
		    printf("\nMotor Moving");
		}
		printf(".");
	    }
	    posMoving = 0;
	    if (cmndSave & CS_BUSY)
	    {
		printf("\nsendTest: Waiting for DONE.");
		while (!(pmotorReg->cmnd & CS_DONE))
		{
		    if (pos == posReg->fine && !posMoving)
		    {
			posMoving = 1;
			printf("\nMotor Stopped");
		    }
		    pos = posReg->fine;
		    printf(".");
		}
	    }
	    printf("\nsendTest: **DONE**\n");
	}

	if (drvV544debug >= 4)
	    printf("send_cmnd: motor %d, pos=%ld, vel=%d, accel=%d, cmnd=0x%04x\n",
		   (card * V544_NUM_CHANNELS) + motor, pos, vel, accel,
		   (cmnd | CS_GO | v544InterruptMask | v544ParkSelect));


    }

    return (return_code);
}



/*****************************************************/
/* Configuration function for  module_types data     */
/* areas. v544Setup()                                */
/*****************************************************/
v544Setup(int num_cards,	/* maximum number of cards in rack */
	  int num_channels,	/* Channels per card (4 - default) */
	  void *addrs,		/* Base Address(0xdd00 - default) */
	  unsigned enableInt,	/* 0=interrupts disabled, 1=VXI standard
				 * vector */
	  int int_level,	/* interrupt level (5 - default) */
	  int scan_rate)	/* polling rate - 1/60 sec units (6 -
				 * default) */
{
    int vector;

    if (num_cards < 1 || num_cards > V544_NUM_CARDS)
	v544_num_cards = V544_NUM_CARDS;
    else
	v544_num_cards = num_cards;

    if (num_channels < 1 || num_channels > V544_NUM_CHANNELS)
	v544_num_channels = V544_NUM_CHANNELS;
    else
	v544_num_channels = num_channels;

    v544_addrs = addrs;

    if ((vector = enableInt) != 0)
    {
	vector = VXI_VECTOR((int) v544_addrs);
	if (vector < 64 || vector > 255)
	{
	    Debug(0, "v544Setup: invalid interrupt vector %d\n", vector);
	    Debug(0, "           generated from VXI address 0x%x\n", addrs);
	    vector = 0;
	}
    }

    v544InterruptVector = vector;


    if (int_level < 1 || int_level > 6)
    {
	Debug(0, "v544Setup: invalid interrupt level %d\n", int_level);
	v544InterruptLevel = V544_INT_LEVEL;
    }
    else
	v544InterruptLevel = int_level;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;
}



/*****************************************************/
/* Interrupt service routine.                        */
/* motorIsr()		                     */
/*****************************************************/
STATIC void motorIsr(int card)
{
    volatile struct mot_state *pmotorState;
    struct vmex_motor *pmotor;


    if (card >= total_cards || (pmotorState = motor_state[card]) == NULL)
    {
	logMsg("Invalid entry-card #%d\n", card, 0, 0, 0, 0, 0);
	return;
    }

    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    if (drvV544debug >= 4)
	logMsg("V544 card %d,vscr=0x%X\n", card, pmotor->vcsr, 0, 0, 0, 0);

    /* Let polling task check for motor done */
    semGive(motor_sem);
}

STATIC int motorIsrSetup(unsigned card)
{
    struct vmex_motor *pmotor;
    long status;

    Debug(6, "motorIsrSetup: Entry card#%d\n", card);

    pmotor = (struct vmex_motor *) (motor_state[card]->localaddr);

    status = devConnectInterrupt(V544_INTERRUPT_TYPE,
				 v544InterruptVector + card,
				 motorIsr, (void *) card);
    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__, "Can't connect to vector %d\n",
		  v544InterruptVector + card);
	v544InterruptVector = 0;/* Disable interrupts */
	v544InterruptMask = CS_IEN_OFF;
	return (ERROR);
    }

    status = devEnableInterruptLevel(V544_INTERRUPT_TYPE,
				     v544InterruptLevel);
    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__,
		  "Can't enable enterrupt level %d\n",
		  v544InterruptLevel);
	v544InterruptVector = 0;/* Disable interrupts */
	v544InterruptMask = CS_IEN_OFF;
	return (ERROR);
    }

    /* Setup mask for interrupt-on-done */
/*     v544InterruptMask = CS_IEN_ON; */
    return (OK);
}

/*****************************************************/
/* initialize all software and hardware		     */
/* motor_init()			     */
/*****************************************************/
STATIC motor_init()
{
    volatile struct vmex_motor *pmotor;
    long status;
    int i, j, k;
    char axis_pos[50];
    char encoder_pos[50];
    int total_encoders = 0;
    int total_axis = 0;
    void *localaddr;
    void *probeAddr;

    /* Check for setup */
    if (v544_num_cards <= 0)
    {
	Debug(1, "motor_init: *V544 driver disabled* \n v544Setup() is missing from startup script.\n", 0);
	return (ERROR);
    }

    /* allocate space for total number of motors */
    (void *) motor_state = malloc(v544_num_cards * sizeof(struct mot_state *));

    /* allocate structure space for each motor present */
    total_cards = 0;

    if (rebootHookAdd((FUNCPTR) v544_reset) == ERROR)
	Debug(1, "vme58 motor_init: v544_reset disabled\n", 0);

    for (i = 0; i < v544_num_cards; i++)
    {
	Debug(2, "motor_init: v544 card %d\n", i);

	probeAddr = v544_addrs + (i * V544_BRD_SIZE);
	Debug(9, "motor_init: locationProbe() on addr 0x%x\n", probeAddr);
	status = locationProbe(V544_ADDRS_TYPE, probeAddr);

	if (PROBE_SUCCESS(status))
	{
	    status = devRegisterAddress(__FILE__,
					V544_ADDRS_TYPE,
					probeAddr,
					V544_BRD_SIZE,
					&localaddr);
	    Debug(9, "motor_init: v544 devRegisterAddress() status = %d\n", status);
	    if (!RTN_SUCCESS(status))
	    {
		errPrintf(status, __FILE__, __LINE__,
			  "Can't register V544 address 0x%x\n", probeAddr);
		return (ERROR);
	    }

	    Debug(9, "motor_init: V544 localaddr = %x\n", localaddr);
	    pmotor = (struct vmex_motor *) localaddr;

	    total_cards++;

	    Debug(9, "motor_init: V544 malloc'ing motor_state\n", 0);
	    motor_state[i] = (struct mot_state *) malloc(
						  sizeof(struct mot_state));
	    motor_state[i]->localaddr = localaddr;
	    motor_state[i]->motor_in_motion = 0;

	    /* Check controller ID */
	    sprintf(motor_state[i]->ident, "%s rev. %d-%c, sn. %d", V544_LABEL,
		    pmotor->pgid, (char) pmotor->prev, pmotor->vsn);
	    Debug(3, "motor_init: V544 card ID = %s \n", motor_state[i]->ident);

	    /* Issue warnings if self-test failed or VXI ID is not correct */
	    if (!(pmotor->vcsr & (VCSR_SFT | VCSR_RDY)))
		Debug(0, "motor_init: V544 card %d SELFTEST FAILED\n", i);
	    if (pmotor->vxid != V544_VXID)
		Debug(0, "motor_init: V544 card %d - WRONG ID\n", i);

	    total_axis = v544_num_channels;

	    /* Test for optional encoders */
	    if (pmotor->opts & OPTS_ENC)
		total_encoders = total_axis;
	    else
		total_encoders = 0;

	    /* Initilize per-axis controller and internal data fields */
	    for (j = 0; j < total_axis; j++)
	    {
		send_cmnd(i, j, 0, 0, 0, CS_IDLE);
		motor_state[i]->motor_info[j].motor_motion = NULL;
		motor_state[i]->motor_info[j].status = 0;
		if (total_encoders)
		    motor_state[i]->motor_info[j].encoder_present = YES;
		else
		    motor_state[i]->motor_info[j].encoder_present = NO;
	    }

	    Debug(3, "motor_init: Total axis = %d\n", total_axis);

	    motor_state[i]->total_axis = total_axis;

	    /* Enable interrupt-when-done if selected */
	    if (v544InterruptVector)
	    {
		if (motorIsrSetup(i) == ERROR)
		    errPrintf(0, __FILE__, __LINE__, "Interrupts Disabled!\n");
	    }

	    for (j = 0; j < total_axis; j++)
	    {
		motor_state[i]->motor_info[j].status = 0;
		motor_state[i]->motor_info[j].no_motion_count = 0;
		motor_state[i]->motor_info[j].encoder_position = 0;
		motor_state[i]->motor_info[j].position = 0;

		if (motor_state[i]->motor_info[j].encoder_present == YES)
		    motor_state[i]->motor_info[j].status |= EA_PRESENT;

		set_status(i, j);
	    }

	    Debug(2, "motor_init: Init Address=0x%08.8x\n", localaddr);
	    Debug(3, "motor_init: Total encoders = %d\n\n", total_encoders);
	}
	else
	{
	    Debug(3, "motor_init: Card NOT found!\n", 0);
	    motor_state[i] = (struct mot_state *) NULL;
	}
    }


    motor_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
    any_motor_in_motion = 0;


    FASTLOCKINIT(&motor_queue);
    FASTLOCK(&motor_queue);
    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&motor_queue);

    FASTLOCKINIT(&motor_freelist);
    FASTLOCK(&motor_freelist);
    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&motor_freelist);

    Debug(3, "Motors initialized\n", 0);

    taskSpawn("tmotor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    Debug(3, "Started motor_task\n", 0);

    return (0);
}


/*****************************************************/
/* send a command to the stepper motor task queue    */
/* motor_send()			     */
/*****************************************************/
STATIC int motor_send(MOTOR_CALL * u_msg)
{
    struct mess_node *new_message;
    int index;

    Debug(9, "motor_send invoked for v544\n", 0);

    new_message = motor_malloc();
    new_message->callback = u_msg->callback;
    new_message->next = (struct mess_node *) NULL;
    new_message->type = u_msg->type;
    new_message->signal = u_msg->signal;
    new_message->card = u_msg->card;
    new_message->precord = u_msg->precord;
    new_message->status = 0;
    new_message->cmndCnt = u_msg->cmndCnt;
    new_message->cmndIndx = 0;

    bcopy((void *) u_msg->cmndList, (void *) new_message->cmndList, sizeof(CMND_DEF) * u_msg->cmndCnt);

    switch (new_message->type)
    {
    case MOTION:
	break;
    case VELOCITY:
	break;
    case MOVE_TERM:
	break;
    case IMMEDIATE:
	break;
    case QUERY:
    case INFO:
	break;
    default:
	return (-1);
	break;
    }

    FASTLOCK(&motor_queue);

    if (mess_queue.tail)
    {
	mess_queue.tail->next = new_message;
	mess_queue.tail = new_message;
    }
    else
    {
	mess_queue.tail = new_message;
	mess_queue.head = new_message;
    }

    FASTUNLOCK(&motor_queue);

    semGive(motor_sem);

    return (0);
}

STATIC struct mess_node *motor_malloc()
{
    struct mess_node *node;

    FASTLOCK(&motor_freelist);

    if (!free_list.head)
	node = (struct mess_node *) malloc(sizeof(struct mess_node));
    else
    {
	node = free_list.head;
	free_list.head = node->next;
	if (!free_list.head)
	    free_list.tail = (struct mess_node *) NULL;
    }

    FASTUNLOCK(&motor_freelist);
    return (node);
}

STATIC int motor_free(struct mess_node * node)
{
    FASTLOCK(&motor_freelist);

    node->next = (struct mess_node *) NULL;

    if (free_list.tail)
    {
	free_list.tail->next = node;
	free_list.tail = node;
    }
    else
    {
	free_list.head = node;
	free_list.tail = node;
    }

    FASTUNLOCK(&motor_freelist);
    return (0);
}

/*---------------------------------------------------------------------*/
/*
 * both of these routines are only to be used at initialization time - before
 * queuing transactions starts for an axis.  These routines do no locking,
 * since transactions will not enter the queue for an axis until after they
 * have run.
 */

/* return the card information to caller */
STATIC int motor_card_info(int card, MOTOR_CARD_QUERY * cq)
{
    if (card >= 0 && 
	card < total_cards && 
	motor_state[card])
    {
	cq->total_axis = motor_state[card]->total_axis;
	cq->card_name = motor_state[card]->ident;
    }
    else
	cq->total_axis = 0;

    return (0);
}

/* return information for an axis to the caller */
STATIC int motor_axis_info(int card, int signal, MOTOR_AXIS_QUERY * aq)
{
    if (card >= 0 && 
	card < total_cards && 
	motor_state[card] &&
	signal >= 0 &&
	signal < motor_state[card]->total_axis)
    {
	aq->position = motor_state[card]->motor_info[signal].position;
	aq->encoder_position =
	    motor_state[card]->motor_info[signal].encoder_position;
	aq->status = motor_state[card]->motor_info[signal].status;
    }
    else
    {
	aq->position = aq->encoder_position = 0;
	aq->status = RA_PROBLEM;
    }

    return (0);
}

/*
 *
 *  Disables interrupts. Called on CTL X reboot.
 *
 */

STATIC void v544_reset()
{
    short card;
    short axis;
    struct vmex_motor *pmotor;
    short status;

    for (card = 0; card < total_cards; card++)
    {
	(void *) pmotor = motor_state[card]->localaddr;
	if (vxMemProbe((char *) pmotor, READ, sizeof(short), (char *) &status) == OK)
	{
	    v544InterruptMask = CS_IEN_OFF;
	    for (axis = 0; axis < v544_num_channels; axis++)
		send_cmnd(card, axis, 0, 0, 0, CS_IDLE);
	}
    }
}


/*---------------------------------------------------------------------*/
