/*
FILENAME...	motordrvCom.c
USAGE... 	This file contains driver functions that are common
		to all motor record driver modules.

Version:	$Revision: 1.2 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-03-03 22:18:30 $
*/

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
 * .01  01-18-93	jbk     initialized
 * .02  11-14-94	jps     copy drvOms.c and modify to point to vme58 driver
 *      ...
 * .14  02-10-95	jps	first released version w/interrupt supprt
 * .15  02-16-95	jps	add better simulation mode support
 * .16  03-10-95	jps	limit switch handling improvements
 * .17  04-11-95	jps  	misc. fixes
 * .18  09-27-95	jps	fix GET_INFO latency bug, add axis argument to
 *				set_status() create add debug verbosity of 3
 * .19  05-03-96	jps     convert 32bit card accesses to 16bit - vme58PCB
 *				version D does not support 32bit accesses.
 * .20  06-20-96	jps     add more security to message parameters (card #)
 * .20a 02-19-97    	tmm     fixed for EPICS 3.13
 */


#include	<vxWorks.h>
#include	<taskLib.h>
#include	<stdlib.h>
#include	<string.h>
#ifdef __cplusplus
extern "C" {
#include	<callback.h>
}
#else
#include	<callback.h>
#endif
#include	<fast_lock.h>
#include	<tickLib.h>

#include	"motor.h"
#include	"motordrvCom.h"

/* Function declarations. */
static int query_axis(int card, struct driver_table *tabptr, ULONG tick);
static void process_messages(struct driver_table *tabptr, ULONG tick);
static struct mess_node *get_head_node(struct driver_table *tabptr);
static struct mess_node *motor_malloc(struct circ_queue *freelistptr, FAST_LOCK *lockptr);

char oms_trans_axis[] = {'X', 'Y', 'Z', 'T', 'U', 'V', 'R', 'S'};


/*
 * FUNCION...   motor_task()
 * LOGIC:
 *  WHILE FOREVER
 *	IF no motors in motion for this board type.
 *	    Set "wait_time" to WAIT_FOREVER.
 *	ELSE
 *	    Set "tick_curr" to time elapsed since last timeout return from semaphore pend.
 *	    Limit "wait_time" to between 2 and "motor_scan_rate" RTOS ticks.
 *	    IF elapsed time (i.e., "tick_curr") < user delay.
 *		Set "wait_time" to (user delay - elapsed time).
 *		IF "wait_time" < 2 OS tick.
 *		    Set "wait_time" to 2 OS tick.
 *		ENDIF
 *	    ELSE
 *		Set "wait_time" to to 2 OS tick.
 *	    ENDIF
 *	ENDIF
 *	Pend on semaphore with "wait_time" timeout argument - call semTake().
 *	Update elapsed time.
 *	IF the "any_motor_in_motion" indicator is ON.
 *	    IF VME58 instance of this task.
 *		Start data area update on all cards - Call start_status().
 *	    ENDIF
 *	    FOR each OMS board.
 *		IF motor data structure defined, AND, motor-in-motion indicator ON.
 *		    Update OMS board status - call query_axis().
 *		ENDIF
 *	    ENDFOR
 *	ENDIF
 *	IF someone posted the semaphore.
 *	    Process commands - call process_messages().
 *	ENDIF
 *  ENDWHILE
 *
 * NOTES... This function MUST BE reentrant.
 */
/*****************************************************/

int motor_task(int scan_rate, int msw_tab_ptr, int lsw_tab_ptr, int a4, int a5,
	       int a6, int a7, int a8, int a9, int a10)
{
    struct driver_table *tabptr;
    ULONG tick_used, tick_curr;
    STATUS sem_ret;
    int wait_time, itera;

    if (sizeof(int) >= sizeof(char *))
	tabptr = (struct driver_table *) msw_tab_ptr;
    else
	tabptr = (struct driver_table *) ((long) msw_tab_ptr << 16 | (long) lsw_tab_ptr);
    
    tick_used = tickGet();

    for(;;)
    {
	if (*tabptr->any_inmotion_ptr == 0)
	    wait_time = WAIT_FOREVER;
	else
	{
	    tick_curr = tickGet() - tick_used;
	    if (tick_curr < (ULONG) scan_rate)
	    {
		wait_time = scan_rate - tick_curr;
		if (wait_time < 1)		
		    wait_time = 1;
	    }
	    else
		wait_time = 1;
	}

	sem_ret = semTake(*tabptr->semptr, wait_time);
	tick_used = tickGet();

	if (*tabptr->any_inmotion_ptr)
	{
	    if (tabptr->strtstat != NULL)
		(*tabptr->strtstat) (ALL_CARDS);	/* Start data area update on motor cards */

	    for (itera = 0; itera < *tabptr->cardcnt_ptr; itera++)
	    {
		struct controller *brdptr = (*tabptr->card_array)[itera];
		if (brdptr != NULL && brdptr->motor_in_motion)
		    query_axis(itera, tabptr, tick_used);
	    }
	}
	if (sem_ret != ERROR)
	    process_messages(tabptr, tick_used);
    }
    return(0);
}


static int query_axis(int card, struct driver_table *tabptr, ULONG tick)
{
    int index;
    struct controller *brdptr;

    brdptr = (*tabptr->card_array)[card];

    for (index = 0; index < brdptr->total_axis; index++)
    {
	register struct mess_info *motor_info;
	register struct mess_node *motor_motion;
	ULONG delay;

	motor_info = &(brdptr->motor_info[index]);
	if ((motor_motion = motor_info->motor_motion) != 0)
	{
	    if (tick >= motor_info->status_delay)
		delay = tick - motor_info->status_delay;
	    else
		delay = tick + (0xFFFFFFFF - motor_info->status_delay);
	}

	if (motor_motion && (delay >= 2) && (*tabptr->setstat) (card, index))
	{
	    struct mess_node *mess_ret;

	    motor_motion->position = motor_info->position;
	    motor_motion->encoder_position = motor_info->encoder_position;
	    motor_motion->velocity = motor_info->velocity;
	    motor_motion->status = motor_info->status;

	    mess_ret = (struct mess_node *) motor_malloc(tabptr->freeptr, tabptr->freelockptr);
	    mess_ret->callback = motor_motion->callback;
	    mess_ret->mrecord = motor_motion->mrecord;
	    mess_ret->position = motor_motion->position;
	    mess_ret->encoder_position = motor_motion->encoder_position;
	    mess_ret->velocity = motor_motion->velocity;
	    mess_ret->status = motor_motion->status;
	    mess_ret->type = motor_motion->type;

	    if (motor_motion->status & RA_OVERTRAVEL ||
		motor_motion->status & RA_DONE ||
		motor_motion->status & RA_PROBLEM)
	    {
		(*tabptr->query_done) (card, index, motor_motion);
		brdptr->motor_in_motion--;
		motor_free(motor_motion, tabptr);
		motor_motion = (struct mess_node *) NULL;
		motor_info->motor_motion = (struct mess_node *) NULL;
	    }

	    callbackRequest(&mess_ret->callback);

	    if (brdptr->motor_in_motion == 0)
	    {
		SET_MM_OFF(*tabptr->any_inmotion_ptr, card);
	    }
	}
    }
    return (0);
}


static void process_messages(struct driver_table *tabptr, ULONG tick)
{
    struct mess_node *node, *motor_motion;
    ULONG delay;

    while ((node = get_head_node(tabptr)))
    {
	if ((node->card >= 0 && node->card < *tabptr->cardcnt_ptr) &&
	    (*tabptr->card_array)[node->card] &&
	    (node->signal >= 0 && node->signal < (*tabptr->card_array)[node->card]->total_axis))
	{
	    struct mess_info *motor_info;

	    motor_info = &((*tabptr->card_array)[node->card]->motor_info[node->signal]);
	    motor_motion = motor_info->motor_motion;

	    switch (node->type)
	    {
	    case QUERY:
		(*tabptr->sendmsg) (node->card, node->message, (char) NULL);
		(*tabptr->getmsg) (node->card, node->message, 1);
		callbackRequest((CALLBACK *) node);
		break;

	    case VELOCITY:
		(*tabptr->sendmsg) (node->card, node->message, (char) NULL);

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
		    (*tabptr->card_array)[node->card]->motor_in_motion++;
		else
		    motor_free(motor_motion, tabptr);

		SET_MM_ON(*tabptr->any_inmotion_ptr, node->card);
		motor_info->motor_motion = node;
		motor_info->status_delay = tick;
		break;

	    case MOTION:
		(*tabptr->sendmsg) (node->card, node->message, (char) NULL);

		/* this is tricky - see velocity comment */
		if (!motor_motion)	/* if NULL */
		    (*tabptr->card_array)[node->card]->motor_in_motion++;
		else
		    motor_free(motor_motion, tabptr);

		SET_MM_ON(*tabptr->any_inmotion_ptr, node->card);
		motor_info->no_motion_count = 0;
		motor_info->motor_motion = node;
		motor_info->status_delay = tick;
		break;

	    case INFO:
		if (tick >= motor_info->status_delay)
		    delay = tick - motor_info->status_delay;
		else
		    delay = tick + (0xFFFFFFFF - motor_info->status_delay);
		if (delay < 2)	/* Status update delay - needed for OMS. */
		    taskDelay((int) (2 - delay));	/* 2 RTOS tick delay. */

		if (tabptr->strtstat != NULL)
		    (*tabptr->strtstat) (node->card);
		(*tabptr->setstat) (node->card, node->signal);

		node->position = motor_info->position;
		node->encoder_position = motor_info->encoder_position;
		node->status = motor_info->status;

/*=============================================================================
* node->status & RA_DONE is not a reliable indicator of anything, in this case,
* since set_status() didn't ask the OMS controller to set the "Done" flag on
* command completion.
* Nevertheless, recMotor:process() needs to know whether the motor has stopped,
* and this we can tell by looking for a struct motor_motion.
==============================================================================*/
		if (motor_motion)
		    node->status &= ~RA_DONE;
		else
		    node->status |= RA_DONE;

		callbackRequest((CALLBACK *) node);
		break;

	    case MOVE_TERM:
		(*tabptr->sendmsg) (node->card, node->message, (char) NULL);
		motor_free(node, tabptr);	/* free message buffer */
		break;

	    default:
		(*tabptr->sendmsg) (node->card, node->message, (char) NULL);
		motor_free(node, tabptr);	/* free message buffer */
		motor_info->status_delay = tick;
		break;
	    }
	}
	else
	{
	    node->position = 0;
	    node->encoder_position = 0;
	    node->velocity = 0;
	    node->status = RA_PROBLEM;
	    callbackRequest((CALLBACK *) node);
	}
    }
}


/*****************************************************/
/* Get a message off the queue */
/* get_head_node()			     */
/*****************************************************/
static struct mess_node *get_head_node(struct driver_table *tabptr)
{
    struct circ_queue *qptr;
    struct mess_node *node;

    FASTLOCK(tabptr->quelockptr);
    qptr = tabptr->queptr;
    node = qptr->head;

    /* delete node from list */
    if (node)
    {
	if (node == qptr->head)
	    qptr->head = node->next;
	if (node == qptr->tail)
	    qptr->tail = NULL;
    }
    FASTUNLOCK(tabptr->quelockptr);
    return (node);
}

/*
 * FUNCTION... motor_send()
 *
 * USAGE... Send a command to the motor task queue.
 *
 * LOGIC...
 *  Allocate message node - call motor_malloc().
 *  Copy info. from input node to new node.
 *  Process node based on "type".
 *  Lock motor task queue.
 *  Insert new node at tail of queue.
 */

int motor_send(struct mess_node *u_msg, struct driver_table *tabptr, char *cmnd_line_terminator)
{
    struct mess_node *new_message;
    struct circ_queue *qptr;

    new_message = motor_malloc(tabptr->freeptr, tabptr->freelockptr);
    new_message->callback = u_msg->callback;
    new_message->next = (struct mess_node *) NULL;
    new_message->type = u_msg->type;
    new_message->signal = u_msg->signal;
    new_message->card = u_msg->card;
    new_message->mrecord = u_msg->mrecord;
    new_message->status = 0;
    strcpy(new_message->message, u_msg->message);
    new_message->postmsgptr = u_msg->postmsgptr;
    new_message->termstring = u_msg->termstring;

    switch (new_message->type)
    {
	case MOVE_TERM:
	case MOTION:
	    if (new_message->termstring != NULL)
		strcat(new_message->message, new_message->termstring);
	    break;
	case VELOCITY:
	case IMMEDIATE:
	case QUERY:
	case INFO:
	    break;
	default:
	    return (-1);
    }

    strcat(new_message->message, cmnd_line_terminator);

    FASTLOCK(tabptr->quelockptr);
    qptr = tabptr->queptr;
    if (qptr->tail)
    {
	qptr->tail->next = new_message;
	qptr->tail = new_message;
    }
    else
    {
	qptr->tail = new_message;
	qptr->head = new_message;
    }
    FASTUNLOCK(tabptr->quelockptr);

    semGive(*tabptr->semptr);
    return (0);
}

static struct mess_node *motor_malloc(struct circ_queue *freelistptr, FAST_LOCK *lockptr)
{
    struct mess_node *node;

    FASTLOCK(lockptr);

    if (!freelistptr->head)
	node = (struct mess_node *) malloc(sizeof(struct mess_node));
    else
    {
	node = freelistptr->head;
	freelistptr->head = node->next;
	if (!freelistptr->head)
	    freelistptr->tail = (struct mess_node *) NULL;
    }
    FASTUNLOCK(lockptr);
    return (node);
}

int motor_free(struct mess_node * node, struct driver_table *tabptr)
{
    struct circ_queue *freelistptr;
    
    freelistptr = tabptr->freeptr;
    FASTLOCK(tabptr->freelockptr);

    node->next = (struct mess_node *) NULL;

    if (freelistptr->tail)
    {
	freelistptr->tail->next = node;
	freelistptr->tail = node;
    }
    else
    {
	freelistptr->head = node;
	freelistptr->tail = node;
    }

    FASTUNLOCK(tabptr->freelockptr);
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
int motor_card_info(int card, MOTOR_CARD_QUERY * cq, struct driver_table *tabptr)
{
    struct controller *brdptr;

    brdptr = (*tabptr->card_array)[card];
    if (card >= 0 &&
	card < *tabptr->cardcnt_ptr &&
	brdptr != NULL)
    {
	cq->total_axis = brdptr->total_axis;
	cq->card_name = brdptr->ident;
	cq->axis_names = oms_trans_axis;
    }
    else
	cq->total_axis = 0;

    return (0);
}

/* return information for an axis to the caller */
int motor_axis_info(int card, int signal, MOTOR_AXIS_QUERY * aq, struct driver_table *tabptr)
{
    struct controller *brdptr;

    brdptr = (*tabptr->card_array)[card];
    if (card >= 0 &&
	card < *tabptr->cardcnt_ptr &&
	brdptr != NULL &&
	signal >= 0 &&
	signal < brdptr->total_axis)
    {
	aq->position = brdptr->motor_info[signal].position;
	aq->encoder_position =
	    brdptr->motor_info[signal].encoder_position;
	aq->status = brdptr->motor_info[signal].status;
    }
    else
    {
	aq->position = aq->encoder_position = 0;
	aq->status = RA_PROBLEM;
    }

    return (0);
}

