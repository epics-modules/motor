/*
FILENAME...	drvIM483PL.cc
USAGE...	Motor record driver level support for Intelligent Motion
		Systems, Inc. IM483(I/IE).

Version:	$Revision: 1.4 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-11-07 22:26:59 $
*/

/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/


/*
 *      Original Author: Ron Sluiter
 *      Date: 07/10/2000
 *
 * Modification Log:
 * -----------------
 * .01 07/10/00 rls copied from drvIM483SM.c
 * .02 10/02/01 rls allow one retry after a communication error.
 * .03 04/15/02 rls Bug fix for limit switches. Set RA_DIRECTION in
 *		    set_status() based on (new - old) commanded position.
 *		    Removed support for "ASCII record separator (IS2) = /x1E"
 *		    from send_mess().
 * .04  03/07/03 rls R3.14 conversion.
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the IM483 must be powered-on when EPICS is first
	booted up.
    2 - The IM483 cannot be power cycled while EPICS is up and running.  The
	consequences are permanent communication lose with the IM483 until
	EPICS is rebooted.
    3 - Like the Newport MM3000, the IM483's position can only be set to zero.
    4 - The IM483 uses an internal look-up table for acceleration/deceleration.
	Translation between the IM483 and the ACCL/BACC fields is not obvious.
*/

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "drvIM483.h"
#include "serialIO.h"
#include "epicsExport.h"

#define STATIC static

/* Read Limit Status response values. */
#define L_ALIMIT	1
#define L_BLIMIT	2
#define L_BOTH_LIMITS	3


#define IM483PL_NUM_CARDS	8
#define MAX_AXES		8
#define BUFF_SIZE 13		/* Maximum length of string to/from IM483PL */


/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int drvIM483PLdebug = 0;
	#define Debug(l, f, args...) {if (l <= drvIM483PLdebug) printf(f, ## args);}
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

/* --- Local data. --- */
int IM483PL_num_cards = 0;
STATIC char IM483PL_axis[8] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC RTN_STATUS send_mess(int card, char const *com, char c);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table IM483PL_access =
{
    motor_init,
    motor_send,
    motor_free,
    motor_card_info,
    motor_axis_info,
    &mess_queue,
    &queue_lock,
    &free_list,
    &freelist_lock,
    &motor_sem,
    &motor_state,
    &total_cards,
    &any_motor_in_motion,
    send_mess,
    recv_mess,
    set_status,
    query_done,
    NULL,
    &initialized,
    IM483PL_axis
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
} drvIM483PL = {2, report, init};

epicsExportAddress(drvet, drvIM483PL);

STATIC struct thread_args targs = {SCAN_RATE, &IM483PL_access};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (IM483PL_num_cards <=0)
	printf("    No IM483PL controllers configured.\n");
    else
    {
	for (card = 0; card < IM483PL_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    IM483PL controller %d connection failed.\n", card);
	    else
	    {
		struct IM483controller *cntrl;

		cntrl = (struct IM483controller *) brdptr->DevicePrivate;
		switch (cntrl->port_type)
		{
		case RS232_PORT: 
		    printf("    IM483PL controller %d port type = RS-232, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		default:
		    printf("    IM483PL controller %d port type = Unknown, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		}
	    }
	}
    }
    return(OK);
}


static long init()
{
   /* 
    * We cannot call motor_init() here, because that function can do GPIB I/O,
    * and hence requires that the drvGPIB have already been initialized.
    * That cannot be guaranteed, so we need to call motor_init from device
    * support
    */
    /* Check for setup */
    if (IM483PL_num_cards <= 0)
    {
	Debug(1, "init(): IM483PL driver disabled. IM483PLSetup() missing from startup script.\n");
    }
    return((long) 0);
}


STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/********************************************************************************
*										*
* FUNCTION NAME: set_status							*
*										*
* LOGIC:									*
*   Initialize.									*
*   Send "Moving Status" query.							*
*   Read response.								*
*   IF normal response to query.						*
*	Set communication status to NORMAL.					*
*   ELSE									*
*	IF communication status is NORMAL.					*
*	    Set communication status to RETRY.					*
*	    NORMAL EXIT.							*
*	ELSE									*
*	    Set communication status error.					*
*	    ERROR EXIT.								*
*	ENDIF									*
*   ENDIF									*
*										*
*   IF "Moving Status" indicates any motion (i.e. status != 0).			*
*	Clear "Done Moving" status bit.						*
*   ELSE									*
*	Set "Done Moving" status bit.						*
*   ENDIF									*
*										*
*   										*
********************************************************************************/

STATIC int set_status(int card, int signal)
{
    struct IM483controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    int status;
    int rtn_state;
    double motorData;
    bool plusdir, ls_active = false;

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    send_mess(card, "? ^", IM483PL_axis[signal]);
    rtn_state = recv_mess(card, buff, 1);
    if (rtn_state > 0)
    {
	cntrl->status = NORMAL;
	motor_info->status &= ~CNTRL_COMM_ERR;
    }
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    return(0);
	}
	else
	{
	    cntrl->status = COMM_ERR;
	    motor_info->status |= CNTRL_COMM_ERR;
	    motor_info->status |= RA_PROBLEM;
	    return(1);
	}
    }

    status = atoi(&buff[4]);

    if (status != 0)
	motor_info->status &= ~RA_DONE;
    else
	motor_info->status |= RA_DONE;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    send_mess(card, "? Z 0", IM483PL_axis[signal]);
    recv_mess(card, buff, 1);

    motorData = atof(&buff[5]);

    if (motorData == motor_info->position)
	motor_info->no_motion_count++;
    else
    {
	epicsInt32 newposition;

	newposition = NINT(motorData);
	if (newposition >= motor_info->position)
	    motor_info->status |= RA_DIRECTION;
	else
	    motor_info->status &= ~RA_DIRECTION;
	motor_info->position = newposition;
	motor_info->no_motion_count = 0;
    }

    plusdir = (motor_info->status & RA_DIRECTION) ? true : false;

    send_mess(card, "? ] 0", IM483PL_axis[signal]);
    recv_mess(card, buff, 1);
    status = atoi(&buff[5]);

    /* Set limit switch error indicators. */
    if (status & 1)
    {
	motor_info->status |= RA_PLUS_LS;
	if (plusdir == true)
	    ls_active = true;
    }
    else
	motor_info->status &= ~RA_PLUS_LS;

    if (status & 2)
    {
	motor_info->status |= RA_MINUS_LS;
	if (plusdir == false)
	    ls_active = true;
    }
    else
	motor_info->status &= ~RA_MINUS_LS;

    send_mess(card, "? ] 1", IM483PL_axis[signal]);
    recv_mess(card, buff, 1);
    status = buff[5];

    if (status & 0x01)
	motor_info->status |= RA_HOME;
    else
	motor_info->status &= ~RA_HOME;

    /* !!! Assume no closed-looped control!!!*/
    motor_info->status &= ~EA_POSITION;

    /* encoder status */
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;

    if (motor_state[card]->motor_info[signal].encoder_present == NO)
	motor_info->encoder_position = 0;
    else
    {
	send_mess(card, "? z 0", IM483PL_axis[signal]);
	recv_mess(card, buff, 1);
	motorData = atof(&buff[5]);
	motor_info->encoder_position = (int32_t) motorData;
    }

    motor_info->status &= ~RA_PROBLEM;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!(motor_info->status & RA_DIRECTION))
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
     (motor_info->status & (RA_DONE | RA_PROBLEM))) ? 1 : 0;

    /* Test for post-move string. */
    if ((motor_info->status & RA_DONE || ls_active == true) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
	strcpy(buff, nodeptr->postmsgptr);
	send_mess(card, buff, IM483PL_axis[signal]);
	nodeptr->postmsgptr = NULL;
    }

    return(rtn_state);
}


/*****************************************************/
/* send a message to the IM483PL board		     */
/* send_mess()			                     */
/*****************************************************/
STATIC RTN_STATUS send_mess(int card, char const *com, char inchar)
{
    char local_buff[MAX_MSG_SIZE];
    struct IM483controller *cntrl;
    int size;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvIM483PL.c:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvIM483PL.c:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcpy(local_buff, com);
    strcat(local_buff, "\n");

    if (inchar != (char) NULL)
	local_buff[0] = inchar;	    /* put in axis */

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    cntrl->serialInfo->serialIOSend(local_buff, strlen(local_buff), SERIAL_TIMEOUT);

    return(OK);
}


/*****************************************************/
/* receive a message from the IM483 board           */
/* recv_mess()			                     */
/*****************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    struct IM483controller *cntrl;
    int timeout;
    int len=0;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= SERIAL_TIMEOUT;

    len = cntrl->serialInfo->serialIORecv(com, BUFF_SIZE, (char *) "\n", timeout);

    if (len == 0)
	com[0] = '\0';
    else
	com[len - 1] = '\0';

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(len);
}


/*****************************************************/
/* Setup system configuration                        */
/* IM483PLSetup()                                     */
/*****************************************************/
RTN_STATUS
IM483PLSetup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > IM483PL_NUM_CARDS)
	IM483PL_num_cards = IM483PL_NUM_CARDS;
    else
	IM483PL_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before IM483Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(IM483PL_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < IM483PL_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* IM483PLConfig()                                    */
/*****************************************************/
RTN_STATUS
IM483PLConfig(int card,	/* card being configured */
              int port_type,      /* N/A - always RS232_PORT */
	      int location,       /* MPF server location */
              const char *name)   /* MPF server task name */
{
    struct IM483controller *cntrl;

    if (card < 0 || card >= IM483PL_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct IM483controller));
    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    cntrl->port_type = RS232_PORT;
    cntrl->serial_card = location;
    strcpy(cntrl->serial_task, name);
    return(OK);
}


/*****************************************************/
/* initialize all software and hardware		     */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()			                     */
/*****************************************************/
STATIC int motor_init()
{
    struct controller *brdptr;
    struct IM483controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status;
    bool success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (IM483PL_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < IM483PL_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->ident[0] = (char) NULL;	/* No controller identification message. */
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct IM483controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = false;
	cntrl->serialInfo = new serialIO(cntrl->serial_card,
				     cntrl->serial_task, &success_rtn);

	if (success_rtn == true)
	{
	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    do
		recv_mess(card_index, buff, FLUSH);
	    while (strlen(buff) != 0);
    
	    for (total_axis = 0; total_axis < MAX_AXES; total_axis++)
	    {
		send_mess(card_index, "? Z 0", IM483PL_axis[total_axis]);
		status = recv_mess(card_index, buff, 1);
		if (status <= 0)
		    break;
	    }
	    brdptr->total_axis = total_axis;
	}

	if (success_rtn == true && total_axis > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];
		int loop_state;

		motor_info->status = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* Assume encoder support, i.e., IM483IE. */
		motor_info->encoder_present = YES;
		motor_info->status |= EA_PRESENT;

                /* Determine if encoder present based on open/closed loop mode. */
		loop_state = 0;
		if (loop_state != 0)
		{
		    motor_info->pid_present = YES;
		    motor_info->status |= GAIN_SUPPORT;
		}

		set_status(card_index, motor_index);  /* Read status of each motor */
	    }
	}
	else
	    motor_state[card_index] = (struct controller *) NULL;
    }

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "IM483PL_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);
    
    return(OK);
}

