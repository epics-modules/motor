/*
FILENAME...	drvIM483SM.c
USAGE...	Motor record driver level support for Intelligent Motion
		Systems, Inc. IM483(I/IE).

Version:	$Revision: 1.1 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-07-25 13:36:44 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 02/10/2000
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
 * .01	02/10/2000	rls     copied from drvMM4000.c
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

#include	<vxWorks.h>
#include	<stdioLib.h>
#include	<sysLib.h>
#include	<string.h>
#include	<taskLib.h>
#include	<math.h>
#include        <rngLib.h>
#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<fast_lock.h>
#include	<recSup.h>
#include	<devSup.h>
#include        <errMdef.h>
#include	<logLib.h>
#include	<osiSleep.h>
#include	<ctype.h>

#include	"motor.h"
#include	"drvIM483.h"
#include        "gpibIO.h"
#include        "serialIO.h"

#define STATIC static
#define INPUT_TERMINATOR  '\n'

/* Read Limit Status response values. */
#define L_ALIMIT	1
#define L_BLIMIT	2
#define L_BOTH_LIMITS	3


#define IM483SM_NUM_CARDS	8
#define BUFF_SIZE 13		/* Maximum length of string to/from IM483 */

/*----------------debugging-----------------*/
#ifdef	DEBUG
    #define Debug(l, f, args...) { if(l<=drvIM483SMdebug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

/* --- Local data. --- */
int IM483SM_num_cards = 0;
volatile int drvIM483SMdebug = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvIM483SMReadbackDelay = 0.;


/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC int send_mess(int card, char const *com, char c);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table IM483SM_access =
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
    NULL
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
} drvIM483SM = {2, report, init};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (IM483SM_num_cards <=0)
	printf("    No IM483SM controllers configured.\n");
    else
    {
	for (card = 0; card < IM483SM_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    IM483SM controller %d connection failed.\n", card);
	    else
	    {
		struct IM483controller *cntrl;

		cntrl = (struct IM483controller *) brdptr->DevicePrivate;
		switch (cntrl->port_type)
		{
		case RS232_PORT: 
		    printf("    IM483SM controller %d port type = RS-232, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		case GPIB_PORT:
		    printf("    IM483SM controller %d port type = GPIB, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		default:
		    printf("    IM483SM controller %d port type = Unknown, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		}
	    }
	}
    }
    return (0);
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
    if (IM483SM_num_cards <= 0)
    {
	Debug(1, "init(): IM483SM driver disabled. IM483SMSetup() missing from startup script.\n");
    }
    return ((long) 0);
}


STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

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

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    send_mess(card, "^", NULL);
    rtn_state = recv_mess(card, buff, 1);
    if (rtn_state <= 0)
	cntrl->status = COMM_ERR;
    else
	cntrl->status = NORMAL;
    status = atoi(&buff[4]);

    if (status != 0)
	motor_info->status &= ~RA_DONE;
    else
    {
	motor_info->status |= RA_DONE;
/* TEMPORARY FIX, Mark Rivers, 2/1/99. The IM483 has reported that the
 * motor is done moving, which means that the "jerk time" is done.  However,
 * the axis can still be settling.  For now we put in a delay and poll the
 * motor position again. This is not a good long-term solution.
 */
	if (motor_info->pid_present == YES && drvIM483SMReadbackDelay != 0.)
	{
	    taskDelay((int)(drvIM483SMReadbackDelay * sysClkRateGet()));
	    send_mess(card, "Z 0", NULL);
	    recv_mess(card, buff, 1);
	}
    }

    send_mess(card, "] 0", NULL);
    recv_mess(card, buff, 1);
    status = atoi(&buff[5]);

    if (status == 0)
	motor_info->status &= ~RA_OVERTRAVEL;
    else
    {
	motor_info->status |= RA_OVERTRAVEL;
	/* Set direction based on which LS activated. */
	if (status == L_ALIMIT)
	    motor_info->status |= RA_DIRECTION;
	else
	    motor_info->status &= ~RA_DIRECTION;
    }

    send_mess(card, "] 1", NULL);
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

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */


    send_mess(card, "Z 0", NULL);
    recv_mess(card, buff, 1);

    motorData = atof(&buff[5]);

    if (motorData == motor_info->position)
	motor_info->no_motion_count++;
    else
    {
	motor_info->position = NINT(motorData);
	if (motor_state[card]->motor_info[signal].encoder_present == YES)
	    motor_info->encoder_position = (int32_t) motorData;
	else
	    motor_info->encoder_position = 0;

	motor_info->no_motion_count = 0;
    }

    motor_info->status &= ~RA_PROBLEM;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!(motor_info->status & RA_DIRECTION))
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count ||
		 (motor_info->status & (RA_OVERTRAVEL | RA_DONE | RA_PROBLEM))) ? 1 : 0;

    /* Test for post-move string. */
    if ((motor_info->status & RA_DONE || motor_info->status & RA_OVERTRAVEL) &&
	 nodeptr != 0 && nodeptr->postmsgptr != 0)
    {
	strcpy(buff, nodeptr->postmsgptr);
	send_mess(card, buff, NULL);
	nodeptr->postmsgptr = NULL;
    }

    if (cntrl->status == COMM_ERR)
	motor_info->status |= CNTRL_COMM_ERR;
    else
	motor_info->status &= ~CNTRL_COMM_ERR;

    return(rtn_state);
}


/*****************************************************/
/* send a message to the IM483SM board		     */
/* send_mess()			                     */
/*****************************************************/
STATIC int send_mess(int card, char const *com, char inchar)
{
    char *head, *end, local_buff[MAX_MSG_SIZE];
    struct IM483controller *cntrl;
    BOOLEAN lastcmnd = OFF;

    if (strlen(com) > MAX_MSG_SIZE)
    {
	logMsg((char *) "drvIM483SM.c:send_mess(); message size violation.\n",
	       0, 0, 0, 0, 0, 0);
	return (-1);
    }
    
    if (!motor_state[card])
    {
	logMsg((char *) "drvIM483SM.c:send_mess() - invalid card #%d\n", card,
	       0, 0, 0, 0, 0);
	return (-1);
    }

    if (inchar != (char) NULL)
    {
	logMsg((char *) "drvIM483SM.c:send_mess() - invalid argument = %c\n", inchar,
	       0, 0, 0, 0, 0);
	return (-1);
    }

    head = (char *) com;
    end = head + strlen(com);

    while (lastcmnd == OFF)
    {
	int size;
	char *delimit;

	delimit = strchr(head, ';');
	if (delimit != NULL)
	{
	    if (*(delimit + 1) == NULL)	/* Test for trailing command terminator. */
		lastcmnd = ON;
	    size = delimit - head;
	}
	else
	{
	    size = end - head;
	    lastcmnd = ON;
	}

	strncpy(local_buff, head, size);
	head += size + 1;
	local_buff[size++] = '\r';	/* Add the command line terminator. */
	local_buff[size] = '\0';
    
	if (inchar != (char) NULL)
	    local_buff[0] = inchar;		/* put in axis */
    
	Debug(2, "send_mess(): message = %s\n", local_buff);
    
	cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    
	serialIOSend(cntrl->serialInfo, local_buff, strlen(local_buff), SERIAL_TIMEOUT);
	if (lastcmnd == OFF)
	    recv_mess(card, local_buff, 1);
    }

    return (0);
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
	return (-1);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= SERIAL_TIMEOUT;

    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE, INPUT_TERMINATOR, timeout);

    if (len == 0)
	com[0] = '\0';
    else
	com[len - 2] = '\0';

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (len);
}


/*****************************************************/
/* Setup system configuration                        */
/* IM483SMSetup()                                     */
/*****************************************************/
int IM483SMSetup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > IM483SM_NUM_CARDS)
	IM483SM_num_cards = IM483SM_NUM_CARDS;
    else
	IM483SM_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before IM483SMConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(IM483SM_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < IM483SM_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return (0);
}


/*****************************************************/
/* Configure a controller                            */
/* IM483SMConfig()                                    */
/*****************************************************/
int IM483SMConfig(int card,	/* card being configured */
            int port_type,      /* GPIB_PORT or RS232_PORT */
	    int addr1,          /* = link for GPIB or hideos_card for RS-232 */
            int addr2)          /* GPIB address or hideos_task */
{
    struct IM483controller *cntrl;

    if (card < 0 || card >= IM483SM_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct IM483controller));
    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    switch (port_type)
    {
    case GPIB_PORT:
        cntrl->port_type = port_type;
        cntrl->gpib_link = addr1;
        cntrl->gpib_address = addr2;
        break;
    case RS232_PORT:
        cntrl->port_type = port_type;
        cntrl->serial_card = addr1;
        strcpy(cntrl->serial_task, (char *) addr2);
        break;
    default:
        return (ERROR);
    }
    return (0);
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
    int card_index, motor_index, arg3, arg4;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status;
    BOOLEAN errind;

    initialized = ON;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (IM483SM_num_cards <= 0)
	return (ERROR);

    for (card_index = 0; card_index < IM483SM_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->cmnd_response = ON;
	total_cards = card_index + 1;
	cntrl = (struct IM483controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	errind = OFF;
	cntrl->serialInfo = serialIOInit(cntrl->serial_card,
					 cntrl->serial_task);
	if (cntrl->serialInfo == NULL)
	    errind = ON;

	if (errind == OFF)
	{
	    int itera;
	    char *src, *dest;

	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    do
		recv_mess(card_index, buff, FLUSH);
	    while (strlen(buff) != 0);
    
	    send_mess(card_index, "\003", NULL);	/* Reset device. */
	    osiSleep(1, 0);
	    send_mess(card_index, " ", NULL);

	    /* Save controller identification message. */
	    src = buff;
	    dest = brdptr->ident;
	    *src = NULL;

	    for (itera = 0; itera < 50; itera++)
	    {
		if (*src == NULL)
		{
		    status = recv_mess(card_index, buff, 1);
		    if (status <= 0)
		    {
			if (itera != 0)
			{
			    *dest = NULL;
			    status = 1;
			}
			break;
		    }
		    src = buff;
		    while(isspace(*src++));
		    --src;
		    if (itera != 0)
		    {
			*dest++ = ' ';
			itera++;
		    }
		}
		else if (isspace(*src))
		{
		    while(isspace(*src++));
		    src -= 2;
		}
		else if (strncmp(src, "AD", 2) == 0)
		{
		    strcpy(dest, "AMS");
		    src += 22;
		    dest += 3;
		    itera += 3;
		}
		*dest++ = *src++;
	    }
	}

	if (errind == OFF && status > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

	    brdptr->total_axis = total_axis = 1;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];
		int loop_state;

		motor_info->status = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;

                /* Determine if encoder present based on open/closed loop mode. */
		loop_state = NULL;
		if (loop_state != 0)
		{
                    motor_info->encoder_present = YES;
		    motor_info->status |= EA_PRESENT;
		    motor_info->pid_present = YES;
		    motor_info->status |= GAIN_SUPPORT;
		}

		set_status(card_index, motor_index);  /* Read status of each motor */
	    }
	}
	else
	    motor_state[card_index] = (struct controller *) NULL;
    }

    motor_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
    any_motor_in_motion = 0;

    FASTLOCKINIT(&queue_lock);
    FASTLOCK(&queue_lock);
    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&queue_lock);

    FASTLOCKINIT(&freelist_lock);
    FASTLOCK(&freelist_lock);
    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&freelist_lock);

    if (sizeof(int) >= sizeof(char *))
    {
	arg3 = (int) (&IM483SM_access);
	arg4 = 0;
    }
    else
    {
	arg3 = (int) ((long) &IM483SM_access >> 16);
	arg4 = (int) ((long) &IM483SM_access & 0xFFFF);
    }
    taskSpawn((char *) "IM483SM_motor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}

