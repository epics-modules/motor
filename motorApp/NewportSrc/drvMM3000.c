/*
FILENAME...	drvMM3000.c
USAGE...	Motor record driver level support for Newport MM3000.

Version:	$Revision: 1.5 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-07-17 17:20:28 $
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/97
 *	Current Author: Ron Sluiter
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
 * .01  10-20-97	mlr     initialized from drvOms58
 * .02  10-30-97	mlr     Replaced driver calls with gpipIO functions
 * .03  10-30-98	mlr     Minor code cleanup, improved formatting
 * .04  02-01-99	mlr     Added temporary fix to delay reading motor
 *                      	positions at the end of a move.
 * .05  04-18-00	rls	MM3000 takes 2 to 5 seconds to respond to
 *				queries after hitting a hard travel limit.
 *				Adjusted GPIB and SERIAL timeouts accordingly.
 *				Deleted communication retries.  Reworked travel
 *				limit processing so that direction status bit
 *				matches limit switch.  Copied recv_mess() logic
 *				from drvMM4000.c.  Use TPE command to determine
 *				if motor has an encoder.
 */


#include	<vxWorks.h>
#include	<stdio.h>
#include	<sysLib.h>
#include	<string.h>
#include	<taskLib.h>
#include	<rngLib.h>
#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<fast_lock.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<errMdef.h>
#include	<logLib.h>

#include	"motor.h"
#include	"drvMMCom.h"
#include	"gpibIO.h"
#include	"serialIO.h"

#define STATIC static

#define READ_RESOLUTION "TU;"
#define READ_STATUS     "MS;"
#define READ_POSITION   "TP;"
#define STOP_ALL        "ST"
#define MOTOR_ON        "MO;"
#define REMOTE_MODE     "MR;"
#define GET_IDENT       "VE"

#define INPUT_TERMINATOR  '\n'

/* Status byte bits */
#define M_AXIS_MOVING     0x01
#define M_MOTOR_POWER     0x02
#define M_MOTOR_DIRECTION 0x04
#define M_PLUS_LIMIT      0x08
#define M_MINUS_LIMIT     0x10
#define M_HOME_SIGNAL     0x20

#define MM3000_NUM_CARDS	4
#define BUFF_SIZE 100       /* Maximum length of string to/from MM3000 */

/* The MM3000 does not respond for 2 to 5 seconds after hitting a travel limit. */
#define GPIB_TIMEOUT	5000	/* Command timeout in msec. */
#define SERIAL_TIMEOUT	5000	/* Command timeout in msec. */

/*----------------debugging-----------------*/
#ifdef	DEBUG
    #define Debug(l, f, args...) { if(l<=drvMM3000debug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

int MM3000_num_cards = 0;
volatile int drvMM3000debug = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvMM3000ReadbackDelay = 0.;


/*----------------functions-----------------*/
STATIC int recv_mess(int card, char *com, int flag);
STATIC int send_mess(int card, char const *com, char c);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MM3000_access =
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
} drvMM3000 = {2, report, init};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (MM3000_num_cards <=0)
	printf("    No MM3000 controllers configured.\n");
    else
    {
	for (card = 0; card < MM3000_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    MM3000 controller %d connection failed.\n", card);
	    else
	    {
		struct MMcontroller *cntrl;

		cntrl = (struct MMcontroller *) brdptr->DevicePrivate;
		switch (cntrl->port_type)
		{
		case RS232_PORT: 
		    printf("    MM3000 controller %d port type = RS-232, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		case GPIB_PORT:
		    printf("    MM3000 controller %d port type = GPIB, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		default:
		    printf("    MM3000 controller %d port type = Unknown, id: %s \n", 
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
    if (MM3000_num_cards <= 0)
    {
	Debug(1, "init(): MM3000 driver disabled. MM3000Setup() missing from startup script.\n");
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
    struct MMcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char *cptr, *tok_save;
    char inbuff[BUFF_SIZE], outbuff[BUFF_SIZE];
    int status, rtn_state;
    double motorData;

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);

    sprintf(outbuff, "%dMS", signal + 1);
    send_mess(card, outbuff, NULL);
    status = recv_mess(card, inbuff, 1);
    if (status <= 0)
    {
	cntrl->status = COMM_ERR;
	motor_info->status |= CNTRL_COMM_ERR;
	return (rtn_state = 1);
    }
    else
    {
	cntrl->status = NORMAL;
	motor_info->status &= ~CNTRL_COMM_ERR;
    }

    status = inbuff[0];
    Debug(5, "set_status(): status byte = %x\n", status);

    nodeptr = motor_info->motor_motion;

    if (status & M_MOTOR_DIRECTION)
	motor_info->status |= RA_DIRECTION;
    else
	motor_info->status &= ~RA_DIRECTION;

    if (status & M_AXIS_MOVING)
	motor_info->status &= ~RA_DONE;
    else
    {
	motor_info->status |= RA_DONE;
/* TEMPORARY FIX, Mark Rivers, 2/1/99. The MM3000 has reported that the
 * motor is done moving, which means that the "jerk time" is done.  However,
 * the axis can still be settling.  For now we put in a delay and poll the
 * motor position again. This is not a good long-term solution.
 */
	if (motor_info->pid_present == YES && drvMM3000ReadbackDelay != 0.)
	{
	    taskDelay((int)(drvMM3000ReadbackDelay * sysClkRateGet()));
	    send_mess(card, READ_POSITION, NULL);
	    recv_mess(card, cntrl->position_string, 1);
	}
    }

    /* Set Travel limit status bit. */
    if (status & M_AXIS_MOVING)
    {
	if (((status & M_PLUS_LIMIT)  &&  (status & M_MOTOR_DIRECTION)) ||
	    ((status & M_MINUS_LIMIT) && !(status & M_MOTOR_DIRECTION)))
	    motor_info->status |= RA_OVERTRAVEL;
	else
	    motor_info->status &= ~RA_OVERTRAVEL;
    }
    else
    {
	if ((status & M_PLUS_LIMIT) || (status & M_MINUS_LIMIT))
	{
	    motor_info->status |= RA_OVERTRAVEL;
	    /* Until status is modified to distinguish +/- limits;
	     * Set RA_DIRECTION to match travel limit switch. */
	    if (status & M_PLUS_LIMIT)
		motor_info->status |= RA_DIRECTION;
	    else
		motor_info->status &= ~RA_DIRECTION;
	}
	else
	    motor_info->status &= ~RA_OVERTRAVEL;
    }

    if (status & M_HOME_SIGNAL)
	motor_info->status |= RA_HOME;
    else
	motor_info->status &= ~RA_HOME;

    if (status & M_MOTOR_POWER)
	motor_info->status &= ~EA_POSITION;
    else
	motor_info->status |= EA_POSITION;

    /* encoder status */
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;

    sprintf(outbuff, "%dTP", signal + 1);
    send_mess(card, outbuff, NULL);
    status = recv_mess(card, inbuff, 1);
    if (status <= 0)
    {
	cntrl->status = COMM_ERR;
	motor_info->status |= CNTRL_COMM_ERR;
	return (rtn_state = 1);
    }
    else
    {
	cntrl->status = NORMAL;
	motor_info->status &= ~CNTRL_COMM_ERR;
    }

    tok_save = NULL;
    cptr = strtok_r(inbuff, " ", &tok_save);
    motorData = atof(cptr);

    if (motorData == motor_info->position)
	motor_info->no_motion_count++;
    else
    {
	motor_info->position = (int32_t) motorData;
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
	strcpy(outbuff, nodeptr->postmsgptr);
	send_mess(card, outbuff, NULL);
	nodeptr->postmsgptr = NULL;
    }

    return (rtn_state);
}


/*****************************************************/
/* send a message to the MM3000 board		     */
/* send_mess()			                     */
/*****************************************************/
STATIC int send_mess(int card, char const *com, char inchar)
{
    struct MMcontroller *cntrl;
    char local_buff[BUFF_SIZE];

    if (strlen(com) > MAX_MSG_SIZE)
    {
	logMsg((char *) "drvMM3000:send_mess(); message size violation.\n",
	       0, 0, 0, 0, 0, 0);
	return (-1);
    }
    
    if (!motor_state[card])
    {
	logMsg((char *) "drvMM3000:send_mess() - invalid card #%d\n", card,
	       0, 0, 0, 0, 0);
	return (-1);
    }

    if (inchar != (char) NULL)
    {
	logMsg((char *) "drvMM3000:send_mess() - invalid argument = %c\n", inchar,
	       0, 0, 0, 0, 0);
	return (-1);
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcpy(local_buff, com);
    strcat(local_buff, "\r");
    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    if (cntrl->port_type == GPIB_PORT)
	gpibIOSend(cntrl->gpibInfo, local_buff, strlen(local_buff), GPIB_TIMEOUT);
    else        
	serialIOSend(cntrl->serialInfo, local_buff, strlen(local_buff), SERIAL_TIMEOUT);
    
    return (0);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int flag)
 *
 * INPUT ARGUMENTS...
 *	card - controller card # (0,1,...).
 *	*com - caller's response buffer.
 *	flag	| FLUSH  = flush controller's output buffer; set timeout = 0.
 *		| !FLUSH = retrieve response into caller's buffer; set timeout.
 *
 * LOGIC...
 *  Initialize:
 *	- receive timeout to zero
 *	- received string length to zero.
 *
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *
 *  SWITCH on port type.
 *	CASE port type is GPIB.
 *	    BREAK.
 *	CASE port type is RS232.
 *	    IF input "flag" indicates NOT flushing the input buffer.
 *		Set receive timeout nonzero.
 *	    ENDIF
 *	    Call serialIORecv().
 *
 *	    NOTE: The MM3000 sometimes responds to an MS command with an error
 *		message (see MM3000 User's Manual Appendix A).  This is an
 *		unconfirmed MM3000 bug.  Retry read if this is a Hard Travel
 *		limit switch error. This effectively flushes the error message.
 *
 *	    IF input "com" buffer length is > 3 characters, AND, the 1st
 *			character is an "E" (Maybe this an unsolicited error
 *			message response?).
 *	    	Call serialIORecv().
 *	    ENDIF
 *	    BREAK
 *    ENDSWITCH
 *		
 *  NORMAL RETURN.
 */

STATIC int recv_mess(int card, char *com, int flag)
{
    struct MMcontroller *cntrl;
    int timeout = 0;
    int len = 0;

    /* Check that card exists */
    if (!motor_state[card])
	return (-1);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    switch (cntrl->port_type)
    {
	case GPIB_PORT:
	    if (flag != FLUSH)
		timeout	= GPIB_TIMEOUT;
	    len = gpibIORecv(cntrl->gpibInfo, com, BUFF_SIZE, INPUT_TERMINATOR,
			     timeout);
	    break;
	case RS232_PORT:
	    if (flag != FLUSH)
		timeout	= SERIAL_TIMEOUT;
	    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE,
			       INPUT_TERMINATOR, timeout);
	    if (len > 3 && com[0] == 'E')
	    {
		long error;

		error = strtol(&com[1], NULL, NULL);
		if (error >= 35 && error <= 42)
		    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE,
				       INPUT_TERMINATOR, timeout);
	    }
	    break;
    }

    if (len <= 0)
    {
	com[0] = '\0';
	len = 0;
    }
    else
	/* MM3000 responses are always terminated with CR/LF combination (see
	 * MM3000 User' Manual Sec. 3.4 NOTE). Strip both CR&LF from buffer
	 * before returning to caller.
	 */
	com[len-2] = '\0';

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (len);
}


/*****************************************************/
/* Setup system configuration                        */
/* MM3000Setup()                                     */
/*****************************************************/
int MM3000Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > MM3000_NUM_CARDS)
	MM3000_num_cards = MM3000_NUM_CARDS;
    else
	MM3000_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before MM3000Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(MM3000_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < MM3000_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return (0);
}


/*****************************************************/
/* Configure a controller                            */
/* MM3000Config()                                    */
/*****************************************************/
int MM3000Config(int card,	/* card being configured */
            PortType port_type,	/* GPIB_PORT or RS232_PORT */
	    int addr1,          /* = link for GPIB or hideos_card for RS-232 */
            int addr2)          /* GPIB address or hideos_task */
{
    struct MMcontroller *cntrl;

    if (card < 0 || card >= MM3000_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MMcontroller));
    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

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
    struct MMcontroller *cntrl;
    int card_index, motor_index, arg3, arg4;
    char axis_pos[BUFF_SIZE];
    char buff[BUFF_SIZE];
    char *tok_save, *bufptr;
    int total_axis = 0;
    int status;
    BOOLEAN errind;

    initialized = ON;	/* Indicate that driver is initialized. */
    
    /* Check for setup */
    if (MM3000_num_cards <= 0)
	return (ERROR);

    tok_save = NULL;

    for (card_index = 0; card_index < MM3000_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = OFF;
	total_cards = card_index + 1;
	cntrl = (struct MMcontroller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	errind = OFF;
	switch (cntrl->port_type)
	{
	    case GPIB_PORT:
		cntrl->gpibInfo = gpibIOInit(cntrl->gpib_link,
					     cntrl->gpib_address);
		if (cntrl->gpibInfo == NULL)
		    errind = ON;
		break;
	    case RS232_PORT:
		cntrl->serialInfo = serialIOInit(cntrl->serial_card,
						 cntrl->serial_task);
		if (cntrl->serialInfo == NULL)
		    errind = ON;
		break;
	}

	if (errind == OFF)
	{
	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    do
		recv_mess(card_index, buff, FLUSH);
	    while (strlen(buff) != 0);
    
	    send_mess(card_index, GET_IDENT, NULL);
	    status = recv_mess(card_index, axis_pos, 1);  
	    /* Return value is length of response string */
	}

	if (errind == OFF && status > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    send_mess(card_index, STOP_ALL, NULL);	/* Stop all motors */
	    send_mess(card_index, GET_IDENT, NULL);	/* Read controller ID string */
	    recv_mess(card_index, buff, 1);
	    strcpy(brdptr->ident, &buff[0]);  /* Skip "VE" */

	    send_mess(card_index, "RC", NULL);
	    recv_mess(card_index, buff, 1);
	    bufptr = strtok_r(buff, "=", &tok_save);
	    bufptr = strtok_r(NULL, " ", &tok_save);

	    /* The return string will tell us how many axes this controller has */
	    for (total_axis = 0; bufptr != NULL; total_axis++)
	    {
		if (strcmp(bufptr, "unused") == 0)
		{
		    cntrl->type[total_axis] = UNUSED;
		    bufptr = NULL;
		}
		else
		{
		    if (strcmp(bufptr, "stepper1.5M") == 0)
			cntrl->type[total_axis] = STEPPER;
		    else if (strcmp(bufptr, "dc") == 0)
			cntrl->type[total_axis] = DC;
		    else
			logMsg((char *) "drvMM3000:motor_init() - invalid RC response = %s\n",
			       (int) bufptr, 0, 0, 0, 0, 0);

		    bufptr = strtok_r(NULL, "=", &tok_save);
		    bufptr = strtok_r(NULL, " ", &tok_save);
		}

    		/* Initialize. */
		brdptr->motor_info[total_axis].motor_motion = NULL;
	    }

	    brdptr->total_axis = total_axis;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;

		if (cntrl->type[total_axis] == DC)
		    motor_info->encoder_present = YES;
		else
		{
		    sprintf(buff, "%dTPE", motor_index + 1);
		    send_mess(card_index, buff, NULL);
		    recv_mess(card_index, buff, 1);

		    if (strcmp(buff, "E01") == 0)
			motor_info->encoder_present = NO;
		    else
			motor_info->encoder_present = YES;
		}
                
		if (motor_info->encoder_present == YES)
		{
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
	arg3 = (int) (&MM3000_access);
	arg4 = 0;
    }
    else
    {
	arg3 = (int) ((long) &MM3000_access >> 16);
	arg4 = (int) ((long) &MM3000_access & 0xFFFF);
    }
    taskSpawn((char *) "MM3000_motor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}

/*---------------------------------------------------------------------*/
