/*
FILENAME...	drvMM3000.cc
USAGE...	Motor record driver level support for Newport MM3000.

Version:	$Revision: 1.8 $
Modified By:	$Author: rivers $
Last Modified:	$Date: 2004-04-20 20:56:04 $
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
 * .01 10-20-97 mlr initialized from drvOms58
 * .02 10-30-97 mlr Replaced driver calls with gpipIO functions
 * .03 10-30-98 mlr Minor code cleanup, improved formatting
 * .04 02-01-99 mlr Added temporary fix to delay reading motor positions at
 *			the end of a move.
 * .05 04-18-00 rls MM3000 takes 2 to 5 seconds to respond to queries after
 *			hitting a hard travel limit.  Adjusted GPIB and SERIAL
 *			timeouts accordingly.  Deleted communication retries.
 *			Reworked travel limit processing so that direction
 *			status bit matches limit switch.  Copied recv_mess()
 *			logic from drvMM4000.c.  Use TPE command to determine
 *			if motor has an encoder.
 * .06 10/02/01 rls - allow one retry after a communication error.
 *		    - use motor status response bit-field.
 * .07 05-22-03	rls - Converted to R3.14.2.
 * .08 10/23/03 rls - recv_mess() eats the controller error response, outputs
 *			an error message and retries.
 * .09 02/03/04 rls - Eliminate erroneous "Motor motion timeout ERROR".
 *
 */

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "NewportRegister.h"
#include "drvMMCom.h"
#include "asynSyncIO.h"
#include "epicsExport.h"

#define STATIC static

#define READ_RESOLUTION "TU;"
#define READ_STATUS     "MS;"
#define READ_POSITION   "TP;"
#define STOP_ALL        "ST"
#define MOTOR_ON        "MO;"
#define REMOTE_MODE     "MR;"
#define GET_IDENT       "VE"

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
#define SERIAL_TIMEOUT	5.0	/* Command timeout in sec. */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int drvMM3000debug = 0;
	#define Debug(l, f, args...) { if(l<=drvMM3000debug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

/* --- Local data. --- */
int MM3000_num_cards = 0;

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

epicsExportAddress(drvet, drvMM3000);

STATIC struct thread_args targs = {SCAN_RATE, &MM3000_access};

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
	    	printf("    MM3000 controller %d asyn port= %s, address=%d, id: %s \n", 
			   card, cntrl->asyn_port, cntrl->asyn_address,
			   brdptr->ident);
		    break;
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
    if (MM3000_num_cards <= 0)
    {
	Debug(1, "init(): MM3000 driver disabled. MM3000Setup() missing from startup script.\n");
    }
    return((long) 0);
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
    MOTOR_STATUS mstat;
    int rtn_state, charcnt;
    double motorData;
    bool plusdir, ls_active = false;
    msta_field status;

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    status.All = motor_info->status.All;

    sprintf(outbuff, "%dMS", signal + 1);
    send_mess(card, outbuff, (char) NULL);
    charcnt = recv_mess(card, inbuff, 1);
    if (charcnt > 0)
    {
	cntrl->status = NORMAL;
	status.Bits.CNTRL_COMM_ERR = 0;
    }
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    rtn_state = 0;
	    goto exit;
	}
	else
	{
	    cntrl->status = COMM_ERR;
	    status.Bits.CNTRL_COMM_ERR = 1;
	    status.Bits.RA_PROBLEM     = 1;
	    rtn_state = 1;
	    goto exit;
	}
    }

    mstat.All = inbuff[0];
    Debug(5, "set_status(): status byte = %x\n", mstat.All);

    nodeptr = motor_info->motor_motion;

    status.Bits.RA_DIRECTION = (mstat.Bits.direction == false) ? 0 : 1;

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    status.Bits.RA_DONE = (mstat.Bits.inmotion == false) ? 1 : 0;

    /* Set Travel limit switch status bits. */
    if (mstat.Bits.plustTL == false)
	status.Bits.RA_PLUS_LS = 0;
    else
    {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	    ls_active = true;
    }

    if (mstat.Bits.minusTL == false)
	status.Bits.RA_MINUS_LS = 0;
    else
    {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	    ls_active = true;
    }

    status.Bits.RA_HOME = (mstat.Bits.homels == false) ? 0 : 1;

    status.Bits.EA_POSITION = (mstat.Bits.NOT_power == false) ? 1 : 0;

    /* encoder status */
    status.Bits.EA_SLIP 	= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;

    sprintf(outbuff, "%dTP", signal + 1);
    send_mess(card, outbuff, (char) NULL);
    charcnt = recv_mess(card, inbuff, 1);
    if (charcnt > 0)
    {
	cntrl->status = NORMAL;
	status.Bits.CNTRL_COMM_ERR = 0;
    }
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    rtn_state = 0;
	    goto exit;
	}
	else
	{
	    cntrl->status = COMM_ERR;
	    status.Bits.CNTRL_COMM_ERR = 1;
	    status.Bits.RA_PROBLEM     = 1;
	    rtn_state = 1;
	    goto exit;
	}
    }

    tok_save = NULL;
    cptr = strtok_r(inbuff, " ", &tok_save);
    motorData = atof(cptr);

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->position = (epicsInt32) motorData;
	if (motor_state[card]->motor_info[signal].encoder_present == YES)
	    motor_info->encoder_position = (epicsInt32) motorData;
	else
	    motor_info->encoder_position = 0;

	motor_info->no_motion_count = 0;
    }

    status.Bits.RA_PROBLEM = 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!status.Bits.RA_DIRECTION)
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
	strcpy(outbuff, nodeptr->postmsgptr);
	send_mess(card, outbuff, (char) NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the MM3000 board		     */
/* send_mess()			                     */
/*****************************************************/
STATIC RTN_STATUS send_mess(int card, char const *com, char inchar)
{
    struct MMcontroller *cntrl;
    char local_buff[BUFF_SIZE];
    int size;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvMM3000:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvMM3000:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (inchar != (char) NULL)
    {
	errlogPrintf("drvMM3000:send_mess() - invalid argument = %c\n", inchar);
	return(ERROR);
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcpy(local_buff, com);
    strcat(local_buff, "\r");
    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    pasynSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff), 
                       SERIAL_TIMEOUT);
    
    return(OK);
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
 *	    Call pasynSyncIO->read().
 *
 *	    NOTE: The MM3000 sometimes responds to an MS command with an error
 *		message (see MM3000 User's Manual Appendix A).  This is an
 *		unconfirmed MM3000 bug.  Retry read if this is a Hard Travel
 *		limit switch error. This effectively flushes the error message.
 *
 *	    IF input "com" buffer length is > 3 characters, AND, the 1st
 *			character is an "E" (Maybe this an unsolicited error
 *			message response?).
 *	   	Call pasynSyncIO->read().
 *	    ENDIF
 *	    BREAK
 *    ENDSWITCH
 *		
 *  NORMAL RETURN.
 */

STATIC int recv_mess(int card, char *com, int flag)
{
    struct MMcontroller *cntrl;
    double timeout = 0.;
    int flush = 1;
    int len = 0;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    if (flag != FLUSH) {
        flush = 0;
	timeout	= SERIAL_TIMEOUT;
    }
    len = pasynSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE, (char *) 
                            "\n", 1, flush, timeout);
    if (len > 3 && com[0] == 'E')
    {
	long error;

	error = strtol(&com[1], NULL, 0);
	if (error >= 35 && error <= 42)
	    len = pasynSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE, 
                                    (char *) "\n", 1, flush, timeout);
    }

    if (len <= 0)
    {
	com[0] = '\0';
	len = 0;
    }
    else
    {
	/* MM3000 responses are always terminated with CR/LF combination (see
	 * MM3000 User' Manual Sec. 3.4 NOTE). Strip both CR&LF from buffer
	 * before returning to caller.
	 */
	com[len-2] = '\0';
	/* Test for "system error" response. */
	if (com[0] == 'E')
	{
	    errPrintf( -1, __FILE__, __LINE__, "%s\n", com);
	    return(recv_mess(card, com, flag));
	}
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (len);
}


/*****************************************************/
/* Setup system configuration                        */
/* MM3000Setup()                                     */
/*****************************************************/
RTN_STATUS
MM3000Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > MM3000_NUM_CARDS)
	MM3000_num_cards = MM3000_NUM_CARDS;
    else
	MM3000_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

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

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* MM3000Config()                                    */
/*****************************************************/
RTN_STATUS
MM3000Config(int card,		/* card being configured */
            const char *port,   /* asyn port name */
            int address)        /* asyn address (GPIB) */
{
    struct MMcontroller *cntrl;

    if (card < 0 || card >= MM3000_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MMcontroller));
    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, port);
    cntrl->asyn_address = address;
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
    struct MMcontroller *cntrl;
    int card_index, motor_index;
    char axis_pos[BUFF_SIZE];
    char buff[BUFF_SIZE];
    char *tok_save, *bufptr;
    int total_axis = 0;
    int status;
    bool success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */
    
    /* Check for setup */
    if (MM3000_num_cards <= 0)
	return(ERROR);

    tok_save = NULL;

    for (card_index = 0; card_index < MM3000_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct MMcontroller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynSyncIO->connect(cntrl->asyn_port, 
                          cntrl->asyn_address, &cntrl->pasynUser);
	if (success_rtn == true)
	{
	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    pasynSyncIO->flush(cntrl->pasynUser);
    
	    send_mess(card_index, GET_IDENT, (char) NULL);
	    status = recv_mess(card_index, axis_pos, 1);  
	    /* Return value is length of response string */
	}

	if (success_rtn == true && status > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    send_mess(card_index, STOP_ALL, (char) NULL);	/* Stop all motors */
	    send_mess(card_index, GET_IDENT, (char) NULL);	/* Read controller ID string */
	    recv_mess(card_index, buff, 1);
	    strncpy(brdptr->ident, &buff[0], 50);  /* Skip "VE" */

	    send_mess(card_index, "RC", (char) NULL);
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
			errlogPrintf("drvMM3000:motor_init() - invalid RC response = %s\n",
			       bufptr);

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

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;

		if (cntrl->type[total_axis] == DC)
		    motor_info->encoder_present = YES;
		else
		{
		    sprintf(buff, "%dTPE", motor_index + 1);
		    send_mess(card_index, buff, (char) NULL);
		    recv_mess(card_index, buff, 1);

		    if (strcmp(buff, "E01") == 0)
			motor_info->encoder_present = NO;
		    else
			motor_info->encoder_present = YES;
		}
                
		if (motor_info->encoder_present == YES)
		{
		    motor_info->status.Bits.EA_PRESENT = 1;
		    motor_info->pid_present = YES;
		    motor_info->status.Bits.GAIN_SUPPORT = 1;
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

    epicsThreadCreate((char *) "MM3000_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

/*---------------------------------------------------------------------*/
