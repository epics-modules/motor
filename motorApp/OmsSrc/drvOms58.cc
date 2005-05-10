/*
FILENAME...	drvOms58.cc
USAGE...	Motor record driver level support for OMS model VME58.

Version:	$Revision: 1.16 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2005-05-10 18:30:26 $
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
 * NOTES
 * -----
 * Verified with firmware:
 *	- VME58 ver 2.16-8
 *	- VME58 ver 2.24-8
 *	- VME58 ver 2.24-8S
 *	- VME58 ver 2.24-4E
 *	- VME58 ver 2.35-8
 *
 * Modification Log:
 * -----------------
 * .01  01-18-93 jbk     initialized
 * .02  11-14-94 jps     copy drvOms.c and modify to point to vme58 driver
 *      ...
 * .14  02-10-95 jps	first released version w/interrupt supprt
 * .15  02-16-95 jps	add better simulation mode support
 * .16  03-10-95 jps	limit switch handling improvements
 * .17  04-11-95 jps  	misc. fixes
 * .18  09-27-95 jps	fix GET_INFO latency bug, add axis argument to
 *			set_status() create add debug verbosity of 3
 * .19  05-03-96 jps     convert 32bit card accesses to 16bit - vme58PCB
 *			version D does not support 32bit accesses.
 * .20  06-20-96 jps     add more security to message parameters (card #)
 * .20a 02-19-97 tmm	fixed for EPICS 3.13
 * .21  01-20-00 rls	Update bit collision check and wait.  Problem
 *			with old position data returned after Done
 *			detected when moving multiple axes on same
 *			controller board.  Fix: Update shared memory
 *			data after Done detected via ASCII controller
 *			commands.  To avoid collisions, skip writing to
 *			control reg. from motorIsr() if update bit is
 *			true.
 * .22  02-22-02 rls	- start_status() was not checking for a valid
 *			motor_state[card] pointer. This causes a bus error if
 *			the controller array has holes in it.
 *			- "total_cards" changed from total detected to total
 *			cards that "memory is allocated for".  This allows
 *			boards after the "hole" to work.
 * .23  06-04-03  rls   Convert to R3.14.x.
 * .24  06-04-03  rls   extended device directive support to PREM and POST.
 * .25  12-12-03  rls - Converted MSTA #define's to bit field.
 *		      - One line of code must be selected based on either
 *			Tornado 2.0.2 (default) or Tornado 2.2.  If Tornado
 *			2.2 is selected, EPICS base patches must be applied as
 *			described in;
 *     http://www.aps.anl.gov/upd/people/sluiter/epics/motor/R5-2/Problems.html
 * .26  02-03-04  rls - Eliminate erroneous "Motor motion timeout ERROR".
 * .27  07-16-04  rls - removed unused <driver>Setup() argument.
 * .28  09-20-04  rls - support for 32axes/controller.
 * .29  12-06-04  rls - Windows compiler support.
 *                    - eliminate calls to devConnectInterrupt() due to C++
 *			problems with devLib.h; i.e. "sorry, not implemented:
 *			`tree_list' not supported..." compiler error message.
 * .30  03-23-05 rls - Make OSI.
 * .31  05-02-05 rls - Bug fix for stale data delay; set delay = 10ms.
 */

#include	<string.h>
#include	<dbCommon.h>
#include	<drvSup.h>
#include	<epicsVersion.h>
#include	<devLib.h>
#include	<dbAccess.h>
#include	<epicsThread.h>
#include	<epicsExit.h>

#include	"motorRecord.h"	/* For Driver Power Monitor feature only. */
#include	"motor.h"
#include	"motordevCom.h"	/* For Driver Power Monitor feature only. */
#include	"drvOms58.h"

#include	"epicsExport.h"

#define PRIVATE_FUNCTIONS 1	/* normal:1, debug:0 */

/* Define for return test on devNoResponseProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

/* jps: INFO messages - add RV and move QA to top */
#define	ALL_INFO	"QA EA"
#define	AXIS_INFO	"QA"
#define	ENCODER_QUERY	"EA ID"
#define	AXIS_CLEAR	"CA"		/* Clear done of addressed axis */
#define	DONE_QUERY	"RA"		/* ?? Is this needed?? */
#define	PID_QUERY	"KK2 ID"


/*----------------debugging-----------------*/
#ifdef __GNUG__
  #ifdef	DEBUG
    #define Debug(l, f, args...) {if (l <= drvOms58debug) \
				    errlogPrintf(f, ## args);}
  #else
    #define Debug(l, f, args...)
  #endif
#else
  #define Debug
#endif
volatile int drvOms58debug = 0;
extern "C" {epicsExportAddress(int, drvOms58debug);}

#define pack2x16(p)      ((epicsUInt32)(((p[0])<<16)|(p[1])))

/* Global data. */
int oms58_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* --- Local data common to all OMS drivers. --- */
static char *oms_addrs = 0x0;
static volatile unsigned omsInterruptVector = 0;
static volatile epicsUInt8 omsInterruptLevel = OMS_INT_LEVEL;
static volatile int max_io_tries = MAX_COUNT;
static volatile int motionTO = 10;
static char *oms58_axis[] = {"X", "Y", "Z", "T", "U", "V", "R", "S"};
static double quantum;

/*----------------functions-----------------*/

/* Common local function declarations. */
static long report(int);
static long init();
static void query_done(int, int, struct mess_node *);
static int set_status(int, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int recv_mess(int, char *, int);
static void motorIsr(int card);
static int motor_init();
static void oms_reset(void *);

static void start_status(int card);
static int motorIsrSetup(int card);

struct driver_table oms58_access =
{
    NULL,
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
    start_status,
    &initialized,
    oms58_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvOms58 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvOms58);}

static struct thread_args targs = {SCAN_RATE, &oms58_access, 0.010};

/*----------------functions-----------------*/

static long report(int level)
{
    int card;

    if (oms58_num_cards <= 0)
	printf("    No VME58 controllers configured.\n");
    else
    {
	for (card = 0; card < oms58_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    Oms Vme58 motor card #%d not found.\n", card);
	    else
		printf("    Oms Vme58 motor card #%d @ 0x%X, id: %s \n", card,
		       (epicsUInt32) brdptr->localaddr,
		       brdptr->ident);
	}
    }
    return (0);
}

static long init()
{
    initialized = true;	/* Indicate that driver is initialized. */
    (void) motor_init();
    return ((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
    char buffer[40];

    send_mess(card, DONE_QUERY, oms58_axis[axis]);
    recv_mess(card, buffer, 1);

#if (CPU == PPC604 || CPU == PPC603)    
    if (strcmp(motor_state[card]->ident, "VME58 ver 2.35-8") == 0)
	epicsThreadSleep(quantum * 2.0); /* Work around for intermittent wrong LS status. */
#endif

    if (nodeptr->status.Bits.RA_PROBLEM)
	send_mess(card, AXIS_STOP, oms58_axis[axis]);
}


/*********************************************************
 * Start card data area update.
 * Each card will take 800-900us to complete update.
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/
static void start_status(int card)
{
    volatile struct vmex_motor *pmotor;
    CNTRL_REG cntrlReg;
    int index;

    if (card >= 0)
    {
	if (motor_state[card] != NULL &&
	    (pmotor = (struct vmex_motor *) motor_state[card]->localaddr) != NULL)
	{
	    /* Wait Forever for controller to update. */
	    cntrlReg.All = pmotor->control.cntrlReg;
	    while(cntrlReg.Bits.update != 0)
	    {
		Debug(1, "start_status(): Update Wait: card #%d\n", card);
		epicsThreadSleep(quantum);
		cntrlReg.All = pmotor->control.cntrlReg;
	    };

	    cntrlReg.Bits.update = 1;
	    pmotor->control.cntrlReg = cntrlReg.All;
	}
    }
    else
    {
	for (index = 0; index < total_cards; index++)
	{
	    if (motor_state[index] != NULL &&
		(pmotor = (struct vmex_motor *) motor_state[index]->localaddr) != NULL)
	    {
		/* Wait Forever for controller to update. */
		cntrlReg.All = pmotor->control.cntrlReg;
		while(cntrlReg.Bits.update != 0)
		{
		    Debug(1, "start_status(): Update Wait: card #%d\n", index);
		    epicsThreadSleep(quantum);
		    cntrlReg.All = pmotor->control.cntrlReg;
		};
    
		cntrlReg.Bits.update = 1;
		pmotor->control.cntrlReg = cntrlReg.All;
	    }
	}
    }
}

/******************************************************************************
* FUNCTION NAME: set_status
*
* ARGUMETS     Type      I/O     Description
* --------     ----      ---     -----------
*
* card         int        I      Controller card index #.
* signal       int        I      Motor index.
*
* LOGIC
*   Initialize.
*   IF encoder present.
*	Get axis and encoder information.
*   ELSE
*       Get axis information.
*   ENDIF
*
*   Process controller response strings.
*
*   ...
*   ...
*   IF "motor-in-motion" (i.e., nodeptr != 0), AND, no limit switch error.
*	IF drive power monitoring enabled.
*	    ...
*	ENDIF
*   ENDIF
*
*   IF no motion, OR, status indicates limit switch error, OR, motor done, OR,
*		controller problem.
*	Set return state to "callback record".
*   ELSE
*	Set return state to skip "record callback".
*   ENDIF
*   IF status indicates DONE/LIMIT, AND, "motor-in-motion" (nodeptr != 0), AND,
*		post-move message is not null.
*	Send post-move message to controller.
*	Clear post-move message pointer.
*   ENDIF
*   EXIT with return state indicator.
******************************************************************************/

static int set_status(int card, int signal)
{
    struct mess_info *motor_info;
    struct mess_node *nodeptr;
    volatile struct vmex_motor *pmotor;
    CNTRL_REG cntrlReg;
    volatile MOTOR_DATA_REGS *pmotorData;
    epicsInt32 motorData;
    /* Message parsing variables */
    char *p, *tok_save;
    struct axis_status *ax_stat;
    struct encoder_status *en_stat;
    char q_buf[50], outbuf[50];
    int index;
    bool ls_active = false;
    bool got_encoder;
    msta_field status;

    int rtn_state;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    pmotor = (struct vmex_motor *) motor_state[card]->localaddr;
    status.All = motor_info->status.All;

    if (motor_info->encoder_present == YES)
    {
	/* get 4 peices of info from axis */
	send_mess(card, ALL_INFO, oms58_axis[signal]);
	recv_mess(card, q_buf, 2);
	got_encoder = true;
    }
    else
    {
	/* get 2 peices of info from axis */
	send_mess(card, AXIS_INFO, oms58_axis[signal]);
	recv_mess(card, q_buf, 1);
	got_encoder = false;
    }

    for (index = 0, p = strtok_r(q_buf, ",", &tok_save); p;
	 p = strtok_r(NULL, ",", &tok_save), index++)
    {
	switch (index)
	{
	case 0:		/* axis status */
	    ax_stat = (struct axis_status *) p;

	    status.Bits.RA_DIRECTION = (ax_stat->direction == 'P') ? 1 : 0;
	    status.Bits.RA_HOME =      (ax_stat->home == 'H')      ? 1 : 0;

	    if (ax_stat->done == 'D')
	    {
		/* Request Data Area Update after DONE detected so that both
		 * ASCII command data and shared memory data match. */
		start_status(card);
		status.Bits.RA_DONE = 1;
	    }
	    else
		status.Bits.RA_DONE = 0;

	    if (ax_stat->overtravel == 'L')
	    {
		ls_active = true;
		if (status.Bits.RA_DIRECTION)
		    status.Bits.RA_PLUS_LS = 1;
		else
		    status.Bits.RA_MINUS_LS = 1;
	    }
	    else
	    {
		ls_active = false;
		status.Bits.RA_PLUS_LS  = 0;
		status.Bits.RA_MINUS_LS = 0;
	    }
	    break;
	case 1:		/* encoder status */
	    en_stat = (struct encoder_status *) p;

	    status.Bits.EA_SLIP       = (en_stat->slip_enable == 'E') ? 1 : 0;
	    status.Bits.EA_POSITION   = (en_stat->pos_enable  == 'E') ? 1 : 0;
	    status.Bits.EA_SLIP_STALL = (en_stat->slip_detect == 'S') ? 1 : 0;
	    status.Bits.EA_HOME       = (en_stat->axis_home   == 'H') ? 1 : 0;
	    break;
	default:
	    break;
	}
    }


    /* Setup mask for data-ready detection */
    cntrlReg.All = 0;
    cntrlReg.Bits.update = 1;

    /* Wait for data area update to complete on card */
    while (cntrlReg.All & pmotor->control.cntrlReg)
    {
	Debug(5, "set_status: DPRAM delay.\n");
	epicsThreadSleep(quantum);
    }

    pmotorData = &pmotor->data[signal];

    /* Get motor position - word access only */
    motorData = pack2x16(pmotorData->cmndPos);

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->position = motorData;
	motor_info->no_motion_count = 0;
    }

    if (motor_info->no_motion_count > motionTO)
    {
	status.Bits.RA_PROBLEM = 1;
	send_mess(card, AXIS_STOP, oms58_axis[signal]);
	motor_info->no_motion_count = 0;
	errlogSevPrintf(errlogMinor, "Motor motion timeout ERROR on card: %d, signal: %d\n",
	    card, signal);
    }
    else
	status.Bits.RA_PROBLEM = 0;

    /* get command velocity - word access only */
    motorData = pack2x16(pmotorData->cmndVel);

    motor_info->velocity = motorData;

    if (!(status.Bits.RA_DIRECTION))
	motor_info->velocity *= -1;

    /* Get encoder position */
    motorData = pack2x16(pmotorData->encPos);

    motor_info->encoder_position = motorData;

    if (nodeptr != NULL && ls_active == false)
    {
	struct motor_trans *trans = (struct motor_trans *) nodeptr->mrecord->dpvt;
	if (trans->dpm == true)
	{
	    unsigned char bitselect;
	    unsigned char inputs = pmotor->control.ioLowReg;
	    bitselect = (1 << signal);
	    if ((inputs & bitselect) == 0)
	    {
		status.Bits.RA_PLUS_LS  = 1;	/* Turn on both limit switches. */
		status.Bits.RA_MINUS_LS = 1;
		errlogPrintf("Drive power failure at VME58 card#%d motor#%d\n",
		       card, signal);
	    }
	}
    }

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		status.Bits.RA_DONE || status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
	char buffer[40];

	/* Test for a "device directive" in the POST string. */
	if (nodeptr->postmsgptr[0] == '@')
	{
	    bool errind = false;
	    char *end = strchr(&nodeptr->postmsgptr[1], '@');
	    if (end == NULL)
		errind = true;
	    else
	    {
		DBADDR addr;
		char *start, *tail;
		int size = (end - &nodeptr->postmsgptr[0]) + 1;

		/* Copy device directive to buffer. */
		strncpy(buffer, nodeptr->postmsgptr, size);
		buffer[size] = (char) NULL;

		if (strncmp(buffer, "@PUT(", 5) != 0)
		    goto errorexit;
		
		/* Point "start" to PV name argument. */
		tail = NULL;
		start = strtok_r(&buffer[5], ",", &tail);
		if (tail == NULL)
		    goto errorexit;

		if (dbNameToAddr(start, &addr))	/* Get address of PV. */
		{
		    errPrintf(-1, __FILE__, __LINE__, "Invalid PV name: %s", start);
		    goto errorexit;
		}

		/* Point "start" to PV value argument. */
		start = strtok_r(NULL, ")", &tail);
		if (dbPutField(&addr, DBR_STRING, start, 1L))
		{
		    errPrintf(-1, __FILE__, __LINE__, "invalid value: %s", start);
		    goto errorexit;
		}
	    }

	    if (errind == true)
errorexit:	errMessage(-1, "Invalid device directive");
	    end++;
	    strcpy(buffer, end);
	}
	else
	    strcpy(buffer, nodeptr->postmsgptr);

	strcpy(outbuf, buffer);
	send_mess(card, outbuf, oms58_axis[signal]);
	nodeptr->postmsgptr = NULL;
    }

    /* Bug fix for DC servo moving away from limit switch, but move is not far enough to
     * get off limit switch; resulting in limit error.  Fix is to force CDIR to match
     * MSTA.RA_DIRECTION.
     */
    if (ls_active == true && status.Bits.GAIN_SUPPORT &&
	status.Bits.EA_POSITION == 0 && nodeptr != 0)
    {
	struct motorRecord *mr = (struct motorRecord *) nodeptr->mrecord;

	if (mr->cdir != (short) status.Bits.RA_DIRECTION)
	    mr->cdir = status.Bits.RA_DIRECTION;
    }

    motor_info->status.All = status.All;	/* Update status from local copy. */
    return (rtn_state);
}


/*****************************************************/
/* send a message to the OMS board		     */
/* send_mess()			     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    volatile struct vmex_motor *pmotor;
    epicsInt16 putIndex;
    char outbuf[MAX_MSG_SIZE], *p;
    RTN_STATUS return_code;

    if (strlen(com) > MAX_MSG_SIZE)
    {
	errlogPrintf("drvOms58.cc:send_mess(); message size violation.\n");
	return (ERROR);
    }

    /* Check that card exists */
    if (!motor_state[card])
    {
	errlogPrintf("drvOms58.cc:send_mess() - invalid card #%d\n", card);
	return (ERROR);
    }

    pmotor = (struct vmex_motor *) motor_state[card]->localaddr;
    Debug(9, "send_mess: pmotor = %x\n", (epicsUInt32) pmotor);

    return_code = OK;

    Debug(9, "send_mess: checking card %d status\n", card);

/* jps: Command error handling has been moved to motorIsr() */
/* Debug(1, "Command error detected! 0x%02x on send_mess\n", status.All); */

    /* see if junk at input port - should not be any data available */
    if (pmotor->inGetIndex != pmotor->inPutIndex)
    {
	Debug(1, "send_mess - clearing data in buffer\n");
	recv_mess(card, NULL, -1);
    }


    if (name == NULL)
	strcpy(outbuf, com);
    else
    {
	strcpy(outbuf, "A");
	strcat(outbuf, name);
	strcat(outbuf, " ");
	strcat(outbuf, com);
    }
    strcat(outbuf, "\n");		/* Add the command line terminator. */

    Debug(9, "send_mess: ready to send message.\n");
    putIndex = pmotor->outPutIndex;
    for (p = outbuf; *p != '\0'; p++)
    {
	pmotor->outBuffer[putIndex++] = *p;
	if (putIndex >= BUFFER_SIZE)
	    putIndex = 0;
    }

    Debug(4, "send_mess: sent card %d message:", card);
    Debug(4, "%s\n", outbuf);

    pmotor->outPutIndex = putIndex;	/* Message Sent */

    while (pmotor->outPutIndex != pmotor->outGetIndex)
    {
#ifdef	DEBUG
	epicsInt16 deltaIndex, delta;

	deltaIndex = pmotor->outPutIndex - pmotor->outGetIndex;
	delta = (deltaIndex < 0) ? BUFFER_SIZE + deltaIndex : deltaIndex;
	Debug(5, "send_mess(): buffer wait; index delta=%d\n", delta);
#endif
	epicsThreadSleep(quantum);
    };

    return (return_code);
}

/*
 * FUNCTION... recv_mess(int card, char *com, int amount)
 *
 * INPUT ARGUMENTS...
 *	card - 
 *	*com -
 *	amount - 
 *
 * LOGIC...
 *  IF controller card does not exist.
 *	ERROR Exit.
 *  ENDIF
 *  IF "amount" indicates buffer flush.
 *	WHILE characters left in input buffer.
 *	    Remove characters from controller's input buffer.
 *	ENDWHILE
 *	NORMAL RETURN.
 *  ENDIF
 *
 *  FOR each message requested (i.e. "amount").
 *	Initialize head and tail pointers.
 *	Initialize local buffer "get" index.
 *	FOR
 *	    IF characters left in controller's input buffer.
 *		
 *	    ENDIF
 *	ENDFOR
 *  ENDFOR
 *  
 */
static int recv_mess(int card, char *com, int amount)
{
    volatile struct vmex_motor *pmotor;
    epicsInt16 getIndex;
    int i, trys;
    char junk;
    unsigned char inchar;
    int piece, head_size, tail_size;

    /* Check that card exists */
    if (!motor_state[card])
    {
	Debug(1, "resv_mess - invalid card #%d\n", card);
	return (-1);
    }

    pmotor = (struct vmex_motor *) motor_state[card]->localaddr;

    if (amount == -1)
    {
	Debug(7, "-------------");
	getIndex = pmotor->inGetIndex;
	for (i = 0, trys = 0; trys < max_io_tries; trys++)
	{
	    while (getIndex != pmotor->inPutIndex)
	    {
		junk = pmotor->inBuffer[getIndex++];
		/* handle circular buffer */
		if (getIndex >= BUFFER_SIZE)
		    getIndex = 0;

		Debug(7, "%c", junk);
		trys = 0;
		i++;
	    }
	}
	pmotor->inGetIndex = getIndex;

	Debug(7, "-------------");
	Debug(1, "\nrecv_mess - cleared %d error data\n", i);
	return (0);
    }

    for (i = 0; amount > 0; amount--)
    {
	Debug(7, "-------------");
	head_size = 0;
	tail_size = 0;

	getIndex = pmotor->inGetIndex;

	for (piece = 0, trys = 0; piece < 3 && trys < max_io_tries; trys++)
	{
	    if (getIndex != pmotor->inPutIndex)
	    {
		inchar = pmotor->inBuffer[getIndex++];
		if (getIndex >= BUFFER_SIZE)
		    getIndex = 0;

		Debug(7, "%02x", inchar);

		switch (piece)
		{
		case 0:	/* header */
		    if (inchar == '\n' || inchar == '\r')
			head_size++;
		    else
		    {
			piece++;
			com[i++] = inchar;
		    }
		    break;
		case 1:	/* body */
		    if (inchar == '\n' || inchar == '\r')
		    {
			piece++;
			tail_size++;
		    }
		    else
			com[i++] = inchar;
		    break;

		case 2:	/* trailer */
		    tail_size++;
		    if (tail_size >= head_size)
			piece++;
		    break;
		}
		trys = 0;
	    }
	}
	pmotor->inGetIndex = getIndex;

	Debug(7, "-------------\n");
	if (trys >= max_io_tries)
	{
	    Debug(1, "Timeout occurred in recv_mess\n");
	    com[i] = '\0';

	    /* ----------------extra junk-------------- */
	    /* jps: command error now cleared by motorIsr() */
	    /* ----------------extra junk-------------- */

	    return (-1);
	}
	com[i++] = ',';
    }

    if (i > 0)
	com[i - 1] = '\0';
    else
	com[i] = '\0';

    Debug(4, "recv_mess: card %d", card);
    Debug(4, " com %s\n", com);
    return (0);
}


/*****************************************************/
/* Configuration function for  module_types data     */
/* areas. omsSetup()                                */
/*****************************************************/
int oms58Setup(int num_cards,	/* maximum number of cards in rack */
	       void *addrs,	/* Base Address(0x0-0xb000 on 4K boundary) */
	       unsigned vector,	/* noninterrupting(0), valid vectors(64-255) */
	       int int_level,	/* interrupt level (1-6) */
	       int scan_rate)	/* polling rate - 1/60 sec units */
{
    if (num_cards < 1 || num_cards > OMS_NUM_CARDS)
	oms58_num_cards = OMS_NUM_CARDS;
    else
	oms58_num_cards = num_cards;

    /* Check range and boundary(4K) on base address */
    if (addrs > (void *) 0xF000 || ((epicsUInt32) addrs & 0xFFF))
    {
	Debug(1, "omsSetup: invalid base address 0x%X\n", (epicsUInt32) addrs);
	oms_addrs = (char *) OMS_NUM_ADDRS;
    }
    else
	oms_addrs = (char *) addrs;

    omsInterruptVector = vector;
    if (vector < 64 || vector > 255)
    {
	if (vector != 0)
	{
	    Debug(1, "omsSetup: invalid interrupt vector %d\n", vector);
	    omsInterruptVector = (unsigned) OMS_INT_VECTOR;
	}
    }

    if (int_level < 1 || int_level > 6)
    {
	Debug(1, "omsSetup: invalid interrupt level %d\n", int_level);
	omsInterruptLevel = OMS_INT_LEVEL;
    }
    else
	omsInterruptLevel = int_level;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;
    return(0);
}


/*****************************************************/
/* Interrupt service routine.                        */
/* motorIsr()		                     */
/*****************************************************/
static void motorIsr(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    STATUS_REG statusBuf;
    epicsUInt8 doneFlags, userIO, slipFlags, limitFlags, cntrlReg;

    if (card >= total_cards || (pmotorState = motor_state[card]) == NULL)
    {
	errlogPrintf("Invalid entry-card #%d\n", card);
	return;
    }

    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    /* Status register - clear irqs on read. */
    statusBuf.All = pmotor->control.statusReg;

    /* Done register - clears on read */
    doneFlags = pmotor->control.doneReg;

    /* UserIO register - clears on read */
    userIO = pmotor->control.ioLowReg;

/* Questioniable Fix for undefined problem Starts Here. The following
 * code is meant to fix a problem that exhibted "spurious" interrupts
 * and "stuck in the Oms ISR" behavior.
*/
    /* Slip detection  - clear on read */
    slipFlags = pmotor->control.slipReg;

    /* Overtravel - clear on read */
    limitFlags = pmotor->control.limitReg;

    /* Only write control register if update bit is false. */
    /* Assure proper control register settings  */
    cntrlReg = pmotor->control.cntrlReg;
    if ((cntrlReg & 0x01) == 0)
	pmotor->control.cntrlReg = (epicsUInt8) 0x90;
/* Questioniable Fix for undefined problem Ends Here. */

#ifdef	DEBUG
    if (drvOms58debug >= 10)
	errlogPrintf("entry card #%d,status=0x%X,done=0x%X\n", card,
		     statusBuf.All, doneFlags);
#endif

    /* Motion done handling */
    if (statusBuf.Bits.done)
	/* Wake up polling task 'motor_task()' to issue callbacks */
	motor_sem.signal();

    if (statusBuf.Bits.cmndError)
	errlogPrintf("command error detected by motorISR() on card %d\n", card);
}

static int motorIsrSetup(int card)
{
#ifdef vxWorks
    volatile struct vmex_motor *pmotor;
    long status;
    CNTRL_REG cntrlBuf;

    Debug(5, "motorIsrSetup: Entry card#%d\n", card);

    pmotor = (struct vmex_motor *) (motor_state[card]->localaddr);

    status = pdevLibVirtualOS->pDevConnectInterruptVME(
	omsInterruptVector + card, (void (*)()) motorIsr, (void *) card);

    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__, "Can't connect to vector %d\n", omsInterruptVector + card);
	omsInterruptVector = 0;	/* Disable interrupts */
	cntrlBuf.All = 0;
	pmotor->control.cntrlReg = cntrlBuf.All;
	return (ERROR);
    }

    status = devEnableInterruptLevel(OMS_INTERRUPT_TYPE,
				     omsInterruptLevel);
    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__, "Can't enable enterrupt level %d\n", omsInterruptLevel);
	omsInterruptVector = 0;	/* Disable interrupts */
	cntrlBuf.All = 0;
	pmotor->control.cntrlReg = cntrlBuf.All;
	return (ERROR);
    }

    /* Setup card for interrupt-on-done */
    pmotor->control.intVector = omsInterruptVector + card;

    /* enable interrupt-when-done irq */
    cntrlBuf.All = 0;
    cntrlBuf.Bits.doneIntEna = 1;
    cntrlBuf.Bits.intReqEna = 1;

    pmotor->control.cntrlReg = cntrlBuf.All;
#endif
    return (OK);
}

/*****************************************************/
/* initialize all software and hardware		     */
/* motor_init()			     */
/*****************************************************/
static int motor_init()
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    STATUS_REG statusReg;
    int8_t omsReg;
    long status;
    int card_index, motor_index;
    char axis_pos[50], encoder_pos[50];
    char *tok_save, *pos_ptr;
    int total_encoders = 0, total_axis = 0, total_pidcnt = 0;
    volatile void *localaddr;
    void *probeAddr;

    tok_save = NULL;
    quantum = epicsThreadSleepQuantum();

    /* Check for setup */
    if (oms58_num_cards <= 0)
    {
	Debug(1, "motor_init: *OMS58 driver disabled* \n oms58Setup() is missing from startup script.\n");
	return (ERROR);
    }

    /* allocate space for total number of motors */
    motor_state = (struct controller **) malloc(oms58_num_cards *
						sizeof(struct controller *));

    /* allocate structure space for each motor present */

    total_cards = oms58_num_cards;

    if (epicsAtExit(oms_reset, NULL) == ERROR)
	Debug(1, "vme58 motor_init: oms_reset disabled\n");

    for (card_index = 0; card_index < oms58_num_cards; card_index++)
    {
	int8_t *startAddr;
	int8_t *endAddr;

	Debug(2, "motor_init: card %d\n", card_index);

	probeAddr = oms_addrs + (card_index * OMS_BRD_SIZE);
	startAddr = (int8_t *) probeAddr;
	endAddr = startAddr + OMS_BRD_SIZE;

	Debug(9, "motor_init: devNoResponseProbe() on addr 0x%x\n", (epicsUInt32) probeAddr);
	/* Scan memory space to assure card id */
#ifdef vxWorks
	do
	{
	    status = devNoResponseProbe(OMS_ADDRS_TYPE, (unsigned int) startAddr, 2);
	    startAddr += 0x100;
	} while (PROBE_SUCCESS(status) && startAddr < endAddr);
#endif
	if (PROBE_SUCCESS(status))
	{
#ifdef vxWorks
	    status = devRegisterAddress(__FILE__, OMS_ADDRS_TYPE,
					(size_t) probeAddr, OMS_BRD_SIZE,
					(volatile void **) &localaddr);
	    Debug(9, "motor_init: devRegisterAddress() status = %d\n", (int) status);
	    if (!RTN_SUCCESS(status))
	    {
		errPrintf(status, __FILE__, __LINE__, "Can't register address 0x%x\n",
			  (unsigned int) probeAddr);
		return (ERROR);
	    }
#endif
	    Debug(9, "motor_init: localaddr = %x\n", (epicsUInt32) localaddr);
	    pmotor = (struct vmex_motor *) localaddr;

	    Debug(9, "motor_init: malloc'ing motor_state\n");
	    motor_state[card_index] = (struct controller *) malloc(sizeof(struct controller));
	    pmotorState = motor_state[card_index];
	    pmotorState->localaddr = (char *) localaddr;
	    pmotorState->motor_in_motion = 0;
	    pmotorState->cmnd_response = false;

	    /* Disable all interrupts */
	    pmotor->control.cntrlReg = 0;

	    send_mess(card_index, "EF", (char) NULL);
	    send_mess(card_index, ERROR_CLEAR, (char) NULL);
	    send_mess(card_index, STOP_ALL, (char) NULL);

	    send_mess(card_index, GET_IDENT, (char) NULL);
	    recv_mess(card_index, (char *) pmotorState->ident, 1);
	    Debug(3, "Identification = %s\n", pmotorState->ident);

	    send_mess(card_index, ALL_POS, (char) NULL);
	    recv_mess(card_index, axis_pos, 1);

	    for (total_axis = 0, pos_ptr = strtok_r(axis_pos, ",", &tok_save);
		 pos_ptr; pos_ptr = strtok_r(NULL, ",", &tok_save), total_axis++)
	    {
		pmotorState->motor_info[total_axis].motor_motion = NULL;
		pmotorState->motor_info[total_axis].status.All = 0;
	    }

	    Debug(3, "motor_init: Total axis = %d\n", total_axis);
	    pmotorState->total_axis = total_axis;

	    /* Assure done is cleared */
	    statusReg.All = pmotor->control.statusReg;
	    omsReg = pmotor->control.doneReg;
	    for (total_encoders = total_pidcnt = 0, motor_index = 0; motor_index < total_axis; motor_index++)
	    {
	    	/* Test if motor has an encoder. */
		send_mess(card_index, ENCODER_QUERY, oms58_axis[motor_index]);
		while (!pmotor->control.doneReg)	/* Wait for command to complete. */
		    epicsThreadSleep(quantum * 2.0);

		statusReg.All = pmotor->control.statusReg;

		if (statusReg.Bits.cmndError)
		{
		    Debug(2, "motor_init: No encoder on axis %d\n", motor_index);
		    pmotorState->motor_info[motor_index].encoder_present = NO;
		}
		else
		{
		    total_encoders++;
		    pmotorState->motor_info[motor_index].encoder_present = YES;
		    recv_mess(card_index, encoder_pos, 1);
		}
		
		/* Test if motor has PID parameters. */
		send_mess(card_index, PID_QUERY, oms58_axis[motor_index]);
		do	/* Wait for command to complete. */
		{
		    epicsThreadSleep(quantum);
		    statusReg.All = pmotor->control.statusReg;
		} while(statusReg.Bits.done == 0);


		if (statusReg.Bits.cmndError)
		{
		    Debug(2, "motor_init: No PID parameters on axis %d\n", motor_index);
		    pmotorState->motor_info[motor_index].pid_present = NO;
		}
		else
		{
		    total_pidcnt++;
		    pmotorState->motor_info[motor_index].pid_present = YES;
		}
	    }

	    /* Enable interrupt-when-done if selected */
	    if (omsInterruptVector)
	    {
		if (motorIsrSetup(card_index) == ERROR)
		    errPrintf(0, __FILE__, __LINE__, "Interrupts Disabled!\n");
	    }

	    start_status(card_index);
	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		pmotorState->motor_info[motor_index].status.All = 0;
		pmotorState->motor_info[motor_index].no_motion_count = 0;
		pmotorState->motor_info[motor_index].encoder_position = 0;
		pmotorState->motor_info[motor_index].position = 0;

		if (pmotorState->motor_info[motor_index].encoder_present == YES)
		    pmotorState->motor_info[motor_index].status.Bits.EA_PRESENT = 1;
		if (pmotorState->motor_info[motor_index].pid_present == YES)
		    pmotorState->motor_info[motor_index].status.Bits.GAIN_SUPPORT = 1;

		set_status(card_index, motor_index);

		send_mess(card_index, DONE_QUERY, oms58_axis[motor_index]); /* Is this needed??? */
		recv_mess(card_index, axis_pos, 1);
	    }

	    Debug(2, "motor_init: Init Address=0x%8.8x\n", (epicsUInt32) localaddr);
	    Debug(3, "motor_init: Total encoders = %d\n", total_encoders);
	    Debug(3, "motor_init: Total with PID = %d\n", total_pidcnt);
	}
	else
	{
	    Debug(3, "motor_init: Card NOT found!\n");
	    motor_state[card_index] = (struct controller *) NULL;
	}
    }

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    Debug(3, "Motors initialized\n");

    epicsThreadCreate((char *) "Oms58_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    Debug(3, "Started motor_task\n");
    return (0);
}


/* Disables interrupts. Called on CTL X reboot. */

static void oms_reset(void *arg)
{
    short card;
    volatile struct vmex_motor *pmotor;
    CNTRL_REG cntrlBuf;

    for (card = 0; card < total_cards; card++)
    {
	if (motor_state[card] != NULL)
	{
	    pmotor = (struct vmex_motor *) motor_state[card]->localaddr;
	    cntrlBuf.All = pmotor->control.cntrlReg;
	    cntrlBuf.Bits.intReqEna = 0;
	    pmotor->control.cntrlReg = cntrlBuf.All;
	}
    }
}


/*---------------------------------------------------------------------*/
