/*
FILENAME...	drvOms.c
USAGE...	Driver level support for OMS models VME8, VME44 and VS4.

Version:	$Revision: 1.7 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2001-12-14 20:52:56 $
*/

/*
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
 *
 * NOTES
 * -----
 * Verified with firmware:
 *	- VME44    ver 1.97-4E
 *	- VME8     ver 1.97-8
 *	- VS4-040  ver 1.04
 *
 * Modification Log:
 * -----------------
 */

/*========================stepper motor driver ========================

 function:
	Allow users to queue messages to axis on a OMS stepper
	motor controller board.  Each axis of every board available can
	be accessed independantly.

 public functions:
	motor_init() -	Initialize the driver task and structures for all
			boards available for the system.
 private functions:
	send_mess() -	Send a message to the OMS board.
	recv_mess() -	Receive a message from the OMS board.


========================stepper motor driver ========================*/

#include	<vxWorks.h>
#include	<stdio.h>
#include	<sysLib.h>
#include	<string.h>
#include	<taskLib.h>
#include	<rebootLib.h>
#include	<logLib.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<devSup.h>
#include	<drvSup.h>
#ifdef __cplusplus
extern "C" {
#include	<recSup.h>
#include	<devLib.h>
#include	<errlog.h>
}
#else
#include	<recSup.h>
#include        <devLib.h>
#include	<errlog.h>
#endif
#include	<errMdef.h>
#include	<intLib.h>

#include	"motor.h"
#include	"drvOms.h"

#define PRIVATE_FUNCTIONS 1	/* normal:1, debug:0 */
#define STATIC static

/* Define for return test on locationProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

#define CMD_CLEAR       '\030'	/* Control-X, clears command errors only */

#define	ALL_INFO	"QA RP RE EA"	/* jps: move QA to top. */
#define	AXIS_INFO	"QA RP"		/* jps: move QA to top. */
#define	ENCODER_QUERY	"EA"
#define	DONE_QUERY	"RA"

/*----------------debugging-----------------*/
#ifdef	DEBUG
    #define Debug(l, f, args...) { if(l<=drvOMSdebug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

/* Global data. */
int oms44_num_cards = 0;
volatile int drvOMSdebug = 0;
volatile int omsSpyClkRate = 0;
#ifdef __cplusplus
extern "C" long locationProbe(epicsAddressType, char *);
#else
extern long locationProbe(epicsAddressType, char *);
#endif

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* --- Local data common to all OMS drivers. --- */
STATIC char *oms_addrs = 0x0;
STATIC volatile unsigned omsInterruptVector = 0;
STATIC volatile epicsUInt8 omsInterruptLevel = OMS_INT_LEVEL;
STATIC volatile int max_io_tries = MAX_COUNT;
STATIC volatile int motionTO = 10;
STATIC char oms_axis[] = {'X', 'Y', 'Z', 'T', 'U', 'V', 'R', 'S'};

/*----------------functions-----------------*/

/* Common local function declarations. */
static long report(int level);
static long init();
static void query_done(int, int, struct mess_node *);
static int set_status(int card, int signal);
static int send_mess(int card, char const *com, char c);
static int recv_mess(int, char *, int);
static void motorIsr(int card);
static int motor_init();
static void oms_reset();

STATIC int omsGet(int card, char *pcom, int timeout);
STATIC int omsPut(int card, char *pcom);
STATIC int omsError(int card);
STATIC int motorIsrEnable(int card);
STATIC void motorIsrDisable(int card);

/*----------------functions-----------------*/

struct driver_table oms_access =
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
    NULL,
    &initialized,
    oms_axis
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
} drvOms = {2, report, init};

static long report(int level)
{
    int card;

    if (oms44_num_cards <= 0)
	printf("    No VME8/44 controllers configured.\n");
    else
    {
	for (card = 0; card < oms44_num_cards; card++)
	    if (motor_state[card])
		printf("    Oms VME8/44 motor card %d @ 0x%X, id: %s \n", card,
		       (uint_t) motor_state[card]->localaddr,
		       motor_state[card]->ident);
    }
    return (0);
}

static long init()
{
    initialized = ON;	/* Indicate that driver is initialized. */
    (void) motor_init();
    return ((long) 0);
}


STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
    char buffer[40];

    send_mess(card, DONE_QUERY, oms_axis[axis]);
    recv_mess(card, buffer, 1);

    if (nodeptr->status & RA_PROBLEM)
	send_mess(card, AXIS_STOP, oms_axis[axis]);
}


STATIC int set_status(int card, int signal)
{
    struct mess_info *motor_info;
    char *p, *tok_save;
    struct axis_status *ax_stat;
    struct encoder_status *en_stat;
    char q_buf[50];
    int i, pos;
    int rtn_state;

    motor_info = &(motor_state[card]->motor_info[signal]);

    if (motor_state[card]->motor_info[signal].encoder_present == YES)
    {
	/* get 4 peices of info from axis */
	send_mess(card, ALL_INFO, oms_axis[signal]);
	recv_mess(card, q_buf, 4);
    }
    else
    {
	send_mess(card, AXIS_INFO, oms_axis[signal]);
	recv_mess(card, q_buf, 2);
    }

    Debug(5, "info = (%s)\n", q_buf);

    for (i = 0, p = strtok_r(q_buf, ",", &tok_save); p;
	 p = strtok_r(NULL, ",", &tok_save), i++)
    {
	switch (i)
	{
	case 0:		/* axis status */
	    ax_stat = (struct axis_status *) p;

	    if (ax_stat->direction == 'P')
		motor_info->status |= RA_DIRECTION;
	    else
		motor_info->status &= ~RA_DIRECTION;

	    if (ax_stat->done == 'D')
		motor_info->status |= RA_DONE;
	    else
		motor_info->status &= ~RA_DONE;

	    if (ax_stat->overtravel == 'L')
		motor_info->status |= RA_OVERTRAVEL;
	    else
		motor_info->status &= ~RA_OVERTRAVEL;

	    if (ax_stat->home == 'H')
		motor_info->status |= RA_HOME;
	    else
		motor_info->status &= ~RA_HOME;

	    break;
	case 1:		/* motor pulse count (position) */
	    sscanf(p, "%i", &pos);

	    if (pos == motor_info->position)
		motor_info->no_motion_count++;
	    else
		motor_info->no_motion_count = 0;

	    if (motor_info->no_motion_count > motionTO)
	    {
		motor_info->status |= RA_PROBLEM;
		send_mess(card, AXIS_STOP, oms_axis[signal]);
		motor_info->no_motion_count = 0;
		errlogSevPrintf(errlogMinor, "Motor motion timeout ERROR on card: %d, signal: %d\n",
		    card, signal);
	    }
	    else
		motor_info->status &= ~RA_PROBLEM;

	    motor_info->position = pos;
	    break;
	case 2:		/* encoder pulse count (position) */
	    {
		int temp;

		sscanf(p, "%i", &temp);
		motor_info->encoder_position = (epicsInt32) temp;
	    }
	    break;
	case 3:		/* encoder status */
	    en_stat = (struct encoder_status *) p;

	    if (en_stat->slip_enable == 'E')
		motor_info->status |= EA_SLIP;
	    else
		motor_info->status &= ~EA_SLIP;

	    if (en_stat->pos_enable == 'E')
		motor_info->status |= EA_POSITION;
	    else
		motor_info->status &= ~EA_POSITION;

	    if (en_stat->slip_detect == 'S')
		motor_info->status |= EA_SLIP_STALL;
	    else
		motor_info->status &= ~EA_SLIP_STALL;

	    if (en_stat->axis_home == 'H')
		motor_info->status |= EA_HOME;
	    else
		motor_info->status &= ~EA_HOME;

	    break;
	default:
	    break;
	}
    }

    /*
     * jps: Velocity should be set based on the actual velocity returned from
     * the 'RV' command (See drvOms58.c). But the polling task does not have
     * time to request additional information so the velocity is set to
     * indicate moving or not-moving.
     */
    if (motor_info->status & RA_DONE)
	motor_info->velocity = 0;
    else
	motor_info->velocity = 1;

    if (!(motor_info->status & RA_DIRECTION))
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count ||
     (motor_info->status & (RA_OVERTRAVEL | RA_DONE | RA_PROBLEM))) ? 1 : 0;

    /* Test for post-move string. */
    if ((motor_info->status & RA_DONE || motor_info->status & RA_OVERTRAVEL) &&
        (motor_info->motor_motion != 0) &&
	(motor_info->motor_motion->postmsgptr != 0))
    {
	send_mess(card, motor_info->motor_motion->postmsgptr, oms_axis[signal]);
	motor_info->motor_motion->postmsgptr = NULL;
    }

    return (rtn_state);
}


/*****************************************************/
/* send a message to the OMS board		     */
/*		send_mess()			     */
/*****************************************************/
STATIC int send_mess(int card, char const *com, char inchar)
{
    char outbuf[MAX_MSG_SIZE];
    int return_code;

    if (strlen(com) > MAX_MSG_SIZE)
    {
	logMsg((char *) "drvOms.cc:send_mess(); message size violation.\n",
	       0, 0, 0, 0, 0, 0);
	return (-1);
    }

    /* Check that card exists */
    if (!motor_state[card])
    {
	logMsg((char *) "drvOms.cc:send_mess() - invalid card #%d\n", card,
	       0, 0, 0, 0, 0);
	return (-1);
    }

    /* Check/Clear command errors */
    omsError(card);

    /* Flush receive buffer */
    recv_mess(card, (char *) NULL, -1);

    if (inchar == NULL)
	strcpy(outbuf, com);
    else
    {
	strcpy(outbuf, "A? ");
	outbuf[1] = inchar;
	strcat(outbuf, com);
    }
    strcat(outbuf, "\n");		/* Add the command line terminator. */

    Debug(9, "send_mess: ready to send message.\n");

    return_code = omsPut(card, outbuf);

    if (return_code == OK)
    {
	Debug(4, "sent message (%s)\n", outbuf);
    }
    else
    {
	Debug(4, "unable to send message (%s)\n", outbuf);
    }
    return (return_code);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int amount)
 *
 * INPUT ARGUMENTS...
 *	card - controller card # (0,1,...).
 *	*com - caller's response buffer.
 *	amount	| -1 = flush controller's output buffer.
 *		| >= 1 = the # of command responses to retrieve into caller's
 *				response buffer.
 *
 * LOGIC...
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *  IF "amount" indicates buffer flush.
 *	WHILE characters left in input buffer.
 *	    Call omsGet().
 *	ENDWHILE
 *  ENDIF
 *
 *  FOR each message requested (i.e. "amount").
 *	Initialize head and tail pointers.
 *	Initialize retry counter and state indicator.
 *	WHILE retry count not exhausted, AND, state indicator is NOT at END.
 *	    IF characters left in controller's input buffer.
 *		Process input character.
 *	    ELSE IF command error occured - call omsError().
 *		ERROR RETURN.
 *	    ENDIF
 *	ENDWHILE
 *	IF retry count exhausted.
 *	    Terminate receive buffer.
 *	    ERROR RETURN.
 *	ENDIF
 *	Terminate command response.
 *  ENDFOR
 *
 *  IF commands processed.
 *	Terminate response buffer.
 *  ELSE
 *	Clear response buffer.
 *  ENDIF
 *  NORMAL RETURN.
 */

STATIC int recv_mess(int card, char *com, int amount)
{
    int i, trys;
    char junk;
    char inchar;
    int piece, head_size, tail_size;
    int return_code;

    return_code = 0;
    inchar = '\0';

    /* Check that card exists */
    if (card >= total_cards)
    {
	Debug(1, "recv_mess - invalid card #%d\n", card);
	return (-1);
    }

    if (amount == -1)
    {
	/* Process request to flush receive queue */
	Debug(7, "recv flush -------------");
	while (omsGet(card, &junk, 0))
	{
	    Debug(7, "%inchar", junk);
	}
	Debug(7, "         -------------");
	return (0);
    }

    for (i = 0; amount > 0; amount--)
    {
	Debug(7, "-------------");
	head_size = 0;
	tail_size = 0;

	for (piece = 0, trys = 0; piece < 3 && trys < 3; trys++)
	{
	    if (omsGet(card, &inchar, max_io_tries))
	    {
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
	    else if (omsError(card))
		/* Command error detected - abort recv */
		return (-1);
	}
	Debug(7, "-------------\n");
	if (trys >= 3)
	{
	    Debug(1, "Timeout occurred in recv_mess\n");
	    com[i] = '\0';
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
/* Get next character from OMS input buffer          */
/*		omsGet()			     */
/*****************************************************/
STATIC int omsGet(int card, char *pchar, int timeout)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    int getCnt = 0;
    int retry = 0;

    pmotorState = motor_state[card];

    if (pmotorState->irqdata->irqEnable)
    {
	/* Get character from isr - if available */
	while ((getCnt = rngBufGet(pmotorState->irqdata->recv_rng, pchar, 1)) == 0 &&
	       retry < timeout)
	{
	    semTake(pmotorState->irqdata->recv_sem, 1);	/* Wait for character */
	    retry += 5000;	/* Compensate for semaphore timeout */
	}
    }
    else
    {
	/* Direct read from card */
	pmotor = (struct vmex_motor *) pmotorState->localaddr;

	while (getCnt == 0 && retry++ < timeout)
	{
	    if (pmotor->status & STAT_INPUT_BUF_FULL)
	    {
		getCnt++;
		*pchar = pmotor->data;
	    }
	}
    }
    return (getCnt);
}

/*****************************************************/
/* Send Message to OMS                               */
/*		omsPut()			     */
/*****************************************************/
STATIC int omsPut(int card, char *pmess)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    int key;
    char *p;
    int putCnt;
    int trys;

    pmotorState = motor_state[card];
    pmotor = (struct vmex_motor *) pmotorState->localaddr;

    /*
     * This section enables the transmitt interrupt - it is not used because
     * of driver-failure during worst-case testing.
     */
    if (FALSE)
    {
	/* Put string into isr transmitt buffer */
	putCnt = strlen(pmess);
	if (rngBufPut(pmotorState->irqdata->send_rng, pmess, putCnt) != putCnt)
	{
	    Debug(1, "omsPut: Put ring full.\n");
	    return (ERROR);
	}

	/* Turn-on transmit buffer interrupt */
	key = intLock();
	pmotor->control |= IRQ_TRANS_BUF;
	intUnlock(key);

	return (semTake(pmotorState->irqdata->send_sem, MAX_COUNT / 5000));
    }
    else
    {
	/* Send next message */
	for (p = pmess; *p != '\0'; p++)
	{
	    trys = 0;
	    while (!(pmotor->status & STAT_TRANS_BUF_EMPTY))
	    {
		if (trys > max_io_tries)
		{
		    Debug(1, "omsPut: Time_out occurred in send\n");
		    return (ERROR);
		}
		if (pmotor->status & STAT_ERROR)
		{
		    Debug(1, "omsPut: error occurred in send\n");
		}
		trys++;
	    }
	    pmotor->data = *p;
	}
    }
    return (OK);
}


/*****************************************************/
/* Clear OMS errors                                  */
/*		omsClearErrors()	             */
/*****************************************************/
STATIC int omsError(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    int rtnStat = FALSE;

    pmotorState = motor_state[card];
    pmotor = (struct vmex_motor *) pmotorState->localaddr;

    if (pmotorState->irqdata->irqEnable)
    {
	/* Check status of last message */
	if (pmotorState->irqdata->irqErrno & STAT_ERROR)
	{
	    /* Error on the card is cleared by the ISR */
	    pmotorState->irqdata->irqErrno &= ~STAT_ERROR;
	    rtnStat = TRUE;
	}
    }
    else
    {
	int i;
	char const *p;

	/* Check/Clear command error from last message */
	if ((pmotor->status) & STAT_ERROR)
	{
	    Debug(1, "omsPut: Error detected! 0x%02x\n", pmotor->status);
	    for (p = ERROR_CLEAR; *p != '\0'; p++)
	    {
		while (!(pmotor->status & STAT_TRANS_BUF_EMPTY));
		pmotor->data = *p;
	    }
	    for (i = 0; i < 20000; i++);
	    rtnStat = TRUE;
	}
    }
    return (rtnStat);
}


/*****************************************************/
/* Interrupt service routine.                        */
/* motorIsr()		                     */
/*****************************************************/
STATIC void motorIsr(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    epicsUInt8 control;
    epicsUInt8 status;
    epicsUInt8 doneFlags;
    char dataChar;

    if (card >= total_cards || (pmotorState = motor_state[card]) == NULL)
    {
	logMsg((char *) "Invalid entry-card #%d\n", card, 0, 0, 0, 0, 0);
	return;
    }

    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    /* Save interrupt state */
    control = pmotor->control;

    /* Status register - clear irqs on read. */
    status = pmotor->status;

    /* Done register - clears on read */
    doneFlags = pmotor->done;

    /* Determine cause of entry */

    if (drvOMSdebug >= 10)
	logMsg((char *) "entry card #%d,status=0x%X,done=0x%X\n", card,
	       status, doneFlags, 0, 0, 0);

    /* Motion done handling */
    if (status & STAT_DONE)
	/* Wake up polling task 'motor_task()' to issue callbacks */
	semGive(motor_sem);

    /* If command error is present - clear it */
    if (status & STAT_ERROR)
    {
	pmotor->data = (epicsUInt8) CMD_CLEAR;

	/* Send null character to indicate error */
	if (drvOMSdebug >= 1)
	    logMsg((char *) "command error detected on card %d\n", card, 0, 0,
		    0, 0, 0);

	pmotorState->irqdata->irqErrno |= STAT_ERROR;
    }

    /* Send message */
/*    if (status & STAT_TRANS_BUF_EMPTY) */
    if (FALSE)
    {
	if (rngBufGet(pmotorState->irqdata->send_rng, &dataChar, 1))
	    pmotor->data = dataChar;
	else
	{
	    /* Transmit done - disable irq */
	    semGive(pmotorState->irqdata->send_sem);
	    control &= ~IRQ_TRANS_BUF;
	}
    }

    /* Read Response */
    if (status & STAT_INPUT_BUF_FULL)
    {
	dataChar = pmotor->data;

	if (!rngBufPut(pmotorState->irqdata->recv_rng, &dataChar, 1))
	{
	    logMsg((char *) "card %d recv ring full, lost '%c'\n", card,
		   dataChar, 0, 0, 0, 0);
	    pmotorState->irqdata->irqErrno |= STAT_INPUT_BUF_FULL;
	}
	semGive(pmotorState->irqdata->recv_sem);
    }
    /* Update-interrupt state */
    pmotor->control = control;
}

STATIC int motorIsrEnable(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    long status;
    epicsUInt8 cardStatus;

    Debug(5, "motorIsrEnable: Entry card#%d\n", card);

    pmotorState = motor_state[card];
    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    status = devConnectInterrupt(intVME, omsInterruptVector + card,
		    (void (*)(void *)) motorIsr, (void *) card);
    
    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__, "Can't connect to vector %d\n",
		  omsInterruptVector + card);
	pmotorState->irqdata->irqEnable = FALSE;	/* Interrupts disable on card */
	pmotor->control = 0;
	return (ERROR);
    }

    status = devEnableInterruptLevel(OMS_INTERRUPT_TYPE,
				     omsInterruptLevel);
    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__,
		  "Can't enable enterrupt level %d\n",
		  omsInterruptLevel);
	pmotorState->irqdata->irqEnable = FALSE;	/* Interrupts disable on card */
	pmotor->control = 0;
	return (ERROR);
    }

    /* Setup card for interrupt-on-done */
    pmotor->vector = omsInterruptVector + card;


    pmotorState->irqdata->recv_rng = rngCreate(OMS_RESP_Q_SZ);
    pmotorState->irqdata->recv_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);

    pmotorState->irqdata->send_rng = rngCreate(MAX_MSG_SIZE * 2);
    pmotorState->irqdata->send_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);

    pmotorState->irqdata->irqEnable = TRUE;
    pmotorState->irqdata->irqErrno = 0;

    /* Clear board status */
    cardStatus = pmotor->status;

    /* enable interrupt-when-done and input-buffer-full interrupts */
    pmotor->control = IRQ_ENABLE_ALL;

    return (OK);
}

STATIC void motorIsrDisable(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    long status;

    Debug(5, "motorIsrDisable: Entry card#%d\n", card);

    pmotorState = motor_state[card];
    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    /* Disable interrupts */
    pmotor->control = 0;

    status = devDisconnectInterrupt(intVME, omsInterruptVector + card,
				    (void (*)(void *)) motorIsr);
    if (!RTN_SUCCESS(status))
	errPrintf(status, __FILE__, __LINE__, "Can't disconnect vector %d\n",
		  omsInterruptVector + card);

    /* Remove interrupt control functions */
    pmotorState->irqdata->irqEnable = FALSE;
    pmotorState->irqdata->irqErrno = 0;
    rngDelete(pmotorState->irqdata->recv_rng);
    rngDelete(pmotorState->irqdata->send_rng);

    semDelete(pmotorState->irqdata->recv_sem);
    semDelete(pmotorState->irqdata->send_sem);
}


/*****************************************************/
/* Configuration function for  module_types data     */
/* areas. omsSetup()                                */
/*****************************************************/
int omsSetup(int num_cards,	/* maximum number of cards in rack */
	     int num_channels,	/* Not used - Channels per card (4 or 8) */
	     void *addrs,	/* Base Address(0x0-0xb000 on 4K boundary) */
	     unsigned vector,	/* noninterrupting(0), valid vectors(64-255) */
	     int int_level,	/* interrupt level (1-6) */
	     int scan_rate)	/* polling rate - 1/60 sec units */
{

    if (num_cards < 1 || num_cards > OMS_NUM_CARDS)
	oms44_num_cards = OMS_NUM_CARDS;
    else
	oms44_num_cards = num_cards;

    /* Check boundary(16byte) on base address */
    if ((uint32_t) addrs & 0xF)
    {
	Debug(1, "omsSetup: invalid base address 0x%X\n", (uint_t) addrs);
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
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;
    return(0);
}

/*****************************************************/
/* initialize all software and hardware		     */
/*		motor_init()			     */
/*****************************************************/
STATIC int motor_init()
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    long status;
    int card_index, motor_index, arg3, arg4;
    char axis_pos[50], encoder_pos[50];
    char *tok_save, *pos_ptr;
    int total_encoders = 0, total_axis = 0;
    void *localaddr, *probeAddr;

    tok_save = NULL;

    /* Check for setup */
    if (oms44_num_cards <= 0)
    {
	Debug(1, "motor_init: *OMS driver disabled* \n omsSetup() is missing from startup script.\n");
	return (ERROR);
    }

    /* allocate space for total number of motors */
    motor_state = (struct controller **) malloc(oms44_num_cards *
						sizeof(struct controller *));

    /* allocate structure space for each motor present */

    total_cards = 0;

    if (rebootHookAdd((FUNCPTR) oms_reset) == ERROR)
	Debug(1, "vme8/44 motor_init: oms_reset disabled\n");

    for (card_index = 0; card_index < oms44_num_cards; card_index++)
    {
	Debug(2, "motor_init: card %d\n", card_index);

	probeAddr = oms_addrs + (card_index * OMS_BRD_SIZE);
	Debug(9, "motor_init: locationProbe() on addr 0x%x\n",
	      (uint_t) probeAddr);
	status = locationProbe(OMS_ADDRS_TYPE, (char *) probeAddr);

	if (PROBE_SUCCESS(status))
	{
	    status = devRegisterAddress(__FILE__, OMS_ADDRS_TYPE,
					(void *) probeAddr, OMS_BRD_SIZE,
					(void **) &localaddr);
	    Debug(9, "motor_init: devRegisterAddress() status = %d\n",
		  (int) status);
	    if (!RTN_SUCCESS(status))
	    {
		errPrintf(status, __FILE__, __LINE__,
			  "Can't register address 0x%x\n", probeAddr);
		return (ERROR);
	    }

	    Debug(9, "motor_init: localaddr = %x\n", (int) localaddr);
	    pmotor = (struct vmex_motor *) localaddr;

	    total_cards++;

	    Debug(9, "motor_init: malloc'ing motor_state\n");
	    motor_state[card_index] = (struct controller *) malloc(sizeof(struct controller));
	    pmotorState = motor_state[card_index];
	    pmotorState->localaddr = (char *) localaddr;
	    pmotorState->motor_in_motion = 0;
	    pmotorState->cmnd_response = OFF;

	    /* Disable Interrupts */
	    pmotorState->irqdata = (struct irqdatastr *) malloc(sizeof(struct irqdatastr));
	    pmotorState->irqdata->irqEnable = FALSE;
	    pmotor->control = 0;

	    send_mess(card_index, "EF", (char) NULL);
	    send_mess(card_index, ERROR_CLEAR, (char) NULL);
	    send_mess(card_index, STOP_ALL, (char) NULL);

	    send_mess(card_index, GET_IDENT, (char) NULL);

	    recv_mess(card_index, (char *) pmotorState->ident, 1);
	    Debug(3, "Identification = %s\n", pmotorState->ident);

	    send_mess(card_index, ALL_POS, (char) NULL);
	    recv_mess(card_index, axis_pos, 1);

	    for (total_axis = 0, pos_ptr = strtok_r(axis_pos, ",", &tok_save);
		 pos_ptr;
		 pos_ptr = strtok_r(NULL, ",", &tok_save), total_axis++)
	    {
		pmotorState->motor_info[total_axis].motor_motion = NULL;
		pmotorState->motor_info[total_axis].status = 0;
	    }

	    Debug(3, "Total axis = %d\n", total_axis);
	    pmotorState->total_axis = total_axis;

	    /*
	     * Enable interrupt-when-done if selected - driver depends on
	     * motor_state->total_axis  being set.
	     */
	    if (omsInterruptVector)
	    {
		if (motorIsrEnable(card_index) == ERROR)
		    errPrintf(0, __FILE__, __LINE__, "Interrupts Disabled!\n");
	    }

	    for (total_encoders = 0, motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		send_mess(card_index, ENCODER_QUERY, oms_axis[motor_index]);
		if (recv_mess(card_index, encoder_pos, 1) == -1)
		{
		    /* Command error - no encoder */
		    Debug(2, "No encoder on %d\n", motor_index);
		    pmotorState->motor_info[motor_index].encoder_present = NO;
		}
		else
		{
		    total_encoders++;
		    pmotorState->motor_info[motor_index].encoder_present = YES;
		}
	    }

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		pmotorState->motor_info[motor_index].status = 0;
		pmotorState->motor_info[motor_index].no_motion_count = 0;
		pmotorState->motor_info[motor_index].encoder_position = 0;
		pmotorState->motor_info[motor_index].position = 0;

		if (pmotorState->motor_info[motor_index].encoder_present == YES)
		    pmotorState->motor_info[motor_index].status |= EA_PRESENT;
		set_status(card_index, motor_index);
	    }

	    Debug(2, "Init Address=0x%08.8x\n", (uint_t) localaddr);
	    Debug(3, "Total encoders = %d\n\n", (int) total_encoders);
	}
	else
	{
	    Debug(3, "motor_init: Card NOT found!\n");
	    motor_state[card_index] = (struct controller *) NULL;
	}
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

    Debug(3, "Motors initialized\n");

    if (sizeof(int) >= sizeof(char *))
    {
	arg3 = (int) (&oms_access);
	arg4 = 0;
    }
    else
    {
	arg3 = (int) ((long) &oms_access >> 16);
	arg4 = (int) ((long) &oms_access & 0xFFFF);
    }
    taskSpawn((char *) "Oms_motor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);

    Debug(3, "Started motor_task\n");
    return (0);
}

/* Disables interrupts. Called on CTL X reboot. */

STATIC void oms_reset()
{
    short card;
    struct vmex_motor *pmotor;
    short status;

    for (card = 0; card < total_cards; card++)
    {
	pmotor = (struct vmex_motor *) motor_state[card]->localaddr;
	if (vxMemProbe((char *) pmotor, READ, sizeof(short), (char *) &status) == OK)
	    pmotor->control &= 0x5f;
    }
}

/*---------------------------------------------------------------------*/
