/*
FILENAME...	drvMAXv.cc
USAGE...	Motor record driver level support for OMS model MAXv.

Version:	$Revision: 1.6 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2005-03-30 19:12:09 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 04/05/04
 *      Current Author: Ron Sluiter
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
 *	- MAXv ver ????
 *
 * Modification Log:
 * -----------------
 * 01  04-05-04 rls Copied from drvOms58.cc
 * 02  09-20-04 rls - support for 32axes/controller.
 *                  - added MAXvConfig() with initilization string.  Axis type
 *                    MUST be set before iocInit is called.
 * 03  12-14-04 rls - MS Visual C compiler support.
 *                  - eliminate calls to devConnectInterrupt() due to C++
 *		      problems with devLib.h; i.e. "sorry, not implemented:
 *		      `tree_list' not supported..." compiler error message.
 * 04  03-21-05 rls - Make MAXv OSI.
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
#include	"drvMAXv.h"

#include	"epicsExport.h"

/* Define for return test on devNoResponseProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

/* jps: INFO messages - add RV and move QA to top */
#define	ALL_INFO	"QA EA"
#define	AXIS_INFO	"QA"
#define	ENCODER_QUERY	"EA ID"
#define	AXIS_CLEAR	"CA"		/* Clear done of addressed axis */
#define	DONE_QUERY	"RA"		/* ?? Is this needed?? */
#define	PID_QUERY	"?KA ID"


/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	#define Debug(l, f, args...) {if (l <= drvMAXvdebug) printf(f, ## args);}
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug
#endif
volatile int drvMAXvdebug = 0;
extern "C" {epicsExportAddress(int, drvMAXvdebug);}

#define pack2x16(p)      ((epicsUInt32)(((p[0])<<16)|(p[1])))
#define INITSTR_SIZE	150	/* 150 byte intialization string. */

/* Global data. */
int MAXv_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* --- Local data common to all OMS drivers. --- */
static char *MAXv_addrs = 0x0;
static epicsAddressType MAXv_ADDRS_TYPE;
static volatile unsigned MAXvInterruptVector = 0;
static volatile epicsUInt8 omsInterruptLevel = OMS_INT_LEVEL;
static volatile int motionTO = 10;
static char *MAXv_axis[] = {"X", "Y", "Z", "T", "U", "V", "R", "S"};
static double quantum;
static char *initstring[MAXv_NUM_CARDS];

/*----------------functions-----------------*/

/* Common local function declarations. */
static long report(int);
static long init();
static void query_done(int, int, struct mess_node *);
static int set_status(int, int);
RTN_STATUS send_mess(int, char const *, char *);
int recv_mess(int, char *, int);
static void motorIsr(int);
static int motor_init();
static void MAXv_reset(void *);
static char *readbuf(volatile struct MAXv_motor *, char *);

static void start_status(int card);
static int motorIsrSetup(int card);

struct driver_table MAXv_access =
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
    MAXv_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvMAXv = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvMAXv);}

static struct thread_args targs = {SCAN_RATE, &MAXv_access};

/*----------------functions-----------------*/

static long report(int level)
{
    int card;

    if (MAXv_num_cards <= 0)
	printf("    No MAXv controllers configured.\n");
    else
    {
	for (card = 0; card < MAXv_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    Oms MAXv motor card #%d not found.\n", card);
	    else
		printf("    Oms MAXv motor card #%d @ 0x%X, id: %s \n", card,
		       (epicsUInt32) motor_state[card]->localaddr,
		       motor_state[card]->ident);
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

    send_mess(card, DONE_QUERY, MAXv_axis[axis]);
    recv_mess(card, buffer, 1);

    if (nodeptr->status.Bits.RA_PROBLEM)
	send_mess(card, AXIS_STOP, MAXv_axis[axis]);
}


/*********************************************************
 * Start card data area update.
 * Each card will take 800-900us to complete update.
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/
static void start_status(int card)
{
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
    volatile struct MAXv_motor *pmotor;
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
    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;
    status.All = motor_info->status.All;

    if (motor_info->encoder_present == YES)
    {
	/* get 4 peices of info from axis */
	send_mess(card, ALL_INFO, MAXv_axis[signal]);
	recv_mess(card, q_buf, 2);
	got_encoder = true;
    }
    else
    {
	/* get 2 peices of info from axis */
	send_mess(card, AXIS_INFO, MAXv_axis[signal]);
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

    /* motor pulse count (position) */
    motorData = pmotor->cmndPos[signal];

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->no_motion_count = 0;
	motor_info->position = motorData;
    }

    if (motor_info->no_motion_count > motionTO)
    {
	status.Bits.RA_PROBLEM = 1;
	send_mess(card, AXIS_STOP, MAXv_axis[signal]);
	motor_info->no_motion_count = 0;
	errlogSevPrintf(errlogMinor, "Motor motion timeout ERROR on card: %d, signal: %d\n",
	    card, signal);
    }
    else
	status.Bits.RA_PROBLEM = 0;
    
    /* Get encoder position */
    motorData = pmotor->encPos[signal];

    motor_info->encoder_position = motorData;

    if (nodeptr != NULL && ls_active == false)
    {
	struct motor_trans *trans = (struct motor_trans *) nodeptr->mrecord->dpvt;
	if (trans->dpm == true)
	{
	    errlogPrintf("Drive power failure at MAXv card#%d motor#%d\n",
			 card, signal);
	}
    }

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

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
	send_mess(card, outbuf, MAXv_axis[signal]);
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
RTN_STATUS send_mess(int card, char const *com, char *name)
{
    volatile struct MAXv_motor *pmotor;
    epicsInt16 putIndex;
    char outbuf[MAX_MSG_SIZE], *p;
    RTN_STATUS return_code;

    if (strlen(com) > MAX_MSG_SIZE)
    {
	errlogPrintf("drvMAXv.cc:send_mess(); message size violation.\n");
	return (ERROR);
    }

    /* Check that card exists */
    if (!motor_state[card])
    {
	errlogPrintf("drvMAXv.cc:send_mess() - invalid card #%d\n", card);
	return (ERROR);
    }

    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;
    Debug(9, "send_mess: pmotor = %x\n", (epicsUInt32) pmotor);

    return_code = OK;

    Debug(9, "send_mess: checking card %d status\n", card);

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
	Debug(5, "send_mess: Waiting for ack: index delta=%d\n", delta);
#endif
	epicsThreadSleep(epicsThreadSleepQuantum());
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
int recv_mess(int card, char *com, int amount)
{
    volatile struct MAXv_motor *pmotor;
    int itera;
    char *bufptr;

    /* Check that card exists */
    if (!motor_state[card])
    {
	Debug(1, "resv_mess - invalid card #%d\n", card);
	return(-1);
    }

    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;

    if (amount == -1)
    {
	if (pmotor->inGetIndex != pmotor->inPutIndex)
	{
	    char junk[80];

	    readbuf(pmotor, junk);

	    Debug(1, "recv_mess(): flushed - %s\n", junk);
	}
	return(0);
    }

    bufptr = com;
    *bufptr = (char) NULL;

    do
    {
	itera = 1;
	double time = 0.0;

	while (pmotor->status1_flag.Bits.text_response == 0 && time < 0.100)
	{
	    Debug(1, "recv_mess(): response wait - %d\n", itera);
	    time += quantum * itera;
	    epicsThreadSleep(quantum * itera);
	    itera++;
	}

	if (pmotor->status1_flag.Bits.text_response == 0)
	{
	    Debug(1, "Timeout occurred in recv_mess\n");
	    *bufptr = '\0';
	    return(-1);
	}

	bufptr = readbuf(pmotor, bufptr);
	if (--amount > 0)
	    *(bufptr-1) = ',';	/* Replace '\n' with delimiter ','. */

    } while (amount > 0);

    Debug(4, "recv_mess(): card#%d - %s\n", card, com);
    return(0);
}

static char *readbuf(volatile struct MAXv_motor *pmotor, char *bufptr)
{
    STATUS1 flag1;
    epicsUInt32 getIndex, putIndex;
    int bufsize;
    char *start, *end, *bufend;
    
    getIndex = pmotor->inGetIndex;
    putIndex = pmotor->inPutIndex;
    bufsize  = putIndex - getIndex;
    
    start  = (char *) &pmotor->inBuffer[getIndex];
    end    = (char *) &pmotor->inBuffer[putIndex];

    if (start < end)	/* Test for message wraparound in buffer. */
    {
	memcpy(bufptr, start, bufsize);
    }
    else
    {
	int size;

	bufend = (char *) &pmotor->inBuffer[BUFFER_SIZE];
	size = bufend - start;
	bufsize += BUFFER_SIZE;

	memcpy(bufptr, start, size);
	memcpy((bufptr + size), (const char *) &pmotor->inBuffer[0], (bufsize - size));
    }
    
    getIndex += bufsize;
    if (getIndex >= BUFFER_SIZE)
	getIndex -= BUFFER_SIZE;
    
    bufptr += (bufsize - 1);
    *bufptr = (char) NULL;

    while (getIndex != pmotor->inPutIndex)
    {
	Debug(1, "readbuf(): flushed - %d\n", pmotor->inBuffer[getIndex]);
	if (++getIndex > BUFFER_SIZE)
	    getIndex = 0;
    }

    pmotor->inGetIndex = getIndex;
    flag1.All = pmotor->status1_flag.All;
    pmotor->status1_flag.All = flag1.All;
    pmotor->msg_semaphore = 0;
    return(bufptr);
}

/*****************************************************/
/* Configuration function for  module_types data     */
/* areas. MAXvSetup()                                */
/*****************************************************/
RTN_VALUES MAXvSetup(int num_cards,	/* maximum number of cards in rack */
	      int addrs_type,	/* VME address type; 16 - A16, 24 - A24 or 32 - A32. */
	      void *addrs,	/* Base Address on 4K (0x1000) boundary. */
	      unsigned vector,	/* noninterrupting(0), valid vectors(64-255) */
	      int int_level,	/* interrupt level (1-6) */
	      int scan_rate)	/* polling rate - 1/60 sec units */
{
    int itera;
    char **strptr;
    RTN_VALUES rtnind = OK;

    if (num_cards < 1 || num_cards > MAXv_NUM_CARDS)
	MAXv_num_cards = MAXv_NUM_CARDS;
    else
	MAXv_num_cards = num_cards;

    switch(addrs_type)
    {
	case 16:
	    MAXv_ADDRS_TYPE = atVMEA16;
	    if ((epicsUInt32) addrs & 0xFFFF0000)
	    {
		errlogPrintf("MAXvSetup(): invalid A16 address = 0x%X.\n",
			     (epicsUInt32) addrs);
		rtnind = ERROR;
	    }
	    else
		MAXv_addrs = (char *) addrs;
	    break;
	case 24:
	    MAXv_ADDRS_TYPE = atVMEA24;
	    if ((epicsUInt32) addrs & 0xF0000000)
	    {
		errlogPrintf("MAXvSetup(): invalid A24 address = 0x%X.\n",
			     (epicsUInt32) addrs);
		rtnind = ERROR;
	    }
	    else
		MAXv_addrs = (char *) addrs;
	    break;
	case 32:
	    MAXv_ADDRS_TYPE = atVMEA32;
	    MAXv_addrs = (char *) addrs;
	    break;
	default:
	    errlogPrintf("MAXvSetup(): invalid VME address type = %d.\n", addrs_type);
	    rtnind = ERROR;
	    break;
    }

    if ((epicsUInt32) addrs & 0x00000FFF)
    {
	errlogPrintf("MAXvSetup(): address = 0x%X, not aligned on 4K boundary.\n",
		     (epicsUInt32) addrs);
	rtnind = ERROR;
    }

    MAXvInterruptVector = vector;
    if (vector < 64 || vector > 255)
    {
	if (vector != 0)
	{
	    errlogPrintf("MAXvSetup: invalid interrupt vector %d\n", vector);
	    MAXvInterruptVector = (unsigned) OMS_INT_VECTOR;
	    rtnind = ERROR;
	}
    }

    if (int_level < 1 || int_level > 6)
    {
	errlogPrintf("MAXvSetup: invalid interrupt level %d\n", int_level);
	omsInterruptLevel = OMS_INT_LEVEL;
	rtnind = ERROR;
    }
    else
	omsInterruptLevel = int_level;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

    /* Allocate memory for initialization strings. */
    for (itera = 0, strptr = &initstring[0]; itera < MAXv_num_cards; itera++, strptr++)
    {
	*strptr = (char *) malloc(INITSTR_SIZE);
	**strptr = (char) NULL;
    }

    return(rtnind);
}

RTN_VALUES MAXvConfig(int card,			/* number of card being configured */
		      const char *initstr)	/* initialization string */
{
    if (card < 0 || card >= MAXv_num_cards)
        return(ERROR);

    if (strlen(initstr) > INITSTR_SIZE)
    {
	errlogPrintf("MAXvConfig: initialization string > %d bytes.\n", INITSTR_SIZE);
	return(ERROR);
    }

    strcpy(initstring[card], initstr);
    return(OK);
}


/*****************************************************/
/* Interrupt service routine.                        */
/* motorIsr()		                     */
/*****************************************************/
static void motorIsr(int card)
{
    volatile struct controller *pmotorState;
    volatile struct MAXv_motor *pmotor;
    STATUS1 status1_flag;

    if (card >= total_cards || (pmotorState = motor_state[card]) == NULL)
    {
	errlogPrintf("Invalid entry-card #%d\n", card);
	return;
    }

    pmotor = (struct MAXv_motor *) (pmotorState->localaddr);
    status1_flag.All = pmotor->status1_flag.All;

    /* Motion done handling */
    if (status1_flag.Bits.done != 0)
    {
	/* Wake up polling task 'motor_task()' to issue callbacks */
	motor_sem.signal();

    }
    if (status1_flag.Bits.cmndError)
	errlogPrintf("command error detected by motorISR() on card %d\n",
		     card);

    if (status1_flag.Bits.text_response != 0)	/* Don't clear this. */
	status1_flag.Bits.text_response = 0;

    pmotor->status1_flag.All = status1_flag.All; /* Release IRQ's. */
}

static int motorIsrSetup(int card)
{
    volatile struct MAXv_motor *pmotor;
    STATUS1 status1_irq;
    long status;

    Debug(5, "motorIsrSetup: Entry card#%d\n", card);

    pmotor = (struct MAXv_motor *) (motor_state[card]->localaddr);

#ifdef vxWorks

    status = pdevLibVirtualOS->pDevConnectInterruptVME(
	MAXvInterruptVector + card, (void (*)()) motorIsr, (void *) card);

    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__, "Can't connect to vector %d\n", MAXvInterruptVector + card);
	return (ERROR);
    }

    status = devEnableInterruptLevel(OMS_INTERRUPT_TYPE,
				     omsInterruptLevel);
    if (!RTN_SUCCESS(status))
    {
	errPrintf(status, __FILE__, __LINE__, "Can't enable enterrupt level %d\n", omsInterruptLevel);
	return (ERROR);
    }

#endif
    
    /* Setup card for interrupt-on-done */
    status1_irq.All = 0;
    status1_irq.Bits.done = 0xFF;
    status1_irq.Bits.cmndError = 1;

    pmotor->status1_irq_enable.All = status1_irq.All;	/* Enable interrupts. */
    pmotor->status2_irq_enable = 0x0;
    return (OK);
}

/*****************************************************/
/* initialize all software and hardware		     */
/* motor_init()			     */
/*****************************************************/
static int motor_init()
{
    struct mess_info *motor_info;
    volatile struct controller *pmotorState;
    volatile struct MAXv_motor *pmotor;
    long status;
    int card_index, motor_index, itera;
    char axis_pos[50], encoder_pos[50], **strptr;
    char *tok_save, *pos_ptr;
    int total_encoders = 0, total_axis = 0, total_pidcnt = 0;
    volatile void *localaddr;
    void *probeAddr;

    tok_save = NULL;
    quantum = epicsThreadSleepQuantum();

    /* Check for setup */
    if (MAXv_num_cards <= 0)
    {
	Debug(1, "motor_init: MAXv driver disabled\nMAXvSetup() is missing from startup script.\n");
	return (ERROR);
    }

    /* allocate space for total number of motors */
    motor_state = (struct controller **) malloc(MAXv_num_cards *
						sizeof(struct controller *));

    /* allocate structure space for each motor present */

    total_cards = MAXv_num_cards;

    if (epicsAtExit(MAXv_reset, NULL) == ERROR)
	Debug(1, "MAXv motor_init: MAXv_reset disabled\n");

    for (card_index = 0; card_index < MAXv_num_cards; card_index++)
    {
	int8_t *startAddr;
	int8_t *endAddr;

	Debug(2, "motor_init: card %d\n", card_index);

	probeAddr = MAXv_addrs + (card_index * MAXv_BRD_SIZE);
	startAddr = (int8_t *) probeAddr;
	endAddr = startAddr + MAXv_BRD_SIZE;

	Debug(9, "motor_init: devNoResponseProbe() on addr 0x%x\n",
	      (epicsUInt32) probeAddr);
	/* Scan memory space to assure card id */
#ifdef vxWorks
	do
	{
	    status = devNoResponseProbe(MAXv_ADDRS_TYPE, (unsigned int) startAddr, 2);
	    startAddr += 0x100;
	} while (PROBE_SUCCESS(status) && startAddr < endAddr);
#endif
	if (PROBE_SUCCESS(status))
	{
#ifdef vxWorks
	    status = devRegisterAddress(__FILE__, MAXv_ADDRS_TYPE,
					(size_t) probeAddr, MAXv_BRD_SIZE,
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
	    pmotor = (struct MAXv_motor *) localaddr;
		
	    if (pmotor->firmware_status.Bits.running == 0)
		errMessage(-1, "controller is NOT running.\n");

	    Debug(9, "motor_init: malloc'ing motor_state\n");
	    motor_state[card_index] = (struct controller *) malloc(sizeof(struct controller));
	    pmotorState = motor_state[card_index];
	    pmotorState->localaddr = (char *) localaddr;
	    pmotorState->motor_in_motion = 0;
	    pmotorState->cmnd_response = false;

	    if (MAXvInterruptVector == 0)
		pmotor->IACK_vector = 0;
	    else
		pmotor->IACK_vector = MAXvInterruptVector + card_index;

	    pmotor->status1_flag.All = 0xFFFFFFFF;
	    pmotor->status2_flag = 0xFFFFFFFF;
	    /* Disable all interrupts */
	    pmotor->status1_irq_enable.All = 0;
	    pmotor->status2_irq_enable = 0;

	    send_mess(card_index, ERROR_CLEAR, (char) NULL);
	    send_mess(card_index, STOP_ALL, (char) NULL);

	    send_mess(card_index, GET_IDENT, (char) NULL);
	    recv_mess(card_index, (char *) pmotorState->ident, 1);
	    Debug(3, "Identification = %s\n", pmotorState->ident);

	    send_mess(card_index, initstring[card_index], (char) NULL);

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

	    for (total_encoders = total_pidcnt = 0, motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		STATUS1 flag1;

	    	/* Test if motor has an encoder. */
		send_mess(card_index, ENCODER_QUERY, MAXv_axis[motor_index]);
		while (!pmotor->status1_flag.Bits.done)	/* Wait for command to complete. */
		    epicsThreadSleep(quantum);

		if (pmotor->status1_flag.Bits.cmndError)
		{
		    Debug(2, "motor_init: No encoder on axis %d\n", motor_index);
		    pmotorState->motor_info[motor_index].encoder_present = NO;
		    flag1.All = pmotor->status1_flag.All;	/* Clear command error. */
		    pmotor->status1_flag.All = flag1.All;
		}
		else
		{
		    total_encoders++;
		    pmotorState->motor_info[motor_index].encoder_present = YES;
		    recv_mess(card_index, encoder_pos, 1);
		}
		
		/* Test if motor has PID parameters. */
		send_mess(card_index, PID_QUERY, MAXv_axis[motor_index]);
		while (!pmotor->status1_flag.Bits.done)	/* Wait for command to complete. */
		    epicsThreadSleep(quantum);
		if (pmotor->status1_flag.Bits.cmndError)
		{
		    Debug(2, "motor_init: No PID parameters on axis %d\n", motor_index);
		    pmotorState->motor_info[motor_index].pid_present = NO;
		    flag1.All = pmotor->status1_flag.All;	/* Clear command error. */
		    pmotor->status1_flag.All = flag1.All;
		}
		else
		{
		    total_pidcnt++;
		    pmotorState->motor_info[motor_index].pid_present = YES;
		    recv_mess(card_index, encoder_pos, FLUSH);	/* Flush response. */
		}
	    }

	    /* Enable interrupt-when-done if selected */
	    if (MAXvInterruptVector)
	    {
		if (motorIsrSetup(card_index) == ERROR)
		    errMessage(-1, "Interrupts Disabled!\n");
	    }

	    start_status(card_index);
	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		motor_info = (struct mess_info *) &pmotorState->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;

		if (motor_info->encoder_present == YES)
		    motor_info->status.Bits.EA_PRESENT = 1;
		if (motor_info->pid_present == YES)
		    motor_info->status.Bits.GAIN_SUPPORT = 1;

		set_status(card_index, motor_index);

		send_mess(card_index, DONE_QUERY, MAXv_axis[motor_index]); /* Is this needed??? */
		recv_mess(card_index, axis_pos, 1);
	    }

	    Debug(2, "motor_init: Init Address=0x%8.8x\n",
		    (epicsUInt32) localaddr);
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

    epicsThreadCreate((char *) "MAXv_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    Debug(3, "Started motor_task\n");

    /* Deallocate memory for initialization strings. */
    for (itera = 0, strptr = &initstring[0]; itera < MAXv_num_cards; itera++, strptr++)
	free(*strptr);

    return (0);
}


/* Disables interrupts. Called on CTL X reboot. */

static void MAXv_reset(void *arg)
{
    short card;
    volatile struct MAXv_motor *pmotor;

    for (card = 0; card < total_cards; card++)
    {
	if (motor_state[card] != NULL)
	{
	    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;
	    pmotor->status1_irq_enable.All = 0;
	}
    }
}

