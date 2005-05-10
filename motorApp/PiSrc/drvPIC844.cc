/*
FILENAME...	drvPIC844.cc
USAGE...	Motor record driver level support for Physik Instrumente (PI)
		GmbH & Co. C-844 motor controller.

Version:	$Revision: 1.12 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2005-05-10 16:42:48 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 12/17/03
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
 * .01 12/17/03 rls - copied from drvIM483PL.cc
 * .02 02/03/04 rls - Eliminate erroneous "Motor motion timeout ERROR".
 * .03 07/12/04 rls - Converted from MPF to asyn.
 * .04 09/09/04 rls - Retry on initial comm. (PIC844 comm. locks up when IOC
 *                    is power cycled).
 * .05 09/21/04 rls - support for 32axes/controller.
 * .06 12/16/04 rls - asyn R4.0 support.
 *		    - make debug variables always available.
 *		    - MS Visual C compatibility; make all epicsExportAddress
 *		      extern "C" linkage.
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the PIC844 must be powered-on when EPICS is first
	booted up.
    2 - The PIC844 cannot be power cycled while EPICS is up and running.  The
	consequences are permanent communication lose with the PIC844 until
	EPICS is rebooted.
*/

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "drvPI.h"
#include "epicsExport.h"

#define GET_IDENT "*IDN?"

#define PIC844_NUM_CARDS	8
#define MAX_AXES		8
#define BUFF_SIZE 100		/* Maximum length of string to/from PIC844 */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	#define Debug(l, f, args...) { if(l<=drvPIC844debug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvPIC844debug = 0;
extern "C" {epicsExportAddress(int, drvPIC844debug);}

/* --- Local data. --- */
int PIC844_num_cards = 0;
static char *PIC844_axis[4] = {"1", "2", "3", "4"};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PIC844_access =
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
    PIC844_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPIC844 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPIC844);}

static struct thread_args targs = {SCAN_RATE, &PIC844_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIC844_num_cards <=0)
	printf("    No PIC844 controllers configured.\n");
    else
    {
	for (card = 0; card < PIC844_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PIC844 controller %d connection failed.\n", card);
	    else
	    {
		struct PIC844controller *cntrl;

		cntrl = (struct PIC844controller *) brdptr->DevicePrivate;
	    	printf("    PIC844 controller #%d, port=%s, id: %s \n", card,
		       cntrl->asyn_port, brdptr->ident);
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
    if (PIC844_num_cards <= 0)
    {
	Debug(1, "init(): PIC844 driver disabled. PIC844Setup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
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

static int set_status(int card, int signal)
{
    struct PIC844controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    C844_Cond_Reg mstat;
    int rtn_state;
    double motorData;
    bool plusdir, ls_active = false, inmotion, plusLS, minusLS;
    msta_field status;

    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    send_mess(card, "AXIS:STAT?", PIC844_axis[signal]);
    recv_mess(card, buff, 1);

    if (strcmp(buff, "ON") == 0)
	status.Bits.EA_POSITION = 1;
    else if (strcmp(buff, "OFF") == 0)
	status.Bits.EA_POSITION = 0;
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    rtn_state = 0;
	}
	else
	{
	    cntrl->status = COMM_ERR;
	    status.Bits.CNTRL_COMM_ERR = 1;
	    status.Bits.RA_PROBLEM     = 1;
	    rtn_state = 1;
	}
	goto exit;
    }
    
    cntrl->status = NORMAL;
    status.Bits.CNTRL_COMM_ERR = 0;

    send_mess(card, "MOT:COND?", (char) NULL);
    recv_mess(card, buff, 1);

    mstat.All = atoi(&buff[0]);
    switch(signal)
    {
	case 0:
	    inmotion = mstat.Bits.axis1IM;
	    break;
	case 1:
	    inmotion = mstat.Bits.axis2IM;
	    break;
	case 2:
	    inmotion = mstat.Bits.axis3IM;
	    break;
	case 3:
	    inmotion = mstat.Bits.axis4IM;
	default:
	    rtn_state = 1;
	    goto exit;
    }

    status.Bits.RA_DONE = (inmotion == YES) ? 0 : 1;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    send_mess(card, "CURR:TPOS?", (char) NULL);
    recv_mess(card, buff, 1);

    motorData = atof(buff);

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	epicsInt32 newposition;

	newposition = NINT(motorData);
	status.Bits.RA_DIRECTION = (newposition >= motor_info->position) ? 1 : 0;
	motor_info->position = newposition;
	motor_info->no_motion_count = 0;
    }

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    switch(signal)
    {
	case 0:
	    plusLS  = mstat.Bits.axis1PL;
	    minusLS = mstat.Bits.axis1ML;
	    break;
	case 1:
	    plusLS  = mstat.Bits.axis2PL;
	    minusLS = mstat.Bits.axis2ML;
	    break;
	case 2:
	    plusLS  = mstat.Bits.axis3PL;
	    minusLS = mstat.Bits.axis3ML;
	    break;
	case 3:
	    plusLS  = mstat.Bits.axis4PL;
	    minusLS = mstat.Bits.axis4ML;
	default:
	    rtn_state = 1;
	    goto exit;
    }

    /* Set limit switch error indicators. */
    if (plusLS == true)
    {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	    ls_active = true;
    }
    else
	status.Bits.RA_PLUS_LS = 0;

    if (minusLS == true)
    {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	    ls_active = true;
    }
    else
	status.Bits.RA_MINUS_LS = 0;

    /* encoder status */
    status.Bits.EA_SLIP	      = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME	      = 0;

    send_mess(card, "AXIS:POS?", (char) NULL);
    recv_mess(card, buff, 1);
    motorData = atof(buff);
    motor_info->encoder_position = (int32_t) motorData;

    status.Bits.RA_PROBLEM	= 0;

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
	strcpy(buff, nodeptr->postmsgptr);
	send_mess(card, buff, (char) NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the PIC844 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    struct PIC844controller *cntrl;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);
    
    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
	errlogMessage("drvPIC844.cc:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (comsize == 0)	/* Normal exit on empty input message. */
	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvPIC844.cc:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    local_buff[0] = (char) NULL;	/* Terminate local buffer. */

    if (name != NULL)
    {
	strcpy(local_buff, "AXIS ");
	strcat(local_buff, name);	/* put in axis. */
	strcat(local_buff, ";");	/* put in comman seperator. */
    }

    /* Make a local copy of the string. */
    strcat(local_buff, com);

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
		       COMM_TIMEOUT, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the PIC844 board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct PIC844controller *cntrl;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;
    
    if (flag == FLUSH)
	pasynOctetSyncIO->flush(cntrl->pasynUser);
    else
	status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
				     COMM_TIMEOUT, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
	com[0] = '\0';
	nread = 0;
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* PIC844Setup()                                     */
/*****************************************************/
RTN_STATUS
PIC844Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PIC844_NUM_CARDS)
	PIC844_num_cards = PIC844_NUM_CARDS;
    else
	PIC844_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before PIC844Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(PIC844_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PIC844_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PIC844Config()                                    */
/*****************************************************/
RTN_STATUS
PIC844Config(int card,		 /* card being configured */
             const char *name,   /* asyn port name */
             int addr)           /* asyn address (GPIB) */
{
    struct PIC844controller *cntrl;

    if (card < 0 || card >= PIC844_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIC844controller));
    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
    cntrl->asyn_address = addr;
    return(OK);
}


/*****************************************************/
/* initialize all software and hardware		     */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()			                     */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct PIC844controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis;
    int status;
    asynStatus success_rtn;
    static const char output_terminator[] = "\n";
    static const char  input_terminator[] = "\n";

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIC844_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PIC844_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->ident[0] = (char) NULL;	/* No controller identification message. */
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct PIC844controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
						&cntrl->pasynUser, NULL);

	if (success_rtn == asynSuccess)
	{
	    int retry = 0;

	    pasynOctetSyncIO->setOutputEos(cntrl->pasynUser, output_terminator,
					   strlen(output_terminator));
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser, input_terminator,
					  strlen(input_terminator));

	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    pasynOctetSyncIO->flush(cntrl->pasynUser);

	    do
	    {
		send_mess(card_index, GET_IDENT, (char) NULL);
		status = recv_mess(card_index, buff, 1);
                retry++;
	    } while (status == 0 && retry < 3);
	}

	if (success_rtn == asynSuccess && status > 0)
	{
	    strcpy(brdptr->ident, &buff[0]);
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    brdptr->total_axis = total_axis = 4;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* PIC844 has DC motor support only */
		motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = YES;
		motor_info->status.Bits.GAIN_SUPPORT = 1;

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

    epicsThreadCreate((char *) "PIC844_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

