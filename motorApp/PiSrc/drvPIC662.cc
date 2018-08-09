/*
FILENAME...	drvPIC662.cc
USAGE...	Motor record driver level support for Physik Instrumente (PI)
		GmbH & Co. C-844 motor controller.

*/

/*
 *      Original Author: Joe Sullivan
 *      Date: 03/08/06
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
 * .01 03/08/06 jps - copied from drvPIC884.cc
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the PIC662 must be powered-on when EPICS is first
	booted up.
    2 - The PIC662 cannot be power cycled while EPICS is up and running.  The
	consequences are permanent communication lose with the PIC662 until
	EPICS is rebooted.
*/

#include <string.h>
#include <math.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <stdlib.h>
#include <errlog.h>
#include "motor.h"
#include "drvPIC662.h"
#include "epicsExport.h"

#define GET_IDENT   "*IDN?"
#define GET_STATUS  "*ESR?"           // Status Register 
#define GET_POS     "POS?"           // Status Register 

#define PIC662_NUM_CARDS	8
#define MAX_AXES		1
#define BUFF_SIZE 100		/* Maximum length of string to/from PIC662 */

/*----------------debugging-----------------*/
volatile int drvPIC662debug = 0;
extern "C" {epicsExportAddress(int, drvPIC662debug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvPIC662debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int PIC662_num_cards = 0;
static char *PIC662_axis[4] = {"1", "2", "3", "4"};

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

struct driver_table PIC662_access =
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
    PIC662_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPIC662 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPIC662);}

static struct thread_args targs = {SCAN_RATE, &PIC662_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIC662_num_cards <=0)
	printf("    No PIC662 controllers configured.\n");
    else
    {
	for (card = 0; card < PIC662_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PIC662 controller %d connection failed.\n", card);
	    else
	    {
		struct PIC662controller *cntrl;

		cntrl = (struct PIC662controller *) brdptr->DevicePrivate;
	    	printf("    PIC662 controller #%d, port=%s, id: %s \n", card,
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
    if (PIC662_num_cards <= 0)
    {
	Debug(1, "init(): PIC662 driver disabled. PIC662Setup() missing from startup script.\n");
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
    struct PIC662controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    E662_ESR_REG mstat;
    int rtn_state;
    int comm_status;
    long motorData;
    bool plusdir, ls_active = false, plusLS, minusLS;
    bool eventflgs;
    msta_field status;

    cntrl = (struct PIC662controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    send_mess(card, GET_STATUS, (char*) NULL);
    comm_status = recv_mess(card, buff, 1);
    if (comm_status == 0)
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
    status.Bits.EA_POSITION = 0;

    mstat.All = atoi(&buff[0]);

    eventflgs = (mstat.Bits.OpComplete || 
		 mstat.Bits.DevError || 
		 mstat.Bits.ExeError)  ? true : false;

    status.Bits.RA_DONE = (eventflgs || cntrl->stop_status)? 1 : 0;
    
    cntrl->stop_status = false;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    send_mess(card, GET_POS, (char*) NULL);
    recv_mess(card, buff, 1);

    motorData = NINT (atof(buff) / cntrl->drive_resolution);

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	  if (motor_info->no_motion_count++ > 1)
	    status.Bits.RA_DONE = 1;  // No done indicator - stop when at position 
	
    }
    else
    {
	status.Bits.RA_DIRECTION = (motorData >= motor_info->position) ? 1 : 0;
	motor_info->position = motorData;
	motor_info->no_motion_count = 0;
    }

    // motor_info->encoder_position = (epicsInt32) motorData;
    status.Bits.RA_PROBLEM	= 0;

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    // No hardware limit switch feedback
    plusLS = false;
    minusLS = false;

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
	send_mess(card, buff, (char*) NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the PIC662 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    struct PIC662controller *cntrl;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);
    
    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
	errlogMessage("drvPIC662.cc:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (comsize == 0)	/* Normal exit on empty input message. */
	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvPIC662.cc:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    local_buff[0] = 0;	/* Terminate local buffer. */

    /* Make a local copy of the string. */
    strcat(local_buff, com);

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct PIC662controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
		       COMM_TIMEOUT, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the PIC662 board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct PIC662controller *cntrl;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct PIC662controller *) motor_state[card]->DevicePrivate;
    
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
/* PIC662Setup()                                     */
/*****************************************************/
RTN_STATUS
PIC662Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PIC662_NUM_CARDS)
	PIC662_num_cards = PIC662_NUM_CARDS;
    else
	PIC662_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before PIC662Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(PIC662_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PIC662_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PIC662Config()                                    */
/*****************************************************/
RTN_STATUS
PIC662Config(int card,		 /* card being configured */
             const char *name)   /* asyn port name */
{
    struct PIC662controller *cntrl;

    if (card < 0 || card >= PIC662_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIC662controller));
    cntrl = (struct PIC662controller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
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
    struct PIC662controller *cntrl;
    int card_index;
    char buff[BUFF_SIZE];
    int status;
    asynStatus success_rtn;
    static const char output_terminator[] = "\n";
    static const char  input_terminator[] = "\n";

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIC662_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PIC662_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->ident[0] = 0;	/* No controller identification message. */
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct PIC662controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
						&cntrl->pasynUser, NULL);
        status = 0;
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
		send_mess(card_index, GET_IDENT, (char*) NULL);
		status = recv_mess(card_index, buff, 1);
                retry++;
	    } while (status == 0 && retry < 3);
	}

	if (success_rtn == asynSuccess && status > 0)
	{
	    strcpy(brdptr->ident, &buff[0]);
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    brdptr->total_axis = 1; /* Single axis controller */

	    /* Fixed resolution */
	    cntrl->res_decpts = 3;
	    cntrl->drive_resolution = 1.0/pow(10.0,cntrl->res_decpts); 


	    /* Set Controller to REMOTE mode */
     	    send_mess(card_index, REMOTE_MODE, (char*) NULL);

	    {
	        struct mess_info *motor_info = &brdptr->motor_info[0];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[0].motor_motion = NULL;
		/* PIC662 has DC motor support only */
		motor_info->encoder_present = NO;
		motor_info->status.Bits.EA_PRESENT = 0;
		motor_info->pid_present = NO;
		motor_info->status.Bits.GAIN_SUPPORT = 0;

		set_status(card_index, 0);  /* Read status of each motor */
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

    epicsThreadCreate((char *) "PIC662_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

