/*
FILENAME...	drvPIC848.cc
USAGE...	Motor record driver level support for Physik Instrumente (PI)
	GmbH & Co. C-848 motor controller.

*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 10/18/05
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
 * .01 10/18/05 rls - copied from drvPIC844.cc
 * .02 10/17/07 rls - Added "Motor motion timeout" error check.
 *                  - Set "reference" home switch indicator.
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the PIC848 must be powered-on when EPICS is first
    booted up.
*/

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <stdlib.h>
#include <errlog.h>
#include "motorRecord.h"
#include "motor.h"
#include "drvPIC848.h"
#include "epicsExport.h"

#define GET_IDENT "*IDN?"

#define PIC848_NUM_CARDS	8
#define MAX_AXES		4
#define BUFF_SIZE 100		/* Maximum length of string to/from PIC848 */

/*----------------debugging-----------------*/
volatile int drvPIC848debug = 0;
extern "C" {epicsExportAddress(int, drvPIC848debug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvPIC848debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int PIC848_num_cards = 0;
static char *PIC848_axis[4] = {"A", "B", "C", "D"};
static volatile int motionTO = 10;

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

struct driver_table PIC848_access =
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
    PIC848_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPIC848 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPIC848);}

static struct thread_args targs = {SCAN_RATE, &PIC848_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIC848_num_cards <=0)
	printf("    No PIC848 controllers configured.\n");
    else
    {
	for (card = 0; card < PIC848_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PIC848 controller %d connection failed.\n", card);
	    else
	    {
		struct PIC848controller *cntrl;

		cntrl = (struct PIC848controller *) brdptr->DevicePrivate;
		printf("    PIC848 controller #%d, port=%s, id: %s \n", card,
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
    if (PIC848_num_cards <= 0)
    {
	Debug(1, "init(): PIC848 driver disabled. PIC848Setup() missing from startup script.\n");
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
    struct PIC848controller *cntrl;
    struct mess_node *nodeptr;
    struct mess_info *motor_info;
    struct motorRecord *mr;
    /* Message parsing variables */
    char buff[BUFF_SIZE], axisID;
    C848_Status_Reg mstat;
    int rtn_state, convert_cnt, charcnt, tempInt;
    epicsInt32 motorData;
    bool plusdir, ls_active = false, plusLS, minusLS;
    msta_field status;

    cntrl = (struct PIC848controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    if (nodeptr != NULL)
	mr = (struct motorRecord *) nodeptr->mrecord;
    else
	mr = NULL;
    status.All = motor_info->status.All;

    if (cntrl->status != NORMAL)
	charcnt = recv_mess(card, buff, FLUSH);

    send_mess(card, "STA? #", PIC848_axis[signal]);
    charcnt = recv_mess(card, buff, 1);
    if (charcnt > 2)
	convert_cnt = sscanf(buff, "%c=%d\n", &axisID, &tempInt);

    if (charcnt > 2 && convert_cnt == 2)
    {
	cntrl->status = NORMAL;
	status.Bits.CNTRL_COMM_ERR = 0;
    }
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    rtn_state = OK;
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

    mstat.All = tempInt;
    status.Bits.RA_DONE = (mstat.Bits.Done) ? 1 : 0;
    status.Bits.EA_POSITION = (mstat.Bits.torque) ? 1 : 0;
    plusLS  = mstat.Bits.plus_ls;
    minusLS = mstat.Bits.minus_ls;

    send_mess(card, "POS? #", PIC848_axis[signal]);
    recv_mess(card, buff, 1);

    motorData = NINT(atof(&buff[2]) / cntrl->drive_resolution[signal]);

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)   /* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {

	status.Bits.RA_DIRECTION = (motorData >= motor_info->position) ? 1 : 0;
	motor_info->position =  motor_info->encoder_position = motorData;
	motor_info->no_motion_count = 0;
    }

    if (motor_info->no_motion_count > motionTO)
    {
        status.Bits.RA_PROBLEM = 1;
        send_mess(card, "HLT  #", PIC848_axis[signal]);
        motor_info->no_motion_count = 0;
        errlogSevPrintf(errlogMinor, "Motor motion timeout ERROR on card: %d, signal: %d\n",
            card, signal);
    }
    else
        status.Bits.RA_PROBLEM = 0;
    
    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

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
	status.Bits.RA_MINUS_LS	= 0;

    /* encoder status */
    status.Bits.EA_SLIP       = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME       = 0;

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
/* send a message to the PIC848 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    struct PIC848controller *cntrl;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
	errlogMessage("drvPIC848.cc:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (comsize == 0)  /* Normal exit on empty input message. */
	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvPIC848.cc:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    local_buff[0] = (char) NULL;    /* Terminate local buffer. */

    if (name == NULL)
	strcat(local_buff, com);    /* Make a local copy of the string. */
    else
    {
	strcpy(local_buff, com);
	local_buff[5] = *name;	/* put in axis. */
    }

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct PIC848controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
			    COMM_TIMEOUT, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the PIC848 board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct PIC848controller *cntrl;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct PIC848controller *) motor_state[card]->DevicePrivate;

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
/* PIC848Setup()                                     */
/*****************************************************/
RTN_STATUS
PIC848Setup(int num_cards,  /* maximum number of controllers in system.  */
	    int scan_rate)  /* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PIC848_NUM_CARDS)
	PIC848_num_cards = PIC848_NUM_CARDS;
    else
	PIC848_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

    /* 
     * Allocate space for motor_state structures.  Note this must be done
     * before PIC848Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(PIC848_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PIC848_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PIC848Config()                                    */
/*****************************************************/
RTN_STATUS
PIC848Config(int card,	     /* card being configured */
	     const char *name,	 /* asyn port name */
	     int addr)		 /* asyn address (GPIB) */
{
    struct PIC848controller *cntrl;

    if (card < 0 || card >= PIC848_num_cards)
	return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIC848controller));
    cntrl = (struct PIC848controller *) motor_state[card]->DevicePrivate;

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
    struct PIC848controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis;
    int status;
    asynStatus success_rtn;
    static const char output_terminator[] = "\n";
    static const char  input_terminator[] = "\n";

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIC848_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PIC848_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->ident[0] = (char) NULL;	/* No controller identification message. */
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct PIC848controller *) brdptr->DevicePrivate;

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

            /* Determine # of axes. Request stage name.  See if it responds */
	    for (total_axis = 0; total_axis < MAX_AXES; total_axis++)
	    {
		send_mess(card_index, "CST? #", PIC848_axis[total_axis]);
		status = recv_mess(card_index, buff, 1);
		if (strcmp(&buff[2],"NOSTAGE") == 0)
		    break;
	    }
	    brdptr->total_axis = total_axis;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* PIC848 has DC motor support only */
		motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = YES;
		motor_info->status.Bits.GAIN_SUPPORT = 1;

		cntrl->drive_resolution[motor_index] = POS_RES;

                /* Determine if stage has a reference (home) switch. */
		send_mess(card_index, "REF? #", PIC848_axis[motor_index]);
		status = recv_mess(card_index, buff, 1);
		if (strcmp(&buff[2],"0") == 0)
                    cntrl->reference[motor_index] = false;
                else
                    cntrl->reference[motor_index] = true;

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

    epicsThreadCreate((char *) "PIC848_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

