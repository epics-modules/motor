/*
FILENAME...	drvMDrive.cc
USAGE...	Motor record driver level support for Intelligent Motion
		Systems, Inc. MDrive series; M17, M23, M34.

Version:	$Revision: 1.13 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-09-20 21:10:34 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 03/21/03
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
 * .01 03/21/03 rls copied from drvIM483PL.c
 * .02 02/03/04 rls Eliminate erroneous "Motor motion timeout ERROR".
 * .03 03/15/04 rls Previous driver releases not working.  Fixed by adding
 *                  Kevin Peterson's eat_garbage() function.  Added support
 *                  for encoder detection via "ident".
 * .04 07/01/04 rls Converted from MPF to asyn.
 * .05 09/20/04 rls - support for 32axes/controller.
 *                  - remove '?' command string padding.
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the MDrive must be powered-on when EPICS is first
	booted up.
    2 - The MDrive cannot be power cycled while EPICS is up and running.  The
	consequences are permanent communication lose with the MDrive until
	EPICS is rebooted.
*/

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "drvIM483.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define MDrive_NUM_CARDS	8
#define MAX_AXES		8
#define BUFF_SIZE 13		/* Maximum length of string to/from MDrive */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int drvMDrivedebug = 0;
	#define Debug(l, f, args...) {if (l <= drvMDrivedebug) printf(f, ## args);}
	epicsExportAddress(int, drvMDrivedebug);
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

/* --- Local data. --- */
int MDrive_num_cards = 0;
static char *MDrive_axis[] = {"1", "2", "3", "4", "5", "6", "7", "8"};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
static int eat_garbage(int, char *, int);
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MDrive_access =
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
    eat_garbage,
    set_status,
    query_done,
    NULL,
    &initialized,
    MDrive_axis
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
} drvMDrive = {2, report, init};

epicsExportAddress(drvet, drvMDrive);

static struct thread_args targs = {SCAN_RATE, &MDrive_access};


/* Single Inputs - response from PR IN command */
typedef union
{
    epicsUInt8 All;
    struct
    {
#ifdef MSB_First
	unsigned int na7:1;
	unsigned int na6:1;
	unsigned int na5:1;
	unsigned int na4:1;
	unsigned int na3:1;
	unsigned int homels:1;		/* Home limit switch.  */
	unsigned int ls_plus:1;		/* Plus limit switch.  */
	unsigned int ls_minus:1;	/* Minus limit switch. */
#else
	unsigned int ls_minus:1;	/* Minus limit switch. */
	unsigned int ls_plus:1;		/* Plus limit switch.  */
	unsigned int homels:1;		/* Home limit switch.  */
	unsigned int na3:1;
	unsigned int na4:1;
	unsigned int na5:1;
	unsigned int na6:1;
	unsigned int na7:1;
#endif
    } Bits;
} SINPUTS;

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (MDrive_num_cards <=0)
	printf("    No MDrive controllers configured.\n");
    else
    {
	for (card = 0; card < MDrive_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    MDrive controller %d connection failed.\n", card);
	    else
	    {
		struct IM483controller *cntrl;

		cntrl = (struct IM483controller *) brdptr->DevicePrivate;
	    	printf("    IM483SM controller #%d, port=%s, id: %s \n", card,
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
    if (MDrive_num_cards <= 0)
    {
	Debug(1, "init(): MDrive driver disabled. MDriveSetup() missing from startup script.\n");
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
    struct IM483controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    int rtnval, rtn_state;
    double motorData;
    SINPUTS inputs;
    bool plusdir, ls_active = false;
    msta_field status;

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    send_mess(card, "PR MV", MDrive_axis[signal]);
    eat_garbage(card, buff, 1);
    rtn_state = recv_mess(card, buff, 1);
    if (rtn_state > 0)
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

    rtnval = atoi(buff);

    status.Bits.RA_DONE = (rtnval != 0) ? 0 : 1;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    send_mess(card, "PR P", MDrive_axis[signal]);
    eat_garbage(card, buff, 1);
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

    send_mess(card, "PR IN", MDrive_axis[signal]);
    eat_garbage(card, buff, 1);
    recv_mess(card, buff, 1);
    inputs.All = atoi(buff);

    /* Set limit switch error indicators. */
    if (inputs.Bits.ls_plus == 0)
    {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	    ls_active = true;
    }
    else
	status.Bits.RA_PLUS_LS = 0;

    if (inputs.Bits.ls_minus == 0)
    {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	    ls_active = true;
    }
    else
	status.Bits.RA_MINUS_LS = 0;

    status.Bits.RA_HOME = (inputs.Bits.homels) ? 1 : 0;

    /* !!! Assume no closed-looped control!!!*/
    status.Bits.EA_POSITION = 0;

    /* encoder status */
    status.Bits.EA_SLIP	      = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME	      = 0;

    if (motor_state[card]->motor_info[signal].encoder_present == NO)
	motor_info->encoder_position = 0;
    else
    {
	send_mess(card, "PR C2", MDrive_axis[signal]);
	eat_garbage(card, buff, 1);
	recv_mess(card, buff, 1);
	motorData = atof(buff);
	motor_info->encoder_position = (int32_t) motorData;
    }

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
	send_mess(card, buff, MDrive_axis[signal]);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the MDrive board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    struct IM483controller *cntrl;
    int size;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvMDrive.c:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvMDrive.c:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    /* Make a local copy of the string and add the command line terminator. */
    if (name != NULL)
    {
	strcpy(local_buff, name);	    /* put in axis */
	strcat(local_buff, com);
    }
    else
	strcpy(local_buff, com);

    strcat(local_buff, "\n");

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
		       COMM_TIMEOUT);

    return(OK);
}


/*****************************************************/
/* eat garbage characters from the MDrive board      */
/* eat_garbage()			             */
/*****************************************************/
static int eat_garbage(int card, char *com, int flag)
{
    struct IM483controller *cntrl;
    int timeout;
    int flush = 0;
    int len = 0;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return (-1);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= COMM_TIMEOUT;

    /* Get the response. */      
    len = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE, (char *) "\r\n",
                            2, flush, timeout, &eomReason);

    Debug(2, "eat_garbage(): len = %i\n", len);
    if (len != 2)
        Debug(2, "eat_garbage(): com = \"%s\"\n", com);

    com[0] = '\0';

    /*Debug(2, "eat_garbage(): message = \"%s\"\n", com);*/
    return (len);
}


/*****************************************************/
/* receive a message from the MDrive board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct IM483controller *cntrl;
    int timeout;
    int flush = 0;
    int len = 0;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= COMM_TIMEOUT;

    /* Get the response. */      
    len = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE, (char *) "\r\n",
                            1, flush, timeout, &eomReason);
    
    if (len == 0)
	com[0] = '\0';
    else
    {
	com[len - 2] = '\0'; /* Strip off trailing <CR LF>. */
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(len);
}


/*****************************************************/
/* Setup system configuration                        */
/* MDriveSetup()                                     */
/*****************************************************/
RTN_STATUS
MDriveSetup(int num_cards,	/* maximum number of controllers in system.  */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > MDrive_NUM_CARDS)
	MDrive_num_cards = MDrive_NUM_CARDS;
    else
	MDrive_num_cards = num_cards;

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
    motor_state = (struct controller **) malloc(MDrive_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < MDrive_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* MDriveConfig()                                    */
/*****************************************************/
RTN_STATUS
MDriveConfig(int card,		/* card being configured */
             const char *name)	/* MPF server task name */
{
    struct IM483controller *cntrl;

    if (card < 0 || card >= MDrive_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct IM483controller));
    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

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
    struct IM483controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (MDrive_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < MDrive_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->ident[0] = (char) NULL;	/* No controller identification message. */
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct IM483controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0, &cntrl->pasynUser);

	if (success_rtn == asynSuccess)
	{
	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
            pasynOctetSyncIO->flush(cntrl->pasynUser);
    
	    for (total_axis = 0; total_axis < MAX_AXES; total_axis++)
	    {
		int retry = 0;
		
		/* Try 3 times to connect to controller. */
		do
		{
		    send_mess(card_index, "PR VR", MDrive_axis[total_axis]);
                    eat_garbage(card_index, buff, 1);
		    status = recv_mess(card_index, buff, 1);
		    retry++;
		} while (status == 0 && retry < 3);

		if (status <= 0)
		    break;
		else if (total_axis == 0)
		    strcpy(brdptr->ident, buff);
	    }
	    brdptr->total_axis = total_axis;
	}

	if (success_rtn == asynSuccess && total_axis > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* Assume no encoder support. */
		motor_info->encoder_present = NO;

                /* Determine if encoder present based last character of "ident". */
		if (brdptr->ident[strlen(brdptr->ident) - 1] == 'E')
		{
		    motor_info->pid_present = YES;
		    motor_info->status.Bits.GAIN_SUPPORT = 1;
		    motor_info->encoder_present = YES;
		    motor_info->status.Bits.EA_PRESENT = 1;
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

    epicsThreadCreate((char *) "MDrive_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

