/*
FILENAME...	drvPM500.cc
USAGE...	Motor record driver level support for Newport PM500.

Version:	$Revision: 1.15 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-12-21 15:38:50 $
*/

/* Device Driver Support routines for PM500 motor controller */
/*
 *      Original Author: Mark Rivers
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
 * .01 11-19-98 mlr Initial development, based on drvMM4000.c
 * .02 06-02-00	rls integrated into standard motor record
 * .03 10/02/01 rls allow one retry after a communication error.
 * .04 05-23-03	rls Converted to R3.14.x.
 * .05 02/03/04 rls Eliminate erroneous "Motor motion timeout ERROR".
 * .06 07/09/04 rls removed unused <driver>Setup() argument.
 * .07 07/28/04 rls "epicsExport" debug variable.
 * .08 09/21/04 rls support for 32axes/controller.
 * .09 12/21/04 rls - MS Visual C compatibility; make all epicsExportAddress
 *		      extern "C" linkage.
 *		    - make debug variables always available.
 */


#include <string.h>
#include <math.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "NewportRegister.h"
#include "drvMMCom.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define READ_RESOLUTION ""
#define READ_STATUS     ""
#define READ_POSITION   ""
#define MOTOR_ON        ""
#define GET_IDENT       "SVN?"

/* Status byte bits */
#define M_AXIS_MOVING     0x01
#define M_MOTOR_POWER     0x02
#define M_MOTOR_DIRECTION 0x04
#define M_PLUS_LIMIT      0x08
#define M_MINUS_LIMIT     0x10
#define M_HOME_SIGNAL     0x20

#define PM500_NUM_CARDS	4
#define PM500_NUM_CHANNELS        12
#define BUFF_SIZE 100       /* Maximum length of string to/from PM500 */

#define SERIAL_TIMEOUT	2.0	/* Command timeout in sec. */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	#define Debug(l, f, args...) { if(l<=drvPM500debug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvPM500debug = 0;
extern "C" {epicsExportAddress(int, drvPM500debug);}

/* --- Local data. --- */
int PM500_num_cards = 0;
static char *PM500_axis_names[] = {"X", "Y", "Z", "A", "B", "C", "D", "E", "F",
				   "G", "H", "I"};

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

struct driver_table PM500_access =
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
    PM500_axis_names
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
} drvPM500 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPM500);}

static struct thread_args targs = {SCAN_RATE, &PM500_access};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PM500_num_cards <=0)
	printf("    No PM500 controllers configured.\n");
    else
    {
	for (card = 0; card < PM500_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PM500 controller #%d connection failed.\n", card);
	    else
	    {
		struct MMcontroller *cntrl;

		cntrl = (struct MMcontroller *) brdptr->DevicePrivate;
	    	printf("    PM500 controller %d port=%s, address=%d, id: %s \n", 
			   card, cntrl->asyn_port, cntrl->asyn_address,
			   brdptr->ident);
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
    if (PM500_num_cards <= 0)
    {
	Debug(1, "init(): PM500 driver disabled. PM500Setup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
    struct MMcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char *axis_name, status_char, dir_char, buff[BUFF_SIZE],
	response[BUFF_SIZE];
    int rtnval, rtn_state = 0;
    double motorData;
    bool ls_active;
    msta_field status;

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    axis_name = PM500_axis_names[signal];
    status.All = motor_info->status.All;

    /* Request the status and position of this motor */
    sprintf(buff, "%sR", axis_name);
    send_mess(card, buff, (char) NULL);
    rtnval = recv_mess(card, response, 1);
    if (rtnval > 0)
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

    status_char = response[1];
    dir_char = response[2];
    motorData = atof(&response[2]) / cntrl->drive_resolution[signal];

    status.Bits.RA_DONE	     = (status_char == 'B') ? 0 : 1;
    status.Bits.RA_PROBLEM   = (status_char == 'E') ? 1 : 0;
    status.Bits.RA_DIRECTION = (dir_char == '+')    ? 1 : 0;
    
    if (status_char == 'L')
    {
	ls_active = true;
	if (dir_char == '+')
	    status.Bits.RA_PLUS_LS = 1;
	else
	    status.Bits.RA_MINUS_LS = 1;
    }
    else
    {
	ls_active = false;
	status.Bits.RA_PLUS_LS = 0;
	status.Bits.RA_MINUS_LS = 0;
    }

    status.Bits.RA_HOME = 0;

    /* encoder status */
    status.Bits.EA_POSITION	= 0;
    status.Bits.EA_SLIP		= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->position = NINT(motorData);
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
	strcpy(buff, nodeptr->postmsgptr);
	strcat(buff, "\r");
	send_mess(card, buff, (char) NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the PM500 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    struct MMcontroller *cntrl;
    int size;
    int nwrite;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvPM500.c:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvPM500.c:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    Debug(2, "send_mess(): message = %s\n", com);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    pasynOctetSyncIO->write(cntrl->pasynUser, com, strlen(com), 
                            SERIAL_TIMEOUT, &nwrite);

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
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int flag)
{
    struct MMcontroller *cntrl;
    double timeout = 0.;
    int flush=1;
    int nread = 0;
    asynStatus status;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    if (flag != FLUSH) {
        flush=0;
	timeout	= SERIAL_TIMEOUT;
    }
    if (flush) status = pasynOctetSyncIO->flush(cntrl->pasynUser);
    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
	com[0] = '\0';
	nread = 0;
    }
    else
    {
	/* Test for "system error" response. */
	if (strncmp(com, "SE", 2) == 0)
	    errlogMessage("recv_mess(): PM500 system error.\n");
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* PM500Setup()                                     */
/*****************************************************/
RTN_STATUS
PM500Setup(int num_cards,	/* maximum number of controllers in system.  */
	   int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PM500_NUM_CARDS)
	PM500_num_cards = PM500_NUM_CARDS;
    else
	PM500_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before PM500Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(PM500_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PM500_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PM500Config()                                    */
/*****************************************************/
RTN_STATUS
PM500Config(int card,	/* card being configured */
            const char *name,   /*asyn port name */
            int address)        /*asyn address (GPIB) */
{
    struct MMcontroller *cntrl;

    if (card < 0 || card >= PM500_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MMcontroller));
    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
    cntrl->asyn_address = address;
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
    struct MMcontroller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status, digits;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PM500_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PM500_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct MMcontroller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 
                          cntrl->asyn_address, &cntrl->pasynUser, NULL);

	if (success_rtn == asynSuccess)
	{
	    /* flush any junk at input port - should not be any data available */
            pasynOctetSyncIO->flush(cntrl->pasynUser);
    
	    /* Send a SCUM 1 command to put device in this mode. */
	    send_mess(card_index, "SCUM 1", (char) NULL);
	    recv_mess(card_index, buff, 1);
	    
	    /* Set up basic controller parameters 
	     *  "ENAINT $AF" means the following:
	     *   Bit 0=1, only affected axis halts on limit
	     *   Bit 1=1, No message when moving axis beyond limit
	     *   Bit 2=1, No query echo, prepends status character to axis.
	     *   Bit 3=1, NO status character inserted in responses.
	     *   Bit 4=0, No acknowledgement when command is received
	     *   Bit 5=1, Disable sign-on message at power-up
	     *   Bit 6=0, No echo
	     *   Bit 7=1, CR terminator only on commands and responses
	     *   Bit 8=0, CR terminator only on commands and responses
	     *   Bit 9=0, No EOI sent
	     *   Bit 10=0, CR terminator only on commands and responses
	     *   Bit 11=0, CR terminator only on commands and responses
	     *   Bit 12=0, Decimal number format 
	     *   Bit 13=0, Eearly serial poll mapping
	     *   Bit 14=0, No SRQ assertion
	     */
	    send_mess(card_index, "SENAINT $AF", (char) NULL);
	    recv_mess(card_index, buff, 1);
    
	    /* Send a message and read response from controller to see if 
	     * it exists */
	    send_mess(card_index, GET_IDENT, (char) NULL);
	    status = recv_mess(card_index, buff, 1);  
	    /* Return value is length of response string */
	}

	if (success_rtn == asynSuccess && status > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    send_mess(card_index, GET_IDENT, (char) NULL);	/* Read controller ID string */
	    recv_mess(card_index, buff, 1);
	    strncpy(brdptr->ident, &buff[2], 50);  /* Skip "XD" */

            /* Figure out how many axes this controller has.
             * Do this by querying status of each axis in order */
            for (total_axis = 0; total_axis < PM500_NUM_CHANNELS; total_axis++)
	    {
 		brdptr->motor_info[total_axis].motor_motion = NULL;
		sprintf(buff, "%cSTAT?", PM500_axis_names[total_axis]);
                send_mess(card_index, buff, (char) NULL);
                recv_mess(card_index, buff, 1);
                if (buff[1] == 'E')
		    break;
                /* Determine axis type and resolution.
                 * This is a real pain, since the only way to get this
                 * information is to ask for the axis firmware and use a
                 * lookup table.  We only have translators, so I don't
                 * really know how to tell them apart.  Our translators
                 * return 302 (50nm stages) or 309 (25nm stages) at the end of
                 * the configuration string.
                 * I assume rotators return something different, but I don't
                 * know what it is.
                 */
            }

	    brdptr->total_axis = total_axis;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];
  		char *firmware, *axis_name = PM500_axis_names[motor_index];
		double res = 0.0;

                sprintf(buff, "%sCONFIG?", axis_name);
	        send_mess(card_index, buff, (char) NULL);
	        recv_mess(card_index, buff, 1);
                firmware = &buff[8];
	        Debug(3, "motor_init: firmware = %s\n", firmware);
		if (!strcmp(firmware, "302"))
		{
		    /* 50 nm translator */
		    res = .01;
		    Debug(3, "motor_init: axis %d is a 50 nm translator\n", motor_index);
		}
		else if (!strcmp(firmware, "309"))
		{
		    /* 25 nm translator */
		    res = .01;
		    Debug(3, "motor_init: axis %d is a 25 nm translator\n", motor_index);
		}
		else if (!strcmp(firmware, "300"))
		{
		    /* ????? translator ?????? */
		    res = .01;
		    Debug(3, "motor_init: axis %d is a ?????? translator\n", motor_index);
		}
		else if (!strcmp(firmware, "XXX"))
		{
		    /* Rotator */
		    res = .01;
		}

                /* Set drive resolution. */
		cntrl->drive_resolution[motor_index] = res;

		digits = (int) -log10(cntrl->drive_resolution[motor_index]) + 2;
		if (digits < 1)
		    digits = 1;
		cntrl->res_decpts[motor_index] = digits;              		

		/* PM500 only supports DC motors. */
		motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = YES;
		motor_info->status.Bits.GAIN_SUPPORT = 1;

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;

		motor_info->encoder_present = NO;
		motor_info->pid_present = NO;

		cntrl->home_preset[motor_index] = 0;

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

    epicsThreadCreate((char *) "PM500_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

