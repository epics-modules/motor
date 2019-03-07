/*
FILENAME...	drvPIE517.cc
USAGE...	Motor record driver level support for Physik Instrumente (PI)
	        GmbH & Co. E-516 motor controller.

Version:	$Revision: 1.1 $
Modified By:	$Author: sullivan $
Last Modified:	$Date: 2007-03-30 20:01:05 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 10/18/05
 *      Current Author: Joe Sullivan
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
 * .01 08/10/16 Bruno Luvizotto (brunoluvizotto@gmail.com) - copied from devPIE516.cc.
 * .02 08/10/16 Bruno Luvizotto (brunoluvizotto@gmail.com) - edited the files for the E517 controller.
 * 	Tested on PI VER:
 * 	FW_DSP: 02.034 
 * 	FW_FPGA: 02.040 
 * 	FW_MCU: 1.22.6
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the PIE517 must be powered-on when EPICS is first
    booted up.
*/

#include <string.h>
#include <stdlib.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <errlog.h>
#include "motorRecord.h"
#include "motor.h"
#include "drvPIE517.h"
#include "epicsExport.h"

#define GET_IDENT     "VER?"       
#define SET_ONLINE    "ONL # 1"      /* Set Online Mode ON */
#define SET_VELCTRL   "VCO # 1"     /* Set Velocity Control Mode - Required for DONE */
#define READ_ONLINE   "ONL? #"       /* Read Online Mode */
#define READ_POS      "POS? #"     /* Read position */
#define READ_OVERFLOW "OVF? #"     /* Read Servo Overflow Status */
#define READ_ONTARGET "ONT? #"     /* Read Position ON Target */
#define READ_SERVO    "SVO? #"     /* Read Servo Enable Status */


#define PIE517_NUM_CARDS	10
#define MAX_AXES		3
#define BUFF_SIZE 100		/* Maximum length of string to/from PIE517 */

/*----------------debugging-----------------*/
volatile int drvPIE517debug = 0;
extern "C" {epicsExportAddress(int, drvPIE517debug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvPIE517debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int PIE517_num_cards = 0;
static const char *PIE517_axis[] = {"1 ", "2 ", "3 "}; //{"A", "B", "C"};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"


/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, const char *, const char *);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PIE517_access =
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
    PIE517_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPIE517 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPIE517);}

static struct thread_args targs = {SCAN_RATE, &PIE517_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIE517_num_cards <=0)
	printf("    No PIE517 controllers configured.\n");
    else
    {
	for (card = 0; card < PIE517_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PIE517 controller %d connection failed.\n", card);
	    else
	    {
		struct PIE517controller *cntrl;

		cntrl = (struct PIE517controller *) brdptr->DevicePrivate;
		printf("    PIE517 controller #%d, port=%s, id: %s \n", card,
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
    if (PIE517_num_cards <= 0)
    {
	Debug(1, "init(): PIE517 driver disabled. PIE517Setup() missing from startup script.\n");
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
    struct PIE517controller *cntrl;
    struct mess_node *nodeptr;
    struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    int rtn_state;
    unsigned int overflow_status, ontarget_status, servo_status;
//    unsigned int online_status;
    epicsInt32 motorData;
    bool plusdir, ls_active, plusLS, minusLS;
    bool readOK; 
    msta_field status;

    cntrl = (struct PIE517controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    recv_mess(card, buff, FLUSH);

    readOK = false;   
//    send_mess(card, READ_ONLINE, PIE517_axis[signal]);
//    if (recv_mess(card, buff, 1) && sscanf(buff, "%d", &online_status))
//      {
//	if (!online_status)
//	  {
//	    /* Assume Controller Reboot - Set ONLINE and Velocity Control ON */
//	    send_mess(card, SET_ONLINE, PIE517_axis[signal]);
//	    //send_mess(card, SET_VELCTRL, NULL);
//	  }

	send_mess(card, READ_ONTARGET, PIE517_axis[signal]);
	if (recv_mess(card, buff, 1) && sscanf(buff, "%d", &ontarget_status))
	  {
	    send_mess(card, READ_OVERFLOW, PIE517_axis[signal]);
	    if (recv_mess(card, buff, 1) && sscanf(buff, "%d", &overflow_status))
	      {
		send_mess(card, READ_SERVO, PIE517_axis[signal]);
		if (recv_mess(card, buff, 1) && sscanf(buff, "%d", &servo_status))
		  {
		    send_mess(card, READ_POS, PIE517_axis[signal]);
		    if (recv_mess(card, buff, 1))
		      {
			motorData = NINT(atof(buff) / cntrl->drive_resolution[signal]);
			readOK = true;
		      }
		  }
	      }
	  }
//      }

    if (readOK)
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


    /* Always DONE if torque disabled */
    status.Bits.RA_DONE = (ontarget_status) ? 1 : 0;
    status.Bits.RA_HOME = status.Bits.RA_DONE;

    status.Bits.EA_POSITION = (servo_status) ? 1 : 0;  /* Torgue disabled flag */

    ls_active = plusLS = minusLS = false;

    /* LS status may be true but servo is not within position error - keep updating */
    /* No Limit switches but if the Servo Controller overflows indicate with a + LS */
    if (status.Bits.RA_DONE)
	plusLS  = overflow_status ? true : false;

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

    status.Bits.RA_PROBLEM  = 0;

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
	send_mess(card, buff, NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the PIE517 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, const char *com, const char *name)
{
    char local_buff[MAX_MSG_SIZE];
    char *pbuff;
    struct PIE517controller *cntrl;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
	errlogMessage("drvPIE517.cc:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (comsize == 0)  /* Normal exit on empty input message. */
	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvPIE517.cc:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    local_buff[0] = 0;    /* Terminate local buffer. */

    if (name == NULL)
	strcat(local_buff, com);    /* Make a local copy of the string. */
    else
    {
	strcpy(local_buff, com);
	pbuff = strchr(local_buff, '#');
        if (pbuff != NULL)
	  *pbuff = *name;
	else
	  Debug(1, "send_mess(): NAME ERROR: message = %s\n", local_buff);	  
    }

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct PIE517controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
			    COMM_TIMEOUT, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the PIE517 board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct PIE517controller *cntrl;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;
    char *pos;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct PIE517controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	pasynOctetSyncIO->flush(cntrl->pasynUser);
    else
	status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
					COMM_TIMEOUT, &nread, &eomReason);
    pos=strchr(com,'=');
    if(pos != NULL)
        strcpy(com,&pos[1]);

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
/* PIE517Setup()                                     */
/*****************************************************/
RTN_STATUS
PIE517Setup(int num_cards,  /* maximum number of controllers in system.  */
	    int scan_rate)  /* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PIE517_NUM_CARDS)
	PIE517_num_cards = PIE517_NUM_CARDS;
    else
	PIE517_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

    /* 
     * Allocate space for motor_state structures.  Note this must be done
     * before PIE517Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(PIE517_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PIE517_num_cards; itera++)
	motor_state[itera] = NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PIE517Config()                                    */
/*****************************************************/
RTN_STATUS
PIE517Config(int card,	     /* card being configured */
	     const char *name,	 /* asyn port name */
	     int addr)		 /* asyn address (GPIB) */
{
    struct PIE517controller *cntrl;

    if (card < 0 || card >= PIE517_num_cards)
	return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIE517controller));
    cntrl = (struct PIE517controller *) motor_state[card]->DevicePrivate;

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
    struct PIE517controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE], *pbuff;
    int total_axis;
    int status;
    int version;
    bool online; 
    asynStatus success_rtn;
    static const char output_terminator[] = EOL_E517;
    static const char  input_terminator[] = EOL_E517;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIE517_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PIE517_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->ident[0] = 0;	/* No controller identification message. */
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct PIE517controller *) brdptr->DevicePrivate;

        status = version = 0;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
						&cntrl->pasynUser, NULL);
	if (success_rtn == asynSuccess)
	{
	    //int retry = 0;

	    pasynOctetSyncIO->setOutputEos(cntrl->pasynUser, output_terminator,
					   strlen(output_terminator));
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser, input_terminator,
					  strlen(input_terminator));

	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    pasynOctetSyncIO->flush(cntrl->pasynUser);

	    /* Assure that Controller is ONLINE */
	    online = true;
	    //do
	    //{
	    //  online = false;
	    //  /* Set Controller to ONLINE mode */
	    //  send_mess(card_index, SET_ONLINE, NULL);
	    //  send_mess(card_index, READ_ONLINE, NULL);
	    //  if ((status = recv_mess(card_index, buff, 1)))
		//online = (atoi(buff)==1) ? true : false;
	    //  else
		//retry++;
	    //} while (online == false && retry < 3);

	    //send_mess(card_index, GET_IDENT, NULL);
	    //status = recv_mess(card_index, buff, 1);
	    
	    /* Parse out E517 revision (2 decimal places) and convert to int */
	    if ((pbuff = strchr(buff, 'V')))
	      version = NINT(atof(pbuff+1) * 100);
	    else
	      version = 0;
	}

	if (success_rtn == asynSuccess && online == true)
	{
	    strcpy(brdptr->ident, buff);
	    brdptr->localaddr = NULL;
	    brdptr->motor_in_motion = 0;

	    /* Check for E517 versions that need the status word shifted up 8 bits */
	    /*if (version >= 311)
	      cntrl->versionSupport = true;
	    else
	      cntrl->versionSupport = false;*/

            /* Determine # of axes. Request stage name.  See if it responds */
	    for (total_axis = 0; total_axis < MAX_AXES; total_axis++)
	    {
			send_mess(card_index, READ_POS, PIE517_axis[total_axis]);
			status = recv_mess(card_index, buff, 1);
			if (!status)
		    	break;
	    }
	    brdptr->total_axis = total_axis;

	    /* Turn ON velocity control mode  - All axis */
	    //send_mess(card_index, SET_VELCTRL, NULL);

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* PIE517 has DC motor support only */
		motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = NO;
		motor_info->status.Bits.GAIN_SUPPORT = 1;

		cntrl->drive_resolution[motor_index] = POS_RES;

		set_status(card_index, motor_index);  /* Read status of each motor */
	    }
	}
	else
	    motor_state[card_index] = NULL;
    }

    any_motor_in_motion = 0;

    mess_queue.head = NULL;
    mess_queue.tail = NULL;

    free_list.head = NULL;
    free_list.tail = NULL;

    epicsThreadCreate((char *) "PIE517_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}


