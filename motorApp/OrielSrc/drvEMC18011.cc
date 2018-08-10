/*
FILENAME...	drvEMC18011.cc
USAGE...	Motor record driver level support for Spectra-Physics
                Encoder Mike Controller (Model: 18011)

*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/97
 *	Current Author: J. Sullivan
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
 * .01 04-08-05 jps initialized from drvMM4000.cc
 */


#include <string.h>
#include <ctype.h>   /* isascii functions */
#include <math.h>
#include <stdio.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <errlog.h>
#include <stdlib.h>
#include "motor.h"
#include "OrielRegister.h"
#include "drvEMC18011.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define CMD_STATUS      "Z"
#define CMD_POS         "A"
#define CMD_STOP        "S"
#define CMD_LOCAL       "L"
#define CMD_REMOTE      "R"
#define CMD_SELECT      "M"

#define RTN_REMOTE     "ON LINE"
#define RTN_READY      "RE"   // READY
#define RTN_OVERLOAD   "OV"   // OVERLOAD


#define EMC18011_NUM_CARDS	16
#define BUFF_SIZE 120       /* Maximum length of string to/from EMC18011 */

#define TIMEOUT	1.0	/* Command timeout in sec. */

/* Delay after START_MOTION before a status update is possible */
#define MOTION_DELAY 0.1

/*----------------debugging-----------------*/
volatile int drvEMC18011debug = 0;
extern "C" {epicsExportAddress(int, drvEMC18011debug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvEMC18011debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int EMC18011_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvEMC18011ReadbackDelay = 0.;


/*----------------functions-----------------*/
static int recv_mess(int card, char *com, int flag);
static int recv_mess(int card, char *com, int flag, int recv_len);
static RTN_STATUS send_mess(int card, const char *, const char *name);
static int send_recv_mess(int card, const char *send_com, char *recv_com);
static int send_recv_mess(int card, const char *send_com, char *recv_com, 
			  int recv_len);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table EMC18011_access =
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

struct drvEMC18011_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvEMC18011 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvEMC18011);}

static struct thread_args targs = {SCAN_RATE, &EMC18011_access, MOTION_DELAY};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (EMC18011_num_cards <=0)
      printf("    No EMC18011 controllers configured.\n");
    else
    {
      for (card = 0; card < EMC18011_num_cards; card++)
	{
	  struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    EMC18011 controller %d connection failed.\n", card);
	    else
	    {
		struct EMC18011Controller *cntrl;

		cntrl = (struct EMC18011Controller *) brdptr->DevicePrivate;
	    	printf("    EMC18011 controller %d, port=%s, address=%d, id: %s \n", 
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
    if (EMC18011_num_cards <= 0)
    {
	Debug(1, "init(): EMC18011 driver disabled. EMC18011Setup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}

/*********************************************************
 * Read the status and position of all motors on a card
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/
// static void start_status(int card)
//{
//}

/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
  struct EMC18011Controller *cntrl;
  struct mess_node *nodeptr;
  register struct mess_info *motor_info;
  char send_buff[80];
  char Zstatus;
  char *recvStr;
  char *brkptr, *endptr;
  int rtn_state;
  int recvCnt;
  int motorData;
  int motor;
  double datad;
  bool recvRetry;
  bool plusdir, ls_active = false;
  msta_field status;

  cntrl = (struct EMC18011Controller *) motor_state[card]->DevicePrivate;
  motor_info = &(motor_state[card]->motor_info[signal]);
  status.All = motor_info->status.All;
  motor = signal+1;


  recvRetry = true;

  /* Initialize motorData in-order to detect motor not moving */
  motorData = motor_info->position;

  /* Status updates only available on selected motor */
  if (signal == cntrl->motorSelect)
    {
      recvStr = cntrl->recv_string[0];
      /* Get Zstatus (one character motion indicator) */
      recvCnt = send_recv_mess(card, CMD_STATUS, recvStr, 1);
      if (recvCnt == 1)
	{
	  /* Test for valid reply */
	  Zstatus = *recvStr;
	  if (Zstatus >= Z_STOPPED && Zstatus <= Z_LSUP)
	    recvRetry = false;
	  else
	    Zstatus = Z_RUNUP;

	  /* Update position after motion has stopped */
	  /* NOTE: This controller does not provide reliable position 
	   *       feedback during motion (BUG?)  */
	  if (Zstatus != Z_RUNUP && Zstatus != Z_RUNDOWN)
	    {
	      recvCnt = send_recv_mess(card, CMD_POS, recvStr);
	      if (recvCnt > 0)
		{
		  datad = strtod(recvStr, &endptr);
		  if (brkptr != endptr)
		    {
		      motorData = NINT(datad / cntrl->drive_resolution);
		      /* Release motor */
		      cntrl->motorSelect = -1;
		      cntrl->motorLock->signal();
		      recvRetry = false;
		    }
		  else
		    /* Don't indicate done until we get a valid position */
		    Zstatus = Z_RUNUP;
		}
	    }
	}
    }
  else
    {
      /* Not the selected motor - no new information */
      recvRetry = false;
      Zstatus = Z_STOPPED;
      epicsThreadSleep(0.1);  /* Pretend we did something - in-case of record retry loop */
    }

  

  /* Check for normal look termination - all queries successful */
  if (recvRetry == false)
    cntrl->status = NORMAL;
  else
    {
      if (cntrl->status == NORMAL)
	{
	  epicsThreadSleep(MOTION_DELAY);
	  cntrl->status = RETRY;
	}
      else
	cntrl->status = COMM_ERR;
    }

    if (cntrl->status != NORMAL)
    {
	if (cntrl->status == COMM_ERR)
	{
	    status.Bits.CNTRL_COMM_ERR = 1;
	    status.Bits.RA_PROBLEM     = 1;
	    rtn_state = 1;
	    goto exit;
	}
	else
	{
	    rtn_state = 0;
	    goto exit;
	}
    }
    else
	status.Bits.CNTRL_COMM_ERR = 0;

    nodeptr = motor_info->motor_motion;

    /* 
     * Parse the status/fault string
     */

    Debug(5, "set_status(): status  = %c\n", Zstatus);

    plusdir = (Zstatus == Z_STOPPED || Zstatus == Z_RUNUP || Zstatus == Z_LSUP) ? true : false;

    status.Bits.RA_DIRECTION = plusdir ? 1 : 0;

    status.Bits.RA_HOME = 0;

    status.Bits.RA_DONE = (Zstatus == Z_STOPPED) ? 1 : 0;

    /* Set Travel limit switch status bits. */
    status.Bits.RA_PLUS_LS = (Zstatus == Z_LSUP) ? 1 : 0;
    status.Bits.RA_MINUS_LS = (Zstatus == Z_LSDOWN) ? 1 : 0;

    ls_active = (status.Bits.RA_PLUS_LS || status.Bits.RA_MINUS_LS) ? true : false;


    /* encoder status */
    status.Bits.EA_POSITION =  0; 
    status.Bits.EA_SLIP 	= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;

    /* Disable no motion test - this driver does not update position  */
    // if (motorData == motor_info->position)
    if (false)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->position = motorData;
        motor_info->encoder_position = 0;
	motor_info->no_motion_count = 0;
    }

    status.Bits.RA_PROBLEM = 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */
    motor_info->velocity = 0;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
        strncpy(send_buff, nodeptr->postmsgptr, 80);
	send_mess(card, send_buff, NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send_receive a message to the EMC18011 board	     */
/* send_recv_mess()		                     */
/*****************************************************/
static int send_recv_mess(int card, const char *send_com, char *recv_com)
{
  return(send_recv_mess(card, send_com, recv_com, 0));
}

static int send_recv_mess(int card, const char *send_com, char *recv_com,
			  int recv_len)
{
    struct EMC18011Controller *cntrl;
    int sendLen, recvLen;
    size_t nwrite;
    size_t nread = 0;
    double timeout = 0.;
    asynStatus status;
    int eomReason;


    sendLen = strlen(send_com);
    /* recv_len argument used to request specific number of response characters */
    recvLen = recv_len ? recv_len : EMC18011_MSG_SIZE;

    recv_com[0] = '\0';

    if (sendLen > MAX_MSG_SIZE)
    {
	errlogMessage("drvEMC18011.c:send_recv_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (sendLen == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvEMC18011.c:send_recv_mess() - invalid card #%d\n", card);
	return(ERROR);
    }
    Debug(2, "send_recv_mess(): message = %s\n", send_com);

    cntrl = (struct EMC18011Controller *) motor_state[card]->DevicePrivate;

    timeout = TIMEOUT;
    /* flush any junk at input port - should not be any data available */
    // pasynOctetSyncIO->flush(cntrl->pasynUser); 

    /* Perform atomic write/read operation  */
    status = pasynOctetSyncIO->writeRead(cntrl->pasynUser, send_com, sendLen, 
                                         recv_com, recvLen ,
                                         TIMEOUT, &nwrite, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
	recv_com[0] = '\0';
	nread = 0;
	Debug(1, "send_recv_mess(): TIMEOUT sending = '%s'\n", send_com);
    }

    Debug(2, "send_recv_mess(): recv message = '%s'\n", recv_com);

    return(nread);
}
/*****************************************************/
/* send a message to the EMC18011 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, const char *com, const char *name)
{
    struct EMC18011Controller *cntrl;
    int size;
    size_t nwrite;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvEMC18011.c:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvEMC18011.c:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (name != NULL)
    {
	errlogPrintf("drvEMC18011.c:send_mess() - invalid argument = %s\n", name);
	return(ERROR);
    }

    Debug(2, "send_mess(): message = %s\n", com);

    cntrl = (struct EMC18011Controller *) motor_state[card]->DevicePrivate;

    /* flush any junk at input port - should not be any data available */
    pasynOctetSyncIO->flush(cntrl->pasynUser);

    pasynOctetSyncIO->write(cntrl->pasynUser, com, strlen(com), 
                            TIMEOUT, &nwrite);

    return(OK);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int flag)
 *             recv_mess(int card, char *com, int flag, int recv_len)
 *
 * INPUT ARGUMENTS...
 *	card - controller card # (0,1,...).
 *	*com - caller's response buffer.
 *	flag	| FLUSH  = this flag is ignored - the receive buffer is flushed
 *                         on every write (see write_mess())
 *      recvLen - receive length (optional) 
 * LOGIC...
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int flag)
{
  return(recv_mess(card, com, flag, 0));
}

static int recv_mess(int card, char *com, int flag, int recv_len)
{
    struct EMC18011Controller *cntrl;
    double timeout = 0.;
    size_t nread = 0;
    asynStatus status;
    int eomReason;
    int recvLen;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct EMC18011Controller *) motor_state[card]->DevicePrivate;

    timeout	= TIMEOUT;

    /* Check if specific number of input characters requested  */
    recvLen = (recv_len ? recv_len : BUFF_SIZE);

    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, recvLen,
                                    timeout, &nread, &eomReason);

    if (nread > 0)
      Debug(2, "recv_mess(): message = '%s'\n", com);

    if ((status != asynSuccess) || (nread <= 0))
    {
	com[0] = '\0';
	nread = 0;
        Debug(1, "recv_mess(): TIMEOUT \n");
    }

    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* EMC18011Setup()                                     */
/*****************************************************/
RTN_STATUS
EMC18011Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > EMC18011_NUM_CARDS)
	EMC18011_num_cards = EMC18011_NUM_CARDS;
    else
	EMC18011_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before EMC18011Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(EMC18011_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < EMC18011_num_cards; itera++)
	motor_state[itera] = NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* EMC18011Config()                                    */
/*****************************************************/
RTN_STATUS
EMC18011Config(int card,		/* card being configured */
	       const char *name)   /* asyn port name */
{
    struct EMC18011Controller *cntrl;

    if (card < 0 || card >= EMC18011_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct EMC18011Controller));
    cntrl = (struct EMC18011Controller *) motor_state[card]->DevicePrivate;

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
    struct EMC18011Controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    char send_buff[20];
    int total_axis = 0;
    int recvCnt, retryCnt;
    bool cardFound = false;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (EMC18011_num_cards <= 0)
	return(ERROR);


    for (card_index = 0; card_index < EMC18011_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct EMC18011Controller *) brdptr->DevicePrivate;

	cntrl->motorLock = new epicsEvent;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 
                                            cntrl->asyn_address, &cntrl->pasynUser, NULL);

	if (success_rtn == asynSuccess)
	{

	      /* Set command End-of-string */
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser,
					  EMC18011_IN_EOS,strlen(EMC18011_IN_EOS));
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser,
					   EMC18011_OUT_EOS,strlen(EMC18011_OUT_EOS));
	    
	    /* Send a message to the board, see if it exists */
	    retryCnt = 0;

	    do
	    {
	      /* Switch to remote - test response */
	      recvCnt = send_recv_mess(card_index, CMD_LOCAL, buff);
	      recvCnt = recv_mess(card_index, buff, 0);   // Could be a second message - get it 
	      recvCnt = send_recv_mess(card_index, CMD_REMOTE, buff);
	      /* Check for valid response -- if not retry */
	      if ((recvCnt > 0) && strstr(buff, RTN_REMOTE))
		cardFound = true;
	    } while(!cardFound  && ++retryCnt < 3);
	}

	if (cardFound)
	{
	
	    strcpy(brdptr->ident, "Oriel Encoder Mike 18011");  /* Save Controller ID  */


	    brdptr->localaddr = NULL;
	    brdptr->motor_in_motion = 0;

	    /* Make sure that motion is stopped - cannot change motor selection */
            recvCnt = send_recv_mess(card_index, CMD_STOP, buff);
	    
	    /* The find how many axes this controller has - fixed number*/
            total_axis = EMC18011_MAX_MOTORS;
	    
	    brdptr->total_axis = total_axis;
	    cntrl->drive_resolution = EMC18011_RESOLUTION;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		brdptr->motor_info[motor_index].motor_motion = NULL;


		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;


		/* NO Encoder support - internal closed loop controller */
	        motor_info->encoder_present = NO;
	        motor_info->status.Bits.EA_PRESENT = 0;
	        motor_info->pid_present = NO;
	        motor_info->status.Bits.GAIN_SUPPORT = 0;

		/* Select motor - for status update */
		sprintf(send_buff, "%s%d", CMD_SELECT, motor_index+1);
		recvCnt = send_recv_mess(card_index, send_buff, buff);		
		cntrl->motorSelect = motor_index;

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

    // epicsThreadCreate((char *) "EMC18011_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);
    epicsThreadCreate((char *) "EMC18011_motor", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);


    return(OK);
}

