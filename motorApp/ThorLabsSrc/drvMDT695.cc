/*
FILENAME...	drvMDT695.cc
USAGE...	Motor record driver level support for ThorLabs 
                Piezo Control Module (Model: MDT695)
		Compatable with MDT694, MDT693


*/

/*
 *      Original Author: Joe Sullivan
 *      Date: 9/27/2006
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
 * .01 09-27-06 jps initialized from drvEMC18011.cc
 */


#include <string.h>
#include <ctype.h>   /* isascii functions */
#include <math.h>
#include <stdio.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <stdlib.h>
#include <errlog.h>
#include "motor.h"
#include "ThorLabsRegister.h"
#include "drvMDT695.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define CMD_ID       "I"
#define CMD_DEV      "D"
#define RESP_DEV     "d"  /* Response if ECHO is ON */
#define CMD_ECHO     "E"
#define CMD_POS       "#R?"

#define MDT695_NUM_CARDS	16
#define BUFF_SIZE 120       /* Maximum length of string to/from MDT695 */

#define TIMEOUT	2.0	/* Command timeout in sec. */

/* Delay after START_MOTION before a status update is possible */
#define MOTION_DELAY 0.1

/*----------------debugging-----------------*/
volatile int drvMDT695debug = 0;
extern "C" {epicsExportAddress(int, drvMDT695debug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvMDT695debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int MDT695_num_cards = 0;
static const char *MDT694_axis[] = {"X", "Y", "Z"};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvMDT695ReadbackDelay = 0.;


/*----------------functions-----------------*/
static int recv_mess(int card, char *com, int flag);
static RTN_STATUS send_mess(int card, const char *, const char *name);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MDT695_access =
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
    MDT694_axis
};

struct drvMDT695_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvMDT695 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvMDT695);}

static struct thread_args targs = {SCAN_RATE, &MDT695_access, MOTION_DELAY};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (MDT695_num_cards <=0)
      printf("    No MDT695 controllers configured.\n");
    else
    {
      for (card = 0; card < MDT695_num_cards; card++)
	{
	  struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    MDT695 controller %d connection failed.\n", card);
	    else
	    {
		struct MDT695Controller *cntrl;

		cntrl = (struct MDT695Controller *) brdptr->DevicePrivate;
	    	printf("    MDT695 controller %d, port=%s, address=%d, id: %s \n", 
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
    if (MDT695_num_cards <= 0)
    {
	Debug(1, "init(): MDT695 driver disabled. MDT695Setup() missing from startup script.\n");
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
  struct MDT695Controller *cntrl;
  struct mess_node *nodeptr;
  register struct mess_info *motor_info;
  char send_buff[80];
  char *startptr, *endptr;
  int rtn_state;
  int recvCnt;
  int motorData;
  int motor;
  bool recvRetry;
  bool ls_active = false;
  msta_field status;

  cntrl = (struct MDT695Controller *) motor_state[card]->DevicePrivate;
  motor_info = &(motor_state[card]->motor_info[signal]);
  status.All = motor_info->status.All;
  motor = signal+1;


  recvRetry = true;

  startptr = cntrl->recv_string;
  send_mess(card, CMD_POS, MDT694_axis[signal]);
  if ((recvCnt = recv_mess(card, startptr, 0)) > 0)
    {
      double datad;

      /* Convert position and check for error */
      Debug(5, "set_status(): RecvStr = %s\n", &startptr[2]);
      datad = strtod(&startptr[2], &endptr);
      if (&startptr[2] != endptr)
	{
          Debug(5, "set_status(): motorData = %.2f\n", datad);
	  motorData = NINT(datad / cntrl->drive_resolution);
	  recvRetry = false;
	}
    }

  /* Check for normal look termination - all queries successful */
  if (recvRetry == false)
    cntrl->status = NORMAL;
  else
    {
      /* Try turning Echo OFF before next retry */
      recvCnt = recv_mess(card, startptr, 0);   /* Flush recv buffer */
      send_mess(card, CMD_ECHO, NULL);
      recvCnt = recv_mess(card, startptr, 0); 

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
     * There is no status information available - assume motion complete
     */

    Debug(5, "set_status(): status  = %s\n", "Done");
    status.Bits.RA_DONE = 1;
    status.Bits.RA_DIRECTION = 1;

    status.Bits.RA_HOME = 0;

    /* Set Travel limit switch status bits. */
    status.Bits.RA_PLUS_LS = 0;
    status.Bits.RA_MINUS_LS = 0;

    ls_active = (status.Bits.RA_PLUS_LS || status.Bits.RA_MINUS_LS) ? true : false;


    /* encoder status */
    status.Bits.EA_POSITION =  0; 
    status.Bits.EA_SLIP 	= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;

    if (motorData == motor_info->position)
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
/* send a message to the MDT695 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, const char *com, const char *name)
{
    struct MDT695Controller *cntrl;
    char local_buff[MAX_MSG_SIZE];
    char *pname;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
	errlogMessage("drvMDT695.c:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (comsize == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvMDT695.c:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    local_buff[0] = 0;    /* Terminate local buffer. */

    if (name == NULL)
	strcat(local_buff, com);    /* Make a local copy of the string. */
    else
    {
	strcpy(local_buff, com);
	// Look for name substitution 
	if ((pname = strchr(local_buff, '#')))
	    *pname = *name;
    }

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct MDT695Controller *) motor_state[card]->DevicePrivate;

    /* flush any junk at input port - should not be any data available */
    pasynOctetSyncIO->flush(cntrl->pasynUser);

    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(com), 
                            TIMEOUT, &nwrite);

    return(OK);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int flag)
 *
 * INPUT ARGUMENTS...
 *	card - controller card # (0,1,...).
 *	*com - caller's response buffer.
 *	flag	| FLUSH  = this flag is ignored - the receive buffer is flushed
 *                         on every write (see write_mess())
 * LOGIC...
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int flag)
{
    struct MDT695Controller *cntrl;
    double timeout = 0.;
    size_t nread = 0;
    asynStatus status;
    int eomReason;
    int rtnVal;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct MDT695Controller *) motor_state[card]->DevicePrivate;

    timeout	= TIMEOUT;

    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    if (nread > 0)
      Debug(2, "recv_mess(): message = '%s'\n", com);

    if (status != asynSuccess)
    {
	com[0] = '\0';
	rtnVal = -1;
        Debug(1, "recv_mess(): TIMEOUT \n");
    }
    else 
      rtnVal = (int)nread;

    return(rtnVal);
}


/*****************************************************/
/* Setup system configuration                        */
/* MDT695Setup()                                     */
/*****************************************************/
RTN_STATUS
MDT695Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > MDT695_NUM_CARDS)
	MDT695_num_cards = MDT695_NUM_CARDS;
    else
	MDT695_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before MDT695Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(MDT695_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < MDT695_num_cards; itera++)
	motor_state[itera] = NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* MDT695Config()                                    */
/*****************************************************/
RTN_STATUS
MDT695Config(int card,		/* card being configured */
	       const char *name)   /* asyn port name */
{
    struct MDT695Controller *cntrl;

    if (card < 0 || card >= MDT695_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MDT695Controller));
    cntrl = (struct MDT695Controller *) motor_state[card]->DevicePrivate;

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
    struct MDT695Controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int recvCnt, retryCnt;
    bool cardFound = false;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (MDT695_num_cards <= 0)
	return(ERROR);


    for (card_index = 0; card_index < MDT695_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct MDT695Controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 
                                            cntrl->asyn_address, &cntrl->pasynUser, NULL);

	if (success_rtn == asynSuccess)
	{

	      /* Set command End-of-string */
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser,
					  MDT695_IN_EOS,strlen(MDT695_IN_EOS));
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser,
					   MDT695_OUT_EOS,strlen(MDT695_OUT_EOS));
	    
	    /* Send a message to the board, see if it exists */
	    retryCnt = 0;

	    do
	    {
	      /* Read device type */
	      send_mess(card_index, CMD_DEV, NULL);
	      recvCnt = recv_mess(card_index, buff, 0); 
	      /* Check for valid response -- if not retry */
	      if ((recvCnt > 0) && strstr(buff, "MDT"))
		cardFound = true;
	      else
		{
		/* Try turning Echo OFF */
	        recvCnt = recv_mess(card_index, buff, 0);   /* Flush recv buffer */
		send_mess(card_index, CMD_ECHO, NULL);
		recvCnt = recv_mess(card_index, buff, 0); 
		}
	    } while(!cardFound  && ++retryCnt < 3);
	}

	if (cardFound)
	{

	    /* Check for 1 or 3 axis models */
	    if (strstr(buff, "694"))
	      total_axis = 1;
	    else
	      total_axis = 3;

	    /* Get controller ID info -- multi-line response */
	    *brdptr->ident = 0;
	    send_mess(card_index, CMD_ID, NULL);
	    do 
	      {
		recvCnt = recv_mess(card_index, buff, 0); 
		if (recvCnt > 0)
		  {
		    if (strstr(buff, "Model"))
		      strcpy(brdptr->ident, buff);
		    else if (strstr(buff, "Version"))
		      {
			strcat(brdptr->ident, ", ");
			strcat(brdptr->ident, buff);		  
		      }
		  }
	      } while(recvCnt >= 0);
		    

	    brdptr->localaddr = NULL;
	    brdptr->motor_in_motion = 0;

	    brdptr->total_axis = total_axis;
	    cntrl->drive_resolution = MDT695_RESOLUTION;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		motor_info->motor_motion = NULL;

		/* NO Encoder support - internal closed loop controller */
	        motor_info->encoder_present = NO;
	        motor_info->status.Bits.EA_PRESENT = 0;
	        motor_info->pid_present = NO;
	        motor_info->status.Bits.GAIN_SUPPORT = 0;

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

    // epicsThreadCreate((char *) "MDT695_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);
    epicsThreadCreate((char *) "MDT695_motor", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);


    return(OK);
}

