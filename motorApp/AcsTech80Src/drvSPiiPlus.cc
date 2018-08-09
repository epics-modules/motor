/*
FILENAME...	drvSPiiPlus.cc
USAGE...	Motor record driver level support for ACS Tech80 
                SPiiPlus


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
 * .01 04-08-05 jps initialized from drvSPiiPlus.cc
 * .02 09-29-10 rls Added req'd initialization of motor_info->motor_motion to
 *                  NULL in motor_init().
 */


#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <drvSup.h>
#include <errlog.h>
#include <stdlib.h>
#include "motor.h"
#include "ACSTech80Register.h"
#include "drvSPiiPlus.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"


#define STOP_ALL         "halt all"
#define MOTOR_ON         "enable(%d)"

#define GET_IDENT       "?VR"
#define SKIP_THIS       "#"

#define ACS_EOS          "\r" /* End-of-string */

#define SPiiPlus_NUM_CARDS	8
#define BUFF_SIZE 120       /* Maximum length of string to/from SPiiPlus */

#define TIMEOUT	2.0	/* Command timeout in sec. */

/*----------------debugging-----------------*/
volatile int drvSPiiPlusdebug = 0;
extern "C" {epicsExportAddress(int, drvSPiiPlusdebug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvSPiiPlusdebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int SPiiPlus_num_cards = 0;

static char *ACSPL_axis[] = {"X", "Y", "Z", "T", "A", "B", "C", "D"};

// Array dimensions are defined in drvSPiiPlusy.h
// First dimension order must match (enum)CMND_MODES
// Second dimension (command list) order must match (enum)QUERY_TYPES 
static char *queryCmnds[MODE_CNT][QUERY_CNT] = {
  {QSTATUS_CMND,QFAULT_CMND,QPOS_CMND,QEA_POS_CMND,QVEL_CMND,QHOME_CMND,QDONE_CMND},
  {QSTATUS_CMND,QFAULT_CMND,QPOS_CMND,QEA_POS_KIN_CMND,QVEL_CMND,SKIP_THIS,SKIP_THIS},
  {QSTATUS_CMND,QFAULT_CMND,QPOS_CMND,QEA_POS_CMND,QVEL_CMND,SKIP_THIS,SKIP_THIS}
  };

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvSPiiPlusReadbackDelay = 0.;


/*----------------functions-----------------*/
static int recv_mess(int card, char *com, int flag);
static RTN_STATUS send_mess(int card, char const *, char *name);
static int send_recv_mess(int card, char const *send_com, char *recv_com);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table SPiiPlus_access =
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

struct drvSPiiPlus_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvSPiiPlus = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvSPiiPlus);}

static struct thread_args targs = {SCAN_RATE, &SPiiPlus_access, 0.010};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (SPiiPlus_num_cards <=0)
	printf("    No SPiiPlus controllers configured.\n");
    else
    {
	for (card = 0; card < SPiiPlus_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    SPiiPlus controller %d connection failed.\n", card);
	    else
	    {
		struct SPiiPlusController *cntrl;

		cntrl = (struct SPiiPlusController *) brdptr->DevicePrivate;
	    	printf("    SPiiPlus controller %d, port=%s, address=%d, id: %s \n", 
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
    if (SPiiPlus_num_cards <= 0)
    {
	Debug(1, "init(): SPiiPlus driver disabled. SPiiPlusSetup() missing from startup script.\n");
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
    struct SPiiPlusController *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    char send_buff[80];
    char **cmndList;
    int cmndID;
    int flags;
    double vel;
    MOTOR_STATUS mstat;
    MOTOR_FAULTS mfault;
    int rtn_state;
    int recvCnt;
    int opReq;
    long motorData;
    bool homing;
    bool plusdir, ls_active = false;
    bool cmndErr;
    msta_field status;

    cntrl = (struct SPiiPlusController *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    status.All = motor_info->status.All;


    // Get correct list of command for controller interface mode
    cmndList = queryCmnds[cntrl->cmndMode];

    for (cmndID=0, cmndErr=false; cmndID < QUERY_CNT && !cmndErr; cmndID++, cmndList++)
      {
	// Check for SKIP_THIS flag 
	if (*cmndList[0] == '#')
	    cntrl->recv_string[cmndID][0] = 0;
	else
	  {
	    if (cmndID == QEA_POS && cntrl->cmndMode == CONNECT)
	      sprintf(send_buff, *cmndList, ACSPL_axis[signal]);
	    else
	      sprintf(send_buff, *cmndList, signal);
	  
	    recvCnt = send_recv_mess(card, send_buff, cntrl->recv_string[cmndID]);

	    // Check for TIMEOUT or Controller Error Reply 
	    if (recvCnt <= 0 || strchr(cntrl->recv_string[cmndID], '?'))
	      cmndErr = true;
	  }
      }

    // Did we get all the way thru the command list?
    if (!cmndErr)
      cntrl->status = NORMAL;
    else
      {
	if (cntrl->status == NORMAL)
	  cntrl->status = RETRY;
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
     * Parse the status/fault strings
     */
    flags = atoi(cntrl->recv_string[QSTATUS]);
    mstat.All = flags;
    mfault.All = atoi(cntrl->recv_string[QFAULT]);
    Debug(5, "set_status(): status byte = %x, fault int = %x\n", mstat.All, mfault.All);

    vel = atof(cntrl->recv_string[QVEL]);
    status.Bits.RA_DIRECTION = (vel >= 0) ? 1 : 0;

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    if (cntrl->cmndMode == BUFFER)
	status.Bits.RA_DONE = (atoi(cntrl->recv_string[QDONE])) ? 1 : 0;
    else
      {
	// if (mstat.Bits.inmotion == false && mstat.Bits.inposition == true)
	if (mstat.Bits.inposition == true)
	  status.Bits.RA_DONE = 1;
	else
	  status.Bits.RA_DONE = 0;
      }

    status.Bits.RA_MOVING = (mstat.Bits.inmotion == true) ? 1 : 0;

    status.Bits.RA_HOME = status.Bits.RA_DONE;
    
    if (cntrl->cmndMode == BUFFER)
      {
	opReq = atoi(cntrl->recv_string[QHOME]);
	homing = (opReq == OP_HOME_F || opReq == OP_HOME_R) ? true : false;
      }
    else
      homing = false;

    /* Set Travel limit switch status bits. */
    if ((mfault.Bits.rl == false && mfault.Bits.srl == false) ||  homing)
	status.Bits.RA_PLUS_LS = 0;
    else
    {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	    ls_active = true;
    }

    if ((mfault.Bits.ll == false && mfault.Bits.sll == false) ||  homing)
	status.Bits.RA_MINUS_LS = 0;
    else
    {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	    ls_active = true;
    }

    /* Position maintence enabled */
    status.Bits.EA_POSITION = (mstat.Bits.enabled) ? 1: 0; 

    /* encoder status */
    status.Bits.EA_SLIP 	= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;

    /* 
     * Parse motor position
     */
    motorData = (long)atof(cntrl->recv_string[QPOS]);

    // printf("motorData=%ld, last_position=%ld, count=%d\n",motorData, motor_info->position, motor_info->no_motion_count);

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->position = motorData;
	motor_info->no_motion_count = 0;
    }

    // Always update encoder position - for non-motion monitoring
    if (motor_state[card]->motor_info[signal].encoder_present == YES)
      motor_info->encoder_position = (long)(atof(cntrl->recv_string[QEA_POS]));
    else
      motor_info->encoder_position = 0;



    status.Bits.RA_PROBLEM = 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = (int)vel;

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
/* send_receive a message to the SPiiPlus board	     */
/* send_recv_mess()		                     */
/*****************************************************/
static int send_recv_mess(int card, char const *send_com, char *recv_com)
{
    struct SPiiPlusController *cntrl;
    int size;
    size_t nwrite;
    size_t nread = 0;
    double timeout = 0.;
    asynStatus status;
    int eomReason;


    size = strlen(send_com);
    recv_com[0] = '\0';

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvSPiiPlus.c:send_recv_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvSPiiPlus.c:send_recv_mess() - invalid card #%d\n", card);
	return(ERROR);
    }
    Debug(2, "send_recv_mess(): message = %s\n", send_com);

    cntrl = (struct SPiiPlusController *) motor_state[card]->DevicePrivate;

    timeout = TIMEOUT;
    /* flush any junk at input port - should not be any data available */
    pasynOctetSyncIO->flush(cntrl->pasynUser);

    /* Perform atomic write/read operation  */
    status = pasynOctetSyncIO->writeRead(cntrl->pasynUser, send_com, strlen(send_com), 
                                         recv_com, ACS_MSG_SIZE,
                                         TIMEOUT, &nwrite, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
	recv_com[0] = '\0';
	nread = 0;
    }

    Debug(2, "send_recv_mess(): recv message = \"%s\"\n", recv_com);

    return(nread);
}
/*****************************************************/
/* send a message to the SPiiPlus board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    struct SPiiPlusController *cntrl;
    int size;
    size_t nwrite;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvSPiiPlus.c:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvSPiiPlus.c:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (name != NULL)
    {
	errlogPrintf("drvSPiiPlus.c:send_mess() - invalid argument = %s\n", name);
	return(ERROR);
    }

    Debug(2, "send_mess(): message = %s\n", com);

    cntrl = (struct SPiiPlusController *) motor_state[card]->DevicePrivate;

    /* flush any junk at input port - should not be any data available */
    pasynOctetSyncIO->flush(cntrl->pasynUser);

    pasynOctetSyncIO->write(cntrl->pasynUser, com, strlen(com), 
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
    struct SPiiPlusController *cntrl;
    double timeout = 0.;
    size_t nread = 0;
    asynStatus status;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct SPiiPlusController *) motor_state[card]->DevicePrivate;

    timeout	= TIMEOUT;

    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

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
/* SPiiPlusSetup()                                     */
/*****************************************************/
RTN_STATUS
SPiiPlusSetup(int num_cards,	/* maximum number of controllers in system.  */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > SPiiPlus_NUM_CARDS)
	SPiiPlus_num_cards = SPiiPlus_NUM_CARDS;
    else
	SPiiPlus_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before SPiiPlusConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(SPiiPlus_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < SPiiPlus_num_cards; itera++)
	motor_state[itera] = NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* SPiiPlusConfig()                                  */
/*****************************************************/

RTN_STATUS
SPiiPlusConfig(int card,		/* card being configured */
	       const char *name,        /* asyn port name */
	       const char *modeStr)    /* command mode [BUFfer/DIRect] */
{
    struct SPiiPlusController *cntrl;
    int modeIdx;
    char modeCas[4];

    if (card < 0 || card >= SPiiPlus_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct SPiiPlusController));
    cntrl = (struct SPiiPlusController *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);

    // Set controller command interface mode - BUFFER is the default 
    // Assure upper case argument - only check first 3 letters 
    modeCas[0]= 0;
    if (modeStr != NULL) {
      for (modeIdx=0; modeIdx < 3; modeIdx++)
	modeCas[modeIdx] = toupper(modeStr[modeIdx]);
      modeCas[3]= 0;				
    }
				
    if (!strncmp(modeCas, DIRECT_STR,3))
      cntrl->cmndMode = DIRECT;
    else if (!strncmp(modeCas, CONNECT_STR,3))
      cntrl->cmndMode = CONNECT;
    else
      cntrl->cmndMode = BUFFER;

    printf("SPiiPlus config mode = %d\n", (int)(cntrl->cmndMode));

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
    struct SPiiPlusController *cntrl;
    int card_index, motor_index;
    // char axis_pos[BUFF_SIZE];
    char buff[BUFF_SIZE];
    char send_buff[30];
    // char *tok_save, *pos_ptr;
    int total_axis = 0;
    int status;
    bool foundAxis;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (SPiiPlus_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < SPiiPlus_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct SPiiPlusController *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 
                                            cntrl->asyn_address, &cntrl->pasynUser, NULL);

	if (success_rtn == asynSuccess)
	{
	    int retry = 0;

	      /* Set command End-of-string */
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser,ACS_EOS,strlen(ACS_EOS));
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser,ACS_EOS,strlen(ACS_EOS));
	    
	    /* Send a message to the board, see if it exists */
	    do
	    {
	      status = send_recv_mess(card_index, GET_IDENT, buff);
		retry++;
		/* Return value is length of response string */
	    } while(status == 0 && !strchr(buff, '?') && retry < 3);
	}

	if (success_rtn == asynSuccess && status > 0)
	{
	    brdptr->localaddr = NULL;
	    brdptr->motor_in_motion = 0;
    	    status = send_recv_mess(card_index, STOP_ALL, buff);   /* Stop all motors */
	    status = send_recv_mess(card_index, GET_IDENT, buff);  /* Read controller ID string */
	    strcpy(brdptr->ident, buff);  /* Save Version  */


	    /* The return string will tell us how many axes this controller has */
	    for (total_axis=0, foundAxis=true; foundAxis; total_axis++)
	      {
		sprintf(send_buff, QPOS_CMND, total_axis);
		status = send_recv_mess(card_index, send_buff, buff);
		if (status <= 0 || strchr(buff, '?'))
		  foundAxis = false;
	      }

	    
	    brdptr->total_axis = --total_axis;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		motor_info->motor_motion = NULL;

		/* Encoder Enable */
                motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = YES;
		motor_info->status.Bits.GAIN_SUPPORT = 1;

                /* Determine low limit */
                // sprintf(send_buff, "?SLLIMIT(%d)", motor_index);
       	        // status = send_recv_mess(card_index, send_buff, buff);  
                // motor_info->low_limit = atof(buff);

                /* Determine high limit */
                // sprintf(send_buff, "?SRLIMIT(%d)", motor_index);
       	        // status = send_recv_mess(card_index, send_buff, buff);  
                // motor_info->high_limit = atof(buff);

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

    // epicsThreadCreate((char *) "SPiiPlus_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);
    epicsThreadCreate((char *) "SPiiPlus_motor", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);


    return(OK);
}

