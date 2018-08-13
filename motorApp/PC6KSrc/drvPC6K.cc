/*
FILENAME...	drvPC6K.cc
USAGE...	Motor record driver level support for Parker Computmotor
                6K Series motor controllers

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
#include <stdlib.h>
#include <errlog.h>
#include "motor.h"
#include "ParkerRegister.h"
#include "drvPC6K.h"
#include "asynOctetSyncIO.h"
#include "asynCommonSyncIO.h"
#include "epicsExport.h"
#include "epicsExit.h"

#define CMD_STATUS      "TAS"
#define CMD_POS         "TPC"
#define CMD_EA_POS      "TPE"
#define CMD_VEL         "TVELA"
#define CMD_DRIVE       "DRIVE"
#define CMD_HIGHLS      "LSPOS"  /* Software travel limit */
#define CMD_LOWLS       "LSNEG"
#define CMD_ERES        "ERES"
#define CMD_DRES        "DRES"
#define CMD_SCLA        "SCLA1"
#define CMD_SCLV        "SCLV1"
#define CMD_SCLD        "SCLD1"
#define CMD_SCALE       "SCALE1"
#define CMD_AXSDEF      "AXSDEF"  /* Axis definition xxxx_xxxx , 0=stepper, 1=servo */
#define CMD_ECHO        "ECHO0"   /* Echo must be off */



#define STOP_ALL        "!K"
#define MOTOR_ON        "%dDRIVE1"
#define COMEXEC_ENA     "COMEXC1"  /* Continuous command mode ON */


#define GET_IDENT       "TREV"
#define REPLY_CHAR      '*'

#define PC6K_NUM_CARDS	16
#define BUFF_SIZE 120       /* Maximum length of string to/from PC6K */

#define TIMEOUT	1.0	/* Command timeout in sec. */

/* Delay after START_MOTION before a status update is possible
 *   *** Limitation of Parker 6K  controller ***
*/
#define MOTION_DELAY 0.05   

/*----------------debugging-----------------*/
volatile int drvPC6Kdebug = 0;
extern "C" {epicsExportAddress(int, drvPC6Kdebug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvPC6Kdebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int PC6K_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvPC6KReadbackDelay = 0.;

/* NOTICE !!!! Command order must match drvPC6K.h/PC6K_query_types !!!! */
static struct {
  const char *cmnd;
  int  cmndLen;
} queryOps[]= {{CMD_STATUS, 0}, {CMD_POS, 0}, {CMD_EA_POS, 0}, {CMD_VEL, 0}, {CMD_DRIVE, 0}};

#define QUERY_CNT PC6K_QUERY_CNT

/* Track open asyn ports - close on IOC exit */
#define MAX_SOCKETS        PC6K_NUM_CARDS+1
#define PORT_NAME_SIZE     100
#define ERROR_STRING_SIZE  100
#define DEFAULT_TIMEOUT    0.2

static int nextSocket = 0;

/* Pointer to the connection info for each socket 
   the asynUser structure is defined in asynDriver.h */
typedef struct {
    asynUser *pasynUser;
    asynUser *pasynUserCommon;
    double timeout;
    char errorString[ERROR_STRING_SIZE];
    bool connected;
} socketStruct;
static socketStruct socketStructs[MAX_SOCKETS];



/*----------------functions-----------------*/
static int recv_mess(int card, char *com, int flag);
static RTN_STATUS send_mess(int card, const char *, const char *name);
static int send_recv_mess(int card, const char *send_com, char *recv_com);
static int send_recv_mess(int card, const char *send_com, char *recv_com, 
			  const char *temp_eos);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);
void closePC6KSockets(void *);

/*----------------functions-----------------*/

struct driver_table PC6K_access =
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

struct drvPC6K_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPC6K = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPC6K);}

static struct thread_args targs = {SCAN_RATE, &PC6K_access, MOTION_DELAY};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PC6K_num_cards <=0)
      printf("    No PC6K controllers configured.\n");
    else
    {
      for (card = 0; card < PC6K_num_cards; card++)
	{
	  struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PC6K controller %d connection failed.\n", card);
	    else
	    {
		struct PC6KController *cntrl;

		cntrl = (struct PC6KController *) brdptr->DevicePrivate;
	    	printf("    PC6K controller %d, port=%s, address=%d, id: %s \n", 
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
    if (PC6K_num_cards <= 0)
    {
	Debug(1, "init(): PC6K driver disabled. PC6KSetup() missing from startup script.\n");
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
  struct PC6KController *cntrl;
  struct mess_node *nodeptr;
  register struct mess_info *motor_info;
  char send_buff[80];
  char *strstrRtn[QUERY_CNT];
  char *recvStr;
  double vel;
  int rtn_state;
  int recvCnt;
  int motorData;
  int motor;
  unsigned int qindex;
  bool recvRetry, recvNext;
  bool plusdir, ls_active = false;
  msta_field status;

  cntrl = (struct PC6KController *) motor_state[card]->DevicePrivate;
  motor_info = &(motor_state[card]->motor_info[signal]);
  status.All = motor_info->status.All;
  motor = signal+1;

  /* LOOP: send all status queries and check for valid response
  *    EXIT LOOP: if communication timeout or invalid response
  *               but allow one retry; 
  */
  qindex = 0;
  recvRetry = recvNext = false;
  do
    {
      strstrRtn[qindex] = NULL;
      sprintf(send_buff, "%d%s", motor, queryOps[qindex].cmnd);
      if ((recvCnt = send_recv_mess(card, send_buff, cntrl->recv_string[qindex])) &&
	  (cntrl->recv_string[qindex][0] == REPLY_CHAR))
	{
	  // Index into return string (add 1 to command length to account for REPLY_CHAR)
	  strstrRtn[qindex] = &cntrl->recv_string[qindex][strlen(send_buff)+1];
        recvRetry = false;
	recvNext = (++qindex >= QUERY_CNT) ? false : true;
	}
      else
	{
        recvNext = false;
	recvRetry = (recvRetry ? false : true);
	}
    } while (recvRetry || recvNext);

    
  /* Check for normal look termination - all queries successful */
  if (qindex >= QUERY_CNT)
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
    recvStr = strstrRtn[QSTATUS];

    Debug(5, "set_status(): status  = %s\n", recvStr);

    status.Bits.RA_DIRECTION = (recvStr[TAS_NEG] == '0') ? 1 : 0;

    status.Bits.RA_HOME = (recvStr[TAS_HOME] == '1') ? 1 : 0;

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    status.Bits.RA_DONE = (recvStr[TAS_INMOTION] == '0') ? 1 : 0;

    /* Set Travel limit switch status bits. */
    if ((recvStr[TAS_HPLUSTL] == '0' && recvStr[TAS_SPLUSTL] == '0') ||  
	status.Bits.RA_HOME)
      status.Bits.RA_PLUS_LS = 0;
    else
      {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	  ls_active = true;
      }
    
    if ((recvStr[TAS_HMINUSTL] == '0' && recvStr[TAS_SMINUSTL] == '0') ||  
	status.Bits.RA_HOME)
      status.Bits.RA_MINUS_LS = 0;
    else
      {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	  ls_active = true;
      }


    /* Position maintence enabled */
    recvStr = strstrRtn[QDRIVE];
    status.Bits.EA_POSITION = (*recvStr == '1') ? 1: 0; 

    /* encoder status */
    status.Bits.EA_SLIP 	= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;

    /* 
     * Parse motor position
     */
    recvStr = strstrRtn[QPOS];
    motorData = (int) atof(recvStr);

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->position = motorData;
	if (motor_state[card]->motor_info[signal].encoder_present == YES)
	  {
	    recvStr = strstrRtn[QEA_POS];
	    motor_info->encoder_position = atoi(recvStr);
	  }
	else
	    motor_info->encoder_position = 0;

	motor_info->no_motion_count = 0;
    }

    status.Bits.RA_PROBLEM = 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    recvStr = strstrRtn[QVEL];
    vel = atof(recvStr);
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
/* send_receive a message to the PC6K board	     */
/* send_recv_mess()		                     */
/*****************************************************/
static int send_recv_mess(int card, const char *send_com, char *recv_com)
{
  return(send_recv_mess(card, send_com, recv_com, NULL));
}

static int send_recv_mess(int card, const char *send_com, char *recv_com,
			  const char *temp_eos)
{
    struct PC6KController *cntrl;
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
	errlogMessage("drvPC6K.c:send_recv_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvPC6K.c:send_recv_mess() - invalid card #%d\n", card);
	return(ERROR);
    }
    Debug(2, "send_recv_mess(): message = %s\n", send_com);

    cntrl = (struct PC6KController *) motor_state[card]->DevicePrivate;

    /* Enable temporary changes to EOS - ie: program creation "-" */
    if (temp_eos != NULL && strlen(temp_eos))
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,temp_eos,strlen(temp_eos));

    timeout = TIMEOUT;
    /* flush any junk at input port - should not be any data available */
    // pasynOctetSyncIO->flush(cntrl->pasynUser); 

    /* Perform atomic write/read operation  */
    status = pasynOctetSyncIO->writeRead(cntrl->pasynUser, send_com, strlen(send_com), 
                                         recv_com, PC6K_MSG_SIZE,
                                         TIMEOUT, &nwrite, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
	recv_com[0] = '\0';
	nread = 0;
    }

    Debug(2, "send_recv_mess(): recv message = \"%s\"\n", recv_com);

    /* Return to standard EOS */
    if (temp_eos != NULL && strlen(temp_eos))
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,
				     PC6K_IN_EOS,strlen(PC6K_IN_EOS));

    return(nread);
}
/*****************************************************/
/* send a message to the PC6K board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, const char *com, const char *name)
{
    struct PC6KController *cntrl;
    int size;
    size_t nwrite;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvPC6K.c:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvPC6K.c:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (name != NULL)
    {
	errlogPrintf("drvPC6K.c:send_mess() - invalid argument = %s\n", name);
	return(ERROR);
    }

    Debug(2, "send_mess(): message = %s\n", com);

    cntrl = (struct PC6KController *) motor_state[card]->DevicePrivate;

    /* flush any junk at input port - should not be any data available */
    // pasynOctetSyncIO->flush(cntrl->pasynUser);

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
    struct PC6KController *cntrl;
    double timeout = 0.;
    size_t nread = 0;
    asynStatus status;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct PC6KController *) motor_state[card]->DevicePrivate;

    timeout	= TIMEOUT;

    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    if (nread > 0)
      Debug(2, "recv_mess(): message = \"%s\"\n", com);

    if ((status != asynSuccess) || (nread <= 0))
    {
	com[0] = '\0';
	nread = 0;
    }

    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* PC6KSetup()                                     */
/*****************************************************/
RTN_STATUS
PC6KSetup(int num_cards,	/* maximum number of controllers in system.  */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PC6K_NUM_CARDS)
	PC6K_num_cards = PC6K_NUM_CARDS;
    else
	PC6K_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before PC6KConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(PC6K_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PC6K_num_cards; itera++)
	motor_state[itera] = NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PC6KConfig()                                    */
/*****************************************************/
RTN_STATUS
PC6KConfig(int card,		/* card being configured */
	       const char *name)   /* asyn port name */
{
    struct PC6KController *cntrl;

    if (card < 0 || card >= PC6K_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PC6KController));
    cntrl = (struct PC6KController *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
    return(OK);
}


/*****************************************************/
/* Upload file to Controller                        */
/* PC6KConfig()                                    */
/*****************************************************/

RTN_STATUS
PC6KUpLoad(int card,               /* Controller Number */
	    const char *file,       /* full path to upload file */
	    const char *progName)   /* PC6K program name - NULL=execute */
{
    FILE *fd;
    char nextLine[BUFF_SIZE];
    // char replyBuff[BUFF_SIZE];
    // char eos_str[] = "-";
    // char *eos_ptr = NULL;
    // int recvCnt;
    int i, lineLen;

    if (card < 0 || card >= total_cards)
      {
	printf("{PC6KUpLoad: Controller does not exist - %d\n",card);
        return(ERROR);
      }

    if (motor_state[card] == NULL)
      {
	printf("PC6KUpLoad: Controller is not configured - %d\n",card);
        return(ERROR);
      }


    if ((fd=fopen(file, "r")) == NULL)
      {
	printf("PC6KUpLoad: File does not exist - %s\n",file);
	return(ERROR);
      }

    if (progName && strlen(progName))
      {
	/* Copy file into PC6K Program */
	sprintf(nextLine, "DEL %s", progName);
	// recvCnt = send_recv_mess(card, nextLine, replyBuff);
	send_mess(card, nextLine, NULL);
	// eos_ptr = eos_str;
	sprintf(nextLine, "DEF %s", progName);
	// recvCnt = send_recv_mess(card, nextLine, replyBuff, eos_ptr);
	// recvCnt = send_recv_mess(card, nextLine, replyBuff);
	send_mess(card, nextLine, NULL);
      }

    while (fgets(nextLine, BUFF_SIZE, fd) != NULL)
      {
	/* Clear control characters */
	for (i = 0, lineLen = strlen(nextLine); i < lineLen; i++)
	  if (!isprint(nextLine[i]))
	    nextLine[i] = ' ';
	
	// recvCnt = send_recv_mess(card, nextLine, replyBuff, eos_ptr);
	// recvCnt = send_recv_mess(card, nextLine, replyBuff);
	send_mess(card, nextLine, NULL);
      }

    fclose(fd);

    if (progName && strlen(progName))
	/* End PC6K Program */
        // recvCnt = send_recv_mess(card, "END", replyBuff);
        send_mess(card, "END", NULL);

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
    struct PC6KController *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    char send_buff[80];
    int total_axis = 0;
    int recvCnt, retryCnt;
    int digits;
    unsigned int i;
    bool nextAxis;
    bool cardFound = false;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */
    nextSocket = 0;

    /* Check for setup */
    if (PC6K_num_cards <= 0)
	return(ERROR);

    /* Initialize command definition array used during set_status() */
    for (i=0; i < QUERY_CNT; i++)
      queryOps[i].cmndLen = strlen(queryOps[i].cmnd);

    for (card_index = 0; card_index < PC6K_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct PC6KController *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 
                                            cntrl->asyn_address, &cntrl->pasynUser, NULL);
	if (success_rtn != asynSuccess) 
	      printf("drvPC68K:motor_init(), error calling pasynOctetSyncIO->connect port=%s error=%d\n", 
		     cntrl->asyn_port, success_rtn);
	else
	{
	     /* Save asyn sockets for IOC exit cleanup */
	     socketStructs[nextSocket].pasynUserCommon = cntrl->pasynUser;
	     socketStructs[nextSocket].connected = true;
	     nextSocket++;


	      /* Set command End-of-string */
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser,
					  PC6K_IN_EOS,strlen(PC6K_IN_EOS));
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser,
					   PC6K_OUT_EOS,strlen(PC6K_OUT_EOS));
	    
	    /* Send a message to the board, see if it exists */
	    retryCnt = 0;

	    do
	    {
	      /* Return value is length of received string */
	      recvCnt = send_recv_mess(card_index, GET_IDENT, buff);
	      /* Check for valid response -- if not retry */
	      if ((recvCnt > 0) && strstr(buff, GET_IDENT) && strstr(buff,"6K"))
		cardFound = true;
	    } while(!cardFound  && ++retryCnt < 3);
	}

	if (cardFound)
	{
	
	    strcpy(brdptr->ident, buff);  /* Save Controller ID  */

	    send_recv_mess(card_index, CMD_ECHO, buff);       /* Turn off echo */

	    brdptr->localaddr = NULL;
	    brdptr->motor_in_motion = 0;
	    /* Stop all motors */
	    send_recv_mess(card_index, STOP_ALL, buff);   
            // All stop requires a delay before the controller starts responding
	    //   again - handshake on some command 
	    retryCnt = 0;
	    do {
	      recvCnt = send_recv_mess(card_index, CMD_DRIVE, buff); 
              if (recvCnt && !strstr(buff, CMD_DRIVE))
			recvCnt = 0;	                  
	    } while (!recvCnt && ++retryCnt < 3);


	    /* send_mess(card_index, COMEXEC_ENA, NULL); */   /* Enable continuous commands */
	    send_recv_mess(card_index, COMEXEC_ENA, buff);   /* Enable continuous commands */
	    // send_recv_mess(card_index, CMD_SCALE, buff);     /* Enable scaling  - unary */

	    /* The find how many axes this controller has */
            total_axis = 0;
	    
	    do {
	      brdptr->motor_info[total_axis].motor_motion = NULL;

	      sprintf(send_buff, "%d%s", total_axis+1, CMD_POS);
	      recvCnt = send_recv_mess(card_index, send_buff, buff);              
	      nextAxis = (recvCnt > 0 && (buff[0] == REPLY_CHAR)) ? true : false;
	      if (nextAxis)
		  total_axis++;
	    }
            while (nextAxis);

	    brdptr->total_axis = total_axis;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;


	    /* Encoder Enable - both STEPPER and DC motors have encoder support */
	        motor_info->encoder_present = YES;
	        motor_info->status.Bits.EA_PRESENT = 1;
	        motor_info->pid_present = YES;
	        motor_info->status.Bits.GAIN_SUPPORT = 1;

		/* Set unary scaling for Position and Velocities - program in counts */
                // sprintf(send_buff, "%d%s", motor_index+1, CMD_SCLD);
		// send_recv_mess(card_index, send_buff, buff);
		// sprintf(send_buff, "%d%s", motor_index+1, CMD_SCLV);
		// send_recv_mess(card_index, send_buff, buff);
		// sprintf(send_buff, "%d%s", motor_index+1, CMD_SCLA);
		// send_recv_mess(card_index, send_buff, buff);		

                /* Determine if motor type = servo  */
                sprintf(send_buff, "%d%s", motor_index+1, CMD_AXSDEF);
       	        if (send_recv_mess(card_index, send_buff, buff) > 0 && 
		    (buff[0] == REPLY_CHAR) &&
		    (buff[strlen(send_buff)+1] == '1'))
		    cntrl->type[motor_index] =  DC;
		else
		    cntrl->type[motor_index] = STEPPER;
		

		/* Determin drive resolution */
		if (cntrl->type[motor_index] ==  DC)
		  sprintf(send_buff, "%d%s", motor_index+1, CMD_ERES);
		else
		  sprintf(send_buff, "%d%s", motor_index+1, CMD_DRES);

       	        if (send_recv_mess(card_index, send_buff, buff) > 0 && (buff[0] == REPLY_CHAR))
		  cntrl->drive_resolution[motor_index] = 1.0 / atof(&buff[strlen(send_buff)+1]);
		
		digits = (int) -log10(cntrl->drive_resolution[motor_index]) + 2;
		if (digits < 1)
		  digits = 1;
		cntrl->res_decpts[motor_index] = digits;

                /* Determine low limit */
                sprintf(send_buff, "%d%s", motor_index+1, CMD_LOWLS);
       	        if (send_recv_mess(card_index, send_buff, buff) > 0 && (buff[0] == REPLY_CHAR))
		  motor_info->low_limit = atof(&buff[strlen(send_buff)+1]);

                /* Determine high limit */
		sprintf(send_buff, "%d%s", motor_index+1, CMD_HIGHLS);
       	        if (send_recv_mess(card_index, send_buff, buff) > 0 && (buff[0] == REPLY_CHAR))
		  motor_info->high_limit = atof(&buff[strlen(send_buff)+1]);


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

    // epicsThreadCreate((char *) "PC6K_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);
    epicsThreadCreate((char *) "PC6K_motor", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    // void epicsAtExit( (*epicsExitFunc)(void *arg), void *arg);
    epicsAtExit(&closePC6KSockets, NULL);

    return(OK);
}

/***************************************************************************************/
static void CloseSocket(int SocketIndex)
{
    socketStruct *psock;
    asynUser *pasynUser;
    int status;

    if ((SocketIndex < 0) || (SocketIndex >= nextSocket)) {
        printf("drvPMNC CloseSocket: invalid SocketIndex %d\n", SocketIndex);
        return;
    }
    psock = &socketStructs[SocketIndex];
    pasynUser = psock->pasynUserCommon;
    status = pasynCommonSyncIO->disconnectDevice(pasynUser);
    if (status != asynSuccess ) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "drvPMNC CloseSocket: error calling pasynCommonSyncIO->disconnect, status=%d, %s\n",
                  status, pasynUser->errorMessage);
        return;
    } else
      printf("drvPC6K CloseSocket: Disconnected SocketIndex %d\n",SocketIndex);

    psock->connected = false;
}

/***************************************************************************************/
void closePC6KSockets(void *arg)
{
    int i;

    for (i=0; i<nextSocket; i++) {
        if (socketStructs[i].connected) CloseSocket(i);
    }
}

/*---------------------------------------------------------------------*/
