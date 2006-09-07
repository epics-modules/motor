/*
FILENAME...	drvPMNC87xx.cc
USAGE...	Motor record driver level support for NewFocus PMNC87xx.

Version:	1.17
Modified By:	sullivan
Last Modified:	2005/03/30 19:10:48
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 06/28/06
 *	Current Author: Joe Sullivan
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
 * .01 10-20-97 mlr initialized from drvOms58
 * .02 10-30-97 mlr Replaced driver calls with gpipIO functions
 * .03 10-30-98 mlr Minor code cleanup, improved formatting
 * .04 02-01-99 mlr Added temporary fix to delay reading motor positions at
 *			the end of a move.
 * .05 04-18-00 rls PMNC87xx takes 2 to 5 seconds to respond to queries after
 *			hitting a hard travel limit.  Adjusted GPIB and SERIAL
 *			timeouts accordingly.  Deleted communication retries.
 *			Reworked travel limit processing so that direction
 *			status bit matches limit switch.  Copied recv_mess()
 *			logic from drvMM4000.c.  Use TPE command to determine
 *			if motor has an encoder.
 * .06 10/02/01 rls - allow one retry after a communication error.
 *		    - use motor status response bit-field.
 * .07 05-22-03	rls - Converted to R3.14.2.
 * .08 10/23/03 rls - recv_mess() eats the controller error response, outputs
 *			an error message and retries.
 * .09 02/03/04 rls - Eliminate erroneous "Motor motion timeout ERROR".
 * .10 07/09/04 rls - removed unused <driver>Setup() argument.
 * .11 07/28/04 rls - "epicsExport" debug variable.
 * .12 09/21/04 rls - support for 32axes/controller.
 * .13 12/21/04 rls - MS Visual C compatibility; make all epicsExportAddress
 *		      extern "C" linkage.
 *		    - make debug variables always available.
 * .14 01/11/06 jps - initialize from drvMM3000.cc
 *
 */

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "NewFocusRegister.h"
#include "drvPMNCCom.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define STATIC static

#define READ_RESOLUTION "VER"
#define READ_STATUS     "STA %d"
#define READ_DIAG       "DIAG %d"
#define READ_POS        "POS %d"
#define READ_CHAN_SELECT "CHL %d"
#define STOP_ALL        "STO"
#define INIT_ALL        "INI"
#define MOTOR_ON        "MON"
#define MOTOR_OFF       "MOF"
#define JOY_ON          "JON"
#define GET_IDENT       "VER"
#define GET_MPV         "MPV"  /* Query Minimum Profile Velocity */

#define GET_DRT         "DRT"  /* Query Driver Types */

#define VER_STR         "Version 1."

#define PROMPT_EOS	">"
#define NL_EOS	        "\n"
#define CMND_EOS        "\r"

/* Status byte bits */
#define M_AXIS_MOVING     0x01
#define M_MOTOR_POWER     0x02
#define M_MOTOR_DIRECTION 0x04
#define M_PLUS_LIMIT      0x08
#define M_MINUS_LIMIT     0x10
#define M_HOME_SIGNAL     0x20

#define BUFF_SIZE 100       /* Maximum length of string to/from PMNC87xx */

#define PMNC_TIMEOUT	3.0	/* Command timeout in sec. */
#define MESS_ERR     -1

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	#define Debug(l, f, args...) { if(l<=drvPMNC87xxdebug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvPMNC87xxdebug = 0;
extern "C" {epicsExportAddress(int, drvPMNC87xxdebug);}

/* --- Local data. --- */
int PMNC87xx_num_cards = 0;
int PMNC87xx_num_drivers = 0;  /* Number of Drivers per Controller */
char nobuff[BUFF_SIZE];        /* Scratch buffer - noreply commands */

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC RTN_STATUS send_mess(int, char const *, char *);
STATIC int send_recv_mess(int, char const *, char *, char const *);
STATIC int set_status(int, int);
STATIC void start_status(int);
static long report(int);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PMNC87xx_access =
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

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPMNC87xx = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPMNC87xx);}

STATIC struct thread_args targs = {SCAN_RATE, &PMNC87xx_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PMNC87xx_num_cards <=0)
	printf("    No PMNC87xx controllers configured.\n");
    else
    {
	for (card = 0; card < PMNC87xx_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PMNC87xx controller %d connection failed.\n", card);
	    else
	    {
		struct PMNCcontroller *cntrl;

		cntrl = (struct PMNCcontroller *) brdptr->DevicePrivate;
	    	printf("    PMNC87xx controller %d asyn port= %s, address=%d, id: %s \n", 
			   card, cntrl->asyn_port, cntrl->asyn_address,
			   brdptr->ident);
		    break;
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
    if (PMNC87xx_num_cards <= 0)
    {
	Debug(1, "init(): PMNC87xx driver disabled. PMNC87xxSetup() missing from startup script.\n");
    }
    return((long) 0);
}


STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/*********************************************************
 * Read the status and position of all motors on a card
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/
STATIC void start_status(int card)
{
}

/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

#define CHECKRTN \
 if (recvRtn > 0) { \
   cntrl->status = NORMAL; \
   status.Bits.CNTRL_COMM_ERR = 0; \
 } \
 else { \
   if (cntrl->status == NORMAL) { \
     cntrl->status = RETRY; \
     setReturn=0; \
   } \
   else { \
     cntrl->status = COMM_ERR; \
     status.Bits.CNTRL_COMM_ERR = 1; \
     status.Bits.RA_PROBLEM = 1; \
     setReturn = 1; \
   } \
   goto exit; \
  }
  

STATIC int set_status(int card, int signal)
{
    struct PMNCcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char *pStr;
    char buff[BUFF_SIZE];
    MOTOR_STATUS mstat;
    MOTOR_AUX_STATUS auxstat;
    struct PMD_axis *signalDef;
    int setReturn;
    int recvRtn;
    int driverID, motorID;
    int recvDriver, recvStatus; 
    long motorData;
    int selectMotor;
    bool plusdir, ls_active = false;
    msta_field status;


    setReturn = 0;
    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    status.All = motor_info->status.All;

    signalDef = &cntrl->axisDef[signal];

    /* Get the motor information  */
    driverID = signalDef->driverNum;
    motorID = signalDef->motorNum;

    if (signalDef->driverType == PMD8753)
      {
	sprintf(buff, READ_CHAN_SELECT, driverID);
	pStr = cntrl->chan_select_string[driverID];
	recvRtn = send_recv_mess(card, buff, pStr, NULL);
	CHECKRTN;
        Debug(2, "start_status(): ChanSelect_string=%s\n", pStr);
        selectMotor = atoi(pStr);
      }
    else
      selectMotor = 0;  /* PMD8751 only has one motor */


    /* Set defaults in-case motor is not selected */
    motorData = cntrl->last_position[signal];
    mstat.All = 0;
    auxstat.All = 0;

    sprintf(buff, READ_STATUS, driverID);
    pStr = cntrl->status_string[driverID];
    /* STATUS command does not return a prompt - bug? */ 
    recvRtn = send_recv_mess(card, buff, pStr, NL_EOS);
    CHECKRTN;
    Debug(2, "set_status(): Status_string=%s\n", pStr);
    if (sscanf(pStr, "A%d=0x%x",&recvDriver, &recvStatus) == 2)
      {
	mstat.All = recvStatus;
	Debug(5, "set_status(): drive %d - status byte = %x\n", recvDriver, recvStatus);
      }
    else
      Debug(1, "set_status(): drive %d - INVALID status string %s\n", driverID, pStr);


    if (signalDef->driverType == PMD8751)
      {
	sprintf(buff, READ_DIAG, driverID);
	pStr = cntrl->aux_status_string[driverID];
	recvRtn = send_recv_mess(card, buff, pStr, NULL);
	CHECKRTN;
	Debug(2, "set_status(): Aux Status_string=%s\n", pStr);
	if (sscanf(pStr, "A%d=0x%x",&recvDriver, &recvStatus) == 2)
	  {
	    auxstat.All = recvStatus;
	    Debug(5, "set_status(): drive %d - status byte = %x\n", recvDriver, recvStatus);
	  }
	else
	  Debug(1, "set_status(): drive %d - INVALID Aux Status string %s\n", driverID, pStr);
      }

    /* Check if set_status() motor is currently selected on driver */
    if (selectMotor == motorID)
      {
	sprintf(buff, READ_POS, driverID);
	pStr = cntrl->position_string[driverID];
	recvRtn = send_recv_mess(card, buff, pStr, NULL);
	CHECKRTN;
	Debug(2, "set_status(): postion_string=%s\n", pStr);
	if (sscanf(pStr, "A%d=%ld",&recvDriver, &motorData) == 2)
	  {
	    Debug(5, "set_status(): drive %d = Position:%ld\n", recvDriver, motorData);
	  }
	else
	  Debug(1, "set_status(): drive %d - INVALID position string %s\n", driverID, pStr);
      }
    else
      {
	if (signalDef->driverType == PMD8753)
	  motorData = cntrl->last_position[signal] = 0; 

        Debug(5, "set_status(): NOT SELECTED drive %d, motor %d\n", driverID, motorID);
      }


    nodeptr = motor_info->motor_motion;


    /* Interpret data and move it into global area */

    if (signalDef->driverType == PMD8753)    // Open-Loop Driver
      {
	/* Drive Enabled */
	status.Bits.EA_POSITION = (mstat.Bits_8753.powerOn) ? 1: 0; 

	if (mstat.Bits_8753.inMotion == false)
	  status.Bits.RA_DONE = 1;
	else if (selectMotor == motorID)   // Moving only applies to selected motor
	  status.Bits.RA_DONE = 0;

	/* Limit Switch Status - NOT AVAILABLE */
	status.Bits.RA_PLUS_LS = 0;
        status.Bits.RA_MINUS_LS = 0;
	status.Bits.RA_HOME = 0;

	/* encoder status - NOT AVAILABLE  */
	status.Bits.EA_SLIP 	= 0;
	status.Bits.EA_SLIP_STALL	= 0;
	status.Bits.EA_HOME		= 0;
      }
    else if (signalDef->driverType == PMD8751)   // Closed-Loop Driver 
      {
	// if (mstat.Bits_8751.moveDone == true && (mstat.Bits_8751.homing == false || motor_info->no_motion_count > 5))
	if (mstat.Bits_8751.moveDone == true)
	  status.Bits.RA_DONE = 1;
	else
	  status.Bits.RA_DONE = 0;

	/* Drive Enabled */
	status.Bits.EA_POSITION = (auxstat.Bits_8751.servoOn) ? 1: 0; 

	if (mstat.Bits_8751.powerOn && auxstat.Bits_8751.servoOn)
	  {
	    /* Limit Switch Status */
	    status.Bits.RA_PLUS_LS = ((mstat.Bits_8751.forLimit) ? 1: 0);
	    status.Bits.RA_MINUS_LS = ((mstat.Bits_8751.revLimit) ? 1: 0);
            status.Bits.RA_HOME = ((mstat.Bits_8751.homing) ? 1: 0);
	  }

	/* encoder status - NOT AVAILABLE  */
	status.Bits.EA_SLIP 	= 0;
	status.Bits.EA_SLIP_STALL	= 0;
	status.Bits.EA_HOME		= 0;
      }


    Debug(2, "set_status(): Axis:%d, New Pos: %ld, Last Pos: %ld", signal, motorData, cntrl->last_position[signal]);

    if (motorData == cntrl->last_position[signal])
      {
	motor_info->no_motion_count++;
        status.Bits.RA_DIRECTION = 1;
      }
    else
    {
        if (signalDef->driverType == PMD8751)   // Closed-Loop Driver 
	  motor_info->position = motorData;
	else
	  motor_info->position += (motorData - cntrl->last_position[signal]);

	if (motor_state[card]->motor_info[signal].encoder_present == YES) 
	  {
	    if (signalDef->driverType == PMD8751)   // Closed-Loop Driver 
	      motor_info->encoder_position = (epicsInt32) motorData;
	    else
	      motor_info->encoder_position += (epicsInt32) (motorData - cntrl->last_position[signal]);
	  }
	else
	    motor_info->encoder_position = 0;

       /* Use sign of motor position (always relative) for direction */
	if (signalDef->driverType == PMD8753)
	  status.Bits.RA_DIRECTION = (motorData >= 0) ? 1 : 0;
	else if (signalDef->driverType == PMD8751)
	  status.Bits.RA_DIRECTION = ((motorData - cntrl->last_position[signal]) >= 0) ? 1 : 0;

        cntrl->last_position[signal] = motorData;
	motor_info->no_motion_count = 0;
    }


    plusdir = (status.Bits.RA_DIRECTION) ? true : false;
    Debug(1, ", Pos: %d, Dir: %x\n", motor_info->position, plusdir); 

    /* Determine if move done because of limit switches */   
    if ((plusdir && status.Bits.RA_PLUS_LS) || (!plusdir && status.Bits.RA_MINUS_LS))
      ls_active = true;

    status.Bits.RA_PROBLEM = 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    // if (!plusdir)
    // motor_info->velocity *= -1;

    setReturn = (!motor_info->no_motion_count || ls_active == true ||
		 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move processing. */
    if ((status.Bits.RA_DONE || ls_active == true) )
    {
      if (signalDef->driverType == PMD8753)
	{
	cntrl->last_position[signal] = 0; /* Reset last position after each move */

        if (selectMotor == motorID)
	  {
	    /* Clear position on selected motor */
	    sprintf(buff, "CHL A%d=%d", driverID, motorID);
	    send_recv_mess(card, buff, nobuff, NULL);
	  }
	}

      if (nodeptr != 0)
	{
	  /* Allways turn joystick on after a move */
	  /* strcpy(buff, JOY_ON);   */
	    
	  if (nodeptr->postmsgptr != 0)  /* Test for post move record string */
	    {
	      strcpy(buff, nodeptr->postmsgptr);
	      send_recv_mess(card, buff, nobuff, NULL);
	    }

	  nodeptr->postmsgptr = NULL;
	}
    }

exit:
    motor_info->status.All = status.All;
    return(setReturn);
}


/*****************************************************/
/* atomic write/read function                        */
/* PMNC87xx motor controller has a  handshake        */
/* with the return of the prompt '>' character 	     */
/* send_recv_mess()			             */
/*****************************************************/
STATIC int send_recv_mess(int card, char const *send_com, char *recv_com, char const *eos)
{
    struct PMNCcontroller *cntrl;
    int size;
    size_t nwrite;
    size_t nread = 0;
    asynStatus status;
    int eomReason;


    size = strlen(send_com);
    recv_com[0] = '\0';

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvPMNC87xx:send_recv_mess(); message size violation.\n");
	return(MESS_ERR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return (OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvPMNC87xx:send_recv_mess() - invalid card #%d\n", card);
	return(MESS_ERR);
    }

    Debug(2, "send_recv_mess(): message = %s\n", send_com);

    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;

    if (eos)
      // Temporary Change to End-of-string
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,eos,strlen(eos));


    /* Perform atomic write/read operation  */
    status = pasynOctetSyncIO->writeRead(cntrl->pasynUser, send_com, size, 
                                         recv_com, BUFF_SIZE-1,
                                         PMNC_TIMEOUT, &nwrite, &nread, &eomReason);
    if (status != asynSuccess)
    {
      Debug(3, "send_recv_mess(): ERROR - staus =%d, nread = %d\n", (int) status, nread);
      recv_com[0] = '\0';
      return(MESS_ERR);
    }

    if (eos)
      // Reset EOS to default 
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,PROMPT_EOS,strlen(PROMPT_EOS));


    Debug(2, "send_recv_mess(): recv message = \"%s\"\n", recv_com);

    return(nread);

}
/*****************************************************/
/* send a message to the PMNC87xx board		     */
/* send_mess()			                     */
/*****************************************************/
STATIC RTN_STATUS send_mess(int card, char const *com, char *name)
{
    struct PMNCcontroller *cntrl;
    int size;
    size_t nwrite;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvPMNC87xx:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvPMNC87xx:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (name != NULL)
    {
	errlogPrintf("drvPMNC87xx:send_mess() - invalid argument = %s\n", name);
	return(ERROR);
    }

    Debug(2, "send_mess(): message = %s\n", com);

    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, com, size, 
                            PMNC_TIMEOUT, &nwrite);
    
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
 *              | 0 = read until '\n'
 *              | 1 = read until '>'
 *
 * LOGIC...
 *  Initialize:
 *	- receive timeout to zero
 *	- received string length to zero.
 *
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *
 *  SWITCH on port type.
 *	CASE port type is GPIB.
 *	    BREAK.
 *	CASE port type is RS232.
 *	    IF input "flag" indicates NOT flushing the input buffer.
 *		Set receive timeout nonzero.
 *	    ENDIF
 *	    Call pasynOctetSyncIO->read().
 *
 *	    NOTE: The PMNC87xx sometimes responds to an MS command with an error
 *		message (see PMNC87xx User's Manual Appendix A).  This is an
 *		unconfirmed PMNC87xx bug.  Retry read if this is a Hard Travel
 *		limit switch error. This effectively flushes the error message.
 *
 *	    IF input "com" buffer length is > 3 characters, AND, the 1st
 *			character is an "E" (Maybe this an unsolicited error
 *			message response?).
 *	   	Call pasynOctetSyncIO->read().
 *	    ENDIF
 *	    BREAK
 *    ENDSWITCH
 *		
 *  NORMAL RETURN.
 */

STATIC int recv_mess(int card, char *com, int flag)
{
    struct PMNCcontroller *cntrl;
    double timeout = 0.;
    int flush = 1;
    size_t nread = 0;
    int eomReason;
    asynStatus status;

    /* Check that card exists */
    if (!motor_state[card])
	return(MESS_ERR);

    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;

    if (flag != FLUSH) {
        flush = 0;
	timeout	= PMNC_TIMEOUT;
    }
    if (flush) status = pasynOctetSyncIO->flush(cntrl->pasynUser);

    if (flag == 0)
      // Temporary Change to EOS to '\n'
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,NL_EOS,strlen(NL_EOS));

    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    if (status != asynSuccess)
    {
	com[0] = '\0';
	return(MESS_ERR);
    }

    if (flag == 0)
      // Reset  EOS to '\n'
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,PROMPT_EOS,strlen(PROMPT_EOS));

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* PMNC87xxSetup()                                     */
/*****************************************************/
RTN_STATUS
PMNC87xxSetup(int num_cards,	/* maximum number of controllers in system.  */
	      int num_drivers,	/* maximum number of drivers per controller */
	      int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PMNC87xx_NUM_CARDS)
	PMNC87xx_num_cards = PMNC87xx_NUM_CARDS;
    else
	PMNC87xx_num_cards = num_cards;

    if (num_drivers < 1 || num_drivers > PMNC87xx_NUM_DRIVERS)
	PMNC87xx_num_drivers = PMNC87xx_NUM_DRIVERS;
    else
	PMNC87xx_num_drivers = num_drivers;


    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before PMNC87xxConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(PMNC87xx_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PMNC87xx_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PMNC87xxConfig()                                    */
/*****************************************************/
RTN_STATUS
PMNC87xxConfig(int card,		/* card being configured */
            const char *port,   /* asyn port name */
            int address)        /* asyn address (GPIB) */
{
    struct PMNCcontroller *cntrl;

    if (card < 0 || card >= PMNC87xx_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PMNCcontroller));
    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, port);
    cntrl->asyn_address = address;
    return(OK);
}



/*****************************************************/
/* initialize all software and hardware		     */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()			                     */
/*****************************************************/
STATIC int motor_init()
{
    struct controller *brdptr;
    struct PMNCcontroller *cntrl;
    struct PMD_axis *axisdef;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    char *bufptr;
    int total_axis = 0;
    int driverIndex; 
    int rtnCnt;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */
    
    /* Check for setup */
    if (PMNC87xx_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PMNC87xx_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct PMNCcontroller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 
                          cntrl->asyn_address, &cntrl->pasynUser, NULL);
	if (success_rtn == asynSuccess)
	{
	  int retry;

	      /* Set command End-of-string */
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser,PROMPT_EOS,strlen(PROMPT_EOS));
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser,CMND_EOS,strlen(CMND_EOS));
	    

	    /* flush any junk at input port - should not be any data available */
	    pasynOctetSyncIO->flush(cntrl->pasynUser);
    
	    rtnCnt = 0;
	    retry = 0;

	    /* Send a message to the board, see if it exists */
	    do
	    {
	      rtnCnt = send_recv_mess(card_index, GET_IDENT, buff, NULL);
	      retry++;
		/* Return value is length of response string */
	    } while(rtnCnt <= 0 && retry < 3);


	    // Resetting the controller loses the ethernet connection - bad idea
	    if (NULL) 
	      {
   	     /* Reset Controller: takes > 5 sec */
	      send_recv_mess(card_index, INIT_ALL, nobuff, NULL); 

	      retry = 0;
	      rtnCnt = 0;
	      /* Wait for Controller to return */
	      do
		{
		  rtnCnt = send_recv_mess(card_index, GET_IDENT, buff, NULL);
		  retry++;
		  /* Return value is length of response string */
		} while(rtnCnt <= 0 && retry < 1000);

	      }

	}

	if (success_rtn == asynSuccess && rtnCnt > 0)
	{
	    strncpy(brdptr->ident, &buff[0], MAX_IDENT_LEN);  /* Save Version info */	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

	    /* Set Motion Master model indicator. */
	    bufptr = strstr(brdptr->ident, VER_STR);
	    if (bufptr == NULL)
	    {
		errlogPrintf("drvPMNC87xx.c:motor_init() - unknown version = %s\n", brdptr->ident);
		motor_state[card_index] = (struct controller *) NULL;
		continue;
	    }
	    /* Check for known version - 1.x */
	    switch (*(bufptr + strlen(VER_STR)))
	      {
	      case '0':
		cntrl->pmnc = PMNC8750;
		break;
	      case '5':
	      case '6':
		cntrl->pmnc = PMNC8752;		
		break;
	      default:
		errlogPrintf("drvPMNC87xx:motor_init() - invalid model = %s\n", brdptr->ident);
		return(ERROR);
		break;
	      }
	    
	    
            total_axis = 0;
	    driverIndex = 0;
	    axisdef = cntrl->axisDef;

	    if (cntrl->pmnc == PMNC8750)
	      {
		/* Use indirect method to determine axis count - Only 8753 driver supported */
		send_mess(card_index, GET_MPV, NULL);
		/* Number of return strings will tell us how many axes this controller has */
		/* Return format: A<driver#> M<motor#>=<parm> */
		while (recv_mess(card_index, buff, 0) > 0)
		  {
		     sscanf(buff, "A%d M%d", &driverIndex, &motor_index);
		     axisdef[total_axis].driverType = PMD8753;
		     axisdef[total_axis].driverNum = driverIndex;
		     axisdef[total_axis].motorNum = motor_index;
		     brdptr->motor_info[total_axis].motor_motion = NULL;
		     Debug(3, "motor_init(): Next Axis %s\n", buff);
		     total_axis++;
		   }
	      }
	    else // PMNC8752
	      {
		union {int All; PMD_model ID;} driver;
		int addAxis, newAxis, tmpAxis; 

		/* Use direct driver type query (DRT) */
		send_mess(card_index, GET_DRT, NULL);
		/* Number of return strings will tell us how many axes this controller has */
		/* Return format: A<driver#> <driveType> */
		while (recv_mess(card_index, buff, 0) > 0)
		  {
		    sscanf(buff, "A%d=%d", &driverIndex, &driver.All);
		    if (driver.ID == PMD8753)
		      addAxis = 3;
		    else if (driver.ID == PMD8751)
		      addAxis = 1;
		    else
		      {
			errlogPrintf("drvPMNC87xx:motor_init() - invalid driver = %d\n", driver.All);
			return(ERROR);
		      }

		    Debug(3, "motor_init(): Next Driver %s\n", buff);

		    for (newAxis = 0, tmpAxis = total_axis; newAxis < addAxis; newAxis++, tmpAxis++)
		      {
			axisdef[tmpAxis].driverType = driver.ID;
			axisdef[tmpAxis].driverNum = driverIndex;
			axisdef[tmpAxis].motorNum = newAxis;
			brdptr->motor_info[tmpAxis].motor_motion = NULL;
		      }

		    total_axis += addAxis;
		  }

	      }
            
	    brdptr->total_axis = total_axis;
            cntrl->total_drivers = driverIndex;
            cntrl->status = NORMAL;
            Debug(2, "motor_init(): Total Drivers %d\nTotal Axis=%d\n", driverIndex, brdptr->total_axis);

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;

		/* Encoder Enable */
                motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		// motor_info->pid_present = YES;
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

    epicsThreadCreate((char *) "PMNC87xx_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

/*---------------------------------------------------------------------*/
