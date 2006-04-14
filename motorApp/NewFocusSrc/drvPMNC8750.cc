/*
FILENAME...	drvPMNC8750.cc
USAGE...	Motor record driver level support for NewFocus PMNC8750.

Version:	1.17
Modified By:	sluiter
Last Modified:	2005/03/30 19:10:48
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/97
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
 * .01 10-20-97 mlr initialized from drvOms58
 * .02 10-30-97 mlr Replaced driver calls with gpipIO functions
 * .03 10-30-98 mlr Minor code cleanup, improved formatting
 * .04 02-01-99 mlr Added temporary fix to delay reading motor positions at
 *			the end of a move.
 * .05 04-18-00 rls PMNC8750 takes 2 to 5 seconds to respond to queries after
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
#include "motor.h"
#include "NewFocusRegister.h"
#include "drvPMNCCom.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define STATIC static

#define READ_RESOLUTION "VER"
#define READ_STATUS     "STA"
#define READ_POSITION   "POS"
#define READ_ONE_POSITION   "POS A1"
#define STOP_ALL        "STO"
#define INIT_ALL        "INI"
#define MOTOR_ON        "MON"
#define MOTOR_OFF       "MOF"
#define JOY_ON          "JON"
#define GET_IDENT       "VER"
#define GET_CHAN_SELECT "CHL"
#define GET_MPV         "MPV"  /* Query Minimum Profile Velocity */

#define NOREPLY_EOS	">"
#define REPLY_EOS	"\n"
#define CMND_EOS               "\r"

/* Status byte bits */
#define M_AXIS_MOVING     0x01
#define M_MOTOR_POWER     0x02
#define M_MOTOR_DIRECTION 0x04
#define M_PLUS_LIMIT      0x08
#define M_MINUS_LIMIT     0x10
#define M_HOME_SIGNAL     0x20

#define BUFF_SIZE 100       /* Maximum length of string to/from PMNC8750 */

/* The PMNC8750 does not respond for 2 to 5 seconds after hitting a travel limit. */
#define SERIAL_TIMEOUT	5.0	/* Command timeout in sec. */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	#define Debug(l, f, args...) { if(l<=drvPMNC8750debug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvPMNC8750debug = 0;
extern "C" {epicsExportAddress(int, drvPMNC8750debug);}

/* --- Local data. --- */
int PMNC8750_num_cards = 0;
int PMNC8750_num_drivers = 0;  /* Number of Drivers per Controller */

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC RTN_STATUS send_mess(int, char const *, char *);
STATIC RTN_STATUS send_noreply_mess(int, char const *, char *);
STATIC int set_status(int, int);
STATIC void start_status(int);
static long report(int);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PMNC8750_access =
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
    start_status,
    &initialized,
    NULL
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPMNC8750 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPMNC8750);}

STATIC struct thread_args targs = {SCAN_RATE, &PMNC8750_access};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PMNC8750_num_cards <=0)
	printf("    No PMNC8750 controllers configured.\n");
    else
    {
	for (card = 0; card < PMNC8750_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PMNC8750 controller %d connection failed.\n", card);
	    else
	    {
		struct PMNCcontroller *cntrl;

		cntrl = (struct PMNCcontroller *) brdptr->DevicePrivate;
	    	printf("    PMNC8750 controller %d asyn port= %s, address=%d, id: %s \n", 
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
    if (PMNC8750_num_cards <= 0)
    {
	Debug(1, "init(): PMNC8750 driver disabled. PMNC8750Setup() missing from startup script.\n");
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
    struct PMNCcontroller *cntrl;
    int itera, drive;
    int recv_flag;
    int total_drives;
    char buff[BUFF_SIZE];

    if (card >= 0)
    {
	cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;

        total_drives = cntrl->total_drivers;
        Debug(3, "start_status(): Total Drives=%d\n", total_drives);

	send_mess(card, READ_STATUS, NULL);
        for (drive=0, recv_flag=1; drive < (total_drives+2) && recv_flag; drive++) 
        {
   	    recv_flag = recv_mess(card, cntrl->status_string[drive], 0);
            Debug(2, "start_status(): Status_string=%s\n", cntrl->status_string[drive]);
	}
	recv_mess(card, buff, 1); /* Get Prompt */
	if (recv_flag)
	{
	    cntrl->status = NORMAL;
	    send_mess(card, GET_CHAN_SELECT, NULL);
            for (drive=0, recv_flag=1; drive < total_drives && recv_flag; drive++)
	      {
		recv_flag = recv_mess(card, cntrl->chan_select_string[drive], 0);
                Debug(2, "start_status(): ChanSelect_string=%s\n", cntrl->chan_select_string[drive]);
	      }
	     recv_mess(card, buff, 1); /* Get Prompt */

	    send_mess(card, READ_POSITION, NULL);
            for (drive=0, recv_flag=1;  drive < total_drives && recv_flag; drive++)
	      {
		recv_flag = recv_mess(card, cntrl->position_string[drive], 0);
                Debug(2, "start_status(): Position_string=%s\n", cntrl->position_string[drive]);
	      }
	      recv_mess(card, buff, 1); /* Get Prompt */
	}
	else
	{
	    if (cntrl->status == NORMAL)
		cntrl->status = RETRY;
	    else
		cntrl->status = COMM_ERR;
	}
    }
    else
    {
	/* 
	 * For efficiency we send messages to all cards, then read all
	 * responses.  This minimizes the latency due to processing on each card
	 */
	for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
	    send_mess(itera, READ_STATUS, NULL);
	for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
	{
	    cntrl = (struct PMNCcontroller *) motor_state[itera]->DevicePrivate;

	    total_drives = cntrl->total_drivers;

	    for (drive=0, recv_flag=1; drive < (total_drives+2) && recv_flag; drive++) 
	    {
   		  recv_flag = recv_mess(itera, cntrl->status_string[drive], 0);
		  Debug(2, "start_status(): Card %d Status_string=%s\n", itera, cntrl->status_string[drive]);
	    }
	    recv_mess(itera, buff, 1); /* Get Prompt */

	    if (recv_flag)
		cntrl->status = NORMAL;
	    else
	    {
		if (cntrl->status == NORMAL)
		    cntrl->status = RETRY;
		else
		    cntrl->status = COMM_ERR;
	    }
	}
        /* Find the selected channel per driver */
	for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
	    send_mess(itera, GET_CHAN_SELECT, NULL);
	for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
	{
	    cntrl = (struct PMNCcontroller *) motor_state[itera]->DevicePrivate;

	    total_drives = cntrl->total_drivers;

            for (drive=0, recv_flag=1; drive < total_drives && recv_flag; drive++)
	    {
		recv_flag = recv_mess(itera, cntrl->chan_select_string[drive], 0);
                Debug(2, "start_status(): Card %d ChanSelect_string=%s\n", itera, cntrl->chan_select_string[drive]);
	    }
	    recv_mess(itera, buff, 1); /* Get Prompt */
	}
        /* Only the selected channel on each driver is returned */
	for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
	    send_mess(itera, READ_POSITION, NULL);
	for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
	{
	    cntrl = (struct PMNCcontroller *) motor_state[itera]->DevicePrivate;

	    total_drives = cntrl->total_drivers;

            for (drive=0, recv_flag=1;  drive < total_drives && recv_flag; drive++)
	      {
		recv_flag = recv_mess(itera, cntrl->position_string[drive], 0);
                Debug(2, "start_status(): Card %d Position_string=%s\n", itera, cntrl->position_string[drive]);
	      }

	    recv_mess(itera, buff, 1); /* Get Prompt */
	}
    }
}
/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

STATIC int set_status(int card, int signal)
{
    struct PMNCcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char *p;
    char buff[BUFF_SIZE];
    MOTOR_STATUS mstat;
    int rtn_state;
    int drive, scan_drive, scan_mstat; 
    int drive_motor, scan_motor;
    long motorData;
    bool plusdir, ls_active = false;
    msta_field status;


    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    status.All = motor_info->status.All;

    if (cntrl->status != NORMAL)
    {
	if (cntrl->status == COMM_ERR)
	{
 	    status.Bits.CNTRL_COMM_ERR = 1;
 	    status.Bits.RA_PROBLEM = 1;
	    rtn_state = 1;
	    goto exit;
	}
	else
	    rtn_state = 0;
	    goto exit;
    }
    else
	status.Bits.CNTRL_COMM_ERR = 0;
    
    nodeptr = motor_info->motor_motion;

    /* Calculate the drive number */
    drive = signal / PMNC8750_NUM_MOTORS;
    drive_motor = signal % PMNC8750_NUM_MOTORS;

    /* Setup equality test by default invalid values*/
    scan_drive= scan_motor = -1;

    /* Find the currently Select drive motor */
    p = cntrl->chan_select_string[drive];
    if (sscanf(p, "A%d=%d",&scan_drive, &scan_motor) == 2)
    {
        Debug(5, "set_status(): Selected drive %d: chan = %x\n", scan_drive, scan_motor);
    }
    else
    {
        Debug(1, "set_status(): Selected drive %d: INVALID chan string %s\n", drive, p);
    }


    /* Set defaults in-case motor is not selected */
    motorData = cntrl->last_position[signal];
    mstat.All = 0;

    /* Status and Position input only apply to selected motor so
     *  only parse the strings if motor is selected */
    if (drive_motor == scan_motor)
    {
        /* 
	 * Index to Drive entry and parse the status string
	 * Status string format: [0] SYSTEM STATUS 0x0 
         *                       [1] A1=0xhh were hh is status byte
         *                       [2] A2=0xhh were hh is status byte
	 *                        .
	 */
    
          p = cntrl->status_string[scan_drive];
	  if (sscanf(p, "A%d=0x%x",&scan_drive, &scan_mstat) == 2)
	    {
	      mstat.All = scan_mstat;
	      Debug(5, "set_status(): drive %d - status byte = %x\n", scan_drive, scan_mstat);
	    }
         else
	     Debug(1, "set_status(): drive %d - INVALID status string %s\n", drive, p);

        /* 
	 * Parse motor position
	 * Position string format: A1=##### <CR>A2=###### <CR>A3=##### .....
	 * Skip to substring for this motor, convert to double
	 */

    
        p = cntrl->position_string[scan_drive-1];  /* Drive numbers start at 1 */
 	if (sscanf(p, "A%d=%ld",&scan_drive, &motorData) == 2)
	  {
	    Debug(6, "set_status(): drive %d - position = %ld\n", scan_drive, motorData);
	  }
	else
	    Debug(1, "set_status(): drive %d - INVALID position string %s\n", drive, p);
    }
    else 
    {
        motorData = cntrl->last_position[signal] = 0;
        Debug(5, "set_status(): NOT SELECTED drive %d, motor %d\n", drive, drive_motor);
    }



    /* Interpret data and move it into global area */

    if (mstat.Bits.inMotion == false)
      status.Bits.RA_DONE = 1;
    else
      status.Bits.RA_DONE = 0;

    /* Drive Enabled */
    status.Bits.EA_POSITION = (mstat.Bits.powerOn) ? 1: 0; 

    /* Limit Switch Status - NOT AVAILABLE */
    status.Bits.RA_PLUS_LS = 0;
    status.Bits.RA_MINUS_LS = 0;
    status.Bits.RA_HOME = 0;

    /* encoder status - NOT AVAILABLE  */
    status.Bits.EA_SLIP 	= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;


    Debug(2, "set_status(): Axis:%d, New Pos: %ld, Last Pos: %ld", signal, motorData, cntrl->last_position[signal]);

    if (motorData == cntrl->last_position[signal])
	motor_info->no_motion_count++;
    else
    {
	motor_info->position += (motorData - cntrl->last_position[signal]);
	if (motor_state[card]->motor_info[signal].encoder_present == YES)
	    motor_info->encoder_position += (epicsInt32) (motorData - cntrl->last_position[signal]);
	else
	    motor_info->encoder_position = 0;

        cntrl->last_position[signal] = motorData;
	motor_info->no_motion_count = 0;
    }

    Debug(1, ", Accum Pos: %d\n", motor_info->position); 

    /* Use sign of motor position (always relative) for direction */
    status.Bits.RA_DIRECTION = (motorData >= 0) ? 1 : 0;

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    status.Bits.RA_PROBLEM = 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!plusdir)
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move processing. */
    if ((status.Bits.RA_DONE || ls_active == true) )
    {
      cntrl->last_position[signal] = 0; /* Reset last position after each move */

      if (drive_motor == scan_motor)
	{
	/* Clear position on selected motor */
	sprintf(buff, "CHL A%d=%d", scan_drive, scan_motor);
	send_noreply_mess(card, buff, NULL);
	}

      if (nodeptr != 0)
	{
	  /* Allways turn joystick on after a move */
	  /* strcpy(buff, JOY_ON);   */
	    
	  if (nodeptr->postmsgptr != 0)  /* Test for post move record string */
	    {
	      strcpy(buff, nodeptr->postmsgptr);
	      send_noreply_mess(card, buff, NULL);
	    }

	  nodeptr->postmsgptr = NULL;
	}
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the                             */
/* PMNC8750 motor controller board with a  handshake */
/* on the return of the prompt '>' character 	     */
/* send_noreply_mess()			             */
/*****************************************************/
STATIC RTN_STATUS send_noreply_mess(int card, char const *com, char *inchar)
{
    struct PMNCcontroller *cntrl;
    char reply_buff[BUFF_SIZE];  /* Used to receive prompt after multiple commands */ 
    int size;
    size_t nwrite;
    size_t nread = 0;
    asynStatus status;
    int eomReason;


    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvPMNC8750:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return (OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvPMNC8750:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (inchar != NULL)
    {
	errlogPrintf("drvPMNC8750:send_mess() - invalid argument = %s\n", inchar);
	return(ERROR);
    }

    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;

     /* Set Reply End-of-string - temporary */
     pasynOctetSyncIO->setInputEos(cntrl->pasynUser,NOREPLY_EOS,strlen(NOREPLY_EOS));

    /* Perform atomic write/read operation  */
    status = pasynOctetSyncIO->writeRead(cntrl->pasynUser, com, size, 
                                         reply_buff, BUFF_SIZE-1,
                                         SERIAL_TIMEOUT, &nwrite, &nread, &eomReason);
    Debug(2, "send_noreply_mess(): message = %s, status = %d\n ", com, status);

    /* Reset Reply End-of-string to default */
    pasynOctetSyncIO->setInputEos(cntrl->pasynUser,REPLY_EOS,strlen(REPLY_EOS));

    if (status != asynSuccess)
      return(ERROR);
    else
      return (OK);
}
/*****************************************************/
/* send a message to the PMNC8750 board		     */
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
	errlogMessage("drvPMNC8750:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvPMNC8750:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (name != NULL)
    {
	errlogPrintf("drvPMNC8750:send_mess() - invalid argument = %s\n", name);
	return(ERROR);
    }

    Debug(2, "send_mess(): message = %s\n", com);

    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, com, size, 
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
 *	    NOTE: The PMNC8750 sometimes responds to an MS command with an error
 *		message (see PMNC8750 User's Manual Appendix A).  This is an
 *		unconfirmed PMNC8750 bug.  Retry read if this is a Hard Travel
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
	return(ERROR);

    cntrl = (struct PMNCcontroller *) motor_state[card]->DevicePrivate;

    if (flag != FLUSH) {
        flush = 0;
	timeout	= SERIAL_TIMEOUT;
    }
    if (flush) status = pasynOctetSyncIO->flush(cntrl->pasynUser);

    if (flag == 1)
      // Temporary Change to EOS
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,NOREPLY_EOS,strlen(NOREPLY_EOS));

    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
	com[0] = '\0';
	nread = 0;
    }
    else
	com[nread-1] = '\0';

    if (flag == 1)
      // Reset EOS to default 
      pasynOctetSyncIO->setInputEos(cntrl->pasynUser,REPLY_EOS,strlen(REPLY_EOS));

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* PMNC8750Setup()                                     */
/*****************************************************/
RTN_STATUS
PMNC8750Setup(int num_cards,	/* maximum number of controllers in system.  */
	      int num_drivers,	/* maximum number of drivers per controller */
	      int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PMNC8750_NUM_CARDS)
	PMNC8750_num_cards = PMNC8750_NUM_CARDS;
    else
	PMNC8750_num_cards = num_cards;

    if (num_drivers < 1 || num_drivers > PMNC8750_NUM_DRIVERS)
	PMNC8750_num_drivers = PMNC8750_NUM_DRIVERS;
    else
	PMNC8750_num_drivers = num_drivers;


    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before PMNC8750Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(PMNC8750_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PMNC8750_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PMNC8750Config()                                    */
/*****************************************************/
RTN_STATUS
PMNC8750Config(int card,		/* card being configured */
            const char *port,   /* asyn port name */
            int address)        /* asyn address (GPIB) */
{
    struct PMNCcontroller *cntrl;

    if (card < 0 || card >= PMNC8750_num_cards)
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
    int card_index, motor_index;
    char axis_pos[BUFF_SIZE];
    char buff[BUFF_SIZE];
    char *bufptr;
    int total_axis = 0;
    RTN_STATUS rtnStatus;
    asynStatus success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */
    
    /* Check for setup */
    if (PMNC8750_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PMNC8750_num_cards; card_index++)
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
	  int retry = 0;

	      /* Set command End-of-string */
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser,REPLY_EOS,strlen(REPLY_EOS));
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser,CMND_EOS,strlen(CMND_EOS));
	    

	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    pasynOctetSyncIO->flush(cntrl->pasynUser);
    

	    do
	    {
	      rtnStatus = send_noreply_mess(card_index, MOTOR_OFF, (char) NULL);
	      retry++;
		/* Return value is length of response string */
	    } while(rtnStatus == ERROR && retry < 3);

	}

	if (success_rtn == asynSuccess && rtnStatus == OK)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    send_noreply_mess(card_index, STOP_ALL, NULL);	/* Stop all motors */
	    send_noreply_mess(card_index, INIT_ALL, NULL);	/* Initalize all devices */
	    send_noreply_mess(card_index, MOTOR_ON, NULL);	/* Initalize all devices */

	    send_mess(card_index, GET_IDENT, NULL);	/* Read controller ID string */
	    recv_mess(card_index, buff, 0);
	    strncpy(brdptr->ident, &buff[0], MAX_IDENT_LEN);  /* Skip "VE" */
	    recv_mess(card_index, buff, 1); /* Get Prompt */


	    /* Set Motion Master model indicator. */
	    bufptr = strstr(brdptr->ident, "Version 1.");
	    if (bufptr == NULL)
	    {
		errlogPrintf("drvPMNC8750.c:motor_init() - unknown version = %s\n", brdptr->ident);
		motor_state[card_index] = (struct controller *) NULL;
		continue;
	    }
          
            if (true)  /* Model check - no way to check model */
	      cntrl->model = PMNC8750;
	    else
	    {
	      errlogPrintf("drvPMNC8750:motor_init() - invalid model = %s\n", brdptr->ident);
	      return(ERROR);
	    }


            total_axis = 0;
	    send_mess(card_index, GET_MPV, NULL);
	    /* Number of return strings will tell us how many axes this controller has */
	    while (recv_mess(card_index, axis_pos, 0))
	      {
		brdptr->motor_info[total_axis].motor_motion = NULL;
		Debug(3, "motor_init(): Next Axis %s\n", axis_pos);
		total_axis++;
	      }

            
	    brdptr->total_axis = total_axis;
            cntrl->total_drivers = total_axis / PMNC8750_NUM_MOTORS;
            Debug(2, "motor_init(): Total Axis=%d\n", brdptr->total_axis);

	    start_status(card_index);

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->encoder_present = NO;
		motor_info->position = 0;

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

    epicsThreadCreate((char *) "PMNC8750_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

/*---------------------------------------------------------------------*/
