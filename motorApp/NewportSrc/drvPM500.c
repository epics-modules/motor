/* File: drvPM500.c 		   	*/
/* Version: 1.00		   	*/
/* Date Last Modified: 11/26/98	        */


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
 * .01  11-19-98        mlr     Initial development, based on drvMM4000.c
 * .02  06-02-00	rls	integrated into standard motor record
 */


#include	<vxWorks.h>
#include	<stdio.h>
#include	<sysLib.h>
#include	<string.h>
#include	<taskLib.h>
#include	<math.h>
#include        <rngLib.h>
#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<fast_lock.h>
#include	<recSup.h>
#include	<devSup.h>
#include        <errMdef.h>
#include	<logLib.h>

#include	"motor.h"
#include	"drvMMCom.h"
#include        "gpibIO.h"
#include        "serialIO.h"

#define STATIC static

#define READ_RESOLUTION ""
#define READ_STATUS     ""
#define READ_POSITION   ""
#define MOTOR_ON        ""
#define GET_IDENT       "SVN?"

#define OUTPUT_TERMINATOR	"\r"
#define INPUT_TERMINATOR	'\r'

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

#define GPIB_TIMEOUT	2000	/* Command timeout in msec. */
#define SERIAL_TIMEOUT	2000	/* Command timeout in msec. */

/*----------------debugging-----------------*/
#ifdef	DEBUG
    #define Debug(l, f, args...) { if(l<=drvPM500debug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

/* --- Local data. --- */
int PM500_num_cards = 0;
volatile int drvPM500debug = 0;
static char PM500_axis_names[PM500_NUM_CHANNELS] = {'X', 'Y', 'Z', 'A', 'B',
    'C', 'D', 'E', 'F', 'G', 'H', 'I'};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvPM500ReadbackDelay = 0.;


/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC int send_mess(int card, char const *com, char c);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

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
		switch (cntrl->port_type)
		{
		case RS232_PORT: 
		    printf("    PM500 controller %d port type = RS-232, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		case GPIB_PORT:
		    printf("    PM500 controller %d port type = GPIB, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		default:
		    printf("    PM500 controller %d port type = Unknown, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		}
	    }
	}
    }
    return (0);
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
    return ((long) 0);
}


STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

STATIC int set_status(int card, int signal)
{
    struct MMcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char axis_name, status_char, dir_char, buff[BUFF_SIZE],
	response[BUFF_SIZE];
    int status, rtn_state = 0;
    double motorData;

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    axis_name = PM500_axis_names[signal];

    /* Request the status and position of this motor */
    sprintf(buff, "%cR", axis_name);
    send_mess(card, buff, NULL);
    status = recv_mess(card, response, 1);
    if (status > 0)
	cntrl->status = NORMAL;
    else
    {
	cntrl->status = COMM_ERR;
	motor_info->status |= RA_PROBLEM;	/* This terminates the transaction. */
	rtn_state = 1;
	goto exit;
    }

    status_char = response[1];
    dir_char = response[2];
    motorData = atof(&response[2]) / cntrl->drive_resolution[signal];

    if (status_char == 'B')
	motor_info->status &= ~RA_DONE;
    else
    {
	motor_info->status |= RA_DONE;
/* TEMPORARY FIX, Mark Rivers, 2/1/99. The PM500 has reported that the
 * motor is done moving, which means that the "jerk time" is done.  However,
 * the axis can still be settling.  For now we put in a delay and poll the
 * motor position again. This is not a good long-term solution.
 */
	if (motor_info->pid_present == YES && drvPM500ReadbackDelay != 0.)
	{
	    taskDelay((int)(drvPM500ReadbackDelay * sysClkRateGet()));
	    send_mess(card, READ_POSITION, NULL);
	    recv_mess(card, cntrl->position_string, 1);
	}
    }

    if (status_char == 'E') 
        motor_info->status |= RA_PROBLEM;
    else
        motor_info->status &= ~RA_PROBLEM;

    motor_info->status &= ~RA_OVERTRAVEL;
    motor_info->status |= RA_DIRECTION;
    if (status_char == 'L')
    {
	motor_info->status |= RA_OVERTRAVEL;
	if (dir_char == '+')
	    motor_info->status |= RA_DIRECTION;
	else
	    motor_info->status &= ~RA_DIRECTION;
    }    

    motor_info->status &= ~RA_HOME;

    /* encoder status */
    motor_info->status &= ~EA_POSITION;
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */


    if (motorData == motor_info->position)
	motor_info->no_motion_count++;
    else
    {
	motor_info->position = NINT(motorData);
	if (motor_state[card]->motor_info[signal].encoder_present == YES)
	    motor_info->encoder_position = (int32_t) motorData;
	else
	    motor_info->encoder_position = 0;

	motor_info->no_motion_count = 0;
    }

    motor_info->status &= ~RA_PROBLEM;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!(motor_info->status & RA_DIRECTION))
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count ||
		 (motor_info->status & (RA_OVERTRAVEL | RA_DONE | RA_PROBLEM))) ? 1 : 0;

    /* Test for post-move string. */
    if ((motor_info->status & RA_DONE || motor_info->status & RA_OVERTRAVEL) &&
	 nodeptr != 0 && nodeptr->postmsgptr != 0)
    {
	strcpy(buff, nodeptr->postmsgptr);
	strcat(buff, "\r");
	send_mess(card, buff, NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    if (cntrl->status == COMM_ERR)
	motor_info->status |= CNTRL_COMM_ERR;
    else
	motor_info->status &= ~CNTRL_COMM_ERR;

    return(rtn_state);
}


/*****************************************************/
/* send a message to the PM500 board		     */
/* send_mess()			                     */
/*****************************************************/
STATIC int send_mess(int card, char const *com, char inchar)
{
    struct MMcontroller *cntrl;
    char local_buff[BUFF_SIZE];

    if (strlen(com) > MAX_MSG_SIZE)
    {
	logMsg((char *) "drvPM500.c:send_mess(); message size violation.\n",
	       0, 0, 0, 0, 0, 0);
	return (-1);
    }
    
    if (!motor_state[card])
    {
	logMsg((char *) "drvPM500.c:send_mess() - invalid card #%d\n", card,
	       0, 0, 0, 0, 0);
	return (-1);
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcpy(local_buff, com);
    strcat(local_buff, OUTPUT_TERMINATOR);

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    switch (cntrl->port_type)
    {
	case GPIB_PORT:
	    gpibIOSend(cntrl->gpibInfo, local_buff, strlen(local_buff), GPIB_TIMEOUT);
	    break;
	case RS232_PORT:
	    serialIOSend(cntrl->serialInfo, local_buff, strlen(local_buff), SERIAL_TIMEOUT);
	    break;
    }
    return (0);
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

STATIC int recv_mess(int card, char *com, int flag)
{
    struct MMcontroller *cntrl;
    int timeout = 0;
    int len = 0;

    /* Check that card exists */
    if (!motor_state[card])
	return (-1);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    switch (cntrl->port_type)
    {
	case GPIB_PORT:
	    if (flag != FLUSH)
		timeout	= GPIB_TIMEOUT;
	    len = gpibIORecv(cntrl->gpibInfo, com, BUFF_SIZE, INPUT_TERMINATOR,
			     timeout);
	    break;
	case RS232_PORT:
	    if (flag != FLUSH)
		timeout	= SERIAL_TIMEOUT;
	    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE,
			       INPUT_TERMINATOR, timeout);
	    break;
    }

    if (len <= 0)
    {
	com[0] = '\0';
	len = 0;
    }
    else
    {
	com[len-1] = '\0';
	/* Test for "system error" response. */
	if (strncmp(com, "SE", 2) == 0)
	    logMsg((char *) "recv_mess(): PM500 system error.\n",
		   0, 0, 0, 0, 0, 0);
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (len);
}


/*****************************************************/
/* Setup system configuration                        */
/* PM500Setup()                                     */
/*****************************************************/
int PM500Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PM500_NUM_CARDS)
	PM500_num_cards = PM500_NUM_CARDS;
    else
	PM500_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;

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

    return (0);
}


/*****************************************************/
/* Configure a controller                            */
/* PM500Config()                                    */
/*****************************************************/
int PM500Config(int card,	/* card being configured */
            PortType port_type,	/* GPIB_PORT or RS232_PORT */
	    int addr1,          /* = link for GPIB or hideos_card for RS-232 */
            int addr2)          /* GPIB address or hideos_task */
{
    struct MMcontroller *cntrl;

    if (card < 0 || card >= PM500_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MMcontroller));
    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    switch (port_type)
    {
    case GPIB_PORT:
        cntrl->port_type = port_type;
        cntrl->gpib_link = addr1;
        cntrl->gpib_address = addr2;
        break;
    case RS232_PORT:
        cntrl->port_type = port_type;
        cntrl->serial_card = addr1;
        strcpy(cntrl->serial_task, (char *) addr2);
        break;
    default:
        return (ERROR);
    }
    return (0);
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
    struct MMcontroller *cntrl;
    int card_index, motor_index, arg3, arg4;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status, digits;
    BOOLEAN errind;

    initialized = ON;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PM500_num_cards <= 0)
	return (ERROR);

    for (card_index = 0; card_index < PM500_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->cmnd_response = ON;
	total_cards = card_index + 1;
	cntrl = (struct MMcontroller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	errind = OFF;
	switch (cntrl->port_type)
	{
	    case GPIB_PORT:
		cntrl->gpibInfo = gpibIOInit(cntrl->gpib_link,
					     cntrl->gpib_address);
		if (cntrl->gpibInfo == NULL)
		    errind = ON;
		break;
	    case RS232_PORT:
		cntrl->serialInfo = serialIOInit(cntrl->serial_card,
						 cntrl->serial_task);
		if (cntrl->serialInfo == NULL)
		    errind = ON;
		break;
	}

	if (errind == OFF)
	{
	    /* flush any junk at input port - should not be any data available */
	    do
		recv_mess(card_index, buff, FLUSH);
	    while (strlen(buff) != 0);
    
	    /* Send a SCUM 1 command to put device in this mode. */
	    send_mess(card_index, "SCUM 1", NULL);
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
	    send_mess(card_index, "SENAINT $AF", NULL);
	    recv_mess(card_index, buff, 1);
    
	    /* Send a message and read response from controller to see if 
	     * it exists */
	    send_mess(card_index, GET_IDENT, NULL);
	    status = recv_mess(card_index, buff, 1);  
	    /* Return value is length of response string */
	}

	if (errind == OFF && status > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    send_mess(card_index, GET_IDENT, NULL);	/* Read controller ID string */
	    recv_mess(card_index, buff, 1);
	    strcpy(brdptr->ident, &buff[2]);  /* Skip "XD" */

            /* Figure out how many axes this controller has.
             * Do this by querying status of each axis in order */
            for (total_axis = 0; total_axis < PM500_NUM_CHANNELS; total_axis++)
	    {
 		brdptr->motor_info[total_axis].motor_motion = NULL;
		sprintf(buff, "%cSTAT?", PM500_axis_names[total_axis]);
                send_mess(card_index, buff, NULL);
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
  		char *firmware, axis_name = PM500_axis_names[motor_index];
		double res = 0.0;

                sprintf(buff, "%cCONFIG?", axis_name);
	        send_mess(card_index, buff, NULL);
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

		digits = -log10(cntrl->drive_resolution[motor_index]) + 2;
		if (digits < 1)
		    digits = 1;
		cntrl->res_decpts[motor_index] = digits;              		

		/* PM500 only supports DC motors. */
		motor_info->encoder_present = YES;
		motor_info->status |= EA_PRESENT;
		motor_info->pid_present = YES;
		motor_info->status |= GAIN_SUPPORT;

		motor_info->status = 0;
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

    motor_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
    any_motor_in_motion = 0;

    FASTLOCKINIT(&queue_lock);
    FASTLOCK(&queue_lock);
    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&queue_lock);

    FASTLOCKINIT(&freelist_lock);
    FASTLOCK(&freelist_lock);
    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&freelist_lock);

    if (sizeof(int) >= sizeof(char *))
    {
	arg3 = (int) (&PM500_access);
	arg4 = 0;
    }
    else
    {
	arg3 = (int) ((long) &PM500_access >> 16);
	arg4 = (int) ((long) &PM500_access & 0xFFFF);
    }
    taskSpawn((char *) "PM500_motor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}

