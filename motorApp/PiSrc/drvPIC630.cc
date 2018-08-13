/* File: drvPIC630.cc                     */

/* Device Driver Support routines for motor */
/*
 *      Original Author: Kurt Goetze
 *      Date: 02-07-2005
 *
 * Modification Log:
 * -----------------
 * .00  02-07-2005  kag  initialized from drvMicos.c
 */

/* The PI C-630 controller is a 3-axis controller/driver.  Up to 3 controllers
 * can be daisy-chained on one serial port (in this code this is considered a "card").
 * More than 1 "card" can be configured, but each will need its own serial port.
 * This means that up to 9 axes are controllable per serial port, addressed 1-9.
 * Each axis has a current setting, which is handled by PIC630_config and motor_init.
 *
 * The Controllers MUST BE ON at bootup or this code will error out.
 */
 
#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <stdlib.h>
#include <errlog.h>
#include "motor.h"
#include "drvPIC630.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define WAIT 1
#define BUFF_SIZE 100       /* Maximum length of string to/from PIC630 */

/*----------------debugging-----------------*/
volatile int drvPIC630debug = 0;
extern "C" {epicsExportAddress(int, drvPIC630debug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvPIC630debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* Debugging notes:
 *   drvPIC630debug == 0  No debugging information is printed
 *   drvPIC630debug >= 1  Warning information is printed
 *   drvPIC630debug >= 2  Time-stamped messages are printed for each string 
 *                       sent to and received from the controller
 *   drvPIC630debug >= 3  Additional debugging messages
 */    

volatile int PIC630_num_cards = 0;
volatile int PIC630_num_axis = 0;
volatile int PIC630_current[9];  /* current settings per axis */

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"

/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, const char *, const char *);
static void start_status(int);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PIC630_access =
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
} drvPIC630 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPIC630);}

static struct thread_args targs = {SCAN_RATE, &PIC630_access};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIC630_num_cards <=0)
	    printf("    No PIC630 controllers configured.\n");
    else
    {
	    for (card = 0; card < PIC630_num_cards; card++)
	    {
	        struct controller *brdptr = motor_state[card];
	        if (brdptr == NULL)
		        printf("    PIC630 controller %d connection failed.\n", card);
	        else
	        {
		        struct PIC630Controller *cntrl;
		        cntrl = (struct PIC630Controller *) brdptr->DevicePrivate;
                        printf("    PIC630 controller #%d, port=%s, id: %s \n", card, cntrl->asyn_port, brdptr->ident);
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
    if (PIC630_num_cards <= 0)
    {
        Debug(1, "init(): PIC630 driver disabled. PIC630Setup() missing from startup script.\n");
    }
    return ((long) 0);
}

static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


static void start_status(int card)
{
    /* The PIC630 cannot query status or positions of all axes with a
     * single command.  This needs to be done on an axis-by-axis basis,
     * so this function does nothing
     */
}


/**************************************************************
 * Query position and status for an axis
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    char command[BUFF_SIZE];
    char response[BUFF_SIZE];
    char cStatus;
    int rtn_state;
    long motorData;
    char buff[BUFF_SIZE];
    bool plusdir, ls_active = false;
    msta_field status;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    signal++;
    /* Request the status of this motor */
    sprintf(command, "%dTS", signal);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);

    /* The response string is of the form "nTS:N" where N is an int (0-255) 
       bit0 (0x01) = moving
       bit1 (0x02) = reference signal flag
       bit2 (0x04) = pos. limit (0 = limit tripped)
       bit3 (0x08) = neg. limit (0 = limit tripped)
       bit4 (0x10) = command error
       bit5 (0x20) = profile error
       bit6 (0x40) = not used
       bit7 (0x80) = Estop (0 = Estop) */

    cStatus = (char) atoi(&response[4]);

    /* check to see if motor is moving */
	status.Bits.RA_DONE = (cStatus & 0x01) ? 0 : 1;

    /* Request the position of this motor, find out which direction we're going */
    /* The response string is of the form "1TP:1000"                            */
	
    sprintf(command, "%dTP", signal);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);
	
    motorData = atoi(&response[4]);
	
    if (motorData == motor_info->position)
    {
        if (nodeptr != 0)	/* Increment counter only if motor is moving. */
        motor_info->no_motion_count++;
    }
    else 
    {
       status.Bits.RA_DIRECTION = (motorData >= motor_info->position) ? 1 : 0;
        motor_info->position = motorData;
        motor_info->no_motion_count = 0;
    }

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    /* check limits, set indicators */
    if (!(cStatus & 0x04))  /* if +lim */
	{  
        status.Bits.RA_PLUS_LS = 1;
        if (plusdir == true)
            ls_active = true;
    }
    else
        status.Bits.RA_PLUS_LS = 0;

    if (!(cStatus & 0x08))  /* if -lim */
	{ 
        status.Bits.RA_MINUS_LS = 1;
        if (plusdir == false)
            ls_active = true;
    }
    else
        status.Bits.RA_MINUS_LS = 0;    

    /* encoder status */
    status.Bits.EA_SLIP	      = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME	      = 0;

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
        /* The PIC630 will send back a response for a 'set' command */
        recv_mess(card, buff, WAIT); 
        nodeptr->postmsgptr = NULL;
    }

    motor_info->status.All = status.All;
    return (rtn_state);
}

/*****************************************************/
/* send a message to the PIC630 board                 */
/* send_mess()                                       */
/*****************************************************/
static RTN_STATUS send_mess(int card, const char *com, const char *name)
{
    char buff[BUFF_SIZE];
    char inp_buff[BUFF_SIZE];
    size_t nwrite;
    struct PIC630Controller *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("send_mess - invalid card #%d\n", card);
        return (ERROR);
    }

    /* If the string is NULL just return */
    if (strlen(com) == 0) return(OK);
    cntrl = (struct PIC630Controller *) motor_state[card]->DevicePrivate;

    strcpy(buff, com);
    Debug(2, "send_mess: sending message to card %d, message=%s\n", card, buff);
	pasynOctetSyncIO->write(cntrl->pasynUser, buff, strlen(buff), COMM_TIMEOUT, &nwrite);

    /* This thing always echos everything sent to it. Read this response. */
    recv_mess(card, inp_buff, WAIT); 

    return (OK);
}


/*****************************************************/
/* Read a response string from the PIC630 board */
/* recv_mess()                                       */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    double timeout;
    size_t nread = 0;
    int eomReason;
    int flush;
    struct PIC630Controller *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("recv_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct PIC630Controller *) motor_state[card]->DevicePrivate;

    Debug(3, "recv_mess entry: card %d, flag=%d\n", card, flag);
    if (flag == FLUSH)
	{
        flush = 1;
        timeout = 0.;
    }
    else
    {
        flush = 0;
        timeout = COMM_TIMEOUT;
    }
    if (flush) pasynOctetSyncIO->flush(cntrl->pasynUser);
    pasynOctetSyncIO->read(cntrl->pasynUser, com, MAX_MSG_SIZE,
                                    timeout, &nread, &eomReason);

    if (nread < 1) com[0] = '\0';

    if (nread > 0) {
        Debug(2, "recv_mess: card %d, message = \"%s\"\n", card, com);
    }
    if (nread == 0) {
        if (flag != FLUSH)  {
            Debug(1, "recv_mess: card %d ERROR: no response\n", card);
        } else {
            Debug(3, "recv_mess: card %d flush returned no characters\n", card);
        }
    }
    return (nread);
}

/*****************************************************/
/* Setup system configuration                        */
/* PIC630Setup()                                     */
/*****************************************************/
RTN_STATUS
PIC630Setup(int num_cards,      /* maximum number of "controllers" in system */
            int num_channels,   /* max number of drivers            */
            int scan_rate)      /* polling rate - 1/60 sec units */
{
    int itera;

    if (num_cards < 1 || num_cards > PIC630_NUM_CARDS)
        PIC630_num_cards = PIC630_NUM_CARDS;
    else
        PIC630_num_cards = num_cards;

    if (num_channels < 1 || num_channels > PIC630_NUM_AXIS)
        PIC630_num_axis = PIC630_NUM_AXIS;
    else
        PIC630_num_axis = num_channels;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
        targs.motor_scan_rate = scan_rate;
    else
        targs.motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structures.  Note this must be done
    * before PIC630Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem since this is just
    * an array of pointers.
    */
    motor_state = (struct controller **) malloc(PIC630_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < PIC630_num_cards; itera++)
        motor_state[itera] = NULL;
    return (OK);
}

/*****************************************************/
/* Configure a controller                            */
/* PIC630Config()                                    */
/*****************************************************/
RTN_STATUS
PIC630Config(int card,          /* "controller" being configured */
            const char *name,   /* port name for asyn */
            int cur0, int cur1, int cur2, int cur3, int cur4,  /* see below */
            int cur5, int cur6, int cur7, int cur8)
{
    struct PIC630Controller *cntrl;

    if (card < 0 || card >= PIC630_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIC630Controller));
    cntrl = (struct PIC630Controller *) motor_state[card]->DevicePrivate;
    strcpy(cntrl->asyn_port, name);

    /* This is a current setting for each axis that will be sent during motor_init.
       Valid settings are:  0=OFF, 1=100mA, 2=200mA, ... 8=800mA.  */
    PIC630_current[0] = cur0;
    PIC630_current[1] = cur1;
    PIC630_current[2] = cur2;
    PIC630_current[3] = cur3;
    PIC630_current[4] = cur4;
    PIC630_current[5] = cur5;
    PIC630_current[6] = cur6;
    PIC630_current[7] = cur7;
    PIC630_current[8] = cur8;
    return (OK);
}

/*****************************************************/
/* initialize all software and hardware              */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()                                      */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct PIC630Controller *cntrl;
    int card_index, motor_index;
    char cmd[BUFF_SIZE];
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status = 0;
    int i;
    asynStatus success_rtn;
    static const char output_terminator[] = "\n";
    static const char  input_terminator[] = "\n";

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIC630_num_cards <= 0)
    {
        Debug(1, "motor_init: *PIC630 driver disabled*\n");
        Debug(1, "PIC630Setup() is missing from startup script.\n");
        return (ERROR);
    }
    for (card_index = 0; card_index < PIC630_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;
        brdptr = motor_state[card_index];
        brdptr->ident[0] = 0;	/* No controller identification message. */
        /* device echos? then set to true. else false */
        brdptr->cmnd_response = false;
        total_cards = card_index + 1;
        cntrl = (struct PIC630Controller *) brdptr->DevicePrivate;

        /* Initialize communications channel */

        success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
						&cntrl->pasynUser, NULL);

        if (success_rtn == asynSuccess)
        {
            int retry = 0;

            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser, output_terminator,
					   strlen(output_terminator));
            pasynOctetSyncIO->setInputEos(cntrl->pasynUser, input_terminator,
					  strlen(input_terminator));

            /* Each "controller chain" can have max 9 axes. */
            total_axis = PIC630_num_axis;
            brdptr->total_axis = total_axis;
			
            /* flush any junk at input port - should not be any data available */
            pasynOctetSyncIO->flush(cntrl->pasynUser);
			
            /* Send a message to each PIC630 motor.  See if controller responds */
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                do
                {
                    sprintf(cmd, "%dTS", (motor_index + 1));
                    send_mess(card_index, cmd, 0);
                    status = recv_mess(card_index, buff, WAIT);
                    retry++;
                    /* Return value is length of response string */
                } while(status == 0 && retry < 3);
                if (status == 0) break;
            }
        }


        if (success_rtn == asynSuccess && status > 0)
        {
            brdptr->localaddr = NULL;
            brdptr->motor_in_motion = 0;
            /* how did this get here? brdptr->cmnd_response = true; */

            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                brdptr->motor_info[motor_index].motor_motion = NULL;
                /* Wish I could turn off echo, can't :(  */
                /* Set motor current */
                sprintf(buff,"%dDC%d", (motor_index + 1),PIC630_current[motor_index]);
                for (i=0; i<=8; i++)
                    Debug(1, "PIC630_current[%d] = %d\n",i,PIC630_current[i]);
                send_mess(card_index, buff, 0);

                /* Stop motor */
                sprintf(buff,"%dAB", (motor_index + 1));
                send_mess(card_index, buff, 0);
                strcpy(brdptr->ident, "PIC630");

                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
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

    Debug(3, "motor_init: spawning motor task\n");

    epicsThreadCreate((char *) "PIC630_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}
