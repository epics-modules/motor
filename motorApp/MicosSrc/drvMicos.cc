/* File: drvMicos.cc                    */

/* Device Driver Support routines for Micos MoCo dc motor controller. */
/*
 *      Original Author: Kurt Goetze
 *      Date: 11/24/2003
 *
 * Modification Log:
 * -----------------
 * .00  11-24-2003   kag  initialized from drvMCB4B.c
 * .01  02-06-2004   rls  Eliminate erroneous "Motor motion timeout ERROR".
 * .02  02-12-2004   rls  Copied from drvMicos.c. Upgraded from R3.14.x
 * .03  02-17-2004   rls  Removed Debug calls to tickGet().
 * .04  07-12-2004   rls  Converted from MPF to asyn.
 */


#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "drvMicos.h"
#include "epicsExport.h"

#define WAIT 1

#define COMM_TIMEOUT 2	/* Command timeout in seconds. */

#define BUFF_SIZE 100       /* Maximum length of string to/from Micos */

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};


/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int drvMicosDebug = 0;
	#define Debug(l, f, args...) {if (l <= drvMicosDebug) printf(f, ## args);}
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

/* Debugging notes:
 *   drvMicosDebug == 0  No debugging information is printed
 *   drvMicosDebug >= 1  Warning information is printed
 *   drvMicosDebug >= 2  Time-stamped messages are printed for each string 
 *                       sent to and received from the controller
 *   drvMicosDebug >= 3  Additional debugging messages
 */    

volatile int Micos_num_cards = 0;
volatile int Micos_num_axis = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int card, const char *com, char c);
static void start_status(int card);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table Micos_access =
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
    &initialized
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvMicos = {2, report, init};

epicsExportAddress(drvet, drvMicos);

static struct thread_args targs = {SCAN_RATE, &Micos_access};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
  int card;

  if (Micos_num_cards <=0)
    printf("    NO Micos controllers found\n");
  else
    {
      for (card = 0; card < Micos_num_cards; card++)
          if (motor_state[card])
             printf("    Micos controller group %d, id: %s \n",
                   card,
                   motor_state[card]->ident);
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
    if (Micos_num_cards <= 0)
    {
        Debug(1, "init: *Micos driver disabled*\n");
        Debug(1, "MicosSetup() is missing from startup script.\n");
        return (ERROR);
    }

    return ((long) 0);
}

static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/*********************************************************
 * Read the status and position of all motors on a card
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/
static void start_status(int card)
{
    /* The Micos cannot query status or positions of all axes with a
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
    register struct mess_info *motor_info;
    char command[BUFF_SIZE];
    char response[BUFF_SIZE];
    struct mess_node *nodeptr;
    int rtn_state, i, j;
    long motorData;
    long bytes[7];
    char temp[5];
    char buff[BUFF_SIZE];
    bool ls_active = false;
    msta_field status;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    /* Request the moving status of this motor */

    /* MM4000 has a delay in case the motor is not done due to servo PID-related settling.. do we need this delay? */
    /* ...after some testing, it doesn't look like we need the delay */

    /* Get the motor status (ts) */
    sprintf(command, "%c%dts", CTLA, signal);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);
    /* The response string is of the form "byte0 byte1 ... byte6" */

    /* Convert ASCII characters to hex */
    temp[0]='0'; temp[1]='x'; temp[2]='0', temp[3]='0', temp[4]='\0';
    if (signal > 9) j = 5;
    else j = 4;
    for (i = 0; i < 7; i++) {
        temp[2] = response[j];
        temp[3] = response[j+1];
        bytes[i] = strtol(temp, (char **)NULL, 0);
        j += 3;
    }
    /* check to see if motor is moving */
    status.Bits.RA_DONE = (bytes[0] & 0x04) ? 1 : 0;

    /* check limits */
    status.Bits.RA_PLUS_LS = status.Bits.RA_MINUS_LS = 0;
    if ((bytes[5] & 0x04) & (bytes[3] & 0x04)) {  /* if +lim AND pos move */
	status.Bits.RA_PLUS_LS = 1;
	ls_active = true;
    }
    if ((bytes[5] & 0x01) & !(bytes[3] & 0x04)) {  /* if -lim AND neg move */
	status.Bits.RA_MINUS_LS = 1;
	ls_active = true;
    }

    /* encoder status */
    status.Bits.EA_SLIP	      = 0;
    status.Bits.EA_POSITION   = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME	      = 0;
    if ((bytes[3] & 0x08) | (bytes[3] & 0x40)) {
        printf("drvMicos: set_status: EA_SLIP_STALL = 1, %ld\n", bytes[3]);
	status.Bits.EA_SLIP_STALL = 1;
    }

    /* Request the position of this motor */
    sprintf(command, "%c%dtp", CTLA, signal);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);
    /* The response string is of the form "0P0:+00001000" */
    if (signal > 9)
        motorData = atoi(&response[5]);
    else
        motorData = atoi(&response[4]);

    /* derive direction information */
    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	status.Bits.RA_DIRECTION = (bytes[3] & 0x04) ? 1 : 0;
        motor_info->position = motorData;
        motor_info->encoder_position = motorData;
        motor_info->no_motion_count = 0;
    }

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
        send_mess(card, buff, (char) NULL);
        /* The Micos will not send back a response for a 'set' command, don't need next line */
        /* recv_mess(card, buff, WAIT); */
        nodeptr->postmsgptr = NULL;
    }

    motor_info->status.All = status.All;
    return (rtn_state);
}


/*****************************************************/
/* send a message to the Micos board                 */
/* send_mess()                                       */
/*****************************************************/
static RTN_STATUS send_mess(int card, const char *com, char c)
{
    char buff[BUFF_SIZE];
    struct MicosController *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("send_mess - invalid card #%d\n", card);
        return(ERROR);
    }

    /* If the string is NULL just return */
    if (strlen(com) == 0) return(OK);
    cntrl = (struct MicosController *) motor_state[card]->DevicePrivate;

    strcpy(buff, com);
    strcat(buff, OUTPUT_TERMINATOR);
    Debug(2, "send_mess: sending message to card %d, message=%s\n", card, buff);
    cntrl = (struct MicosController *) motor_state[card]->DevicePrivate;
    pasynSyncIO->write(cntrl->pasynUser, buff, strlen(buff), COMM_TIMEOUT);

    return (OK);
}


/*****************************************************/
/* Read a response string from the Micos board */
/* recv_mess()                                       */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    int timeout;
    int flush = 0;
    int len=0;
    int eomReason;
    struct MicosController *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("recv_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct MicosController *) motor_state[card]->DevicePrivate;

    Debug(3, "recv_mess entry: card %d, flag=%d\n", card, flag);
    if (flag == FLUSH)
        timeout = 0;
    else
        timeout = COMM_TIMEOUT;
    
    len = pasynSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE, (char *) "\3",
                            1, flush, timeout, &eomReason);

    /* The response from the Micos is terminated with <CR><LF><ETX>.  Remove */
    if (len < 3) com[0] = '\0'; 
    else com[len-3] = '\0';
    if (len > 0) {
        Debug(2, "recv_mess: card %d, message = \"%s\"\n", card, com);
    }
    if (len == 0) {
        if (flag != FLUSH)  {
            Debug(1, "recv_mess: card %d ERROR: no response\n", card);
        } else {
            Debug(3, "recv_mess: card %d flush returned no characters\n", card);
        }
    }
    return (len);
}



/*****************************************************/
/* Setup system configuration                        */
/* MicosSetup()                                     */
/*****************************************************/
RTN_STATUS
MicosSetup(int num_cards,   /* maximum number of "controllers" in system */
           int num_channels,   /* max number of drivers            */
           int scan_rate)       /* polling rate - 1/60 sec units */
{
    int itera;

    if (num_cards < 1 || num_cards > MICOS_NUM_CARDS)
        Micos_num_cards = MICOS_NUM_CARDS;
    else
        Micos_num_cards = num_cards;

    if (num_channels < 1 || num_channels > MICOS_NUM_AXIS)
        Micos_num_axis = MICOS_NUM_AXIS;
    else
        Micos_num_axis = num_channels;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structure pointers.  Note this must be done
    * before MicosConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem since this is just
    * an array of pointers.
    */
    motor_state = (struct controller **) malloc(Micos_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < Micos_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;
    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* MicosConfig()                                    */
/*****************************************************/
RTN_STATUS
MicosConfig(int card,		/* "controller" being configured */
	    const char *name)	/* asyn server task name */
{
    struct MicosController *cntrl;

    if (card < 0 || card >= Micos_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MicosController));
    
    cntrl = (struct MicosController *) motor_state[card]->DevicePrivate;
    strcpy(cntrl->asyn_port, name);
    return(OK);
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
    struct MicosController *cntrl;
    int card_index, motor_index;
    char cmd[BUFF_SIZE];
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status = 0;
    bool errind;
    asynStatus success_rtn;

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (Micos_num_cards <= 0)
    {
        Debug(1, "motor_init: *Micos driver disabled*\n");
        Debug(1, "MicosSetup() is missing from startup script.\n");
        return (ERROR);
    }

    for (card_index = 0; card_index < Micos_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        cntrl = (struct MicosController *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        errind = false;
	success_rtn = pasynSyncIO->connect(cntrl->asyn_port, 0, &cntrl->pasynUser);
        
	if (success_rtn == asynSuccess)
        {
            int retry = 0;

            /* Each "controller" can have max 16 axes. */
            total_axis = Micos_num_axis;
            brdptr->total_axis = total_axis;

            /* flush any junk at input port - should not be any data available */
            do {
                recv_mess(card_index, buff, FLUSH);
            } while (strlen(buff) != 0);

            /* Send a message to each Micos driver.  See if it responds */
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                do
                {
                    sprintf(cmd, "%c%dts", CTLA, motor_index);
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
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;
            brdptr->cmnd_response = false;

            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                brdptr->motor_info[motor_index].motor_motion = NULL;
                /* turn off echo */
                sprintf(buff, "%c%def", CTLA, motor_index);
                send_mess(card_index, buff, 0);
                /* Don't turn on motor power, too dangerous */
		   /*sprintf(buff,"#%02dW=1", motor_index); */
                /* send_mess(card_index, buff, 0); */
                /* Stop motor */
                sprintf(buff,"%c%dab1", CTLA, motor_index);
                send_mess(card_index, buff, 0);
		   /* recv_mess(card_index, buff, WAIT);    Throw away response */
                strcpy(brdptr->ident, "MICOS");

                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;

                motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = YES;
		motor_info->status.Bits.GAIN_SUPPORT = 1;

                set_status(card_index, motor_index);  /* Read status of each motor */
            }

        }
        else
            motor_state[card_index] = (struct controller *) NULL;
    }

    Debug(3, "motor_init: spawning motor task\n");
    
    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "tMicos", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return (0);
}
