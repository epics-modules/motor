/* File: drvPM304.c                     */
/* Version: 2.01                        */
/* Date Last Modified: 10/26/99         */


/* Device Driver Support routines for motor */
/*
 *      Original Author: Mark Rivers
 *      Date: 11/20/98
 *
 * Modification Log:
 * -----------------
 * .01  11-20-98   mlr  initialized from drvMM4000
 * .02  09-29-99   mlr  Converted to motor record V4.04
 * .03  10-26-99   mlr  Minor fixes for motor record V4.0
 * .04  08-16-00   mlr  Fixed serious problem with limits - they were not
 *                      correct, bring extract from wrong character in response
 *                      Minor fixes to avoid compiler warnings
 * .05  11-27-01   mlr  Added global variable drvPM304ReadbackDelay.  This is a
 *                      double time in seconds to wait after the PM304 says the move
 *                      is complete before reading the encoder position the final
 *                      time.
 * .06  07-03-2002 rls  replaced RA_OVERTRAVEL with RA_PLUS_LS and RA_MINUS_LS
 */


#include        <vxWorks.h>
#include        <stdioLib.h>
#include        <sysLib.h>
#include        <string.h>
#include        <taskLib.h>
#include        <tickLib.h>
#include        <rngLib.h>
#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbAccess.h>
#include        <fast_lock.h>
#include        <recSup.h>
#include        <devSup.h>
#include        <drvSup.h>
#include        <errMdef.h>
#include        <logLib.h>

#include        "motor.h"
#include        "drvPM304.h"
#include        "gpibIO.h"
#include        "serialIO.h"

#define STATIC static

#define WAIT 0

#define SERIAL_TIMEOUT 2000 /* Command timeout in msec */

#define BUFF_SIZE 100       /* Maximum length of string to/from PM304 */

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvPM304ReadbackDelay = 0.;

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};


#ifdef NODEBUG
#define Debug(L,FMT,V) ;
#else
#define Debug(L,FMT,V...) {  if(L <= drvPM304Debug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,##V); } }
#endif

/* Debugging notes:
 *   drvPM304Debug == 0  No debugging information is printed
 *   drvPM304Debug >= 1  Warning information is printed
 *   drvPM304Debug >= 2  Time-stamped messages are printed for each string 
 *                       sent to and received from the controller
 *   drvPM304Debug >= 3  Additional debugging messages
 */    

int PM304_num_cards = 0;
volatile int drvPM304Debug = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC int send_mess(int card, const char *com, char c);
STATIC void start_status(int card);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PM304_access =
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
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvPM304 = {2, report, init};




/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
  int card;

  if (PM304_num_cards <=0)
    printf("    NO PM304 controllers found\n");
  else
    {
      for (card = 0; card < PM304_num_cards; card++)
          if (motor_state[card])
             printf("    PM304 controller %d, id: %s \n",
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
    if (PM304_num_cards <= 0)
    {
        Debug(1, "init: *PM304 driver disabled*\n");
        Debug(1, "PM304Setup() is missing from startup script.\n");
        return (ERROR);
    }

    return ((long) 0);
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
    /* The PM304 cannot query status or positions of all axes with a
     * single command.  This needs to be done on an axis-by-axis basis,
     * so this function does nothing
     */
}


/**************************************************************
 * Query position and status for an axis
 * set_status()
 ************************************************************/

STATIC int set_status(int card, int signal)
{
    register struct mess_info *motor_info;
    char command[BUFF_SIZE];
    char response[BUFF_SIZE];
    struct mess_node *nodeptr;
    int rtn_state;
    long motorData;
    char buff[BUFF_SIZE];
    BOOLEAN ls_active = OFF;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    /* Request the status of this motor */
    sprintf(command, "%dOS;", signal+1);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);
    /* The response string is an eight character string of ones an zeroes */

    if (strcmp(response, "00000000") == 0)
        motor_info->status &= ~RA_DONE;
    else {
        motor_info->status |= RA_DONE;
        if (drvPM304ReadbackDelay != 0.)
            taskDelay((int)(drvPM304ReadbackDelay * sysClkRateGet()));
    }

    if (response[2] == '1')
        motor_info->status |= RA_PROBLEM;
    else
        motor_info->status &= ~RA_PROBLEM;

    motor_info->status &= ~(RA_PLUS_LS | RA_MINUS_LS);
    if (response[1] == '1') {
        motor_info->status |= RA_PLUS_LS;
        motor_info->status |= RA_DIRECTION;
	ls_active = ON;
    }
    if (response[0] == '1') {
        motor_info->status |= RA_MINUS_LS;
        motor_info->status &= ~RA_DIRECTION;
	ls_active = ON;
    }

    /* encoder status */
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_POSITION;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;

    /* Request the position of this motor */
    sprintf(command, "%dOA;", signal+1);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);
    /* Parse the response string which is of the form "AP=10234" */
    motorData = atoi(&response[3]);

    if (motorData == motor_info->position)
        motor_info->no_motion_count++;
    else
        {
            motor_info->position = motorData;
            motor_info->encoder_position = motorData;
            motor_info->no_motion_count = 0;
        }

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0.;

    if (!(motor_info->status & RA_DIRECTION))
        motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == ON ||
		 (motor_info->status & (RA_DONE | RA_PROBLEM))) ? 1 : 0;

    /* Test for post-move string. */
    if ((motor_info->status & RA_DONE || ls_active == ON) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
        strcpy(buff, nodeptr->postmsgptr);
        strcat(buff, "\r");
        send_mess(card, buff, NULL);
        /* The PM304 always sends back a response, read it and discard */
        recv_mess(card, buff, WAIT);
        nodeptr->postmsgptr = NULL;
    }

    return (rtn_state);
}


/*****************************************************/
/* send a message to the PM304 board                 */
/* send_mess()                                       */
/*****************************************************/
STATIC int send_mess(int card, const char *com, char c)
{
    char *p, *tok_save;
    char buff[BUFF_SIZE];
    char response[BUFF_SIZE];
    int len=0;
    struct PM304controller *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("send_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;

    /* Device support can send us multiple commands separated with ';'
     * characters.  The PM304 cannot handle more than 1 command on a line
     * so send them separately */
    for (p = strtok_r(com, ";", &tok_save);
                ((p != NULL) && (strlen(p) != 0));
                p = strtok_r(NULL, ";", &tok_save)) {
        strcpy(buff, p);
        strcat(buff, OUTPUT_TERMINATOR);
        Debug(2, "%.2f : send_mess: sending message to card %d, message=%s\n",
                    tickGet()/60., card, buff);
        /* The PM304 always sends a response string to every command. There
           could be some stale characters in the input buffer - flush them. */
        do recv_mess(card, response, FLUSH); while (strlen(response) != 0);
	serialIOSend(cntrl->serialInfo, buff, strlen(buff), SERIAL_TIMEOUT);
    }

    return (len);
}



/*****************************************************/
/* receive a message from the PM304 board           */
/* recv_mess()                                       */
/*****************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    int timeout;
    int len=0;
    struct PM304controller *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("resv_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
        timeout = 0;
    else
        timeout = SERIAL_TIMEOUT;
    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE,
                       INPUT_TERMINATOR, timeout);

    /* The response from the PM304 is terminated with CR/LF.  Remove these */
    if (len < 2) com[0] = '\0'; else com[len-2] = '\0';
    if (len > 0) {
        Debug(2, "%.2f : recv_mess: card %d, message = \"%s\"\n", 
            tickGet()/60., card, com);
    }
    if (len == 0) {
        if (flag == WAIT)  {
            Debug(1, "%.2f: recv_mess: card %d ERROR: no response\n", 
                tickGet()/60., card);
        } else {
            Debug(3, "%.2f: recv_mess: card %d flush returned no characters\n", 
                tickGet()/60., card);
        }
    }
    return (len);
}



/*****************************************************/
/* Setup system configuration                        */
/* PM304Setup()                                     */
/*****************************************************/
int PM304Setup(int num_cards,   /* maximum number of controllers in system */
            int num_channels,   /* NOT USED            */
           int scan_rate)       /* polling rate - 1/60 sec units */
{
    int itera;

    if (num_cards < 1 || num_cards > PM304_NUM_CARDS)
        PM304_num_cards = PM304_NUM_CARDS;
    else
        PM304_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
        motor_scan_rate = sysClkRateGet() / scan_rate;
    else
        motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structure pointers.  Note this must be done
    * before PM304Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem since this is just
    * an array of pointers.
    */
    motor_state = (struct controller **) malloc(PM304_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < PM304_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;
    return (0);
}


/*****************************************************/
/* Configure a controller                            */
/* PM304Config()                                    */
/*****************************************************/
int PM304Config(int card,       /* card being configured */
            int addr1,          /* hideos_card for RS-232 */
            int addr2)          /* hideos_task for RS-232 */
{
    struct PM304controller *cntrl;

    if (card < 0 || card >= PM304_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PM304controller));
    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;
    cntrl->serial_card = addr1;
    strcpy(cntrl->serial_task, (char *) addr2);
    return (0);
}



/*****************************************************/
/* initialize all software and hardware              */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()                                      */
/*****************************************************/
STATIC int motor_init()
{
    struct controller *brdptr;
    struct PM304controller *cntrl;
    int card_index, motor_index, arg3, arg4;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status = 0;
    BOOLEAN errind;

    initialized = ON;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (PM304_num_cards <= 0)
    {
        Debug(1, "motor_init: *PM304 driver disabled*\n");
        Debug(1, "PM304Setup() is missing from startup script.\n");
        return (ERROR);
    }

    for (card_index = 0; card_index < PM304_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        cntrl = (struct PM304controller *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        errind = OFF;

        cntrl->serialInfo = serialIOInit(cntrl->serial_card,
                                         cntrl->serial_task);
        if (cntrl->serialInfo == NULL) 
            errind = ON;

        if (errind == OFF)
        {
            int retry = 0;

            /* Send a message to the board, see if it exists */
            /* flush any junk at input port - should not be any data available */
            do {
                recv_mess(card_index, buff, FLUSH);
            } while (strlen(buff) != 0);

            do
            {
                send_mess(card_index, "1OA;", 0);
                status = recv_mess(card_index, buff, WAIT);
                retry++;
                /* Return value is length of response string */
            } while(status == 0 && retry < 3);
        }

        if (errind == OFF && status > 0)
        {
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;

            /* Don't turn on motor power, too dangerous */
            /* send_mess(i, "1RSES;", buff); */
            send_mess(card_index, "1ST;", 0);     /* Stop motor */
            recv_mess(card_index, buff, WAIT);    /* Throw away response */
            send_mess(card_index, "1ID;", 0);    /* Read controller ID string */
            recv_mess(card_index, buff, WAIT);
            strcpy(brdptr->ident, buff);

            /* For now we assume that this controller has only 1 axis. */
            total_axis = 1;
            brdptr->total_axis = total_axis;
            brdptr->motor_info[total_axis-1].motor_motion = NULL;
            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];

                motor_info->status = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;

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
        arg3 = (int) (&PM304_access);
        arg4 = 0;
    }
    else
    {
        arg3 = (int) ((long) &PM304_access >> 16);
        arg4 = (int) ((long) &PM304_access & 0xFFFF);
    }
    Debug(3, "motor_init: spawning motor task\n");
    taskSpawn((char *) "tPM304", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
              motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}
