#include    <string.h>
#include    <math.h>
#include    <stdio.h>
#include    <epicsThread.h>
#include    <epicsString.h>
#include    <drvSup.h>
#include        "motor.h"
#include        "drvLinMot.h"
#include        "asynOctetSyncIO.h"
#include    "epicsExport.h"

#define STATIC static

#define TIMEOUT 2.0 /* Command timeout in sec */

#define BUFF_SIZE 200       /* Maximum length of string to/from LinMot */

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvLinMotReadbackDelay = 0.;

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};

/*----------------debugging-----------------*/
volatile int drvLinMotDebug = 0;
extern "C" {epicsExportAddress(int, drvLinMotDebug);}

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvLinMotDebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* Debugging notes:
 *   drvLinMotDebug == 0  No debugging information is printed
 *   drvLinMotDebug >= 1  Warning information is printed
 *   drvLinMotDebug >= 2  Time-stamped messages are printed for each string
 *                       sent to and received from the controller
 *   drvLinMotDebug >= 3  Additional debugging messages
 */

int LinMot_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/*----------------functions-----------------*/
STATIC int recv_mess(int card, char *buff, int len);
STATIC RTN_STATUS send_mess(int, const char *, char *);
STATIC int send_recv_mess(int card, const char *out, char *in);
STATIC void start_status(int card);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table LinMot_access =
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

struct drvLinMot_drvet
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvLinMot = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvLinMot);}

STATIC struct thread_args targs = {SCAN_RATE, &LinMot_access, 0.0};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
  int card;

  if (LinMot_num_cards <=0)
    printf("    NO LinMot controllers found\n");
  else
    {
      for (card = 0; card < LinMot_num_cards; card++)
          if (motor_state[card])
             printf("    LinMot controller %d, id: %s \n",
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
    if (LinMot_num_cards <= 0)
    {
        Debug(1, "init: *LinMot driver disabled*\n");
        Debug(1, "LinMotSetup() is missing from startup script.\n");
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
    /* The LinMot cannot query status or positions of all axes with a
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
    bool ls_active = false;
    struct LinMotController *cntrl;
    msta_field status;

    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    /* Request the status of this motor */
    sprintf(command, "%dOS;", signal+1);
    send_recv_mess(card, command, response);
    Debug(2, "set_status, status query, card %d, response=%s\n", card, response);

    status.Bits.RA_PLUS_LS = 0;
    status.Bits.RA_MINUS_LS = 0;

    /* The response string is an eight character string of ones and zeroes */
    
    if (response[3] == '0') {
        status.Bits.RA_DONE = 0;
    } else {
        status.Bits.RA_DONE = 1;
        if (drvLinMotReadbackDelay != 0.)
            epicsThreadSleep(drvLinMotReadbackDelay);
    }
    
    /* We used to check just response[2] for problem.
     * However, it turns out that in firmware version 6.15 that bit=1 for no problem,
     * but in 6.17 it is 0 for no problem!  Check the last 4 bits individually instead. */
    status.Bits.RA_PROBLEM = 0;
    if ((response[4] == '1') || 
        (response[5] == '1') || 
        (response[6] == '1') || 
        (response[7] == '1')) status.Bits.RA_PROBLEM = 1;
    
    if (response[1] == '1') {
        status.Bits.RA_PLUS_LS = 1;
        status.Bits.RA_DIRECTION = 1;
        ls_active = true;
    }
    if (response[0] == '1') {
        status.Bits.RA_MINUS_LS = 1;
        status.Bits.RA_DIRECTION = 0;
        ls_active = true;
    }

    /* encoder status */
    status.Bits.EA_SLIP       = 0;
    status.Bits.EA_POSITION   = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME       = 0;

    /* Request the position of this motor */
    sprintf(command, "%dOA;", signal+1);
    send_recv_mess(card, command, response);
    /* Parse the response string which is of the form "AP=10234" (LinMot) or 01:10234 (PM600)*/
    motorData = atoi(&response[3]);
    Debug(2, "set_status, position query, card %d, response=%s\n", card, response);

    if (motorData == motor_info->position)
    {
    if (nodeptr != 0)   /* Increment counter only if motor is moving. */
        motor_info->no_motion_count++;
    }
    else
    {
    status.Bits.RA_DIRECTION = (motorData >= motor_info->position) ? 1 : 0;
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
        strcat(buff, "\r");
        send_mess(card, buff, (char) NULL);
        nodeptr->postmsgptr = NULL;
    }

    motor_info->status.All = status.All;
    Debug(2, "set_status, return value=%d\n", rtn_state);
    return (rtn_state);
}


/*****************************************************/
/* send a message to the LinMot board                 */
/* this function should be used when the LinMot       */
/* response string should be ignorred.               */
/* It reads the response string, since the LinMot     */
/* always sends one, but discards it.                */
/* Note that since it uses serialIOSendRecv it       */
/* flushes any remaining characters in the input     */
/* ring buffer                                       */
/* send_mess()                                       */
/*****************************************************/
STATIC RTN_STATUS send_mess(int card, const char *com, char *name)
{
    char *p, *tok_save;
    char response[BUFF_SIZE];
    char temp[BUFF_SIZE];
    struct LinMotController *cntrl;
    size_t nwrite, nread;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("send_mess - invalid card #%d\n", card);
    return(ERROR);
    }

    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;

    /* Device support can send us multiple commands separated with ';'
     * characters.  The LinMot cannot handle more than 1 command on a line
     * so send them separately */
    strcpy(temp, com);
    for (p = epicsStrtok_r(temp, ";", &tok_save);
                ((p != NULL) && (strlen(p) != 0));
                p = epicsStrtok_r(NULL, ";", &tok_save)) {
        Debug(2, "send_mess: sending message to card %d, message=%s\n", card, p);
    pasynOctetSyncIO->writeRead(cntrl->pasynUser, p, strlen(p), response,
                BUFF_SIZE, TIMEOUT, &nwrite, &nread, &eomReason);
        Debug(2, "send_mess: card %d, response=%s\n", card, response);
    }

    return(OK);
}



/*****************************************************/
/* receive a message from the LinMot board            */
/* NOTE: This function is required by motordrvCom,   */
/* but it should not be used.  Use send_recv_mess    */
/* instead, since it sends a receives a message as   */
/* atomic operation.                                 */
/* recv_mess()                                       */
/*****************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    double timeout;
    char *pos;
    char temp[BUFF_SIZE];
    int flush;
    asynStatus status;
    size_t nread=0;
    int eomReason;
    struct LinMotController *cntrl;

    com[0] = '\0';
    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("resv_mess - invalid card #%d\n", card);
        return(-1);
    }

    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH) {
        flush = 1;
        timeout = 0;
    } else {
        flush = 0;
        timeout = TIMEOUT;
    }
    if (flush) status = pasynOctetSyncIO->flush(cntrl->pasynUser);
    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    /* The response from the LinMot is terminated with CR/LF.  Remove these */
    if (nread == 0) com[0] = '\0';
    if (nread > 0) {
        Debug(2, "recv_mess: card %d, flag=%d, message = \"%s\"\n", card, flag, com);
    }
    if (nread == 0) {
        if (flag != FLUSH)  {
            Debug(1, "recv_mess: card %d read ERROR: no response\n", card);
        } else {
            Debug(3, "recv_mess: card %d flush returned no characters\n", card);
        }
    }
    return(strlen(com));
}



/*****************************************************/
/* Send a message to the LinMot board and receive a   */
/* response as an atomic operation.                  */
/* This function should be used when the LinMot       */
/* response string is required.                      */
/* Note that since it uses serialIOSendRecv it       */
/* flushes any remaining characters in the input     */
/* ring buffer                                       */
/* send_recv_mess()                                  */
/*****************************************************/
STATIC int send_recv_mess(int card, const char *out, char *response)
{
    char *p, *tok_save;
    struct LinMotController *cntrl;
    char *pos;
    asynStatus status;
    size_t nwrite=0, nread=0;
    int eomReason;
    char temp[BUFF_SIZE];

    response[0] = '\0';

    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("send_recv_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;

    /* Device support can send us multiple commands separated with ';'
     * characters.  The LinMot cannot handle more than 1 command on a line
     * so send them separately */
    strcpy(temp, out);
    for (p = epicsStrtok_r(temp, ";", &tok_save);
                ((p != NULL) && (strlen(p) != 0));
                p = epicsStrtok_r(NULL, ";", &tok_save)) {
        Debug(2, "send_recv_mess: sending message to card %d, message=%s\n", card, p);
    status = pasynOctetSyncIO->writeRead(cntrl->pasynUser, p, strlen(p),
                         response, BUFF_SIZE, TIMEOUT,
                         &nwrite, &nread, &eomReason);
    }

    /* The response from the LinMot is terminated with CR/LF.  Remove these */
    if (nread == 0) response[0] = '\0';;
    if (nread > 0) {
        Debug(2, "send_recv_mess: card %d, response = \"%s\"\n", card, response);
    }
    if (nread == 0) {
        Debug(1, "send_recv_mess: card %d ERROR: no response\n", card);
    }
    return(strlen(response));
}



/*****************************************************/
/* Setup system configuration                        */
/* LinMotSetup()                                      */
/*****************************************************/
RTN_STATUS
LinMotSetup(int num_cards,       /* maximum number of controllers in system */
           int scan_rate)       /* polling rate - 1/60 sec units */
{
    int itera;

    if (num_cards < 1 || num_cards > LinMot_NUM_CARDS)
        LinMot_num_cards = LinMot_NUM_CARDS;
    else
        LinMot_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
    targs.motor_scan_rate = scan_rate;
    else
    targs.motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structure pointers.  Note this must be done
    * before LinMotConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem since this is just
    * an array of pointers.
    */
    motor_state = (struct controller **) malloc(LinMot_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < LinMot_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;
    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* LinMotConfig()                                    */
/*****************************************************/
RTN_STATUS
LinMotConfig(int card,           /* card being configured */
            const char *port,   /* asyn port name */
            int n_axes)         /* Number of axes */
{
    struct LinMotController *cntrl;

    if (card < 0 || card >= LinMot_num_cards)
        return (ERROR);

    if (n_axes == 0) n_axes=1;  /* This is a new parameter, some startup files don't have it yet */
    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct LinMotController));
    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;
    cntrl->n_axes = n_axes;
    strcpy(cntrl->port, port);
    return(OK);
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
    struct LinMotController *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    char command[20];
    int total_axis = 0;
    int status;
    bool success_rtn;

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (LinMot_num_cards <= 0)
    {
        Debug(1, "motor_init: *LinMot driver disabled*\n");
        Debug(1, "LinMotSetup() is missing from startup script.\n");
        return (ERROR);
    }

    for (card_index = 0; card_index < LinMot_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        cntrl = (struct LinMotController *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        success_rtn = false;

        status = pasynOctetSyncIO->connect(cntrl->port, 0, &cntrl->pasynUser, NULL);
        success_rtn = (status == asynSuccess);
        Debug(1, "motor_init, return from pasynOctetSyncIO->connect for port %s = %d, pasynUser=%p\n", cntrl->port, success_rtn, cntrl->pasynUser);

        if (success_rtn == true)
        {
            int retry = 0;

            /* Send a message to the board, see if it exists */
            /* flush any junk at input port - should not be any data available */
            do {
                recv_mess(card_index, buff, FLUSH);
            } while (strlen(buff) != 0);

            do
            {
                send_recv_mess(card_index, "1OA;", buff);
                retry++;
                /* Return value is length of response string */
            } while(strlen(buff) == 0 && retry < 3);
        }

        if (success_rtn == true && strlen(buff) > 0)
        {
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;
            /* Leave bdptr->cmnd_response false because we read each response */
            /* in send_mess and send_recv_mess. */
            brdptr->cmnd_response = false;

            /* Don't turn on motor power, too dangerous */
            /* send_mess(i, "1RSES;", buff); */
            send_mess(card_index, "1ST;", 0);     /* Stop motor */
            send_recv_mess(card_index, "1ID;", buff);    /* Read controller ID string */
            strncpy(brdptr->ident, buff, MAX_IDENT_LEN);

            total_axis = cntrl->n_axes;
            brdptr->total_axis = total_axis;
            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];

                motor_info->motor_motion = NULL;
                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
                /* Figure out if we have an encoder or not.  If so use 0A, else use OC for readback. */
                sprintf(command, "%dID", motor_index+1);
                send_recv_mess(card_index, command, buff);    /* Read controller ID string for this axis */

                set_status(card_index, motor_index);  /* Read status of each motor */
            }

        }
        else
            motor_state[card_index] = (struct controller *) NULL;
    }

    any_motor_in_motion = 0;

    Debug(3, "motor_init: spawning motor task\n");

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "tLinMot", epicsThreadPriorityMedium,
              epicsThreadGetStackSize(epicsThreadStackMedium),
              (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}
