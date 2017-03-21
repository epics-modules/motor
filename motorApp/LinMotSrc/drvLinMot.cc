#include <string.h>
#include <ctype.h>   /* isascii functions */
#include <math.h>
#include <stdio.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <drvSup.h>
#include <iocsh.h>
#include "motor.h"
#include "motorRecord.h"
#include "drvLinMot.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define BUFF_SIZE 120       /* Maximum length of string to/from LinMot */

#define TIMEOUT 3.0     /* Command timeout in sec. */

#define LINMOT_MAX_CARDS 4

/* Delay after START_MOTION before a status update is possible */
#define MOTION_DELAY 0.1

/*----------------debugging-----------------*/
volatile int drvLinMotDebug = 0;
extern "C" {epicsExportAddress(int, drvLinMotDebug);}

static inline void Debug(int level, const char *format, ...) {
    if (level < drvLinMotDebug) {
        va_list pVar;
        va_start(pVar, format);
        vprintf(format, pVar);
        va_end(pVar);
    }
}

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvLinMotReadbackDelay = 0.;

int LinMot_num_cards = 0;


/*----------------functions-----------------*/
static int recv_mess(int card, char *com, int flag);
static RTN_STATUS send_mess(int card, char const *, char *name);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

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
    NULL,
    &initialized,
    NULL
};

struct drvLinMot_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvLinMot = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvLinMot);}

static struct thread_args targs = {SCAN_RATE, &LinMot_access, MOTION_DELAY};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
  int card;

  if (LinMot_num_cards <=0)
    printf("[WARNING] No LinMot controllers found\n");
  else
  {
    for (card = 0; card < LinMot_num_cards; card++)
        if (motor_state[card])
            printf("LinMot controller %d, id: %s \n",
                card, motor_state[card]->ident);
  }
  return (0);
}


static long init()
{
    if (LinMot_num_cards <= 0)
    {
        Debug(1, "LinMotSetup() is missing from startup script.\n");
        return (ERROR);
    }
    return (OK);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}

static int send_recv_mess(int card, const char *out, char *response)
{
    char *p, *tok_save;
    struct LinMotController *cntrl;
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

    if (nread == 0) response[0] = '\0';
    if (nread > 0) {
        Debug(2, "send_recv_mess: card %d, response = \"%s\"\n", card, response);
    }
    if (nread == 0) {
        Debug(1, "send_recv_mess: card %d ERROR: no response\n", card);
    }
    return(strlen(response));
}


/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
    return (0);
}


/*****************************************************/
/* send a message to the LinMot board                 */
/* send_mess()                                       */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
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


/*
 * FUNCTION... recv_mess(int card, char *com, int flag)
 *
 * INPUT ARGUMENTS...
 *      card - controller card # (0,1,...).
 *      *com - caller's response buffer.
 *      flag    | FLUSH  = this flag is ignored - the receive buffer is flushed
 *                         on every write (see write_mess())
 * LOGIC...
 *  IF controller card does not exist.
 *      ERROR RETURN.
 *  ENDIF
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int flag)
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

    /* The response from the PM304 is terminated with CR/LF.  Remove these */
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
/* Setup system configuration                        */
/* LinMotSetup()                                     */
/*****************************************************/
RTN_STATUS
LinMotSetup(int num_cards,       /* maximum number of controllers in system.  */
           int scan_rate)       /* polling rate - 1/60 sec units.  */
{
    if (num_cards < 1 || num_cards > LINMOT_MAX_CARDS)
        LinMot_num_cards = LINMOT_MAX_CARDS;
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

    for (int card = 0; card < LinMot_num_cards; card++)
        motor_state[card] = (struct controller *) NULL;
		
    Debug(3, "LinMotSetup completed successfully\n");
    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* LinMotConfig()                                    */
/*****************************************************/
RTN_STATUS
LinMotConfig(int card,          /* card being configured */
            const char *port,   /* asyn port name */
            int n_axes          /* Number of axes */
			)
{
    struct LinMotController *cntrl;
    if (card < 0 || card >= LinMot_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct LinMotController));
    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;
    cntrl->n_axes = n_axes;
    strcpy(cntrl->port, port);
    Debug(3, "LinMotConfig completed successfully\n");
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
                send_recv_mess(card_index, "!GPA", buff);
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
            send_mess(card_index, "!SS1A;", 0);     /* Stop motor */
            send_recv_mess(card_index, "!PVA;", buff);    /* Read controller ID string */
            strncpy(brdptr->ident, buff, MAX_IDENT_LEN);

            total_axis = cntrl->n_axes;
            brdptr->total_axis = total_axis;
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];

                motor_info->motor_motion = NULL;
                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
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

extern "C"
{

// Setup arguments
    static const iocshArg setupArg0 = {"Maximum # of cards", iocshArgInt};
    static const iocshArg setupArg1 = {"Polling rate (HZ)", iocshArgInt};
// Config arguments
    static const iocshArg configArg0 = {"Card# being configured", iocshArgInt};
    static const iocshArg configArg1 = {"asyn port name", iocshArgString};
    static const iocshArg configArg2 = {"asyn address (GPIB)", iocshArgInt};

    static const iocshArg *const SetupArgs[2]  = {&setupArg0, &setupArg1};
    static const iocshArg *const ConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

    static const iocshFuncDef setupLinMot  = {"LinMotSetup", 2, SetupArgs};
    static const iocshFuncDef configLinMot = {"LinMotConfig", 3, ConfigArgs};

    static void setupLinMotCallFunc(const iocshArgBuf *args)
    {
        LinMotSetup(args[0].ival, args[1].ival);
    }
    static void configLinMotCallFunc (const iocshArgBuf *args)
    {
        LinMotConfig(args[0].ival, args[1].sval, args[2].ival);
    }

    static void LinMotRegister(void)
    {
        iocshRegister(&setupLinMot, setupLinMotCallFunc);
        iocshRegister(&configLinMot, configLinMotCallFunc);
    }

    epicsExportRegistrar(LinMotRegister);

} // extern "C"

