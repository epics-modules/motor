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
}

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
  return (0);
}

static long init()
{
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
    return (0);
}

STATIC RTN_STATUS send_mess(int card, const char *com, char *name)
{
    return(OK);
}

STATIC int recv_mess(int card, char *com, int flag)
{
    return(0);
}

STATIC int send_recv_mess(int card, const char *out, char *response)
{
    return(0);
}

RTN_STATUS
LinMotSetup(int num_cards,       /* maximum number of controllers in system */
           int scan_rate)       /* polling rate - 1/60 sec units */
{
    return(OK);
}

RTN_STATUS
LinMotConfig(int card,           /* card being configured */
            const char *port,   /* asyn port name */
            int n_axes)
{
    return(OK);
}

STATIC int motor_init()
{
    return(OK);
}
