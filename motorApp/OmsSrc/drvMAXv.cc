/*
FILENAME...     drvMAXv.cc
USAGE...        Motor record driver level support for OMS model MAXv.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 04/05/04
 *      Current Author: Ron Sluiter
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
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * NOTES
 * -----
 * Verified with MAXv firmware:
 *      - ver:1.25
 *      - ver:1.29 (has ECO #1432; fixes initialization problem).
 *      - ver:1.31 (fixes DPRAM encoder position data problem when using
 *                  mixed motor types.)
 *      - ver:1.33, FPGA:B2:A6 BOOT:1.2 (Watchdog Timeout Counter added)
 *      - ver:1.34, FPGA:03:A6 BOOT:1.3
 *      - ver:1.41 & 1.42, suffers from having limit switches disabled at
 *                  power-up.
 *      - ver:1.44, No known problems.
 *      - ver:1.45, limit switches are disabled by default.
 *
 * Modification Log:
 * -----------------
 * 01  04-05-04 rls Copied from drvOms58.cc
 * 02  09-20-04 rls - support for 32axes/controller.
 *                  - added MAXvConfig() with initilization string.  Axis type
 *                    MUST be set before iocInit is called.
 * 03  12-14-04 rls - MS Visual C compiler support.
 *                  - eliminate calls to devConnectInterrupt() due to C++
 *                    problems with devLib.h; i.e. "sorry, not implemented:
 *                    `tree_list' not supported..." compiler error message.
 * 04  03-21-05 rls - Make MAXv OSI.
 * 05  05-02-05 rls - Bug fix for stale data delay; set delay = 10ms.
 * 06  05-17-06 rls - Allow polling rate up to 1/epicsThreadSleepQuantum().
 *                  - Protect against multiple MAXvSetup() calls.
 * 07  06-05-07 rls - Added Jens Eden (BESSY) modifications;
 *                    - register iocsh commands.
 *                    - added USE_DEVLIB with RTEMS conditionial.
 *                    - replaced errlogPrintf calls in ISR with
 *                      epicsInterruptContextMessage calls.
 * 08  08-20-07 rls - Make send_mess() and recv_mess() non-global.
 *                  - removed unneeded stub start_status().
 * 09  02-26-08 rls - set "update delay" to zero.
 * 10  05-14-08 rls - read the commanded velocity.
 * 11  05-20-08 rls - A24/A32 address mode bug fix.
 * 12  01-05-09 rls - Dirk Zimoch's (PSI) bug fix for set_status() overwriting
 *                    the home switch status in the response string.
 * 13  06-18-09 rls - Make MAXvSetup() error messages more prominent.
 * 14  07-02-09 rls - backwards compatibility with ver:1.29 and earlier
 *                    firmware. OMS changed from '<LF><NULL>' to '<LF>' for
 *                    RA, QA, EA and RL command with ver:1.30
 * 15  09-09-09 rls - board "running" error check added.
 * 16  03-08-10 rls - sprintf() not callable from RTEMS interrupt context.
 * 17  03-09-10 rls - sprintf() not callable from any OS ISR.
 * 18  06-01-10 rls - Save firmware version in static float array.
 *                  - For firmware ver:1.33 and above, read Watchdog Timeout
 *                    Counter. If Counter is nonzero, print error message and
 *                    clear Counter.
 * 19  06-07-10 rls - Disable board if WDT CTR is nonzero; don't clear CTR.
 * 20  02-03-11 rls - Increase max. config. string size from 150 to 300 bytes.
 *                  - Increase all receive buffer sizes to same 300 bytes.
 *                  - Add error checks for buffer overflow with MAXvConfig()'s
 *                    configuration string argument and in readbuf().
 * 21  02-04-11 rls - Added counter to send_mess()'s "waiting for message
 *                    acknowledgement" loop to prevent infinite loop.
 * 22  09-23-11 ajr - Added configuration word MAXvConfig.
 * 23  10-26-11 rls - Changed Debug() to Mark River's variable arguments macro.
 *                  - Added MAXvController data structure using private data in
 *                    motor record to store motor type. Motor type used in
 *                    device support (devOmsCom.cc) to allow MRES and ERES with
 *                    different polarity (signs).
 * 24  02-24-14 rls - After the initialization string is read, if limit mode is
 *                    "Off", set it to "Hard".
 */

#include <string.h>
#include <stdio.h>
#include <dbCommon.h>
#include <drvSup.h>
#include <epicsVersion.h>
#include <epicsString.h>
#include <devLib.h>
#include <dbAccess.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsInterrupt.h>
#include <iocsh.h>
#include <epicsExit.h>
#include <cantProceed.h>

#include "motorRecord.h" /* For Driver Power Monitor feature only. */
#include "motor.h"
#include "motordevCom.h" /* For Driver Power Monitor feature only. */
#include "drvMAXv.h"

#include "epicsExport.h"

#define MUTEX(card) ((struct MAXvController *)(motor_state[card]->DevicePrivate))->message_mutex

/* Define for return test on devNoResponseProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

/* Are we using VME-Bus/devLib */
#if (defined(vxWorks) || defined(__rtems__))
    #define USE_DEVLIB
#endif

/* jps: INFO messages - add RV and move QA to top */
#define ALL_INFO        "QA EA"
#define AXIS_INFO       "QA"
#define ENCODER_QUERY   "EA ID"
#define AXIS_CLEAR      "CA"            /* Clear done of addressed axis */
#define DONE_QUERY      "RA"            /* ?? Is this needed?? */
#define PID_QUERY       "?KA ID"

/*----------------debugging-----------------*/
volatile int drvMAXvdebug = 0;
extern "C" {epicsExportAddress(int, drvMAXvdebug);}

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvMAXvdebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

#define pack2x16(p)      ((epicsUInt32)(((p[0])<<16)|(p[1])))
#define INITSTR_SIZE    300     /* 300 byte configuration string. */

/* Global data. */
int MAXv_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"

/* --- Local data common to all OMS drivers. --- */
static char *MAXv_addrs = 0x0;
static epicsAddressType MAXv_ADDRS_TYPE;
static volatile unsigned MAXvInterruptVector = 0;
static volatile epicsUInt8 omsInterruptLevel = OMS_INT_LEVEL;
static volatile int motionTO = 10;
static char *MAXv_axis[] = {"X", "Y", "Z", "T", "U", "V", "R", "S"};
static double quantum;
static char **initstring = 0;
static epicsUInt32 MAXv_brd_size;  /* card address boundary */
static char cmndbuf[MAX_MSG_SIZE]; /* Command buffer used by send_mess() and
                                    * motorIsr if there is a "command error"*/

/* First 8-bits [0..7] used to indicate absolute or */
/* incremental position registers to be read */
static int configurationFlags[MAXv_NUM_CARDS] = {0};

/*----------------functions-----------------*/

/* Common local function declarations. */
extern "C" {
RTN_STATUS MAXvSetup(int, int, unsigned int, unsigned int, int, int);
RTN_VALUES MAXvConfig(int, const char *, int);
}
static long report(int);
static long init();
static void query_done(int, int, struct mess_node *);
static int set_status(int, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int recv_mess(int, char *, int);
static int getPositions(int card, epicsInt32 *positions, int nPositions);
static int send_recv_mess(int card, char const * command, char *axis, char *buf, int nMessages);

extern "C" {

int MAXV_send_mess(int card, char const * command, char *axis) {
    return (int)send_mess(card, command, axis);
}

int MAXV_recv_mess(int card, char *buf, int nMessages) {
    return recv_mess(card, buf, nMessages);
}

int MAXV_send_recv_mess(int card, char const * command, char *axis, char *buf, int nMessages) {
    return send_recv_mess(card, command, axis, buf, nMessages);
}

int MAXV_getPositions(int card, epicsInt32 *positions, int nPositions) {
    return getPositions(card, positions, nPositions);
}

}

static int getPositions(int card, epicsInt32 *positions, int nPositions) {
    volatile struct MAXv_motor *pmotor;
    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;
    int i;
    for (i=0; i<nPositions; i++) {
        positions[i] = pmotor->cmndPos[i];
        Debug(5, "getPositions: motor %d position=%d\n", i, positions[i]);
    }
    return(0);
}

static void motorIsr(int);
static int motor_init();
static void MAXv_reset(void *);
static char *readbuf(volatile struct MAXv_motor *, char *);

static int motorIsrSetup(int card);

struct driver_table MAXv_access =
{
    NULL,
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
    MAXv_axis
};

struct drvMAXv_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvMAXv = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvMAXv);}

static struct thread_args targs = {SCAN_RATE, &MAXv_access, 0.000};

static char wdctrmsg[] = "\n***MAXv card #%d Disabled*** Watchdog Timeout CTR %s\n\n";
static char norunmsg[] = "\n*** MAXv card #%d is NOT running *** status = 0x%x\n";


/*----------------functions-----------------*/

static long report(int level)
{
    int card;

    if (MAXv_num_cards <= 0)
        printf("    No MAXv controllers configured.\n");
    else
    {
        for (card = 0; card < MAXv_num_cards; card++)
        {
            struct controller *brdptr = motor_state[card];

            if (brdptr == NULL)
                printf("    Oms MAXv motor card #%d not found.\n", card);
            else
                printf("    Oms MAXv motor card #%d @ %p, id: %s \n", card,
                       motor_state[card]->localaddr, motor_state[card]->ident);
        }
    }
    return (0);
}

static long init()
{
    initialized = true; /* Indicate that driver is initialized. */
    (void) motor_init();
    return ((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
    char buffer[MAX_IDENT_LEN];

    send_recv_mess(card, DONE_QUERY, MAXv_axis[axis], buffer, 1);

    if (nodeptr->status.Bits.RA_PROBLEM)
        send_mess(card, AXIS_STOP, MAXv_axis[axis]);
}


/******************************************************************************
* FUNCTION NAME: set_status
*
* ARGUMETS     Type      I/O     Description
* --------     ----      ---     -----------
*
* card         int        I      Controller card index #.
* signal       int        I      Motor index.
*
* LOGIC
*   Initialize.
*   IF encoder present.
*       Get axis and encoder information.
*   ELSE
*       Get axis information.
*   ENDIF
*
*   Process controller response strings.
*
*   ...
*   ...
*   IF "motor-in-motion" (i.e., nodeptr != 0), AND, no limit switch error.
*       IF drive power monitoring enabled.
*           ...
*       ENDIF
*   ENDIF
*
*   IF no motion, OR, status indicates limit switch error, OR, motor done, OR,
*               controller problem.
*       Set return state to "callback record".
*   ELSE
*       Set return state to skip "record callback".
*   ENDIF
*   IF status indicates DONE/LIMIT, AND, "motor-in-motion" (nodeptr != 0), AND,
*               post-move message is not null.
*       Send post-move message to controller.
*       Clear post-move message pointer.
*   ENDIF
*   EXIT with return state indicator.
******************************************************************************/

static int set_status(int card, int signal)
{
    struct mess_info *motor_info;
    struct mess_node *nodeptr;
    volatile struct MAXv_motor *pmotor;
    epicsInt32 motorData;
    /* Message parsing variables */
    char *p, *tok_save;
    struct axis_status *ax_stat;
    struct encoder_status *en_stat;
    struct controller *brdptr;
    struct MAXvController *MAXvCntrl;

    char q_buf[MAX_IDENT_LEN], outbuf[50];
    int index;
    bool ls_active = false;
    bool got_encoder;
    msta_field status;

    int absoluteAxis = (configurationFlags[card] & (1 << signal));

    int rtn_state;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;
    status.All = motor_info->status.All;

    if ((brdptr = motor_state[card]) == NULL)   /* Test for board disabled. */
        return(rtn_state = 1);                  /* End move. */

    MAXvCntrl = (struct MAXvController *) brdptr->DevicePrivate;
    if (MAXvCntrl->fwver >= 1.33)
    {
        send_recv_mess(card, "#WS", (char) NULL, q_buf, 1);
        if (strcmp(q_buf, "=0") != 0)
        {
            errlogPrintf(wdctrmsg, card, q_buf);
            status.Bits.RA_PROBLEM = 1;
            motor_info->status.All = status.All;
            send_mess(card, STOP_ALL, (char) NULL);
            /* Disable board. */
            motor_state[card] = (struct controller *) NULL;
            return(rtn_state = 1); /* End move. */
        }
    }

    if (motor_info->encoder_present == YES)
    {
        /* get 4 pieces of info from axis */
        send_recv_mess(card, ALL_INFO, MAXv_axis[signal], q_buf, 2);
        got_encoder = true;
    }
    else
    {
        /* get 2 pieces of info from axis */
        send_recv_mess(card, AXIS_INFO, MAXv_axis[signal], q_buf, 1);
        got_encoder = false;
    }

    for (index = 0, p = epicsStrtok_r(q_buf, ",", &tok_save); p;
         p = epicsStrtok_r(NULL, ",", &tok_save), index++)
    {
        switch (index)
        {
        case 0:         /* axis status */
            ax_stat = (struct axis_status *) p;

            status.Bits.RA_DIRECTION = (ax_stat->direction == 'P') ? 1 : 0;
            status.Bits.RA_HOME =      (ax_stat->home == 'H')      ? 1 : 0;
            status.Bits.RA_DONE =      (ax_stat->done == 'D')      ? 1 : 0;

            if (ax_stat->overtravel == 'L')
            {
                ls_active = true;
                if (status.Bits.RA_DIRECTION)
                    status.Bits.RA_PLUS_LS = 1;
                else
                    status.Bits.RA_MINUS_LS = 1;
            }
            else
            {
                ls_active = false;
                status.Bits.RA_PLUS_LS  = 0;
                status.Bits.RA_MINUS_LS = 0;
            }
            break;

        case 1:         /* encoder status */
            en_stat = (struct encoder_status *) p;

            status.Bits.EA_SLIP       = (en_stat->slip_enable == 'E') ? 1 : 0;
            status.Bits.EA_POSITION   = (en_stat->pos_enable  == 'E') ? 1 : 0;
            status.Bits.EA_SLIP_STALL = (en_stat->slip_detect == 'S') ? 1 : 0;
            status.Bits.EA_HOME       = (en_stat->axis_home   == 'H') ? 1 : 0;
            break;

        default:
            break;
        }
    }

    /* motor pulse count (position) */
    motorData = pmotor->cmndPos[signal];

    if (motorData == motor_info->position)
    {
        if (nodeptr != 0)       /* Increment counter only if motor is moving. */
            motor_info->no_motion_count++;
    }
    else
    {
        motor_info->no_motion_count = 0;
        motor_info->position = motorData;
    }

    if (motor_info->no_motion_count > motionTO)
    {
        status.Bits.RA_PROBLEM = 1;
        send_mess(card, AXIS_STOP, MAXv_axis[signal]);
        motor_info->no_motion_count = 0;
        errlogSevPrintf(errlogMinor, "Motor motion timeout ERROR on card: %d, signal: %d\n",
            card, signal);
    }
    else if (pmotor->firmware_status.Bits.running == 0)
    {
        status.Bits.RA_PROBLEM = 1;
        errlogPrintf(norunmsg, card, (unsigned int) pmotor->firmware_status.All);
    }
    else
        status.Bits.RA_PROBLEM = 0;

    /* get command velocity */
    send_recv_mess(card, "RV", MAXv_axis[signal], q_buf, 1);
    motor_info->velocity = atoi(q_buf);

    /* Get encoder position */
    if (absoluteAxis)
        motorData = pmotor->absPos[signal];
    else
        motorData = pmotor->encPos[signal];

    motor_info->encoder_position = motorData;

    if (nodeptr != NULL && ls_active == false)
    {
        struct motor_trans *trans = (struct motor_trans *) nodeptr->mrecord->dpvt;
        if (trans->dpm == true)
        {
            errlogPrintf("Drive power failure at MAXv card#%d motor#%d\n",
                         card, signal);
        }
    }

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
                status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
        nodeptr->postmsgptr != 0)
    {
        char buffer[40];

        /* Test for a "device directive" in the POST string. */
        if (nodeptr->postmsgptr[0] == '@')
        {
            bool errind = false;
            char *end = strchr(&nodeptr->postmsgptr[1], '@');
            if (end == NULL)
                errind = true;
            else
            {
                DBADDR addr;
                char *start, *tail;
                int size = (end - &nodeptr->postmsgptr[0]) + 1;

                /* Copy device directive to buffer. */
                strncpy(buffer, nodeptr->postmsgptr, size);
                buffer[size] = (char) NULL;

                if (strncmp(buffer, "@PUT(", 5) != 0)
                    goto errorexit;
                
                /* Point "start" to PV name argument. */
                tail = NULL;
                start = epicsStrtok_r(&buffer[5], ",", &tail);
                if (tail == NULL)
                    goto errorexit;

                if (dbNameToAddr(start, &addr)) /* Get address of PV. */
                {
                    errPrintf(-1, __FILE__, __LINE__, "Invalid PV name: %s", start);
                    goto errorexit;
                }

                /* Point "start" to PV value argument. */
                start = epicsStrtok_r(NULL, ")", &tail);
                if (dbPutField(&addr, DBR_STRING, start, 1L))
                {
                    errPrintf(-1, __FILE__, __LINE__, "invalid value: %s", start);
                    goto errorexit;
                }
            }

            if (errind == true)
errorexit:      errMessage(-1, "Invalid device directive");
            end++;
            strcpy(buffer, end);
        }
        else
            strcpy(buffer, nodeptr->postmsgptr);

        strcpy(outbuf, buffer);
        send_mess(card, outbuf, MAXv_axis[signal]);
        nodeptr->postmsgptr = NULL;
    }

    /* Bug fix for DC servo moving away from limit switch, but move is not far enough to
     * get off limit switch; resulting in limit error.  Fix is to force CDIR to match
     * MSTA.RA_DIRECTION.
     */
    if (ls_active == true && status.Bits.GAIN_SUPPORT &&
        status.Bits.EA_POSITION == 0 && nodeptr != 0)
    {
        struct motorRecord *mr = (struct motorRecord *) nodeptr->mrecord;

        if (mr->cdir != (short) status.Bits.RA_DIRECTION)
            mr->cdir = status.Bits.RA_DIRECTION;
    }

    motor_info->status.All = status.All;        /* Update status from local copy. */
    return (rtn_state);
}

/**************************************************
 * send a message to the OMS board and get the reply
 **************************************************/
static int send_recv_mess(int card, char const * command, char *axis, char *buf, int nMessages) {
    int retval;

    if (!epicsMutexTryLock(MUTEX(card))) {
        Debug(1, "send_recv_mess: waiting for mutex\n");
        epicsMutexLock(MUTEX(card));
    }
    retval = (int)send_mess(card, command, axis);
    retval |= recv_mess(card, buf, nMessages);
    epicsMutexUnlock(MUTEX(card));
    return(retval);
}

/*****************************************************/
/* send a message to the OMS board                   */
/* send_mess()                       */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    volatile struct MAXv_motor *pmotor;
    epicsInt16 putIndex;
    char *pcmndbuf;
    RTN_STATUS return_code;
    int count;

    if (strlen(com) > MAX_MSG_SIZE)
    {
        errlogPrintf("drvMAXv.cc:send_mess(); message size violation.\n");
        return (ERROR);
    }

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("drvMAXv.cc:send_mess() - invalid card #%d\n", card);
        return (ERROR);
    }

    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;
    Debug(9, "send_mess: pmotor = %p\n", pmotor);

    if (!epicsMutexTryLock(MUTEX(card))) {
        Debug(1, "send_mess: waiting for mutex\n");
        epicsMutexLock(MUTEX(card));
    }

    return_code = OK;

    Debug(9, "send_mess: checking card %d status\n", card);

    /* see if junk at input port - should not be any data available */
    if (pmotor->inGetIndex != pmotor->inPutIndex)
    {
        Debug(1, "send_mess - clearing data in buffer\n");
        recv_mess(card, cmndbuf, FLUSH);
    }


    if (name == NULL)
        strcpy(cmndbuf, com);
    else
    {
        strcpy(cmndbuf, "A");
        strcat(cmndbuf, name);
        strcat(cmndbuf, " ");
        strcat(cmndbuf, com);
    }

    Debug(9, "send_mess: ready to send message.\n");
    putIndex = pmotor->outPutIndex;
    for (pcmndbuf = cmndbuf; *pcmndbuf != '\0'; pcmndbuf++)
    {
        pmotor->outBuffer[putIndex++] = *pcmndbuf;
        if (putIndex >= BUFFER_SIZE)
            putIndex = 0;
    }

    Debug(4, "send_mess: sent card %d message:", card);
    Debug(4, "%s\n", cmndbuf);

    pmotor->outPutIndex = putIndex;     /* Message Sent */

    for (count = 0; (pmotor->outPutIndex != pmotor->outGetIndex) &&
         (count < 1000); count++)
    {
#ifdef  DEBUG
        epicsInt16 deltaIndex, delta;

        deltaIndex = pmotor->outPutIndex - pmotor->outGetIndex;
        delta = (deltaIndex < 0) ? BUFFER_SIZE + deltaIndex : deltaIndex;
        Debug(5, "send_mess: Waiting for ack: index delta=%d\n", delta);
#endif
        epicsThreadSleep(epicsThreadSleepQuantum());
    };
    
    if (count >= 1000)
    {
        errlogPrintf("\n*** MAXv card #%d communication timeout ***\n", card);
        return_code = ERROR;
    }

    epicsMutexUnlock(MUTEX(card));
    return (return_code);
}

/*
 * FUNCTION... recv_mess(int card, char *com, int amount)
 *
 * INPUT ARGUMENTS...
 *      card - 
 *      *com -
 *      amount - 
 *
 * LOGIC...
 *  IF controller card does not exist.
 *      ERROR Exit.
 *  ENDIF
 *  IF "amount" indicates buffer flush.
 *      WHILE characters left in input buffer.
 *          Remove characters from controller's input buffer.
 *      ENDWHILE
 *      NORMAL RETURN.
 *  ENDIF
 *
 *  FOR each message requested (i.e. "amount").
 *      Initialize head and tail pointers.
 *      Initialize local buffer "get" index.
 *      FOR
 *          IF characters left in controller's input buffer.
 *              
 *          ENDIF
 *      ENDFOR
 *  ENDFOR
 *  
 */
static int recv_mess(int card, char *com, int amount)
{
    volatile struct MAXv_motor *pmotor;
    int itera;
    char *bufptr;

    /* Check that card exists */
    if (!motor_state[card])
    {
        Debug(1, "recv_mess - invalid card #%d\n", card);
        return(-1);
    }

    pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;

    if (!epicsMutexTryLock(MUTEX(card))) {
        Debug(1, "recv_mess: waiting for mutex\n");
        epicsMutexLock(MUTEX(card));
    }

    if (amount == -1)
    {
        if (pmotor->inGetIndex != pmotor->inPutIndex)
        {
            char junk[MAX_IDENT_LEN];

            readbuf(pmotor, junk);

            Debug(1, "recv_mess(): flushed - %s\n", junk);
        }
        epicsMutexUnlock(MUTEX(card));
        return(0);
    }

    bufptr = com;
    *bufptr = (char) NULL;

    do
    {
        itera = 1;
        double time = 0.0;

        while (pmotor->status1_flag.Bits.text_response == 0 && time < 0.100)
        {
            Debug(1, "recv_mess(): response wait - %d\n", itera);
            time += quantum * itera;
            epicsThreadSleep(quantum * itera);
            itera++;
        }

        if (pmotor->status1_flag.Bits.text_response == 0)
        {
            Debug(1, "Timeout occurred in recv_mess\n");
            *bufptr = '\0';
            epicsMutexUnlock(MUTEX(card));
            return(-1);
        }

        bufptr = readbuf(pmotor, bufptr);
        if (--amount > 0)
        {
            if (*(bufptr-1) == '\n') /* For ver:1.29 and before firmware, */
                *(bufptr-1) = ',';   /* replace <LF><0> with <,><0>.      */
            else                     /* For ver:1.30 and after firmware,  */
                *(bufptr++) = ',';   /* replace <LF> with <,>.            */
        }
    } while (amount > 0);

    epicsMutexUnlock(MUTEX(card));
    Debug(4, "recv_mess(): card#%d - %s\n", card, com);
    return(0);
}

static char *readbuf(volatile struct MAXv_motor *pmotor, char *bufptr)
{
    STATUS1 flag1;
    epicsUInt32 getIndex, putIndex;
    int bufsize;
    char *start, *end, *bufend;
    
    getIndex = pmotor->inGetIndex;
    putIndex = pmotor->inPutIndex;
    bufsize  = putIndex - getIndex;
    
    start  = (char *) &pmotor->inBuffer[getIndex];
    end    = (char *) &pmotor->inBuffer[putIndex];

    if (start < end)    /* Test for message wraparound in buffer. */
        memcpy(bufptr, start, bufsize);
    else if (start == end) /* Test for empty input buffer. */
    {
        static char emptymsg[] = 
        "MAXv DPRAM input buffer empty; inGetIndex = %d inPutIndex = %d\n";
 
        errlogPrintf(emptymsg, getIndex, putIndex);
        return(bufptr);
    }
    else
    {
        int size;

        bufend = (char *) &pmotor->inBuffer[BUFFER_SIZE];
        size = bufend - start;
        bufsize += BUFFER_SIZE;

        if (bufsize > MAX_IDENT_LEN)
        {
            errlogPrintf("\n*** MAXv readbuf() overrun ***; bufsize = %d\n\n", bufsize);
            return(bufptr);
        }
        else
        {
            memcpy(bufptr, start, size);
            memcpy((bufptr + size), (const char *) &pmotor->inBuffer[0], (bufsize - size));
        }
    }
    
    getIndex += bufsize;
    if (getIndex >= BUFFER_SIZE)
        getIndex -= BUFFER_SIZE;
    
    bufptr += (bufsize - 1);
    *bufptr = (char) NULL;

    while (getIndex != pmotor->inPutIndex)
    {
        Debug(1, "readbuf(): flushed - %d\n", pmotor->inBuffer[getIndex]);
        if (++getIndex > BUFFER_SIZE)
            getIndex = 0;
    }

    pmotor->inGetIndex = getIndex;
    flag1.All = pmotor->status1_flag.All;
    pmotor->status1_flag.All = flag1.All;
    pmotor->msg_semaphore = 0;
    return(bufptr);
}

/*****************************************************/
/* Configuration function for  module_types data     */
/* areas. MAXvSetup()                                */
/*****************************************************/
RTN_STATUS
MAXvSetup(int num_cards,        /* maximum number of cards in rack */
          int addrs_type,       /* VME address type; 16 - A16, 24 - A24 or 32 - A32. */
          unsigned int addrs,   /* Base Address. */
          unsigned int vector,  /* noninterrupting(0), valid vectors(64-255) */
          int int_level,        /* interrupt level (1-6) */
          int scan_rate)        /* 1 <= polling rate <= (1/epicsThreadSleepQuantum) */
{
    int itera;
    char **strptr;
    RTN_STATUS rtncode = OK;
    double frequency;
    char errbase[] = "\nMAXvSetup: *** invalid ";

    if (initstring == NULL)
       initstring = (char **) callocMustSucceed(1,
                    sizeof(char *) * MAXv_NUM_CARDS, "MAXvSetup() initstring");
    else
    {
        /* Deallocate memory for initialization strings. */
        for (itera = 0, strptr = &initstring[0]; itera < MAXv_num_cards; itera++, strptr++)
            free(*strptr);
    }

    if (num_cards < 1 || num_cards > MAXv_NUM_CARDS)
    {
        char format[] = "%snumber of cards specified = %d ***\n";
        MAXv_num_cards = MAXv_NUM_CARDS;
        errlogPrintf(format, errbase, num_cards);
        errlogPrintf("             *** using maximum number = %d ***\n", MAXv_NUM_CARDS);
        epicsThreadSleep(5.0);
        rtncode = ERROR;
    }
    else
        MAXv_num_cards = num_cards;

    {
        char addmsg[] = "%sA%d address *** = 0x%X.\n";
        switch (addrs_type)
        {
        case 16:
            MAXv_ADDRS_TYPE = atVMEA16;
            if ((epicsUInt32) addrs & 0xFFFF0FFF)
            {
                errlogPrintf(addmsg, errbase, 16, (epicsUInt32) addrs);
                rtncode = ERROR;
            }
            else
            {
                MAXv_addrs = (char *) addrs;
                MAXv_brd_size = 0x1000;
            }
            break;
        case 24:
            MAXv_ADDRS_TYPE = atVMEA24;
            if ((epicsUInt32) addrs & 0xFF00FFFF)
            {
                errlogPrintf(addmsg, errbase, 24, (epicsUInt32) addrs);
                rtncode = ERROR;
            }
            else
            {
                MAXv_addrs = (char *) addrs;
                MAXv_brd_size = 0x10000;
            }
            break;
        case 32:
            MAXv_ADDRS_TYPE = atVMEA32;
            if ((epicsUInt32) addrs & 0x00FFFFFF)
            {
                errlogPrintf(addmsg, errbase, 32, (epicsUInt32) addrs);
                rtncode = ERROR;
            }
            else
            {
                MAXv_addrs = (char *) addrs;
                MAXv_brd_size = 0x1000000;
            }
            break;
        default:
            {
                char format[] = "%sVME address type = %d ***\n";
                errlogPrintf(format, errbase, addrs_type);
            }
            rtncode = ERROR;
            break;
        }
    }

    if (rtncode == ERROR)
        epicsThreadSleep(5.0);

    MAXvInterruptVector = vector;
    if (vector < 64 || vector > 255)
    {
        if (vector != 0)
        {
            char format[] = "%sinterrupt vector = %d ***\n";
            MAXvInterruptVector = (unsigned) OMS_INT_VECTOR;
            errlogPrintf(format, errbase, vector);
            epicsThreadSleep(5.0);
            rtncode = ERROR;
        }
    }

    if (int_level < 1 || int_level > 6)
    {
        char format[] = "%sinterrupt level = %d ***\n";
        omsInterruptLevel = OMS_INT_LEVEL;
        errlogPrintf(format, errbase, int_level);
        epicsThreadSleep(5.0);
        rtncode = ERROR;
    }
    else
        omsInterruptLevel = int_level;

    quantum = epicsThreadSleepQuantum();
    frequency = 1.0 / quantum;
    
    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= frequency)
        targs.motor_scan_rate = scan_rate;
    else
    {
        char format[] = "%spolling rate = %d ***\n";
        targs.motor_scan_rate = (int) frequency;
        errlogPrintf(format, errbase, scan_rate);
        epicsThreadSleep(5.0);
        rtncode = ERROR;
    }

    /* Allocate memory for initialization strings. */
    for (itera = 0, strptr = &initstring[0]; itera < MAXv_num_cards; itera++, strptr++)
    {
        *strptr = (char *) malloc(INITSTR_SIZE);
        **strptr = (char) NULL;
    }

    return(rtncode);
}

RTN_VALUES MAXvConfig(int card,                 /* number of card being configured */
                      const char *initstr,      /* configuration string */
                      int AbsConfig)            /* absolute encoder configuration */
{
    if (card < 0 || card >= MAXv_num_cards)
    {
        errlogPrintf("MAXvConfig: invalid card %d\n", card);
        epicsThreadSleep(5.0);
        return(ERROR);
    }
    
    if (strlen(initstr) > INITSTR_SIZE)
    {
        errlogPrintf("\n*** MAXvConfig ERROR ***\n");
        errlogPrintf("Configuration string: %d bytes > %d maximum.\n\n", (int)strlen(initstr), INITSTR_SIZE);
        epicsThreadSleep(5.0);
        return(ERROR);
    }
    strcpy(initstring[card], initstr);
    
    /* Save absolute encoder and grey code configuation flags */
    configurationFlags[card]  = AbsConfig;

    return(OK);
}


/*****************************************************/
/* Interrupt service routine.                        */
/* motorIsr()                                        */
/*****************************************************/
static void motorIsr(int card)
{
    volatile struct controller *pmotorState;
    volatile struct MAXv_motor *pmotor;
    STATUS1 status1_flag;
    static char errmsg1[] = "drvMAXv.cc:motorIsr: ***Invalid entry*** - card xx\n";
    static char errmsg2[] = "drvMAXv.cc:motorIsr: ***Command Error*** - card xx\n";

    if (card >= total_cards || (pmotorState = motor_state[card]) == NULL)
    {
        errmsg1[51-2] = '0' + card%10;
        errmsg1[51-3] = '0' + (card/10)%10;
        epicsInterruptContextMessage(errmsg1);
        return;
    }

    pmotor = (struct MAXv_motor *) (pmotorState->localaddr);
    status1_flag.All = pmotor->status1_flag.All;

    /* Motion done handling */
    if (status1_flag.Bits.done != 0)
        motor_sem.signal();  /* Wake up 'motor_task()' to issue callbacks */

    if (status1_flag.Bits.cmndError)
    {
        errmsg2[51-2] = '0' + card%10;
        errmsg2[51-3] = '0' + (card/10)%10;
        epicsInterruptContextMessage(errmsg2);
        strcat(cmndbuf,"\n\n");
        epicsInterruptContextMessage(cmndbuf);
    }

    if (status1_flag.Bits.text_response != 0)   /* Don't clear this. */
        status1_flag.Bits.text_response = 0;

    pmotor->status1_flag.All = status1_flag.All; /* Release IRQ's. */
}

static int motorIsrSetup(int card)
{
    volatile struct MAXv_motor *pmotor;
    STATUS1 status1_irq;
#ifdef USE_DEVLIB
    long status;
#endif

    Debug(5, "motorIsrSetup: Entry card#%d\n", card);

    pmotor = (struct MAXv_motor *) (motor_state[card]->localaddr);

#ifdef USE_DEVLIB

    status = pdevLibVirtualOS->pDevConnectInterruptVME(
        MAXvInterruptVector + card,
#if LT_EPICSBASE(3,14,8)
        (void (*)()) motorIsr,
#else
        (void (*)(void *)) motorIsr,
#endif
        (void *) card);

    if (!RTN_SUCCESS(status))
    {
        errPrintf(status, __FILE__, __LINE__, "Can't connect to vector %d\n", MAXvInterruptVector + card);
        return (ERROR);
    }

    status = devEnableInterruptLevel(OMS_INTERRUPT_TYPE,
                                     omsInterruptLevel);
    if (!RTN_SUCCESS(status))
    {
        errPrintf(status, __FILE__, __LINE__, "Can't enable enterrupt level %d\n", omsInterruptLevel);
        return (ERROR);
    }

#endif

    /* Setup card for interrupt-on-done */
    status1_irq.All = 0;
    status1_irq.Bits.done = 0xFF;
    status1_irq.Bits.cmndError = 1;

    pmotor->status1_irq_enable.All = status1_irq.All;   /* Enable interrupts. */
    pmotor->status2_irq_enable = 0x0;
    return (OK);
}

/*****************************************************/
/* initialize all software and hardware              */
/* motor_init()                      */
/*****************************************************/
static int motor_init()
{
    struct mess_info *motor_info;
    volatile struct controller *pmotorState;
    volatile struct MAXv_motor *pmotor;
    struct MAXvController *pvtdata;
    long status=0;
    int card_index, motor_index, itera;
    char axis_pos[MAX_IDENT_LEN], encoder_pos[MAX_IDENT_LEN], **strptr;
    char *tok_save, *pos_ptr;
    int total_encoders = 0, total_axis = 0, total_pidcnt = 0;
    volatile void *localaddr=0;
    void *probeAddr;

    tok_save = NULL;

    /* Check for setup */
    if (MAXv_num_cards <= 0)
    {
        Debug(1, "\nmotor_init: MAXv driver disabled\nMAXvSetup() is missing from startup script.\n");
        return(ERROR);
    }

    /* allocate space for total number of motors */
    motor_state = (struct controller **) malloc(MAXv_num_cards *
                                                sizeof(struct controller *));

    /* allocate structure space for each motor present */

    total_cards = MAXv_num_cards;

    if (epicsAtExit(MAXv_reset, NULL) == ERROR)
        Debug(1, "MAXv motor_init: MAXv_reset disabled\n");

    for (card_index = 0; card_index < MAXv_num_cards; card_index++)
    {
        epicsInt8 *startAddr;
        epicsInt8 *endAddr;
        bool wdtrip;

        Debug(2, "motor_init: card %d\n", card_index);

        probeAddr = MAXv_addrs + (card_index * MAXv_brd_size);
        startAddr = (epicsInt8 *) probeAddr;
        endAddr = startAddr + MAXv_brd_size;

        Debug(9, "motor_init: devNoResponseProbe() on addr %p\n", probeAddr);
        /* Scan memory space to assure card id */
#ifdef USE_DEVLIB
        do
        {
            status = devNoResponseProbe(MAXv_ADDRS_TYPE, (unsigned int) startAddr, 2);
            startAddr += (MAXv_brd_size / 10);
        } while (PROBE_SUCCESS(status) && startAddr < endAddr);
#endif
        if (!PROBE_SUCCESS(status))
        {
            Debug(3, "motor_init: Card NOT found!\n");
            motor_state[card_index] = (struct controller *) NULL;
            goto loopend;
        }

#ifdef USE_DEVLIB
        status = devRegisterAddress(__FILE__, MAXv_ADDRS_TYPE,
                                    (size_t) probeAddr, MAXv_brd_size,
                                    (volatile void **) &localaddr);
        Debug(9, "motor_init: devRegisterAddress() status = %d\n", (int) status);
        if (!RTN_SUCCESS(status))
        {
            errPrintf(status, __FILE__, __LINE__, "Can't register address 0x%x\n",
                      (unsigned int) probeAddr);
            motor_state[card_index] = (struct controller *) NULL;
            goto loopend;
        }
#endif

        Debug(9, "motor_init: localaddr = %p\n", localaddr);
        pmotor = (struct MAXv_motor *) localaddr;
            
        if (pmotor->firmware_status.Bits.running == 0)
        {
            errlogPrintf(norunmsg, card_index, (unsigned int) pmotor->firmware_status.All);
            motor_state[card_index] = (struct controller *) NULL;
            goto loopend;
        }

        Debug(9, "motor_init: malloc'ing motor_state\n");
        motor_state[card_index] = (struct controller *) malloc(sizeof(struct controller));
        pmotorState = motor_state[card_index];
        pmotorState->localaddr = (char *) localaddr;
        pmotorState->motor_in_motion = 0;
        pmotorState->cmnd_response = false;

        pvtdata = (struct MAXvController *) malloc(sizeof(struct MAXvController));
        pvtdata->message_mutex = epicsMutexMustCreate();
        pmotorState->DevicePrivate = pvtdata;

        if (MAXvInterruptVector == 0)
            pmotor->IACK_vector = 0;
        else
            pmotor->IACK_vector = MAXvInterruptVector + card_index;

        pmotor->status1_flag.All = 0xFFFFFFFF;
        pmotor->status2_flag = 0xFFFFFFFF;
        /* Disable all interrupts */
        pmotor->status1_irq_enable.All = 0;
        pmotor->status2_irq_enable = 0;

        send_mess(card_index, ERROR_CLEAR, (char) NULL);
        send_mess(card_index, STOP_ALL, (char) NULL);

        send_recv_mess(card_index, GET_IDENT, (char) NULL, (char *) pmotorState->ident, 1);
        Debug(3, "Identification = %s\n", pmotorState->ident);

        /* Save firmware version. */
        pos_ptr = strchr((char *)pmotorState->ident, ':');
        sscanf(++pos_ptr, "%f", &pvtdata->fwver);

        wdtrip = false;

        if (pvtdata->fwver >= 1.33)
        {
            send_recv_mess(card_index, "#WS", (char) NULL, axis_pos, 1);
            if (strcmp(axis_pos, "=0") != 0)
            {
                errlogPrintf(wdctrmsg, card_index, axis_pos);
                epicsThreadSleep(2.0);
                motor_state[card_index] = (struct controller *) NULL;
                wdtrip = true;
            }
        }

        if (wdtrip == false)
        {
            send_mess(card_index, initstring[card_index], (char) NULL);

            send_recv_mess(card_index, ALL_POS, (char) NULL, axis_pos, 1);

            for (total_axis = 0, pos_ptr = epicsStrtok_r(axis_pos, ",", &tok_save);
                 pos_ptr; pos_ptr = epicsStrtok_r(NULL, ",", &tok_save), total_axis++)
            {
                pmotorState->motor_info[total_axis].motor_motion = NULL;
                pmotorState->motor_info[total_axis].status.All = 0;
            }

            Debug(3, "motor_init: Total axis = %d\n", total_axis);
            pmotorState->total_axis = total_axis;

            for (total_encoders = total_pidcnt = 0, motor_index = 0; motor_index < total_axis; motor_index++)
            {
                motor_info = (struct mess_info *) &pmotorState->motor_info[motor_index];
                STATUS1 flag1;

                /* Test if motor has an encoder. */
                send_mess(card_index, ENCODER_QUERY, MAXv_axis[motor_index]);
                while (!pmotor->status1_flag.Bits.done) /* Wait for command to complete. */
                    epicsThreadSleep(quantum);

                if (pmotor->status1_flag.Bits.cmndError)
                {
                    Debug(2, "motor_init: No encoder on axis %d\n", motor_index);
                    motor_info->encoder_present = NO;
                    flag1.All = pmotor->status1_flag.All;       /* Clear command error. */
                    pmotor->status1_flag.All = flag1.All;
                }
                else
                {
                    total_encoders++;
                    motor_info->encoder_present = YES;
                    recv_mess(card_index, encoder_pos, 1);
                }
                
                /* Test if motor has PID parameters. */
                send_mess(card_index, PID_QUERY, MAXv_axis[motor_index]);
                while (!pmotor->status1_flag.Bits.done) /* Wait for command to complete. */
                    epicsThreadSleep(quantum);
                if (pmotor->status1_flag.Bits.cmndError)
                {
                    Debug(2, "motor_init: No PID parameters on axis %d\n", motor_index);
                    motor_info->pid_present = NO;
                    flag1.All = pmotor->status1_flag.All;       /* Clear command error. */
                    pmotor->status1_flag.All = flag1.All;
                }
                else
                {
                    total_pidcnt++;
                    motor_info->pid_present = YES;
                    recv_mess(card_index, encoder_pos, FLUSH);  /* Flush response. */
                }

                // Set motor type.
                if (motor_info->pid_present == YES)
                    pvtdata->typeID[motor_index] = PSM;
                else if (motor_info->encoder_present == YES)
                    pvtdata->typeID[motor_index] = PSE;
                else
                    pvtdata->typeID[motor_index] = PSO;

                if (pvtdata->fwver >= 1.30)
                {
                    send_recv_mess(card_index, "LM?", MAXv_axis[motor_index], axis_pos, 1);
                    if (strcmp(axis_pos, "=f") == 0) /* If limit mode is set to "Off". */
                        send_mess(card_index, "LMH", MAXv_axis[motor_index]); /* Set limit mode to "Hard". */
                }
            }

            /* Enable interrupt-when-done if selected */
            if (MAXvInterruptVector)
            {
                if (motorIsrSetup(card_index) == ERROR)
                    errMessage(-1, "Interrupts Disabled!\n");
            }

            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                motor_info = (struct mess_info *) &pmotorState->motor_info[motor_index];

                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;

                if (motor_info->encoder_present == YES)
                    motor_info->status.Bits.EA_PRESENT = 1;
                if (motor_info->pid_present == YES)
                    motor_info->status.Bits.GAIN_SUPPORT = 1;

                set_status(card_index, motor_index);
                /* Is this needed??? */
                send_recv_mess(card_index, DONE_QUERY, MAXv_axis[motor_index], axis_pos, 1);
            }

            Debug(2, "motor_init: Init Address=%p\n", localaddr);
            Debug(3, "motor_init: Total encoders = %d\n", total_encoders);
            Debug(3, "motor_init: Total with PID = %d\n", total_pidcnt);
        }
loopend:;
    }

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    Debug(3, "Motors initialized\n");

    epicsThreadCreate((char *) "MAXv_motor", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    Debug(3, "Started motor_task\n");

    /* Deallocate memory for configuration strings. */
    for (itera = 0, strptr = &initstring[0]; itera < MAXv_num_cards; itera++, strptr++)
        free(*strptr);
    free(initstring);
    initstring = NULL;

    return(0);
}


/* Disables interrupts. Called on CTL X reboot. */

static void MAXv_reset(void *arg)
{
    short card;
    volatile struct MAXv_motor *pmotor;

    for (card = 0; card < total_cards; card++)
    {
        if (motor_state[card] != NULL)
        {
            pmotor = (struct MAXv_motor *) motor_state[card]->localaddr;
            pmotor->status1_irq_enable.All = 0;
        }
    }
}

extern "C"
{

// Oms Setup arguments
    static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
    static const iocshArg setupArg1 = {"VME address type", iocshArgInt};
    static const iocshArg setupArg2 = {"Base Address on 4K (0x1000) boundary", iocshArgInt};
    static const iocshArg setupArg3 = {"noninterrupting(0), valid vectors(64-255)", iocshArgInt};
    static const iocshArg setupArg4 = {"interrupt level (1-6)", iocshArgInt};
    static const iocshArg setupArg5 = {"polling rate (Hz)", iocshArgInt};
// Oms Config arguments
    static const iocshArg configArg0 = {"Card # being configured", iocshArgInt};
    static const iocshArg configArg1 = {"configuration string", iocshArgString};
    static const iocshArg configArg2 = {"absolute encoder flags (0/1 - incremental/absolute, 0x07 -> ZYX)", iocshArgInt};

    static const iocshArg * const OmsSetupArgs[6] = {&setupArg0, &setupArg1,
        &setupArg2, &setupArg3, &setupArg4, &setupArg5};
    static const iocshArg * const OmsConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

    static const iocshFuncDef setupMAXv = {"MAXvSetup", 6, OmsSetupArgs};

    static const iocshFuncDef configMAXv = {"MAXvConfig", 3, OmsConfigArgs};

    static void setupMAXvCallFunc(const iocshArgBuf *args)
    {
        MAXvSetup(args[0].ival, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
    }

    static void configMAXvCallFunc(const iocshArgBuf *args)
    {
        MAXvConfig(args[0].ival, args[1].sval, args[2].ival);
    }

    static void OmsMAXvRegister(void)
    {
        iocshRegister(&setupMAXv, setupMAXvCallFunc);
        iocshRegister(&configMAXv, configMAXvCallFunc);
    }

    epicsExportRegistrar(OmsMAXvRegister);

} // extern "C"
