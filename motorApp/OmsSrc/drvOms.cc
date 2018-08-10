/*
FILENAME...     drvOms.cc
USAGE...        Driver level support for OMS models VME8, VME44, VS4 and VX2.

*/

/*
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
 *
 * NOTES
 * -----
 * Verified with firmware:
 *      - VME44    ver 1.97-4E
 *      - VME8     ver 1.97-8
 *                 ver 2.16-8
 *      - VS4-040  ver 1.04
 *      - VX2-006  ver 1.05 (1.04 has control register initialization problem)
 *
 *
 * Modification Log:
 * -----------------
 * .00  02-22-02 rls - "total_cards" changed from total detected to total
 *                      cards that "memory is allocated for".  This allows
 *                      boards after the "hole" to work.
 * .01  06-05-03 rls - Convert to R3.14.x.
 * .02  06-05-03 rls - extended device directive support to PREM and POST.
 * .03  10-23-03 rls - VX2 spurious interrupt fix; support transmit buffer
 *                      empty interrupt in omsPut() and motorIsr().
 * .04  10-28-03 rls - moved OMS specific "irqdatastr" from motordrvCom.h
 *                      drvOms.h and DevicePrivate.
 *                   - removed "max_io_tries" from timeout calculations.
 *                   - changed omsGet() timeout argument to type bool.
 *                   - changed recv_rng and send_rng from C to C++ interface.
 * .05  12-03-03 rls - update rate bug fix.
 * .06  12-12-03 rls - Converted MSTA #define's to bit field.
 *                   - Two lines of code must be selected based on either
 *                      Tornado 2.0.2 (default) or Tornado 2.2.  If Tornado
 *                      2.2 is selected, EPICS base patches must be applied as
 *                      described in;
 *              http://www.aps.anl.gov/upd/people/sluiter/epics/motor/R5-2/Problems.html
 * .07  02-03-04 rls - Eliminate erroneous "Motor motion timeout ERROR".
 * .08  03-02-04 rls - Reduce omsGet() timeout from 1sec. to 250msec.
 * .09  09-20-04 rls - support for 32axes/controller.
 * .10  12-06-04 rls - Windows compiler support.
 *                   - eliminate calls to devConnectInterrupt() due to C++
 *                     problems with devLib.h; i.e. "sorry, not implemented:
 *                     `tree_list' not supported..." compiler error message.
 * .11  03-23-05 rls - Make OSI.
 * .12  05-02-05 rls - Bug fix for stale data delay; set delay = 10ms.
 * .13  03-14-08 rls - 64-bit compatiability.
 *                   - changed IRS to task comm. mechanism from epicsRingPointer
 *                     to epicsRingByte.
 * .14  06-18-09 rls - Make omsSetup() error messages more prominent.
 * .15  03-15-10 rls - sprintf() not callable from any OS ISR.
 * .16  04-12-10 rls - enable interrupts after encoder check in motor_init() so
 *                     user does not see "motorIsr: command error" messages at
 *                     iocInit time.
 * .17  07-26-12 rls - Added reboot flag to IRQ control register. Driver
 *                     sets IRQ_RESET_ID bit on; set_status() and send_mess()
 *                     read IRQ register and disable board if flag is off.
 * .18  11-15-03 rls - Added bus flushing read on exit from ISR to prevent
 *                     "out of order transactions".
 */

/*========================stepper motor driver ========================

 function:
    Allow users to queue messages to axis on a OMS stepper
    motor controller board.  Each axis of every board available can
    be accessed independantly.

 public functions:
    motor_init() -      Initialize the driver task and structures for all
            boards available for the system.
 private functions:
    send_mess() -       Send a message to the OMS board.
    recv_mess() -       Receive a message from the OMS board.


========================stepper motor driver ========================*/

#include        <string.h>
#include        <stdio.h>
#include        <drvSup.h>
#include        <epicsVersion.h>
#include        <epicsString.h>
#include        <devLib.h>
#include        <dbAccess.h>
#include        <epicsThread.h>
#include        <epicsInterrupt.h>
#include        <epicsExit.h>
#include        <epicsEvent.h>
#include        <errlog.h>
#include        <stdlib.h>

#include        "motor.h"
#include        "drvOms.h"
#include        "epicsExport.h"
#include        "iocsh.h"


/* Define for return test on devNoResponseProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

#define CMD_CLEAR       '\030'  /* Control-X, clears command errors only */

#define ALL_INFO        "QA RP RE EA"   /* jps: move QA to top. */
#define AXIS_INFO       "QA RP"         /* jps: move QA to top. */
#define ENCODER_QUERY   "EA"
#define DONE_QUERY      "RA"

/*----------------debugging-----------------*/
int drvOMSdebug = 0;
extern "C" {epicsExportAddress(int, drvOMSdebug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvOMSdebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* Global data. */
unsigned int oms44_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/* --- Local data common to all OMS drivers. --- */
static epicsUInt32 oms_addrs = 0x0;
static unsigned omsInterruptVector = 0;
static epicsUInt8 omsInterruptLevel = OMS_INT_LEVEL;
static int motionTO = 10;
static const char *oms_axis[] = {"X", "Y", "Z", "T", "U", "V", "R", "S"};
static double quantum;

/*----------------functions-----------------*/

/* Common local function declarations. */
static long report(int);
static long init();
static void query_done(int, int, struct mess_node *);
static int set_status(int, int);
static RTN_STATUS send_mess(int, const char *, const char *);
static int recv_mess(int, char *, int);
static void motorIsr(int);
static int motor_init();
static void oms_reset(void *);

static int omsGet(int card, char *pcom, bool timeout);
static RTN_STATUS omsPut(int card, char *pcom);
static int omsError(int card);
static int motorIsrEnable(int card);
static void motorIsrDisable(int card);

struct driver_table oms_access =
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
    oms_axis
};

struct drvOms_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvOms = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvOms);}

static struct thread_args targs = {SCAN_RATE, &oms_access, 0.010};
static const char rebootmsg[] = "\n\n*** OMS card #%d Disabled *** Reboot Detected.\n\n";

/*----------------functions-----------------*/

static long report(int level)
{
    unsigned int card;

    if (oms44_num_cards <= 0)
        printf("    No VME8/44 controllers configured.\n");
    else
    {
        for (card = 0; card < oms44_num_cards; card++)
            if (motor_state[card])
                printf("    Oms VME8/44 motor card %u @ %p, id: %s \n", card,
                       motor_state[card]->localaddr, motor_state[card]->ident);
    }
    return(0);
}

static long init()
{
    initialized = true; /* Indicate that driver is initialized. */
    (void) motor_init();
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
    char buffer[40];

    send_mess(card, DONE_QUERY, oms_axis[axis]);
    recv_mess(card, buffer, 1);

    if (nodeptr->status.Bits.RA_PROBLEM)
        send_mess(card, AXIS_STOP, oms_axis[axis]);
}


static int set_status(int card, int signal)
{
    struct mess_info *motor_info;
    struct mess_node *nodeptr;
    char *p, *tok_save;
    struct axis_status *ax_stat;
    struct encoder_status *en_stat;
    char q_buf[50], outbuf[50];
    int index, motorData;
    int rtn_state;
    bool ls_active = false;
    msta_field status;
    struct controller *pmotorState;
    struct vmex_motor *pmotor;

    if ((pmotorState = motor_state[card]) == NULL ||
        (pmotor = (struct vmex_motor *) pmotorState->localaddr) == NULL)
        return(rtn_state = 1); /* End move. */

    motor_info = &(pmotorState->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    if ((pmotor->control & IRQ_RESET_ID) != 0x01)   /* Test if board has rebooted. */
    {
        errlogPrintf(rebootmsg, card);
        status.Bits.RA_PROBLEM = 1;
        motor_info->status.All = status.All;
        /* Disable board. */
        motor_state[card] = NULL;
        return(rtn_state = 1); /* End move. */
    }

    if (motor_state[card]->motor_info[signal].encoder_present == YES)
    {
        /* get 4 peices of info from axis */
        send_mess(card, ALL_INFO, oms_axis[signal]);
        recv_mess(card, q_buf, 4);
    }
    else
    {
        send_mess(card, AXIS_INFO, oms_axis[signal]);
        recv_mess(card, q_buf, 2);
    }

    Debug(5, "info = (%s)\n", q_buf);

    for (index = 0, p = epicsStrtok_r(q_buf, ",", &tok_save); p;
        p = epicsStrtok_r(NULL, ",", &tok_save), index++)
    {
        switch (index)
        {
        case 0:     /* axis status */
            ax_stat = (struct axis_status *) p;

            status.Bits.RA_DIRECTION = (ax_stat->direction == 'P') ? 1 : 0;
            status.Bits.RA_DONE =      (ax_stat->done == 'D')      ? 1 : 0;
            status.Bits.RA_HOME =      (ax_stat->home == 'H')      ? 1 : 0;

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
        case 1:     /* motor pulse count (position) */
            sscanf(p, "%index", &motorData);

            if (motorData == motor_info->position)
            {
                if (nodeptr != 0)   /* Increment counter only if motor is moving. */
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
                send_mess(card, AXIS_STOP, oms_axis[signal]);
                motor_info->no_motion_count = 0;
                errlogSevPrintf(errlogMinor, "Motor motion timeout ERROR on card: %d, signal: %d\n",
                                card, signal);
            }
            else
                status.Bits.RA_PROBLEM = 0;

            break;
        case 2:     /* encoder pulse count (position) */
            {
                int temp;

                sscanf(p, "%index", &temp);
                motor_info->encoder_position = (epicsInt32) temp;
            }
            break;
        case 3:     /* encoder status */
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

    /*
     * jps: Velocity should be set based on the actual velocity returned from
     * the 'RV' command (See drvOms58.c). But the polling task does not have
     * time to request additional information so the velocity is set to
     * indicate moving or not-moving.
     */
    if (status.Bits.RA_DONE)
        motor_info->velocity = 0;
    else
        motor_info->velocity = 1;

    if (!(status.Bits.RA_DIRECTION))
        motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
                 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && (nodeptr != 0) &&
        (nodeptr->postmsgptr != 0))
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
                buffer[size] = 0;

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
        send_mess(card, outbuf, oms_axis[signal]);
        nodeptr->postmsgptr = NULL;
    }

    motor_info->status.All = status.All;    /* Update status from local copy. */
    return(rtn_state);
}


/*****************************************************/
/* send a message to the OMS board                   */
/*              send_mess()                          */
/*****************************************************/
static RTN_STATUS send_mess(int card, const char *com, const char *name)
{
    char outbuf[MAX_MSG_SIZE];
    RTN_STATUS return_code;
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;

    if (strlen(com) > MAX_MSG_SIZE)
    {
        errlogPrintf("drvOms.cc:send_mess(); message size violation.\n");
        return(ERROR);
    }

    /* Check that card exists */
    if ((pmotorState = motor_state[card]) == NULL)
    {
        errlogPrintf("drvOms.cc:send_mess() - invalid card #%d\n", card);
        return(ERROR);
    }

    pmotor = (struct vmex_motor *) pmotorState->localaddr;

    if ((pmotor->control & IRQ_RESET_ID) != 0x01)    /* Test if board has rebooted. */
    {
        errlogPrintf(rebootmsg, card);
        /* Disable board. */
        motor_state[card] = NULL;
        return(ERROR);
    }

    /* Check/Clear command errors */
    omsError(card);

    /* Flush receive buffer */
    recv_mess(card, NULL, -1);

    if (name == NULL)
        strcpy(outbuf, com);
    else
    {
        strcpy(outbuf, "A");
        strcat(outbuf, name);
        strcat(outbuf, " ");
        strcat(outbuf, com);
    }
    strcat(outbuf, "\n");       /* Add the command line terminator. */

    Debug(9, "send_mess: ready to send message.\n");

    return_code = omsPut(card, outbuf);

    if (return_code == OK)
    {
        Debug(4, "sent message: (%s)\n", outbuf);
    }
    else
    {
        Debug(4, "unable to send message (%s)\n", outbuf);
    }
    return(return_code);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int amount)
 *
 * INPUT ARGUMENTS...
 *      card - controller card # (0,1,...).
 *      *com - caller's response buffer.
 *      amount  | -1 = flush controller's output buffer.
 *              | >= 1 = the # of command responses to retrieve into caller's
 *                              response buffer.
 *
 * LOGIC...
 *  IF controller card does not exist.
 *      ERROR RETURN.
 *  ENDIF
 *  IF "amount" indicates buffer flush.
 *      WHILE characters left in input buffer.
 *          Call omsGet().
 *      ENDWHILE
 *  ENDIF
 *
 *  FOR each message requested (i.e. "amount").
 *      Initialize head and tail pointers.
 *      Initialize retry counter and state indicator.
 *      WHILE retry count not exhausted, AND, state indicator is NOT at END.
 *          IF characters left in controller's input buffer.
 *              Process input character.
 *          ELSE IF command error occured - call omsError().
 *              ERROR RETURN.
 *          ENDIF
 *      ENDWHILE
 *      IF retry count exhausted.
 *          Terminate receive buffer.
 *          ERROR RETURN.
 *      ENDIF
 *      Terminate command response.
 *  ENDFOR
 *
 *  IF commands processed.
 *      Terminate response buffer.
 *  ELSE
 *      Clear response buffer.
 *  ENDIF
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int amount)
{
    int i, trys;
    char junk;
    char inchar;
    int piece, head_size, tail_size;

    inchar = '\0';

    /* Check that card exists */
    if (card >= total_cards)
    {
        Debug(1, "recv_mess - invalid card #%d\n", card);
        return(-1);
    }

    if (amount == -1)
    {
        /* Process request to flush receive queue */
        Debug(7, "recv flush -------------");
        while (omsGet(card, &junk, false))
        {
            Debug(7, "%inchar", junk);
        }
        Debug(7, "         -------------");
        return(0);
    }

    for (i = 0; amount > 0; amount--)
    {
        Debug(7, "-------------");
        head_size = 0;
        tail_size = 0;

        for (piece = 0, trys = 0; piece < 3 && trys < 3; trys++)
        {
            if (omsGet(card, &inchar, true))
            {
                Debug(7, "%02x", inchar);

                switch (piece)
                {
                case 0: /* header */
                    if (inchar == '\n' || inchar == '\r')
                        head_size++;
                    else
                    {
                        piece++;
                        com[i++] = inchar;
                    }
                    break;
                case 1: /* body */
                    if (inchar == '\n' || inchar == '\r')
                    {
                        piece++;
                        tail_size++;
                    }
                    else
                        com[i++] = inchar;
                    break;

                case 2: /* trailer */
                    tail_size++;
                    if (tail_size >= head_size)
                        piece++;
                    break;
                }

                trys = 0;
            }
            else if (omsError(card))
                /* Command error detected - abort recv */
                return(-1);
        }
        Debug(7, "-------------\n");
        if (trys >= 3)
        {
            Debug(1, "Timeout occurred in recv_mess\n");
            com[i] = '\0';
            return(-1);
        }
        com[i++] = ',';
    }

    if (i > 0)
        com[i - 1] = '\0';
    else
        com[i] = '\0';

    Debug(4, "recv_mess: card %d, msg: (%s)\n", card, com);
    return(0);
}


/*****************************************************/
/* Get next character from OMS input buffer          */
/*              omsGet()                             */
/*****************************************************/
static int omsGet(int card, char *pchar, bool timeout)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    struct irqdatastr *irqdata;
    int getCnt = 0;
    int retry = 0;

    pmotorState = motor_state[card];
    irqdata = (struct irqdatastr *) pmotorState->DevicePrivate;

    if (irqdata->irqEnable)
    {
        /* Get character from isr - if available */
        while (epicsRingBytesIsEmpty(irqdata->recv_rng) && timeout == true && retry < 5)
        {
            irqdata->recv_sem->wait(0.05);  /* Wait 250ms for character */
            retry ++;
        }
        if (!epicsRingBytesIsEmpty(irqdata->recv_rng))
        {
            epicsRingBytesGet(irqdata->recv_rng, pchar, 1);
            getCnt = 1;
        }
    }
    else
    {
        int maxtrys = (int) (0.250 / quantum);  /* Wait 250ms for character */

        /* Direct read from card */
        pmotor = (struct vmex_motor *) pmotorState->localaddr;

        if (timeout == true)
            while (retry++ < maxtrys && !(pmotor->status & STAT_INPUT_BUF_FULL))
            {
                Debug(5, "omsGet: wait count = %d\n", retry);
                epicsThreadSleep(quantum);
            }

        if (pmotor->status & STAT_INPUT_BUF_FULL)
        {
            getCnt++;
            *pchar = pmotor->data;
        }
    }
    return(getCnt);
}

/*****************************************************/
/* Send Message to OMS                               */
/*              omsPut()                             */
/*****************************************************/
static RTN_STATUS omsPut(int card, char *pmess)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    struct irqdatastr *irqdata;
    int key, msgsize;

    if ((pmotorState = motor_state[card]) == NULL)
        return(ERROR);
    irqdata = (struct irqdatastr *) pmotorState->DevicePrivate;
    pmotor = (struct vmex_motor *) pmotorState->localaddr;
    msgsize = strlen(pmess);

    if (irqdata->irqEnable)
    {
        /* Put string into isr transmitt buffer */
        if (epicsRingBytesPut(irqdata->send_rng, pmess, msgsize) != msgsize)
        {
            errlogPrintf("omsPut: card %d send ring full!\n", card);
            return(ERROR);
        }

        /* Turn-on transmit buffer interrupt */
        key = epicsInterruptLock();
        pmotor->control |= IRQ_TRANS_BUF;
        epicsInterruptUnlock(key);
    }
    else
    {
        char *putptr;

        /* Send next message */
        for (putptr = pmess; *putptr != '\0'; putptr++)
        {
            int trys = 0;
            int maxtrys = (int) (0.01 / quantum);

            while (!(pmotor->status & STAT_TRANS_BUF_EMPTY))
            {
                if (trys > maxtrys) /* Set timeout to 0.01 sec. */
                {
                    Debug(1, "omsPut: Time_out occurred in send\n");
                    return(ERROR);
                }
                if (pmotor->status & STAT_ERROR)
                {
                    Debug(1, "omsPut: error occurred in send\n");
                }
                trys++;
                Debug(5, "omsPut: wait count = %d\n", trys);
                epicsThreadSleep(quantum);
            }
            pmotor->data = *putptr;
        }
    }
    return(OK);
}


/*****************************************************/
/* Clear OMS errors                                  */
/*              omsClearErrors()                     */
/*****************************************************/
static int omsError(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    struct irqdatastr *irqdata;
    int rtnStat = FALSE;

    pmotorState = motor_state[card];
    irqdata = (struct irqdatastr *) pmotorState->DevicePrivate;
    pmotor = (struct vmex_motor *) pmotorState->localaddr;

    if (irqdata->irqEnable)
    {
        /* Check status of last message */
        if (irqdata->irqErrno & STAT_ERROR)
        {
            /* Error on the card is cleared by the ISR */
            irqdata->irqErrno &= ~STAT_ERROR;
            rtnStat = TRUE;
        }
    }
    else
    {
        int i;
        const char *p;

        /* Check/Clear command error from last message */
        if ((pmotor->status) & STAT_ERROR)
        {
            Debug(1, "omsPut: Error detected! 0x%02x\n", pmotor->status);
            for (p = ERROR_CLEAR; *p != '\0'; p++)
            {
                while (!(pmotor->status & STAT_TRANS_BUF_EMPTY));
                pmotor->data = *p;
            }
            for (i = 0; i < 20000; i++);
            rtnStat = TRUE;
        }
    }
    return(rtnStat);
}


/*****************************************************/
/* Interrupt service routine.                        */
/* motorIsr()                                */
/*****************************************************/
static void motorIsr(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    struct irqdatastr *irqdata;
    epicsUInt8 control;
    epicsUInt8 status;
    epicsUInt8 doneFlags;
    char dataChar;
    static char errmsg1[] = "\ndrvOms.cc:motorIsr: Invalid entry - card xx\n";
    static char errmsg2[] = "\ndrvOms.cc:motorIsr: command error - card xx\n";
    static char errmsg3[] = "\ndrvOms.cc:motorIsr: Rx ring overflow - card xx\n";

    if (card >= total_cards || (pmotorState = motor_state[card]) == NULL)
    {
        errmsg1[45-2] = '0' + card%10;
        errmsg1[45-3] = '0' + (card/10)%10;
        epicsInterruptContextMessage(errmsg1);
        return;
    }

    irqdata = (struct irqdatastr *) pmotorState->DevicePrivate;
    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    /* Save interrupt state */
    control = pmotor->control;

    /* Status register - clear irqs on read. */
    status = pmotor->status;

    /* Done register - clears on read */
    doneFlags = pmotor->done;

    /* Determine cause of entry */

    /* Motion done handling */
    if (status & STAT_DONE)
        /* Wake up polling task 'motor_task()' to issue callbacks */
        motor_sem.signal();

    /* If command error is present - clear it */
    if (status & STAT_ERROR)
    {
        pmotor->data = (epicsUInt8) CMD_CLEAR;
        errmsg2[45-2] = '0' + card%10;
        errmsg2[45-3] = '0' + (card/10)%10;
        epicsInterruptContextMessage(errmsg2);
        irqdata->irqErrno |= STAT_ERROR;
    }

    /* Send message */
    if (status & STAT_TRANS_BUF_EMPTY)
    {
        if (epicsRingBytesIsEmpty(irqdata->send_rng))
            control &= ~IRQ_TRANS_BUF;  /* Transmit done - disable irq */
        else
        {
            epicsRingBytesGet(irqdata->send_rng, &dataChar, 1);
            pmotor->data = dataChar;
        }
    }

    /* Read Response */
    if (status & STAT_INPUT_BUF_FULL)
    {
        dataChar = pmotor->data;

        if (epicsRingBytesPut(irqdata->recv_rng, &dataChar, 1) == 0)
        {
            errmsg3[48-2] = '0' + card%10;
            errmsg3[48-3] = '0' + (card/10)%10;
            epicsInterruptContextMessage(errmsg3);
            irqdata->irqErrno |= STAT_INPUT_BUF_FULL;
        }
        irqdata->recv_sem->signal();
    }
    pmotor->control = control;  /* Update-interrupt state. */
    control = pmotor->control;  /* Read it back to flush last write cycle. */
}

static int motorIsrEnable(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    struct irqdatastr *irqdata;
    epicsUInt8 cardStatus;
    long status;

    Debug(5, "motorIsrEnable: Entry card#%d\n", card);

    pmotorState = motor_state[card];
    irqdata = (struct irqdatastr *) pmotorState->DevicePrivate;
    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    status = pdevLibVirtualOS->pDevConnectInterruptVME(
                                                      omsInterruptVector + card,
#if LT_EPICSBASE(3,14,8,0)
                                                      (void (*)()) motorIsr,
#else
                                                      (void (*)(void *)) motorIsr,
#endif
                                                      (void *)(size_t) card);

    if (!RTN_SUCCESS(status))
    {
        errPrintf(status, __FILE__, __LINE__,
                  "Can't connect to vector %d\n",
                  omsInterruptVector + card);
        irqdata->irqEnable = FALSE; /* Interrupts disable on card */
        pmotor->control = IRQ_RESET_ID;
        return(ERROR);
    }

    status = devEnableInterruptLevel(OMS_INTERRUPT_TYPE,
                                     omsInterruptLevel);
    if (!RTN_SUCCESS(status))
    {
        errPrintf(status, __FILE__, __LINE__,
                  "Can't enable enterrupt level %d\n",
                  omsInterruptLevel);
        irqdata->irqEnable = FALSE; /* Interrupts disable on card */
        pmotor->control = IRQ_RESET_ID;
        return(ERROR);
    }

    /* Setup card for interrupt-on-done */
    pmotor->vector = omsInterruptVector + card;

    irqdata->recv_rng = epicsRingBytesCreate(OMS_RESP_Q_SZ);
    irqdata->recv_sem = new epicsEvent(epicsEventEmpty);

    irqdata->send_rng = epicsRingBytesCreate(MAX_MSG_SIZE * 2);

    irqdata->irqEnable = TRUE;
    irqdata->irqErrno = 0;

    /* Clear board status */
    cardStatus = pmotor->status;

    /* enable interrupt-when-done and input-buffer-full interrupts */
    pmotor->control = (IRQ_ENABLE_ALL | IRQ_RESET_ID);
    return(OK);
}

static void motorIsrDisable(int card)
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    struct irqdatastr *irqdata;
    long status;

    Debug(5, "motorIsrDisable: Entry card#%d\n", card);

    pmotorState = motor_state[card];
    irqdata = (struct irqdatastr *) pmotorState->DevicePrivate;
    pmotor = (struct vmex_motor *) (pmotorState->localaddr);

    /* Disable interrupts */
    pmotor->control = IRQ_RESET_ID;

    status = pdevLibVirtualOS->pDevDisconnectInterruptVME(omsInterruptVector + card, (void (*)(void *)) motorIsr);

    if (!RTN_SUCCESS(status))
        errPrintf(status, __FILE__, __LINE__, "Can't disconnect vector %d\n",
                  omsInterruptVector + card);

    /* Remove interrupt control functions */
    irqdata->irqEnable = FALSE;
    irqdata->irqErrno = 0;
    epicsRingBytesDelete(irqdata->recv_rng);
    epicsRingBytesDelete(irqdata->send_rng);
    delete irqdata->recv_sem;
}


/*****************************************************/
/* Configuration function for  module_types data     */
/* areas. omsSetup()                                */
/*****************************************************/
RTN_STATUS
omsSetup(int num_cards,  /* maximum number of cards in rack */
         unsigned addrs, /* Base Address(see README for details) */
         unsigned vector,/* noninterrupting(0), valid vectors(64-255) */
         int int_level,  /* interrupt level (1-6) */
         int scan_rate)  /* polling rate - 1-60 Hz */
{
    RTN_STATUS rtncode = OK;
    char errbase[] = "\nomsSetup: *** invalid ";

    if (num_cards < 1 || num_cards > OMS_NUM_CARDS)
    {
        char format[] = "%snumber of cards specified = %d ***\n";
        oms44_num_cards = OMS_NUM_CARDS;
        errlogPrintf(format, errbase, num_cards);
        errlogPrintf("             *** using maximum number = %d ***\n", OMS_NUM_CARDS);
        epicsThreadSleep(5.0);
        rtncode = ERROR;
    }
    else
        oms44_num_cards = num_cards;

    /* Check boundary(16byte) on base address */
    if (addrs & 0xF)
    {
        char format[] = "%s invalid address 0x%x  ***\n";
        omsInterruptVector = (unsigned) OMS_INT_VECTOR;
        errlogPrintf(format, errbase, addrs);
        rtncode = ERROR;
    }
    oms_addrs = addrs;

    omsInterruptVector = vector;
    if (vector < 64 || vector > 255)
    {
        if (vector != 0)
        {
            char format[] = "%s interrupt vector %d ***\n";
            omsInterruptVector = (unsigned) OMS_INT_VECTOR;
            errlogPrintf(format, errbase, vector);
            epicsThreadSleep(5.0);
            rtncode = ERROR;
        }
    }

    if (int_level < 1 || int_level > 6)
    {
        char format[] = "%s interrupt level %d ***\n";
        omsInterruptLevel = OMS_INT_LEVEL;
        errlogPrintf(format, errbase, int_level);
        epicsThreadSleep(5.0);
        rtncode = ERROR;
    }
    else
        omsInterruptLevel = int_level;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= MAX_SCAN_RATE)
        targs.motor_scan_rate = scan_rate;
    else
    {
        char format[] = "%s invalid poll rate - %d HZ\n";
        targs.motor_scan_rate = SCAN_RATE;
        errlogPrintf(format, errbase, scan_rate);
        epicsThreadSleep(5.0);
        rtncode = ERROR;
    }
    return(rtncode);
}

/*****************************************************/
/* initialize all software and hardware              */
/*              motor_init()                         */
/*****************************************************/
static int motor_init()
{
    volatile struct controller *pmotorState;
    volatile struct vmex_motor *pmotor;
    long status = 0;
    unsigned int card_index, motor_index;
    char axis_pos[50], encoder_pos[50];
    char *tok_save, *pos_ptr;
    unsigned int total_encoders = 0, total_axis = 0;
    volatile void *localaddr;
    epicsUInt32 probeAddr;

    tok_save = NULL;
    quantum = epicsThreadSleepQuantum();

    /* Check for setup */
    if (oms44_num_cards <= 0)
    {
        Debug(1, "motor_init: *OMS driver disabled* \n omsSetup() is missing from startup script.\n");
        return(ERROR);
    }

    /* allocate space for total number of motors */
    motor_state = (struct controller **) malloc(oms44_num_cards *
                                                sizeof(struct controller *));

    /* allocate structure space for each motor present */

    total_cards = oms44_num_cards;

    if (epicsAtExit(oms_reset, NULL) == ERROR)
        Debug(1, "vme8/44 motor_init: oms_reset disabled\n");

    for (card_index = 0; card_index < oms44_num_cards; card_index++)
    {
        epicsUInt32 startAddr;
        epicsUInt32 endAddr;

        Debug(2, "motor_init: card %d\n", card_index);

        probeAddr = oms_addrs + (card_index * OMS_BRD_SIZE);
        startAddr = probeAddr + 1;
        endAddr = startAddr + OMS_BRD_SIZE;

        Debug(9, "motor_init: devNoResponseProbe() on addr 0x%x\n", probeAddr);
        /* Scan memory space to assure card id */
        do
        {
            status = devNoResponseProbe(OMS_ADDRS_TYPE, startAddr, 1);
            startAddr += 0x2;
        } while (PROBE_SUCCESS(status) && startAddr < endAddr);

        if (PROBE_SUCCESS(status))
        {
            struct irqdatastr *irqdata;

            status = devRegisterAddress(__FILE__, OMS_ADDRS_TYPE,
                                        probeAddr, OMS_BRD_SIZE,
                                        &localaddr);
            Debug(9, "motor_init: devRegisterAddress() status = %ld\n",
                  status);
            if (!RTN_SUCCESS(status))
            {
                errPrintf(status, __FILE__, __LINE__,
                          "Can't register address 0x%x\n", probeAddr);
                return(ERROR);
            }

            Debug(9, "motor_init: localaddr = %p\n", localaddr);
            pmotor = (struct vmex_motor *) localaddr;

            Debug(9, "motor_init: malloc'ing motor_state\n");
            motor_state[card_index] = (struct controller *) malloc(sizeof(struct controller));
            pmotorState = motor_state[card_index];
            pmotorState->localaddr = (char*) localaddr;
            pmotorState->motor_in_motion = 0;
            pmotorState->cmnd_response = false;

            /* Disable Interrupts */
            irqdata = (struct irqdatastr *) malloc(sizeof(struct irqdatastr));
            pmotorState->DevicePrivate = irqdata;
            irqdata->irqEnable = FALSE;
            pmotor->control = IRQ_RESET_ID;

            send_mess(card_index, "EF", NULL);
            send_mess(card_index, ERROR_CLEAR, NULL);
            send_mess(card_index, STOP_ALL, NULL);

            send_mess(card_index, GET_IDENT, NULL);

            recv_mess(card_index, (char *) pmotorState->ident, 1);
            Debug(3, "Identification = %s\n", pmotorState->ident);

            send_mess(card_index, ALL_POS, NULL);
            recv_mess(card_index, axis_pos, 1);

            for (total_axis = 0, pos_ptr = epicsStrtok_r(axis_pos, ",", &tok_save);
                pos_ptr;
                pos_ptr = epicsStrtok_r(NULL, ",", &tok_save), total_axis++)
            {
                pmotorState->motor_info[total_axis].motor_motion = NULL;
                pmotorState->motor_info[total_axis].status.All = 0;
            }

            Debug(3, "Total axis = %d\n", total_axis);
            pmotorState->total_axis = total_axis;

            for (total_encoders = 0, motor_index = 0; motor_index < total_axis; motor_index++)
            {
                send_mess(card_index, ENCODER_QUERY, oms_axis[motor_index]);
                if (recv_mess(card_index, encoder_pos, 1) == -1)
                {
                    /* Command error - no encoder */
                    Debug(2, "No encoder on %d\n", motor_index);
                    pmotorState->motor_info[motor_index].encoder_present = NO;
                }
                else
                {
                    total_encoders++;
                    pmotorState->motor_info[motor_index].encoder_present = YES;
                }
            }

            /* Enable interrupt-when-done if selected. */

            if (omsInterruptVector)
            {
                if (motorIsrEnable(card_index) == ERROR)
                    errPrintf(0, __FILE__, __LINE__, "Interrupts Disabled!\n");
            }

            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                pmotorState->motor_info[motor_index].status.All = 0;
                pmotorState->motor_info[motor_index].no_motion_count = 0;
                pmotorState->motor_info[motor_index].encoder_position = 0;
                pmotorState->motor_info[motor_index].position = 0;

                if (pmotorState->motor_info[motor_index].encoder_present == YES)
                    pmotorState->motor_info[motor_index].status.Bits.EA_PRESENT = 1;
                set_status(card_index, motor_index);
            }

            Debug(2, "Init Address=%p\n", localaddr);
            Debug(3, "Total encoders = %d\n\n", total_encoders);
        }
        else
        {
            Debug(3, "motor_init: Card NOT found!\n");
            motor_state[card_index] = NULL;
        }
    }

    any_motor_in_motion = 0;

    mess_queue.head = NULL;
    mess_queue.tail = NULL;

    free_list.head = NULL;
    free_list.tail = NULL;

    Debug(3, "Motors initialized\n");

    epicsThreadCreate((const char *) "Oms_motor", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    Debug(3, "Started motor_task\n");
    return(0);
}

/* Disables interrupts. Called on CTL X reboot. */

static void oms_reset(void *arg)
{
    short card;
    struct vmex_motor *pmotor;

    for (card = 0; card < total_cards; card++)
    {
        if (motor_state[card] != NULL)
        {
            pmotor = (struct vmex_motor *) motor_state[card]->localaddr;
            pmotor->control = IRQ_RESET_ID;    /* Disable all interrupts. */
        }
    }
}

/* Epics iocsh bindings */

static const iocshArg omsArg0 = {"num_card",  iocshArgInt};
static const iocshArg omsArg1 = {"addrs",     iocshArgInt};
static const iocshArg omsArg2 = {"vector",    iocshArgInt};
static const iocshArg omsArg3 = {"int_level", iocshArgInt};
static const iocshArg omsArg4 = {"scan_rate", iocshArgInt};

static const iocshArg* const omsArgs[5] = {&omsArg0, &omsArg1, &omsArg2, &omsArg3, &omsArg4};

static const iocshFuncDef omsFuncDef = {"omsSetup", 5, omsArgs};

static void omsCallFunc(const iocshArgBuf* args)
{
	omsSetup(args[0].ival, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}

void omsRegistrar(void)
{
	iocshRegister(&omsFuncDef, &omsCallFunc);
}

extern "C"{
epicsExportRegistrar(omsRegistrar);
}


/*---------------------------------------------------------------------*/
