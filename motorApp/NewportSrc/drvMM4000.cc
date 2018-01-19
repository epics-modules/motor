/*
FILENAME... drvMM4000.cc
USAGE...    Motor record driver level support for Newport MM4000.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/97
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
 *        The Controls and Automation Group (AT-8)
 *        Ground Test Accelerator
 *        Accelerator Technology Division
 *        Los Alamos National Laboratory
 *
 *      Co-developed with
 *        The Controls and Computing Group
 *        Accelerator Systems Division
 *        Advanced Photon Source
 *        Argonne National Laboratory
 *
 *
 * NOTES
 * -----
 * Verified with firmware:
 *
 *  - MM4000 2.04
 *  - MM4005 2.40
 *  - MM4005 2.42
 *  - MM4005 3.14b
 *  - MM4006 7.01c date 04-02-2004
 *
 *
 * Modification Log:
 * -----------------
 * .01 10-20-97 mlr initialized from drvOms58
 * .02 10-30-97 mlr Replaced driver calls with gpipIO functions
 * .03 10-30-98 mlr Minor code cleanup, improved formatting
 * .04 02-01-99 mlr Added temporary fix to delay reading motor positions at
 *                  the end of a move.
 * .05 10-13-99 rls modified for standardized motor record.
 * .06 09-17-01 rls
 *      - created a bit-field for motor status response.
 *      - start_status() allows one retry after a communication error.
 *      - set_status() sets RA_PROBLEM along with CNTRL_COMM_ERR to terminate node.
 * .07 05-19-03 rls Converted to R3.14.x.
 * .08 11-04-03 mlr Added a final poll of motor status if drvMM4000ReadbackDelay is
 *                  non-zero.  This is required to work around a bug in the firmware
 *                  (versions 2.40 and 2.44) that the motor is reported as done moving
 *                  on the first poll after a move is begun with the motor power off.
 * .09 02/03/04 rls Eliminate erroneous "Motor motion timeout ERROR".
 * .10 07/09/04 rls removed unused <driver>Setup() argument.
 * .11 07/28/04 rls "epicsExport" debug variable.
 * .12 09/21/04 rls support for 32axes/controller.
 * .13 12/21/04 rls
 *      - support for MM4006.
 *      - MS Visual C compatibility; make all epicsExportAddress extern "C" linkage.
 *      - make debug variables always available.
 * .14 02/18/09 rls Check for controller error.
 * .15 02/17/10 rls Bug fix controller error check overwriting position buffer.
 * .16 08/25/11 kmp Bug fix for drvMM4000Readback delay. The first call after the
 *                  readback delay now gets and sets the status properly. Also, the delay
 *                  was changed from a double (seconds) to an int (milliseconds) because 
 *                  of problems passing floating-point values from the VxWorks shell.
 * .17 08/25/11 rls - Added feedback position.
 *                  - Increased max. # of cards to 10 and buffer size to 160 bytes.
 */


#include <string.h>
#include <math.h>
#include <stdio.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <drvSup.h>
#include "motor.h"
#include "NewportRegister.h"
#include "drvMMCom.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define READ_RESOLUTION "TU;"
#define READ_STATUS     "MS;"
#define READ_POSITION   "TH;"
#define READ_FEEDBACK   "TP;"
#define STOP_ALL        "ST;"
#define MOTOR_ON        "MO;"
#define GET_IDENT       "VE;"

/* Status byte bits */
#define M_AXIS_MOVING     0x01
#define M_MOTOR_POWER     0x02
#define M_MOTOR_DIRECTION 0x04
#define M_PLUS_LIMIT      0x08
#define M_MINUS_LIMIT     0x10
#define M_HOME_SIGNAL     0x20

#define MM4000_NUM_CARDS 10
#define BUFF_SIZE 160       /* Maximum length of string to/from MM4000 */

#define TIMEOUT 2.0 /* Command timeout in sec. */

/*----------------debugging-----------------*/
volatile int drvMM4000debug = 0;
extern "C" {epicsExportAddress(int, drvMM4000debug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvMM4000debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int MM4000_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include    "motordrvComCode.h"


/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes. drvMM4000ReadbackDelay is in milliseconds
 */
volatile int drvMM4000ReadbackDelay = 0;


/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *name);
static void start_status(int card);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MM4000_access =
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

struct drvMM4000_drvet
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvMM4000 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvMM4000);}

static struct thread_args targs = {SCAN_RATE, &MM4000_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (MM4000_num_cards <=0)
        printf("    No MM4000 controllers configured.\n");
    else
    {
        for (card = 0; card < MM4000_num_cards; card++)
        {
            struct controller *brdptr = motor_state[card];

            if (brdptr == NULL)
                printf("    MM4000 controller %d connection failed.\n", card);
            else
            {
                struct MMcontroller *cntrl;

                cntrl = (struct MMcontroller *) brdptr->DevicePrivate;
                printf("    MM4000 controller %d, port=%s, address=%d, id: %s \n",
                       card, cntrl->asyn_port, cntrl->asyn_address,
                       brdptr->ident);
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
    if (MM4000_num_cards <= 0)
    {
        Debug(1, "init(): MM4000 driver disabled. MM4000Setup() missing from startup script.\n");
    }
    return((long) 0);
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
    struct MMcontroller *cntrl;
    int itera, status;

    if (card >= 0)
    {
        cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;
        send_mess(card, READ_STATUS, (char*) NULL);
        status = recv_mess(card, cntrl->status_string, 1);
        if (status > 0)
        {
            cntrl->status = NORMAL;
            send_mess(card, READ_POSITION, (char*) NULL);
            recv_mess(card, cntrl->position_string, 1);
            send_mess(card, READ_FEEDBACK, (char*) NULL);
            recv_mess(card, cntrl->feedback_string, 1);
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
            send_mess(itera, READ_STATUS, (char*) NULL);
        for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
        {
            cntrl = (struct MMcontroller *) motor_state[itera]->DevicePrivate;
            status = recv_mess(itera, cntrl->status_string, 1);
            if (status > 0)
            {
                cntrl->status = NORMAL;
                send_mess(itera, READ_FEEDBACK, (char*) NULL);
                recv_mess(itera, cntrl->feedback_string, 1);
            }
            else
            {
                if (cntrl->status == NORMAL)
                    cntrl->status = RETRY;
                else
                    cntrl->status = COMM_ERR;
            }
        }
        for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
            send_mess(itera, READ_POSITION, (char*) NULL);
        for (itera = 0; (itera < total_cards) && motor_state[itera]; itera++)
        {
            cntrl = (struct MMcontroller *) motor_state[itera]->DevicePrivate;
            recv_mess(itera, cntrl->position_string, 1);
        }
    }
}


/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
    struct MMcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char *p, *tok_save;
    char buff[BUFF_SIZE];
    int itera, pos;
    MOTOR_STATUS mstat;
    int rtn_state;
    double motorData;
    bool plusdir, ls_active = false;
    msta_field status;

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    status.All = motor_info->status.All;

    if (cntrl->status != NORMAL)
    {
        if (cntrl->status == COMM_ERR)
        {
            status.Bits.CNTRL_COMM_ERR = 1;
            status.Bits.RA_PROBLEM     = 1;
            rtn_state = 1;
            goto exit;
        }
        else
        {
            rtn_state = 0;
            goto exit;
        }
    }
    else
        status.Bits.CNTRL_COMM_ERR = 0;

    nodeptr = motor_info->motor_motion;

    /*
     * Parse the status string
     * Status string format: 1MSx,2MSy,3MSz,... where x, y and z are the status
     * bytes for the motors
     */
    pos = signal*5 + 3;  /* Offset in status string */
    mstat.All = cntrl->status_string[pos];
    Debug(5, "set_status(): status byte = %x on card #%d\n", mstat.All, card);

    status.Bits.RA_DIRECTION = (mstat.Bits.direction == false) ? 0 : 1;

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    if (mstat.Bits.inmotion == false)
    {
        status.Bits.RA_DONE = 1;
/* TEMPORARY FIX, Mark Rivers, 2/1/99. The MM4000 has reported that the
 * motor is done moving, which means that the "jerk time" is done.  However,
 * the axis can still be settling.  For now we put in a delay and poll the
 * motor position again. This is not a good long-term solution.
 *
 * Another TEMPORARY FIX, Mark Rivers, 4/13/03.  The MM4000 has reported that
 * the motor is done moving.  However, if the motor power was off, and the firmware
 * version is 2.43 or 2.44 then the controller "lies" and says the motor is done
 * on the first poll, when it is really moving.  We work around this by reading
 * the status again after the delay, in case it is really still moving.
 */
        if (motor_info->pid_present == YES && drvMM4000ReadbackDelay != 0)
        {
            epicsThreadSleep((double) drvMM4000ReadbackDelay/1000.0);
            send_mess(card, READ_STATUS, (char*) NULL);
            recv_mess(card, cntrl->status_string, 1);
            pos = signal*5 + 3;  /* Offset in status string */
            mstat.All = cntrl->status_string[pos];
            if (mstat.Bits.inmotion == true)
                status.Bits.RA_DONE = 0;
            send_mess(card, READ_POSITION, 0);
            recv_mess(card, cntrl->position_string, 1);
        }
    }
    else
        status.Bits.RA_DONE = 0;

    /* Set Travel limit switch status bits. */
    if (mstat.Bits.plustTL == false)
        status.Bits.RA_PLUS_LS = 0;
    else
    {
        status.Bits.RA_PLUS_LS = 1;
        if (plusdir == true)
            ls_active = true;
    }

    if (mstat.Bits.minusTL == false)
        status.Bits.RA_MINUS_LS = 0;
    else
    {
        status.Bits.RA_MINUS_LS = 1;
        if (plusdir == false)
            ls_active = true;
    }

    status.Bits.RA_HOME =     (mstat.Bits.homels    == false) ? 0 : 1;
    status.Bits.EA_POSITION = (mstat.Bits.NOT_power == false) ? 1 : 0;

    /* encoder status */
    status.Bits.EA_SLIP     = 0;
    status.Bits.EA_SLIP_STALL   = 0;
    status.Bits.EA_HOME     = 0;

    /*
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    strcpy(buff, cntrl->position_string);
    tok_save = NULL;
    p = epicsStrtok_r(buff, ",", &tok_save);
    for (itera = 0; itera < signal; itera++)
        p = epicsStrtok_r(NULL, ",", &tok_save);
    Debug(6, "set_status(): position substring = %s on card #%d\n", p, card);
    motorData = atof(p+3) / cntrl->drive_resolution[signal];

    if (motorData == motor_info->position)
    {
        if (nodeptr != 0)   /* Increment counter only if motor is moving. */
            motor_info->no_motion_count++;
    }
    else
    {
        motor_info->position = NINT(motorData);
        motor_info->no_motion_count = 0;
    }

    if (motor_state[card]->motor_info[signal].encoder_present == YES)
    {
        strcpy(buff, cntrl->feedback_string);
        tok_save = NULL;
        p = epicsStrtok_r(buff, ",", &tok_save);
        for (itera = 0; itera < signal; itera++)
            p = epicsStrtok_r(NULL, ",", &tok_save);
        Debug(6, "set_status(): feedback substring = %s on card #%d\n", p, card);
        motorData = atof(p+3) / cntrl->drive_resolution[signal];
        motor_info->encoder_position = (epicsInt32) motorData;
    }
    else
        motor_info->encoder_position = 0;


    /* Check for controller error. */
    send_mess(card, "TE;", (char*) NULL);
    recv_mess(card, buff, 1);
    if (buff[2] == '@')
        status.Bits.RA_PROBLEM = 0;
    else
    {
        status.Bits.RA_PROBLEM = 1;
        rtn_state = 1;
        goto exit;
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
        send_mess(card, buff, (char*) NULL);
        nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the MM4000 board            */
/* send_mess()                               */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    struct MMcontroller *cntrl;
    size_t size;
    size_t nwrite;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
        errlogMessage("drvMM4000.c:send_mess(); message size violation.\n");
        return(ERROR);
    }
    else if (size == 0) /* Normal exit on empty input message. */
        return(OK);

    if (!motor_state[card])
    {
        errlogPrintf("drvMM4000.c:send_mess() - invalid card #%d\n", card);
        return(ERROR);
    }

    if (name != NULL)
    {
        errlogPrintf("drvMM4000.c:send_mess() - invalid argument = %s\n", name);
        return(ERROR);
    }

    Debug(2, "send_mess(): message = %s on card #%d\n", com, card);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    pasynOctetSyncIO->write(cntrl->pasynUser, com, strlen(com), TIMEOUT, &nwrite);

    return(OK);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int flag)
 *
 * INPUT ARGUMENTS...
 *  card - controller card # (0,1,...).
 *  *com - caller's response buffer.
 *  flag    | FLUSH  = flush controller's output buffer; set timeout = 0.
 *      | !FLUSH = retrieve response into caller's buffer; set timeout.
 *
 * LOGIC...
 *  IF controller card does not exist.
 *  ERROR RETURN.
 *  ENDIF
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int flag)
{
    struct MMcontroller *cntrl;
    double timeout = 0.;
    size_t nread = 0;
    asynStatus status;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
        return(ERROR);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    timeout = TIMEOUT;
    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
        com[0] = '\0';
        nread = 0;
    }

    Debug(2, "recv_mess(): message = \"%s\" on card #%d\n", com, card);
    return((int)nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* MM4000Setup()                                     */
/*****************************************************/
RTN_STATUS
MM4000Setup(int num_cards,  /* maximum number of controllers in system.  */
            int scan_rate)  /* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > MM4000_NUM_CARDS)
    {
        epicsThreadSleep(5.0);
        errlogPrintf("\n*** ERROR *** Number specified (%d) exceeds maximum allowed (%d).\n\n", num_cards, MM4000_NUM_CARDS);
        epicsThreadSleep(5.0);
        MM4000_num_cards = MM4000_NUM_CARDS;
    }
    else
        MM4000_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
        targs.motor_scan_rate = scan_rate;
    else
        targs.motor_scan_rate = SCAN_RATE;

    /*
     * Allocate space for motor_state structures.  Note this must be done
     * before MM4000Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(MM4000_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < MM4000_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* MM4000Config()                                    */
/*****************************************************/
RTN_STATUS
MM4000Config(int card,      /* card being configured */
             const char *name,   /* asyn port name */
             int addr)           /* asyn address (GPIB) */
{
    struct MMcontroller *cntrl;

    if (card < 0 || card >= MM4000_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MMcontroller));
    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
    cntrl->asyn_address = addr;
    return(OK);
}



/*****************************************************/
/* initialize all software and hardware          */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()                              */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct MMcontroller *cntrl;
    int card_index, motor_index;
    char axis_pos[BUFF_SIZE];
    char buff[BUFF_SIZE];
    char *tok_save, *pos_ptr;
    int total_axis = 0;
    int status=0, model_num, digits;
    asynStatus success_rtn;

    initialized = true; /* Indicate that driver is initialized. */

    /* Check for setup */
    if (MM4000_num_cards <= 0)
        return(ERROR);

    for (card_index = 0; card_index < MM4000_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        brdptr->cmnd_response = false;
        total_cards = card_index + 1;
        cntrl = (struct MMcontroller *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port,
                                                cntrl->asyn_address, &cntrl->pasynUser, NULL);

        if (success_rtn == asynSuccess)
        {
            int retry = 0;

            /* Send a message to the board, see if it exists */
            /* flush any junk at input port - should not be any data available */
            pasynOctetSyncIO->flush(cntrl->pasynUser);

            do
            {
                send_mess(card_index, READ_POSITION, (char*) NULL);
                status = recv_mess(card_index, axis_pos, 1);
                retry++;
                /* Return value is length of response string */
            } while (status == 0 && retry < 3);
        }

        if (success_rtn == asynSuccess && status > 0)
        {
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;
            send_mess(card_index, STOP_ALL, (char*) NULL);   /* Stop all motors */
            send_mess(card_index, GET_IDENT, (char*) NULL);  /* Read controller ID string */
            recv_mess(card_index, buff, 1);
            strcpy(brdptr->ident, &buff[2]);  /* Skip "VE" */

            /* Set Motion Master model indicator. */
            pos_ptr = strstr(brdptr->ident, "MM");
            if (pos_ptr == NULL)
            {
                errlogPrintf("drvMM4000.c:motor_init() - invalid model = %s\n", brdptr->ident);
                motor_state[card_index] = (struct controller *) NULL;
                continue;
            }
            model_num = atoi(pos_ptr + 2);
            if (model_num == 4000)
                cntrl->model = MM4000;
            else if (model_num == 4005 || model_num == 4006)
                cntrl->model = MM4005;
            else
            {
                errlogPrintf("drvMM4000.c:motor_init() - invalid model = %s\n", brdptr->ident);
                motor_state[card_index] = (struct controller *) NULL;
                continue;
            }

            send_mess(card_index, READ_POSITION, (char*) NULL);
            recv_mess(card_index, axis_pos, 1);

            /* The return string will tell us how many axes this controller has */
            for (total_axis = 0, tok_save = NULL, pos_ptr = epicsStrtok_r(axis_pos, ",", &tok_save);
                pos_ptr != 0; pos_ptr = epicsStrtok_r(NULL, ",", &tok_save), total_axis++)
                brdptr->motor_info[total_axis].motor_motion = NULL;

            brdptr->total_axis = total_axis;

            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                int loop_state;

                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;

                motor_info->status.Bits.GAIN_SUPPORT = 1;

                /* Determine if encoder present based on open/closed loop mode. */
                sprintf(buff, "%dTC", motor_index + 1);
                send_mess(card_index, buff, (char*) NULL);
                recv_mess(card_index, buff, 1);
                loop_state = atoi(&buff[3]);    /* Skip first 3 characters */
                if (loop_state != 0)
                {
                    motor_info->encoder_present = YES;
                    motor_info->status.Bits.EA_PRESENT = 1;
                    motor_info->pid_present = YES;
                }

                /* Determine drive resolution. */
                sprintf(buff, "%dTU", motor_index + 1);
                send_mess(card_index, buff, (char*) NULL);
                recv_mess(card_index, buff, 1);
                cntrl->drive_resolution[motor_index] = atof(&buff[3]);

                digits = (int) -log10(cntrl->drive_resolution[motor_index]) + 2;
                if (digits < 1)
                    digits = 1;
                cntrl->res_decpts[motor_index] = digits;

                /* Save home preset position. */
                sprintf(buff, "%dXH", motor_index + 1);
                send_mess(card_index, buff, (char*) NULL);
                recv_mess(card_index, buff, 1);
                cntrl->home_preset[motor_index] = atof(&buff[3]);

                /* Determine low limit */
                sprintf(buff, "%dTL", motor_index + 1);
                send_mess(card_index, buff, (char*) NULL);
                recv_mess(card_index, buff, 1);
                motor_info->low_limit = atof(&buff[3]);

                /* Determine high limit */
                sprintf(buff, "%dTR", motor_index + 1);
                send_mess(card_index, buff, (char*) NULL);
                recv_mess(card_index, buff, 1);
                motor_info->high_limit = atof(&buff[3]);

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

    epicsThreadCreate((char *) "MM4000_motor",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

