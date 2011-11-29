/*
FILENAME... drvMCDC2805.cc
USAGE...    Motor record driver level support for Faulhaber MCDC2805

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
Last Modified:  $Date$
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/2005
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
 * Modification Log:
 * -----------------
 * .01 10/20/05 mlr Initialize from ImsDrc/MDrive.cc
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the MCDC2805 must be powered-on when EPICS is first
    booted up.
*/

#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <iocsh.h>
#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "drvMCDC2805.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define MCDC2805_NUM_CARDS    8
#define BUFF_SIZE 100        /* Maximum length of string to/from MCDC2805 */

/*----------------debugging-----------------*/
volatile int drvMCDC2805debug = 0;
extern "C" {epicsExportAddress(int, drvMCDC2805debug);}

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvMCDC2805debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int MCDC2805_num_cards = 0;
static char *MCDC2805_axis[] = {"0", "1", "2", "3", "4", "5", "6", "7"};

/* Local data required for every driver; see "motordrvComCode.h" */
#include    "motordrvComCode.h"

/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MCDC2805_access =
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
    MCDC2805_axis
};

struct drvMCDC2805_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvMCDC2805 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvMCDC2805);}

static struct thread_args targs = {SCAN_RATE, &MCDC2805_access, 0.0};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (MCDC2805_num_cards <=0)
    printf("    No MCDC2805 controllers configured.\n");
    else
    {
    for (card = 0; card < MCDC2805_num_cards; card++)
    {
        struct controller *brdptr = motor_state[card];

        if (brdptr == NULL)
        printf("    MCDC2805 controller %d connection failed.\n", card);
        else
        {
        struct MCDC2805controller *cntrl;

        cntrl = (struct MCDC2805controller *) brdptr->DevicePrivate;
            printf("    MCDC2805SM controller #%d, port=%s, id: %s \n", card,
               cntrl->asyn_port, brdptr->ident);
        }
    }
    }
    return(OK);
}


static long init()
{
   /*
    * We cannot call motor_init() here, because that function can do serial I/O,
    * and hence requires that the serial port have already been initialized.
    * That cannot be guaranteed, so we need to call motor_init from device
    * support
    */
    /* Check for setup */
    if (MCDC2805_num_cards <= 0)
    {
    Debug(1, "init(): MCDC2805 driver disabled. MCDC2805Setup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/********************************************************************************
*                                       *
* FUNCTION NAME: set_status                         *
*                                       *
* LOGIC:                                    *
*   Initialize.                                 *
*   Send "Moving Status" query.                         *
*   Read response.                              *
*   IF normal response to query.                        *
*   Set communication status to NORMAL.                 *
*   ELSE                                    *
*   IF communication status is NORMAL.                  *
*       Set communication status to RETRY.                  *
*       NORMAL EXIT.                            *
*   ELSE                                    *
*       Set communication status error.                 *
*       ERROR EXIT.                             *
*   ENDIF                                   *
*   ENDIF                                   *
*                                       *
*   IF "Moving Status" indicates any motion (i.e. status != 0).         *
*   Clear "Done Moving" status bit.                     *
*   ELSE                                    *
*   Set "Done Moving" status bit.                       *
*   ENDIF                                   *
*                                       *
*                                           *
********************************************************************************/

static int set_status(int card, int signal)
{
    struct MCDC2805controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    char status_buff[BUFF_SIZE];
    int rtnval, rtn_state;
    double motorData;
    epicsUInt8 Lswitch;
    bool plusdir, ls_active = false;
    msta_field status;

    cntrl = (struct MCDC2805controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    send_mess(card, "GST", MCDC2805_axis[signal]);
    rtn_state = recv_mess(card, status_buff, 1);
    if (rtn_state > 0)
    {
    cntrl->status = NORMAL;
    status.Bits.CNTRL_COMM_ERR = 0;
    }
    else
    {
       if (cntrl->status == NORMAL)
       {
           cntrl->status = RETRY;
           rtn_state = OK;
           goto exit;
       }
       else
       {
           cntrl->status = COMM_ERR;
           status.Bits.CNTRL_COMM_ERR = 1;
           status.Bits.RA_PROBLEM     = 1;
           rtn_state = 1;
           goto exit;
       }
    }

    rtnval = status_buff[4] == '1';

    /* We can't use this logic for done, because it won't work in jog mode or when
     * a move is stopped by setting velocity=0. */

    /* status.Bits.RA_DONE = (rtnval == 0) ? 0 : 1; */

    /*
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    send_mess(card, "POS", MCDC2805_axis[signal]);
    recv_mess(card, buff, 1);

    motorData = atof(buff);

    if (motorData == motor_info->position)
    {
        if (nodeptr != 0)   /* Increment counter only if motor is moving. */
            motor_info->no_motion_count++;
    }
    else
    {
       epicsInt32 newposition;

       newposition = NINT(motorData);
       status.Bits.RA_DIRECTION = (newposition >= motor_info->position) ? 1 : 0;
       motor_info->position = newposition;
       motor_info->no_motion_count = 0;
    }

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    send_mess(card, "GAST", MCDC2805_axis[signal]);
    recv_mess(card, buff, 1);
    Lswitch = buff[0] == '1';

    /* Set limit Lswitch error indicators. */
    if (Lswitch != 0)
    {
       status.Bits.RA_PLUS_LS = 1;
       if (plusdir == true)
       ls_active = true;
    }
    else
       status.Bits.RA_PLUS_LS = 0;

    Lswitch = buff[1] == '1';

    if (Lswitch != 0)
    {
       status.Bits.RA_MINUS_LS = 1;
       if (plusdir == false)
       ls_active = true;
    }
    else
       status.Bits.RA_MINUS_LS = 0;

    /* If there is an active limit switch (i.e. one that stopped motion) then
     * we need to do an absolute move to the current position.
     * If this is not done then as soon as the limit switch is released the motor will
     * start moving, which is not good.
     */
     if (ls_active) {
        sprintf(buff, "LA %ld", NINT(motorData));
        send_mess(card, buff, MCDC2805_axis[signal]);
        send_mess(card, "M", MCDC2805_axis[signal]);
     }

     Lswitch = status_buff[6] == '1';
     status.Bits.RA_HOME = (Lswitch) ? 1 : 0;

    /* !!! Assume no closed-looped control!!!*/
    status.Bits.EA_POSITION = 0;

    /* encoder status */
    status.Bits.EA_SLIP       = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME       = 0;

    if (motor_state[card]->motor_info[signal].encoder_present == NO)
    motor_info->encoder_position = 0;
    else
    {
       motor_info->encoder_position = (int) motorData;
    }

    status.Bits.RA_PROBLEM  = 0;

    /* Read the actual velocity.  Use this to set the DONE bit.
     * This is needed because the "reached command position bit in GST does
     * not work for jog commands or when move is stopped by setting velocity=0 */
    send_mess(card, "GV", MCDC2805_axis[signal]);
    recv_mess(card, buff, 1);
    motor_info->velocity = NINT(atof(buff));
    status.Bits.RA_DONE = (motor_info->velocity == 0) ? 1 : 0;

    if (!status.Bits.RA_DIRECTION)
    motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
        status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
    nodeptr->postmsgptr != 0)
    {
       strcpy(buff, nodeptr->postmsgptr);
       send_mess(card, buff, MCDC2805_axis[signal]);
       nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the MCDC2805 board            */
/* send_mess()                               */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    struct MCDC2805controller *cntrl;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
    errlogMessage("drvMCDC2805.c:send_mess(); message size violation.\n");
    return(ERROR);
    }
    else if (comsize == 0)  /* Normal exit on empty input message. */
    return(OK);

    if (!motor_state[card])
    {
    errlogPrintf("drvMCDC2805.c:send_mess() - invalid card #%d\n", card);
    return(ERROR);
    }

    /* Make a local copy of the string and add the command line terminator. */
    if (namesize != 0)
    {
    strcpy(local_buff, name);       /* put in axis */
    strcat(local_buff, com);
    }
    else
    strcpy(local_buff, com);

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct MCDC2805controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
               COMM_TIMEOUT, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the MCDC2805 board           */
/* recv_mess()                               */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct MCDC2805controller *cntrl;
    const double timeout = 1.0;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
    return(ERROR);

    cntrl = (struct MCDC2805controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
    pasynOctetSyncIO->flush(cntrl->pasynUser);
    else
    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                    timeout, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
    com[0] = '\0';
    nread = 0;
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* MCDC2805Setup()                                     */
/*****************************************************/
RTN_STATUS
MCDC2805Setup(int num_cards,  /* maximum number of chains in system.  */
        int scan_rate)  /* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > MCDC2805_NUM_CARDS)
    MCDC2805_num_cards = MCDC2805_NUM_CARDS;
    else
    MCDC2805_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
    targs.motor_scan_rate = scan_rate;
    else
    targs.motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structures.  Note this must be done
    * before MCDC2805Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(MCDC2805_num_cards *
                        sizeof(struct controller *));

    for (itera = 0; itera < MCDC2805_num_cards; itera++)
    motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* MCDC2805Config()                                    */
/*****************************************************/
RTN_STATUS
MCDC2805Config(int card,          /* chain being configured */
               int num_motors,    /* Number of MCDC-2805 units on this serial port */
               const char *name)  /* ASYN port name */
{
    struct MCDC2805controller *cntrl;

    if (card < 0 || card >= MCDC2805_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MCDC2805controller));
    cntrl = (struct MCDC2805controller *) motor_state[card]->DevicePrivate;

    cntrl->num_motors = num_motors;
    strcpy(cntrl->asyn_port, name);
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
    struct MCDC2805controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status;
    asynStatus success_rtn;

    initialized = true; /* Indicate that driver is initialized. */

    /* Check for setup */
    if (MCDC2805_num_cards <= 0)
    return(ERROR);

    for (card_index = 0; card_index < MCDC2805_num_cards; card_index++)
    {
    if (!motor_state[card_index])
        continue;

    brdptr = motor_state[card_index];
    brdptr->ident[0] = (char) NULL; /* No controller identification message. */
    brdptr->cmnd_response = false;
    total_cards = card_index + 1;
    cntrl = (struct MCDC2805controller *) brdptr->DevicePrivate;

    /* Initialize communications channel */
    success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
                &cntrl->pasynUser, NULL);

    if (success_rtn == asynSuccess)
    {
        /* Send a message to the board, see if it exists */
        /* flush any junk at input port - should not be any data available */
            pasynOctetSyncIO->flush(cntrl->pasynUser);

        for (total_axis = 0; total_axis < cntrl->num_motors; total_axis++)
        {
        int retry = 0;

        /* Try 3 times to connect to controller. */
        do
        {
            send_mess(card_index, "VER", MCDC2805_axis[total_axis]);
            status = recv_mess(card_index, buff, 1);
            retry++;
        } while (status == 0 && retry < 3);

        if (status <= 0)
            break;
        else if (total_axis == 0)
            strcpy(brdptr->ident, buff);
        }
        brdptr->total_axis = total_axis;
    }

    if (success_rtn == asynSuccess && total_axis > 0)
    {
        brdptr->localaddr = (char *) NULL;
        brdptr->motor_in_motion = 0;

        for (motor_index = 0; motor_index < total_axis; motor_index++)
        {
           struct mess_info *motor_info = &brdptr->motor_info[motor_index];

           motor_info->status.All = 0;
           motor_info->no_motion_count = 0;
           motor_info->encoder_position = 0;
           motor_info->position = 0;
           brdptr->motor_info[motor_index].motor_motion = NULL;

           motor_info->pid_present = YES;
           motor_info->status.Bits.GAIN_SUPPORT = 1;
           motor_info->encoder_present = YES;
           motor_info->status.Bits.EA_PRESENT = 1;

           /* Set the velocity control to be RS-232 */
           send_mess(card_index, "SOR 0", MCDC2805_axis[motor_index]);

           /* Set the limit switch configuration. */
           /* We assume the following:
            *   Analog input = Input 1 = Home input = 1 bit
            *   Fault pin =    Input 2 = CW limit   = 2 bit
            *   Input 3        Input 3 = CCW limit  = 4 bit
            */
           /* Program fault pin as limit switch input */
           send_mess(card_index, "REFIN", MCDC2805_axis[motor_index]);
           /* Program limit polarity for rising edge and high level */
           send_mess(card_index, "HP7", MCDC2805_axis[motor_index]);
           /* Program the motor to hard block on the the limit switch inputs */
           send_mess(card_index, "HB6", MCDC2805_axis[motor_index]);
           /* Program the limit switch directions to block 
            * + direction on input 2, - direction on input 3 */
           send_mess(card_index, "HD2", MCDC2805_axis[motor_index]);
           /* Program homing sequence on input 1*/
           send_mess(card_index, "HL1", MCDC2805_axis[motor_index]);
           send_mess(card_index, "HA1", MCDC2805_axis[motor_index]);
           send_mess(card_index, "CAHOSEQ", MCDC2805_axis[motor_index]);

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

    epicsThreadCreate((char *) "MCDC2805_motor", epicsThreadPriorityMedium,
    epicsThreadGetStackSize(epicsThreadStackMedium),
    (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

extern "C"
{

// Faulhaber Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Faulhaber Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"# modules on this serial port", iocshArgInt};
static const iocshArg configArg2 = {"asyn port name", iocshArgString};

static const iocshArg * const setupArgs[2] = {&setupArg0,
                                              &setupArg1};

static const iocshArg * const configArgs[3] = {&configArg0,
                                               &configArg1,
                                               &configArg2};

static const iocshFuncDef setupMCDC2805  = {"MCDC2805Setup",  2, setupArgs};

static const iocshFuncDef configMCDC2805  = {"MCDC2805Config",  3, configArgs};

static void setupMCDC2805CallFunc(const iocshArgBuf *args)
{
    MCDC2805Setup(args[0].ival, args[1].ival);
}

static void configMCDC2805CallFunc(const iocshArgBuf *args)
{
    MCDC2805Config(args[0].ival, args[1].ival, args[2].sval);
}


static void MCDC2805Register(void)
{
    iocshRegister(&setupMCDC2805,  setupMCDC2805CallFunc);
    iocshRegister(&configMCDC2805,  configMCDC2805CallFunc);
}

epicsExportRegistrar(MCDC2805Register);

} // extern "C"


