/*
FILENAME... drvSmartMotor.cc
USAGE...    Motor record driver level support for Animatics Corporation SmartMotors.

*/

/*
 *      Original Author: Shifu Xu 
 *      Date: 03/21/03
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
 * NOTES
 * -----
 * - This device driver only supports SmartMotor firmware versions 4.15
 *    and above.
 *
 * Modification Log:
 * -----------------
 * .01 02/20/07 ses copied from drvSmartMotorPL.c
 * .02 09/06/07 Eric Norum discovered this device needs an update delay.
 */

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <iocsh.h>
#include <stdlib.h>
#include <errlog.h>
#include "motor.h"
#include "motorRecord.h"
#include "drvSmartMotor.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"
#include "epicsTime.h"


#define SmartMotor_NUM_CARDS    7
#define BUFF_SIZE 20  /* Maximum length of string to/from SmartMotor */

/*----------------debugging-----------------*/
volatile int drvSmartMotordebug = 0;
extern "C" {epicsExportAddress(int, drvSmartMotordebug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvSmartMotordebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int SmartMotor_num_cards = 0;

static char *SmartMotor_addr[] = {"129", "130", "131", "132", "133", "134",
    "135", "136", "137", "138", "139", "140",
    "141", "142", "143", "144", "145", "146",
    "147", "148", "149", "150", "151", "152",
    "153", "154", "155", "156", "157", "158",
    "159", "160"};

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

struct driver_table SmartMotor_access =
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
    SmartMotor_addr
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvSmartMotor =
{
    2, report, init
};

extern "C"
{
    epicsExportAddress(drvet, drvSmartMotor);
}

static struct thread_args targs = {SCAN_RATE, &SmartMotor_access, 0.1};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (SmartMotor_num_cards <= 0)
        printf("    No SmartMotor controllers configured.\n");
    else
    {
        for (card = 0; card < SmartMotor_num_cards; card++)
        {
            struct controller *brdptr = motor_state[card];

            if (brdptr == NULL)
                printf("    SmartMotor controller %d connection failed.\n", card);
            else
            {
                struct SmartMotorcontroller *cntrl;

                cntrl = (struct SmartMotorcontroller *) brdptr->DevicePrivate;
                printf("    SmartMotorSM controller #%d, port=%s, id: %s \n", card,
                       cntrl->asyn_port, brdptr->ident);
            }
        }
    }
    return(OK);
}


static long init()
{
    /*
     * We cannot call motor_init() here, because that function can do GPIB I/O,
     * and hence requires that the drvGPIB have already been initialized. That
     * cannot be guaranteed, so we need to call motor_init from device support
     */
    /* Check for setup */
    if (SmartMotor_num_cards <= 0)
    {
        Debug(1, "init(): SmartMotor driver disabled. SmartMotorSetup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node * nodeptr)
{
}


/*****************************************************************************
*
* FUNCTION NAME: set_status
*
* LOGIC:
*   Initialize.
*   Send "Moving Status" query.
*   Read response.
*   IF normal response to query.
*       Set communication status to NORMAL.
*   ELSE
*       IF communication status is NORMAL.
*           Set communication status to RETRY.
*           NORMAL EXIT.
*       ELSE
*           Set communication status error.
*           ERROR EXIT.
*       ENDIF
*   ENDIF
*
*   IF "Moving Status" indicates any motion (i.e. status != 0).
*       Clear "Done Moving" status bit.
*   ELSE
*       Set "Done Moving" status bit.
*   ENDIF
*
*****************************************************************************/

static int set_status(int card, int signal)
{
    struct SmartMotorcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    int rtnval, rtn_state, r_RBt;
    double motorData;
    int Lswitch;
    bool plusdir, ls_active = false;
    msta_field status;

    cntrl = (struct SmartMotorcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    buff[0] = '\0';
    send_mess(card, "RBt", SmartMotor_addr[signal]);
    rtn_state = recv_mess(card, buff, 1);
    r_RBt = atoi(buff);

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
            status.Bits.RA_PROBLEM = 1;
            rtn_state = 1;
            goto exit;
        }
    }

    /* decide if move is done */
    status.Bits.RA_DONE = (r_RBt == 0) ? 1 : 0;
    send_mess(card, "RP", SmartMotor_addr[signal]);
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

    if (nodeptr != 0) /* If moving, set direction based on commanded positon. */
    {
        struct motorRecord *mr = (struct motorRecord *) nodeptr->mrecord;
        status.Bits.RA_DIRECTION = mr->cdir;
    }

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    /* Set limit switch indicators */
    send_mess(card, "RBp", SmartMotor_addr[signal]);
    recv_mess(card, buff, 1);
    Lswitch = atoi(buff);
    if (Lswitch != 0)
    {
        status.Bits.RA_PLUS_LS = 1;
        if (plusdir == true)
            ls_active = true;
    }
    else
        status.Bits.RA_PLUS_LS = 0;

    send_mess(card, "RBm", SmartMotor_addr[signal]);
    recv_mess(card, buff, 1);
    Lswitch = atoi(buff);

    if (Lswitch != 0)
    {
        status.Bits.RA_MINUS_LS = 1;
        if (plusdir == false)
            ls_active = true;
    }
    else
        status.Bits.RA_MINUS_LS = 0;

    send_mess(card, "RBr", SmartMotor_addr[signal]);
    recv_mess(card, buff, 1);
    Lswitch = atoi(buff);
    if (Lswitch)
        send_mess(card, "Zr", SmartMotor_addr[signal]);
    send_mess(card, "RBl", SmartMotor_addr[signal]);
    recv_mess(card, buff, 1);
    Lswitch = atoi(buff);
    if (Lswitch)
        send_mess(card, "Zl", SmartMotor_addr[signal]);

    send_mess(card, "RBo", SmartMotor_addr[signal]);
    rtn_state = recv_mess(card, buff, 1);
    rtnval = atoi(buff);
    status.Bits.EA_POSITION = (rtnval != 0) ? 0 : 1;

    /* encoder status */
    status.Bits.EA_SLIP = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME = 0;

    if (motor_state[card]->motor_info[signal].encoder_present == NO)
        motor_info->encoder_position = 0;
    else
    {
        send_mess(card, "RP", SmartMotor_addr[signal]);
        recv_mess(card, buff, 1);
        motorData = atof(buff);
        motor_info->encoder_position = (epicsInt32) motorData;
    }

    status.Bits.RA_PROBLEM = 0;

    /* Parse motor velocity? */
    send_mess(card, "RV", SmartMotor_addr[signal]);
    recv_mess(card, buff, 1);
    rtnval = atoi(buff);
    motor_info->velocity = rtnval;

    if (!status.Bits.RA_DIRECTION)
        motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
                 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
        nodeptr->postmsgptr != 0)
    {
        strcpy(buff, nodeptr->postmsgptr);
        send_mess(card, buff, SmartMotor_addr[signal]);
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

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}

/****************************************************/
/* send a message to the SmartMotor board           */
/* send_mess()                                      */
/****************************************************/

static RTN_STATUS send_mess(int card, char const * com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    char echo_buff[BUFF_SIZE];
    struct SmartMotorcontroller *cntrl;
    int comsize, namesize, addr;
    size_t nwrite;

    static const char input_terminator[] = "\r";
    static const char echo_terminator[] = "\n";

    local_buff[0] = '\0';
    echo_buff[0] = '\0';
    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
        errlogMessage("drvSmartMotor.c:send_mess(); message size violation.\n");
        return(ERROR);
    }
    else if (comsize == 0)  /* Normal exit on empty input message. */
        return(OK);

    if (!motor_state[card])
    {
        errlogPrintf("drvSmartMotor.c:send_mess() - invalid card #%d\n", card);
        return(ERROR);
    }

    cntrl = (struct SmartMotorcontroller *) motor_state[card]->DevicePrivate;
    if (cntrl->num_motors != 1) /* Test for daisy chain; echo mode. */
    {
        /* convert addr to binary character */
        sprintf(local_buff, "%c", atoi(name));
        strcat(local_buff, com);
        addr = atoi(name) - 128;
    }
    else
        strcpy(local_buff, com);

    Debug(2, "send_mess(): message = %s for card# %d, addr# %d\n", local_buff, card, addr);

    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
                            COMM_TIMEOUT, &nwrite);
    if (cntrl->num_motors == 1)
    {
        /* do not strip off echoed command */
    }
    else
    {   /* strip off echoed command to get device response */
        pasynOctetSyncIO->setInputEos(cntrl->pasynUser, echo_terminator,
                                      strlen(echo_terminator));
        recv_mess(card, echo_buff, 1);
    }
    pasynOctetSyncIO->setInputEos(cntrl->pasynUser, input_terminator,
                                  strlen(input_terminator));
    return(OK);
}

/****************************************************/
/* receive a message from the SmartMotor board      */
/* recv_mess()                                      */
/****************************************************/

static int recv_mess(int card, char *com, int flag)
{
    struct SmartMotorcontroller *cntrl;
    const double timeout = 0.25;/* !!! orginal 1.0 */
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;
    char recv_buf[20], *recv_string = recv_buf;

    /* Check that card exists */
    if (!motor_state[card])
        return(ERROR);

    cntrl = (struct SmartMotorcontroller *) motor_state[card]->DevicePrivate;
    recv_string[0] = '\0';
    com[0] = '\0';

    if (flag == FLUSH)
        pasynOctetSyncIO->flush(cntrl->pasynUser);
    else
        status = pasynOctetSyncIO->read(cntrl->pasynUser, recv_string, BUFF_SIZE,
                                        timeout, &nread, &eomReason);
    if ((status != asynSuccess) || (nread <= 0))
    {
        com[0] = '\0';
        nread = 0;
    }
    else
    {
        strcpy(com, recv_string);
    }
    Debug(2, "recv_mess(): message = \"%s\"\n", com);

    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* SmartMotorSetup()                                 */
/*****************************************************/
RTN_STATUS SmartMotorSetup(int num_cards, int scan_rate)    /* maximum number of
                                 * chains in system.  */
/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > SmartMotor_NUM_CARDS)
        SmartMotor_num_cards = SmartMotor_NUM_CARDS;
    else
        SmartMotor_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
        targs.motor_scan_rate = scan_rate;
    else
        targs.motor_scan_rate = SCAN_RATE;

    /*
     * Allocate space for motor_state structures.  Note this must be done
     * before SmartMotorConfig is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing if it
     * really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(SmartMotor_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < SmartMotor_num_cards; itera++)
        motor_state[itera] = NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* SmartMotorConfig()                                */
/*****************************************************/
RTN_STATUS SmartMotorConfig(int card, const char *name) /* (chain #, ASYN port) */
{
    struct SmartMotorcontroller *cntrl;

    if (card < 0 || card >= SmartMotor_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct SmartMotorcontroller));
    cntrl = (struct SmartMotorcontroller *) motor_state[card]->DevicePrivate;

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
    struct SmartMotorcontroller *cntrl;
    int card_index, motor_index, total_motors;
    char buff[BUFF_SIZE];
    int status;
    asynStatus success_rtn;
    static const char output_terminator[] = "\n";
    static const char input_terminator[] = "\r";

    initialized = true;     /* Indicate that driver is initialized. */

    /* Check for setup */
    if (SmartMotor_num_cards <= 0)
        return(ERROR);

    for (card_index = 0; card_index < SmartMotor_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        brdptr->ident[0] = 0; /* No controller identification message. */
        brdptr->total_axis = 0;         /* Default to zero motors. */
        brdptr->cmnd_response = false;
        total_cards = card_index + 1;
        cntrl = (struct SmartMotorcontroller *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
                                                &cntrl->pasynUser, NULL);

        if (success_rtn == asynSuccess)
        {
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser, output_terminator,
                                           strlen(output_terminator));
            pasynOctetSyncIO->setInputEos(cntrl->pasynUser, input_terminator,
                                          strlen(input_terminator));
            /* Send a message to the motor, see if it exists */
            /* flush any junk at input port - should not be any data available */
            pasynOctetSyncIO->flush(cntrl->pasynUser);

            cntrl->num_motors = 1; /* Default 1 motor (No Echo) for send_mess. */

            for (total_motors = 0; total_motors < MAX_AXIS; total_motors++)
            {
                /* Try 3 times to connect to motor. */
                int retry = 0;
                do
                {
                    send_mess(card_index, "RBe", SmartMotor_addr[total_motors]);
                    status = recv_mess(card_index, buff, 1);
                    /* 1st iteration + No Echo mode + non-null response + cmnd echoed. */
                    if (total_motors == 0 && cntrl->num_motors == 1 && status > 0 &&
                        strncmp(buff, "RBe", 3) == 0)
                    {
                        cntrl->num_motors = 2;  /* Echo ON. */
                        status = retry = 0;     /* start over. */
                        pasynOctetSyncIO->flush(cntrl->pasynUser);
                    }
                    if (buff[0] != '0' && buff[0] != '1')
                        status = 0;
                    retry++;
                } while (status == 0 && retry < 3);

                if (status <= 0)
                    break;
                if (cntrl->num_motors == 1 && status > 0)
                {
                    total_motors = 1;
                    break;
                }
            }
        }

        if (success_rtn == asynSuccess && total_motors > 0)
        {
            cntrl->num_motors = brdptr->total_axis = total_motors;
            brdptr->localaddr = NULL;
            brdptr->motor_in_motion = 0;

            for (motor_index = 0; motor_index < brdptr->total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
                brdptr->motor_info[motor_index].motor_motion = NULL;
                /* Assume no encoder support. */
                motor_info->encoder_present = NO;
                motor_info->pid_present = YES;
                motor_info->status.Bits.GAIN_SUPPORT = 1;
                motor_info->encoder_present = YES;
                motor_info->status.Bits.EA_PRESENT = 1;
                /* Read status of each motor */
                set_status(card_index, motor_index);
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

    epicsThreadCreate((char *) "SmartMotor_motor", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

extern "C"
{

// SmartMotor Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};

// SmartMotor Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};

static const iocshArg * const SmartMotorSetupArgs[2]  = {&setupArg0,  &setupArg1};
static const iocshArg * const SmartMotorConfigArgs[2] = {&configArg0, &configArg1};

static const iocshFuncDef setupSmartMotor   = {"SmartMotorSetup",  2, SmartMotorSetupArgs};
static const iocshFuncDef configSmartMotor  = {"SmartMotorConfig", 2, SmartMotorConfigArgs};

static void setupSmartMotorCallFunc(const iocshArgBuf *args)
{
    SmartMotorSetup(args[0].ival, args[1].ival);
}

static void configSmartMotorCallFunc(const iocshArgBuf *args)
{
    SmartMotorConfig(args[0].ival, args[1].sval);
}

static void SmartMotormotorRegister(void)
{
    iocshRegister(&setupSmartMotor,  setupSmartMotorCallFunc);
    iocshRegister(&configSmartMotor,  configSmartMotorCallFunc);
}

epicsExportRegistrar(SmartMotormotorRegister);

} // extern "C"

