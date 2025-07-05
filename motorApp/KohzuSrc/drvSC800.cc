/*
FILENAME...     drvSC800.cc
USAGE...        Motor record driver level support for Kohzu SC800                


*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 11/09/2007
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
 * Modification Log:
 * -----------------
 * .01 11-09-07 rls copied from drvMDT695.cc
 * .02 09-22-09 rls Added support for SC200/400
 *
 */


#include <string.h>
#include <ctype.h>   /* isascii functions */
#include <math.h>
#include <stdio.h>
#include <epicsThread.h>
#include <drvSup.h>
#include <iocsh.h>
#include <errlog.h>
#include <stdlib.h>
#include "motor.h"
#include "motorRecord.h"
#include "drvSC800.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define STX         '\002'
#define GET_IDENT   "IDN"

#define SC800_NUM_CARDS 16
#define BUFF_SIZE 120       /* Maximum length of string to/from SC800 */

#define TIMEOUT 3.0     /* Command timeout in sec. */

/* Delay after START_MOTION before a status update is possible */
#define MOTION_DELAY 0.1

/*----------------debugging-----------------*/
volatile int drvSC800debug = 0;
extern "C" {epicsExportAddress(int, drvSC800debug);}

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvSC800debug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int SC800_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvSC800ReadbackDelay = 0.;


/*----------------functions-----------------*/
static int recv_mess(int card, char *com, int flag);
static RTN_STATUS send_mess(int card, char const *, char *name);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table SC800_access =
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

struct drvSC800_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvSC800 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvSC800);}

static struct thread_args targs = {SCAN_RATE, &SC800_access, MOTION_DELAY};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (SC800_num_cards <=0)
        printf("    No SC800 controllers configured.\n");
    else
    {
        for (card = 0; card < SC800_num_cards; card++)
        {
            struct controller *brdptr = motor_state[card];

            if (brdptr == NULL)
                printf("    SC800 controller %d connection failed.\n", card);
            else
            {
                struct SC800Controller *cntrl;

                cntrl = (struct SC800Controller *) brdptr->DevicePrivate;
                printf("    SC800 controller %d, port=%s, address=%d, id: %s \n", 
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
    if (SC800_num_cards <= 0)
    {
        Debug(1, "init(): SC800 driver disabled. SC800Setup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
    struct SC800Controller *cntrl;
    struct mess_node *nodeptr;
    struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    
    int rtn_state, charcnt, convert_cnt;
    epicsInt32 motorData;
    bool plusdir, ls_active = false, plusLS, minusLS;
    msta_field status;
    int str_axis, str_move, str_norg, str_orgg, str_cwlm, str_ccwl;
    int str_swng, str_errr;

    cntrl = (struct SC800Controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    if (cntrl->status != NORMAL)
	charcnt = recv_mess(card, buff, FLUSH);

    sprintf(buff,"STR1/%d",(signal + 1));
    send_mess(card, buff, (char*) NULL);		/*  Tell Status */
    charcnt = recv_mess(card, buff, 1);
    convert_cnt = sscanf(buff, "C\tSTR%d\t1\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
                         &str_axis, &str_move, &str_norg, &str_orgg,
                         &str_cwlm, &str_ccwl, &str_swng, &str_errr);

    if (charcnt > 0 && convert_cnt == 8)
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
   
    status.Bits.RA_DONE = (str_move == 0) ? 1 : 0;
    plusLS  = (str_cwlm == 1);
    minusLS = (str_ccwl == 1);

   /* Parse motor position */
    sprintf(buff,"RDP%d/0", (signal + 1));
    send_mess(card, buff, (char*) NULL);  /*  Tell Position */
    recv_mess(card, buff, 1);
    convert_cnt = sscanf(buff, "C\tRDP%d\t%d", &str_axis, &motorData);
    
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

    /* Torque enabled? */
    sprintf(buff,"RSY%d/21", (signal + 1));
    send_mess(card, buff, (char*) NULL);  /*  Tell Position */
    recv_mess(card, buff, 1);
    convert_cnt = sscanf(buff, "C\tRSY%d\t21\t%d", &str_axis, &str_move);
    status.Bits.EA_POSITION = (str_move == 0) ? 1 : 0;

    /* Set limit switch error indicators. */
    if (plusLS == true)
    {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	    ls_active = true;
    }
    else
	status.Bits.RA_PLUS_LS = 0;

    if (minusLS == true)
    {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	    ls_active = true;
    }
    else
	status.Bits.RA_MINUS_LS = 0;

    /* encoder status */
    status.Bits.EA_SLIP       = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME       = 0;

    status.Bits.RA_PROBLEM  = 0;

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
/* send a message to the SC800 board                 */
/* send_mess()                                       */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    struct SC800Controller *cntrl;
    char local_buff[MAX_MSG_SIZE];
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
        errlogMessage("drvSC800.c:send_mess(); message size violation.\n");
        return(ERROR);
    }
    else if (comsize == 0)      /* Normal exit on empty input message. */
        return(OK);

    if (!motor_state[card])
    {
        errlogPrintf("drvSC800.c:send_mess() - invalid card #%d\n", card);
        return(ERROR);
    }

    local_buff[0] = (char) STX;
    local_buff[1] = (char) NULL;    /* Terminate local buffer. */

    /* Make a local copy of the string. */
    strcat(&local_buff[1], com);

    Debug(2, "send_mess(): message = %s\n", &local_buff[1]);

    cntrl = (struct SC800Controller *) motor_state[card]->DevicePrivate;

    /* flush any junk at input port - should not be any data available */
    pasynOctetSyncIO->flush(cntrl->pasynUser);

    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff), 
                            TIMEOUT, &nwrite);

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
    struct SC800Controller *cntrl;
    double timeout = 0.;
    size_t nread = 0;
    asynStatus status;
    int eomReason;
    int rtnVal;

    /* Check that card exists */
    if (!motor_state[card])
        return(ERROR);

    cntrl = (struct SC800Controller *) motor_state[card]->DevicePrivate;

    timeout = TIMEOUT;

    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    if (nread > 0)
        Debug(2, "recv_mess(): message = '%s'\n", com);

    if (status != asynSuccess)
    {
        com[0] = '\0';
        rtnVal = -1;
        Debug(1, "recv_mess(): TIMEOUT \n");
    }
    else
        rtnVal = (int)nread;

    return(rtnVal);
}


/*****************************************************/
/* Setup system configuration                        */
/* SC800Setup()                                     */
/*****************************************************/
RTN_STATUS
SC800Setup(int num_cards,       /* maximum number of controllers in system.  */
           int scan_rate)       /* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > SC800_NUM_CARDS)
        SC800_num_cards = SC800_NUM_CARDS;
    else
        SC800_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
        targs.motor_scan_rate = scan_rate;
    else
        targs.motor_scan_rate = SCAN_RATE;

    /* 
     * Allocate space for motor_state structures.  Note this must be done
     * before SC800Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(SC800_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < SC800_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* SC800Config()                                    */
/*****************************************************/
RTN_STATUS
SC800Config(int card,           /* card being configured */
            const char *name,   /* asyn port name */
            int address)        /* asyn address (GPIB) */
{
    struct SC800Controller *cntrl;

    if (card < 0 || card >= SC800_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct SC800Controller));
    cntrl = (struct SC800Controller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
    cntrl->asyn_address = address;
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
    struct SC800Controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int status;
    int total_axis = 0;
    asynStatus success_rtn;
    int version;
    char errbase[] = "\ndrvSC800.cc:motor_init() *** ";

    initialized = true; /* Indicate that driver is initialized. */

    /* Check for setup */
    if (SC800_num_cards <= 0)
        return(ERROR);


    for (card_index = 0; card_index < SC800_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        brdptr->cmnd_response = true;
        total_cards = card_index + 1;
        cntrl = (struct SC800Controller *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 
                                                cntrl->asyn_address, &cntrl->pasynUser, NULL);

        if (success_rtn != asynSuccess)
        {
            char format[] = "%s asyn connection error on port = %s, address = %d ***\n\n";
            errlogPrintf(format, errbase, cntrl->asyn_port, cntrl->asyn_address);
            epicsThreadSleep(5.0);
        }
        else
        {
	    int retry = 0;

            /* Set command End-of-string */
            pasynOctetSyncIO->setInputEos(cntrl->pasynUser,
                                          SC800_IN_EOS,strlen(SC800_IN_EOS));
            pasynOctetSyncIO->setOutputEos(cntrl->pasynUser,
                                           SC800_OUT_EOS,strlen(SC800_OUT_EOS));

            do
            {
                /* Read device type */
                send_mess(card_index, GET_IDENT, NULL);
		status = recv_mess(card_index, buff, 1);
                if (status > 0)
                {
                    int convert_800_cnt, convert_400_cnt, convert_200_cnt;

                    convert_800_cnt = sscanf(buff, "C\tIDN0\t800\t%d", &version);
                    convert_400_cnt = sscanf(buff, "C\tIDN0\t400\t%d", &version);
                    convert_200_cnt = sscanf(buff, "C\tIDN0\t200\t%d", &version);
                    if (convert_800_cnt == 1)
                        cntrl->model = SC800;
                    else if (convert_400_cnt == 1)
                        cntrl->model = SC400;
                    else if (convert_200_cnt == 1)
                        cntrl->model = SC200;
                    else
                        status = 0;
                }
		retry++;
	    } while (status == 0 && retry < 3);
	}

	if (success_rtn == asynSuccess && status > 0)
	{
            cntrl->status = NORMAL;
            if (cntrl->model == SC800)
            {
                sprintf(brdptr->ident, "SC-800 Ver%d", version);
                total_axis = 8;
            }
            else if (cntrl->model == SC400)
            {
                sprintf(brdptr->ident, "SC-400 Ver%d", version);
                total_axis = 4;
            }
            else if (cntrl->model == SC200)
            {
                sprintf(brdptr->ident, "SC-200 Ver%d", version);
                total_axis = 2;
            }
            brdptr->total_axis = total_axis;
	    
            brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;


            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                motor_info->motor_motion = NULL;
                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;


                /* NO Encoder support - internal closed loop controller */
                motor_info->encoder_present = NO;
                motor_info->status.Bits.EA_PRESENT = 0;
                motor_info->pid_present = NO;
                motor_info->status.Bits.GAIN_SUPPORT = 1;

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

    // epicsThreadCreate((char *) "SC800_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);
    epicsThreadCreate((char *) "SC800_motor", 
                      epicsThreadPriorityMedium,
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

    static const iocshFuncDef setupSC800  = {"SC800Setup", 2, SetupArgs};
    static const iocshFuncDef configSC800 = {"SC800Config", 3, ConfigArgs};

    static void setupSC800CallFunc(const iocshArgBuf *args)
    {
        SC800Setup(args[0].ival, args[1].ival);
    }
    static void configSC800CallFunc (const iocshArgBuf *args)
    {
        SC800Config(args[0].ival, args[1].sval, args[2].ival);
    }

    static void SC800Register(void)
    {
        iocshRegister(&setupSC800, setupSC800CallFunc);
        iocshRegister(&configSC800, configSC800CallFunc);
    }

    epicsExportRegistrar(SC800Register);

} // extern "C"

