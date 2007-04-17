/*
FILENAME...     drvOmsPC68.cc
USAGE...        Motor record driver level support for OMS PC68 serial device.

Version:	$Revision: 1.5 $
Modified By:	$Author: rivers $
Last Modified:	$Date: 2007-04-17 21:08:44 $
*/

/*
 *      Original Author: Brian Tieman
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
 *
 * NOTES
 * -----
 * Verified with firmware:
 *	- PC78 ver 1.14-46
 *      - PC68 ver 6.51-2008
 *
 * Modification Log:
 * -----------------
 * .01 02/07/06 dmk - modified motor_init() to skip the encoder query if the
 *                    controller has been configured for no encoder.
 * .02 04/24/06 rls - support for both PC68 and PC78.
 *                  - test for encoder support.
 *                  - test for servo support.
 * .03 08/11/06 rls - work around for erroneous response after response to
 *                    "?KP" command at boot-up; resulted in 1st axis having
 *                    same position (RP command) as last axis.
 * .04 02/09/07 dmk - modified to support interrupts.
 */

#include <string.h>
#include <math.h>
#include <epicsThread.h>
#include <dbAccess.h>
#include <drvSup.h>
#include <iocsh.h>

#include "motor.h"
#include "drvOmsPC68Com.h"
#include "epicsExport.h"

#define PC68_MAX_NUM_CARDS  (10)    /* Maximum # of cards. */
#define BUFF_SIZE           (100)   /* Maximum length of command string */

#define TIMEOUT             (2.0)   /* Command timeout in sec. */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef      DEBUG
        #define Debug(l, f, args...) {if(l<=drvOmsPC68debug) printf(f,## args);}
    #else
        #define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

volatile int drvOmsPC68debug = 0;

extern "C" {epicsExportAddress(int, drvOmsPC68debug);}

/* --- Global data. --- */
int OmsPC68_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"

static volatile int motionTO = 10;

//OmsPC68 generic controller commands
static char *oms_axis[] = {"X", "Y", "Z", "T", "U", "V", "R", "S"};


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
static int omsGet(int card, char *pcom);

struct driver_table OmsPC68_access =
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
    oms_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvOmsPC68 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvOmsPC68);}

static struct thread_args targs = {SCAN_RATE, &OmsPC68_access, 0.0};

//_____________________________________________________________________________
/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (OmsPC68_num_cards <= 0)
        printf("    No MDrive controllers configured.\n");
    else
    {
        for (card = 0; card < OmsPC68_num_cards; card++)
        {
            struct controller *brdptr = motor_state[card];

            if (brdptr == NULL)
                printf("    PC68 controller %d connection failed.\n", card);
            else
            {
                struct OmsPC68controller *cntrl;

                cntrl = (struct OmsPC68controller *) brdptr->DevicePrivate;
                printf("    PC68 controller #%d, port=%s, id: %s - err %d\n", card,
                       cntrl->asyn_port, brdptr->ident,cntrl->errcnt);
            }
        }
    }
    return(OK);
}


//_____________________________________________________________________________

static long init()
{
    /* Check for setup */
    if (OmsPC68_num_cards <= 0)
    {
        Debug(1, "init(): OmsPC68 driver disabled.\n");
        Debug(1, "init(): OmsPC68Setup() missing from startup script.\n");
    }

    motor_init ();
    initialized = true;
    return(0);
}

//_____________________________________________________________________________

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
}
//_____________________________________________________________________________
/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/
static int set_status(int card, int signal)
{
    struct OmsPC68controller    *cntrl;
    struct mess_info        *motor_info;
    struct mess_node        *nodeptr;
    char                    *p, *tok_save;
    struct axis_status      *ax_stat;
    struct encoder_status   *en_stat;
    char                    q_buf[50], outbuf[50];
    int                     index, motorData, rtn_state;
    bool                    ls_active = false;
    msta_field              status;

    cntrl = (struct OmsPC68controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    if (cntrl->status != NORMAL)
        recv_mess(card, q_buf, FLUSH);

    if (motor_state[card]->motor_info[signal].encoder_present == YES)
    {
        /* get 4 peices of info from axis */
        send_mess(card, ALL_INFO, oms_axis[signal]);
        rtn_state = recv_mess(card, q_buf, 4);
    }
    else
    {
        send_mess(card, AXIS_INFO, oms_axis[signal]);
        rtn_state = recv_mess(card, q_buf, 2);
    }

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

    Debug(5, "info = (%s)\n", q_buf);

    for (index = 0, p = strtok_r(q_buf, ",", &tok_save); p;
         p = strtok_r(NULL, ",", &tok_save), index++)
    {
        switch (index)
        {
        case 0:
            /* axis status */
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
        case 1:
            /* motor pulse count (position) */
            sscanf(p, "%index", &motorData);

            if (motorData == motor_info->position)
            {
                /* Increment counter only if motor is moving. */
                if (nodeptr != 0)
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
                errlogSevPrintf(errlogMinor,
                  "Motor motion timeout ERROR on card: %d, signal: %d\n", card,
                                signal);
            }
            else
                status.Bits.RA_PROBLEM = 0;

            break;
        case 2:
            {
                /* encoder pulse count (position) */
                int temp;

                sscanf(p, "%index", &temp);
                motor_info->encoder_position = (epicsInt32) temp;
            }
            break;
        case 3:
            /* encoder status */
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
                buffer[size] = '\0';

                if (strncmp(buffer, "@PUT(", 5) != 0)
                    goto errorexit;

                /* Point "start" to PV name argument. */
                tail = NULL;
                start = strtok_r(&buffer[5], ",", &tail);
                if (tail == NULL)
                    goto errorexit;

                if (dbNameToAddr(start, &addr)) /* Get address of PV. */
                {
                    errPrintf(-1, __FILE__, __LINE__, "Invalid PV name: %s",
                              start);
                    goto errorexit;
                }

                /* Point "start" to PV value argument. */
                start = strtok_r(NULL, ")", &tail);
                if (dbPutField(&addr, DBR_STRING, start, 1L))
                {
                    errPrintf(-1, __FILE__, __LINE__, "invalid value: %s",
                              start);
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

exit:
    motor_info->status.All = status.All;        /* Update status from local copy. */
    return(rtn_state);
}


//_____________________________________________________________________________
/*****************************************************/
/* send a message to the OmsPC68 board               */
/* send_mess()                                       */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    struct OmsPC68controller    *cntrl;
    size_t              size,
    nwrite;
    int             error_code;
    char                outbuf[MAX_MSG_SIZE];

    if (!motor_state[card])
    {
        errlogPrintf("drvOmsPC68.c:send_mess() - invalid card #%d\n", card);
        return(ERROR);
    }

    if (name == NULL)
        strcpy(outbuf, com);
    else
    {
        strcpy(outbuf, "A");
        strcat(outbuf, name);
        strcat(outbuf, " ");
        strcat(outbuf, com);
    }

    size = strlen (outbuf);
    if (size > MAX_MSG_SIZE)
    {
        errlogMessage("drvOmsPC68.c:send_mess(); message size violation.\n");
        return(ERROR);
    }
    else
        if (size == 0)  /* Normal exit on empty input message. */
            return(OK);

    Debug(2, "send_mess(): message = %s\n", com);

    cntrl = (struct OmsPC68controller *) motor_state[card]->DevicePrivate;

    error_code = cntrl->pasynOctet->write(cntrl->octetPvt,cntrl->pasynUser,outbuf,size,&nwrite);
    if (error_code == OK)
    {
        Debug(4, "sent message: (%s)\n", outbuf);
    }
    else
    {
        Debug(4, "unable to send message (%s)\n", outbuf);
    }
    return(OK);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int amount)
 *
 * INPUT ARGUMENTS...
 *	card - controller card # (0,1,...).
 *	*com - caller's response buffer.
 *	amount	| -1 = flush controller's output buffer.
 *		| >= 1 = the # of command responses to retrieve into caller's
 *				response buffer.
 *
 * LOGIC...
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *  IF "amount" indicates buffer flush.
 *	WHILE characters left in input buffer.
 *	    Call omsGet().
 *	ENDWHILE
 *  ENDIF
 *
 *  FOR each message requested (i.e. "amount").
 *	Initialize head and tail pointers.
 *	Initialize retry counter and state indicator.
 *	WHILE retry count not exhausted, AND, state indicator is NOT at END.
 *	    IF characters left in controller's input buffer.
 *		Process input character.
 *	    ELSE IF command error occured - call omsError().
 *		ERROR RETURN.
 *	    ENDIF
 *	ENDWHILE
 *	IF retry count exhausted.
 *	    Terminate receive buffer.
 *	    ERROR RETURN.
 *	ENDIF
 *	Terminate command response.
 *  ENDFOR
 *
 *  IF commands processed.
 *	Terminate response buffer.
 *  ELSE
 *	Clear response buffer.
 *  ENDIF
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int amount)
{
    struct OmsPC68controller *cntrl;
    int itera, trys, piece, head_size, tail_size;
    char inchar;

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
        cntrl = (struct OmsPC68controller *) motor_state[card]->DevicePrivate;
        cntrl->pasynOctet->flush(cntrl->octetPvt,cntrl->pasynUser);
        return(0);
    }

    for (itera = 0; amount > 0; amount--)
    {
	Debug(7, "-------------");
	head_size = 0;
	tail_size = 0;

	for (piece = 0, trys = 0; piece < 3 && trys < 3; trys++)
	{
	    if (omsGet(card, &inchar))
	    {
		Debug(7, "%02x", inchar);

		switch (piece)
		{
		case 0:	/* header */
		    if (inchar == '\n' || inchar == '\r')
			head_size++;
                    else if ((inchar == '#') ||      //command error
                             (inchar == '$') ||      //motor slip
                             (inchar == '@') ||      //over travel
                             (inchar == '!'))        //done
                    {
                        if (inchar == '#')
                        {
                            Debug(4, "command error: card %d\n", card);
                            return(-1);
                        }
			head_size++;
                    }
		    else
		    {
			piece++;
			com[itera++] = inchar;
		    }
		    break;
		case 1:	/* body */
		    if (inchar == '\n' || inchar == '\r')
		    {
			piece++;
			tail_size++;
		    }
		    else
			com[itera++] = inchar;
		    break;

		case 2:	/* trailer */
		    tail_size++;
		    if (tail_size >= head_size)
			piece++;
		    break;
		}

		trys = 0;
	    }
	}
	Debug(7, "-------------\n");
	if (trys >= 3)
	{
	    Debug(1, "Timeout occurred in recv_mess\n");
	    com[itera] = '\0';
	    return(0);
	}
    com[itera++] = ',';
    }

    if (itera > 0)
        com[itera - 1] = '\0';
    else
        com[itera] = '\0';

    Debug(4, "recv_mess: card %d, msg: (%s)\n", card, com);
    return(itera);
}


/*****************************************************/
/* Get next character from OMS input buffer          */
/*		omsGet()			     */
/*****************************************************/
static int omsGet(int card, char *pchar)
{
    int         eomReason;
    size_t      nread;
    asynStatus  status;
    struct OmsPC68controller *cntrl;
    
    cntrl = (struct OmsPC68controller *) motor_state[card]->DevicePrivate;
    status = cntrl->pasynOctet->read(cntrl->octetPvt,cntrl->pasynUser,pchar,1,&nread,&eomReason);

    return(nread);
}

static void asynCallback(void *drvPvt,asynUser *pasynUser,char *data,size_t len, int eomReason, asynStatus status)
{
    int d,cnt,stat,done;
    OmsPC68controller* pcntrl;
    struct controller* pstate;

    pcntrl = (OmsPC68controller*)drvPvt;
    pstate = motor_state[pcntrl->card];

//printf("drvOmsPC68:asynCallback - %2.2d - cnt %6.6d - stat 0x%2.2X - done 0x%2.2X\n",pcntrl->card,cnt,stat,done);

    if( pcntrl->card >= total_cards || pstate == NULL )
    {
        errlogPrintf("Invalid entry-card #%d\n", pcntrl->card);
        return;
    }

    d = *(int*)data;
    cnt  = (d & 0xFFFF0000) >> 16;
    stat = (d & 0x0000FF00) >> 8;
    done = (d & 0x000000FF);

    if( stat & STAT_DONE )
        if( stat & STAT_ERROR_MSK )
            ++pcntrl->errcnt;
        else
            motor_sem.signal();
}
//_____________________________________________________________________________
/*****************************************************/
/* initialize all software and hardware              */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()                                      */
/*****************************************************/
static int motor_init()
{
    volatile struct controller* pmotorState;
    struct OmsPC68controller* cntrl;
    int card_index, status, success_rtn, total_axis, motor_index;
    char axis_pos[50], encoder_pos[50], *tok_save, *pos_ptr;
    asynUser      *pasynUser;
    asynInterface *pasynInterface;

    /* Check for setup */
    if (OmsPC68_num_cards <= 0)
    {
        Debug(1, "motor_init: OmsPC68 driver disabled* \n");
        Debug(1, "motor_init: OmsPC68Setup() is missing from startup script.\n");
        return(ERROR);
    }

    total_cards = OmsPC68_num_cards;

    for (card_index=0;card_index<OmsPC68_num_cards;card_index++)
    {
        if (!motor_state[card_index])
            continue;

        pmotorState = motor_state[card_index];

        /* Initialize communications channel */
        cntrl = (struct OmsPC68controller*)pmotorState->DevicePrivate;

        pasynUser = pasynManager->createAsynUser(0,0);
        pasynUser->userPvt = cntrl;
        cntrl->pasynUser = pasynUser;

        success_rtn = pasynManager->connectDevice(pasynUser,cntrl->asyn_port,0);
        if( success_rtn )
        {
            Debug(1,"can't connect to port %s: %s\n",cntrl->asyn_port,pasynUser->errorMessage);
            return(ERROR);
        }

        pasynInterface = pasynManager->findInterface(pasynUser,asynOctetType,1);
        if( pasynInterface )
        {
            cntrl->pasynOctet = (asynOctet*)pasynInterface->pinterface;
            cntrl->octetPvt = pasynInterface->drvPvt;
        }
        else
        {
            Debug(1,"%s driver not supported\n",asynOctetType);
            return(ERROR);
        }

        success_rtn = cntrl->pasynOctet->registerInterruptUser(cntrl->octetPvt,pasynUser,asynCallback,cntrl,&cntrl->registrarPvt);
        if( success_rtn )
        {
            Debug(1,"registerInterruptUser failed - %s: %s\n",cntrl->asyn_port,pasynUser->errorMessage);
            return(ERROR);
        }

        if (success_rtn == asynSuccess)
        {
            int retry = 0;

            /* Send a message to the board, see if it exists */
            /* flush any junk at input port - should be no data available */
            cntrl->pasynOctet->flush(cntrl->octetPvt,cntrl->pasynUser);

            /* Try 3 times to connect to controller. */
            do
            {
                send_mess (card_index, GET_IDENT, (char) NULL);
                status = recv_mess(card_index, (char *) pmotorState->ident, 1);
                retry++;
            } while (status == 0 && retry < 3);

            Debug(3, "Identification = %s\n", pmotorState->ident);
        }

        if (success_rtn == asynSuccess && status > 0)
        {
            pmotorState->motor_in_motion = 0;
            pmotorState->cmnd_response = false;

            send_mess (card_index, ECHO_OFF, (char) NULL);
            send_mess (card_index, ERROR_CLEAR, (char) NULL);
            send_mess (card_index, STOP_ALL, (char) NULL);

            send_mess (card_index, ALL_POS, (char) NULL);
            recv_mess (card_index, axis_pos, 1);

            for (total_axis = 0, pos_ptr = strtok_r(axis_pos, ",", &tok_save);
                 pos_ptr; pos_ptr = strtok_r(NULL, ",", &tok_save),
                 total_axis++)
            {
                pmotorState->motor_info[total_axis].motor_motion = NULL;
                pmotorState->motor_info[total_axis].status.All = 0;
            }

            Debug(3, "Total axis = %d\n", total_axis);
            pmotorState->total_axis = total_axis;

            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                send_mess(card_index, ENCODER_QUERY, oms_axis[motor_index]);
                if (recv_mess(card_index, encoder_pos, 1) == -1)
                    pmotorState->motor_info[motor_index].encoder_present = NO;
                else
                    pmotorState->motor_info[motor_index].encoder_present = YES;

                /* Test if motor has PID parameters. */
                send_mess(card_index, PID_QUERY, oms_axis[motor_index]);
                if (recv_mess(card_index, encoder_pos, 1) == -1)
                    pmotorState->motor_info[motor_index].pid_present = NO;
                else
                    pmotorState->motor_info[motor_index].pid_present = YES;
            }
            
            /* Testing for PID parameters (?KP) causes erroneous response from
             * "report position" command at boot-up. Work around is the following
             * dummy communication transaction.
             */

            send_mess (card_index, ALL_POS, (char) NULL);
            recv_mess (card_index, axis_pos, 1);

            for (motor_index=0;motor_index<total_axis;motor_index++)
            {
                pmotorState->motor_info[motor_index].status.All = 0;
                pmotorState->motor_info[motor_index].no_motion_count = 0;
                pmotorState->motor_info[motor_index].encoder_position = 0;
                pmotorState->motor_info[motor_index].position = 0;

                if (pmotorState->motor_info[motor_index].encoder_present == YES)
                    pmotorState->motor_info[motor_index].status.Bits.EA_PRESENT = 1;
		if (pmotorState->motor_info[motor_index].pid_present == YES)
		    pmotorState->motor_info[motor_index].status.Bits.GAIN_SUPPORT = 1;

                set_status (card_index, motor_index);
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

    Debug(3, "Motors initialized\n");
    epicsThreadCreate((char *) "OmsPC68_motor", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);
    Debug(3, "Started motor_task\n");

    return(0);
}

//_____________________________________________________________________________
/*****************************************************/
/* Setup system configuration                        */
/* OmsPC68Setup()                                     */
/*****************************************************/
RTN_STATUS OmsPC68Setup (int num_cards, int scan_rate)
{
    int itera;

    if (num_cards < 1 || num_cards > PC68_MAX_NUM_CARDS)
        OmsPC68_num_cards = PC68_MAX_NUM_CARDS;
    else
        OmsPC68_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
        targs.motor_scan_rate = scan_rate;
    else
        targs.motor_scan_rate = SCAN_RATE;

    /* 
     * Allocate space for motor_state structures.  Note this must be done
     * before OmsPC68Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **)
                  malloc(OmsPC68_num_cards * sizeof(struct controller *));

    for (itera = 0; itera < OmsPC68_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;

    return(OK);
}
//___________________________________________
/*******************************************/
/* Configure a controller                  */
/* OmsPC68Config()                         */
/*                                         */
/* Recieves:                               */
/*    card       - card being configured   */
/*    name       - asyn port name          */
/*******************************************/
RTN_STATUS OmsPC68Config(int card, const char *name)
{
    struct OmsPC68controller *cntrl;

    if (card < 0 || card >= OmsPC68_num_cards)
        return(ERROR);

    motor_state[card] = (struct controller*)calloc(1,sizeof(struct controller));
    motor_state[card]->DevicePrivate = calloc(1,sizeof(struct OmsPC68controller));
    cntrl = (struct OmsPC68controller*) motor_state[card]->DevicePrivate;

    cntrl->card = card;
    strcpy(cntrl->asyn_port,name);
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

    static const iocshArg *const SetupArgs[2]  = {&setupArg0, &setupArg1};
    static const iocshArg *const ConfigArgs[2] = {&configArg0, &configArg1};

    static const iocshFuncDef setupOmsPC68  = {"OmsPC68Setup", 2, SetupArgs};
    static const iocshFuncDef configOmsPC68 = {"OmsPC68Config", 2, ConfigArgs};

    static void setupOmsPC68CallFunc(const iocshArgBuf *args)
    {
        OmsPC68Setup(args[0].ival, args[1].ival);
    }
    static void configOmsPC68CallFunc (const iocshArgBuf *args)
    {
        OmsPC68Config(args[0].ival, args[1].sval);
    }

    static void OmsPC68Register(void)
    {
        iocshRegister(&setupOmsPC68, setupOmsPC68CallFunc);
        iocshRegister(&configOmsPC68, configOmsPC68CallFunc);
    }

    epicsExportRegistrar(OmsPC68Register);

} // extern "C"


//_____________________________________________________________________________
