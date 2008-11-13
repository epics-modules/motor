/*
FILENAME... drvEnsemble.cc
USAGE...    Motor record driver level support for Aerotech Ensemble.

Version:    1.00
Modified By:    weimer
Last Modified:  2008/04/10 05:51:48 PM
*/

/*
 *      Original Author: Mark Rivers
 *      Date: 10/20/97
 *		Current Author: Chad Weimer
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
 * .01	04-04-08	caw		initialized from drvMM4000.cc (Newport)
 */


#include <string.h>
#include <math.h>
#include <stdio.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <drvSup.h>
#include "motor.h"
#include "drvEnsemble.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

/* Status byte bits */
#define ENABLED_BIT		0x00000001
#define IN_POSITION_BIT	0x00000004
#define IN_MOTION_BIT	0x00000008
#define DIRECTION_BIT	0x00000200
#define PLUS_LIMIT_BIT	0x00004000
#define MINUS_LIMIT_BIT	0x00008000
#define HOME_LIMIT_BIT	0x00010000
#define HOME_MARKER_BIT	0x00020000

// This can really be any number, because there isn't any theoretical
// restriction on the number of Ensembles that can be on a network.
// However, a relatively low number was picked for practical purposes
#define Ensemble_NUM_CARDS    10

// Due to the way some of the commands are implemented in the Ensemble,
// such as HOME, the command response is not returned until the command
// has actually finished executing. This requires a fairly large timeout
// value. However, even a large value will not guarantee that the response will
// be sent back in the time alloted.
#define TIMEOUT 20.0 /* Command timeout in sec. */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef  DEBUG
    #define Debug(l, f, args...) { if(l<=drvEnsembledebug) printf(f,## args); }
    #else
    #define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvEnsembledebug = 0;
extern "C" {epicsExportAddress(int, drvEnsembledebug);}

/* --- Local data. --- */
int Ensemble_num_cards = 0;
int num_cmds = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include    "motordrvComCode.h"


/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *name);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table Ensemble_access =
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

struct
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvEnsemble = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvEnsemble);}

static struct thread_args targs = {SCAN_RATE, &Ensemble_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
	int card;

	if (Ensemble_num_cards <=0)
		printf("    No Ensemble controllers configured.\n");
	else
	{
		for (card = 0; card < Ensemble_num_cards; card++)
		{
			struct controller *brdptr = motor_state[card];

			if (brdptr == NULL)
				printf("    Ensemble controller %d connection failed.\n", card);
			else
			{
				struct Ensemblecontroller *cntrl;

				cntrl = (struct Ensemblecontroller *) brdptr->DevicePrivate;
				printf("    Ensemble controller %d, port=%s, address=%d, id: %s \n",
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
	if (Ensemble_num_cards <= 0)
	{
		Debug(1, "init(): Ensemble driver disabled. EnsembleSetup() missing from startup script.\n");
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
	struct Ensemblecontroller *cntrl;
	struct mess_node *nodeptr;
	register struct mess_info *motor_info;
	double motorData, pfbk;
	bool plusdir, ls_active = false;
	msta_field status;
	int rtn_state, comm_status, axis_status;
	char buff[BUFF_SIZE];
	
	cntrl = (struct Ensemblecontroller *) motor_state[card]->DevicePrivate;
	motor_info = &(motor_state[card]->motor_info[signal]);
	status.All = motor_info->status.All;

	// get the axis status
	sprintf(buff, "AXISSTATUS(@%d)", signal);
	send_mess(card, buff, (char) NULL);
	comm_status = recv_mess(card, buff, 1);
	if(buff[0] == ASCII_ACK_CHAR)
	{
		// convert to an integer
		axis_status = atoi(&buff[1]);
	}
	else
	{
		axis_status = 0;
	}

    if (comm_status <= 0)
	{
        if (cntrl->status == NORMAL)
        {
            cntrl->status = RETRY;
            rtn_state = OK;
        }
        else
        {
            cntrl->status = COMM_ERR;
            status.Bits.CNTRL_COMM_ERR = 1;
            status.Bits.RA_PROBLEM = 1;
            rtn_state = 1;
        }
    }
    else
    {
        cntrl->status = NORMAL;
        status.Bits.CNTRL_COMM_ERR = 0;
		
		nodeptr = motor_info->motor_motion;
		
		status.Bits.EA_SLIP = 0;
		status.Bits.EA_SLIP_STALL = 0;
		
		// fill in the status
		status.Bits.RA_DIRECTION = axis_status & DIRECTION_BIT ? 1 : 0;
		status.Bits.RA_DONE = axis_status & IN_POSITION_BIT ? 1 : 0;
		status.Bits.RA_PLUS_LS = axis_status & PLUS_LIMIT_BIT ? 1 : 0;
		status.Bits.RA_HOME = axis_status & HOME_LIMIT_BIT ? 1 : 0;
		status.Bits.EA_POSITION = axis_status & ENABLED_BIT ? 1 : 0;
		status.Bits.EA_HOME = axis_status & HOME_MARKER_BIT ? 1 : 0;
		status.Bits.RA_MOVING = axis_status & IN_MOTION_BIT ? 1 : 0;
		status.Bits.RA_MINUS_LS = axis_status & MINUS_LIMIT_BIT ? 1 : 0;
		
		plusdir = status.Bits.RA_DIRECTION ? true : false;
		if((status.Bits.RA_PLUS_LS && plusdir) || (status.Bits.RA_MINUS_LS && !plusdir))
		{
			ls_active = true;
		}

		// get the position
		sprintf(buff, "PFBKPROG(@%d)", signal);
		send_mess(card, buff, (char) NULL);
		recv_mess(card, buff, 1);
		if(buff[0] == ASCII_ACK_CHAR)
		{
			// convert to an integer
			pfbk = atof(&buff[1]);
		}
		else
		{
			pfbk = 0;
		}
		
		// fill in the position
		motorData = pfbk / cntrl->drive_resolution[signal];
		
		if(motorData == motor_info->position)
		{
			// only increment the counter if the motor is moving
			if(nodeptr != 0)
			{
				motor_info->no_motion_count++;
			}
		}
		else
		{
			motor_info->position = NINT(motorData);
			if(motor_info->encoder_present == YES)
			{
				motor_info->encoder_position = NINT(motorData);
			}
			else
			{
				motor_info->encoder_position = 0;
			}
			
			motor_info->no_motion_count = 0;
		}
		
		// velocity is not used, so don't bother doing a command down
		// to the controller to get it
		motor_info->velocity = 0;
		
		// do this "last", so that we know no errors occurred
		status.Bits.RA_PROBLEM = 0;
		
		rtn_state = (!motor_info->no_motion_count || ls_active ||
			status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;
		
		// test for post-move string
		if((status.Bits.RA_DONE || ls_active) && nodeptr != 0 && nodeptr->postmsgptr != 0)
		{
			strcpy(buff, nodeptr->postmsgptr);
			send_mess(card, buff, (char) NULL);
			nodeptr->postmsgptr = NULL;
		}
	}
	
	
	motor_info->status.All = status.All;
	return (rtn_state);
}


/*****************************************************/
/* send a message to the Ensemble board            */
/* send_mess()                               */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
	struct Ensemblecontroller *cntrl;
	int size;
	size_t nwrite;
	char *eos_tok, com_cpy[BUFF_SIZE], buff[80];
	asynStatus status;
	
	size = strlen(com);
	
	if(size > MAX_MSG_SIZE)
	{
		errlogMessage("drvEnsemble.c:send_mess(); message size violation.\n");
		return (ERROR);
	}
	else if(size == 0)
	{
		return (OK);
	}
	
	if(!motor_state[card])
	{
		errlogMessage("drvEnsemble.c:send_mess() - invalid card #%d\n");
		return (ERROR);
	}
	
	if(name != NULL)
	{
		errlogMessage("drvEnsemble.c:send_mess() - invalid argument = %s\n");
		return (ERROR);
	}

    Debug(2, "send_mess(): message = %s\n", com);

	// We need to track the number of individual Ensemble commands that are
	// being sent, so that we can read back the same number of responses.
	// This is necessary, because the Ensemble will send individual responses
	// terminated by the EOS char for each command sent, even if those commands
	// were all sent as part of one motor record command, such as LOAD_POS

	num_cmds = 0;
	strcpy(com_cpy, com);
	eos_tok = strtok(com_cpy, ASCII_EOS_STR);
	while(eos_tok != NULL)
	{
		num_cmds++;
		eos_tok = strtok(NULL, ASCII_EOS_STR);
	}
	
	cntrl = (struct Ensemblecontroller *) motor_state[card]->DevicePrivate;
	
	status = pasynOctetSyncIO->write(cntrl->pasynUser, com, size, TIMEOUT, &nwrite);

	if(status != asynSuccess || nwrite <= 0)
	{
		errlogMessage(cntrl->pasynUser->errorMessage);
		return (ERROR);
	}
	
	return (OK);
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
	struct Ensemblecontroller *cntrl;
	double timeout = 0;
	size_t nread = 0;
	asynStatus status;
	int eomReason, i;
	char buff[BUFF_SIZE];
	
	if(!motor_state[card])
	{
		return (ERROR);
	}
	
	cntrl = (struct Ensemblecontroller *) motor_state[card]->DevicePrivate;
	
	timeout = TIMEOUT;
	com[0] = '\0';
	for(i = 0; i < num_cmds; i++)
	{
		status = pasynOctetSyncIO->read(cntrl->pasynUser, buff, BUFF_SIZE, timeout, &nread, &eomReason);
	
		if(status == asynSuccess && nread > 0)
		{
			strcat(com, buff);
		}
		else
		{
			com[0] = '\0';
			nread = 0;
			break;
		}
	}	
	
	Debug(2, "recv_mess(): message = \"%s\"\n", com);
	return (nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* EnsembleSetup()                                     */
/*****************************************************/
RTN_STATUS EnsembleSetup(int num_cards,  /* maximum number of controllers in system.  */
        int scan_rate)  /* polling rate - 1/60 sec units.  */
{
	int i;
	
	if(num_cards < 1 || num_cards > Ensemble_NUM_CARDS)
	{
		Ensemble_num_cards = Ensemble_NUM_CARDS;
	}
	else
	{
		Ensemble_num_cards = num_cards;
	}
	
	// Set motor polling task rate
	if(scan_rate >= 1 && scan_rate <= 60)
	{
		targs.motor_scan_rate = scan_rate;
	}
	else
	{
		targs.motor_scan_rate = SCAN_RATE;
	}
	
	/*
	 * Allocate space for motor_state structures.  Note this must be done
	 * before EnsembleConfig is called, so it cannot be done in motor_init()
	 * This means that we must allocate space for a card without knowing
	 * if it really exists, which is not a serious problem
	 */
	motor_state = (struct controller **) malloc(Ensemble_num_cards * sizeof(struct controller *));
	
	for(i = 0; i < Ensemble_num_cards; i++)
	{
		motor_state[i] = (struct controller *) NULL;
	}
	
	return (OK);
}


/*****************************************************/
/* Configure a controller                            */
/* EnsembleConfig()                                    */
/*****************************************************/
RTN_STATUS EnsembleConfig(int card,      /* card being configured */
            const char *name,   /* asyn port name */
            int addr)           /* asyn address (GPIB) */
{
	struct Ensemblecontroller *cntrl;
	
	if(card < 0 || card >= Ensemble_num_cards)
	{
		return (ERROR);
	}
	
	motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
	motor_state[card]->DevicePrivate = malloc(sizeof(struct Ensemblecontroller));
	cntrl = (struct Ensemblecontroller *) motor_state[card]->DevicePrivate;
	
	strcpy(cntrl->asyn_port, name);
	cntrl->asyn_address = addr;
	
	return (OK);
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
	struct Ensemblecontroller *cntrl;
	int card_index, motor_index, status, digits;
	char buff[BUFF_SIZE];
	asynStatus success_rtn;
	
	// Indicates that the driver is initialized
	initialized = true;
	
	if(Ensemble_num_cards <= 0)
	{
		return (ERROR);
	}
	
	for(card_index = 0; card_index < Ensemble_num_cards; card_index++)
	{
		if(!motor_state[card_index])
		{
			continue;
		}
		
		brdptr = motor_state[card_index];
		brdptr->cmnd_response = true;
		total_cards = card_index + 1;
		cntrl = (struct Ensemblecontroller *)brdptr->DevicePrivate;
		
		// Initialize communications channel
		success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, cntrl->asyn_address, &cntrl->pasynUser, NULL);
		
		if(success_rtn == asynSuccess)
		{
			int retry = 0;
			
			// Send a message to the baord, see if it exists
			// flush any junk at input port - should not be any data available
			pasynOctetSyncIO->flush(cntrl->pasynUser);
			
			do
			{
				// we only care if we get a response
				// so we don't need to send a valid command
				strcpy(buff, "NONE");
				send_mess(card_index, buff, (char) NULL);
				status = recv_mess(card_index, buff, 1);
				
				retry++;
			} while(!status && retry < 3);
			
			if(status > 0)
			{
				brdptr->localaddr = (char *) NULL;
				brdptr->motor_in_motion = 0;
				// Read controller ID string
				strcpy(buff, "GETPARM(CONTROL, 265)"); //UserString1
				send_mess(card_index, buff, (char) NULL);
				recv_mess(card_index, buff, 1);
				if(buff[0] == ASCII_ACK_CHAR)
				{
					strcpy(brdptr->ident, &buff[1]);
				}
				else
				{
					sprintf(brdptr->ident, "Ensemble%d", card_index);
				}
				
				// Get the number of axes
				brdptr->total_axis = 0;
				for(motor_index = 0; motor_index < 10; motor_index++)
				{
					// Does this axis actually exist?
					sprintf(buff, "GETPARM(@%d, 257)", motor_index); //AxisName
					send_mess(card_index, buff, (char) NULL);
					recv_mess(card_index, buff, 1);

					// We know the axis exists if we got an ACK response
					if(buff[0] == ASCII_ACK_CHAR)
					{
						cntrl->axes[motor_index] = 1;
						brdptr->total_axis++;
					}
				}
				
				for(motor_index = 0; motor_index < 10; motor_index++)
				{
					if(cntrl->axes[motor_index])
					{
						struct mess_info *motor_info = &brdptr->motor_info[motor_index];
					
						motor_info->status.All = 0;
						motor_info->no_motion_count = 0;
						motor_info->encoder_position = 0;
						motor_info->position = 0;

						// Determine if encoder present based on open/closed loop mode.
						sprintf(buff, "GETPARM(@%d, 58)", motor_index); //CfgFbkPosType
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if(buff[0] == ASCII_ACK_CHAR)
						{
							if(atoi(&buff[1]) > 0)
							{
								motor_info->encoder_present = YES;
								motor_info->status.Bits.EA_PRESENT = 1;
							}
						}

						// Determine if gains are supported based on the motor type.
						sprintf(buff, "GETPARM(@%d, 33)", motor_index); //CfgMotType
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if(buff[0] == ASCII_ACK_CHAR)
						{
							if(atoi(&buff[1]) != 3)
							{
								motor_info->pid_present = YES;
								motor_info->status.Bits.GAIN_SUPPORT = 1;
							}
						}
				
						// Stop all motors
						sprintf(buff, "ABORT @%d", motor_index);
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
					
						// Determive drive resolution
						sprintf(buff, "GETPARM(@%d, 3)", motor_index); //PosScaleFactor
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if(buff[0] == ASCII_ACK_CHAR)
						{
							cntrl->drive_resolution[motor_index] = 1 / atof(&buff[1]);
						}
						else
						{
							cntrl->drive_resolution[motor_index] = 1;
						}
					
						digits = (int) -log10(cntrl->drive_resolution[motor_index]) + 2;
						if(digits < 1)
						{
							digits = 1;
						}
						cntrl->res_decpts[motor_index] = digits;
					
						// Save home preset position
						sprintf(buff, "GETPARM(@%d, 108)", motor_index); //HomeOffset
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if(buff[0] == ASCII_ACK_CHAR)
						{
							cntrl->home_preset[motor_index] = atof(&buff[1]);
						}
					
						// Determine low limit
						sprintf(buff, "GETPARM(@%d, 47)", motor_index); //ThresholdSoftCCW
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if(buff[0] == ASCII_ACK_CHAR)
						{
							motor_info->low_limit = atof(&buff[1]);
						}
					
						// Determine high limit
						sprintf(buff, "GETPARM(@%d, 48)", motor_index); //ThresholdSoftCW
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if(buff[0] == ASCII_ACK_CHAR)
						{
							motor_info->high_limit = atof(&buff[1]);
						}
					
						// Read status of each motor
						set_status(card_index, motor_index);
					}
				}
			}
			else
			{
				motor_state[card_index] = (struct controller *) NULL;
			}
		}
	}

	any_motor_in_motion = 0;

	mess_queue.head = (struct mess_node *) NULL;
	mess_queue.tail = (struct mess_node *) NULL;

	free_list.head = (struct mess_node *) NULL;
	free_list.tail = (struct mess_node *) NULL;

	epicsThreadCreate((char *) "Ensemble_motor",
						epicsThreadPriorityMedium,
						epicsThreadGetStackSize(epicsThreadStackMedium),
						(EPICSTHREADFUNC) motor_task, (void *) &targs);

	return (OK);
}

