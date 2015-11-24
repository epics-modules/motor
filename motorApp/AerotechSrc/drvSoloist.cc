/*
* FILENAME... drvSoloist.cc
* USAGE...    Motor record driver level support for Aerotech Soloist.
*
*/

/*
*	Original Author: Hong Fung
*	Date: 04/29/09
*	Current Author: Aerotech, Inc.
*
*	Experimental Physics and Industrial Control System (EPICS)
*
*	Copyright 1991, the Regents of the University of California,
*	and the University of Chicago Board of Governors.
*
*	This software was produced under  U.S. Government contracts:
*	(W-7405-ENG-36) at the Los Alamos National Laboratory,
*	and (W-31-109-ENG-38) at Argonne National Laboratory.
*
*	Initial development by:
*		The Controls and Automation Group (AT-8)
*		Ground Test Accelerator
*		Accelerator Technology Division
*		Los Alamos National Laboratory
*
*   Co-developed with
*		The Controls and Computing Group
*		Accelerator Systems Division
*		Advanced Photon Source
*		Argonne National Laboratory
*
* Modification Log:
* -----------------
* .01	04-29-09 hcf - initialized from drvEnsemble.cc (Aerotech)
* .02	07-07-09 cjb - merged fixes from Ensemble code (drvEnsemble.cc) in R6-4-4
*/


#include <string.h>
#include <math.h>
#include <stdio.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <drvSup.h>
#include "motor.h"
#include "drvSoloist.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

/* Status byte bits */
#define ENABLED_BIT		0x00000001
#define IN_POSITION_BIT	0x00000004
#define IN_MOTION_BIT	0x00000008
#define DIRECTION_BIT	0x00000200
#define HOME_LIMIT_BIT  0x01000000
#define HOME_MARKER_BIT 0x02000000

/* Fault status bits */
#define CW_FAULT_BIT	0x004
#define CCW_FAULT_BIT   0x008

// This can really be any number, because there isn't any theoretical
// restriction on the number of Soloist that can be on a network.
// However, a relatively low number was picked for practical purposes
#define Soloist_NUM_CARDS    10

// Due to the way some of the commands are implemented in the Soloist,
// such as HOME, the command response is not returned until the command
// has actually finished executing. This requires a fairly large timeout
// value. However, even a large value will not guarantee that the response will
// be sent back in the time alloted.
#define TIMEOUT 20.0 /* Command timeout in sec. */

/*----------------debugging-----------------*/
volatile int drvSoloistdebug = 0;
extern "C" {epicsExportAddress(int, drvSoloistdebug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvSoloistdebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int Soloist_num_cards = 0;
int Soloist_num_cmds = 0;

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

struct driver_table Soloist_access =
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
	long (*report) (int);
	long (*init) (void);
} drvSoloist = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvSoloist);}

static struct thread_args targs = {SCAN_RATE, &Soloist_access, 0.0};

/*********************************************************
* Print out driver status report
*********************************************************/
static long report(int level)
{
	int card;

	if (Soloist_num_cards <=0)
		printf("    No Soloist controllers configured.\n");
	else
	{
		for (card = 0; card < Soloist_num_cards; card++)
		{
			struct controller *brdptr = motor_state[card];

			if (brdptr == NULL)
				printf("    Soloist controller %d connection failed.\n", card);
			else
			{
				struct Soloistcontroller *cntrl;

				cntrl = (struct Soloistcontroller *) brdptr->DevicePrivate;
				printf("    Soloist controller %d, port=%s, address=%d, id: %s \n",
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
	if (Soloist_num_cards <= 0)
	{
		Debug(1, "init(): Soloist driver disabled. SoloistSetup() missing from startup script.\n");
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
	struct Soloistcontroller *cntrl;
	struct mess_node *nodeptr;
	register struct mess_info *motor_info;
	double motorData, pfbk;
	bool plusdir, ls_active = false;
	msta_field status;
	int rtn_state, comm_status, axis_status;
	char buff[BUFF_SIZE];

	cntrl = (struct Soloistcontroller *) motor_state[card]->DevicePrivate;
	motor_info = &(motor_state[card]->motor_info[signal]);
	status.All = motor_info->status.All;

	// get the axis status
	sprintf(buff, "AXISSTATUS()");
	send_mess(card, buff, (char) NULL);
	comm_status = recv_mess(card, buff, 1);
	if (comm_status > 0 && buff[0] == ASCII_ACK_CHAR)
	{
		cntrl->status = NORMAL;
		status.Bits.CNTRL_COMM_ERR = 0;
	}
	else if (comm_status <= 0)
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
		goto exit;
	}
	else if (buff[0] != ASCII_ACK_CHAR)
	{
		cntrl->status = COMM_ERR;
		status.Bits.CNTRL_COMM_ERR = 1;
		status.Bits.RA_PROBLEM = 1;
		rtn_state = 1;
		goto exit;
	}

	cntrl->status = NORMAL;
	status.Bits.CNTRL_COMM_ERR = 0;

	// convert to an integer
	axis_status = atoi(&buff[1]);

	nodeptr = motor_info->motor_motion;

	status.Bits.EA_SLIP = 0;
	status.Bits.EA_SLIP_STALL = 0;

	// fill in the status
	status.Bits.RA_DIRECTION = axis_status & DIRECTION_BIT	 ? 1 : 0;
	status.Bits.RA_DONE		 = axis_status & IN_POSITION_BIT ? 1 : 0;
	status.Bits.RA_HOME		 = axis_status & HOME_LIMIT_BIT	 ? 1 : 0;
	status.Bits.EA_POSITION	 = axis_status & ENABLED_BIT	 ? 1 : 0;
	status.Bits.EA_HOME		 = axis_status & HOME_MARKER_BIT ? 1 : 0;
	status.Bits.RA_MOVING	 = axis_status & IN_MOTION_BIT	 ? 1 : 0;

	/* get the axis fault status */
	sprintf(buff, "AXISFAULT()");
	send_mess(card, buff, (char) NULL);
	comm_status = recv_mess(card, buff, 1);
	axis_status = atoi(&buff[1]);
	status.Bits.RA_PLUS_LS   = axis_status & CW_FAULT_BIT ? 1 : 0;
	status.Bits.RA_MINUS_LS  = axis_status & CCW_FAULT_BIT ? 1 : 0;

	plusdir = status.Bits.RA_DIRECTION ? true : false;
	if ((status.Bits.RA_PLUS_LS && plusdir) || (status.Bits.RA_MINUS_LS && !plusdir))
	{
		ls_active = true;
	}

	// get the position
	sprintf(buff, "PFBKPROG()");
	send_mess(card, buff, (char) NULL);
	recv_mess(card, buff, 1);
	if (buff[0] == ASCII_ACK_CHAR)
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

	if (motorData == motor_info->position)
	{
		// only increment the counter if the motor is moving
		if (nodeptr != 0)
		{
			motor_info->no_motion_count++;
		}
	}
	else
	{
		motor_info->position = NINT(motorData);
		if (motor_info->encoder_present == YES)
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
	if ((status.Bits.RA_DONE || ls_active) && nodeptr != 0 && nodeptr->postmsgptr != 0)
	{
		strcpy(buff, nodeptr->postmsgptr);
		send_mess(card, buff, (char) NULL);
		nodeptr->postmsgptr = NULL;
	}

exit:
	motor_info->status.All = status.All;
	return(rtn_state);
}


/*****************************************************/
/* send a message to the Soloist board            */
/* send_mess()                               */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
	struct Soloistcontroller *cntrl;
	int size;
	size_t nwrite;
	char *eos_tok, com_cpy[BUFF_SIZE];
	asynStatus status;

	size = strlen(com);

	if (size > MAX_MSG_SIZE)
	{
		errlogMessage("drvSoloist.c:send_mess(); message size violation.\n");
		return(ERROR);
	}
	else if (size == 0)
	{
		return(OK);
	}

	if (!motor_state[card])
	{
		errlogMessage("drvSoloist.c:send_mess() - invalid card #%d\n");
		return(ERROR);
	}

	if (name != NULL)
	{
		errlogMessage("drvSoloist.c:send_mess() - invalid argument = %s\n");
		return(ERROR);
	}

	Debug(2, "send_mess(): message = %s\n", com);

	// We need to track the number of individual Soloist commands that are
	// being sent, so that we can read back the same number of responses.
	// This is necessary, because the Soloist will send individual responses
	// terminated by the EOS char for each command sent, even if those commands
	// were all sent as part of one motor record command, such as LOAD_POS

	Soloist_num_cmds = 0;
	strcpy(com_cpy, com);
	eos_tok = strtok(com_cpy, ASCII_EOS_STR);
	while (eos_tok != NULL)
	{
		Soloist_num_cmds++;
		eos_tok = strtok(NULL, ASCII_EOS_STR);
	}

	cntrl = (struct Soloistcontroller *) motor_state[card]->DevicePrivate;

	status = pasynOctetSyncIO->write(cntrl->pasynUser, com, size, TIMEOUT, &nwrite);

	if (status != asynSuccess || nwrite <= 0)
	{
		errlogMessage(cntrl->pasynUser->errorMessage);
		return(ERROR);
	}

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
	struct Soloistcontroller *cntrl;
	double timeout = 0;
	size_t nread = 0;
	asynStatus status;
	int eomReason, i;
	char buff[BUFF_SIZE];

	if (!motor_state[card])
	{
		return(ERROR);
	}

	cntrl = (struct Soloistcontroller *) motor_state[card]->DevicePrivate;

	timeout = TIMEOUT;
	com[0] = '\0';
	for (i = 0; i < Soloist_num_cmds; i++)
	{
		status = pasynOctetSyncIO->read(cntrl->pasynUser, buff, BUFF_SIZE, timeout, &nread, &eomReason);

		if (status == asynSuccess && nread > 0)
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
	return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* SoloistSetup()                                     */
/*****************************************************/
RTN_STATUS SoloistSetup(int num_cards,  /* maximum number of controllers in system.  */
						int scan_rate)  /* polling rate - 1/60 sec units.  */
{
	int i;

	if (num_cards < 1 || num_cards > Soloist_NUM_CARDS)
	{
		Soloist_num_cards = Soloist_NUM_CARDS;
	}
	else
	{
		Soloist_num_cards = num_cards;
	}

	// Set motor polling task rate
	if (scan_rate >= 1 && scan_rate <= 60)
	{
		targs.motor_scan_rate = scan_rate;
	}
	else
	{
		targs.motor_scan_rate = SCAN_RATE;
	}

	/*
	* Allocate space for motor_state structures.  Note this must be done
	* before SoloistConfig is called, so it cannot be done in motor_init()
	* This means that we must allocate space for a card without knowing
	* if it really exists, which is not a serious problem
	*/
	motor_state = (struct controller **) malloc(Soloist_num_cards * sizeof(struct controller *));

	for (i = 0; i < Soloist_num_cards; i++)
	{
		motor_state[i] = (struct controller *) NULL;
	}

	return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* SoloistConfig()                                    */
/*****************************************************/
RTN_STATUS SoloistConfig(int card,      /* card being configured */
						 const char *name,   /* asyn port name */
						 int addr)           /* asyn address (GPIB) */
{
	struct Soloistcontroller *cntrl;

	if (card < 0 || card >= Soloist_num_cards)
	{
		return(ERROR);
	}

	motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
	motor_state[card]->DevicePrivate = malloc(sizeof(struct Soloistcontroller));
	cntrl = (struct Soloistcontroller *) motor_state[card]->DevicePrivate;

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
	struct Soloistcontroller *cntrl;
	int card_index, motor_index, status, digits;
	char buff[BUFF_SIZE];
	asynStatus success_rtn;

	// Indicates that the driver is initialized
	initialized = true;

	if (Soloist_num_cards <= 0)
	{
		return(ERROR);
	}

	for (card_index = 0; card_index < Soloist_num_cards; card_index++)
	{
		if (!motor_state[card_index])
		{
			continue;
		}

		brdptr = motor_state[card_index];
		brdptr->cmnd_response = true;
		total_cards = card_index + 1;
		cntrl = (struct Soloistcontroller *)brdptr->DevicePrivate;

		// Initialize communications channel
		success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, cntrl->asyn_address, &cntrl->pasynUser, NULL);

		if (success_rtn == asynSuccess)
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
			} while (!status && retry < 3);

			if (status > 0)
			{
				brdptr->localaddr = (char *) NULL;
				brdptr->motor_in_motion = 0;
				// Read controller ID string
				strcpy(buff, "GETPARM(265)"); //UserString1
				send_mess(card_index, buff, (char) NULL);
				recv_mess(card_index, buff, 1);
				if (buff[0] == ASCII_ACK_CHAR)
				{
					strcpy(brdptr->ident, &buff[1]);
				}
				else
				{
					sprintf(brdptr->ident, "Soloist%d", card_index);
				}

				// Get the number of axes
				brdptr->total_axis = 0;
				for (motor_index = 0; motor_index < 1; motor_index++)  // one motor (axis) per card
				{
					// Does this axis actually exist?
					sprintf(buff, "GETPARM(257)"); //AxisName
					send_mess(card_index, buff, (char) NULL);
					recv_mess(card_index, buff, 1);

					// We know the axis exists if we got an ACK response
					if (buff[0] == ASCII_ACK_CHAR)
					{
						cntrl->axes[motor_index] = 1;
						brdptr->total_axis++;
					}
				}

				for (motor_index = 0; motor_index < 1; motor_index++)  // one motor (axis) per card
				{
					if (cntrl->axes[motor_index])
					{
						struct mess_info *motor_info = &brdptr->motor_info[motor_index];

						motor_info->status.All = 0;
						motor_info->no_motion_count = 0;
						motor_info->encoder_position = 0;
						motor_info->position = 0;
						brdptr->motor_info[motor_index].motor_motion = NULL;

						// Determine if encoder present based on open/closed loop mode.
						sprintf(buff, "GETPARM(58)"); //CfgFbkPosType
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if (buff[0] == ASCII_ACK_CHAR)
						{
							if (atoi(&buff[1]) > 0)
							{
								motor_info->encoder_present = YES;
								motor_info->status.Bits.EA_PRESENT = 1;
							}
						}

						// Determine if gains are supported based on the motor type.
						sprintf(buff, "GETPARM(33)"); //CfgMotType
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if (buff[0] == ASCII_ACK_CHAR)
						{
							if (atoi(&buff[1]) != 3)
							{
								motor_info->pid_present = YES;
								motor_info->status.Bits.GAIN_SUPPORT = 1;
							}
						}

						// Stop all motors
						sprintf(buff, "ABORT");
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);

						// Determine drive resolution
						sprintf(buff, "GETPARM(3)"); //PosScaleFactor
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if (buff[0] == ASCII_ACK_CHAR)
						{
							cntrl->drive_resolution[motor_index] = 1 / fabs(atof(&buff[1]));
						}
						else
						{
							cntrl->drive_resolution[motor_index] = 1;
						}

						digits = (int) -log10(cntrl->drive_resolution[motor_index]) + 2;
						if (digits < 1)
						{
							digits = 1;
						}
						cntrl->res_decpts[motor_index] = digits;

						// Save home preset position
						sprintf(buff, "GETPARM(108)"); //HomeOffset
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if (buff[0] == ASCII_ACK_CHAR)
						{
							cntrl->home_preset[motor_index] = atof(&buff[1]);
						}

						// Determine low limit
						sprintf(buff, "GETPARM(47)"); //ThresholdSoftCCW
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if (buff[0] == ASCII_ACK_CHAR)
						{
							motor_info->low_limit = atof(&buff[1]);
						}

						// Determine high limit
						sprintf(buff, "GETPARM(48)"); //ThresholdSoftCW
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if (buff[0] == ASCII_ACK_CHAR)
						{
							motor_info->high_limit = atof(&buff[1]);
						}

						// Save the HomeDirection parameter
						sprintf(buff, "GETPARM(106)"); //HomeDirection
						send_mess(card_index, buff, (char) NULL);
						recv_mess(card_index, buff, 1);
						if (buff[0] == ASCII_ACK_CHAR)
						{
							cntrl->home_dparam[motor_index] = atoi(&buff[1]);
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

	epicsThreadCreate((char *) "Soloist_motor",
		epicsThreadPriorityMedium,
		epicsThreadGetStackSize(epicsThreadStackMedium),
		(EPICSTHREADFUNC) motor_task, (void *) &targs);

	return(OK);
}

