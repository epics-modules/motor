/*
FILENAME...	drvPIC862.cc
USAGE...	Motor record driver level support for Physik Instrumente (PI)
	GmbH & Co. C-862 motor controller.
*/
/*
 *      Original Author: Ron Sluiter
 *      Current Author: Mohan Ramanathan
 *      Date: 09/04/2006
 *
 * NOTES
 * -----
 * - This driver works with both the C-862 and C-863.
 *
 * Modification Log:
 * -----------------
 * .00  09/05/2006  mr  copied from drvPIC848.cc
 * .01  09/25/2006 rls  Set LS error indicator based on LS active high/low
 *                      configuration indicator and the state of the LS.
 * .02  09/28/2006 rls  C-862 drops transmitted characters on move command;
 *                      need "status update delay".
 */


/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the PIC862 must be powered-on when EPICS is first
    booted up.
*/

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motorRecord.h"
#include "motor.h"
#include "drvPIC862.h"
#include "epicsExport.h"

#define GET_IDENT 0x01

#define PIC862_NUM_CARDS	8
#define MAX_AXES		1
#define BUFF_SIZE 100		/* Maximum length of string to/from PIC862 */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	#define Debug(l, f, args...) { if(l<=drvPIC862debug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvPIC862debug = 0;
extern "C" {epicsExportAddress(int, drvPIC862debug);}

/* --- Local data. --- */
int PIC862_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"


/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PIC862_access =
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
} drvPIC862 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPIC862);}

static struct thread_args targs = {SCAN_RATE, &PIC862_access, 0.017};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIC862_num_cards <=0)
	printf("    No PIC862 controllers configured.\n");
    else
    {
	for (card = 0; card < PIC862_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PIC862 controller %d connection failed.\n", card);
	    else
	    {
		struct PIC862controller *cntrl;

		cntrl = (struct PIC862controller *) brdptr->DevicePrivate;
		printf("    PIC862 controller #%d, port=%s, id: %s \n", card,
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
     * and hence requires that the drvGPIB have already been initialized.
     * That cannot be guaranteed, so we need to call motor_init from device
     * support
     */
    /* Check for setup */
    if (PIC862_num_cards <= 0)
    {
	Debug(1, "init(): PIC862 driver disabled. PIC862Setup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/********************************************************************************
*										*
* FUNCTION NAME: set_status							*
*										*
* LOGIC:									*
*   Initialize.									*
*   Send "Moving Status" query.							*
*   Read response.								*
*   IF normal response to query.						*
*	Set communication status to NORMAL.					*
*   ELSE									*
*	IF communication status is NORMAL.					*
*	    Set communication status to RETRY.					*
*	    NORMAL EXIT.							*
*	ELSE									*
*	    Set communication status error.					*
*	    ERROR EXIT.								*
*	ENDIF									*
*   ENDIF									*
*										*
*   IF "Moving Status" indicates any motion (i.e. status != 0).			*
*	Clear "Done Moving" status bit.						*
*   ELSE									*
*	Set "Done Moving" status bit.						*
*   ENDIF									*
*										*
*   										*
********************************************************************************/

static int set_status(int card, int signal)
{
    struct PIC862controller *cntrl;
    struct mess_node *nodeptr;
    struct mess_info *motor_info;
    struct motorRecord *mr;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    C862_Status_Reg1 mstat1;
    C862_Status_Reg2 mstat2;
    C862_Status_Reg3 mstat3;
    C862_Status_Reg4 mstat4;
    C862_Status_Reg5 mstat5;
    epicsUInt16 dev_sts1, dev_sts2, dev_sts3, dev_sts4, dev_sts5, dev_sts6;
    
    int rtn_state, convert_cnt, charcnt;
    epicsInt32 motorData;
    bool plusdir, ls_active = false, plusLS, minusLS, LSactiveH;
    msta_field status;

    cntrl = (struct PIC862controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    if (nodeptr != NULL)
	mr = (struct motorRecord *) nodeptr->mrecord;
    else
	mr = NULL;
    status.All = motor_info->status.All;

    if (cntrl->status != NORMAL)
	charcnt = recv_mess(card, buff, FLUSH);

    send_mess(card, "TS", (char) NULL);		/*  Tell Status */
    charcnt = recv_mess(card, buff, 1);
    if (charcnt > 18)
	convert_cnt = sscanf(buff, "S:%2hx %2hx %2hx %2hx %2hx %2hx\n", 
		            &dev_sts1,&dev_sts2,&dev_sts3,&dev_sts4,&dev_sts5,&dev_sts6);

    if (charcnt > 18 && convert_cnt == 6)
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

    mstat1.All = dev_sts1;
    mstat2.All = dev_sts2;
    mstat3.All = dev_sts3;
    mstat4.All = dev_sts4;
    mstat5.All = dev_sts5;
   
    status.Bits.RA_DONE = (mstat1.Bits.trty_done) ? 1 : 0;
    status.Bits.EA_POSITION = (mstat1.Bits.motor_off) ? 0 : 1;
    status.Bits.RA_DIRECTION = (mstat3.Bits.mvdir_pol) ? 1 : 0;

    /* Set +/- LS indicator based on limit switch active high/low
       configuration and the state of the limits switch. */

    LSactiveH = (mstat4.Bits.lmt_high) ? true : false;
    if (LSactiveH == true)
    {
        plusLS  = mstat5.Bits.plus_ls;
        minusLS = mstat5.Bits.minus_ls;
    }
    else
    {
        plusLS  = !mstat5.Bits.plus_ls;
        minusLS = !mstat5.Bits.minus_ls;
    }


   /* Parse motor position */
    send_mess(card, "TP", (char) NULL);  /*  Tell Position */
    recv_mess(card, buff, 1);
    motorData = NINT(atof(&buff[2]));
     
    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)   /* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	motor_info->position =  motor_info->encoder_position = motorData;
	motor_info->no_motion_count = 0;
    }

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

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
	status.Bits.RA_MINUS_LS	= 0;

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
	send_mess(card, buff, (char) NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the PIC862 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    struct PIC862controller *cntrl;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
	errlogMessage("drvPIC862.cc:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (comsize == 0)  /* Normal exit on empty input message. */
	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvPIC862.cc:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    local_buff[0] = (char) NULL;    /* Terminate local buffer. */

    /*  this device deos not have axis info and so name is ignored!  */

    strcat(local_buff, com);    /* Make a local copy of the string. */
    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct PIC862controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
			    COMM_TIMEOUT, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the PIC862 board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct PIC862controller *cntrl;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct PIC862controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	pasynOctetSyncIO->flush(cntrl->pasynUser);
    else
	status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
					COMM_TIMEOUT, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
	com[0] = '\0';
	nread = 0;
    }
    else
        com[nread - 1] = '\0'; /* Strip traling CR. */

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* PIC862Setup()                                     */
/*****************************************************/
RTN_STATUS
PIC862Setup(int num_cards,  /* maximum number of controllers in system.  */
	    int scan_rate)  /* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PIC862_NUM_CARDS)
	PIC862_num_cards = PIC862_NUM_CARDS;
    else
	PIC862_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

    /* 
     * Allocate space for motor_state structures.  Note this must be done
     * before PIC862Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(PIC862_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PIC862_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PIC862Config()                                    */
/*****************************************************/
RTN_STATUS
PIC862Config(int card,	     /* card being configured */
	     const char *name,	 /* asyn port name */
	     int addr)		 /* asyn address ( device address for daisy chaining */
{
    struct PIC862controller *cntrl;

    if (card < 0 || card >= PIC862_num_cards)
	return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIC862controller));
    cntrl = (struct PIC862controller *) motor_state[card]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
    cntrl->asyn_address = addr;
    return(OK);
}


/*****************************************************/
/* initialize all software and hardware		     */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()			                     */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct PIC862controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int status;
    asynStatus success_rtn;
    static const char output_terminator[] = "\r";
    static const char  input_terminator[] = "\n\03";

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIC862_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < PIC862_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->ident[0] = (char) NULL;	/* No controller identification message. */
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct PIC862controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
						&cntrl->pasynUser, NULL);

	if (success_rtn == asynSuccess)
	{
	    int retry = 0;

	    pasynOctetSyncIO->setOutputEos(cntrl->pasynUser, output_terminator,
					   strlen(output_terminator));
	    pasynOctetSyncIO->setInputEos(cntrl->pasynUser, input_terminator,
					  strlen(input_terminator));

	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    pasynOctetSyncIO->flush(cntrl->pasynUser);
	    
	    /*  To start communicating with the device requires to enable the device
	    	To do this send "0x01" and a singel letter address (0-F)
	    	To make sure we talk ask the status with "TB" command  for the address
	    	It replies "B:000x"  where x is 0-F
	        The command "VE" provides a complete Identification string if we need.
	    */
            

	    do
	    {
                sprintf(buff,"\001%1XVE", cntrl->asyn_address);
                send_mess(card_index, buff, (char) NULL);
		status = recv_mess(card_index, buff, 1);
		retry++;
	    } while (status == 0 && retry < 3);
	}

	if (success_rtn == asynSuccess && status > 0)
	{
	    strcpy(brdptr->ident, &buff[0]);
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

            /* number of axes is always one.*/
	    brdptr->total_axis = 1;
	    motor_index = 0;

		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* PIC862 has DC motor support only */
		motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = YES;
		motor_info->status.Bits.GAIN_SUPPORT = 1;


		set_status(card_index, motor_index);  /* Read status of each motor */
	}
	else
	    motor_state[card_index] = (struct controller *) NULL;
    }

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "PIC862_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

