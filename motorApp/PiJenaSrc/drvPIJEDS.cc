/*
FILENAME...	drvPIJEDS.cc
USAGE...	Motor record driver level support for piezosystem jena
	        GmbH & Co. E-516 motor controller.


/*
 *      Original Author: Joe Sullivan
 *      Date: 6/07
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
 *	      The Controls and Automation Group (AT-8)
 *	      Ground Test Accelerator
 *	      Accelerator Technology Division
 *	      Los Alamos National Laboratory
 *
 *      Co-developed with
 *	      The Controls and Computing Group
 *	      Accelerator Systems Division
 *	      Advanced Photon Source
 *	      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01 03/26/07 jps - copied from drvPIC710.cc tested on PI VER "DSP V3.11,MCU V5"
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the PIJEDS must be powered-on when EPICS is first
    booted up.
*/

#include <string.h>
#include <math.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motorRecord.h"
#include "motor.h"
#include "drvPIJEDS.h"
#include "epicsExport.h"

#define GET_IDENT     ""       

#define BUFF_SIZE 100		/* Maximum length of string to/from PIJEDS */

/*----------------debugging-----------------*/
volatile int drvPIJEDSdebug = 0;
extern "C" {epicsExportAddress(int, drvPIJEDSdebug);}
static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < drvPIJEDSdebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}

/* --- Local data. --- */
int PIJEDS_num_cards = 0;
static char *PIJEDS_axis[] = {"0", "1", "2", "3", "4", "5"};

/* Command Information - used by set_status() */
#define EDS_CMNDS_MAX 3
#define EDS_POS    0
#define EDS_STATUS 1
#define EDS_SLEW   2

#define READ_POS      "mess,#"   /* Read position command */
#define READ_STATUS   "stat,#"   /* Read Motor Status */
#define READ_SLEW     "sr,#"     /* Read Slew Velocity V/ms */

static char *EDS_CMNDS[] = {READ_POS, READ_STATUS, READ_SLEW};

/* See fillCmndInfo() */
static struct cmndInfo_struct {
  char sendStr[10];        /* Command String */
  char searchStr[3];       /* Partial command string to check reply */
  int dataIndex;          /* Index into reply for data */
} cmndInfo[EDS_CMNDS_MAX];
    
static unsigned long fdbk_tolerance;       /* Divisor to shift position - for DONE */

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"


/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void fillCmndInfo();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PIJEDS_access =
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
    PIJEDS_axis
};

struct drvPIJEDS_drvet
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPIJEDS = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPIJEDS);}

static struct thread_args targs = {SCAN_RATE, &PIJEDS_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIJEDS_num_cards <=0)
	printf("    No PIJEDS controllers configured.\n");
    else
    {
	for (card = 0; card < PIJEDS_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PIJEDS controller %d connection failed.\n", card);
	    else
	    {
		struct PIJEDScontroller *cntrl;

		cntrl = (struct PIJEDScontroller *) brdptr->DevicePrivate;
		printf("    PIJEDS controller #%d, port=%s, id: %s \n", card,
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
    if (PIJEDS_num_cards <= 0)
    {
	Debug(1, "init(): PIJEDS driver disabled. PIJEDSSetup() missing from startup script.\n");
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
    struct PIJEDScontroller *cntrl;
    struct mess_node *nodeptr;
    struct mess_info *motor_info;
    struct motorRecord *mr;
    /* Message parsing variables */
    struct cmndInfo_struct *pInfo;
    char buff[BUFF_SIZE];
    char *pdata;
    int rtn_state;
    EDS_Status_Reg statusReg;
    epicsInt32 motorData;
    unsigned long posDelta;
    bool plusdir, ls_active, plusLS, minusLS;
    bool readOK; 
    msta_field status;

    cntrl = (struct PIJEDScontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    if (nodeptr != NULL)
	mr = (struct motorRecord *) nodeptr->mrecord;
    else
	mr = NULL;
    status.All = motor_info->status.All;

    recv_mess(card, buff, FLUSH);

    readOK = false;   

    // Read controller status word
    pInfo = &cmndInfo[EDS_STATUS]; 
    send_mess(card, pInfo->sendStr, PIJEDS_axis[signal]);
    if (recv_mess(card, buff, 1) && (pdata=strstr(buff,pInfo->searchStr)))
      {
	//	sscanf(pdata+pInfo->dataIndex, "%d", (int *)&statusReg.All);
	statusReg.All = atoi(pdata+pInfo->dataIndex);
        Debug(2, "set_status(): statusStr: %s, statusReg=0x%x\n", pdata, statusReg.All);	  
	
	// Read motor position 
	pInfo = &cmndInfo[EDS_POS]; 
	send_mess(card, pInfo->sendStr, PIJEDS_axis[signal]);
	if (recv_mess(card, buff, 1) && (pdata=strstr(buff,pInfo->searchStr)))
	  {
	    motorData = NINT(atof(pdata+pInfo->dataIndex) / cntrl->drive_resolution[signal]);
	    Debug(2, "set_status(): posString: %s, posData=%d\n", pdata, motorData);	  
	    readOK = true;
	  }
	else
	  Debug(1, "set_status(): posString: ERROR!\n");	  

      }
    else
         Debug(1, "set_status(): statusString: ERROR!\n");	  


    if (readOK)
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

    /* No direct DONE status on the EDS controller
     * DONE is derived from delta position test */
    status.Bits.RA_DONE = 0;

    status.Bits.RA_HOME = status.Bits.RA_DONE;
    status.Bits.EA_POSITION = (statusReg.Bits.closeLoop) ? 1 : 0;  /* Torgue disabled flag */

    /* No Limit switches */
    ls_active = plusLS = minusLS = false;

    // When comparing feedback position - allow for sloppyness
    posDelta = abs(motorData-motor_info->position);
    Debug(5, "set_status(): deltaPos: %ld\n", posDelta);	  

    /* Reference position is not available so 
     * widen the 'no motion' test because of encoder
     * jitter. */
      
    // if (motorData == motor_info->position)
    if (posDelta < fdbk_tolerance)
    {
        if (motor_info->no_motion_count)
	  {
	    status.Bits.RA_DONE = 1;
	    status.Bits.RA_HOME = status.Bits.RA_DONE;
	  }
	
	if (nodeptr != 0)   /* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	status.Bits.RA_DIRECTION = (motorData >= motor_info->position) ? 1 : 0;
	motor_info->no_motion_count = 0;
    }

    motor_info->position =  motor_info->encoder_position = motorData;


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

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

   // if (!status.Bits.RA_DIRECTION)
   //     motor_info->velocity *= -1;

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
/* send a message to the PIJEDS board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    char *pbuff;
    struct PIJEDScontroller *cntrl;
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
	errlogMessage("drvPIJEDS.cc:send_mess(); message size violation.\n");
	return(ERROR);
    }
    //  Allow empty message strings - returns version info */
    // else if (comsize == 0)  /* Normal exit on empty input message. */
    //   	return(OK);

    if (!motor_state[card])
    {
	errlogPrintf("drvPIJEDS.cc:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    local_buff[0] = (char) NULL;    /* Terminate local buffer. */

    if (name == NULL)
	strcat(local_buff, com);    /* Make a local copy of the string. */
    else
    {
	strcpy(local_buff, com);
	pbuff = strchr(local_buff, '#');
        if (pbuff != NULL)
	  *pbuff = *name;
	else
	  Debug(1, "send_mess(): NAME ERROR: message = %s\n", local_buff);	  
    }

    Debug(3, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct PIJEDScontroller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
			    COMM_TIMEOUT, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the PIJEDS board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct PIJEDScontroller *cntrl;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct PIJEDScontroller *) motor_state[card]->DevicePrivate;

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

    Debug(3, "recv_mess(): message = \"%s\"\n", com);
    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* PIJEDSSetup()                                     */
/*****************************************************/
RTN_STATUS
PIJEDSSetup(int num_cards,  /* maximum number of controllers in system.  */
	    int scan_rate)  /* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PIJEDS_NUM_CARDS)
	PIJEDS_num_cards = PIJEDS_NUM_CARDS;
    else
	PIJEDS_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

    /* 
     * Allocate space for motor_state structures.  Note this must be done
     * before PIJEDSConfig is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(PIJEDS_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PIJEDS_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PIJEDSConfig()                                    */
/*****************************************************/
RTN_STATUS
PIJEDSConfig(int card,	     /* card being configured */
	     const char *name,	 /* asyn port name */
	     int addr)		 /* asyn address (GPIB) */
{
    struct PIJEDScontroller *cntrl;

    if (card < 0 || card >= PIJEDS_num_cards)
	return(ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIJEDScontroller));
    cntrl = (struct PIJEDScontroller *) motor_state[card]->DevicePrivate;

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
    struct PIJEDScontroller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE], *pbuff;
    int total_axis;
    int status;
    int version;
    bool online; 
    asynStatus success_rtn;
    static const char output_terminator[] = EDS_OUT_EOS;
    static const char input_terminator[] = EDS_IN_ETX;

    initialized = true;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIJEDS_num_cards <= 0)
	return(ERROR);

    /* Fill out EDS command string information - used by set_status() */
    fillCmndInfo();

    for (card_index = 0; card_index < PIJEDS_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->ident[0] = (char) NULL;	/* No controller identification message. */
	brdptr->cmnd_response = true;
	total_cards = card_index + 1;
	cntrl = (struct PIJEDScontroller *) brdptr->DevicePrivate;

        status = version = 0;

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

	    /* Assure that Controller is ONLINE */
	    do
	    {
	      online = false;
	      /* Set Controller to ONLINE mode */
	      send_mess(card_index, GET_IDENT, (char) NULL);
	      if ((status = recv_mess(card_index, buff, 1)))
		online = (strstr(buff,"DSM")) ? true : false;
	      else
		retry++;
	    } while (online == false && retry < 3);

	}

	if (success_rtn == asynSuccess && online == true)
	{
	    /* Parse out EDS revision (3 decimal places) and convert to int */
	    if ((pbuff = strchr(buff, 'V')))
	      version = NINT(atof(pbuff+1) * 1000);
	    else
	      version = 0;

	    strcpy(brdptr->ident, buff);
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

	    /* Check for supported EDS versions */
	    if (version >= 1959)
	      cntrl->versionSupport = true;
	    else
	      cntrl->versionSupport = false;

            /* Determine # of axes. Request stage name.  See if it responds */
	    for (total_axis = 0; total_axis < EDS_MAX_MOTORS; total_axis++)
	    {
		send_mess(card_index, READ_POS, PIJEDS_axis[total_axis]);
		status = recv_mess(card_index, buff, 1);
		if (strstr(buff,"not present"))
		    break;
	    }
	    brdptr->total_axis = total_axis;

	    /* fdbk_tolerance = 10^^(MAX_RES - 1) */
	    /* Used in set_status() to calculate DONE/NO_MOTION */
	    fdbk_tolerance = (unsigned long)pow(10.0,(double)(EDS_MAX_RES-1));

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* PIJEDS has DC motor support only */
		motor_info->encoder_present = YES;
		motor_info->status.Bits.EA_PRESENT = 1;
		motor_info->pid_present = NO;
		motor_info->status.Bits.GAIN_SUPPORT = 1;

		cntrl->drive_resolution[motor_index] = 1.0/pow(10.0,(double)EDS_MAX_RES);

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

    epicsThreadCreate((char *) "PIJEDS_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}


static void fillCmndInfo()
{
  int index;
  struct cmndInfo_struct *pInfo = cmndInfo;
  char **pCmndStr = EDS_CMNDS;

  for (index = 0; index < EDS_CMNDS_MAX; index++, pInfo++, pCmndStr++)
    {
    strcpy(pInfo->sendStr,*pCmndStr);           /* Command String */
    strncpy(pInfo->searchStr,*pCmndStr,2);      /* Partial Cmnd to check reply */
    pInfo->dataIndex = strlen(*pCmndStr) + 1; /* Index to reply data */
    }

}

