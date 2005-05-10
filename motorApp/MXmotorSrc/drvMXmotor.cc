/*
FILENAME...	drvMXmotor.cc
USAGE...	Motor record driver level support for MX device driver.

Version:	$Revision: 1.9 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2005-05-10 16:34:35 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 06/15/99
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 */

#include <epicsThread.h>
#include <drvSup.h>

#include "motor.h"
#include "motordrvCom.h"
#include "MXmotor.h"
#include "epicsExport.h"

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int drvMXmotordebug = 0;
	#define Debug(l, f, args...) { if(l<=drvMXmotordebug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

/* Global data. */
int MXmotor_num_cards = 0;
MX_RECORD *MXmotor_record_list;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* Common local function declarations. */

static long report(int level);
static long init();
static void query_done(int, int, struct mess_node *);
static int set_status(int card, int signal);
static RTN_STATUS send_mess(int, char const *, char *);
static int recv_mess(int, char *, int);
static int motor_init();

struct driver_table MXmotor_access =
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
    NULL
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvMXmotor = {2, report, init};

extern "C" {epicsExportAddress(drvet,drvMXmotor);}

static struct thread_args targs = {SCAN_RATE, &MXmotor_access, 0.0};

/*----------------functions-----------------*/

static long report(int level)
{
    int card;

    if (MXmotor_num_cards <= 0)
	printf("    No MX motors configured.\n");
    else
    {
	for (card = 0; card < MXmotor_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    MX motor #%d not found.\n", card);
	    else
		printf("    MX motor #%d\n", card);
	}
    }
    return (0);
}

static long init()
{
    initialized = true;	/* Indicate that driver is initialized. */
    (void) motor_init();
    return ((long) 0);
}


static void query_done(int, int, struct mess_node *)
{
}


RTN_STATUS MXmotorSetup(int max_motors,		/* Maximum number of motors. */
			char const *filename,	/* MX data file. */
			int scan_rate)		/* polling rate - 1/60 sec units */
{
    RTN_STATUS rtnval = OK;
    mx_status_type mx_status;

    if (max_motors > 1000)
	MXmotor_num_cards = 1000;
    else if (max_motors < 0)
	MXmotor_num_cards = 0;
    else
	MXmotor_num_cards = max_motors;
    
    mx_status = mx_setup_database(&MXmotor_record_list, (char *) filename);
    if (mx_status.code != MXE_SUCCESS)
    {
	printf("MXmotorSetup: database setup error.\n");
	return(ERROR);
    }

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

    return(rtnval);
}


/* Initialize all MX. */
static int motor_init()
{
    struct controller *pmotorState;
    int card_index, motor_index;
    int total_encoders = 0, total_axis = 0;

    /* Check for setup */
    if (MXmotor_num_cards <= 0)
    {
	Debug(1, "motor_init: *OMS driver disabled* \n omsSetup() is missing from startup script.\n");
	return (ERROR);
    }

    /* allocate space for total number of motors */
    motor_state = (struct controller **) malloc(MXmotor_num_cards *
						sizeof(struct controller *));

    /* allocate structure space for each motor present */

    total_cards = MXmotor_num_cards;

    for (card_index = 0; card_index < MXmotor_num_cards; card_index++)
    {
	Debug(2, "motor_init: card %d\n", card_index);

	motor_state[card_index] = (struct controller *) malloc(sizeof(struct controller));
	motor_state[card_index]->DevicePrivate = malloc(sizeof(struct MXcontroller));
	pmotorState = motor_state[card_index];
	strcpy(pmotorState->ident, "MXmotor");
	pmotorState->localaddr = (char *) 0;
	pmotorState->motor_in_motion = 0;
	pmotorState->cmnd_response = false;

	Debug(3, "Total axis = %d\n", total_axis);
	pmotorState->total_axis = total_axis = 1;

	for (total_encoders = 0, motor_index = 0; motor_index < total_axis; motor_index++)
	    pmotorState->motor_info[motor_index].encoder_present = NO;

	for (motor_index = 0; motor_index < total_axis; motor_index++)
	{
	    pmotorState->motor_info[motor_index].motor_motion = NULL;
	    pmotorState->motor_info[motor_index].status = 0;
	    pmotorState->motor_info[motor_index].no_motion_count = 0;
	    pmotorState->motor_info[motor_index].encoder_position = 0;
	    pmotorState->motor_info[motor_index].position = 0;

	    if (pmotorState->motor_info[motor_index].encoder_present == YES)
		pmotorState->motor_info[motor_index].status |= EA_PRESENT;
	}
    }

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    Debug(3, "Motors initialized\n");

    epicsThreadCreate((const char *) "MX_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    Debug(3, "Started motor_task\n");
    return (0);
}


static int set_status(int card, int signal)
{
    struct mess_info *motor_info;

    struct controller *brdptr;
    struct MXcontroller *cntrl;
    mx_status_type mx_status;

    int rtn_state;
    bool plusdir, ls_active = false;
    unsigned long status;
    double motorData;

    brdptr = motor_state[card];
    motor_info = &(brdptr->motor_info[signal]);
    cntrl = (struct MXcontroller *) brdptr->DevicePrivate;

    mx_status = mx_motor_get_extended_status(cntrl->MXmotor_record, &motorData,
					     &status);
    if (mx_status.code != MXE_SUCCESS)
	return(rtn_state = ERROR);

    if (status & MXSF_MTR_IS_BUSY )
	motor_info->status &= ~RA_DONE;
    else
	motor_info->status |= RA_DONE;

    if (motorData == motor_info->position)
	motor_info->no_motion_count++;
    else
    {
	epicsInt32 newposition;

	newposition = NINT(motorData);
	if (newposition >= motor_info->position)
	    motor_info->status |= RA_DIRECTION;
	else
	    motor_info->status &= ~RA_DIRECTION;
	motor_info->position = newposition;
	motor_info->no_motion_count = 0;
    }

    plusdir = (motor_info->status & RA_DIRECTION) ? true : false;

    /* Set limit switch error indicators. */
    if (status & MXSF_MTR_POSITIVE_LIMIT_HIT)
    {
	motor_info->status |= RA_PLUS_LS;
	if (plusdir == true)
	    ls_active = true;
    }
    else
	motor_info->status &= ~RA_PLUS_LS;

    if (status & MXSF_MTR_NEGATIVE_LIMIT_HIT)
    {
	motor_info->status |= RA_MINUS_LS;
	if (plusdir == false)
	    ls_active = true;
    }
    else
	motor_info->status &= ~RA_MINUS_LS;

    if (status & MXSF_MTR_ERROR)
	motor_info->status |= RA_PROBLEM;
    else
	motor_info->status &= ~RA_PROBLEM;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
     (motor_info->status & (RA_DONE | RA_PROBLEM))) ? 1 : 0;

    return(rtn_state);
}

static RTN_STATUS send_mess(int card, char const *com, char *name)
{
    struct controller *brdptr;
    struct MXcontroller *cntrl;
    motor_cmnd command;
    mx_status_type mx_status;
    char *cmndptr, *argptr;
    double darg;
    int iarg, flags = MXF_MTR_IGNORE_BACKLASH;

    brdptr = motor_state[card];
    cntrl = (struct MXcontroller *) brdptr->DevicePrivate;
    
    argptr = NULL;
    cmndptr = strtok_r((char *) com, " ", &argptr);
    if (cmndptr == NULL)
	return(OK);
    command = (motor_cmnd) atoi(cmndptr);

    switch (command)
    {
	case MOVE_ABS:
	case MOVE_REL:
	case LOAD_POS:
	case SET_VEL_BASE:
	case SET_VELOCITY:
	    darg = atof(argptr);
	    break;

	case HOME_FOR:
	case HOME_REV:
	    iarg = atoi(argptr);
	    break;
 
	default:
	    break;
    }


    switch (command)
    {
	case MOVE_ABS:
	    mx_status = mx_motor_move_absolute(cntrl->MXmotor_record, darg, flags);
    	    break;
	
	case MOVE_REL:
	    mx_status = mx_motor_move_relative(cntrl->MXmotor_record, darg, flags);
	    break;

	case HOME_FOR:
	case HOME_REV:
	    mx_status = mx_motor_find_home_position(cntrl->MXmotor_record, iarg);
	    break;

	case SET_VEL_BASE:
	    mx_status = mx_motor_set_base_speed(cntrl->MXmotor_record, darg);
	    break;

	case SET_VELOCITY:
	    mx_status = mx_motor_set_speed(cntrl->MXmotor_record, darg);
	    break;

	case SET_ACCEL:
	    break;

	case LOAD_POS:
	    mx_status = mx_motor_set_position(cntrl->MXmotor_record, darg);
	    break;

	case STOP_AXIS:
	    mx_status = mx_motor_soft_abort(cntrl->MXmotor_record);
	    break;
	
	default:
	    break;
    }

    if (mx_status.code != MXE_SUCCESS)
	return(ERROR);

    return(OK);
}


static int recv_mess(int card, char *com, int amount)
{
    return (0);
}


