/* File: devV544.c 		   	*/
/* Version: 1.2a		   	*/
/* Date Last Modified: 2/19/97	*/

/* Device Support Routines for motor */
/*
 *      Original Author: Jim Kowalkowski
 *      Current Author: Joe Sullivan
 *      Date: 11/14/94
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
 * .01  01-18-93	jbk     initialized
 * .02  11-14-94	jps     copy devOms.c and modify to point to vme58 driver
 * .03  ??-??-95	jps     copy devOms58.c and modify to point to Highland driver
 * .04  06-20-96	jps     allow for bumpless-reboot on position
 * .04a 02-19-97	tmm     fixed for EPICS 3.13
 */

#define VERSION 1.3

#include	<vxWorks.h>
#include	<stdioLib.h>
#include	<string.h>
#include	<math.h>
#include        <semLib.h>	/* jps: include for init_record wait */
#include	<alarm.h>
#ifdef __cplusplus
extern "C" {
#include	<callback.h>
#include	<dbAccess.h>
#include	<recSup.h>
}
#else
#include	<callback.h>
#include	<dbAccess.h>
#include	<recSup.h>
#endif
#include	<dbDefs.h>
#include	<dbCommon.h>
#include	<fast_lock.h>
#include	<devSup.h>
#include	<drvSup.h>

#include	"motorRecord.h"
#include	"motor.h"
#include	"drvV544.h"

#define PRIVATE_FUNCTIONS 0	/* normal:1, debug:0 */

#define STATIC

#ifdef NODEBUG
#define Debug(L,FMT,V) ;
#else
#define Debug(L,FMT,V) {  if(L <= devV544debug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,V); } }
#endif

#define NINT(f)	(long)((f)>0 ? (f)+0.5 : (f)-0.5)	/* tmm */

/* ----------------Create the dsets for devV544----------------- */
static long report();
STATIC long v544_init(int after);
STATIC long v544_init_record(struct motorRecord *);
STATIC long get_ioint_info();
STATIC long v544_start_trans(struct motorRecord *);
STATIC long v544_update_values(struct motorRecord *);
STATIC long v544_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC long v544_end_trans(struct motorRecord *);
STATIC void v544_callback(MOTOR_RETURN *);
STATIC void v544_init_callback(MOTOR_RETURN *);

struct motor_dset devV544 =
{
    {8, NULL, v544_init, v544_init_record, NULL},
    v544_update_values,
    v544_start_trans,
    v544_build_trans,
    v544_end_trans
};

/* xxDSET devXxxx={ 5,report,init,init_rec,get_ioint_info,read_write}; */

/* --------------------------- program data --------------------- */

volatile int devV544debug = 0;


struct v544_motor_table
{
    int type;
    uint16_t command;
    int num_parms;
};

static struct v544_motor_table v544_table[] = {
/*	  type      cmd  num_parms */
    {MOTION, CS_MOVE_ABS, 1},	/* MOVE_ABS */
    {MOTION, CS_MOVE_INC, 1},	/* MOVE_REL */
    {MOTION, CS_SEEK_INDX, 1},	/* HOME_FOR */
    {MOTION, CS_SEEK_INDX, 1},	/* HOME_REV */
    {IMMEDIATE, CS_SET_POSITION, 1},	/* LOAD_POS */
    {IMMEDIATE, -1, 1},		/* SET_VELO_BASE  - Not supported */
    {IMMEDIATE, -1, 1},		/* SET_VELO */
    {IMMEDIATE, -1, 1},		/* SET_ACCEL */
    {IMMEDIATE, CS_GO, 0},	/* GO */
    {IMMEDIATE, -1, 2},		/* SET_ENC_RATIO - Not supported */
    {INFO, -1, 0},		/* GET_INFO */
    {MOVE_TERM, CS_IDLE, 0},	/* STOP_AXIS */
    {VELOCITY, -1, 1},		/* JOG */
    {UNDEFINED, 0, 0},		/* SET_PGAIN */
    {UNDEFINED, 0, 0},		/* SET_IGAIN */
    {UNDEFINED, 0, 0},		/* SET_DGAIN */
    {UNDEFINED, 0, 0},		/* ENABLE_TORQUE */
    {UNDEFINED, 0, 0},		/* DISABL_TORQUE */
    {UNDEFINED, 0, 0},		/* PRIMITIVE */
    {UNDEFINED, 0, 0},		/* SET_HIGH_LIMIT */
    {UNDEFINED, 0, 0}		/* SET_LOW_LIMIT */
};

/* status of an axis of a board, one present for each axis of each board*/
struct a_axis
{
/*???     int encoder_present; */
    int in_use;
};

/* the device private data structure of the record */
struct v544_trans
{
    int state;
    FAST_LOCK lock;
    MOTOR_CALL motor_call;
    int callback_changed;
    int32_t motor_pos;
    int32_t encoder_pos;
    int32_t vel;
    unsigned long status;
    SEM_ID initSem;
};

/* the board status structure */
struct v544_stat
{
    int exists;
    int total_axis;
    struct a_axis *v544_axis;
};

STATIC struct v544_stat *v544_cards;
extern struct v544_support v544_access;

#define IDLE_STATE 0
#define BUILD_STATE 1
#define YES 1
#define NO 0

#define SEM_TIMEOUT  50


/* --------------------------- program data --------------------- */

/* initialize device support for V544 stepper motor */
STATIC long v544_init(int after)
{
    MOTOR_CARD_QUERY card_query;
    int i, j;

    Debug(1, "v544_init: entry...\n", 0);
    if (after)
	return (0);

    /* allocate space for total cards in system */
    v544_cards = (struct v544_stat *) malloc(V544_NUM_CARDS *
					     sizeof(struct v544_stat));

    /* for each card ask driver for num of axis supported and name */
    for (i = 0; i < V544_NUM_CARDS; i++)
    {
	(*v544_access.get_card_info) (i, &card_query);

	if (card_query.total_axis == 0)
	    v544_cards[i].exists = NO;
	else
	{
	    v544_cards[i].exists = YES;
	    v544_cards[i].total_axis = card_query.total_axis;
	    Debug(5, "v544_init: calling malloc(card_query...)\n total axis =%d\n",
		v544_cards[i].total_axis);
	    v544_cards[i].v544_axis = (struct a_axis *) malloc(
			     card_query.total_axis * sizeof(struct a_axis));

	    for (j = 0; j < card_query.total_axis; j++)
		v544_cards[i].v544_axis[j].in_use = 0;
	}
    }

    Debug(1, "v544_init: exit...\n", 0);
    return (0);
}

/* initialize a record instance */
STATIC long v544_init_record(struct motorRecord * mr)
{
    int card, signal;
    int initEncoder, initPos;
    struct v544_trans *ptrans;
    MOTOR_AXIS_QUERY axis_query;
    MOTOR_CALL *motor_call;
    double ep_mp[2];		/* encoder pulses, motor pulses */
    int rtnStat;

    Debug(1, "v544_init_record: entry...\n", 0);

    /* allocate space for private field - an v544_trans structure */
    Debug(5, "v544_init_record: calling malloc(...v544_trans...)\n", 0);
    mr->dpvt = (struct v544_trans *) malloc(sizeof(struct v544_trans));

    ptrans = mr->dpvt;
    ptrans->state = IDLE_STATE;
    ptrans->callback_changed = NO;
    FASTLOCKINIT(&ptrans->lock);
    motor_call = &(ptrans->motor_call);

    callbackSetCallback((void (*)(struct callbackPvt *)) v544_callback,
			&(motor_call->callback));
    callbackSetPriority(priorityMedium, &(motor_call->callback));

    /* check if axis already in use, then mark card, axis as in use */

    /* out must be an VME_IO */
    switch (mr->out.type)
    {
    case (VME_IO):
	break;
    default:
	recGblRecordError(S_dev_badBus, (void *) mr,
			  (char *) "devV544 (init_record) Illegal OUT Bus Type");
	return (S_dev_badBus);
    }

    card = mr->out.value.vmeio.card;
    Debug(5, "v544_init_record: card %d\n", card);
    signal = mr->out.value.vmeio.signal;
    Debug(5, "v544_init_record: signal %d\n", signal);

    rtnStat = 0;

    if (card < 0 || card >= V544_NUM_CARDS || v544_cards[card].exists == NO)
    {
	recGblRecordError(S_db_badField, (void *) mr,
			  (char *) "devV544 (init_record) card does not exist!");
	rtnStat = S_db_badField;
    }
    else if (signal < 0 || signal >= v544_cards[card].total_axis)
    {
	recGblRecordError(S_db_badField, (void *) mr,
			  (char *) "devV544 (init_record) signal does not exist!");
	rtnStat = S_db_badField;
    }
    else if (v544_cards[card].v544_axis[signal].in_use == YES)
    {
	recGblRecordError(S_db_badField, (void *) mr,
			  "(char *) devV544 (init_record) motor already in use!");
	rtnStat = S_db_badField;
    }

    if (rtnStat)
    {
	/* Initialize readback fields for simulation */
	mr->msta = RA_PROBLEM;
	mr->res = mr->mres;
	mr->rmp = 0;		/* raw motor pulse count */
	mr->rep = 0;		/* raw encoder pulse count */
	return (rtnStat);
    }
    else
	/* query motor for all info to fill into record */
	(*v544_access.get_axis_info) (card, signal, &axis_query);

    mr->msta = axis_query.status;	/* status info */

    v544_cards[card].v544_axis[signal].in_use = YES;


/*jps: setting the encoder ratio was moved from init_record() remove  callbacks during iocInit */
    /*
     * Set the encoder ratio.  Note this is blatantly device dependent.
     * Determine the number of encoder pulses and the number of motor pulses
     * per engineering unit (EGU).  Send an array containing this information
     * to device support.
     */
    if ((initEncoder = (mr->msta & EA_PRESENT)) && mr->ueip)
    {
	if (fabs(mr->mres) < 1.e-9)
	    mr->mres = 1.;
	if (fabs(mr->eres) < 1.e-9)
	    mr->eres = mr->mres;
	{
	    int m;
	    for (m=10000000; (m > 1) &&
		(fabs(m/mr->eres) > 1.e6 || fabs(m/mr->mres) > 1.e6);
		m /= 10)
		;
	    Debug(5, "oms58_init_record: ER mult = %d\n", m);
	    ep_mp[0] = m / mr->eres;	/* encoder pulses per ...*/
	    ep_mp[1] = m / mr->mres;	/*    motor pulses */
	}
	mr->res = mr->eres;
    }
    else
    {
	ep_mp[0] = 1.;
	ep_mp[1] = 1.;
	mr->res = mr->mres;
    }


    /* Program the device if an initial position is programmed or 
     * an encoder is present. */

    if ((initPos = (mr->dval && mr->res)) || initEncoder)
    {

	/*
	 * Semaphore used to hold initialization until device is programmed -
	 * cleared by callback
	 */
	ptrans->initSem = semBCreate(SEM_Q_FIFO, SEM_EMPTY);

	/*
	 * Switch to special init callback so that record will not be
	 * processed during iocInit
	 */
	callbackSetCallback((void (*)(struct callbackPvt *)) v544_init_callback,
			    &(motor_call->callback));

	if (initEncoder)
	{
	    Debug(7, "v544_init_record: encoder pulses per EGU = %f\n", ep_mp[0]);
	    Debug(7, "v544_init_record: motor pulses per EGU = %f\n", ep_mp[1]);

	
	    v544_start_trans(mr);
	    v544_build_trans(SET_ENC_RATIO, ep_mp, mr);
	    v544_end_trans(mr);
	}

	if (initPos)
        {
	    double setPos = mr->dval / mr->res;

	    Debug(7, "v544_init_record: initial raw position = %f\n", setPos);
	
	    v544_start_trans(mr);
	    v544_build_trans(LOAD_POS, &setPos, mr);
	    v544_end_trans(mr);
        }

	/* Changing positions or encoder ratio may have 
	 * changed the readback value. */
	v544_start_trans(mr);
	v544_build_trans(GET_INFO, NULL, mr);
	v544_end_trans(mr);

	/* Wait for callback w/timeout */
	if ((rtnStat = semTake(ptrans->initSem, SEM_TIMEOUT)) == ERROR)
	{
	    recGblRecordError(S_dev_NoInit, (void *) mr,
			      (char *) "dev_NoInit (init_record: callback2 timeout");
	}
	semDelete(ptrans->initSem);

	/* Restore regular record callback */
	callbackSetCallback((void (*)(struct callbackPvt *)) v544_callback,
			    &(motor_call->callback));
    }


    /* query motor for all info to fill into record */

    (*v544_access.get_axis_info) (card, signal, &axis_query);

    mr->rmp = axis_query.position;	/* raw motor pulse count */
    mr->rep = axis_query.encoder_position;	/* raw encoder pulse count */
    mr->msta = axis_query.status;	/* status info */
    Debug(7, "v544_init_record: rmp = %f\n", mr->rmp);
    Debug(7, "v544_init_record: rep = %f\n", mr->rep);
    Debug(7, "v544_init_record: msta = %d\n", mr->msta);

    Debug(1, "v544_init_record: exit...\n", 0);
    return (0);
}


STATIC long v544_update_values(struct motorRecord * mr)
{
    struct v544_trans *ptrans;
    long rc;

    rc = NOTHING_DONE;
    ptrans = (struct v544_trans *) mr->dpvt;

    FASTLOCK(&ptrans->lock);

    /* raw motor pulse count */
    if (ptrans->callback_changed == YES)
    {
	mr->rmp = ptrans->motor_pos;
	mr->rep = ptrans->encoder_pos;
	mr->rvel = ptrans->vel;
	mr->msta = ptrans->status;
	ptrans->callback_changed = NO;
	rc = CALLBACK_DATA;
    }

    FASTUNLOCK(&ptrans->lock);

    return (rc);
}


/* start building a transaction */
STATIC long v544_start_trans(struct motorRecord * mr)
{
    int card = mr->out.value.vmeio.card;
    int axis = mr->out.value.vmeio.signal;
    struct v544_trans *trans = (struct v544_trans *) mr->dpvt;
    MOTOR_CALL *motor_call;
    CMND_DEF *motor_cmnd;

    if (!mr->dpvt)
	return (S_dev_NoInit);

    motor_call = &(trans->motor_call);

    /*
     * initialize area to device private field to store command that is to be
     * built and mark as command build in progress
     */

    trans->state = BUILD_STATE;
    motor_call->card = card;
    motor_call->signal = axis;
    motor_call->type = UNDEFINED;
    motor_call->precord = (struct dbCommon *) mr;
    motor_call->cmndCnt = 0;
    motor_call->cmndIndx = 0;

    /* Zero command define structure */
    bzero((void *)motor_call->cmndList, sizeof(CMND_DEF)*MAX_LIST_SIZE);

    Debug(3, "v544_start_trans: motor %d command started\n", (card * V544_NUM_CHANNELS) + axis);
    return (0);
}

/* end building a transaction */
STATIC long v544_end_trans(struct motorRecord * mr)
{
    struct v544_trans *trans = (struct v544_trans *) mr->dpvt;
    MOTOR_CALL *motor_call;
    long rc;
    int card = mr->out.value.vmeio.card;
    int axis = mr->out.value.vmeio.signal;


    rc = 0;
    motor_call = &(trans->motor_call);

    switch (trans->state)
    {
    case BUILD_STATE:
	/* shut off command build in process thing */
	trans->state = IDLE_STATE;

	Debug(3, "v544_end_trans: motor %d command send\n", (card * V544_NUM_CHANNELS) + axis);

	rc = (*v544_access.send) (motor_call);
	break;

    case IDLE_STATE:
    default:
	rc = ERROR;
    }

    return (rc);
}

/* add a part to the transaction */
STATIC long v544_build_trans(motor_cmnd command, double *parms, struct motorRecord * mr)
{
    struct v544_trans *trans = (struct v544_trans *) mr->dpvt;
    MOTOR_CALL *motor_call;
    char buffer[20];
    int  parmInt;
    int first_one, i;

    motor_call = &(trans->motor_call);

    if (v544_table[command].type == UNDEFINED)
	return(OK);
    
    if (v544_table[command].type > motor_call->type)
	motor_call->type = v544_table[command].type;

    if (devV544debug >= 4)
      printf("build_trans: v544 cmndCode  %d, cmndParm %12.4f\n",  command, *parms);

    /* concatenate onto the dpvt message field */
    switch (trans->state)
    {
    case BUILD_STATE:
	/* put in command */

	switch (command)
	{
	case MOVE_ABS:
	case MOVE_REL:
	case HOME_FOR:
	case HOME_REV:
	    motor_call->cmndList[motor_call->cmndCnt].cmnd |= v544_table[command].command;
	    motor_call->cmndList[motor_call->cmndCnt].pos = NINT(*parms);
	    break;
	case LOAD_POS:
	    motor_call->cmndList[motor_call->cmndCnt].cmnd |= v544_table[command].command;
	    motor_call->cmndList[motor_call->cmndCnt].pos = NINT(*parms);
	    motor_call->cmndCnt++;
	    break;
	case SET_VEL_BASE:
	    break;
	case SET_VELOCITY:
	    parmInt = NINT(*parms/(double)VEL_SCALE);
	    motor_call->cmndList[motor_call->cmndCnt].vel = parmInt;
	    break;
	case SET_ACCEL:
	    parmInt = NINT(*parms/(double)(ACC_SCALE*ACC_SCALE));
	    if (parmInt == 0)
	      parmInt = 1;
	    motor_call->cmndList[motor_call->cmndCnt].accel = parmInt;
	    break;
	case GO:
	    motor_call->cmndList[motor_call->cmndCnt].cmnd |= v544_table[command].command;
	    motor_call->cmndCnt++;
	    break;
	case SET_ENC_RATIO:
	case GET_INFO:
	    motor_call->cmndCnt++;
	    break;
	case STOP_AXIS:
	    motor_call->cmndList[motor_call->cmndCnt].cmnd |= CS_IDLE;
	    motor_call->cmndList[motor_call->cmndCnt].vel = 0;
	    motor_call->cmndCnt++;
	    break;
	case JOG:
	    if (*parms >= 0)
	    {
		motor_call->cmndList[motor_call->cmndCnt].cmnd |= CS_RUN_CW;

		parmInt = NINT(*parms/(double)VEL_SCALE);
		motor_call->cmndList[motor_call->cmndCnt].vel = parmInt;
	    }
	    else
	    {
		motor_call->cmndList[motor_call->cmndCnt].cmnd |= CS_RUN_CCW;
		parmInt = -1*NINT(*parms/(double)VEL_SCALE);
		motor_call->cmndList[motor_call->cmndCnt].vel = parmInt;
	    }
	    motor_call->cmndCnt++;
	    break;
	default:
	    break;
	}
	break;
    case IDLE_STATE:
    default:
	return ERROR;
    }

    return (0);
}

/* callback from V544 driver for motor in motion */
STATIC void v544_callback(MOTOR_RETURN * motor_return)
{
    struct motorRecord *mr = (struct motorRecord *) motor_return->precord;
    struct rset *prset = (struct rset *) (motor_return->precord->rset);
    struct v544_trans *ptrans;

    Debug(5, "v544_callback: entry\n", 0);
    ptrans = (struct v544_trans *) mr->dpvt;

    Debug(6, "v544_callback: FASTLOCK\n", 0);
    FASTLOCK(&ptrans->lock);

    /* raw motor pulse count */
    Debug(6, "v544_callback: update ptrans\n", 0);
    ptrans->callback_changed = YES;
    ptrans->motor_pos = motor_return->position;
    /* raw encoder pulse count */
    ptrans->encoder_pos = motor_return->encoder_position;
    /* raw motor velocity */
    ptrans->vel = motor_return->velocity*VEL_SCALE;
    ptrans->status = motor_return->status;	/* status */

    if (devV544debug >= 4)
      printf("v544 callback: pos %d, enc %d, vel %d, status =0x%04x\n",
	     ptrans->motor_pos, ptrans->encoder_pos,ptrans->vel, ptrans->status);

    Debug(6, "v544_callback: FASTUNLOCK\n", 0);
    FASTUNLOCK(&ptrans->lock);

    /* free the return data buffer */
    Debug(6, "v544_callback: free MOTOR_RETURN\n", 0);
    (*v544_access.free) (motor_return);

    Debug(6, "v544_callback: dbScanLock\n", 0);
    dbScanLock((struct dbCommon *) mr);
    Debug(6, "v544_callback: calling process\n", 0);
    (*prset->process) ((struct dbCommon *) mr);
    Debug(6, "v544_callback: dbScanUnlock\n", 0);
    dbScanUnlock((struct dbCommon *) mr);
    Debug(5, "v544_callback: exit..\n", 0);
    return;
}


/* callback from V544 driver for init_record  - duplicate of
 * the v544_callback without the call to process the record */
STATIC void v544_init_callback(MOTOR_RETURN * motor_return)
{
    struct motorRecord *mr = (struct motorRecord *) motor_return->precord;
    struct rset *prset = (struct rset *) (motor_return->precord->rset);
    struct v544_trans *ptrans;

    Debug(5, "v544_init_callback: entry\n", 0);
    ptrans = (struct v544_trans *) mr->dpvt;

    Debug(6, "v544_init_callback: FASTLOCK\n", 0);
    FASTLOCK(&ptrans->lock);

    /* raw motor pulse count */
    Debug(6, "v544_init_callback: update ptrans\n", 0);
    ptrans->callback_changed = YES;
    ptrans->motor_pos = motor_return->position;
    /* raw encoder pulse count */
    ptrans->encoder_pos = motor_return->encoder_position;
    /* raw motor velocity */
    ptrans->vel = motor_return->velocity;

    ptrans->status = motor_return->status;	/* status */

    /* free the return data buffer */
    Debug(6, "v544_init_callback: free MOTOR_RETURN\n", 0);
    (*v544_access.free) (motor_return);

    /* Continue record init. */
    Debug(6, "v544_init_callback: Continue sig.\n", 0);
    semGive(ptrans->initSem);

    Debug(6, "v544_init_callback: FASTUNLOCK\n", 0);
    FASTUNLOCK(&ptrans->lock);

    Debug(5, "v544_init_callback: exit\n", 0);
    return;
}
