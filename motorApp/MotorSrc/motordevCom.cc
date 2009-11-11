/*
FILENAME: motordevCom.cc
USAGE... This file contains device functions that are common to all motor
    record device support modules.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Jim Kowalkowski
 *      Current Author: Joe Sullivan
 *      Date: 01/18/93
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
 * .01  01-18-93 jbk initialized
 *      ...
 * .03  03-19-96 tmm v1.10: modified encoder-ratio calculation
 * .04  11-26-96 jps allow for bumpless-reboot on position
 * .04a 02-19-97 tmm fixed for EPICS 3.13
 * .05  06/13/03 rls Ported to R3.14.
 * .06  02/03/04 rls Initialize PID parameters from motor_init_record_com().
 * .07  06/07/05 rls Use RDBD as threshold for controller's position takes
 *                   precedence over the save/restore value at initialization.
 * .08  10/18/05 rls Use MAX_TIMEOUT.
 * .09  02/27/07 rls Bug fix in motor_init_record_com() for logic that
 *                   determines precedence between controller or save/restore
 *                   motor position at boot-up; negative controller positions
 *                   were not handled correctly.
 * .10  10/17/07 rls Raised the precedence of the INIT string for controllers
 *                   (PI C-848) that require an INIT string primitive before a
 *                   LOAD_POS can be executed.
 * .11  0214/08  rls Post RVEL changes.
 */


#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <callback.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <dbEvent.h>
#include <devSup.h>

#include "motorRecord.h"
#include "motor.h"
#include "motordrvCom.h"

#define   epicsExportSharedSymbols
#include <shareLib.h>
#include "motordevCom.h"

static void motor_callback(struct mess_node * motor_return);
static void motor_init_callback(struct mess_node * motor_return);

/* Command set used by record support.  WARNING! this must match
   "motor_cmnds" in motor.h .
*/

epicsShareFunc long motor_init_com(int after, int brdcnt, struct driver_table *tabptr,
              struct board_stat ***sptr)        /* Initialize motor record device support. */
{
    MOTOR_CARD_QUERY card_query;
    int card, motor;

    if (after == 0)
    {
        /* allocate space for maximum possible cards in system */
        *sptr = (struct board_stat **) malloc(brdcnt * sizeof(struct board_stat *));

        /* for each card ask driver for num of axis supported and name */
        for (card = 0; card < brdcnt; card++)
        {
            struct board_stat *brdptr;

            brdptr = (*sptr)[card] = (struct board_stat *) malloc(sizeof(struct board_stat));
            (tabptr->get_card_info) (card, &card_query, tabptr);
            if (card_query.total_axis == 0)
                brdptr->exists = NO;
            else
            {
                brdptr->exists = YES;
                brdptr->name = card_query.card_name;
                brdptr->total_axis = card_query.total_axis;
                brdptr->axis_stat = (struct axis_stat *) malloc(
                                 card_query.total_axis * sizeof(struct axis_stat));
    
                for (motor = 0; motor < card_query.total_axis; motor++)
                   brdptr->axis_stat[motor].in_use = false;
            }
        }
    }
    return (0);
}


/*
FUNCTION... motor_init_record_com - initialize a record instance.

SYNOPSIS... motor_init_record_com(mr      - motor record pointer,
                                  brdcnt  - controller board count,
                                  tabptr  - driver table pointer,
                                  sptr    - controller board status pointer)
LOGIC...
    Allocate memory for motor command transaction data structure (struct motor_trans)
        and point the Device Private record field to it.
    Initialize the motor command transaction data structure.
    ...
    ...
    ...
    Error check "card" index, "signal" index and motor "in_use" indicator.
    IF error detected.
        Set the MSTA PROBLEM bit true.
        Set RMP <- REP <- 0.
        ERROR RETURN.
    ENDIF
    
    Get motor information - call get_axis_info() via driver table.
    Update MSTA.
    Set axis assigned to a motor record indicator true.

    Set Initialize encoder indicator based on (MSTA indicates an encoder is
        present, AND, UEIP set to YES).

    IF Initialize encoder indicator is true.
        ...
        ...
    ELSE
        Set local encoder ratio to unity.
    ENDIF

    Set Initialize position indicator based on (|DVAL| > RDBD, AND, MRES != 0,
        AND, the above |"get_axis_info()" position| < RDBD) [NOTE: |controller
        position| >= RDBD takes precedence over save/restore position].
    Set Command Primitive Initialization string indicator based on (non-NULL "init"
        pointer, AND, non-zero string length.

    IF (Initialize position, OR, encoder, OR, string indicators are true)
        Create initialization semaphore.
        ...
        ...
        IF Initialize encoder indicator is true.
            Send Set Encoder Ratio command to controller.
        ENDIF
        IF Initialize position indicator is true.
            Send Load Position command to controller.
        ENDIF
        IF Command Primitive Initialization string present.
            Send Initialization string to controller.
        ENDIF
        
        Send Get Info command to controller.
        ...
        ...
    ENDIF

    Get motor information - call get_axis_info() via driver table.
    Set RMP, REP and MSTA based on updated motor information.
    NORMAL RETURN.
*/

epicsShareFunc long
motor_init_record_com(struct motorRecord *mr, int brdcnt, struct driver_table *tabptr,
                      struct board_stat *sptr[])
{
    struct motor_dset *pdset = (struct motor_dset *) (mr->dset);
    struct board_stat *brdptr;
    int card, signal;
    bool initEncoder, initPos, initString, initPID;
    struct motor_trans *ptrans;
    MOTOR_AXIS_QUERY axis_query;
    struct mess_node *motor_call;
    double ep_mp[2];            /* encoder pulses, motor pulses */
    int rtnStat;
    msta_field msta;

    /* allocate space for private field - an motor_trans structure */
    mr->dpvt = (struct motor_trans *) malloc(sizeof(struct motor_trans));

    /* Initialize the private field. */
    ptrans = (struct motor_trans *) mr->dpvt;
    ptrans->state = IDLE_STATE;
    ptrans->callback_changed = NO;
    ptrans->tabptr = tabptr;
    ptrans->dpm = false;

    /* Semaphore on private to record field data transfers */
    ptrans->lock = new epicsEvent(epicsEventFull);
                                                        
    motor_call = &(ptrans->motor_call);

    callbackSetCallback((void (*)(struct callbackPvt *)) motor_callback,
                        &(motor_call->callback));
    callbackSetPriority(priorityMedium, &(motor_call->callback));

    /* check if axis already in use, then mark card, axis as in use */
    if (mr->out.type != VME_IO) /* out must be VME_IO. */
    {
        recGblRecordError(S_dev_badBus, (void *) mr,
            (char *) "motor_init_record_com(): Illegal OUT Bus Type");
        return(S_dev_badBus);
    }

    card = mr->out.value.vmeio.card;
    brdptr = sptr[card];
    signal = mr->out.value.vmeio.signal;
    rtnStat = 0;

    if (card < 0 || card >= brdcnt || brdptr->exists == NO)
    {
        recGblRecordError(S_db_badField, (void *) mr,
            (char *) "motor_init_record_com(): card does not exist!");
        rtnStat = S_db_badField;
    }
    else if (signal < 0 || signal >= brdptr->total_axis)
    {
        recGblRecordError(S_db_badField, (void *) mr,
            (char *) "motor_init_record_com(): signal does not exist!");
        rtnStat = S_db_badField;
    }
    else if (brdptr->axis_stat[signal].in_use == YES)
    {
        recGblRecordError(S_db_badField, (void *) mr,
            (char *) "motor_init_record_com(): motor already in use!");
        rtnStat = S_db_badField;
    }

    if (rtnStat)
    {
        /* Initialize readback fields for simulation */
        msta.All = 0;
        msta.Bits.RA_PROBLEM = 1;
        mr->msta = msta.All;
        mr->rmp = 0;            /* raw motor pulse count */
        mr->rep = 0;            /* raw encoder pulse count */
        return(rtnStat);
    }
        
    /* query motor for all info to fill into record */
    (tabptr->get_axis_info) (card, signal, &axis_query, tabptr);
    msta.All = mr->msta = axis_query.status.All;        /* status info */
    brdptr->axis_stat[signal].in_use = true;

/*jps: setting the encoder ratio was moved from init_record() remove callbacks during iocInit */
    /*
     * Set the encoder ratio.  Note this is blatantly device dependent.
     * Determine the number of encoder pulses and the number of motor pulses
     * per engineering unit (EGU).  Send an array containing this information
     * to device support.
     */
    initEncoder = (msta.Bits.EA_PRESENT && mr->ueip) ? true : false;
    if (initEncoder == true)
    {
        if (fabs(mr->mres) < 1.e-9)
            mr->mres = 1.;
        if (fabs(mr->eres) < 1.e-9)
            mr->eres = mr->mres;
        {
            int m;
            for (m = 10000000; (m > 1) && (fabs(m / mr->eres) > 1.e6 ||
                        fabs(m / mr->mres) > 1.e6); m /= 10);
            ep_mp[0] = m / mr->eres;    /* encoder pulses per ... */
            ep_mp[1] = m / mr->mres;    /* motor pulses */
        }
    }
    else
    {
        ep_mp[0] = 1.;
        ep_mp[1] = 1.;
    }

    initPos = (fabs(mr->dval) > mr->rdbd && mr->mres != 0 &&
               fabs(axis_query.position * mr->mres) < mr->rdbd)
               ? true : false;
    /* Test for command primitive initialization string. */
    initString = (mr->init != NULL && strlen(mr->init)) ? true : false;
    /* Test for PID support. */
    initPID = (msta.Bits.GAIN_SUPPORT) ? true : false;

    /* Program the device if an encoder is present */
    if (initPos == true || initEncoder == true || initString == true || initPID)
    {
        /* Semaphore used to hold initialization until device is programmed - cleared by callback. */
        ptrans->initSem = new epicsEvent(epicsEventEmpty);

        /* Switch to special init callback so that record will not be processed during iocInit. */
        callbackSetCallback((void (*)(struct callbackPvt *)) motor_init_callback,
                            &(motor_call->callback));

        if (initString == true)
        {
            (*pdset->start_trans)(mr);
            (*pdset->build_trans)(PRIMITIVE, NULL, mr);
            (*pdset->end_trans)(mr);
        }
        
        if (initEncoder == true)
        {
            (*pdset->start_trans)(mr);
            (*pdset->build_trans)(SET_ENC_RATIO, ep_mp, mr);
            (*pdset->end_trans)(mr);
        }

        if (initPos == true)
        {
            double setPos = mr->dval / mr->mres;

            (*pdset->start_trans)(mr);
            (*pdset->build_trans)(LOAD_POS, &setPos, mr);
            (*pdset->end_trans)(mr);
        }

        if (initPID == true)
        {
            double pidcoef;

            if ((pidcoef = mr->pcof) > 0.0)
            {
                (*pdset->start_trans)(mr);
                (*pdset->build_trans)(SET_PGAIN, &pidcoef, mr);
                (*pdset->end_trans)(mr);
            }

            if ((pidcoef = mr->icof) > 0.0)
            {
                (*pdset->start_trans)(mr);
                (*pdset->build_trans)(SET_IGAIN, &pidcoef, mr);
                (*pdset->end_trans)(mr);
            }

            if ((pidcoef = mr->dcof) > 0.0)
            {
                (*pdset->start_trans)(mr);
                (*pdset->build_trans)(SET_DGAIN, &pidcoef, mr);
                (*pdset->end_trans)(mr);
            }
        }

        /* Changing encoder ratio may have changed the readback value. */
        (*pdset->start_trans)(mr);
        (*pdset->build_trans)(GET_INFO, NULL, mr);
        (*pdset->end_trans)(mr);

        /* Wait for callback w/timeout */
        if (ptrans->initSem->wait(MAX_TIMEOUT) == FALSE)
            recGblRecordError(S_dev_NoInit, (void *) mr,
                (char *) "dev_NoInit (init_record_com: callback2 timeout");
        delete(ptrans->initSem);

        /* Restore regular record callback */
        callbackSetCallback((void (*)(struct callbackPvt *)) motor_callback,
                            &(motor_call->callback));
    }

    /* query motor for all info to fill into record */

    (tabptr->get_axis_info) (card, signal, &axis_query, tabptr);

    mr->rmp = axis_query.position;      /* raw motor pulse count */
    mr->rep = axis_query.encoder_position;      /* raw encoder pulse count */
    mr->msta = axis_query.status.All;   /* status info */
    return(OK);
}

/*
FUNCTION... long motor_update_values(struct motorRecord *)
USAGE... Update the following motor record fields with the latest driver data:
        RMP  - Raw Motor Position.
        REP  - Raw Encoder Position.
        RVEL - Raw Velocity.
        MSTA - Motor Status.
NOTES... This function MUST BE reentrant.
*/

epicsShareFunc CALLBACK_VALUE motor_update_values(struct motorRecord * mr)
{
    struct motor_trans *ptrans;
    CALLBACK_VALUE rc;

    rc = NOTHING_DONE;
    ptrans = (struct motor_trans *) mr->dpvt;

    ptrans->lock->wait();

    /* raw motor pulse count */
    if (ptrans->callback_changed == YES)
    {
        mr->rmp = ptrans->motor_pos;
        mr->rep = ptrans->encoder_pos;
        if (mr->rvel != ptrans->vel)
        {
            mr->rvel = ptrans->vel;
            db_post_events(mr, &mr->rvel, DBE_VAL_LOG);
        }
        mr->msta = ptrans->status.All;
        ptrans->callback_changed = NO;
        rc = CALLBACK_DATA;
    }

    /* load event for next transfer */
    ptrans->lock->signal();

    return (rc);
}


/*
FUNCTION... long motor_start_trans_com(struct motorRecord *, struct board_stat **)
USAGE... Start building a transaction.
NOTES... This function MUST BE reentrant.
*/

epicsShareFunc long motor_start_trans_com(struct motorRecord *mr, struct board_stat **sptr)
{
    int card = mr->out.value.vmeio.card;
    int axis = mr->out.value.vmeio.signal;
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;

    if (!mr->dpvt)
        return (S_dev_NoInit);

    motor_call = &(trans->motor_call);

    /* initialize area to device private field to store command that is to be
     * built and mark as command build in progress. */

    trans->state = BUILD_STATE;
    motor_call->card = card;
    motor_call->signal = axis;
    motor_call->type = UNDEFINED;
    motor_call->mrecord = (struct dbCommon *) mr;
    motor_call->message[0] = (char) NULL;
    motor_call->postmsgptr = (char) NULL;
    motor_call->termstring = (char) NULL;
    
    return (0);
}


/*
FUNCTION... long motor_end_trans_com(struct motorRecord *, struct driver_table *)
USAGE... Finish building a transaction.
NOTES... This function MUST BE reentrant.
*/

epicsShareFunc RTN_STATUS motor_end_trans_com(struct motorRecord *mr, struct driver_table *tabptr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    RTN_STATUS rc;

    rc = OK;
    motor_call = &(trans->motor_call);
    if ((*trans->tabptr->card_array)[motor_call->card] == NULL)
    {
        msta_field msta;

        /* If the controller does not exits, then set "done moving"
         * and communication error TRUE.
         */
        mr->dmov = TRUE;
        db_post_events(mr, &mr->dmov, DBE_VAL_LOG);
        msta.All = mr->msta;
        msta.Bits.CNTRL_COMM_ERR = 1;
        mr->msta = msta.All;
        return(rc = ERROR);
    }

    switch (trans->state)
    {
        case BUILD_STATE:
            /* shut off command build in process thing */
            trans->state = IDLE_STATE;
            rc = (*tabptr->send) (motor_call, tabptr);
            break;

        case IDLE_STATE:
        default:
            rc = ERROR;
    }
    return (rc);
}


/*
FUNCTION... static void motor_callback(struct mess_node * motor_return)
USAGE... Callback from  driver for a motor in motion.
NOTES... This function MUST BE reentrant.
*/

static void motor_callback(struct mess_node * motor_return)
{
    struct motorRecord *mr = (struct motorRecord *) motor_return->mrecord;
    struct motor_trans *ptrans;

    ptrans = (struct motor_trans *) mr->dpvt;

    ptrans->lock->wait();

    ptrans->callback_changed = YES;
    ptrans->motor_pos = motor_return->position;
    ptrans->encoder_pos = motor_return->encoder_position;
    ptrans->vel = motor_return->velocity;
    ptrans->status = motor_return->status;

    /* load event for next transfer */
    ptrans->lock->signal();

    /* free the return data buffer */
    (ptrans->tabptr->free) (motor_return, ptrans->tabptr);

    dbScanLock((struct dbCommon *) mr);
    dbProcess((struct dbCommon *) mr);  /* Process the motor record. */
    dbScanUnlock((struct dbCommon *) mr);
    return;
}

/* callback from OMS driver for init_record  - duplicate of
 * the motor_callback without the call to process the record */
static void motor_init_callback(struct mess_node * motor_return)
{
    struct motorRecord *mr = (struct motorRecord *) motor_return->mrecord;
    struct motor_trans *ptrans;

    ptrans = (struct motor_trans *) mr->dpvt;

    ptrans->lock->wait();

    ptrans->callback_changed = YES;
    ptrans->motor_pos = motor_return->position;
    ptrans->encoder_pos = motor_return->encoder_position;
    ptrans->vel = motor_return->velocity;
    ptrans->status = motor_return->status;

    /* free the return data buffer */
    (ptrans->tabptr->free) (motor_return, ptrans->tabptr);
    ptrans->initSem->signal();

    /* load event for next transfer */
    ptrans->lock->signal();
    return;
}

