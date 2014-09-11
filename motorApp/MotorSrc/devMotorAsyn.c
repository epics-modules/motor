/* 
 * devMotorAsyn.c
 * 
 * Motor record common Asyn device support layer
 * 
 * Copyright (C) 2005-6 Peter Denison, Diamond Light Source
 *
 * This software is distributed subject to the EPICS Open Licence, which can
 * be found at http://www.aps.anl.gov/epics/licence/open.php
 * 
 * Notwithstanding the above, explicit permission is granted for APS to 
 * redistribute this software.
 *
 * Version: $Revision: 1.32 $
 * Modified by: $Author: rivers $
 * Last Modified: $Date: 2009-09-01 14:05:38 $
 *
 * Original Author: Peter Denison
 * Current Author: Peter Denison
 *
 * Modification Log:
 * -----------------
 * .01 2009-04-15 rls
 * Added logic to asynCallback() to prevent moveRequestPending left nonzero
 * after motor record LOAD_POS command before dbScanLockOK is true (i.e., from
 * save/restore at boot-up).
 * Eliminated compiler warnings.
 *
 * .02 2009-04-29 MRP
 * Fix for motor simulator stuck in Moving state after multiple LOAD_POS
 * commands to the same position; set needUpdate = 1 in asynCallback() before
 * dbProcess.
 * 
 * .03 2013-01-02 rls
 * Support for motor record raw actual velocity (RVEL).
 *
 * .04 2014-08-05 MRP
 * Fix for manual set position after IOC startup. The encoder ratio was not being set at IOC startup
 * which meant that set position failed for Asyn drivers. We now always set the encoder ratio
 * at the beginning of the init_controller function.
 * 
 * .05 2014-09-11 RLS
 * Moved CA posting of changes to the RMP, REP and RVEL fields from motor record to update_values().
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <errlog.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsEvent.h>
#include <cantProceed.h> /* !! for callocMustSucceed() */
#include <dbEvent.h>

#include <asynDriver.h>
#include <asynInt32.h>
#include <asynFloat64.h>
#include <asynDrvUser.h>
#include <asynFloat64Array.h>
#include <asynGenericPointer.h>
#include <asynEpicsUtils.h>

#include "motorRecord.h"
#include "motor.h"
#include "epicsExport.h"
#include "asynMotorController.h"
#include "motor_interface.h"

/*Create the dset for devMotor */
static long init( int after );
static long init_record(struct motorRecord *);
static CALLBACK_VALUE update_values(struct motorRecord *);
static long start_trans(struct motorRecord *);
static RTN_STATUS build_trans( motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS end_trans(struct motorRecord *);
static void asynCallback(asynUser *);
static void statusCallback(void *, asynUser *, void *);

typedef enum {int32Type, float64Type, float64ArrayType} interfaceType;

struct motor_dset devMotorAsyn={ 
    {
         8,
         NULL,
         (DEVSUPFUN) init,
         (DEVSUPFUN) init_record,
         NULL 
    },
    update_values,
    start_trans,
    build_trans,
    end_trans
};

epicsExportAddress(dset,devMotorAsyn);

/* Note, we define these commands here.  These are not pasynUser->reason, they are
 * an index into those reasons returned from driver */
typedef enum motorCommand {
    motorMoveAbs,
    motorMoveRel,
    motorMoveVel,
    motorHome,
    motorStop,
    motorVelocity,
    motorVelBase,
    motorAccel,
    motorPosition,
    motorResolution,
    motorEncRatio,
    motorPGain,
    motorIGain,
    motorDGain,
    motorHighLimit,
    motorLowLimit,
    motorSetClosedLoop,
    motorStatus,
    motorUpdateStatus,
    lastMotorCommand
} motorCommand;
#define NUM_MOTOR_COMMANDS lastMotorCommand

typedef struct {
    motorCommand command;
    interfaceType interface;
    int ivalue;
    double dvalue;
} motorAsynMessage;

typedef struct
{
    struct motorRecord * pmr;
    int moveRequestPending;
    struct MotorStatus status;
    motorCommand move_cmd;
    double param;
    int needUpdate;
    asynUser *pasynUser;
    asynInt32 *pasynInt32;
    void *asynInt32Pvt;
    asynFloat64 *pasynFloat64;
    void *asynFloat64Pvt;
    asynFloat64Array *pasynFloat64Array;
    void *asynFloat64ArrayPvt;
    asynDrvUser *pasynDrvUser;
    void *asynDrvUserPvt;
    asynGenericPointer *pasynGenericPointer;
    void *asynGenericPointerPvt;
    void *registrarPvt;
    epicsEventId initEvent;
    int driverReasons[NUM_MOTOR_COMMANDS];
} motorAsynPvt;



/* The init routine is used to set a flag to indicate that it is OK to call dbScanLock */
static int dbScanLockOK = 0;
static long init( int after )
{
    dbScanLockOK = (after!=0);
    return 0;
}

static void init_controller(struct motorRecord *pmr, asynUser *pasynUser )
{
    /* This routine is copied out of the old motordevCom and initialises the controller
       based on the record values. I think most of it should be transferred to init_record
       which is one reason why I have separated it into another routine */
    motorAsynPvt *pPvt = (motorAsynPvt *)pmr->dpvt;
    double position = pPvt->status.position;
    double rdbd = (fabs(pmr->rdbd) < fabs(pmr->mres) ? fabs(pmr->mres) : fabs(pmr->rdbd) );
    double encRatio[2] = {pmr->mres, pmr->eres};

    /*Before setting position, set the correct encoder ratio.*/
    start_trans(pmr);
    build_trans(SET_ENC_RATIO, encRatio, pmr);
    end_trans(pmr);

    if ((fabs(pmr->dval) > rdbd && pmr->mres != 0) &&
        (fabs(position * pmr->mres) < rdbd))
    {
        double setPos = pmr->dval / pmr->mres;
        epicsEventId initEvent = epicsEventCreate( epicsEventEmpty );

        pPvt->initEvent = initEvent;

        start_trans(pmr);
        build_trans(LOAD_POS, &setPos, pmr);
        end_trans(pmr);

        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "devMotorAsyn::init_controller, %s set position to %f\n",
                  pmr->name, setPos );

        if ( initEvent )
        {
            epicsEventMustWait(initEvent);
            epicsEventDestroy(initEvent);
            pPvt->initEvent = 0;
        }
    }
    else
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "devMotorAsyn::init_controller, %s setting of position not required, position=%f, mres=%f, dval=%f, rdbd=%f",
                  pmr->name, position, pmr->mres, pmr->dval, rdbd );

}

static long findDrvInfo(motorRecord *pmotor, asynUser *pasynUser, char *drvInfoString, int command)
{
    motorAsynPvt *pPvt = (motorAsynPvt *)pmotor->dpvt;

    /* Look up the pasynUser->reason */
    if (pPvt->pasynDrvUser->create(pPvt->asynDrvUserPvt, pasynUser, drvInfoString, NULL, NULL) != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::findDrvInfo, %s drvUserCreate failed for %s\n",
                  pmotor->name, drvInfoString);
        return(-1);
    }
    pPvt->driverReasons[command] = pasynUser->reason;
    return(0);
}

static long init_record(struct motorRecord * pmr )
{
    asynUser *pasynUser;
    char *port, *userParam;
    int signal;
    asynStatus status;
    asynInterface *pasynInterface;
    motorAsynPvt *pPvt;
    /*    double resolution;*/

    /* Allocate motorAsynPvt private structure */
    pPvt = callocMustSucceed(1, sizeof(motorAsynPvt), "devMotorAsyn init_record()");

    /* Create asynUser */
    pasynUser = pasynManager->createAsynUser(asynCallback, 0);
    pasynUser->userPvt = pPvt;
    pPvt->pasynUser = pasynUser;
    pPvt->pmr = pmr;
    pmr->dpvt = pPvt;

    status = pasynEpicsUtils->parseLink(pasynUser, &pmr->out,
                                        &port, &signal, &userParam);
    if (status != asynSuccess) {
        errlogPrintf("devMotorAsyn::init_record %s bad link %s\n",
                     pmr->name, pasynUser->errorMessage);
        goto bad;
    }

    /* Connect to device */
    status = pasynManager->connectDevice(pasynUser, port, signal);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record, %s connectDevice failed to %s\n",
                  pmr->name, port);
        goto bad;
    }

    /* Get the asynInt32 interface */
    pasynInterface = pasynManager->findInterface(pasynUser, asynInt32Type, 1);
    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record, %s find int32 interface failed\n",
                  pmr->name);
        goto bad;
    }
    pPvt->pasynInt32 = (asynInt32 *)pasynInterface->pinterface;
    pPvt->asynInt32Pvt = pasynInterface->drvPvt;

    /* Get the asynFloat64 interface */
    pasynInterface = pasynManager->findInterface(pasynUser, asynFloat64Type, 1);    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record, %s find float64 interface failed\n",
                  pmr->name);
        goto bad;
    }
    pPvt->pasynFloat64 = (asynFloat64 *)pasynInterface->pinterface;
    pPvt->asynFloat64Pvt = pasynInterface->drvPvt;

    /* Get the asynDrvUser interface */
    pasynInterface = pasynManager->findInterface(pasynUser, asynDrvUserType, 1);    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record, %s find drvUser interface failed\n",
                  pmr->name);
        goto bad;
    }
    pPvt->pasynDrvUser = (asynDrvUser *)pasynInterface->pinterface;
    pPvt->asynDrvUserPvt = pasynInterface->drvPvt;

    /* Now that we have the drvUser interface get pasynUser->reason for each command */
    if (findDrvInfo(pmr, pasynUser, motorMoveRelString,                motorMoveRel)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorMoveAbsString,                motorMoveAbs)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorMoveVelString,                motorMoveVel)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorHomeString,                   motorHome)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorStopString,                   motorStop)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorVelocityString,               motorVelocity)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorVelBaseString,                motorVelBase)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorAccelString,                  motorAccel)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorPositionString,               motorPosition)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorResolutionString,             motorResolution)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorEncoderRatioString,           motorEncRatio)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorPGainString,                  motorPGain)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorIGainString,                  motorIGain)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorDGainString,                  motorDGain)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorHighLimitString,              motorHighLimit)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorLowLimitString,               motorLowLimit)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorClosedLoopString,             motorSetClosedLoop)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorStatusString,                 motorStatus)) goto bad;
    if (findDrvInfo(pmr, pasynUser, motorUpdateStatusString,           motorUpdateStatus)) goto bad;
    
    /* Get the asynFloat64Array interface */
    pasynInterface = pasynManager->findInterface(pasynUser,
                                                 asynFloat64ArrayType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record, %s find float64Array interface failed\n",
                  pmr->name);
        goto bad;
    }
    pPvt->pasynFloat64Array = (asynFloat64Array *)pasynInterface->pinterface;
    pPvt->asynFloat64ArrayPvt = pasynInterface->drvPvt;

    /* Get the asynGenericPointer interface */
    pasynInterface = pasynManager->findInterface(pasynUser,
                                                 asynGenericPointerType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record, %s find genericPointer interface failed\n",
                  pmr->name);
        goto bad;
    }
    pPvt->pasynGenericPointer = (asynGenericPointer *)pasynInterface->pinterface;
    pPvt->asynGenericPointerPvt = pasynInterface->drvPvt;

    /* Now connect the callback, to the Generic Pointer interface, which passes MotorStatus structure */
    pasynUser = pasynManager->duplicateAsynUser(pPvt->pasynUser, asynCallback, 0);
    pasynUser->reason = pPvt->driverReasons[motorStatus];
    status = pPvt->pasynGenericPointer->registerInterruptUser(pPvt->asynGenericPointerPvt,
                               pasynUser,
                               statusCallback,
                               pPvt,
                               &pPvt->registrarPvt);
    if(status!=asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record %s devMotorAsyn::init_record registerInterruptUser failed, error=%s\n",
                  pmr->name, pasynUser->errorMessage);
    }

    /* Initiate calls to get the initial motor parameters
       Have to do it the long-winded way, because before iocInit, none of the
       locks or scan queues are initialised, so calls to scanOnce(),
       dbScanLock() etc will fail. */
    pasynUser = pasynManager->duplicateAsynUser(pPvt->pasynUser, asynCallback, 0);

    /* Send the motor resolution to the driver.  This should be done in the record
     * in the future ? */
/*  DON'T DO THIS FOR NOW.  THE NUMBER CAN COME TOO LATE TO BE OF USE TO THE DRIVER
    resolution = pmr->mres;
    pasynUser->reason = pPvt->driverReasons[motorResolution];
    pPvt->pasynFloat64->write(pPvt->asynFloat64Pvt, pasynUser,
                  resolution);
*/
    pasynUser->reason = pPvt->driverReasons[motorStatus];
    status = pPvt->pasynGenericPointer->read(pPvt->asynGenericPointerPvt, pasynUser,
                      (void *)&pPvt->status);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::init_record: %s pasynGenericPointer->read returned %s", 
                  pmr->name, pasynUser->errorMessage);
    }

    /* We must get the first set of status values from the controller before
     * the initial setting of position. Otherwise we won't be able to decide
     * whether or not to write new position values to the controller.
     */
    init_controller(pmr, pasynUser);
    /* Do not need to manually retrieve the new status values, as if they are
     * set, a callback will be generated
     */

    /* Finally, indicate to the motor record that these values can be used. */
    pasynManager->freeAsynUser(pasynUser);
    pPvt->needUpdate = 1;
    
    return(0);
bad:
    pmr->pact=1;
    return(0);
}



CALLBACK_VALUE update_values(struct motorRecord * pmr)
{
    motorAsynPvt * pPvt = (motorAsynPvt *) pmr->dpvt;
    CALLBACK_VALUE rc;

    rc = NOTHING_DONE;

    asynPrint(pPvt->pasynUser, ASYN_TRACEIO_DEVICE,
        "%s devMotorAsyn::update_values, needUpdate=%d\n",
        pmr->name, pPvt->needUpdate);
    if ( pPvt->needUpdate )
    {
        epicsInt32 rawvalue;

        rawvalue = (epicsInt32)floor(pPvt->status.position + 0.5);
        if (pmr->rmp != rawvalue)
        {
            pmr->rmp = rawvalue;
            db_post_events(pmr, &pmr->rmp, DBE_VAL_LOG);
        }

        rawvalue = (epicsInt32)floor(pPvt->status.encoderPosition + 0.5);
        if (pmr->rep != rawvalue)
        {
            pmr->rep = rawvalue;
            db_post_events(pmr, &pmr->rep, DBE_VAL_LOG);
        }

        /* Don't post MSTA changes here; motor record's process() function does efficent MSTA posting. */
        pmr->msta = pPvt->status.status;

        rawvalue = (epicsInt32)floor(pPvt->status.velocity);
        if (pmr->rvel != rawvalue)
        {
            pmr->rvel = rawvalue;
            db_post_events(pmr, &pmr->rvel, DBE_VAL_LOG);
        }

        rc = CALLBACK_DATA;
        pPvt->needUpdate = 0;
    }
    return (rc);
}

static long start_trans(struct motorRecord * pmr )
{
    return(OK);
}

static RTN_STATUS build_trans( motor_cmnd command, 
                   double * param,
                   struct motorRecord * pmr )
{
    RTN_STATUS rtnind = OK;
    asynStatus status;
    motorAsynPvt *pPvt = (motorAsynPvt *)pmr->dpvt;
    asynUser *pasynUser = pPvt->pasynUser;
    motorAsynMessage *pmsg;
    int need_call=0;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "devMotorAsyn::build_trans: %s motor_cmnd=%d, pact=%d, value=%f\n",
              pmr->name, command, pmr->pact, param ? *param : 0.0);

    /* These primitives don't cause a call to the Asyn layer */
    switch ( command ) {
        case MOVE_ABS:
            pPvt->move_cmd = motorMoveAbs;
            pPvt->param = *param;
            break;
        case MOVE_REL:
            pPvt->move_cmd = motorMoveRel;
            pPvt->param = *param;
            break;
        case HOME_FOR:
            pPvt->move_cmd = motorHome;
            pPvt->param = 1;
            break;
        case HOME_REV:
            pPvt->move_cmd = motorHome;
            pPvt->param = 0;
            break;
        default:
            need_call = 1;
    }

    /* Decide whether to quit here */
    if (!need_call) {
        return (OK);
    }

    /* If we are already in COMM_ALARM then this server is not reachable,
     * return */
    if ((pmr->nsta == COMM_ALARM) || (pmr->stat == COMM_ALARM))
        return(ERROR);

   /* Make a copy of asynUser.  This is needed because we can have multiple
    * requests queued.  It will be freed in the callback */
    pasynUser = pasynManager->duplicateAsynUser(pasynUser, asynCallback, 0);
    pmsg = pasynManager->memMalloc(sizeof *pmsg);
    pmsg->ivalue=0;
    pmsg->dvalue=0.;
    pmsg->interface = float64Type;
    pasynUser->userData = pmsg;
 
    switch (command) {
        case LOAD_POS:
            pmsg->command = motorPosition;
            pmsg->dvalue = *param;
            pPvt->moveRequestPending++;
            break;
        case SET_VEL_BASE:
            pmsg->command = motorVelBase;
            pmsg->dvalue = *param;
            break;
        case SET_VELOCITY:
            pmsg->command = motorVelocity;
            pmsg->dvalue = *param;
            break;
        case SET_ACCEL:
            pmsg->command = motorAccel;
            pmsg->dvalue = *param;
            break;
        case GO:
            pmsg->command = pPvt->move_cmd;
            pmsg->dvalue = pPvt->param;
            pPvt->move_cmd = -1;
            pPvt->moveRequestPending++;
            /* Do we need to set needUpdate and schedule a process here? */
            /* or can we always guarantee to get at least one callback? */
            /* Do we really need the callback? I assume so */
            break;
        case SET_ENC_RATIO:
            pmsg->command = motorEncRatio;
            pmsg->dvalue = param[0]/param[1];
            break;
        case STOP_AXIS:
            pmsg->command = motorStop;
            pmsg->interface = int32Type;
            break;
        case JOG:
        case JOG_VELOCITY:
            pmsg->command = motorMoveVel;
            pmsg->dvalue = *param;
            pPvt->moveRequestPending++;
            break;
        case SET_PGAIN:
            pmsg->command = motorPGain;
            pmsg->dvalue = *param;
            break;
        case SET_IGAIN:
            pmsg->command = motorIGain;
            pmsg->dvalue = *param;
            break;
        case SET_DGAIN:
            pmsg->command = motorDGain;
            pmsg->dvalue = *param;
            break;
        case ENABLE_TORQUE:
            pmsg->command = motorSetClosedLoop;
            pmsg->ivalue = 1;
            pmsg->interface = int32Type;
            break;
        case DISABL_TORQUE:
            pmsg->command = motorSetClosedLoop;
            pmsg->ivalue = 0;
            pmsg->interface = int32Type;
            break;
        case PRIMITIVE:
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::build_trans: %s: PRIMITIVE no longer supported\n",
                  pmr->name);
            return(ERROR);
        case SET_HIGH_LIMIT:
            pmsg->command = motorHighLimit;
            pmsg->dvalue = *param;
            break;
        case SET_LOW_LIMIT:
            pmsg->command = motorLowLimit;
            pmsg->dvalue = *param;
            break;
        case GET_INFO:
            pmsg->command = motorUpdateStatus;
            pmsg->interface = int32Type;
            break;
        case SET_RESOLUTION:
            pmsg->command = motorResolution;
            pmsg->dvalue = *param;
            break;
        default:
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::build_trans: %s: motor command %d not recognised\n",
                  pmr->name, command);
            return(ERROR);
    }

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
        "devAsynMotor::build_trans: calling queueRequest, pmsg=%p, sizeof(*pmsg)=%d"
        "pmsg->command=%d, pmsg->interface=%d, pmsg->dvalue=%f\n",
        pmsg, (int)sizeof(*pmsg), pmsg->command, pmsg->interface, pmsg->dvalue);   

    /* Queue asyn request, so we get a callback when driver is ready */
    pasynUser->reason = pPvt->driverReasons[pmsg->command];
    status = pasynManager->queueRequest(pasynUser, 0, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "devMotorAsyn::build_trans: %s error calling queueRequest, %s\n",
              pmr->name, pasynUser->errorMessage);
        rtnind = ERROR;
    }
    return(rtnind);
}

static RTN_STATUS end_trans(struct motorRecord * pmr )
{
  return(OK);
}

/**
 * Called once the request comes off the Asyn internal queue.
 *
 * The request is still "on its way down" at this point
 */
static void asynCallback(asynUser *pasynUser)
{
    motorAsynPvt *pPvt = (motorAsynPvt *)pasynUser->userPvt;
    motorRecord *pmr = pPvt->pmr;
    motorAsynMessage *pmsg = pasynUser->userData;
    int status;
    int commandIsMove = 0;

    pasynUser->reason = pPvt->driverReasons[pmsg->command];
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "devMotorAsyn::asynCallback: %s pmsg=%p, sizeof(*pmsg)=%d, pmsg->command=%d,"
              "pmsg->interface=%d, pmsg->ivalue=%d, pmsg->dvalue=%f, pasynUser->reason=%d\n",
              pmr->name, pmsg, (int)sizeof(*pmsg), pmsg->command, 
              pmsg->interface, pmsg->ivalue, pmsg->dvalue, pasynUser->reason);

    switch (pmsg->command) {
        case motorStatus:
            /* Read the current status of the device */
            status = pPvt->pasynGenericPointer->read(pPvt->asynGenericPointerPvt,
                                  pasynUser,
                                  (void *)&pPvt->status);
            if (status != asynSuccess) {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                          "devMotorAsyn::asynCallback: %s pasynGenericPointer->read returned %s\n", 
                          pmr->name, pasynUser->errorMessage);
            }
            break;

        case motorUpdateStatus:
            status = pPvt->pasynInt32->write(pPvt->asynInt32Pvt, pasynUser,
                                             pmsg->ivalue);
            break;

        case motorMoveAbs:
        case motorMoveRel:
        case motorHome:
        case motorPosition:
        case motorMoveVel:
        commandIsMove = 1;
        /* Intentional fall-through */
        default:
            if (pmsg->interface == int32Type) {
                status = pPvt->pasynInt32->write(pPvt->asynInt32Pvt, pasynUser,
                             pmsg->ivalue);
            } else {
                status = pPvt->pasynFloat64->write(pPvt->asynFloat64Pvt, pasynUser,
                               pmsg->dvalue);
            }
            if (status != asynSuccess) {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                          "devMotorAsyn::asynCallback: %s pasyn{Float64,Int32}->write returned %s\n", 
                          pmr->name, pasynUser->errorMessage);
            }
            break;
    }

    if (dbScanLockOK) { /* effectively if iocInit has completed */
        dbScanLock((dbCommon *)pmr);
        if (commandIsMove) {
            pPvt->moveRequestPending--;
            if (!pPvt->moveRequestPending) {
                pPvt->needUpdate = 1;
                dbProcess((dbCommon*)pmr);
            }
        }
        dbScanUnlock((dbCommon *)pmr);
    }
    else if (pmsg->command == motorPosition)
        pPvt->moveRequestPending = 0;

    pasynManager->memFree(pmsg, sizeof(*pmsg));
    status = pasynManager->freeAsynUser(pasynUser);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::asynCallback: %s error in freeAsynUser, %s\n",
                  pmr->name, pasynUser->errorMessage);
    }

    if ( pPvt->initEvent && pmsg->command == motorPosition) {
        epicsEventSignal( pPvt->initEvent );
    }
}

/**
 * True callback to notify that controller status has changed.
 */
static void statusCallback(void *drvPvt, asynUser *pasynUser,
               void *pValue)
{
    motorAsynPvt *pPvt = (motorAsynPvt *)drvPvt;
    motorRecord *pmr = pPvt->pmr;
    MotorStatus *value = (MotorStatus *)pValue;

    asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
              "%s devMotorAsyn::statusCallback new value=[p:%f,e:%f,s:%x] %c%c\n",
              pmr->name, value->position, value->encoderPosition, value->status,
              pPvt->needUpdate ? 'N':' ', 
              pPvt->moveRequestPending ? 'P':' ');

    if (dbScanLockOK) {
        dbScanLock((dbCommon *)pmr);
        memcpy(&pPvt->status, value, sizeof(struct MotorStatus));
        if (!pPvt->moveRequestPending) {
        pPvt->needUpdate = 1;
        /* pmr->rset->process((dbCommon*)pmr); */
            dbProcess((dbCommon*)pmr);
        }
        dbScanUnlock((dbCommon*)pmr);
    } else {
        memcpy(&pPvt->status, value, sizeof(struct MotorStatus));
        pPvt->needUpdate = 1;
    }
}

