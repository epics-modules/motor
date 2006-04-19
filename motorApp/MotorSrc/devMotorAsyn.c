/* devMotorAsyn.c */
/* Example device support module */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <devSup.h>
#include <alarm.h>
#include <cantProceed.h> /* !! for callocMustSucceed() */

#include <asynDriver.h>
#include <asynInt32.h>
#include <asynFloat64.h>
#include <asynFloat64Array.h>
#include <asynEpicsUtils.h>

#include "motorRecord.h"
#include "motor.h"
#include "epicsExport.h"
#include "drvMotorAsyn.h"
#include "motor_interface.h"

/*Create the dset for devMotor */
static long init_record(struct motorRecord *);
static CALLBACK_VALUE update_values(struct motorRecord *);
static long start_trans(struct motorRecord *);
static RTN_STATUS build_trans( motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS end_trans(struct motorRecord *);
static void asynCallback(asynUser *);
static void statusCallback(void *, asynUser *, epicsInt32);
static void positionCallback(void *, asynUser *, epicsFloat64);
static void encoderCallback(void *, asynUser *, epicsFloat64);

typedef enum {int32Type, float64Type, float64ArrayType} interfaceType;

struct motor_dset devMotorAsyn={ { 8,
				NULL,
				NULL,
				(DEVSUPFUN) init_record,
                                  NULL },
				update_values,
				start_trans,
				build_trans,
				end_trans };

epicsExportAddress(dset,devMotorAsyn);

typedef struct {
    motorCommand command;
    interfaceType interface;
    int ivalue;
    double dvalue;
} motorAsynMessage;

typedef struct
{
    struct motorRecord * pmr;
    epicsUInt32 status;  /**< bit mask of errors and other binary information. The
			 bit positions are in motor.h */
    epicsInt32 position; /**< Current motor position in motor steps (if not
			 servoing) or demand position (if servoing) */
    epicsInt32 encoder_position; /**< Current motor position in encoder units 
				 (only available if a servo system). */
    motor_cmnd move_cmd;
    double param;
    int needUpdate;
    asynUser *pasynUser;
    asynInt32 *pasynInt32;
    void *asynInt32Pvt;
    asynFloat64 *pasynFloat64;
    void *asynFloat64Pvt;
    asynFloat64Array *pasynFloat64Array;
    void *asynFloat64ArrayPvt;
    void *registrarPvt1;
    void *registrarPvt2;
    void *registrarPvt3;
} motorAsynPvt;



static long init_record(struct motorRecord * pmr )
{
    asynUser *pasynUser;
    char *port, *userParam;
    int signal;
    asynStatus status;
    asynInterface *pasynInterface;
    motorAsynPvt *pPvt;
    double resolution;

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
                  "devMcaAsyn::init_record, %s connectDevice failed to %s\n",
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

    /* Now connect the various callbacks, one for status, position and
       encoder position */
    pasynUser = pasynManager->duplicateAsynUser(pPvt->pasynUser, asynCallback, 0);
    pasynUser->reason = motorStatus;
    status = pPvt->pasynInt32->registerInterruptUser(pPvt->asynInt32Pvt,
						     pasynUser,
						     statusCallback,
						     pPvt,&pPvt->registrarPvt1);
    if(status!=asynSuccess) {
	printf("%s devMotorAsyn::init_record registerInterruptUser %s\n",
	       pmr->name, pasynUser->errorMessage);
    }

    pasynUser = pasynManager->duplicateAsynUser(pPvt->pasynUser, asynCallback, 0);
    pasynUser->reason = motorPosition;
    status = pPvt->pasynFloat64->registerInterruptUser(pPvt->asynFloat64Pvt,
						       pasynUser,
						       positionCallback,
						       pPvt,&pPvt->registrarPvt2);
    if(status!=asynSuccess) {
	printf("%s devMotorAsyn::init_record registerInterruptUser %s\n",
	       pmr->name,pPvt->pasynUser->errorMessage);
    }

    pasynUser = pasynManager->duplicateAsynUser(pPvt->pasynUser, asynCallback, 0);
    pasynUser->reason = motorEncoderPosition;
    status = pPvt->pasynFloat64->registerInterruptUser(pPvt->asynFloat64Pvt,
						       pasynUser,
						       encoderCallback,
						       pPvt,&pPvt->registrarPvt3);
    if(status!=asynSuccess) {
	printf("%s devMotorAsyn::init_record registerInterruptUser %s\n",
	       pmr->name,pPvt->pasynUser->errorMessage);
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
    pasynUser->reason = motorResolution;
    pPvt->pasynFloat64->write(pPvt->asynFloat64Pvt, pasynUser,
			      resolution);
*/
    pasynUser->reason = motorStatus;
    pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser,
			   &pPvt->status);
    pasynUser->reason = motorPosition;
    pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser,
			   &pPvt->position);
    pasynUser->reason = motorEncoderPosition;
    pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser,
			   &pPvt->encoder_position);
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

    if ( pPvt->needUpdate ) {
	pmr->rmp = pPvt->position;
	pmr->rep = pPvt->encoder_position;
	/* pmr->rvel = ptrans->vel; */
	pmr->msta = pPvt->status;
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
    RTN_STATUS status = OK;
    motorAsynPvt *pPvt = (motorAsynPvt *)pmr->dpvt;
    asynUser *pasynUser = pPvt->pasynUser;
    motorAsynMessage *pmsg;
    int need_call=0;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "devMotorAsyn::send_msg: %s command=%d, pact=%d\n",
              pmr->name, command, pmr->pact);

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
    if ((pmr->nsta == COMM_ALARM) || (pmr->stat == COMM_ALARM)) return(-1);

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
	break;
    case SET_PGAIN:
	pmsg->command = motorPgain;
	pmsg->dvalue = *param;
	break;
    case SET_IGAIN:
	pmsg->command = motorIgain;
	pmsg->dvalue = *param;
	break;
    case SET_DGAIN:
	pmsg->command = motorDgain;
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
    case SET_HIGH_LIMIT:
	pmsg->command = motorHighLim;
	pmsg->dvalue = *param;
	break;
    case SET_LOW_LIMIT:
	pmsg->command = motorLowLim;
	pmsg->dvalue = *param;
	break;
    case GET_INFO:
	pmsg->command = motorStatus;
	pmsg->interface = float64ArrayType;
	/* Check this - needUpdate can cause the callback mechanism to get
	   stuck. Must ensure that the record will be processed after this */
	pPvt->needUpdate = 1; 
	break;
    case SET_RESOLUTION:
	pmsg->command = motorResolution;
	pmsg->dvalue = *param;
	break;
    default:
	status = ERROR;
    }
    
    /* Queue asyn request, so we get a callback when driver is ready */
    pasynUser->reason = pmsg->command;
    status = pasynManager->queueRequest(pasynUser, 0, 0);
    if (status != asynSuccess) {
	asynPrint(pasynUser, ASYN_TRACE_ERROR,
		  "devMotorAsyn::send_msg: %s error calling queueRequest, %s\n",
		  pmr->name, pasynUser->errorMessage);
	return(ERROR);
    }
    return(OK);
}

static RTN_STATUS end_trans(struct motorRecord * pmr )
{
  return(OK);
}

static void asynCallback(asynUser *pasynUser)
{
    motorAsynPvt *pPvt = (motorAsynPvt *)pasynUser->userPvt;
    motorRecord *pmr = pPvt->pmr;
    motorAsynMessage *pmsg = pasynUser->userData;
    int status;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "devMotorAsyn::asynCallback: %s command=%d, ivalue=%d, dvalue=%f\n",
              pmr->name, pmsg->command, pmsg->ivalue, pmsg->dvalue);
    pasynUser->reason = pmsg->command;

    switch (pmsg->command) {
    case motorStatus:
	/* Read the current status of the device */
	pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser,
			       &pPvt->status);
	pasynUser->reason = motorPosition;
	pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser,
			       &pPvt->position);
	pasynUser->reason = motorEncoderPosition;
	pPvt->pasynInt32->read(pPvt->asynInt32Pvt, pasynUser,
			       &pPvt->encoder_position);
	scanOnce(pmr);
	break;

    default:
        if (pmsg->interface == int32Type) {
            pPvt->pasynInt32->write(pPvt->asynInt32Pvt, pasynUser,
                                    pmsg->ivalue);
        } else {
            pPvt->pasynFloat64->write(pPvt->asynFloat64Pvt, pasynUser,
                                      pmsg->dvalue);
        }
        break;
    }
    pasynManager->memFree(pmsg, sizeof(*pmsg));
    status = pasynManager->freeAsynUser(pasynUser);
    if (status != asynSuccess) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "devMotorAsyn::asynCallback: %s error in freeAsynUser, %s\n",
                  pmr->name, pasynUser->errorMessage);
    }
}

static void statusCallback(void *drvPvt, asynUser *pasynUser,
			   epicsInt32 value)
{
    motorAsynPvt *pPvt = (motorAsynPvt *)drvPvt;
    motorRecord *pmr = pPvt->pmr;

    asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
        "%s devMotorAsyn::statusCallback new value=%d\n",
        pmr->name, value);

    if (interruptAccept) {
        dbScanLock((dbCommon *)pmr);
        pPvt->status = value;
        dbScanUnlock((dbCommon*)pmr);
        if (!pPvt->needUpdate) {
	    pPvt->needUpdate = 1;
	    scanOnce(pmr);
        }
    } else {
        pPvt->status = value;
        pPvt->needUpdate = 1;
    }
}

static void positionCallback(void *drvPvt, asynUser *pasynUser,
			     epicsFloat64 value)
{
    motorAsynPvt *pPvt = (motorAsynPvt *)drvPvt;
    motorRecord *pmr = pPvt->pmr;

    asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
        "%s devMotorAsyn::positionCallback new value=%f\n",
        pmr->name, value);

    if (interruptAccept) {
        dbScanLock((dbCommon *)pmr);
        pPvt->position = (epicsInt32)floor(value+0.5);
        dbScanUnlock((dbCommon*)pmr);
        if (!pPvt->needUpdate) {
	    pPvt->needUpdate = 1;
	    scanOnce(pmr);
        }
    } else {
        pPvt->position = (epicsInt32)floor(value+0.5);
        pPvt->needUpdate = 1;
    }
}

static void encoderCallback(void *drvPvt, asynUser *pasynUser,
			    epicsFloat64 value)
{
    motorAsynPvt *pPvt = (motorAsynPvt *)drvPvt;
    motorRecord *pmr = pPvt->pmr;

    asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
        "%s devMotorAsyn::encoderCallback new value=%f\n",
        pmr->name, value);

    if (interruptAccept) {
        dbScanLock((dbCommon *)pmr);
        pPvt->encoder_position = (epicsInt32)floor(value+0.5);
        dbScanUnlock((dbCommon*)pmr);
        if (!pPvt->needUpdate) {
   	    pPvt->needUpdate = 1;
	    scanOnce(pmr);
        }
    } else {
        pPvt->encoder_position = (epicsInt32)floor(value+0.5);
        pPvt->needUpdate = 1;
    }
}
