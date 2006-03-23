/* drvMotorAsyn.c

    Derived from ip330 driver from GSE-CARS
    Original Authors: Jim Kowalkowski, Mark Rivers, Joe Sullivan, and Marty Kraimer
********************COPYRIGHT NOTIFICATION**********************************
This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
****************************************************************************

    22-Sep-2005  Peter Denison
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef vxWorks
#include <taskLib.h>
#include <intLib.h>
#endif

#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsMessageQueue.h>
#include <errlog.h>
#include <cantProceed.h> /* !! for callocMustSucceed() */

#include <asynDriver.h>
#include <asynInt32.h>
#include <asynFloat64.h>
#include <asynFloat64Array.h>
#include <asynDrvUser.h>

#include <drvSup.h>
#include <registryDriverSupport.h>

#include "drvMotorAsyn.h"
#include "motor_interface.h"

/* Message queue size */
#define MAX_MESSAGES 100

typedef struct {
    motorCommand command;
    char *commandString;
} motorCommandStruct;

static motorCommandStruct motorCommands[] = {
    {motorMoveRel,    "MOVE_REL"},
    {motorMoveAbs,    "MOVE_ABS"},
    {motorMoveVel,    "MOVE_VEL"},
    {motorHome,       "HOME"},
    {motorStop,       "STOP_AXIS"},
    {motorVelocity,   "VELOCITY"},
    {motorVelBase,    "VEL_BASE"},
    {motorAccel,      "ACCEL"},
    {motorPosition,   "POSITION"},
    {motorEncRatio,   "ENC_RATIO"},
    {motorPgain,      "PGAIN"},
    {motorIgain,      "IGAIN"},
    {motorDgain,      "DGAIN"},
    {motorHighLim,    "HIGH_LIMIT"},
    {motorLowLim,     "LOW_LIMIT"},
    {motorSetClosedLoop, "SET_CLOSED_LOOP"},
    {motorStatus,     "STATUS"},
};

typedef enum{typeInt32, typeFloat64, typeFloat64Array} dataType;

struct drvmotorPvt;

typedef struct drvmotorAxisPvt {
    AXIS_HDL axis;
    int num;
    /* Current parameters for move commands */
    double max_velocity;
    double min_velocity;
    double accel;
    /* Received status */
    epicsInt32 status;
    struct drvmotorPvt *pPvt;
} drvmotorAxisPvt;

typedef struct drvmotorPvt {
    char *portName;
    motorAxisDrvSET_t *drvset;
    int card;
    drvmotorAxisPvt *axisData;
    /* Housekeeping */
    epicsMutexId lock;
    int rebooting;
    epicsMessageQueueId intMsgQId;
    int messagesSent;
    int messagesFailed;
  /* Asyn interfaces */
    asynInterface common;
    asynInterface int32;
    void *int32InterruptPvt;
    asynInterface float64;
    void *float64InterruptPvt;
    asynInterface float64Array;
    void *float64ArrayInterruptPvt;
    asynInterface drvUser;
    asynUser *pasynUser;
} drvmotorPvt;

/* These functions are used by the interfaces */
static asynStatus readInt32         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *value);
static asynStatus writeInt32        (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 value);
static asynStatus getBounds         (void *drvPvt, asynUser *pasynUser,
                                     epicsInt32 *low, epicsInt32 *high);
static asynStatus readFloat64       (void *drvPvt, asynUser *pasynUser,
                                     epicsFloat64 *value);
static asynStatus writeFloat64      (void *drvPvt, asynUser *pasynUser,
                                     epicsFloat64 value);
static asynStatus drvUserCreate     (void *drvPvt, asynUser *pasynUser,
                                     const char *drvInfo, 
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType    (void *drvPvt, asynUser *pasynUser,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy    (void *drvPvt, asynUser *pasynUser);

static void report                  (void *drvPvt, FILE *fp, int details);
static asynStatus connect           (void *drvPvt, asynUser *pasynUser);
static asynStatus disconnect        (void *drvPvt, asynUser *pasynUser);


/* These are private functions, not used in any interfaces */
static void intCallback(void *drvPvt, unsigned int num, unsigned int *changed);
static void setDefaults(drvmotorAxisPvt *pAxis);
static int config      (drvmotorPvt *pPvt);
     
static asynCommon drvMotorCommon = {
    report,
    connect,
    disconnect
};

static asynInt32 drvMotorInt32 = {
    writeInt32,
    readInt32,
    getBounds
};

static asynFloat64 drvMotorFloat64 = {
    writeFloat64,
    readFloat64
};

static asynFloat64Array drvMotorFloat64Array = {
    NULL,
    NULL,
    NULL,
    NULL
};
static asynDrvUser drvMotorDrvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};


int drvAsynMotorConfigure(const char *portName, const char *driverName, int card, int num_axes)
{
    drvmotorPvt *pPvt;
    drvmotorAxisPvt *pAxis;
    asynStatus status;
    int i;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "drvAsynMotorConfigure");
    pPvt->portName = epicsStrDup(portName);
    pPvt->drvset = (motorAxisDrvSET_t *) registryDriverSupportFind( driverName );
    if (pPvt->drvset == NULL) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't find driver: %s\n", driverName);
	return -1;
    }

    /* Link with higher level routines */
    pPvt->common.interfaceType = asynCommonType;
    pPvt->common.pinterface  = (void *)&drvMotorCommon;
    pPvt->common.drvPvt = pPvt;
    pPvt->int32.interfaceType = asynInt32Type;
    pPvt->int32.pinterface  = (void *)&drvMotorInt32;
    pPvt->int32.drvPvt = pPvt;
    pPvt->float64.interfaceType = asynFloat64Type;
    pPvt->float64.pinterface  = (void *)&drvMotorFloat64;
    pPvt->float64.drvPvt = pPvt;
    pPvt->float64Array.interfaceType = asynFloat64ArrayType;
    pPvt->float64Array.pinterface  = (void *)&drvMotorFloat64Array;
    pPvt->float64Array.drvPvt = pPvt;
    pPvt->drvUser.interfaceType = asynDrvUserType;
    pPvt->drvUser.pinterface  = (void *)&drvMotorDrvUser;
    pPvt->drvUser.drvPvt = pPvt;
    status = pasynManager->registerPort(portName,
                                        ASYN_MULTIDEVICE, /*is multiDevice*/
                                        1,  /*  autoconnect */
                                        0,  /* medium priority */
                                        0); /* default stack size */
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register port\n");
        return -1;
    }
    status = pasynManager->registerInterface(portName,&pPvt->common);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register common.\n");
        return -1;
    }
    status = pasynInt32Base->initialize(pPvt->portName,&pPvt->int32);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register int32\n");
        return -1;
    }
    pasynManager->registerInterruptSource(portName, &pPvt->int32,
                                          &pPvt->int32InterruptPvt);
    status = pasynFloat64Base->initialize(pPvt->portName,&pPvt->float64);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register float64\n");
        return -1;
    }
    pasynManager->registerInterruptSource(portName, &pPvt->float64,
                                          &pPvt->float64InterruptPvt);
    status = pasynFloat64ArrayBase->initialize(pPvt->portName,&pPvt->float64Array);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register float64Array\n");
        return -1;
    }
    pasynManager->registerInterruptSource(portName, &pPvt->float64Array,
                                          &pPvt->float64ArrayInterruptPvt);
    status = pasynManager->registerInterface(pPvt->portName,&pPvt->drvUser);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure ERROR: Can't register drvUser\n");
        return -1;
    }
    /* Create asynUser for debugging */
    pPvt->pasynUser = pasynManager->createAsynUser(0, 0);

    /* Connect to device */
    status = pasynManager->connectDevice(pPvt->pasynUser, portName, -1);
    if (status != asynSuccess) {
        errlogPrintf("drvAsynMotorConfigure, connectDevice failed\n");
        return -1;
    }

    pPvt->lock = epicsMutexCreate();
    pPvt->card = card;
    config(pPvt);

    pPvt->axisData = pasynManager->memMalloc(num_axes * sizeof(drvmotorAxisPvt));
    if (!pPvt->axisData) {
	errlogPrintf("drvAsynMotorConfigure ERROR: Cannot allocate memory\n");
	return -1;
    }
    for ( i = 0; i < num_axes; i++) {
	pAxis = &pPvt->axisData[i];
	pAxis->axis = (*pPvt->drvset->open)(card, i, "");
	pAxis->num = i;
	pAxis->pPvt = pPvt;
	(*pPvt->drvset->setCallback)(pAxis->axis, intCallback, (void *)pAxis);
	setDefaults(pAxis);
    }

    return 0;
}


static int config(drvmotorPvt *pPvt)
{
    /* XXX call the driver config? with pPvt->card */
    return(0);
}

static asynStatus readInt32(void *drvPvt, asynUser *pasynUser, 
                            epicsInt32 *value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;

    pasynManager->getAddr(pasynUser, &channel);
    pAxis = &pPvt->axisData[channel];

    switch(command) {
    case motorPosition:
    case motorEncoderPosition:
	(*pPvt->drvset->getInteger)(pAxis->axis, command, value);
	break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvMotorAsyn::readInt32 invalid command=%d",
                      command);
        return(asynError);
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::readInt32, value=%d", *value);
    return(asynSuccess);
}


static asynStatus readFloat64(void *drvPvt, asynUser *pasynUser,
                              epicsFloat64 *value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;
    asynStatus status = asynSuccess;

    pasynManager->getAddr(pasynUser, &channel);
    pAxis = &pPvt->axisData[channel];

    switch(command) {
    case motorVelocity:
	*value = pAxis->max_velocity;
	break;
    case motorVelBase:
	*value = pAxis->min_velocity;
	break;
    case motorAccel:
	*value = pAxis->accel;
	break;
    case motorPosition:
    case motorEncoderPosition:
	(*pPvt->drvset->getDouble)(pAxis->axis, command, value);
	break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvMotorAsyn::readFloat64 invalid command=%d",
                      command);
        return(asynError);
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::readFloat64, value=%f", *value);
    return(status);
}

static asynStatus getBounds(void *drvPvt, asynUser *pasynUser,
                            epicsInt32 *low, epicsInt32 *high)
{
    *low = 0;
    *high = 65535;
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::getBounds,low=%d, high=%d\n", *low, *high);
    return(asynSuccess);
}

static asynStatus writeInt32(void *drvPvt, asynUser *pasynUser, 
                             epicsInt32 value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;
    asynStatus status;

    pasynManager->getAddr(pasynUser, &channel);
    pAxis = &pPvt->axisData[channel];

    switch(command) {
    case motorStop:
	status = (*pPvt->drvset->stop)(pAxis->axis, pAxis->accel);
	break;
    case motorSetClosedLoop:
	status = (*pPvt->drvset->setInteger)(pAxis->axis, motorAxisClosedLoop,
					     value);
	break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvMotorAsyn::writeInt32D invalid command=%d",
                      command);
        return(asynError);
	break;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
	      "drvMotorAsyn::writeInt32, value=%d", value);
    return(status);
}

static asynStatus writeFloat64(void *drvPvt, asynUser *pasynUser, 
                               epicsFloat64 value)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    drvmotorAxisPvt *pAxis;
    int channel;
    motorCommand command = pasynUser->reason;
    asynStatus status = asynError;

    pasynManager->getAddr(pasynUser, &channel);
    pAxis = &pPvt->axisData[channel];

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
	      "drvMotorAsyn::writeFloat64, reason=%d, pasynUser=%p pAxis=%p\n",
	      command, pasynUser, pAxis);

    switch(command) {
    case motorMoveRel:
	status = (*pPvt->drvset->move)(pAxis->axis, value, 1, pAxis->min_velocity,
			       pAxis->max_velocity, pAxis->accel);
	break;
    case motorMoveAbs:
	status = (*pPvt->drvset->move)(pAxis->axis, value, 0, pAxis->min_velocity,
			       pAxis->max_velocity, pAxis->accel);
	break;
    case motorMoveVel:
	status = (*pPvt->drvset->velocityMove)(pAxis->axis, pAxis->min_velocity,
				       abs(value), (value < 0) ? 0 : 1);
	break;
    case motorHome:
	status = (*pPvt->drvset->home)(pAxis->axis, pAxis->min_velocity, abs(value), pAxis->accel,
			       (value < 0) ? 0 : 1);
	break;
    case motorVelocity:
	pAxis->max_velocity = value;
	break;
    case motorVelBase:
	pAxis->min_velocity = value;
	break;
    case motorAccel:
	pAxis->accel = value;
	break;
    case motorPosition:
    case motorEncRatio:
    case motorPgain:
    case motorIgain:
    case motorDgain:
    case motorHighLim:
    case motorLowLim:
	status = (*pPvt->drvset->setDouble)(pAxis->axis, command, value);
	break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvMotorAsyn::writeFloat64 invalid command=%d",
                      command);
        return(asynError);
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvMotorAsyn::writeFloat64, reason=%s, value=%f", motorCommands[command].commandString, value);
    return(status);
}

static void setDefaults(drvmotorAxisPvt *pAxis)
{
    pAxis->max_velocity = 200.0;
    pAxis->min_velocity = 0.0;
    pAxis->accel = 100.0;
}

static void intCallback(void *axisPvt, unsigned int nChanged,
			unsigned int *changed)
{
    drvmotorAxisPvt *pAxis = (drvmotorAxisPvt *)axisPvt;
    drvmotorPvt *pPvt = pAxis->pPvt;
    int addr, reason;
    ELLLIST *pclientList;
    interruptNode *pnode;
    int ivalue;
    double dvalue;
    int i, bit_num;
    epicsInt32 changedmask = 0;

    /* We are called back with an array of things that have changed.
       First put these into a single int32 word for passing up to higher layers
       Note that for now this relies on the order of the changed flags being
       correct */
    for (i = 0; i < nChanged; i++) {
	if (changed[i] >= motorAxisDirection && 
	    changed[i] <= motorAxisLowHardLimit) {
	    bit_num = changed[i] - motorAxisDirection;
	    changedmask |= (1 << bit_num);
	    (*pPvt->drvset->getInteger)(pAxis->axis, changed[i], &ivalue);
	    BIT_SET(bit_num, &pAxis->status, ivalue);
	}
    }

    /* Pass int32 interrupts */
    pasynManager->interruptStart(pPvt->int32InterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
	asynInt32Interrupt *pint32Interrupt = pnode->drvPvt;
	addr = pint32Interrupt->addr;
	reason = pint32Interrupt->pasynUser->reason;
	if (addr == pAxis->num) {
	    if ( reason >= motorStatusDirection && 
		 reason < motorStatusLast ) {
		if (BIT_ISSET(reason - motorStatusDirection, 1, &changedmask))
		{
		    (*pPvt->drvset->getInteger)(pAxis->axis, reason, &ivalue);
		    pint32Interrupt->callback(pint32Interrupt->userPvt, 
					      pint32Interrupt->pasynUser,
					      ivalue);
		}
	    }
	    /* If we've subscribed to the aggregate status */
	    if (reason == motorStatus) {
		pint32Interrupt->callback(pint32Interrupt->userPvt,
					  pint32Interrupt->pasynUser,
					  pAxis->status);
	    }
	}
	pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(pPvt->int32InterruptPvt);
    
    /* Pass float64 interrupts */
    pasynManager->interruptStart(pPvt->float64InterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
	asynFloat64Interrupt *pfloat64Interrupt = pnode->drvPvt;
	addr = pfloat64Interrupt->addr;
	reason = pfloat64Interrupt->pasynUser->reason;
	if (addr == pAxis->num) {
	    switch(reason) {
	    case motorPosition:
	    case motorEncoderPosition:
		for (i = 0; i < nChanged; i++) {
		    if (changed[i] == reason) {
			(*pPvt->drvset->getDouble)(pAxis->axis, changed[i], &dvalue);
			pfloat64Interrupt->callback(pfloat64Interrupt->userPvt, 
						    pfloat64Interrupt->pasynUser,
						    dvalue);
		    }
		}
		break;
	    }
	}
	pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(pPvt->float64InterruptPvt);

    /* Pass float64Array interrupts */
    pasynManager->interruptStart(pPvt->float64ArrayInterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
	asynFloat64ArrayInterrupt *pfloat64ArrayInterrupt = pnode->drvPvt;
	reason = pfloat64ArrayInterrupt->pasynUser->reason;
	switch(reason) {
	case motorPosition:
	    /*	    pfloat64ArrayInterrupt->callback(pfloat64ArrayInterrupt->userPvt, 
		    pfloat64ArrayInterrupt->pasynUser,
		    pPvt->position, 
		    MAX_AXES);*/
	    break;
	}
	pnode = (interruptNode *)ellNext(&pnode->node);
    }
    pasynManager->interruptEnd(pPvt->float64ArrayInterruptPvt);
}


static void rebootCallback(void *drvPvt)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    /* Anything special we have to do on reboot */
    pPvt->rebooting = 1;
}

/* asynDrvUser routines */
static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
                                const char *drvInfo, 
                                const char **pptypeName, size_t *psize)
{
    int i;
    char *pstring;
    int ncommands = sizeof(motorCommands)/sizeof(motorCommands[0]);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvMotorAsyn::drvUserCreate, drvInfo=%s, pptypeName=%p, psize=%p, pasynUser=%p\n", drvInfo, pptypeName, psize, pasynUser);

    for (i=0; i < ncommands; i++) {
        pstring = motorCommands[i].commandString;
        if (epicsStrCaseCmp(drvInfo, pstring) == 0) {
	    break;
        }
    }
    if (i < ncommands) {
	pasynUser->reason = motorCommands[i].command;
	if (pptypeName) {
	    *pptypeName = epicsStrDup(pstring);
	}
	if (psize) {
	    *psize = sizeof(motorCommands[i].command);
	}
	asynPrint(pasynUser, ASYN_TRACE_FLOW,
		  "drvMotorAsyn::drvUserCreate, command=%s\n", pstring);
	return(asynSuccess);
    } else {
	epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
		      "drvmotorAsyn::drvUserCreate, unknown command=%s", drvInfo);
	return(asynError);
    }
}
    
static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
                                 const char **pptypeName, size_t *psize)
{
    motorCommand command = pasynUser->reason;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvMotorAsyn::drvUserGetType entered");

    *pptypeName = NULL;
    *psize = 0;
    if (pptypeName)
        *pptypeName = epicsStrDup(motorCommands[command].commandString);
    if (psize) *psize = sizeof(command);
    return(asynSuccess);
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvMotorAsyn::drvUserDestroy, drvPvt=%p, pasynUser=%p\n",
	      drvPvt, pasynUser);

    return(asynSuccess);
}

/* asynCommon routines */

/* Report  parameters */
static void report(void *drvPvt, FILE *fp, int details)
{
    drvmotorPvt *pPvt = (drvmotorPvt *)drvPvt;
    interruptNode *pnode;
    ELLLIST *pclientList;

    fprintf(fp, "Port: %s\n", pPvt->portName);
    if (details >= 1) {
        fprintf(fp, "    messages sent OK=%d; send failed (queue full)=%d\n",
                pPvt->messagesSent, pPvt->messagesFailed);
        /* Report int32 interrupts */
        pasynManager->interruptStart(pPvt->int32InterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynInt32Interrupt *pint32Interrupt = pnode->drvPvt;
            fprintf(fp, "    int32 callback client address=%p, addr=%d, reason=%d\n",
                    pint32Interrupt->callback, pint32Interrupt->addr, 
                    pint32Interrupt->pasynUser->reason);
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->int32InterruptPvt);

        /* Report float64 interrupts */
        pasynManager->interruptStart(pPvt->float64InterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynFloat64Interrupt *pfloat64Interrupt = pnode->drvPvt;
            fprintf(fp, "    float64 callback client address=%p, addr=%d, reason=%d\n",
                    pfloat64Interrupt->callback, pfloat64Interrupt->addr, 
                    pfloat64Interrupt->pasynUser->reason);
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->float64InterruptPvt);

        /* Report int32Array interrupts */
        pasynManager->interruptStart(pPvt->float64ArrayInterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynFloat64ArrayInterrupt *pfloat64ArrayInterrupt = pnode->drvPvt;
            fprintf(fp, "    float64Array callback client address=%p, reason=%d\n",
                    pfloat64ArrayInterrupt->callback, 
                    pfloat64ArrayInterrupt->pasynUser->reason); 
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->float64ArrayInterruptPvt);
    }
}

/* Connect */
static asynStatus connect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionConnect(pasynUser);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
	      "drvMotorAsyn::connect, pasynUser=%p\n", pasynUser);
    return(asynSuccess);
}

/* Disconnect */
static asynStatus disconnect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionDisconnect(pasynUser);
    return(asynSuccess);
}

static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "driverName",iocshArgString};
static const iocshArg initArg2 = { "cardNum",iocshArgInt};
static const iocshArg initArg3 = { "numAxes",iocshArgInt};
static const iocshArg * const initArgs[4] = {&initArg0,
                                             &initArg1,
					     &initArg2,
					     &initArg3};
static const iocshFuncDef initFuncDef = {"drvAsynMotorConfigure",4,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    drvAsynMotorConfigure(args[0].sval, args[1].sval, args[2].ival,
			  args[3].ival);
}

void motorRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(motorRegister);
