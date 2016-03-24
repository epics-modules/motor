/*
FILENAME...     PIasynController.cpp
USAGE...        PI GCS2 Motor Support.
 
*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************
 
 
Original Author: Steffen Rau
Created: January 2011

Based on drvMotorSim.c, Mark Rivers, December 13, 2009

*/


#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>
//#include <motorVersion.h>
#include <motor_interface.h>
#include <ctype.h>

#include "PIasynController.h"
#include "PIGCSController.h"
#include "PIasynAxis.h"
#include "PIInterface.h"
#include <epicsExport.h>


//#undef asynPrint
//#define asynPrint(user,reason,format...) 0


static const char *driverName = "PIasynDriver";


static ELLLIST PIasynControllerList;
static int PIasynControllerListInitialized = 0;

PIasynController::PIasynController(const char *portName, const char* asynPort, int numAxes, int priority, int stackSize, int movingPollPeriod, int idlePollPeriod)
    : asynMotorController(portName, numAxes, 10,
            asynInt32Mask | asynFloat64Mask,
            asynInt32Mask | asynFloat64Mask,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE,
            1, // autoconnect
            priority, stackSize)
	, movesDeferred( 0 )
	, m_pGCSController( NULL )
{
    createParam(PI_SUP_POSITION_String,		asynParamFloat64,	&PI_SUP_POSITION);
    createParam(PI_SUP_TARGET_String,		asynParamFloat64,	&PI_SUP_TARGET);
    createParam(PI_SUP_SERVO_String,		asynParamInt32,		&PI_SUP_SERVO);
    createParam(PI_SUP_LAST_ERR_String,		asynParamInt32,		&PI_SUP_LAST_ERR);
    createParam(PI_SUP_PIVOT_X_String,		asynParamFloat64,	&PI_SUP_PIVOT_X);
    createParam(PI_SUP_PIVOT_Y_String,		asynParamFloat64,	&PI_SUP_PIVOT_Y);
    createParam(PI_SUP_PIVOT_Z_String,		asynParamFloat64,	&PI_SUP_PIVOT_Z);
    createParam(PI_SUP_RBPIVOT_X_String,	asynParamFloat64,	&PI_SUP_RBPIVOT_X);
    createParam(PI_SUP_RBPIVOT_Y_String,	asynParamFloat64,	&PI_SUP_RBPIVOT_Y);
    createParam(PI_SUP_RBPIVOT_Z_String,	asynParamFloat64,	&PI_SUP_RBPIVOT_Z);

    int axis;
    PIasynAxis *pAxis;
    PIasynControllerNode *pNode;

    if (!PIasynControllerListInitialized)
    {
        PIasynControllerListInitialized = 1;
        ellInit(&PIasynControllerList);
    }

    // We should make sure this portName is not already in the list */
    pNode = (PIasynControllerNode*) calloc(1, sizeof(PIasynControllerNode));
    pNode->portName = epicsStrDup(portName);
    pNode->pController = this;
    ellAdd(&PIasynControllerList, (ELLNODE *)pNode);


    asynStatus status;
    asynUser* pAsynCom;
    status = pasynOctetSyncIO->connect(asynPort, 0, &pAsynCom, NULL);
    if (status)
    {
    	asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
          "echoHandler: unable to connect to port %s\n",
          asynPort);
    	return;
	}
	status = pasynOctetSyncIO->setInputEos(pAsynCom, "\n", 1);
	if (status) {
		asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "echoHandler: unable to set input EOS on %s: %s\n",
			  asynPort, pAsynCom->errorMessage);
		return;
	}
	status = pasynOctetSyncIO->setOutputEos(pAsynCom, "", 0);
	if (status) {
		asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "echoHandler: unable to set output EOS on %s: %s\n",
			  asynPort, pAsynCom->errorMessage);
		return;
	}

	PIInterface* pInterface = new PIInterface(pAsynCom);
	char inputBuff[256];
	inputBuff[0] = '\0';
	status = pInterface->sendAndReceive("*IDN?", inputBuff, 255, pAsynCom);
	asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "read from %s: %s\n",
			  asynPort, inputBuff);

	char* p = inputBuff;
	while (*p != '\0') { *p = toupper(*p); p++; }

	m_pGCSController = PIGCSController::CreateGCSController(pInterface, inputBuff);
	if (NULL == m_pGCSController)
	{
		asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "PIasynController: unknown controller type %s: %s\n",
			  asynPort, inputBuff);
		return;
	}

	m_pGCSController->init();

    if (numAxes < 1 ) numAxes = 1;
    this->numAxes_ = numAxes;

	if (m_pGCSController->getNrFoundAxes()<size_t(numAxes))
	{
		// more axes configured than connected to controller
		asynPrint(pAsynCom, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
			  "PIasynController: requested number of axes (%d) out of range, only %d axis/axes supported\n",
			  numAxes, int(m_pGCSController->getNrFoundAxes()));
		delete m_pGCSController;
		m_pGCSController = NULL;
		return;
	}

    for (axis=0; axis<numAxes; axis++)
    {
        pAxis  = new PIasynAxis(this, m_pGCSController, axis, m_pGCSController->getAxesID(axis));
        pAxis->Init(portName);
    }

    startPoller(double(movingPollPeriod)/1000, double(idlePollPeriod)/1000, 10);
}




void PIasynController::report(FILE *fp, int level)
{
    int axis;
    PIasynAxis *pAxis;

    fprintf(fp, "Simulation motor driver %s, numAxes=%d\n", 
        this->portName, this->numAxes_);

    for (axis=0; axis<this->numAxes_; axis++)
    {
        pAxis = getPIAxis(axis);
        fprintf(fp, "  axis %d\n", 
            pAxis->axisNo_);

        if (level > 0)
        {
            if (pAxis->m_isHoming)
            {
            	fprintf(fp, "    Currently homing axis\n" );
            }
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

//PIasynAxis * PIasynController::getPIAxis(asynUser *pasynUser)
//{
//    int axis;
//    PIasynAxis *pAxis;
//
//    getAddress(pasynUser, &axis);
//    pAxis = this->m_pAxes[axis];
//    pAxis->m_pasynUser = pasynUser;
//    return(pAxis);
//}
    


asynStatus PIasynController::processDeferredMoves()
{
    asynStatus status = asynError;
    int axis;
    PIasynAxis *pAxesArray[PIGCSController::MAX_NR_AXES];
    int targetsCts[PIGCSController::MAX_NR_AXES];

    int numDeferredAxes = 0;
    for (axis=0; axis<this->numAxes_; axis++)
    {
    	PIasynAxis *pAxis = getPIAxis(axis);
        if (pAxis->deferred_move)
        {
        	pAxesArray[numDeferredAxes] = pAxis;
        	targetsCts[numDeferredAxes] = pAxis->deferred_position;
        	pAxis->setIntegerParam(motorStatusDone_, 0);
        	pAxis->callParamCallbacks();
        	numDeferredAxes++;
        }
    }
    if (numDeferredAxes > 0)
    {
    	status = m_pGCSController->moveCts(pAxesArray, targetsCts, numDeferredAxes);
    }

    for (axis=0; axis<this->numAxes_; axis++)
    {
        if (getPIAxis(axis)->deferred_move)
        {
        	getPIAxis(axis)->deferred_move = 0;
        }
    }
    epicsEventSignal(pollEventId_);

    return status;
}


asynStatus PIasynController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	if (NULL == m_pGCSController)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
          "PIasynController::writeInt32() GCS controller not initialized!\n");

		return asynError;
	}
	m_pGCSController->m_pInterface->m_pCurrentLogSink = pasynUser;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    PIasynAxis *pAxis = (PIasynAxis *)this->getAxis(pasynUser);
    static const char *functionName = "writeInt32";
    
    lock();
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = pAxis->setIntegerParam(function, value);
    
    if (function == motorClosedLoop_)
    {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
        		"%s:%s: %sing Closed-Loop Control flag on driver %s\n",
        		value != 0.0?"Enabl":"Disabl",
        		driverName, functionName, this->portName);
        status = m_pGCSController->setServo(pAxis, (value!=0)?1:0);

    }
    else if (function == motorDeferMoves_)
    {
    	asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: %sing Deferred Move flag on driver %s\n",
            value != 0.0?"Sett":"Clear",
            driverName, functionName, this->portName);
        if (value == 0.0 && this->movesDeferred != 0)
        {
            processDeferredMoves();
        }
        this->movesDeferred = value;
    } else {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeInt32(pasynUser, value);
    }
    unlock();
    /* Do callbacks so higher layers see any changes */
    pAxis->callParamCallbacks();
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}

asynStatus PIasynController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
	if (NULL == m_pGCSController)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
          "PIasynController::writeFloat64() GCS controller not initialized!\n");

		return asynError;
	}
	m_pGCSController->m_pInterface->m_pCurrentLogSink = pasynUser;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    PIasynAxis *pAxis = (PIasynAxis *)this->getAxis(pasynUser);
    static const char *functionName = "writeFloat64";
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = pAxis->setDoubleParam(function, value);
    
    if (function == PI_SUP_TARGET)
    {
    	printf("PI_SUP_TargetAO: %f for axis %d\n", value, pAxis->axisNo_);
    }
    else if (function == PI_SUP_PIVOT_X)
    {
    	status = m_pGCSController->SetPivotX(value);
    }
    else if (function == PI_SUP_PIVOT_Y)
    {
    	status = m_pGCSController->SetPivotY(value);
    }
    else if (function == PI_SUP_PIVOT_Z)
    {
    	status = m_pGCSController->SetPivotZ(value);
    }
//    else if (function == motorPosition_) // Entspricht das DFH ?
//    {
//  //      pAxis->enc_offset = (double) value - pAxis->nextpoint.axis[0].p;
//        asynPrint(pasynUser, ASYN_TRACE_FLOW,
//            "%s:%s: Set axis %d to position %d",
//            driverName, functionName, pAxis->axisNo_, value);
//    }
    else if (function == motorResolution_ )
    {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
    else if (function == motorEncoderRatio_)
    {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
    else
    {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
    /* Do callbacks so higher layers see any changes */
    pAxis->callParamCallbacks();
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
              "%s:%s: error, status=%d function=%d, value=%f\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%f\n", 
              driverName, functionName, function, value);
    return status;
}




asynStatus PIasynController::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger )
{
	asynPrint(pasynUser, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
			"PIasynController::profileMove() - not implemented\n");
	return asynError;
}

asynStatus PIasynController::triggerProfile(asynUser *pasynUser)
{
	asynPrint(pasynUser, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
			"PIasynController::profileMove() - not implemented\n");
	return asynError;
}

asynStatus PIasynController::configAxis(PIasynAxis *pAxis)
{
	asynUser* logSink = pasynManager->createAsynUser(0,0);
	asynStatus status = pasynManager->connectDevice(logSink, portName, pAxis->getAxisNo());
	if (status != asynSuccess)
	{
		asynPrint(logSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
				"PIasynController::configAxis() - connectDevice() failed\n");
		return status;
	}
	m_pGCSController->m_pInterface->m_pCurrentLogSink = logSink;
    
    pAxis->setIntegerParam(this->motorStatusGainSupport_, 1);
    pAxis->callParamCallbacks();

    m_pGCSController->initAxis(pAxis);
    double resolution;
    m_pGCSController->getResolution(pAxis, resolution);
    m_pGCSController->getAxisVelocity(pAxis);
    m_pGCSController->getAxisPositionCts(pAxis);
    pAxis->setDoubleParam(this->motorPosition_, pAxis->m_positionCts);
    pAxis->setDoubleParam(this->motorMoveAbs_, pAxis->m_positionCts);
    double negLimit, posLimit;
    m_pGCSController->getTravelLimits(pAxis, negLimit, posLimit);
    pAxis->setDoubleParam(this->motorLowLimit_, negLimit);
    pAxis->setDoubleParam(this->motorHighLimit_, posLimit);
    m_pGCSController->getReferencedState(pAxis);
	pAxis->setIntegerParam( this->motorStatusHomed_,        pAxis->m_homed );

    pasynManager->freeAsynUser(logSink);

    /* Send a signal to the poller task which will make it do a poll,
     * updating values for this axis to use the new resolution (stepSize) */
    epicsEventSignal(pollEventId_);

    return(asynSuccess);
}

/**\defgroup PIasynTask Routines to implement the motor axis simulation task
@{
*/

/** Process one iteration of an axis

    This routine takes a single axis and propogates its motion forward a given amount
    of time.

    \param pAxis  [in]   Pointer to axis information.
    \param delta  [in]   Time in seconds to propogate motion forwards.

    \return Integer indicating 0 (asynSuccess) for success or non-zero for failure. 
*/


//
//static void PIasynTaskC(void *drvPvt)
//{
//    PIasynController *pController = (PIasynController*)drvPvt;
//    pController->PIasynTask();
//}
//
//
//
//void PIasynController::PIasynTask()
//{
//	double timeout = m_idlePollingRate;
//    int axis;
//    PIasynAxis *pAxis;
//    epicsEventSignal(pollEventId_);  /* Force on poll at startup */
//
//    int status;
//    while ( 1 )
//    {
//        if (timeout != 0.) status = epicsEventWaitWithTimeout(pollEventId_, timeout);
//        else               status = epicsEventWait(pollEventId_);
//
//        lock();
//        m_pGCSController->getGlobalState(pAxes_, numAxes_);
//        unlock();
//
//        bool bAnyAxisMoving = false;
//		for (axis=0; axis<this->numAxes_; axis++)
//		{
//			lock();
//			pAxis = getPIAxis(axis);
//			process(pAxis);
//			pAxis->callParamCallbacks();
//			unlock();
//			bAnyAxisMoving = (pAxis->m_bMoving != 0);
//		}
//		m_pGCSController->m_bAnyAxisMoving = bAnyAxisMoving;
//		if (m_pGCSController->m_bAnyAxisMoving)
//			timeout = m_movingPollingRate;
//		else
//			timeout = m_idlePollingRate;
//    }
//}

asynStatus PIasynController::poll()
{
    m_pGCSController->getGlobalState(pAxes_, numAxes_);

    setDoubleParam( 0, PI_SUP_RBPIVOT_X, m_pGCSController->GetPivotX());
    setDoubleParam( 0, PI_SUP_RBPIVOT_Y, m_pGCSController->GetPivotY());
    setDoubleParam( 0, PI_SUP_RBPIVOT_Z, m_pGCSController->GetPivotZ());

    setIntegerParam( 0, PI_SUP_LAST_ERR, m_pGCSController->GetLastError() );

    callParamCallbacks();
    return asynSuccess;
}


/** Configuration command, called directly or from iocsh */
extern "C" int PI_GCS2_CreateController(const char *portName, const char* asynPort, int numAxes, int priority, int stackSize, int movingPollingRate, int idlePollingRate)
{
    PIasynController *pasynController
        = new PIasynController(portName, asynPort, numAxes, priority, stackSize, movingPollingRate, idlePollingRate);
    pasynController = NULL;
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg PI_GCS2_CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg PI_GCS2_CreateControllerArg1 = {"asyn Port name", iocshArgString};
static const iocshArg PI_GCS2_CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg PI_GCS2_CreateControllerArg3 = {"priority", iocshArgInt};
static const iocshArg PI_GCS2_CreateControllerArg4 = {"stackSize", iocshArgInt};
static const iocshArg PI_GCS2_CreateControllerArg5 = {"moving polling time [msec]", iocshArgInt};
static const iocshArg PI_GCS2_CreateControllerArg6 = {"idle polling time [msec]", iocshArgInt};
static const iocshArg * const PI_GCS2_CreateControllerArgs[] =  {&PI_GCS2_CreateControllerArg0,
                                                          &PI_GCS2_CreateControllerArg1,
                                                          &PI_GCS2_CreateControllerArg2,
                                                          &PI_GCS2_CreateControllerArg3,
                                                          &PI_GCS2_CreateControllerArg4,
                                                          &PI_GCS2_CreateControllerArg5,
                                                          &PI_GCS2_CreateControllerArg6};
static const iocshFuncDef PI_GCS2_CreateControllerDef = {"PI_GCS2_CreateController", 7, PI_GCS2_CreateControllerArgs};
static void PI_GCS2_CreateControllerCallFunc(const iocshArgBuf *args)
{
    PI_GCS2_CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}


static void PIasynDriverRegister(void)
{
    iocshRegister(&PI_GCS2_CreateControllerDef, PI_GCS2_CreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(PIasynDriverRegister);
}
