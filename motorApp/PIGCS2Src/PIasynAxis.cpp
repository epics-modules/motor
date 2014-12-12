/*
FILENAME...     PIasynController.cpp
USAGE...

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision: 4$
Modified By:    $Author: Steffen Rau$
Last Modified:  $Date: 25.10.2013 10:43:08$
HeadURL:        $URL$

Original Author: Steffen Rau 

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
#include <motor_interface.h>

#include "PIasynAxis.h"
#include "PIasynController.h"
#include "PIGCSController.h"
#include "PIInterface.h"

#include <epicsExport.h>

//#undef asynPrint
//#define asynPrint(user,reason,format...) 0

static const char *driverName = "PIasynAxis";

PIasynAxis::PIasynAxis(PIasynController *pController, PIGCSController* pGCSController, int axis, const char* szName )
: asynMotorAxis((asynMotorController*)pController, axis)
, pController_(pController)
, m_szAxisName(NULL)
, m_isHoming(0)
, m_homed(0)
, m_acceleration(0.0)
, m_maxAcceleration(-1.0)
, m_lastDirection(0)
, m_CPUnumerator(1000)
, m_CPUdenominator(1)
, m_pasynUser(NULL)
, m_bHasLimitSwitches(false)
, m_bHasReference(false)
, m_bProblem(false)
, m_bServoControl(false)
, m_bMoving(false)
, m_pGCSController(pGCSController)
{
      if (szName != NULL)
      {
    	  m_szAxisName = new char[strlen(szName)+1];
    	  strcpy(m_szAxisName, szName);
      }

      printf("PIasynAxis::PIasynAxis() %d: %s\n",
  			  axis, m_szAxisName);
}


void PIasynAxis::Init(const char *portName)
{
	asynUser* logSink = pasynManager->createAsynUser(0,0);
	asynStatus status = pasynManager->connectDevice(logSink, portName, getAxisNo());
	if (status != asynSuccess)
	{
		asynPrint(logSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
				"PIasynController::configAxis() - connectDevice() failed\n");
		return;
	}
	m_pGCSController->m_pInterface->m_pCurrentLogSink = logSink;

	setIntegerParam(motorAxisHasClosedLoop, 1);

	m_pGCSController->initAxis(this);
	double resolution;
	m_pGCSController->getResolution(this, resolution);
	m_pGCSController->getAxisVelocity(this);
	m_pGCSController->getAxisPositionCts(this);
	setDoubleParam(pController_->motorPosition_, m_positionCts);
	setDoubleParam(pController_->motorMoveAbs_, m_positionCts);
	m_pGCSController->getTravelLimits(this, negLimit_, posLimit_);
	setDoubleParam(pController_->motorLowLimit_, negLimit_);
	setDoubleParam(pController_->motorHighLimit_, posLimit_);
	m_pGCSController->getReferencedState(this);
	setIntegerParam( pController_->motorStatusHomed_, m_homed );
	callParamCallbacks();

	pasynManager->freeAsynUser(logSink);

}

PIasynAxis::~PIasynAxis()
{
	if (m_szAxisName != NULL)
	{
		delete [] m_szAxisName;
	}
}

asynStatus PIasynAxis::poll(bool *returnMoving)
{
    int done = 0;

    int moving, negLimit, posLimit, servoControl;
    int oldHoming = m_isHoming;
    m_pGCSController->getStatus(this, m_isHoming, moving, negLimit, posLimit, servoControl);
    if (moving == 0 && m_isHoming == 0)
    	done = 1;

    m_bMoving = (done!=1);
    if (!m_isHoming || m_pGCSController->CanCommunicateWhileHoming())
    {
		if (oldHoming && oldHoming != m_isHoming)
		{
			m_pGCSController->getReferencedState(this);
		    asynPrint(pasynUser_, ASYN_TRACE_ERROR, //FIXME: ASYN_TRACE_FLOW,
		        "PIasynAxis::poll() axis %d referencing state changed, homed = %d\n",
		        axisNo_, m_homed );
		}
		if (m_bServoControl && servoControl == 0) // servo changed without user interaction!
		{
			m_bProblem = true;
		}
		if (!m_isHoming || m_pGCSController->IsGCS2())
		{
			m_bServoControl = (servoControl == 1);
			m_pGCSController->getAxisPositionCts(this);
			double realPosition;
			m_pGCSController->getAxisPosition(this, realPosition);
			setDoubleParam(pController_->PI_SUP_POSITION,      realPosition );
		}
    }
    if (m_isHoming)
    {
	    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
	        "PIasynAxis::poll() axis %d referencing ...\n", axisNo_ );
    }
    setDoubleParam(pController_->motorPosition_,          m_positionCts );
    setDoubleParam(pController_->motorEncoderPosition_,   m_positionCts);
    setIntegerParam(pController_->motorStatusDirection_,   m_lastDirection);
    setIntegerParam(pController_->motorStatusDone_,        done );
    setIntegerParam(pController_->motorStatusHighLimit_,   posLimit);
    setIntegerParam(pController_->motorStatusHomed_,       m_homed );
    setIntegerParam(pController_->motorStatusMoving_,      !done );
    setIntegerParam(pController_->motorStatusLowLimit_,    negLimit);
    setIntegerParam(pController_->motorStatusGainSupport_,	true);
    setIntegerParam(pController_->motorStatusProblem_,		m_bProblem);
    setIntegerParam(pController_->motorStatusPowerOn_,		m_bServoControl);
    setIntegerParam(pController_->PI_SUP_SERVO,	      	m_bServoControl );

    callParamCallbacks();

    *returnMoving = m_bMoving;
    return asynSuccess;
}

asynStatus PIasynAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
	m_pGCSController->m_pInterface->m_pCurrentLogSink = pasynUser_;
	asynStatus status = asynError;
	static const char *functionName = "moveAxis";

	if (!m_pGCSController->AcceptsNewTarget())
	{
	    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
	        "%s:%s: Set port %s, axis %d - controller does not accept new target (busy?)\n",
	        driverName, functionName, pC_->portName, axisNo_ );
	    printf("%s:%s: Set port %s, axis %d - controller does not accept new target (busy?)\n",
	        driverName, functionName, pC_->portName, axisNo_ );
		return status;
	}

    if (relative)
    {
    	//TODO: MVR oder letztes target!
    	//TODO: when is this used?
    }

    if (pController_->movesDeferred != 0)
    { /*Deferred moves.*/
        deferred_position = position;
        deferred_move = 1;
        deferred_relative = relative;
        setIntegerParam(pController_->motorStatusDone_, 0);
        callParamCallbacks();
        return asynSuccess;
    }
    else
    {
    	if (maxVelocity != 0)
		{
			status = m_pGCSController->setVelocityCts(this, maxVelocity);
			if (asynSuccess != status)
			{
				return status;
			}
		}
		if (acceleration != 0)
		{
			status = m_pGCSController->setAccelerationCts(this, acceleration);
			if (asynSuccess != status)
			{
				return status;
			}
		}

		setIntegerParam(pController_->motorStatusDone_, 0);
		callParamCallbacks();

		status = m_pGCSController->moveCts(this, position);
   }
    epicsEventSignal(pController_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f, deffered=%d - status=%d\n",
        driverName, functionName, pC_->portName, axisNo_, position, minVelocity, maxVelocity, acceleration, pController_->movesDeferred, int(status) );
    return status;
}

asynStatus PIasynAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
	m_pGCSController->m_pInterface->m_pCurrentLogSink = pasynUser_;
	asynStatus status = asynError;
    static const char *functionName = "moveVelocityAxis";

	if (!m_pGCSController->AcceptsNewTarget())
	{
	    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
	        "%s:%s: Set port %s, axis %d - controller does not accept new target (busy?)",
	        driverName, functionName, pController_->portName, axisNo_ );
		return status;
	}


    setIntegerParam(pController_->motorStatusDone_, 0);
    callParamCallbacks();


    double target = maxVelocity > 0 ? posLimit_ : negLimit_;

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set port %s, axis %d move with velocity of %f, accel=%f / target %f - BEFORE MOV\n",
        driverName, functionName, pController_->portName, axisNo_, maxVelocity, acceleration, target );

    m_pGCSController->setVelocityCts(this, maxVelocity);
    m_pGCSController->move(this, target);

    epicsEventSignal(pController_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set port %s, axis %d move with velocity of %f, accel=%f / target %f - AFTER MOV\n",
        driverName, functionName, pController_->portName, axisNo_, maxVelocity, acceleration, target );
    return status;
}

asynStatus PIasynAxis::stop(double acceleration)
{
	m_pGCSController->m_pInterface->m_pCurrentLogSink = pasynUser_;
    static const char *functionName = "stopAxis";

    deferred_move = 0;

    m_pGCSController->haltAxis(this);

    epicsEventSignal(pController_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set axis %d to stop with accel=%f",
        driverName, functionName, axisNo_, acceleration );
    return asynSuccess;
}



asynStatus PIasynAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
	m_pGCSController->m_pInterface->m_pCurrentLogSink = pasynUser_;
    asynStatus status = asynError;
    static const char *functionName = "homeAxis";

    m_isHoming = 1;
    setIntegerParam(pController_->motorStatusDone_, 0 );
    callParamCallbacks();

    status = m_pGCSController->referenceVelCts(this, maxVelocity, forwards);
    if (asynSuccess != status)
    {
    	return status;
    }
    setIntegerParam(pController_->motorStatusHomed_, m_homed );
    epicsEventSignal(pController_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set driver %s, axis %d to home %s, min vel=%f, max_vel=%f, accel=%f",
        driverName, functionName, pController_->portName, axisNo_, (forwards?"FORWARDS":"REVERSE"), minVelocity, maxVelocity, acceleration );
    return status;
}

asynStatus PIasynAxis::setPosition(double position)
{
	m_pGCSController->m_pInterface->m_pCurrentLogSink = pasynUser_;
	asynStatus status = asynError;
	status = m_pGCSController->setAxisPositionCts(this, position);
    epicsEventSignal(pController_->pollEventId_);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set driver %s, axis %d set position to %f - status=%d\n",
        driverName, "setPositionAxis", pC_->portName, axisNo_, position, int(status) );
    return status;
}


