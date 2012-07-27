/*
 * PIC702Controller
 *
 *      Author: sra
 */

/*
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

#include "PIC702Controller.h"
#include "PIasynAxis.h"
#include <stdlib.h>
#include <math.h>


//#undef asynPrint
//#define asynPrint(user,reason,format...) 0


asynStatus PIC702Controller::getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl)
{
	int busy;

    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);
    if(epicsTimeDiffInSeconds(&now,&m_timeREFstarted) < 1.0)
    {
    	homing = 1;
    	moving = 1;
    	return asynSuccess;
    }


	asynStatus status = getBusy(pAxis, busy);
    if (status != asynSuccess)
    {
    	return status;
    }

    negLimit = 0;
   	posLimit = 0;
   	homing = busy;
   	if (busy)
   	{
   		moving = busy;
   		return status;
   	}

	status = getMoving(pAxis, moving);
    if (status != asynSuccess)
    {
    	return status;
    }

    return status;
}

asynStatus PIC702Controller::findConnectedAxes()
{
	// GCS! - axes are not separated by LineFeeds
	// axis identifier is only one single char, all axes returned as single "word"
	m_nrFoundAxes = 0;
	for (size_t i=0; i<MAX_NR_AXES; i++)
	{
		m_axesIDs[i] = NULL;
	}
	char allAxesIDs[127];
	asynStatus status = PIGCSController::sendAndReceive("SAI?", allAxesIDs, 127);

	if (asynSuccess != status)
	{
		return status;
	}
	m_nrFoundAxes = strlen(allAxesIDs);
	if (MAX_NR_AXES <= m_nrFoundAxes)
	{
		return asynError;
	}
	char* p = m_allAxesIDs;
	for (size_t i=0; i<m_nrFoundAxes; i++)
	{
		*p = allAxesIDs[m_nrFoundAxes - 1 - i];
		m_axesIDs[i] = p;
		p++;
		*p = '\0';
		p++;
	}
	return status;
}

asynStatus PIC702Controller::getMaxAcceleration(PIasynAxis* pAxis)
{
	double maxAcc, maxDec;
	asynStatus status = getGCSParameter(pAxis, PI_PARA_MOT_CURR_ACCEL, maxAcc);
	if (asynSuccess != status)
		return status;
	status = getGCSParameter(pAxis, PI_PARA_MOT_CURR_DECEL, maxDec);
	if (asynSuccess != status)
		return status;

	if (maxAcc < maxDec)
	{
		pAxis->m_maxAcceleration = maxAcc;
	}
	else
	{
		pAxis->m_maxAcceleration = maxDec;
	}
	return status;
}

asynStatus PIC702Controller::hasReferenceSensor(PIasynAxis* pAxis)
{
	double hasref;
	asynStatus status = getGCSParameter(pAxis, PI_PARA_MOT_HAT_REF, hasref);
	if (asynSuccess != status)
	{
		return status;
	}
    pAxis->m_bHasReference = hasref > 0.1;

    asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIC702Controller::hasReferenceSwitch() axis has %sreference sensor\n",
   		 pAxis->m_bHasReference?"":"no ");
	return status;
}

asynStatus PIC702Controller::getReferencedState(PIasynAxis* pAxis)
{
	double isref;
	asynStatus status = getGCSParameter(pAxis, PI_PARA_C702_REFERENCED, isref);
    if (status != asynSuccess)
    {
    	return status;
    }
    pAxis->m_homed = (isref > 0.1)?1:0;

    return status;
}

asynStatus PIC702Controller::referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards)
{
	asynStatus status = setServo(pAxis, 1);
    if (asynSuccess != status)
    	return status;

    status = setVelocityCts(pAxis, velocity);
    if (asynSuccess != status)
    	return status;

    char cmd[100];
	if (pAxis->m_bHasReference)
	{
		// call REF - find reference
		sprintf(cmd,"REF %s", pAxis->m_szAxisName);
	}
	else if (pAxis->m_bHasLimitSwitches)
	{
		if (forwards)
		{
			// call MPL - find positive limit switch
			sprintf(cmd,"MPL %s", pAxis->m_szAxisName);
		}
		else
		{
			// call MNL - find negative limit switch
			sprintf(cmd,"MNL %s", pAxis->m_szAxisName);
		}
	}
	else
	{
	    asynPrint(m_pCurrentLogSink, ASYN_TRACE_ERROR,
	    		"PIC702Controller::referenceVelCts() failed - axis has no reference/limit switch\n");
		epicsSnprintf(pAxis->m_pasynUser->errorMessage,pAxis->m_pasynUser->errorMessageSize,
			"PIC702Controller::referenceVelCts() failed - axis has no reference/limit switch\n");
		return asynError;
	}
	status = sendOnly(cmd);
	if (asynSuccess != status)
	{
		asynPrint(m_pCurrentLogSink, ASYN_TRACE_ERROR,
    		"PIC702Controller::referenceVelCts() failed\n");
		return status;
	}
	pAxis->m_isHoming = 1;
    epicsTimeGetCurrent(&m_timeREFstarted);

	return asynSuccess;
}

