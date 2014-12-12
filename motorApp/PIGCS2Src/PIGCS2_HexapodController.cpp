/*
FILENAME...     PIHexapodController.cpp

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision: 1$
Modified By:    $Author: Steffen Rau$
Last Modified:  $Date: 05.11.2013 17:38:06$
HeadURL:        $URL$

Original Author: Steffen Rau
*/

#include "PIGCS2_HexapodController.h"
#include "PIasynAxis.h"
#include "PIInterface.h"
#include <stdlib.h>
#include <math.h>
#include <epicsThread.h>

//#undef asynPrint
//#define asynPrint(user,reason,format...) 0

/**
 *  get velocity of axis in physical units (EGU) as defined on the controller
 *  and set PIasynAxis::m_velocity
 *  For GCS2 Hexapods this is the platform velocity read with VLS
 */
asynStatus PIGCS2_HexapodController::getAxisVelocity(PIasynAxis* pAxis)
{
	char buf[255];
	asynStatus status = m_pInterface->sendAndReceive("VLS?", buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}
	if (!getValue(buf, pAxis->m_velocity))
	{
		status = asynError;
	}
    return status;
}

asynStatus PIGCS2_HexapodController::setVelocityCts( PIasynAxis* pAxis, double velocity )
{
	char cmd[100];
	velocity = fabs(velocity) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;
    sprintf(cmd,"VLS %f", velocity);
    asynStatus status = m_pInterface->sendOnly(cmd);
    if (asynSuccess == status)
    {
    	pAxis->m_velocity = velocity;
    }
    return status;
}


asynStatus PIGCS2_HexapodController::referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards)
{
	asynStatus status = setServo(pAxis, 1);
    if (asynSuccess != status)
    	return status;

    // velocity is ignored!
	char cmd[100];
	sprintf(cmd,"FRF %s", pAxis->m_szAxisName);
	status = m_pInterface->sendOnly(cmd);
	if (asynSuccess != status)
		return status;
	int errorCode = getGCSError();
	if (errorCode == 0)
	{
		return asynSuccess;
	}
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_ERROR,
    		"PIGCS2_HexapodController::referenceVelCts() failed\n");
	epicsSnprintf(pAxis->m_pasynUser->errorMessage,pAxis->m_pasynUser->errorMessageSize,
		"PIGCS2_HexapodController::referenceVelCts() failed - GCS Error %d\n",errorCode);
	return asynError;

}
