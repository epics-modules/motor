/*
 * PIE755Controller
 *
 *      Author: sra
 */
 
/*
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

#include "PIHexapodController.h"
#include "PIasynAxis.h"
#include <stdlib.h>
#include <epicsThread.h>

//#undef asynPrint
//#define asynPrint(user,reason,format...) 0

asynStatus PIHexapodController::init(void)
{
	asynStatus status = PIGCSController::init();
	if (status != asynSuccess)
	{
		return status;
	}

	// Try to find out if #4 is supported
	char buf[200];
	status = sendAndReceive(char(4), buf, 199);
	if (status == asynSuccess)
	{
		m_bCanReadStatusWithChar4 = true;
	}
	else if (status == asynTimeout)
	{
		m_bCanReadStatusWithChar4 = false;
		getGCSError(); // clear error UNKNOWN COMMAND
	}
//	status = sendAndReceive(char(3), buf, 199);
//	if (status == asynSuccess)
//	{
//		m_bCanReadPosWithChar3 = true;
//	}
//	else if (status == asynTimeout)
	{
		m_bCanReadPosWithChar3 = false;
		getGCSError(); // clear error UNKNOWN COMMAND
	}

	m_bHoming = false;

	ReadPivotSettings();

	return status;
}


asynStatus PIHexapodController::getGlobalState(asynMotorAxis** pAxes, int numAxes)
{
	// do not call moving here, at least simulation software does return
	// values != 0 with bit 1 not set (e.g. "2")
	char buf[255];
    asynStatus status = sendAndReceive(char(5), buf, 99);;
    if (status != asynSuccess)
    {
printf("PIGCSController::getGlobalState() failed, status %d", status);
return status;
    }

    char* pStr;
   long moving = strtol(buf, &pStr, 16);
    m_bAnyAxisMoving = (moving != 0);
    if (m_bHoming && (moving == 0))
    {
    	m_bHoming = false;
    }
    for(int i=0; i<numAxes; i++)
    {
    	((PIasynAxis*)pAxes[i])->m_bMoving = (moving != 0);
    }
    return asynSuccess;

}

asynStatus PIHexapodController::getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl)
{
    negLimit = 0;
   	posLimit = 0;
   	homing = m_bHoming ? 1:0;

   	moving = pAxis->m_bMoving || pAxis->deferred_move;
    return asynSuccess;
}

asynStatus PIHexapodController::findConnectedAxes()
{
	for (size_t i=0; i<MAX_NR_AXES; i++)
	{
		m_axesIDs[i] = NULL;
	}
	sprintf(m_allAxesIDs, "X\nY\nZ\nU\nV\nW");
	char* szAxis = strtok(m_allAxesIDs, "\n");
	while (szAxis != NULL)
	{
		int i=strlen(szAxis)-1;
		while (szAxis[i] == ' ')
		{
			szAxis[i] = '\0';
			i--;
		}
		if (MAX_NR_AXES <= m_nrFoundAxes)
		{
			return asynError;
		}
		m_axesIDs[m_nrFoundAxes] = szAxis;
		m_nrFoundAxes++;
		szAxis = strtok(NULL, "\n");
	}
	return asynSuccess;
}

asynStatus PIHexapodController::initAxis(PIasynAxis* pAxis)
{
    pAxis->m_movingStateMask = 1;
	return setServo(pAxis, 1);
}

asynStatus PIHexapodController::getReferencedState(PIasynAxis* pAxis)
{
	if (!m_bCanReadStatusWithChar4)
	{
	    pAxis->m_homed = 1;
	    return asynSuccess;
	}

	char buf[255];
    asynStatus status = sendAndReceive(char(4), buf, 99);
    if (status != asynSuccess)
    {
    	return status;
    }
    long mask = strtol(buf, NULL, 10);
    pAxis->m_homed = (mask & 0x10000) ? 1 : 0;
    return status;
}

asynStatus PIHexapodController::moveCts( PIasynAxis* pAxis, int targetCts )
{
//	printf("PIHexapodController::moveCts(,%d)...\n",targetCts);
	asynStatus status;
	char cmd[100];
	double target = double(targetCts) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;
    sprintf(cmd,"MOV %s %f", pAxis->m_szAxisName, target);
    status = sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    epicsThreadSleep( 0.2 ); // TODO test if we do need this
    asynMotorAxis* pAsynAxis = (asynMotorAxis*)pAxis;
    status = getGlobalState(&pAsynAxis, 1);
    if (asynSuccess != status)
    {
//    	printf("PIHexapodController::moveCts(,%d) - getGlobalState() failed, status=%d\n", targetCts, status);
    	return status;
    }
    if (!pAxis->m_bMoving)
    {
        int errorCode = getGCSError();
//    	printf("PIHexapodController::moveCts(,%d) - not moving after MOV() gcserror=%d\n",targetCts, errorCode);
        if (errorCode != 0)
        {
        	asynPrint(m_pInterface, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
        		"PIHexapodController::moveCts() failed, GCS error %d\n", errorCode);
        	return asynError;
        }
    }
    m_bAnyAxisMoving = true;
    pAxis->m_lastDirection = (targetCts > pAxis->m_positionCts) ? 1 : 0;
	printf("PIHexapodController::moveCts(,%d) - OK!\n",targetCts);
   	return status;
}

asynStatus PIHexapodController::moveCts( PIasynAxis** pAxesArray, int* pTargetCtsArray, int numAxes)
{
	asynStatus status;
	char cmd[1000] = "MOV";
	char subCmd[100];
	for (int axis = 0; axis <numAxes; axis++)
	{
		PIasynAxis* pAxis = pAxesArray[axis];
		double target = double(pTargetCtsArray[axis]) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;
		sprintf(subCmd," %s %f", pAxis->m_szAxisName, target);
		strcat(cmd, subCmd);
	    pAxis->m_lastDirection = (pTargetCtsArray[axis] > pAxis->m_positionCts) ? 1 : 0;
	}
    status = sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    epicsThreadSleep(0.2);
    status = getGlobalState((asynMotorAxis**)pAxesArray, numAxes);
    if (asynSuccess != status)
    {
    	return status;
    }
    if (!pAxesArray[0]->m_bMoving)
    {
        int errorCode = getGCSError();
        if (errorCode != 0)
        {
        	asynPrint(m_pInterface, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
        		"PIHexapodController::moveCts() failed, GCS error %d\n", errorCode);
        	return asynError;
        }
    }
    m_bAnyAxisMoving = true;
    return status;
}


asynStatus PIHexapodController::getAxisPosition(PIasynAxis* pAxis, double& position)
{
	if (!m_bAnyAxisMoving)
	{
		return PIGCSController::getAxisPosition(pAxis, position);
	}
	if (m_bCanReadPosWithChar3)
	{
		char buf[255];
		asynStatus status = sendAndReceive(char(3), buf, 99);
		if (status != asynSuccess)
		{
			return status;
		}
		char *pStart = buf;
		for(;;)
		{
			bool bEnd = false;
			while(*pStart == ' ') pStart++;
			char *pLF = strstr(pStart, "\n");
			if (pLF != NULL)
			{
				*pLF = '\0';
			}
			else
			{
				bEnd = true;
			}
			// single line will look like "X = 1.0 \n"
			if (pStart[0] == pAxis->m_szAxisName[0] )
			{
				double value;
				if (!getValue(pStart, value))
				{
					return asynError;
				}
				position = value;
				return status;
			}
			if (bEnd)
			{
				break;
			}
			pStart = pLF + 1;
			if (*pStart == '\0')
			{
				break;
			}
		}
		// should not come here!
		return asynError;
	}
	// return last position
    position = double(pAxis->m_positionCts) * double(pAxis->m_CPUdenominator) / double(pAxis->m_CPUnumerator);

	return asynSuccess;
}

asynStatus PIHexapodController::SetPivotX(double value)
{
    if (NULL != m_pCurrentLogSink)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
    			"PIHexapodController::SetPivotX() value %f", value);
    }
    asynStatus status = SetPivot('R', value);
	if (status== asynSuccess)
	{
		m_PivotX = value;
	}
	return status;
}

asynStatus PIHexapodController::SetPivotY(double value)
{
    if (NULL != m_pCurrentLogSink)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIHexapodController::SetPivotY() value %f", value);
    }
    asynStatus status = SetPivot('S', value);
	if (status== asynSuccess)
	{
		m_PivotY = value;
	}
	return status;
}

asynStatus PIHexapodController::SetPivotZ(double value)
{
    if (NULL != m_pCurrentLogSink)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIHexapodController::SetPivotZ() value %f", value);
    }
	asynStatus status = SetPivot('T', value);
	if (status== asynSuccess)
	{
		m_PivotZ = value;
	}
	return status;
}

asynStatus PIHexapodController::SetPivot(char cAxis, double value)
{
	if (m_bAnyAxisMoving)
	{
	    if (NULL != m_pCurrentLogSink)
	    {
	    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
    			"PIHexapodController::SetPivot() cannot change pivot point while platform is moving");
	    }
	    return asynError;
	}

	asynStatus status;
	char cmd[100];
    sprintf(cmd,"SPI %c %f", cAxis, value);
    status = sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }

    int errorCode = getGCSError();
    if (errorCode != 0)
    {
    	asynPrint(m_pInterface, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIHexapodController::SetPivot() failed, GCS error %d\n", errorCode);
    	return asynError;
    }
	return status;

}

asynStatus PIHexapodController::ReadPivotSettings()
{
	char buf[100];
	char cmd[] = "SPI? RST";
	asynStatus status = sendAndReceive(cmd, buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}
	char* pStart = buf;
	bool bEnd = false;
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	for(;;)
	{
		while(*pStart == ' ') pStart++;
		char *pLF = strstr(pStart, "\n");
		if (pLF != NULL)
		{
			*pLF = '\0';
		}
		else
		{
			bEnd = true;
		}
		// single line will look like "R = 1.0 \n"
		double value;
		if (!getValue(pStart, value))
		{
			return asynError;
		}
		switch(pStart[0])
		{
			case'R': px = value; break;
			case'S': py = value; break;
			case'T': pz = value; break;
		}
		if (bEnd)
		{
			break;
		}
		pStart = pLF + 1;
		if (*pStart == '\0')
		{
			break;
		}
	}
	m_PivotX = px;
	m_PivotY = py;
	m_PivotZ = pz;

	return status;
}

asynStatus PIHexapodController::referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards)
{
	asynStatus status = sendOnly("INI X");
	if (asynSuccess != status)
	{
		int errorCode = getGCSError();
	   asynPrint(m_pCurrentLogSink, ASYN_TRACE_ERROR,
	    		"PIHexapodController::referenceVelCts() failed\n");
		epicsSnprintf(pAxis->m_pasynUser->errorMessage,pAxis->m_pasynUser->errorMessageSize,
			"PIHexapodController::referenceVelCts() failed - GCS Error %d\n",errorCode);
		return status;
	}
	m_bHoming = true;
 	return asynSuccess;
}

asynStatus PIHexapodController::haltAxis(PIasynAxis* pAxis)
{
    asynStatus status = sendOnly(char(24));
    if (status != asynSuccess)
    {
    	return status;
    }
    //epicsThreadSleep(0.1); // TODO test value
    int err = getGCSError();
	// controller will set error code to PI_CNTR_STOP (10)
    if (err != PI_CNTR_STOP)
    {
        asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
        		"PIGCSController::haltAxis() failed, GCS error %d", err);
        return asynError;
    }
    return status;
}


