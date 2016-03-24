/*
FILENAME...     PIGCSController.cpp 

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
Created: 15.12.2010
*/

#include <asynOctetSyncIO.h>
#include <math.h>
#include <stdlib.h>
#include "PIGCSController.h"
#include "PIasynAxis.h"
#include "PIInterface.h"
#include "PIGCSMotorController.h"
#include "PIGCSPiezoController.h"
#include "PIE517Controller.h"
#include "PIE755Controller.h"
#include "PIHexapodController.h"
#include "PIGCS2_HexapodController.h"
#include "PIC702Controller.h"


//#undef asynPrint
//#define asynPrint(user,reason,format...) 0


extern "C" {
int TranslatePIError(const int error, char* szBuffer, const int maxlen);
}


/**
 *  create instance of GCS controller depending on identification (\a szIDN)
 */
PIGCSController* PIGCSController::CreateGCSController(PIInterface* pInterface, const char* szIDN)
{
	if ( 		strstr(szIDN, "C-663") != NULL
			||	strstr(szIDN, "C-863") != NULL
			||	strstr(szIDN, "C-867") != NULL
		)
	{
		return new PIGCSMotorController(pInterface, szIDN);
	}
	else if ( strstr(szIDN, "E-517") != NULL)
	{
		return new PIE517Controller(pInterface, szIDN);
	}
	else if ( 		strstr(szIDN, "E-753") != NULL
				||	strstr(szIDN, "E-709") != NULL
				||	strstr(szIDN, "E-725") != NULL
				||	strstr(szIDN, "E-727") != NULL
				)
	{
		return new PIGCSPiezoController(pInterface, szIDN);
	}
	else if ( strstr(szIDN, "E-755") != NULL)
	{
		return new PIE755Controller(pInterface, szIDN);
	}
	else if ( strstr(szIDN, "C-702") != NULL)
	{
		return new PIC702Controller(pInterface, szIDN);
	}
	else if ( 		strstr(szIDN, "HEXAPOD") != NULL
				||	strstr(szIDN, "F-HEX") != NULL
				||	strstr(szIDN, "F-206") != NULL
				||	strstr(szIDN, "M-8") != NULL
				||	strstr(szIDN, "C-887") != NULL
		)
	{
		if (IsGCS2(pInterface))
		{
			return new PIGCS2_HexapodController(pInterface, szIDN);
		}
		else
		{
			return new PIHexapodController(pInterface, szIDN);
		}
	}
	else
	{
		return NULL;
	}
}

bool PIGCSController::IsGCS2(PIInterface* pInterface)
{
	char buf[256];
	asynStatus status = pInterface->sendAndReceive("CSV?", buf, 255);
	if (asynTimeout == status)
	{
		return false;
	}
	else if (asynSuccess != status)
	{
		return false;
	}
	float csv = atof(buf);
	return (csv >= 2.0);
}

asynStatus PIGCSController::setVelocityCts( PIasynAxis* pAxis, double velocity )
{
	char cmd[100];
	velocity = fabs(velocity) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;
    sprintf(cmd,"VEL %s %f", pAxis->m_szAxisName, velocity);
    asynStatus status = m_pInterface->sendOnly(cmd);
    if (asynSuccess == status)
    {
    	pAxis->m_velocity = velocity;
    }
    return status;
}

asynStatus PIGCSController::moveCts( PIasynAxis** pAxesArray, int* pTargetCtsArray, int numAxes)
{
//TODO: 	"Use MVE or set vel/acc so axes reach target at the same time""


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
    status = m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    int errorCode = getGCSError();
    if (errorCode == 0)
    	return asynSuccess;

    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::moveCts(array) failed, GCS error %d\n", errorCode);
    return asynError;
}

asynStatus PIGCSController::setAxisPositionCts(PIasynAxis* pAxis, double positionCts)
{
	double position = double(positionCts) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;

	asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
		"PIGCSController::setAxisPositionCts(, %d) \n", positionCts);
	return setAxisPosition(pAxis, position);
}

asynStatus PIGCSController::setAxisPosition(PIasynAxis* pAxis, double position)
{
	asynStatus status;
	char cmd[100];
    sprintf(cmd,"RON %s 0", pAxis->m_szAxisName);
    status = m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::setAxisPosition() sent \"%s\"\n", cmd);
    sprintf(cmd,"POS %s %f", pAxis->m_szAxisName, position);
    status = m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::setAxisPosition() sent \"%s\"\n", cmd);
    sprintf(cmd,"RON %s 1", pAxis->m_szAxisName);
    status = m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::setAxisPosition() sent \"%s\"\n", cmd);

    int errorCode = getGCSError();
    if (errorCode == 0)
    	return asynSuccess;

    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::setAxisPosition() failed, GCS error %d\n", errorCode);
    return asynError;

}


asynStatus PIGCSController::moveCts( PIasynAxis* pAxis, int targetCts )
{
	double target = double(targetCts) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::moveCts(, %d) \n", targetCts);
	return move(pAxis, target);
}

asynStatus PIGCSController::move( PIasynAxis* pAxis, double target )
{
	asynStatus status;
	char cmd[100];
    sprintf(cmd,"MOV %s %f", pAxis->m_szAxisName, target);
    status = m_pInterface->sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::move() sent \"%s\"\n", cmd);
    pAxis->m_lastDirection = (target > pAxis->m_position) ? 1 : 0;
    int errorCode = getGCSError();
    if (errorCode == 0)
    	return asynSuccess;

    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::move() failed, GCS error %d\n", errorCode);
    return asynError;
}

int PIGCSController::getGCSError()
{
	char buf[256];
	asynStatus status = m_pInterface->sendAndReceive("ERR?", buf, 255);
	if (asynTimeout == status)
	{
		return COM_TIMEOUT;
	}
	else if (asynSuccess != status)
	{
		return COM_ERROR;
	}
	int errorCode = atoi(buf);
	if (0 != errorCode)
	{
		m_LastError = errorCode;
        asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
               "PIGCSController::getGCSError() GCS error code = %d\n",
                  errorCode);
		char szErrorMsg[1024];
		if (TranslatePIError(errorCode, szErrorMsg, 1024))
		{
	        asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
	                  "PIGCSController::getGCSError() GCS error, %s\n",
	                  szErrorMsg);

		}
	}
	return errorCode;
}

asynStatus PIGCSController::haltAxis(PIasynAxis* pAxis)
{
	char cmd[100];
    sprintf(cmd,"HLT %s", pAxis->m_szAxisName);
    asynStatus status = m_pInterface->sendOnly(cmd);
    if (status != asynSuccess)
    {
    	return status;
    }
    int err = getGCSError();
	// controller will set error code to PI_CNTR_STOP (10)
    if (err != PI_CNTR_STOP)
    {
        asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
        		"PIGCSController::haltAxis() failed, GCS error %d", err);
        return asynError;
    }
    return status;
}

/**
 *  get position of axis in physical units (EGU) as defined on the controller
 */
asynStatus PIGCSController::getAxisPosition(PIasynAxis* pAxis, double& position)
{
	char cmd[100];
	char buf[255];
	sprintf(cmd, "POS? %s", pAxis->m_szAxisName);
	asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}
	if (!getValue(buf, position))
	{
		status = asynError;
	}
	return status;
}

/**
 *  get velocity of axis in physical units (EGU) as defined on the controller
 *  and set PIasynAxis::m_velocity
 */
asynStatus PIGCSController::getAxisVelocity(PIasynAxis* pAxis)
{
	char cmd[100];
	char buf[255];
	sprintf(cmd, "VEL? %s", pAxis->m_szAxisName);
	asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
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

/**
 * Find travel range for axis.
 */
asynStatus PIGCSController::getTravelLimits(PIasynAxis* pAxis, double& negLimit, double& posLimit)
{
	char cmd[100];
	char buf[255];
    sprintf(cmd, "TMN? %s", pAxis->m_szAxisName);
    asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
    if (status != asynSuccess)
    {
    	return status;
    }
	if (!getValue(buf, negLimit))
	{
		return asynError;
	}
	sprintf(cmd, "TMX? %s", pAxis->m_szAxisName);
     status = m_pInterface->sendAndReceive(cmd, buf, 99);
     if (status != asynSuccess)
     {
     	return status;
     }
	if (!getValue(buf, posLimit))
	{
		return asynError;
	}

     return status;
}

asynStatus PIGCSController::hasLimitSwitches(PIasynAxis* pAxis)
{
	char cmd[100];
	char buf[255];
     sprintf(cmd, "LIM? %s", pAxis->m_szAxisName);
     asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
     if (status != asynSuccess)
     {
     	return status;
     }
		if (!getValue(buf, pAxis->m_bHasLimitSwitches))
		{
			return asynError;
		}
    if (!pAxis->m_bHasLimitSwitches)
     {
         sprintf(cmd, "HAR? %s", pAxis->m_szAxisName);
         asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
         if (status == asynSuccess)
         {
     		if (!getValue(buf, pAxis->m_bHasLimitSwitches))
     		{
     			return asynError;
     		}
        }
         else if (status == asynTimeout)
         {
        	 int err = getGCSError();
        	 if (err == PI_CNTR_UNKNOWN_COMMAND)
        	 {
            	 // "HAR?" not known
        		 pAxis->m_bHasLimitSwitches = false;
        	 }
        	 else
        	 {
        		 return status;
        	 }
         }
         else
         {
        	 return status;
         }
     }

     asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW,
    		 "PIGCSController::hasLimitSwitches() axis has %slimit switches\n",
    		 pAxis->m_bHasLimitSwitches?"":"no ");
     return status;
}

asynStatus PIGCSController::hasReferenceSensor(PIasynAxis* pAxis)
{
	char cmd[100];
	char buf[255];
    sprintf(cmd, "TRS? %s", pAxis->m_szAxisName);
    asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
    if (status != asynSuccess)
    {
    	return status;
    }

	if (!getValue(buf, pAxis->m_bHasReference))
	{
		return asynError;
	}

    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIGCSController::hasReferenceSwitch() axis has %sreference sensor\n",
   		 pAxis->m_bHasReference?"":"no ");
    return status;
}

/**
 *  get position of axis in counts as used in EPICS.
 *  getAxisPosition() is called and position is covnerted to counts using the
 *  counts-per-unit (CPU) fraction of the axis.
 */
asynStatus PIGCSController::getAxisPositionCts(PIasynAxis* pAxis)
{
	double pos;
	asynStatus status = getAxisPosition(pAxis, pos);
    if (status != asynSuccess)
    {
    	return status;
    }
    pAxis->m_position = pos;
    if (pAxis->m_CPUdenominator==0 || pAxis->m_CPUnumerator==0)
    {
    	pAxis->m_positionCts = pos;
    	return status;
    }

    pAxis->m_positionCts = int( (pos * double(pAxis->m_CPUnumerator) / double(pAxis->m_CPUdenominator))+0.5);
    if (m_pInterface->m_pCurrentLogSink != NULL)
    {
    	asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW,
               "PIGCSController::getAxisPositionCts() pos:%d\n",
               pAxis->m_positionCts);
    }
    return status;
}

//void PIGCSController::calcAxisPositionCts();

asynStatus PIGCSController::setServo(PIasynAxis* pAxis, int servoState)
{
    char cmd[100];
    sprintf(cmd, "SVO %s %d", pAxis->m_szAxisName, servoState);
    asynStatus status = m_pInterface->sendOnly(cmd);
    if (status != asynSuccess)
    {
    	return status;
    }
    int err = getGCSError();
    if (COM_NO_ERROR == err)
    {
    	pAxis->m_bServoControl = (servoState == 1);
    	if (pAxis->m_bProblem && pAxis->m_bServoControl)
    	{
    		pAxis->m_bProblem = false;
    	}
    	return asynSuccess;
    }
    asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
              "Could not set servo state!\n");
	return asynError;

}

asynStatus PIGCSController::getMoving(PIasynAxis* pAxis, int& moving)
{
	char buf[255];
    asynStatus status = m_pInterface->sendAndReceive(char(5), buf, 99);;
    if (status != asynSuccess)
    {
//printf("PIGCSController::getMoving() failed, status %d", status);
return status;
    }

    char* pStr;
    long movingState = strtol(buf, &pStr, 16);
    moving = (movingState & pAxis->m_movingStateMask) != 0 ? 1 : 0;

    return status;
}

asynStatus PIGCSController::getBusy(PIasynAxis* pAxis, int& busy)
{
	char buf[255];
    asynStatus status = m_pInterface->sendAndReceive(char(7), buf, 99);;
    if (status != asynSuccess)
    {
    	return status;
    }

    unsigned char c = (unsigned char)buf[0];
    busy = (c==0xB0);

    return status;
}

asynStatus PIGCSController::setGCSParameter(PIasynAxis* pAxis, unsigned int paramID, double value)
{
    char cmd[100];
    sprintf(cmd, "SPA %s %d %.12g", pAxis->m_szAxisName, paramID, value);
    asynStatus status = m_pInterface->sendOnly(cmd);
    return status;
}

asynStatus PIGCSController::getGCSParameter(PIasynAxis* pAxis, unsigned int paramID, double& value)
{
	char cmd[100];
	char buf[255];
	sprintf(cmd, "SPA? %s %d", pAxis->m_szAxisName, paramID);
	asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}

	if (!getValue(buf, value))
	{
		return asynError;
	}
    return status;

}

PIGCSController::PIGCSController(PIInterface* pInterface, const char* szIDN)
: m_pInterface(pInterface)
, m_bAnyAxisMoving(false)
, m_nrFoundAxes(0)
, m_LastError(0)
{
	strncpy(szIdentification, szIDN, 199);
}

PIGCSController::~PIGCSController()
{
	if (NULL != m_pInterface)
	{
		delete m_pInterface;
	}
}

asynStatus PIGCSController::initAxis(PIasynAxis* pAxis)
{
	// read stage name - to have it in logfile and find
	// problems because of mis/non-configured controllers
	char cmd[100];
	char buf[255];
    sprintf(cmd, "CST? %s", pAxis->m_szAxisName);
    asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);;
    if (status != asynSuccess)
    {
    	return status;
    }
    if (NULL != m_pInterface->m_pCurrentLogSink)
    {
    	asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIGCSController::initAxis() stage configuration: %s\n", buf);
    }
    pAxis->m_movingStateMask = pow(2.0, pAxis->getAxisNo());

	return setServo(pAxis, 1);
}

asynStatus PIGCSController::init(void)
{
	asynStatus status;
	status = findConnectedAxes();
	return status;
}

asynStatus PIGCSController::findConnectedAxes()
{
	m_nrFoundAxes = 0;
	for (size_t i=0; i<MAX_NR_AXES; i++)
	{
		m_axesIDs[i] = NULL;
	}
	asynStatus status = m_pInterface->sendAndReceive("SAI?", m_allAxesIDs, 255);

	if (asynSuccess != status)
	{
		return status;
	}
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
	return status;
}

asynStatus PIGCSController::getReferencedState(PIasynAxis* pAxis)
{
	char cmd[100];
	char buf[255];
    sprintf(cmd, "FRF? %s", pAxis->m_szAxisName);
    asynStatus status = m_pInterface->sendAndReceive(cmd, buf, 99);;
    if (status != asynSuccess)
    {
    	return status;
    }
    if (getValue(buf, pAxis->m_homed))
    {
    	return asynError;
    }
    return status;
}

asynStatus PIGCSController::getResolution(PIasynAxis* pAxis, double& resolution )
{
	resolution = 0.0001;
    pAxis->m_CPUnumerator = 10000;
    pAxis->m_CPUdenominator = 1;
	return asynSuccess;
}

asynStatus PIGCSController::SetPivotX(double value)
{
    if (NULL != m_pInterface->m_pCurrentLogSink)
    {
    	asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIGCSController::SetPivotX() ignored");
    }
	return asynSuccess;
}

asynStatus PIGCSController::SetPivotY(double value)
{
    if (NULL != m_pInterface->m_pCurrentLogSink)
    {
    	asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIGCSController::SetPivotY() ignored");
    }
	return asynSuccess;
}

asynStatus PIGCSController::SetPivotZ(double value)
{
    if (NULL != m_pInterface->m_pCurrentLogSink)
    {
    	asynPrint(m_pInterface->m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIGCSController::SetPivotZ() ignored");
    }
	return asynSuccess;
}

bool PIGCSController::getValue(const char* szMsg, double& value)
{
	const char* p = strstr(szMsg, "=");
	if (p==NULL || *p == '\0')
	{
		return false;
	}
	value = atof(p+1);
	return true;
}

bool PIGCSController::getValue(const char* szMsg, int& value)
{
	const char* p = strstr(szMsg, "=");
	if (p==NULL || *p == '\0')
	{
		return false;
	}
	value = atoi(p+1);
	return true;
}

bool PIGCSController::getValue(const char* szMsg, bool& value)
{
	const char* p = strstr(szMsg, "=");
	if (p==NULL || *p == '\0')
	{
		return false;
	}
	int ivalue = atoi(p+1);
	value = (ivalue =! 0);
	return true;
}
