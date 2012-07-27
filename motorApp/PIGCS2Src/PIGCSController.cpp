/*
 * PIGCSController.cpp
 *
 *  Created on: 15.12.2010
 *      Author: sra
 */
 
/*
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

#include <asynOctetSyncIO.h>
#include <math.h>
#include <stdlib.h>
#include "PIGCSController.h"
#include "PIasynAxis.h"
#include "PIGCSMotorController.h"
#include "PIGCSPiezoController.h"
#include "PIE517Controller.h"
#include "PIE755Controller.h"
#include "PIHexapodController.h"
#include "PIC702Controller.h"


//#undef asynPrint
//#define asynPrint(user,reason,format...) 0


extern "C" {
int TranslatePIError(const int error, char* szBuffer, const int maxlen);
}

double PIGCSController::TIMEOUT = 5.0;

/**
 *  create instance of GCS controller depending on identification (\a szIDN)
 */
PIGCSController* PIGCSController::CreateGCSController(asynUser* pInterface, const char* szIDN)
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
		return new PIHexapodController(pInterface, szIDN);
	}
	else
	{
		return NULL;
	}
}

asynStatus PIGCSController::sendOnly(asynUser* pInterface, const char *outputBuff, asynUser* logSink)
{
    size_t nRequested=strlen(outputBuff);
    size_t nActual;
    asynStatus status;

    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIGCSController::sendOnly() sending \"%s\"\n", outputBuff);
    //printf("PIGCSController::sendOnly() sending \"%s\"\n", outputBuff);

    status = pasynOctetSyncIO->write(pInterface, outputBuff,
                                     nRequested, TIMEOUT, &nActual);
    if (nActual != nRequested)
		status = asynError;
    status = pasynOctetSyncIO->write(pInterface, "\n",
                                     1, TIMEOUT, &nActual);
    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendOnly: error sending command %s, sent=%d, status=%d\n",
                  outputBuff, nActual, status);
    }
    return(status);
}

asynStatus PIGCSController::sendOnly(asynUser* pInterface, char c, asynUser* logSink)
{
    size_t nActual;
    asynStatus status;

    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIGCSController::sendOnly() sending \"#%d\"\n", int(c));
    //printf("PIGCSController::sendOnly() sending \"#%d\"\n", int(c));

    status = pasynOctetSyncIO->write(pInterface, &c,
                                     1, TIMEOUT, &nActual);
    if (nActual != 1)
		status = asynError;
    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendOnly: error sending command %d, sent=%d, status=%d\n",
                  int(c), nActual, status);
        //printf("PIGCSController::sendOnly() sending \"#%d\"\n", int(c));
    }
    return(status);
}


asynStatus PIGCSController::sendAndReceive(asynUser* pInterface, const char *outputBuff, char *inputBuff, int inputSize, asynUser* logSink)
{
    size_t nWriteRequested=strlen(outputBuff);
    size_t nWrite, nRead;
    int eomReason;
    asynStatus status;
    size_t pos = 0;
    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIGCSController::sendAndReceive() sending \"%s\"\n", outputBuff);
//    //printf("PIGCSController::sendAndReceive() sending \"%s\"\n", outputBuff);

    status = pasynOctetSyncIO->write(pInterface, outputBuff,
    		nWriteRequested, TIMEOUT, &nWrite);
    if (nWrite != nWriteRequested)
	{
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendAndReceive error calling write, output=%s status=%d, error=%s\n",
                  outputBuff, status, pInterface->errorMessage);
    	return asynError;
	}

     status = pasynOctetSyncIO->writeRead(pInterface,
                                         "\n", 1,
                                         inputBuff, inputSize,
                                         TIMEOUT, &nWrite, &nRead, &eomReason);
    if (nWrite != 1)
	{
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendAndReceive error calling write, output=%s status=%d, error=%s\n",
                  outputBuff, status, pInterface->errorMessage);
    	return asynError;
	}

    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendAndReceive error calling writeRead, output=%s status=%d, error=%s\n",
                  outputBuff, status, pInterface->errorMessage);
    }
    while(inputBuff[strlen(inputBuff)-1] == ' ')
    {
    	inputBuff[strlen(inputBuff)] = '\n';
    	pos += nRead + 1;
        status = pasynOctetSyncIO->read(logSink,
                                             inputBuff+pos, inputSize-pos,
                                             TIMEOUT, &nRead, &eomReason);

    }
    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIGCSController::sendAndReceive() received \"%s\"\n", inputBuff);
 //   //printf("PIGCSController::sendAndReceive() received \"%s\"\n", inputBuff);

   return(status);
}

asynStatus PIGCSController::sendOnly(const char *outputBuff)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pInterface;
	m_interfaceMutex.lock();
	asynStatus status =  sendOnly(m_pInterface, outputBuff, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIGCSController::sendOnly(char c)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pInterface;
	m_interfaceMutex.lock();
	asynStatus status =  sendOnly(m_pInterface, c, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIGCSController::sendAndReceive(char c, char *inputBuff, int inputSize)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pInterface;
	m_interfaceMutex.lock();
	asynStatus status = sendAndReceive(m_pInterface, c, inputBuff, inputSize, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIGCSController::sendAndReceive(const char* output, char *inputBuff, int inputSize)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pInterface;
	m_interfaceMutex.lock();
	asynStatus status = sendAndReceive(m_pInterface, output, inputBuff, inputSize, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIGCSController::sendAndReceive(asynUser* pInterface, char c, char *inputBuff, int inputSize, asynUser* logSink)
{
    size_t nWrite, nRead;
    int eomReason;
    asynStatus status;
    size_t pos = 0;

    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIGCSController::sendAndReceive() sending \"#%d\"\n", int(c));
    //printf("PIGCSController::sendAndReceive() sending \"#%d\"\n", int(c));
    status = pasynOctetSyncIO->writeRead(pInterface,
                                         &c, 1,
                                         inputBuff, inputSize,
                                         TIMEOUT, &nWrite, &nRead, &eomReason);
    if (nWrite != 1)
		status = asynError;

    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController::sendAndReceive error calling writeRead, output=%d status=%d, error=%s\n",
                  int(c), status, pInterface->errorMessage);
        //printf("PIGCSController::sendAndReceive error calling writeRead, output=%d status=%d, error=%s\n", int(c), status, pInterface->errorMessage);
    }

    while(inputBuff[strlen(inputBuff)-1] == ' ')
    {
    	pos += nRead;
        status = pasynOctetSyncIO->writeRead(logSink,
                                             &c, 1,
                                             inputBuff+pos, inputSize-pos,
                                             TIMEOUT, &nWrite, &nRead, &eomReason);
//printf("PIGCSController::sendAndReceive(char) in while loop. inputBuff: \"%s\"\n", inputBuff);
    }
    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIGCSController::sendAndReceive() received \"%s\"\n", inputBuff);
    //printf("PIGCSController::sendAndReceive() received \"%s\" - (0x%02X)\n", inputBuff, int(inputBuff[0]));

    return(status);
}

asynStatus PIGCSController::setVelocityCts( PIasynAxis* pAxis, double velocity )
{
	char cmd[100];
	velocity = fabs(velocity) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;
    sprintf(cmd,"VEL %s %f", pAxis->m_szAxisName, velocity);
    asynStatus status = sendOnly(cmd);
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
    status = sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    int errorCode = getGCSError();
    if (errorCode == 0)
    	return asynSuccess;

    asynPrint(m_pInterface, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::moveCts(array) failed, GCS error %d\n", errorCode);
    return asynError;
}



asynStatus PIGCSController::moveCts( PIasynAxis* pAxis, int targetCts )
{
	asynStatus status;
	char cmd[100];
	double target = double(targetCts) * pAxis->m_CPUdenominator / pAxis->m_CPUnumerator;
    asynPrint(m_pInterface, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::moveCts(, %d) \n", targetCts);
	return move(pAxis, target);
}

asynStatus PIGCSController::move( PIasynAxis* pAxis, double target )
{
	asynStatus status;
	char cmd[100];
    sprintf(cmd,"MOV %s %f", pAxis->m_szAxisName, target);
    status = sendOnly(cmd);
    if (asynSuccess != status)
    {
    	return status;
    }
    asynPrint(m_pInterface, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::move() sent \"%s\"\n", cmd);
    pAxis->m_lastDirection = (target > pAxis->m_position) ? 1 : 0;
    int errorCode = getGCSError();
    if (errorCode == 0)
    	return asynSuccess;

    asynPrint(m_pInterface, ASYN_TRACE_FLOW|ASYN_TRACE_ERROR,
    		"PIGCSController::move() failed, GCS error %d\n", errorCode);
    return asynError;
}

int PIGCSController::getGCSError()
{
	char buf[256];
	asynStatus status = sendAndReceive("ERR?", buf, 255);
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
        asynPrint(m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
               "PIGCSController::getGCSError() GCS error code = %d\n",
                  errorCode);
		char szErrorMsg[1024];
		if (TranslatePIError(errorCode, szErrorMsg, 1024))
		{
	        asynPrint(m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
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
    asynStatus status = sendOnly(cmd);
    if (status != asynSuccess)
    {
    	return status;
    }
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

/**
 *  get position of axis in physical units (EGU) as defined on the controller
 */
asynStatus PIGCSController::getAxisPosition(PIasynAxis* pAxis, double& position)
{
	char cmd[100];
	char buf[255];
	sprintf(cmd, "POS? %s", pAxis->m_szAxisName);
	asynStatus status = sendAndReceive(cmd, buf, 99);
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
	asynStatus status = sendAndReceive(cmd, buf, 99);
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
    asynStatus status = sendAndReceive(cmd, buf, 99);
    if (status != asynSuccess)
    {
    	return status;
    }
	if (!getValue(buf, negLimit))
	{
		return asynError;
	}
	sprintf(cmd, "TMX? %s", pAxis->m_szAxisName);
     status = sendAndReceive(cmd, buf, 99);
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
     asynStatus status = sendAndReceive(cmd, buf, 99);
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
         asynStatus status = sendAndReceive(cmd, buf, 99);
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

     asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
    		 "PIGCSController::hasLimitSwitches() axis has %slimit switches\n",
    		 pAxis->m_bHasLimitSwitches?"":"no ");
     return status;
}

asynStatus PIGCSController::hasReferenceSensor(PIasynAxis* pAxis)
{
	char cmd[100];
	char buf[255];
    sprintf(cmd, "TRS? %s", pAxis->m_szAxisName);
    asynStatus status = sendAndReceive(cmd, buf, 99);
    if (status != asynSuccess)
    {
    	return status;
    }

	if (!getValue(buf, pAxis->m_bHasReference))
	{
		return asynError;
	}

    asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
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
    if (m_pCurrentLogSink != NULL)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
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
    asynStatus status = sendOnly(cmd);
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
    asynPrint(m_pCurrentLogSink, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
              "Could not set servo state!\n");
	return asynError;

}

asynStatus PIGCSController::getMoving(PIasynAxis* pAxis, int& moving)
{
	char buf[255];
    asynStatus status = sendAndReceive(char(5), buf, 99);;
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
    asynStatus status = sendAndReceive(char(7), buf, 99);;
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
    asynStatus status = sendOnly(cmd);
    return status;
}

asynStatus PIGCSController::getGCSParameter(PIasynAxis* pAxis, unsigned int paramID, double& value)
{
	char cmd[100];
	char buf[255];
	sprintf(cmd, "SPA? %s %d", pAxis->m_szAxisName, paramID);
	asynStatus status = sendAndReceive(cmd, buf, 99);
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

PIGCSController::PIGCSController(asynUser* pCom, const char* szIDN)
: m_pCurrentLogSink(NULL)
, m_bAnyAxisMoving(false)
, m_pInterface(pCom)
, m_nrFoundAxes(0)
, m_LastError(0)
{
	strncpy(szIdentification, szIDN, 199);
}

PIGCSController::~PIGCSController()
{
}

asynStatus PIGCSController::initAxis(PIasynAxis* pAxis)
{
	// read stage name - to have it in logfile and find
	// problems because of mis/non-configured controllers
	char cmd[100];
	char buf[255];
    sprintf(cmd, "CST? %s", pAxis->m_szAxisName);
    asynStatus status = sendAndReceive(cmd, buf, 99);;
    if (status != asynSuccess)
    {
    	return status;
    }
    if (NULL != m_pCurrentLogSink)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
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
	asynStatus status = PIGCSController::sendAndReceive("SAI?", m_allAxesIDs, 255);

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
    asynStatus status = sendAndReceive(cmd, buf, 99);;
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
    if (NULL != m_pCurrentLogSink)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIGCSController::SetPivotX() ignored");
    }
	return asynSuccess;
}

asynStatus PIGCSController::SetPivotY(double value)
{
    if (NULL != m_pCurrentLogSink)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
   		 "PIGCSController::SetPivotY() ignored");
    }
	return asynSuccess;
}

asynStatus PIGCSController::SetPivotZ(double value)
{
    if (NULL != m_pCurrentLogSink)
    {
    	asynPrint(m_pCurrentLogSink, ASYN_TRACE_FLOW,
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
