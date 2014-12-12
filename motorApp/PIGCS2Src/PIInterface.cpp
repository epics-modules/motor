/*
FILENAME...     PIInterface.cpp

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision: 1$
Modified By:    $Author: Steffen Rau$
Last Modified:  $Date: 25.10.2013 10:42:52$
HeadURL:        $URL$

Original Author: Steffen Rau 
Created: 15.12.2010
*/

#include <asynOctetSyncIO.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "PIInterface.h"


//#undef asynPrint
//#define asynPrint(user,reason,format...) 0


extern "C" {
int TranslatePIError(const int error, char* szBuffer, const int maxlen);
}

double PIInterface::TIMEOUT = 5.0;


PIInterface::PIInterface(asynUser* pCom)
: m_pCurrentLogSink (NULL)
, m_pAsynInterface	(pCom)
{
}

PIInterface::~PIInterface()
{
}


asynStatus PIInterface::sendOnly(const char *outputBuff, asynUser* logSink)
{
    size_t nRequested = strlen(outputBuff);
    size_t nActual;
    asynStatus status;

    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIInterface::sendOnly() sending \"%s\"\n", outputBuff);
    //printf("PIInterface::sendOnly() sending \"%s\"\n", outputBuff);

    status = pasynOctetSyncIO->write(m_pAsynInterface, outputBuff,
                                     nRequested, TIMEOUT, &nActual);
    if (nActual != nRequested)
		status = asynError;
    status = pasynOctetSyncIO->write(m_pAsynInterface, "\n",
                                     1, TIMEOUT, &nActual);
    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendOnly: error sending command %s, sent=%d, status=%d\n",
                  outputBuff, nActual, status);
    }
    return(status);
}

asynStatus PIInterface::sendOnly(char c, asynUser* logSink)
{
    size_t nActual;
    asynStatus status;

    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIInterface::sendOnly() sending \"#%d\"\n", int(c));
    //printf("PIInterface::sendOnly() sending \"#%d\"\n", int(c));

    status = pasynOctetSyncIO->write(m_pAsynInterface, &c,
                                     1, TIMEOUT, &nActual);
    if (nActual != 1)
		status = asynError;
    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendOnly: error sending command %d, sent=%d, status=%d\n",
                  int(c), nActual, status);
        //printf("PIInterface::sendOnly() sending \"#%d\"\n", int(c));
    }
    return(status);
}


asynStatus PIInterface::sendAndReceive(const char *outputBuff, char *inputBuff, int inputSize, asynUser* logSink)
{
    size_t nWriteRequested=strlen(outputBuff);
    size_t nWrite, nRead;
    int eomReason;
    asynStatus status;
    size_t pos = 0;
    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIInterface::sendAndReceive() sending \"%s\"\n", outputBuff);
//    //printf("PIInterface::sendAndReceive() sending \"%s\"\n", outputBuff);

    status = pasynOctetSyncIO->write(m_pAsynInterface, outputBuff,
    		nWriteRequested, TIMEOUT, &nWrite);
    if (nWrite != nWriteRequested)
	{
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendAndReceive error calling write, output=%s status=%d, error=%s\n",
                  outputBuff, status, m_pAsynInterface->errorMessage);
    	return asynError;
	}

     status = pasynOctetSyncIO->writeRead(m_pAsynInterface,
                                         "\n", 1,
                                         inputBuff, inputSize,
                                         TIMEOUT, &nWrite, &nRead, &eomReason);
    if (nWrite != 1)
	{
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendAndReceive error calling write, output=%s status=%d, error=%s\n",
                  outputBuff, status, m_pAsynInterface->errorMessage);
    	return asynError;
	}

    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIGCSController:sendAndReceive error calling writeRead, output=%s status=%d, error=%s\n",
                  outputBuff, status, m_pAsynInterface->errorMessage);
    }
    while(inputBuff[strlen(inputBuff)-1] == ' ')
    {
    	inputBuff[strlen(inputBuff)] = '\n';
    	pos += nRead + 1;
        status = pasynOctetSyncIO->read(m_pAsynInterface,
                                             inputBuff+pos, inputSize-pos,
                                             TIMEOUT, &nRead, &eomReason);

    }
    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIInterface::sendAndReceive() received \"%s\"\n", inputBuff);
 //   //printf("PIInterface::sendAndReceive() received \"%s\"\n", inputBuff);

   return(status);
}


asynStatus PIInterface::sendOnly(const char *outputBuff)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pAsynInterface;
	m_interfaceMutex.lock();
	asynStatus status =  sendOnly(outputBuff, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIInterface::sendOnly(char c)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pAsynInterface;
	m_interfaceMutex.lock();
	asynStatus status =  sendOnly(c, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIInterface::sendAndReceive(char c, char *inputBuff, int inputSize)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pAsynInterface;
	m_interfaceMutex.lock();
	asynStatus status = sendAndReceive(c, inputBuff, inputSize, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIInterface::sendAndReceive(const char* output, char *inputBuff, int inputSize)
{
	asynUser* logSink = m_pCurrentLogSink;
	if (NULL == logSink)
		logSink = m_pAsynInterface;
	m_interfaceMutex.lock();
	asynStatus status = sendAndReceive(output, inputBuff, inputSize, logSink);
	m_interfaceMutex.unlock();
	return status;
}

asynStatus PIInterface::sendAndReceive(char c, char *inputBuff, int inputSize, asynUser* logSink)
{
    size_t nWrite, nRead;
    int eomReason;
    asynStatus status;
    size_t pos = 0;

    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIInterface::sendAndReceive() sending \"#%d\"\n", int(c));
    //printf("PIInterface::sendAndReceive() sending \"#%d\"\n", int(c));
    status = pasynOctetSyncIO->writeRead(m_pAsynInterface,
                                         &c, 1,
                                         inputBuff, inputSize,
                                         TIMEOUT, &nWrite, &nRead, &eomReason);
    if (nWrite != 1)
		status = asynError;

    if (status != asynSuccess)
    {
        asynPrint(logSink, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "PIInterface::sendAndReceive error calling writeRead, output=%d status=%d, error=%s\n",
                  int(c), status, m_pAsynInterface->errorMessage);
        //printf("PIInterface::sendAndReceive error calling writeRead, output=%d status=%d, error=%s\n", int(c), status, pInterface->errorMessage);
    }

    while(inputBuff[strlen(inputBuff)-1] == ' ')
    {
    	pos += nRead;
        status = pasynOctetSyncIO->writeRead(m_pAsynInterface,
                                             &c, 1,
                                             inputBuff+pos, inputSize-pos,
                                             TIMEOUT, &nWrite, &nRead, &eomReason);
//printf("PIInterface::sendAndReceive(char) in while loop. inputBuff: \"%s\"\n", inputBuff);
    }
    asynPrint(logSink, ASYN_TRACEIO_DRIVER,
    		"PIInterface::sendAndReceive() received \"%s\"\n", inputBuff);
    //printf("PIInterface::sendAndReceive() received \"%s\" - (0x%02X)\n", inputBuff, int(inputBuff[0]));

    return(status);
}

