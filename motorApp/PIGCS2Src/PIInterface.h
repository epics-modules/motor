/*
FILENAME...     PIInterface.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
Created: 15.12.2010
*/

#ifndef PIINTERFACE_H_INCLUDED
#define PIINTERFACE_H_INCLUDED

#include <epicsMutex.h>


class PIInterface
{
public:
	PIInterface(asynUser* pCom);
	virtual ~PIInterface();

	asynStatus sendOnly(const char *outputBuff, asynUser* logSink);
	asynStatus sendOnly(char c, asynUser* logSink);

	asynStatus sendAndReceive(const char *outputBuff, char *inputBuff, int inputSize, asynUser* logSink);

	asynStatus sendOnly(const char *outputBuff);
	asynStatus sendOnly(char c);

	asynStatus sendAndReceive(char c, char *inputBuff, int inputSize);
	asynStatus sendAndReceive(const char* output, char *inputBuff, int inputSize);
	asynStatus sendAndReceive(char c, char *inputBuff, int inputSize, asynUser* logSink);

    asynUser* m_pCurrentLogSink;

protected:
    static double TIMEOUT;
	epicsMutex m_interfaceMutex;

	asynUser* m_pAsynInterface;
};

#endif // PIINTERFACE_H_INCLUDED
