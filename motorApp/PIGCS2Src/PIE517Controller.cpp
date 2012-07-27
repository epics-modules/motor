/*
 * PIGCSPiezoController
 *
 *  Created on: 15.12.2010
 *      Author: sra
 */

#include "PIE517Controller.h"
#include "PIasynAxis.h"
#include <stdlib.h>
#include <math.h>

#undef asynPrint
#define asynPrint(user,reason,format...) 0


asynStatus PIE517Controller::init()
{
	asynStatus status;
	status = PIGCSPiezoController::init();
	if (asynSuccess != status)
	{
		return status;
	}

	// all output channels need to be set "online" before
	// any commands are accepted over the interface.
	status = getNrOutputChannels();
	if (asynSuccess != status)
	{
		return status;
	}
	for (int ch=1; ch<=m_nrOutputChannels; ch++)
	{
		status = setOnline(ch, 1);
		if (asynSuccess != status)
		{
			return status;
		}
	}

	return status;
}

asynStatus PIE517Controller::setOnline(int channel, int onlineState)
{
    char cmd[100];
    sprintf(cmd, "ONL %d %d", channel, onlineState);
    asynStatus status = sendOnly(cmd);
    return status;
}

asynStatus PIE517Controller::getNrOutputChannels()
{
	char buf[255];
	asynStatus status = sendAndReceive("TPC?", buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}
	m_nrOutputChannels = atoi(buf);
    return status;

}

asynStatus PIE517Controller::initAxis(PIasynAxis* pAxis)
{
    pAxis->m_movingStateMask = pow(2.0, pAxis->getAxisNo());

	return setServo(pAxis, 1);
}


