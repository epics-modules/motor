/*
FILENAME...     PIGCSPiezoController 

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision: 3$
Modified By:    $Author: Steffen Rau$
Last Modified:  $Date: 25.10.2013 10:43:08$
HeadURL:        $URL$

Original Author: Steffen Rau 
Created: 15.12.2010
*/

#include "PIE517Controller.h"
#include "PIasynAxis.h"
#include "PIInterface.h"
#include <stdlib.h>
#include <math.h>

//#undef asynPrint
//#define asynPrint(user,reason,format...) 0


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
    asynStatus status = m_pInterface->sendOnly(cmd);
    return status;
}

asynStatus PIE517Controller::getNrOutputChannels()
{
	char buf[255];
	asynStatus status = m_pInterface->sendAndReceive("TPC?", buf, 99);
	if (status != asynSuccess)
	{
		return status;
	}
	m_nrOutputChannels = atoi(buf);
    return status;

}


