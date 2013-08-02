/*
FILENAME...     PIE755Controller.cpp

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$

Original Author: Steffen Rau 
*/

#include "PIE755Controller.h"
#include "PIasynAxis.h"
#include <stdlib.h>

#undef asynPrint
#define asynPrint(user,reason,format...) 0


asynStatus PIE755Controller::getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl)
{
	int busy;
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
