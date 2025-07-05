/*
FILENAME...     PIHexapodController.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
*/

#ifndef PIGCS2_HEXAPODCONTROLLER_H_
#define PIGCS2_HEXAPODCONTROLLER_H_

#include "PIHexapodController.h"

/**
 * class representing PI GCS2 Hexapods.
 */
class PIGCS2_HexapodController : public PIHexapodController
{
public:
	PIGCS2_HexapodController(PIInterface* pInterface, const char* szIDN)
	: PIHexapodController(pInterface, szIDN)
	{
	}
	~PIGCS2_HexapodController() {}

    virtual asynStatus setVelocityCts( PIasynAxis* pAxis, double velocity );
    virtual asynStatus getAxisVelocity(PIasynAxis* pAxis);
	virtual asynStatus referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards);
    virtual asynStatus getAxisPosition(PIasynAxis* pAxis, double& position)
    {
    	return PIGCSController::getAxisPosition(pAxis, position);
    }
    virtual bool AcceptsNewTarget() { return true; }
    virtual bool CanCommunicateWhileHoming() { return true; }

protected:
	virtual const char* GetReadPivotCommand() { return "SPI? R S T"; }
private:
};

#endif /* PIGCS2_HEXAPODCONTROLLER_H_ */
