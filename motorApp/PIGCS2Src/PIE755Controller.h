/*
FILENAME...     PIE755Controller.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
*/

#ifndef PIE755CONTROLLER_H_
#define PIE755CONTROLLER_H_

#include "PIGCSMotorController.h"

/**
 * class representing PI E-755.
 *
 * These controllers share most of the parameters with digital piezo controllers.
 * Encoders for Nexline stages are often only incremental, so they need to be homed.
 * Homing works with "hard stops" and "Limit switches".
 * From an EPICS point of view this behaves more like a motor than a piezo controller.
 */
class PIE755Controller : public PIGCSMotorController
{
public:
	PIE755Controller(PIInterface* pInterface, const char* szIDN)
	: PIGCSMotorController(pInterface, szIDN)
	{
	}
	~PIE755Controller() {}

    virtual asynStatus getResolution(PIasynAxis* pAxis, double& resolution )
    {
    	return PIGCSController::getResolution(pAxis, resolution);
    }
	virtual asynStatus setAccelerationCts( PIasynAxis* pAxis, double acceleration)	{ return asynSuccess; }
	virtual asynStatus setAcceleration( PIasynAxis* pAxis, double acceleration)	{ return asynSuccess; }
    virtual asynStatus getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl);

private:

};

#endif /* PIE755CONTROLLER_H_ */
