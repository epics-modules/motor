/*
FILENAME...     PIGCScontroller.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
Created: 15.12.2010
*/

#ifndef PIGCSPIEZOCONTROLLER_H_
#define PIGCSPIEZOCONTROLLER_H_

#include "PIGCSController.h"
#include <asynDriver.h>

/**
 * Class representing PI's GCS2 digital piezo controllers.
 *
 * Main difference to motor controllers is the usage of absolute sensors.
 */
class PIGCSPiezoController : public PIGCSController
{
public:
	PIGCSPiezoController(PIInterface* pInterface, const char* szIDN)
	: PIGCSController(pInterface, szIDN)
	{
	}
	~PIGCSPiezoController() {}

	virtual asynStatus init(void) { return PIGCSController::init(); }
	virtual asynStatus initAxis(PIasynAxis* pAxis);
	virtual asynStatus haltAxis(PIasynAxis* pAxis);

    virtual asynStatus getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl);
    virtual asynStatus getReferencedState(PIasynAxis* pAxis);


private:

};

#endif /* PIGCSPIEZOCONTROLLER_H_ */
