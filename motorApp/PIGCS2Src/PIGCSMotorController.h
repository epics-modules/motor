/*
FILENAME...     PIGCSMotorController.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
Created: 15.12.2010
*/

#ifndef PIGCSMOTORCONTROLLER_H_
#define PIGCSMOTORCONTROLLER_H_

#include "PIGCSController.h"
#include <asynDriver.h>

/**
 * class representing PI GCS2 motor controllers.
 *
 * "motor" does not strictly restrict the driving principle to DC stages or
 * stepper but includes other controller families (e.g. for piezo motors) also.
 */
class PIGCSMotorController : public PIGCSController
{
public:
	PIGCSMotorController(PIInterface* pInterface, const char* szIDN)
	: PIGCSController(pInterface, szIDN)
	{
	}
	~PIGCSMotorController() {}
	virtual asynStatus initAxis(PIasynAxis* pAxis);

	virtual asynStatus setAccelerationCts( PIasynAxis* pAxis, double acceleration);
	virtual asynStatus setAcceleration( PIasynAxis* pAxis, double acceleration);
	virtual asynStatus getMaxAcceleration( PIasynAxis* pAxis );
	virtual asynStatus referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards);
    virtual asynStatus getResolution(PIasynAxis* pAxis, double& resolution );
    virtual asynStatus getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl);

protected:
    enum
    {
    	PI_PARA_MOT_CURR_ACCEL		= 0x000000BUL,
    	PI_PARA_MOT_CURR_DECEL		= 0x000000CUL,
    	PI_PARA_MOT_CPU_Z			= 0x000000EUL,
    	PI_PARA_MOT_CPU_N			= 0x000000FUL,
    	PI_PARA_MOT_HAT_REF			= 0x0000014UL,
    	PI_PARA_MOT_MAX_ACCEL		= 0x000004AUL,
    	PI_PARA_MOT_MAX_DECEL		= 0x000004BUL
    };

private:

};

#endif /* PIGCSMOTORCONTROLLER_H_ */
