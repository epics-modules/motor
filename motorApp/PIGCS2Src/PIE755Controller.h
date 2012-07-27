/*
 * PIE755Controller.h
 *
 *      Author: sra
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
	PIE755Controller(asynUser* pCom, const char* szIDN)
	: PIGCSMotorController(pCom, szIDN)
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
