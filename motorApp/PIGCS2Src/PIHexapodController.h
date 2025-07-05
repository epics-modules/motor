/*
FILENAME...     PIHexapodController.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
*/

#ifndef PIHEXAPODCONTROLLER_H_
#define PIHEXAPODCONTROLLER_H_

#include "PIGCSController.h"

/**
 * class representing PI Hexapods.
 */
class PIHexapodController : public PIGCSController
{
public:
	PIHexapodController(PIInterface* pInterface, const char* szIDN)
	: PIGCSController(pInterface, szIDN)
	{
	}
	~PIHexapodController() {}

	virtual asynStatus init(void);
	virtual asynStatus initAxis(PIasynAxis* pAxis);

    virtual asynStatus setVelocityCts( PIasynAxis* pAxis, double velocity )
    {
    	return PIGCSController::setVelocityCts(pAxis, velocity);
    }

	virtual asynStatus setAccelerationCts( PIasynAxis* pAxis, double acceleration)	{ return asynSuccess; }
	virtual asynStatus setAcceleration( PIasynAxis* pAxis, double acceleration)	{ return asynSuccess; }
    virtual asynStatus getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl);
    virtual asynStatus getGlobalState(asynMotorAxis** Axes, int numAxes);
    virtual asynStatus getTravelLimits(PIasynAxis* pAxis, double& negLimit, double& posLimit)
    {
    	negLimit = -100;
    	posLimit = 100;
    	return asynSuccess;
    }
    virtual asynStatus getReferencedState(PIasynAxis* pAxis);

	virtual asynStatus haltAxis(PIasynAxis* pAxis);

    virtual bool AcceptsNewTarget() { return !m_bAnyAxisMoving; }
    virtual bool CanCommunicateWhileHoming() { return false; }

	virtual asynStatus moveCts( PIasynAxis* pAxis, int target);
	virtual asynStatus moveCts( PIasynAxis** pAxesArray, int* pTargetCtsArray, int numAxes);
    virtual asynStatus getAxisPosition(PIasynAxis* pAxis, double& position);

    virtual asynStatus SetPivotX(double value);
    virtual asynStatus SetPivotY(double value);
    virtual asynStatus SetPivotZ(double value);

    virtual double GetPivotX() { return m_PivotX; }
    virtual double GetPivotY() { return m_PivotY; }
    virtual double GetPivotZ() { return m_PivotZ; }

    virtual asynStatus referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards);

protected:
    virtual asynStatus findConnectedAxes();
	virtual const char* GetReadPivotCommand() { return "SPI? RST"; }
    asynStatus ReadPivotSettings();
    asynStatus SetPivot(char cAxis, double value);

    double m_PivotX;
    double m_PivotY;
    double m_PivotZ;

    bool m_bHoming;

private:
    bool m_bCanReadStatusWithChar4;
    bool m_bCanReadPosWithChar3;
};

#endif /* PIHEXAPODCONTROLLER_H_ */
