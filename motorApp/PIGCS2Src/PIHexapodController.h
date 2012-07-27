/*
 * PIE755Controller.h
 *
 *      Author: sra
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
	PIHexapodController(asynUser* pCom, const char* szIDN)
	: PIGCSController(pCom, szIDN)
	{
	}
	~PIHexapodController() {}

	virtual asynStatus init(void);
	virtual asynStatus initAxis(PIasynAxis* pAxis);

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
