/*
 * PIGCScontroller.h
 *
 *  Created on: 15.12.2010
 *      Author: sra
 */
 
/*
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

#ifndef PIGCSCONTROLLER_H_
#define PIGCSCONTROLLER_H_

#include <asynDriver.h>
#include <string.h>
#include <epicsMutex.h>
#include "picontrollererrors.h"


class PIasynAxis;
class asynMotorAxis;

/**
 * Base class for all PI GCS(2) controllers.
 *
 * Most functions will be implemented here, since for basic functionality
 * GCS commands are always the same.
 */
class PIGCSController
{
public:
	PIGCSController(asynUser* pCom, const char* szIDN);
	virtual ~PIGCSController();

	static PIGCSController* CreateGCSController(asynUser* pCom, const char* szIDN);

	virtual asynStatus init(void);
	virtual asynStatus initAxis(PIasynAxis* pAxis);

	static asynStatus sendOnly(asynUser* pInterface, char c, asynUser* logSink);
	static asynStatus sendOnly(asynUser* pInterface, const char *outputBuff, asynUser* logSink);
    static asynStatus sendAndReceive(asynUser* pInterface, const char *outputBuff, char *inputBuff, int inputSize, asynUser* logSink);
    static asynStatus sendAndReceive(asynUser* pInterface, char c, char *inputBuff, int inputSize, asynUser* logSink);

    bool getValue(const char* szMsg, double& value);
    bool getValue(const char* szMsg, int& value);
    bool getValue(const char* szMsg, bool& value);

    virtual asynStatus setVelocityCts( PIasynAxis* pAxis, double velocity );
	virtual asynStatus setAccelerationCts( PIasynAxis* pAxis, double acceleration)	{ return asynSuccess; }
	virtual asynStatus setAcceleration( PIasynAxis* pAxis, double acceleration)	{ return asynSuccess; }
	virtual asynStatus move( PIasynAxis* pAxis, double target);
	virtual asynStatus moveCts( PIasynAxis* pAxis, int target);
	virtual asynStatus moveCts( PIasynAxis** pAxesArray, int* pTargetCtsArray, int numAxes);
	virtual asynStatus referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards) { return asynSuccess;	}
	virtual asynStatus haltAxis(PIasynAxis* pAxis);

    virtual asynStatus getAxisPosition(PIasynAxis* pAxis, double& position);
    virtual asynStatus getAxisVelocity(PIasynAxis* pAxis);
    virtual asynStatus getAxisPositionCts(PIasynAxis* pAxis);
    virtual asynStatus setServo(PIasynAxis* pAxis, int servoState);
    virtual asynStatus getResolution(PIasynAxis* pAxis, double& resolution );
    virtual asynStatus getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl) = 0;
    virtual asynStatus getGlobalState( asynMotorAxis** Axes, int numAxes ) { return asynSuccess; }
    virtual asynStatus getMoving(PIasynAxis* pAxis, int& homing);
    virtual asynStatus getBusy(PIasynAxis* pAxis, int& busy);
    virtual asynStatus getTravelLimits(PIasynAxis* pAxis, double& negLimit, double& posLimit);
    virtual asynStatus hasLimitSwitches(PIasynAxis* pAxis);
    virtual asynStatus hasReferenceSensor(PIasynAxis* pAxis);
    virtual asynStatus getReferencedState(PIasynAxis* axis);

    virtual asynStatus SetPivotX(double value);
    virtual asynStatus SetPivotY(double value);
    virtual asynStatus SetPivotZ(double value);

    virtual double GetPivotX() { return 0.0; }
    virtual double GetPivotY() { return 0.0; }
    virtual double GetPivotZ() { return 0.0; }

    virtual bool AcceptsNewTarget() { return true; }
    virtual bool CanCommunicateWhileHoming() { return true; }

	asynStatus sendOnly(const char *outputBuff);
	asynStatus sendOnly(char c);
    asynStatus sendAndReceive(const char *outputBuff, char *inputBuff, int inputSize);
    asynStatus sendAndReceive(char c, char *inputBuff, int inputSize);

    const char* getAxesID(size_t axisIdx) { return m_axesIDs[axisIdx]; }
    size_t getNrFoundAxes() { return m_nrFoundAxes; }

    virtual bool IsGCS2() { return true; }

    int getGCSError();

    int GetLastError() { return m_LastError; }

    asynUser* m_pCurrentLogSink;
    static const size_t MAX_NR_AXES = 64;
	bool m_bAnyAxisMoving;
protected:
    asynStatus setGCSParameter(PIasynAxis* pAxis, unsigned int paramID, double value);
    asynStatus getGCSParameter(PIasynAxis* pAxis, unsigned int paramID, double& value);

    virtual asynStatus findConnectedAxes();

	asynUser* m_pInterface;
	epicsMutex m_interfaceMutex;
    static double TIMEOUT;
	char szIdentification[200];
	int m_nrAxesOnController;
	char* m_axesIDs[MAX_NR_AXES];
	size_t m_nrFoundAxes;
	char m_allAxesIDs[255];
	int m_LastError;
};

#endif /* PIGCSCONTROLLER_H_ */
