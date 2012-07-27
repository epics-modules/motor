/*
 * PIC702Controller.h
 *
 *      Author: sra
 */

#ifndef PIC702CONTROLLER_H_
#define PIC702CONTROLLER_H_

#include "PIGCSMotorController.h"
#include <epicsTime.h>

/**
 * class representing PI C-702.
 *
 */
class PIC702Controller : public PIGCSMotorController
{
public:
	PIC702Controller(asynUser* pCom, const char* szIDN)
	: PIGCSMotorController(pCom, szIDN)
	{
	}
	~PIC702Controller() {}

    virtual asynStatus getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl);
	virtual asynStatus getMaxAcceleration( PIasynAxis* pAxis );
    virtual asynStatus hasReferenceSensor(PIasynAxis* pAxis);
    virtual asynStatus getReferencedState(PIasynAxis* axis);
	virtual asynStatus referenceVelCts( PIasynAxis* pAxis, double velocity, int forwards);

    virtual bool IsGCS2() { return false; }

protected:
    virtual asynStatus findConnectedAxes();
    enum
    {
    	PI_PARA_C702_REFERENCED		= 0x000001CUL
    };

private:
    epicsTimeStamp m_timeREFstarted;
};

#endif /* PIC702CONTROLLER_H_ */
