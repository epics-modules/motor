/*
FILENAME...     PIasynAxis.h
USAGE...        PI GCS Motor Support.
 
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
Created: January 2011

Based on drvMotorSim.c, Mark Rivers, December 13, 2009
*/


#include <asynDriver.h> // for asynStatus
#include <asynMotorAxis.h>

class PIasynController;
class PIGCSController;

class PIasynAxis : public asynMotorAxis
{
public:
    PIasynAxis(class PIasynController *pController, PIGCSController* pGCSController, int axis, const char* szName);
    virtual~PIasynAxis();

    void Init(const char *portName);

    class PIasynController *pController_;
    int getAxisNo() { return axisNo_; }

    virtual asynStatus poll(bool *moving);
    virtual asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
    virtual asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
    virtual asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    virtual asynStatus stop(double acceleration);


    char* m_szAxisName;			///< GCS name

    int m_isHoming;				///< if \b TRUE indicating that axis is currently homing/referencing
    double deferred_position;   ///< currently not used
    int deferred_move;   		///< currently not used
    int deferred_relative;   	///< currently not used
    int m_homed;   				///< if \b TRUE axis was homed and absolute positions are correct

    double m_velocity;
    double m_acceleration;
    double m_maxAcceleration;
    int m_positionCts;
    double m_position;
    int m_lastDirection;

    int m_CPUnumerator;
    int m_CPUdenominator;

    asynUser* m_pasynUser;

    bool m_bHasLimitSwitches;
    bool m_bHasReference;
    bool m_bProblem;
    bool m_bServoControl;
    bool m_bMoving;
    int m_movingStateMask;

    friend class PIasynController;
private:

    double negLimit_;
    double posLimit_;
    PIGCSController* m_pGCSController;

};

