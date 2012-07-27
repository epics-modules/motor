/*
FILENAME...    PIasynController.cpp
USAGE...    Simulated Motor Support.

Based on drvMotorSim.c

Mark Rivers
December 13, 2009

*/

#ifndef PI_ASYN_DRIVER_INCLUDED_
#define PI_ASYN_DRIVER_INCLUDED_

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class PIasynAxis;
class PIGCSController;

class PIasynController : asynMotorController {
public:
    PIasynController(const char *portName, const char* asynPort, int numAxes, int priority, int stackSize, int movingPollPeriod, int idlePollPeriod);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    void report(FILE *fp, int level);
    asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
    asynStatus triggerProfile(asynUser *pasynUser);
    asynStatus configAxis(PIasynAxis *pAxis);

    PIasynAxis* getPIAxis(asynUser *pasynUser) { return (PIasynAxis*)asynMotorController::getAxis(pasynUser); }
    PIasynAxis* getPIAxis(int axisNo) { return (PIasynAxis*)asynMotorController::getAxis(axisNo); }

    virtual asynStatus poll();

    friend class PIasynAxis;

private:
//    void process(PIasynAxis *pAxis);
    epicsThreadId motorThread;
    int movesDeferred;
    //int numAxes;
    asynStatus processDeferredMoves();
    //PIasynAxis** m_pAxes;
    
    PIGCSController* m_pGCSController;

    int PI_SUP_POSITION;
    int PI_SUP_TARGET;
    int PI_SUP_SERVO;
    int PI_SUP_LAST_ERR;
    int PI_SUP_PIVOT_X;
    int PI_SUP_PIVOT_Y;
    int PI_SUP_PIVOT_Z;
    int PI_SUP_RBPIVOT_X;
    int PI_SUP_RBPIVOT_Y;
    int PI_SUP_RBPIVOT_Z;

};


#define PI_SUP_POSITION_String		"PI_SUP_POSITION"
#define PI_SUP_TARGET_String		"PI_SUP_TARGET"
#define PI_SUP_SERVO_String			"PI_SUP_SERVO"
#define PI_SUP_LAST_ERR_String		"PI_SUP_LAST_ERR"
#define PI_SUP_PIVOT_X_String		"PI_SUP_PIVOT_X"
#define PI_SUP_PIVOT_Y_String		"PI_SUP_PIVOT_Y"
#define PI_SUP_PIVOT_Z_String		"PI_SUP_PIVOT_Z"
#define PI_SUP_RBPIVOT_X_String		"PI_SUP_RBPIVOT_X"
#define PI_SUP_RBPIVOT_Y_String		"PI_SUP_RBPIVOT_Y"
#define PI_SUP_RBPIVOT_Z_String		"PI_SUP_RBPIVOT_Z"


typedef struct PIasynControllerNode {
    ELLNODE node;
    const char *portName;
    PIasynController *pController;
} PIasynControllerNode;

#endif // PI_ASYN_DRIVER_INCLUDED_
