/*
FILENAME...    motorSimController.h
USAGE...    Simulated Motor Support.

Based on drvMotorSim.c

Mark Rivers
March 28, 2010

*/


#include <epicsTime.h>
#include <epicsThread.h>

#include "asynMotorDriver.h"
#include "route.h"

#define NUM_SIM_CONTROLLER_PARAMS 0

class motorSimAxis 
{
public:
    motorSimAxis(class motorSimController *pController, int axis, double lowLimit, double hiLimit, double home, double start);
    asynStatus velocity(double velocity, double acceleration);
    class motorSimController *pController_;
    int axis_;
    ROUTE_ID route_;
    route_reroute_t reroute_;
    route_demand_t endpoint_;
    route_demand_t nextpoint_;
    double lowHardLimit_;
    double hiHardLimit_;
    double enc_offset_;
    double home_;
    int homing_;
    epicsTimeStamp tLast_;
    double deferred_position_;
    int deferred_move_;
    int deferred_relative_;
    friend class motorSimController;
};

class motorSimController : asynMotorController {
public:
    motorSimController(const char *portName, int numAxes, int priority, int stackSize);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    void report(FILE *fp, int level);
    asynStatus moveAxis(asynUser *pasynUser, double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocityAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration);
    asynStatus homeAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stopAxis(asynUser *pasynUser, double acceleration);
    asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
    asynStatus triggerProfile(asynUser *pasynUser);
    asynStatus configAxis(int axis, int hiHardLimit, int lowHardLimit, int home, int start);
    void motorSimTask();  // Should be pivate, but called from non-member function

private:
    motorSimAxis* getAxis(asynUser *pasynUser);
    void process(motorSimAxis *pAxis, double delta );
    asynStatus processDeferredMoves();
    epicsThreadId motorThread_;
    epicsTimeStamp now_;
    int movesDeferred_;
    int numAxes_;
    motorSimAxis** pAxes_;
};
