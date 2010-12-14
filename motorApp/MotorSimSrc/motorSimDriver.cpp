/*
FILENAME...    motorSimController.cpp
USAGE...    Simulated Motor Support.

Based on drvMotorSim.c

Mark Rivers
December 13, 2009

*/


#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <iocsh.h>

#include "asynMotorDriver.h"
#include <epicsExport.h>

#include "route.h"

#define DEFAULT_LOW_LIMIT -10000
#define DEFAULT_HI_LIMIT   10000
#define DEFAULT_HOME       0
#define DEFAULT_START      0

#define NUM_SIM_CONTROLLER_PARAMS 0

static const char *driverName = "motorSimDriver";

class motorSimAxis 
{
public:
    motorSimAxis(class motorSimController *pController, int axis, double lowLimit, double hiLimit, double home, double start);
    asynStatus velocity(double velocity, double acceleration);
    class motorSimController *pController;
    int axis;
    ROUTE_ID route;
    route_reroute_t reroute;
    route_demand_t endpoint;
    route_demand_t nextpoint;
    double lowHardLimit;
    double hiHardLimit;
    double enc_offset;
    double home;
    int homing;
    epicsTimeStamp tLast;
    double deferred_position;
    int deferred_move;
    int deferred_relative;
    friend class motorSimController;
};

class motorSimController : asynMotorDriver {
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
    epicsThreadId motorThread;
    epicsTimeStamp now;
    int movesDeferred;
    int numAxes;
    asynStatus processDeferredMoves();
    motorSimAxis** pAxes;
};

typedef struct motorSimControllerNode {
    ELLNODE node;
    const char *portName;
    motorSimController *pController;
} motorSimControllerNode;

static ELLLIST motorSimControllerList;
static int motorSimControllerListInitialized = 0;

void motorSimController::report(FILE *fp, int level)
{
    int axis;
    motorSimAxis *pAxis;

    fprintf(fp, "Simulation motor driver %s, numAxes=%d\n", 
        this->portName, this->numAxes);

    for (axis=0; axis<this->numAxes; axis++) {
        pAxis = this->pAxes[axis];
        fprintf(fp, "  axis %d\n", 
            pAxis->axis);

        if (level > 0)
        {
            double lowSoftLimit=0.0;
            double hiSoftLimit=0.0;

            fprintf(fp, "    Current position = %f, velocity = %f at current time: %f\n", 
                   pAxis->nextpoint.axis[0].p, 
                   pAxis->nextpoint.axis[0].v,
                   pAxis->nextpoint.T);
            fprintf(fp, "    Destination posn = %f, velocity = %f at desination time:  %f\n",
                   pAxis->endpoint.axis[0].p, 
                   pAxis->endpoint.axis[0].v,
                   pAxis->endpoint.T);

            fprintf(fp, "    Hard limits: %f, %f\n", pAxis->lowHardLimit, pAxis->hiHardLimit);
            fprintf(fp, "           Home: %f\n", pAxis->home);
            fprintf(fp, "    Enc. offset: %f\n", pAxis->enc_offset);
            getDoubleParam(pAxis->axis, pAxis->pController->motorHighLim, &hiSoftLimit);
            getDoubleParam(pAxis->axis, pAxis->pController->motorLowLim, &lowSoftLimit);
            fprintf(fp, "    Soft limits: %f, %f\n", lowSoftLimit, hiSoftLimit );

            if (pAxis->homing) fprintf(fp, "    Currently homing axis\n" );
        }
    }

    // Call the base class method
    asynMotorDriver::report(fp, level);
}

motorSimAxis * motorSimController::getAxis(asynUser *pasynUser)
{
    int axis;
    motorSimAxis *pAxis;
    
    getAddress(pasynUser, &axis);
    pAxis = this->pAxes[axis];
    return(pAxis);
}
    

asynStatus motorSimController::processDeferredMoves()
{
    asynStatus status = asynError;
    double position = 0.0;
    int axis;
    motorSimAxis *pAxis;

    for (axis=0; axis<this->numAxes; axis++)
    {
        pAxis = this->pAxes[axis];
        if (pAxis->deferred_move) {
            position = pAxis->deferred_position;
            /* Check to see if in hard limits */
            if ((pAxis->nextpoint.axis[0].p >= pAxis->hiHardLimit  &&  position > pAxis->nextpoint.axis[0].p) ||
                (pAxis->nextpoint.axis[0].p <= pAxis->lowHardLimit &&  position < pAxis->nextpoint.axis[0].p)  ) return asynError;
            pAxis->endpoint.axis[0].p = position - pAxis->enc_offset;
            pAxis->endpoint.axis[0].v = 0.0;        
            setIntegerParam(axis, motorStatusDone, 0);
            pAxis->deferred_move = 0;
        }
    }
    return status;
}


asynStatus motorSimController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "writeInt32";
    
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(pAxis->axis, function, value);
    
    if (function == motorDeferMoves)
    {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: %sing Deferred Move flag on driver %s\n",
            value != 0.0?"Sett":"Clear",
            driverName, functionName, this->portName);
        if (value == 0.0 && this->movesDeferred != 0)
        {
            processDeferredMoves();
        }
        this->movesDeferred = value;
    } else {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorDriver::writeInt32(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axis, pAxis->axis);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}

asynStatus motorSimController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "writeFloat64";
    
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(pAxis->axis, function, value);
    
    if (function == motorPosition) 
    {
        pAxis->enc_offset = (double) value - pAxis->nextpoint.axis[0].p;
        asynPrint(pasynUser, ASYN_TRACE_FLOW, 
            "%s:%s: Set axis %d to position %d", 
            driverName, functionName, pAxis->axis, value);
    } else {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorDriver::writeFloat64(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axis, pAxis->axis);
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%f\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%f\n", 
              driverName, functionName, function, value);
    return status;
}

asynStatus motorSimController::moveAxis(asynUser*pasynUser, double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    route_pars_t pars;
    static const char *functionName = "moveAxis";

    if (relative) position += pAxis->endpoint.axis[0].p + pAxis->enc_offset;

    /* Check to see if in hard limits */
    if ((pAxis->nextpoint.axis[0].p >= pAxis->hiHardLimit  &&  position > pAxis->nextpoint.axis[0].p) ||
        (pAxis->nextpoint.axis[0].p <= pAxis->lowHardLimit &&  position < pAxis->nextpoint.axis[0].p)  ) return asynError;

    if (this->movesDeferred == 0) { /*Normal move.*/
        pAxis->endpoint.axis[0].p = position - pAxis->enc_offset;
        pAxis->endpoint.axis[0].v = 0.0;
    } else { /*Deferred moves.*/
        pAxis->deferred_position = position;
        pAxis->deferred_move = 1;
        pAxis->deferred_relative = relative;
    }
    routeGetParams(pAxis->route, &pars);
    if (max_velocity != 0) pars.axis[0].Vmax = fabs(max_velocity);
    if (acceleration != 0) pars.axis[0].Amax = fabs(acceleration);
    routeSetParams( pAxis->route, &pars ); 

    setIntegerParam(pAxis->axis, motorStatusDone, 0);
    callParamCallbacks(pAxis->axis, pAxis->axis);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f",
        driverName, functionName, this->portName, pAxis->axis, position, min_velocity, max_velocity, acceleration );
    return asynSuccess;
}

asynStatus motorSimAxis::velocity(double velocity, double acceleration )
{
    route_pars_t pars;
    double deltaV = velocity - this->nextpoint.axis[0].v;                
    double time;

    /* Check to see if in hard limits */
    if ((this->nextpoint.axis[0].p > this->hiHardLimit && velocity > 0) ||
        (this->nextpoint.axis[0].p < this->lowHardLimit && velocity < 0)  ) return asynError;

    routeGetParams( this->route, &pars );
    if (acceleration != 0) pars.axis[0].Amax = fabs(acceleration);
    routeSetParams( this->route, &pars );

    time = fabs( deltaV / pars.axis[0].Amax );

    this->endpoint.axis[0].v = velocity;
    this->endpoint.axis[0].p = ( this->nextpoint.axis[0].p +
                  time * ( this->nextpoint.axis[0].v + 0.5 * deltaV ));
    this->reroute = ROUTE_NEW_ROUTE;
    return asynSuccess;
}


asynStatus motorSimController::homeAxis(asynUser *pasynUser, double min_velocity, double max_velocity, double acceleration, int forwards )
{
    asynStatus status = asynError;
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveAxis";

    status = pAxis->velocity((forwards? max_velocity: -max_velocity), acceleration );
    pAxis->homing = 1;
    setIntegerParam(pAxis->axis, motorStatusDone, 0 );
    callParamCallbacks(pAxis->axis, pAxis->axis);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set dirver %s, axis %d to home %s, min vel=%f, max_vel=%f, accel=%f",
        driverName, functionName, this->portName, pAxis->axis, (forwards?"FORWARDS":"REVERSE"), min_velocity, max_velocity, acceleration );
    return status;
}


asynStatus motorSimController::moveVelocityAxis(asynUser *pasynUser, double min_velocity, double velocity, double acceleration )
{
    asynStatus status = asynError;
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveVelocityAxis";

    status = pAxis->velocity(velocity, acceleration );
    setIntegerParam(pAxis->axis, motorStatusDone, 0);
    callParamCallbacks(pAxis->axis, pAxis->axis);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set port %s, axis %d move with velocity of %f, accel=%f",
        driverName, functionName, this->portName, pAxis->axis, velocity, acceleration );
    return status;
}

asynStatus motorSimController::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger )
{
  return asynError;
}

asynStatus motorSimController::triggerProfile(asynUser *pasynUser)
{
  return asynError;
}

asynStatus motorSimController::stopAxis(asynUser *pasynUser, double acceleration )
{
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveVelocityAxis";

    pAxis->velocity(0.0, acceleration );
    pAxis->deferred_move = 0;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set axis %d to stop with accel=%f",
        driverName, functionName, pAxis->axis, acceleration );
    return asynSuccess;
}

asynStatus motorSimController::configAxis(int axis, int hiHardLimit, int lowHardLimit, int home, int start)
{
    motorSimAxis *pAxis = this->pAxes[axis];
    
    pAxis->hiHardLimit = hiHardLimit;
    pAxis->lowHardLimit = lowHardLimit;
    pAxis->home = home;
    pAxis->enc_offset = start;
    return(asynSuccess);
}

/**\defgroup motorSimTask Routines to implement the motor axis simulation task
@{
*/

/** Process one iteration of an axis

    This routine takes a single axis and propogates its motion forward a given amount
    of time.

    \param pAxis  [in]   Pointer to axis information.
    \param delta  [in]   Time in seconds to propogate motion forwards.

    \return Integer indicating 0 (asynSuccess) for success or non-zero for failure. 
*/

void motorSimController::process(motorSimAxis *pAxis, double delta )
{
    double lastpos;
    int done = 0;

    lastpos = pAxis->nextpoint.axis[0].p;
    pAxis->nextpoint.T += delta;
    routeFind( pAxis->route, pAxis->reroute, &(pAxis->endpoint), &(pAxis->nextpoint) );
    /*  if (pAxis->reroute == ROUTE_NEW_ROUTE) routePrint( pAxis->route, pAxis->reroute, &(pAxis->endpoint), &(pAxis->nextpoint), stdout ); */
    pAxis->reroute = ROUTE_CALC_ROUTE;

    /* No, do a limits check */
    if (pAxis->homing && 
        ((lastpos - pAxis->home) * (pAxis->nextpoint.axis[0].p - pAxis->home)) <= 0)
    {
        /* Homing and have crossed the home sensor - return to home */
        pAxis->homing = 0;
        pAxis->reroute = ROUTE_NEW_ROUTE;
        pAxis->endpoint.axis[0].p = pAxis->home;
        pAxis->endpoint.axis[0].v = 0.0;
    }
    if ( pAxis->nextpoint.axis[0].p > pAxis->hiHardLimit && pAxis->nextpoint.axis[0].v > 0 )
    {
        if (pAxis->homing) pAxis->velocity(-pAxis->endpoint.axis[0].v, 0.0 );
        else
        {
            pAxis->reroute = ROUTE_NEW_ROUTE;
            pAxis->endpoint.axis[0].p = pAxis->hiHardLimit;
            pAxis->endpoint.axis[0].v = 0.0;
        }
    }
    else if (pAxis->nextpoint.axis[0].p < pAxis->lowHardLimit && pAxis->nextpoint.axis[0].v < 0)
    {
        if (pAxis->homing) pAxis->velocity(-pAxis->endpoint.axis[0].v, 0.0 );
        else
        {
            pAxis->reroute = ROUTE_NEW_ROUTE;
            pAxis->endpoint.axis[0].p = pAxis->lowHardLimit;
            pAxis->endpoint.axis[0].v = 0.0;
        }
    }

    if (pAxis->nextpoint.axis[0].v ==  0) {
        if (!pAxis->deferred_move) {
            done = 1;
        }
    } else {
        done = 0;
    }

    setDoubleParam(  pAxis->axis, this->motorPosition,          (pAxis->nextpoint.axis[0].p+pAxis->enc_offset) );
    setDoubleParam(  pAxis->axis, this->motorEncoderPosition,   (pAxis->nextpoint.axis[0].p+pAxis->enc_offset) );
    setIntegerParam( pAxis->axis, this->motorStatusDirection,   (pAxis->nextpoint.axis[0].v >  0) );
    setIntegerParam( pAxis->axis, this->motorStatusDone,        done );
    setIntegerParam( pAxis->axis, this->motorStatusHighLimit,   (pAxis->nextpoint.axis[0].p >= pAxis->hiHardLimit) );
    setIntegerParam( pAxis->axis, this->motorStatusHome,        (pAxis->nextpoint.axis[0].p == pAxis->home) );
    setIntegerParam( pAxis->axis, this->motorStatusMoving,      !done );
    setIntegerParam( pAxis->axis, this->motorStatusLowLimit,    (pAxis->nextpoint.axis[0].p <= pAxis->lowHardLimit) );
}

static void motorSimTaskC(void *drvPvt)
{
    motorSimController *pController = (motorSimController*)drvPvt;
    pController->motorSimTask();
}
    

#define DELTA 0.1
void motorSimController::motorSimTask()
{
    epicsTimeStamp now;
    double delta;
    int axis;
    motorSimAxis *pAxis;

    while ( 1 )
    {
        /* Get a new timestamp */
        epicsTimeGetCurrent( &now );
        delta = epicsTimeDiffInSeconds( &now, &(this->now) );
        this->now = now;

        if ( delta > (DELTA/4.0) && delta <= (4.0*DELTA) )
        {
            /* A reasonable time has elapsed, it's not a time step in the clock */
            for (axis=0; axis<this->numAxes; axis++) 
            {       
                this->lock();
                pAxis = this->pAxes[axis];
                this->process(pAxis, delta );
                callParamCallbacks(axis, axis);
                this->unlock();
            }
        }
        epicsThreadSleep( DELTA );
    }
}

motorSimAxis::motorSimAxis(motorSimController *pController, int axis, double lowHardLimit, double hiHardLimit, double home, double start )
    : pController(pController), axis(axis), lowHardLimit(lowHardLimit), hiHardLimit(hiHardLimit), home(home)
{
      route_pars_t pars;

      pars.numRoutedAxes = 1;
      pars.routedAxisList[0] = 1;
      pars.Tsync = 0.0;
      pars.Tcoast = 0.0;
      pars.axis[0].Amax = 1.0;
      pars.axis[0].Vmax = 1.0;

      this->endpoint.T = 0;
      this->endpoint.axis[0].p = start;
      this->endpoint.axis[0].v = 0;
      this->nextpoint.T = 0;
      this->nextpoint.axis[0].p = start;
      this->route = routeNew( &(this->endpoint), &pars );
      this->deferred_move = 0;
}


motorSimController::motorSimController(const char *portName, int numAxes, int priority, int stackSize)
    :   asynMotorDriver(portName, numAxes, NUM_SIM_CONTROLLER_PARAMS, 
            asynInt32Mask | asynFloat64Mask, 
            asynInt32Mask | asynFloat64Mask,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
            1, // autoconnect
            priority, stackSize)
{
    int axis;
    motorSimAxis *pAxis;
    motorSimControllerNode *pNode;
    
    if (!motorSimControllerListInitialized) {
        motorSimControllerListInitialized = 1;
        ellInit(&motorSimControllerList);
    }
    
    // We should make sure this portName is not already in the list */
    pNode = (motorSimControllerNode*) calloc(1, sizeof(motorSimControllerNode));
    pNode->portName = epicsStrDup(portName);
    pNode->pController = this;
    ellAdd(&motorSimControllerList, (ELLNODE *)pNode);

    if (numAxes < 1 ) numAxes = 1;
    this->numAxes = numAxes;
    this->movesDeferred = 0;
    this->pAxes = (motorSimAxis**) calloc(numAxes, sizeof(motorSimAxis*));
    for (axis=0; axis<numAxes; axis++) {
        pAxis  = new motorSimAxis(this, axis, DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_START);
        this->pAxes[axis] = pAxis;
        setDoubleParam(axis, this->motorPosition, DEFAULT_START);
    }

    this->motorThread = epicsThreadCreate( "motorSimThread", 
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC) motorSimTaskC, (void *) this);
}

/** Configuration command, called directly or from iocsh */
extern "C" int motorSimCreateController(const char *portName, int numAxes, int priority, int stackSize)
{
    motorSimController *pSimController
        = new motorSimController(portName,numAxes, priority, stackSize);
    pSimController = NULL;
    return(asynSuccess);
}

extern "C" int motorSimConfigAxis(const char *portName, int axis, int hiHardLimit, int lowHardLimit, int home, int start)
{
    motorSimControllerNode *pNode;
    static const char *functionName = "motorSimConfigAxis";
    
    // Find this controller
    if (!motorSimControllerListInitialized) {
        printf("%s:%s: ERROR, controller list not initialized\n",
            driverName, functionName);
        return(-1);
    }
    pNode = (motorSimControllerNode*)ellFirst(&motorSimControllerList);
    while(pNode) {
        if (strcmp(pNode->portName, portName) == 0) {
            printf("%s:%s: configuring controller %s axis %d\n",
                driverName, functionName, pNode->portName, axis); 
            pNode->pController->configAxis(axis, hiHardLimit, lowHardLimit, home, start);
            return(0);
        }
        pNode = (motorSimControllerNode*)ellNext((ELLNODE*)pNode);
    }
    printf("Controller not found\n");
    return(-1);
}

/** Code for iocsh registration */
static const iocshArg motorSimCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg motorSimCreateControllerArg1 = {"Number of axes", iocshArgInt};
static const iocshArg motorSimCreateControllerArg2 = {"priority", iocshArgInt};
static const iocshArg motorSimCreateControllerArg3 = {"stackSize", iocshArgInt};
static const iocshArg * const motorSimCreateControllerArgs[] =  {&motorSimCreateControllerArg0,
                                                          &motorSimCreateControllerArg1,
                                                          &motorSimCreateControllerArg2,
                                                          &motorSimCreateControllerArg3};
static const iocshFuncDef motorSimCreateControllerDef = {"motorSimCreateController", 4, motorSimCreateControllerArgs};
static void motorSimCreateContollerCallFunc(const iocshArgBuf *args)
{
    motorSimCreateController(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}

static const iocshArg motorSimConfigAxisArg0 = { "Post name",     iocshArgString};
static const iocshArg motorSimConfigAxisArg1 = { "Axis #",        iocshArgInt};
static const iocshArg motorSimConfigAxisArg2 = { "High limit",    iocshArgInt};
static const iocshArg motorSimConfigAxisArg3 = { "Low limit",     iocshArgInt};
static const iocshArg motorSimConfigAxisArg4 = { "Home position", iocshArgInt};
static const iocshArg motorSimConfigAxisArg5 = { "Start posn",    iocshArgInt};

static const iocshArg *const motorSimConfigAxisArgs[] = {
    &motorSimConfigAxisArg0,
    &motorSimConfigAxisArg1,
    &motorSimConfigAxisArg2,
    &motorSimConfigAxisArg3,
    &motorSimConfigAxisArg4,
    &motorSimConfigAxisArg5
};
static const iocshFuncDef motorSimConfigAxisDef ={"motorSimConfigAxis",6,motorSimConfigAxisArgs};

static void motorSimConfigAxisCallFunc(const iocshArgBuf *args)
{
    motorSimConfigAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void motorSimDriverRegister(void)
{

    iocshRegister(&motorSimCreateControllerDef, motorSimCreateContollerCallFunc);
    iocshRegister(&motorSimConfigAxisDef, motorSimConfigAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(motorSimDriverRegister);
}
