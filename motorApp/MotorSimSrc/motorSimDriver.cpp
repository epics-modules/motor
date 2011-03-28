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
#include "motorSimDriver.h"
#include <epicsExport.h>

#define DEFAULT_LOW_LIMIT -10000
#define DEFAULT_HI_LIMIT   10000
#define DEFAULT_HOME       0
#define DEFAULT_START      0

static const char *driverName = "motorSimDriver";

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
        this->portName, numAxes_);

    for (axis=0; axis<numAxes_; axis++) {
        pAxis = pAxes_[axis];
        fprintf(fp, "  axis %d\n", 
            pAxis->axis_);

        if (level > 0)
        {
            double lowSoftLimit=0.0;
            double hiSoftLimit=0.0;

            fprintf(fp, "    Current position = %f, velocity = %f at current time: %f\n", 
                   pAxis->nextpoint_.axis[0].p, 
                   pAxis->nextpoint_.axis[0].v,
                   pAxis->nextpoint_.T);
            fprintf(fp, "    Destination posn = %f, velocity = %f at desination time:  %f\n",
                   pAxis->endpoint_.axis[0].p, 
                   pAxis->endpoint_.axis[0].v,
                   pAxis->endpoint_.T);

            fprintf(fp, "    Hard limits: %f, %f\n", pAxis->lowHardLimit_, pAxis->hiHardLimit_);
            fprintf(fp, "           Home: %f\n", pAxis->home_);
            fprintf(fp, "    Enc. offset: %f\n", pAxis->enc_offset_);
            getDoubleParam(pAxis->axis_, motorHighLimit_, &hiSoftLimit);
            getDoubleParam(pAxis->axis_, motorLowLimit_, &lowSoftLimit);
            fprintf(fp, "    Soft limits: %f, %f\n", lowSoftLimit, hiSoftLimit );

            if (pAxis->homing_) fprintf(fp, "    Currently homing axis\n" );
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

motorSimAxis * motorSimController::getAxis(asynUser *pasynUser)
{
    int axis;
    motorSimAxis *pAxis;
    
    getAddress(pasynUser, &axis);
    pAxis = pAxes_[axis];
    return(pAxis);
}
    

asynStatus motorSimController::processDeferredMoves()
{
    asynStatus status = asynError;
    double position = 0.0;
    int axis;
    motorSimAxis *pAxis;

    for (axis=0; axis<numAxes_; axis++)
    {
        pAxis = pAxes_[axis];
        if (pAxis->deferred_move_) {
            position = pAxis->deferred_position_;
            /* Check to see if in hard limits */
            if ((pAxis->nextpoint_.axis[0].p >= pAxis->hiHardLimit_  &&  position > pAxis->nextpoint_.axis[0].p) ||
                (pAxis->nextpoint_.axis[0].p <= pAxis->lowHardLimit_ &&  position < pAxis->nextpoint_.axis[0].p)  ) return asynError;
            pAxis->endpoint_.axis[0].p = position - pAxis->enc_offset_;
            pAxis->endpoint_.axis[0].v = 0.0;        
            setIntegerParam(axis, motorStatusDone_, 0);
            pAxis->deferred_move_ = 0;
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
    status = setIntegerParam(pAxis->axis_, function, value);
    
    if (function == motorDeferMoves_)
    {
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: %sing Deferred Move flag on driver %s\n",
            value != 0.0?"Sett":"Clear",
            driverName, functionName, this->portName);
        if (value == 0.0 && movesDeferred_ != 0)
        {
            processDeferredMoves();
        }
        movesDeferred_ = value;
    } else {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeInt32(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axis_);
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
    status = setDoubleParam(pAxis->axis_, function, value);
    
    if (function == motorPosition_) 
    {
        pAxis->enc_offset_ = (double) value - pAxis->nextpoint_.axis[0].p;
        asynPrint(pasynUser, ASYN_TRACE_FLOW, 
            "%s:%s: Set axis %d to position %d", 
            driverName, functionName, pAxis->axis_, value);
    } else {
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeFloat64(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axis_);
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

asynStatus motorSimController::moveAxis(asynUser*pasynUser, double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    route_pars_t pars;
    static const char *functionName = "moveAxis";

    if (relative) position += pAxis->endpoint_.axis[0].p + pAxis->enc_offset_;

    /* Check to see if in hard limits */
    if ((pAxis->nextpoint_.axis[0].p >= pAxis->hiHardLimit_  &&  position > pAxis->nextpoint_.axis[0].p) ||
        (pAxis->nextpoint_.axis[0].p <= pAxis->lowHardLimit_ &&  position < pAxis->nextpoint_.axis[0].p)  ) return asynError;

    if (movesDeferred_ == 0) { /*Normal move.*/
        pAxis->endpoint_.axis[0].p = position - pAxis->enc_offset_;
        pAxis->endpoint_.axis[0].v = 0.0;
    } else { /*Deferred moves.*/
        pAxis->deferred_position_ = position;
        pAxis->deferred_move_ = 1;
        pAxis->deferred_relative_ = relative;
    }
    routeGetParams(pAxis->route_, &pars);
    if (maxVelocity != 0) pars.axis[0].Vmax = fabs(maxVelocity);
    if (acceleration != 0) pars.axis[0].Amax = fabs(acceleration);
    routeSetParams( pAxis->route_, &pars ); 

    setIntegerParam(pAxis->axis_, motorStatusDone_, 0);
    callParamCallbacks(pAxis->axis_);

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, maxVel=%f, accel=%f",
        driverName, functionName, this->portName, pAxis->axis_, position, minVelocity, maxVelocity, acceleration );
    return asynSuccess;
}

asynStatus motorSimAxis::velocity(double velocity, double acceleration )
{
    route_pars_t pars;
    double deltaV = velocity - this->nextpoint_.axis[0].v;                
    double time;

    /* Check to see if in hard limits */
    if ((this->nextpoint_.axis[0].p > hiHardLimit_ && velocity > 0) ||
        (this->nextpoint_.axis[0].p < lowHardLimit_ && velocity < 0)  ) return asynError;

    routeGetParams( this->route_, &pars );
    if (acceleration != 0) pars.axis[0].Amax = fabs(acceleration);
    routeSetParams( this->route_, &pars );

    time = fabs( deltaV / pars.axis[0].Amax );

    this->endpoint_.axis[0].v = velocity;
    this->endpoint_.axis[0].p = ( this->nextpoint_.axis[0].p +
                  time * ( this->nextpoint_.axis[0].v + 0.5 * deltaV ));
    this->reroute_ = ROUTE_NEW_ROUTE;
    return asynSuccess;
}


asynStatus motorSimController::homeAxis(asynUser *pasynUser, double minVelocity, double maxVelocity, double acceleration, int forwards )
{
    asynStatus status = asynError;
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveAxis";

    status = pAxis->velocity((forwards? maxVelocity: -maxVelocity), acceleration );
    pAxis->homing_ = 1;
    setIntegerParam(pAxis->axis_, motorStatusDone_, 0 );
    callParamCallbacks(pAxis->axis_);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set driver %s, axis %d to home %s, min vel=%f, maxVel=%f, accel=%f",
        driverName, functionName, this->portName, pAxis->axis_, (forwards?"FORWARDS":"REVERSE"), minVelocity, maxVelocity, acceleration );
    return status;
}


asynStatus motorSimController::moveVelocityAxis(asynUser *pasynUser, double minVelocity, double velocity, double acceleration )
{
    asynStatus status = asynError;
    motorSimAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveVelocityAxis";

    status = pAxis->velocity(velocity, acceleration );
    setIntegerParam(pAxis->axis_, motorStatusDone_, 0);
    callParamCallbacks(pAxis->axis_);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set port %s, axis %d move with velocity of %f, accel=%f",
        driverName, functionName, this->portName, pAxis->axis_, velocity, acceleration );
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
    pAxis->deferred_move_ = 0;

    asynPrint(pasynUser, ASYN_TRACE_FLOW, 
        "%s:%s: Set axis %d to stop with accel=%f",
        driverName, functionName, pAxis->axis_, acceleration );
    return asynSuccess;
}

asynStatus motorSimController::configAxis(int axis, int hiHardLimit, int lowHardLimit, int home, int start)
{
    motorSimAxis *pAxis = pAxes_[axis];
    
    pAxis->hiHardLimit_ = hiHardLimit;
    pAxis->lowHardLimit_ = lowHardLimit;
    pAxis->home_ = home;
    pAxis->enc_offset_ = start;
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

    lastpos = pAxis->nextpoint_.axis[0].p;
    pAxis->nextpoint_.T += delta;
    routeFind( pAxis->route_, pAxis->reroute_, &(pAxis->endpoint_), &(pAxis->nextpoint_) );
    /*  if (pAxis->reroute == ROUTE_NEW_ROUTE) routePrint( pAxis->route, pAxis->reroute, &(pAxis->endpoint), &(pAxis->nextpoint_), stdout ); */
    pAxis->reroute_ = ROUTE_CALC_ROUTE;

    /* No, do a limits check */
    if (pAxis->homing_ && 
        ((lastpos - pAxis->home_) * (pAxis->nextpoint_.axis[0].p - pAxis->home_)) <= 0)
    {
        /* Homing and have crossed the home sensor - return to home */
        pAxis->homing_ = 0;
        pAxis->reroute_ = ROUTE_NEW_ROUTE;
        pAxis->endpoint_.axis[0].p = pAxis->home_;
        pAxis->endpoint_.axis[0].v = 0.0;
    }
    if ( pAxis->nextpoint_.axis[0].p > pAxis->hiHardLimit_ && pAxis->nextpoint_.axis[0].v > 0 )
    {
        if (pAxis->homing_) pAxis->velocity(-pAxis->endpoint_.axis[0].v, 0.0 );
        else
        {
            pAxis->reroute_ = ROUTE_NEW_ROUTE;
            pAxis->endpoint_.axis[0].p = pAxis->hiHardLimit_;
            pAxis->endpoint_.axis[0].v = 0.0;
        }
    }
    else if (pAxis->nextpoint_.axis[0].p < pAxis->lowHardLimit_ && pAxis->nextpoint_.axis[0].v < 0)
    {
        if (pAxis->homing_) pAxis->velocity(-pAxis->endpoint_.axis[0].v, 0.0 );
        else
        {
            pAxis->reroute_ = ROUTE_NEW_ROUTE;
            pAxis->endpoint_.axis[0].p = pAxis->lowHardLimit_;
            pAxis->endpoint_.axis[0].v = 0.0;
        }
    }

    if (pAxis->nextpoint_.axis[0].v ==  0) {
        if (!pAxis->deferred_move_) {
            done = 1;
        }
    } else {
        done = 0;
    }

    setDoubleParam(  pAxis->axis_, this->motorPosition_,          (pAxis->nextpoint_.axis[0].p+pAxis->enc_offset_) );
    setDoubleParam(  pAxis->axis_, this->motorEncoderPosition_,   (pAxis->nextpoint_.axis[0].p+pAxis->enc_offset_) );
    setIntegerParam( pAxis->axis_, this->motorStatusDirection_,   (pAxis->nextpoint_.axis[0].v >  0) );
    setIntegerParam( pAxis->axis_, this->motorStatusDone_,        done );
    setIntegerParam( pAxis->axis_, this->motorStatusHighLimit_,   (pAxis->nextpoint_.axis[0].p >= pAxis->hiHardLimit_) );
    setIntegerParam( pAxis->axis_, this->motorStatusHome_,        (pAxis->nextpoint_.axis[0].p == pAxis->home_) );
    setIntegerParam( pAxis->axis_, this->motorStatusMoving_,      !done );
    setIntegerParam( pAxis->axis_, this->motorStatusLowLimit_,    (pAxis->nextpoint_.axis[0].p <= pAxis->lowHardLimit_) );
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
        delta = epicsTimeDiffInSeconds( &now, &(now_) );
        now_ = now;

        if ( delta > (DELTA/4.0) && delta <= (4.0*DELTA) )
        {
            /* A reasonable time has elapsed, it's not a time step in the clock */
            for (axis=0; axis<numAxes_; axis++) 
            {       
                this->lock();
                pAxis = pAxes_[axis];
                this->process(pAxis, delta );
                callParamCallbacks(axis);
                this->unlock();
            }
        }
        epicsThreadSleep( DELTA );
    }
}

motorSimAxis::motorSimAxis(motorSimController *pController, int axis, double lowHardLimit, double hiHardLimit, double home, double start )
    : pController_(pController), axis_(axis), lowHardLimit_(lowHardLimit), hiHardLimit_(hiHardLimit), home_(home)
{
      route_pars_t pars;

      pars.numRoutedAxes = 1;
      pars.routedAxisList[0] = 1;
      pars.Tsync = 0.0;
      pars.Tcoast = 0.0;
      pars.axis[0].Amax = 1.0;
      pars.axis[0].Vmax = 1.0;

      this->endpoint_.T = 0;
      this->endpoint_.axis[0].p = start;
      this->endpoint_.axis[0].v = 0;
      this->nextpoint_.T = 0;
      this->nextpoint_.axis[0].p = start;
      this->route_ = routeNew( &(this->endpoint_), &pars );
      this->deferred_move_ = 0;
}


motorSimController::motorSimController(const char *portName, int numAxes, int priority, int stackSize)
    :   asynMotorController(portName, numAxes, NUM_SIM_CONTROLLER_PARAMS, 
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
    numAxes_ = numAxes;
    this->movesDeferred_ = 0;
    pAxes_ = (motorSimAxis**) calloc(numAxes, sizeof(motorSimAxis*));
    for (axis=0; axis<numAxes; axis++) {
        pAxis  = new motorSimAxis(this, axis, DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_START);
        pAxes_[axis] = pAxis;
        setDoubleParam(axis, this->motorPosition_, DEFAULT_START);
    }

    this->motorThread_ = epicsThreadCreate( "motorSimThread", 
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
