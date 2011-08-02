/*
FILENAME...     MotorSimController.cpp
USAGE...        Simulated Motor Support.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$

Original Author: Mark Rivers
Based on drvMotorSim.c
December 13, 2009

*/

#include <stdlib.h>
#include <math.h>

#include <ellLib.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExport.h>

#include "motorSimController.h"
#include "motorSimAxis.h"

static const char *driverName = "motorSimDriver";

static void motorSimTaskC(void *drvPvt);

bool motorSimControllerListInitialized = false;
ELLLIST motorSimControllerList;


motorSimController::motorSimController(const char *portName, int numAxes, int priority, int stackSize)
:  asynMotorController(portName, numAxes,
                       asynInt32Mask | asynFloat64Mask, 
                       asynInt32Mask | asynFloat64Mask,
                       ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                       1, // autoconnect
                       priority, stackSize),
initialized(false)
{
  motorSimControllerNode *pNode;

  if (motorSimControllerListInitialized == false)
  {
    motorSimControllerListInitialized = true;
    ellInit(&motorSimControllerList);
  }

  // We should make sure this portName is not already in the list */
  pNode = (motorSimControllerNode*) calloc(1, sizeof(motorSimControllerNode));
  pNode->portName = epicsStrDup(portName);
  pNode->pController = this;
  ellAdd(&motorSimControllerList, (ELLNODE *)pNode);

  if (numAxes < 1 )
    numAxes = 1;
  numAxes_ = numAxes;
  this->movesDeferred_ = 0;

  this->motorThread_ = epicsThreadCreate("motorSimThread", epicsThreadPriorityLow,
                                         epicsThreadGetStackSize(epicsThreadStackMedium),
                                         (EPICSTHREADFUNC) motorSimTaskC, (void *) this);
}

asynStatus motorSimController::postInitDriver()
{
  int axis;
  asynMotorAxis *pAxis;
  for (axis=0; axis<numAxes_; axis++) {
    pAxis  = new motorSimAxis(this, axis, DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_START);
    pAxis->initializeAxis();
  }
  initialized = true; // Make poller wait on this indicator.
  return asynSuccess;
}


void motorSimController::report(FILE *fp, int level)
{
  int axis;
//  motorSimAxis *pAxis;
  asynMotorAxis *pAxis;

  fprintf(fp, "Simulation motor driver %s, numAxes=%d\n", 
          this->portName, numAxes_);

  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    pAxis->axisReport(fp, level);
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

asynStatus motorSimController::processDeferredMoves()
{
  asynStatus status = asynError;
  int axis;
  motorSimAxis *pAxis;

  for (axis=0; axis<numAxes_; axis++)
  {
    pAxis = static_cast<motorSimAxis*>(getAxis(axis));
    status = pAxis->doDeferredMove();
    if (status != asynSuccess) break;
  }
  return status;
}

asynStatus motorSimController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  asynMotorAxis *pAxis = this->getAxis(pasynUser);
  static const char *functionName = "writeInt32";


  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  pAxis->setIntegerParam(function, value);

  if (function == motorDeferMoves_)
  {
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: %sing Deferred Move flag on driver %s\n",
              value != 0.0 ? "Sett":"Clear", driverName, functionName, this->portName);
    if (value == 0.0 && movesDeferred_ != 0)
      processDeferredMoves();
    movesDeferred_ = value;
  }
  else /* Call base class call its method (if we have our parameters check this here) */
    status = asynMotorController::writeInt32(pasynUser, value);

  /* Do callbacks so higher layers see any changes */
  pAxis->callParamCallbacks();
  if (status)
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
  return (status);
}

asynStatus motorSimController::profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger )
{
  return asynError;
}

asynStatus motorSimController::triggerProfile(asynUser *pasynUser)
{
  return asynError;
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

  while(initialized == false)
    epicsThreadSleep(1.0);

  while ( 1 )
  {
    /* Get a new timestamp */
    epicsTimeGetCurrent( &now );
    delta = epicsTimeDiffInSeconds( &now, &(prevTime_) );
    prevTime_ = now;

    if ( delta > (DELTA/4.0) && delta <= (4.0*DELTA) )
    {
      /* A reasonable time has elapsed, it's not a time step in the clock */
      for (axis=0; axis<numAxes_; axis++) 
      {     
        this->lock();
        pAxis = static_cast<motorSimAxis*>(getAxis(axis));
        pAxis->process(delta );
        this->unlock();
      }
    }
    epicsThreadSleep( DELTA );
  }
}

int motorSimController::getNumParams()
{
  return NUM_SIM_CONTROLLER_PARAMS + asynMotorController::getNumParams() + motorSimAxis::getNumParams();
}

/**
 *
 */
bool motorSimController::areMovesDeferred()
{
  return (movesDeferred_ == 1);
}


/** Configuration command, called directly or from iocsh */
extern "C" int motorSimCreateController(const char *portName, int numAxes, int priority, int stackSize)
{
  motorSimController *pSimController = new motorSimController(portName,numAxes, priority, stackSize);
  pSimController->initializePortDriver();
  return(asynSuccess);
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

static void motorSimControlRegister(void)
{

  iocshRegister(&motorSimCreateControllerDef, motorSimCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(motorSimControlRegister);
}
