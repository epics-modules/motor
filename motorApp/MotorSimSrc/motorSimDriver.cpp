/*
FILENAME...  motorSimController.cpp
USAGE...     Simulated Motor Support.

Based on drvMotorSim.c

Mark Rivers
December 13, 2009

*/


#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <iocsh.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "motorSimDriver.h"

#define DEFAULT_LOW_LIMIT -10000
#define DEFAULT_HI_LIMIT   10000
#define DEFAULT_HOME       0
#define DEFAULT_START      0

static const char *driverName = "motorSimDriver";

static void motorSimTaskC(void *drvPvt);

typedef struct motorSimControllerNode {
  ELLNODE node;
  const char *portName;
  motorSimController *pController;
} motorSimControllerNode;

static ELLLIST motorSimControllerList;
static int motorSimControllerListInitialized = 0;

motorSimAxis::motorSimAxis(motorSimController *pController, int axis, double lowHardLimit, double hiHardLimit, double home, double start )
  : asynMotorAxis(pController, axis),
    pC_(pController),
    lowHardLimit_(lowHardLimit), hiHardLimit_(hiHardLimit), home_(home)
{
  route_pars_t pars;

  pars.numRoutedAxes = 1;
  pars.routedAxisList[0] = 1;
  pars.Tsync = 0.0;
  pars.Tcoast = 0.0;
  pars.axis[0].Amax = 1.0;
  pars.axis[0].Vmax = 1.0;

  endpoint_.T = 0;
  endpoint_.axis[0].p = start;
  endpoint_.axis[0].v = 0;
  nextpoint_.T = 0;
  nextpoint_.axis[0].p = start;
  route_ = routeNew( &(this->endpoint_), &pars );
  deferred_move_ = 0;
  delayedDone_ = 0;
  lastDone_ = 1;
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
}


motorSimController::motorSimController(const char *portName, int numAxes, int priority, int stackSize)
  :  asynMotorController(portName, numAxes, NUM_SIM_CONTROLLER_PARAMS, 
                         asynInt32Mask | asynFloat64Mask, 
                         asynInt32Mask | asynFloat64Mask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         priority, stackSize)
{
  int axis;
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
  for (axis=0; axis<numAxes; axis++) {
    new motorSimAxis(this, axis, DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, DEFAULT_HOME, DEFAULT_START);
    setDoubleParam(axis, this->motorPosition_, DEFAULT_START);
	setDoubleParam(axis, this->motorPostMoveDelay_, 0.0);
  }

  this->motorThread_ = epicsThreadCreate("motorSimThread", 
                                         epicsThreadPriorityLow,
                                         epicsThreadGetStackSize(epicsThreadStackMedium),
                                         (EPICSTHREADFUNC) motorSimTaskC, (void *) this);
}

void motorSimController::report(FILE *fp, int level)
{
  int axis;
  motorSimAxis *pAxis;

  fprintf(fp, "Simulation motor driver %s, numAxes=%d\n", 
          this->portName, numAxes_);

  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    fprintf(fp, "  axis %d\n", pAxis->axisNo_);

    if (level > 0)
    {
      double lowSoftLimit=0.0;
      double hiSoftLimit=0.0;

      fprintf(fp, "  Current position = %f, velocity = %f at current time: %f\n", 
           pAxis->nextpoint_.axis[0].p, 
           pAxis->nextpoint_.axis[0].v,
           pAxis->nextpoint_.T);
      fprintf(fp, "  Destination posn = %f, velocity = %f at desination time:  %f\n",
           pAxis->endpoint_.axis[0].p, 
           pAxis->endpoint_.axis[0].v,
           pAxis->endpoint_.T);

      fprintf(fp, "    Hard limits: %f, %f\n", pAxis->lowHardLimit_, pAxis->hiHardLimit_);
      fprintf(fp, "           Home: %f\n", pAxis->home_);
      fprintf(fp, "    Enc. offset: %f\n", pAxis->enc_offset_);
      getDoubleParam(pAxis->axisNo_, motorHighLimit_, &hiSoftLimit);
      getDoubleParam(pAxis->axisNo_, motorLowLimit_, &lowSoftLimit);
      fprintf(fp, "    Soft limits: %f, %f\n", lowSoftLimit, hiSoftLimit );

      if (pAxis->homing_) fprintf(fp, "    Currently homing axis\n" );
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

asynStatus motorSimController::processDeferredMoves()
{
  asynStatus status = asynError;
  double position = 0.0;
  int axis;
  motorSimAxis *pAxis;

  for (axis=0; axis<numAxes_; axis++)
  {
    pAxis = getAxis(axis);
    if (pAxis->deferred_move_) {
      position = pAxis->deferred_position_;
      /* Check to see if in hard limits */
      if ((pAxis->nextpoint_.axis[0].p >= pAxis->hiHardLimit_  &&  position > pAxis->nextpoint_.axis[0].p) ||
          (pAxis->nextpoint_.axis[0].p <= pAxis->lowHardLimit_ &&  position < pAxis->nextpoint_.axis[0].p)) return asynError;
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
  pAxis->setIntegerParam(function, value);
  
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
  pAxis->callParamCallbacks();
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

motorSimAxis* motorSimController::getAxis(asynUser *pasynUser)
{
  return static_cast<motorSimAxis*>(asynMotorController::getAxis(pasynUser));
}

motorSimAxis* motorSimController::getAxis(int axisNo)
{
  return static_cast<motorSimAxis*>(asynMotorController::getAxis(axisNo));
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
        pAxis = getAxis(axis);
        pAxis->process(delta );
        this->unlock();
      }
    }
    epicsThreadSleep( DELTA );
  }
}

asynStatus motorSimAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  route_pars_t pars;
  double currentPos;
  static const char *functionName = "move";
  pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &currentPos);
  std::cerr << "motorSimAxis::move axis " << axisNo_ << " from " << currentPos << "|" << endpoint_.axis[0].p + enc_offset_ << " to " << position << (relative ? " (relative)" : " (absolute)") << " speed min/max " << minVelocity << "/" << maxVelocity << " acceleration " << acceleration << std::endl;

  if (relative) position += endpoint_.axis[0].p + enc_offset_;

  /* Check to see if in hard limits */
  if ((nextpoint_.axis[0].p >= hiHardLimit_  &&  position > nextpoint_.axis[0].p) ||
    (nextpoint_.axis[0].p <= lowHardLimit_ &&  position < nextpoint_.axis[0].p)  )
  {
      std::cerr << "motorSimAxis::move failed (hard limits)" << std::endl;
      return asynError;
  }

  if (pC_->movesDeferred_ == 0) { /*Normal move.*/
    endpoint_.axis[0].p = position - enc_offset_;
    endpoint_.axis[0].v = 0.0;
  } else { /*Deferred moves.*/
    deferred_position_ = position;
    deferred_move_ = 1;
    deferred_relative_ = relative;
  }
  routeGetParams(route_, &pars);
  if (maxVelocity != 0) pars.axis[0].Vmax = fabs(maxVelocity);
  if (acceleration != 0) pars.axis[0].Amax = fabs(acceleration);
  routeSetParams( route_, &pars ); 

  setIntegerParam(pC_->motorStatusDone_, 0);
  callParamCallbacks();

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, maxVel=%f, accel=%f\n",
            driverName, functionName, pC_->portName, axisNo_, position, minVelocity, maxVelocity, acceleration );
  return asynSuccess;
}

asynStatus motorSimAxis::setVelocity(double velocity, double acceleration )
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
  this->endpoint_.axis[0].p = (this->nextpoint_.axis[0].p +
                              time * ( this->nextpoint_.axis[0].v + 0.5 * deltaV));
  this->reroute_ = ROUTE_NEW_ROUTE;
  std::cerr << "motorSimAxis::setVelocity axis " << axisNo_ << " velocity " << velocity << " acceleration " << acceleration << std::endl;
  return asynSuccess;
}


asynStatus motorSimAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards )
{
  asynStatus status = asynError;
  // static const char *functionName = "home";

  std::cerr << "motorSimAxis::home axis " << axisNo_ << " direction " << (forwards ? "forwards" : "backwards") << std::endl;
  status = setVelocity((forwards? maxVelocity: -maxVelocity), acceleration );
  homing_ = 1;
  return status;
}


asynStatus motorSimAxis::moveVelocity(double minVelocity, double velocity, double acceleration )
{
  asynStatus status = asynError;
  std::cerr << "motorSimAxis::moveVelocity axis " << axisNo_ << " velocity " << velocity << " acceleration " << acceleration << std::endl;   
  // static const char *functionName = "moveVelocity";

  status = setVelocity(velocity, acceleration );
  return status;
}

asynStatus motorSimAxis::stop(double acceleration )
{
  // static const char *functionName = "moveVelocityAxis";

  std::cerr << "motorSimAxis::stop axis " << axisNo_ << " acceleration " << acceleration << std::endl;   
  setVelocity(0.0, acceleration );
  deferred_move_ = 0;
  return asynSuccess;
}

asynStatus motorSimAxis::setPosition(double position)
{
  std::cerr << "motorSimAxis::setPosition axis " << axisNo_ << " position " << position << std::endl;
  enc_offset_ = position - nextpoint_.axis[0].p;
  return asynSuccess;
}

asynStatus motorSimAxis::setEncoderPosition(double position)
{
  std::cerr << "motorSimAxis::setEncoderPosition axis " << axisNo_ << " position " << position << std::endl;
  // currently we do not track encoder separately but just keep in syn with motor
  return asynMotorAxis::setEncoderPosition(position);
}

asynStatus motorSimAxis::config(int hiHardLimit, int lowHardLimit, int home, int start)
{
  hiHardLimit_ = hiHardLimit;
  lowHardLimit_ = lowHardLimit;
  home_ = home;
  enc_offset_ = start;
  return asynSuccess;
}

asynStatus motorSimAxis::poll(bool *moving)
{
  return asynSuccess;
}


/** Set the high limit position of the motor.
  * \param[in] highLimit The new high limit position that should be set in the hardware. Units=steps.*/
asynStatus motorSimAxis::setHighLimit(double highLimit)
{
  hiHardLimit_ = highLimit;
  return asynSuccess;
}


/** Set the low limit position of the motor.
  * \param[in] lowLimit The new low limit position that should be set in the hardware. Units=steps.*/
asynStatus motorSimAxis::setLowLimit(double lowLimit)
{
  lowHardLimit_ = lowLimit;
  return asynSuccess;
}


/** Process one iteration of an axis

  This routine takes a single axis and propogates its motion forward a given amount
  of time.

  \param delta  [in]   Time in seconds to propogate motion forwards.

  \return Integer indicating 0 (asynSuccess) for success or non-zero for failure. 
*/

void motorSimAxis::process(double delta )
{
  double lastpos;
  int done = 0;
  double postMoveDelay = 0.0;
  epicsTimeStamp nowTime;
  double nowTimeSecs = 0.0;

  lastpos = nextpoint_.axis[0].p;
  nextpoint_.T += delta;
  routeFind( route_, reroute_, &endpoint_, &nextpoint_ );
  /*  if (reroute_ == ROUTE_NEW_ROUTE) routePrint( route_, reroute_, &endpoint_, &nextpoint_, stdout ); */
  reroute_ = ROUTE_CALC_ROUTE;

  /* No, do a limits check */
  if (homing_ && 
    ((lastpos - home_) * (nextpoint_.axis[0].p - home_)) <= 0)
  {
    /* Homing and have crossed the home sensor - return to home */
    homing_ = 0;
    reroute_ = ROUTE_NEW_ROUTE;
    endpoint_.axis[0].p = home_;
    endpoint_.axis[0].v = 0.0;
  }
  
  route_pars_t pars;
  routeGetParams(route_, &pars);
  
  if ( nextpoint_.axis[0].p > hiHardLimit_ && nextpoint_.axis[0].v > 0 )
  {
    if (homing_) setVelocity(-endpoint_.axis[0].v, 0.0 );
    else
    {
	  stop(pars.axis[0].Amax);
    }
  }
  else if (nextpoint_.axis[0].p < lowHardLimit_ && nextpoint_.axis[0].v < 0)
  {
    if (homing_) setVelocity(-endpoint_.axis[0].v, 0.0 );
    else
    {
	  stop(pars.axis[0].Amax);
    }
  }

  if (nextpoint_.axis[0].v ==  0) {
    if (!deferred_move_) {
      if (!delayedDone_) {
	done = 1;
      }
    }
  } else {
    done = 0;
  }

  //Post move delay
  epicsTimeGetCurrent(&nowTime);
  pC_->getDoubleParam(axisNo_, pC_->motorPostMoveDelay_, &postMoveDelay);
  if ((lastDone_ == 0) && (done == 1)) {
    if (postMoveDelay > 0) {
      delayedDone_ = 1;
      done = 0;
      lastTimeSecs_ = nowTime.secPastEpoch + (nowTime.nsec / 1.e9);
    }
  }
  if (delayedDone_ == 1) {
    nowTimeSecs = nowTime.secPastEpoch + (nowTime.nsec / 1.e9);
    if ((nowTimeSecs - lastTimeSecs_) >= postMoveDelay) {
      done = 1;
      delayedDone_ = 0;
    }
  }

  double encRatio;
  pC_->getDoubleParam(axisNo_, pC_->motorEncoderRatio_, &encRatio);
  if (!lastDone_ && done) {
      std::cerr << "motorSimAxis::process axis " << axisNo_ << " has stopped moving: motor=" << nextpoint_.axis[0].p+enc_offset_ << " encoder=" << (nextpoint_.axis[0].p+enc_offset_) * encRatio << std::endl;
  }
  if (lastDone_ && !done) {
      std::cerr << "motorSimAxis::process axis " << axisNo_ << " has started moving: motor=" << nextpoint_.axis[0].p+enc_offset_ << " encoder=" << (nextpoint_.axis[0].p+enc_offset_) * encRatio << std::endl;
  }
  lastDone_ = done;

  setDoubleParam (pC_->motorPosition_,         (nextpoint_.axis[0].p+enc_offset_));
  setDoubleParam (pC_->motorEncoderPosition_,  (nextpoint_.axis[0].p+enc_offset_) * encRatio);
  setIntegerParam(pC_->motorStatusDirection_,  (nextpoint_.axis[0].v >  0));
  setIntegerParam(pC_->motorStatusDone_,       done);
  setIntegerParam(pC_->motorStatusHighLimit_,  (nextpoint_.axis[0].p >= hiHardLimit_));
  setIntegerParam(pC_->motorStatusHome_,       (nextpoint_.axis[0].p == home_));
  setIntegerParam(pC_->motorStatusMoving_,     !done);
  setIntegerParam(pC_->motorStatusLowLimit_,   (nextpoint_.axis[0].p <= lowHardLimit_));
  callParamCallbacks();
}

/** Configuration command, called directly or from iocsh */
extern "C" int motorSimCreateController(const char *portName, int numAxes, int priority, int stackSize)
{
  new motorSimController(portName,numAxes, priority, stackSize);
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
             pNode->pController->getAxis(axis)->config(hiHardLimit, lowHardLimit, home, start);
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
