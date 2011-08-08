/*
FILENAME...  motorSimAxis.cpp
USAGE...     Simulated Motor Axis Support.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$

Original Author: Mark Rivers
Based on drvMotorSim.c
December 13, 2009

*/

#include <math.h>
#include <string.h>

#include <iocsh.h>
#include <epicsExport.h>

#include "MotorSimController.h"
#include "MotorSimAxis.h"

extern bool motorSimControllerListInitialized;
extern ELLLIST motorSimControllerList;

motorSimAxis::motorSimAxis(motorSimController *pController, int axis, double lowHardLimit, double hiHardLimit, double home, double start)
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
}


void motorSimAxis::axisReport(FILE *& fp, int & level)
{
    fprintf(fp, "  axis %d\n", getAxisIndex());
    if(level > 0){
        double lowSoftLimit = 0.0;
        double hiSoftLimit = 0.0;
        fprintf(fp, "  Current position = %f, velocity = %f at current time: %f\n", nextpoint_.axis[0].p, nextpoint_.axis[0].v, nextpoint_.T);
        fprintf(fp, "  Destination posn = %f, velocity = %f at desination time:  %f\n", endpoint_.axis[0].p, endpoint_.axis[0].v, endpoint_.T);
        fprintf(fp, "    Hard limits: %f, %f\n", lowHardLimit_, hiHardLimit_);
        fprintf(fp, "           Home: %f\n", home_);
        fprintf(fp, "    Enc. offset: %f\n", enc_offset_);
        pC_->getDoubleParam(axisNo_, getMotorHighLimitIndex(), &hiSoftLimit);
        pC_->getDoubleParam(axisNo_, getMotorLowLimitIndex(), &lowSoftLimit);
        fprintf(fp, "    Soft limits: %f, %f\n", lowSoftLimit, hiSoftLimit);
        if(homing_)
            fprintf(fp, "    Currently homing axis\n");

    }
}

asynStatus motorSimAxis::doDeferredMove()
{
	asynStatus status = asynSuccess;
    if(deferred_move_){
        double position = 0.0;
        position = deferred_position_;
        /* Check to see if in hard limits */
        if((nextpoint_.axis[0].p >= hiHardLimit_ && position > nextpoint_.axis[0].p) || (nextpoint_.axis[0].p <= lowHardLimit_ && position < nextpoint_.axis[0].p)){
            status = asynError;
        }else{
            endpoint_.axis[0].p = position - enc_offset_;
            endpoint_.axis[0].v = 0.0;
            getStatus()->setDoneMoving(false);
            deferred_move_ = 0;
        }
    }
    return status;
}

asynStatus motorSimAxis::move(double position, bool relative, double minVelocity, double maxVelocity, double acceleration)
{
  route_pars_t pars;
  static const char *functionName = "move";

  if (relative) position += endpoint_.axis[0].p + enc_offset_;

  /* Check to see if in hard limits */
  if ((nextpoint_.axis[0].p >= hiHardLimit_  &&  position > nextpoint_.axis[0].p) ||
    (nextpoint_.axis[0].p <= lowHardLimit_ &&  position < nextpoint_.axis[0].p)  ) return asynError;

  if (!((static_cast<motorSimController*>(pC_))->areMovesDeferred())) { /*Normal move.*/
    endpoint_.axis[0].p = position - enc_offset_;
    endpoint_.axis[0].v = 0.0;
  } else { /*Deferred moves.*/
    deferred_position_ = position;
    deferred_move_ = 1;
    deferred_relative_ = relative;
  }
  routeGetParams(route_, &pars);
  if (maxVelocity != 0)
    pars.axis[0].Vmax = fabs(maxVelocity);
  if (acceleration != 0)
    pars.axis[0].Amax = fabs(acceleration);
  routeSetParams( route_, &pars ); 

  getStatus()->setDoneMoving(false);
  callParamCallbacks();

  asynPrint(pasynUser_, ASYN_TRACE_FLOW, 
            "motorSimAxis:%s: Set driver %s, axis %d move to %f, min vel=%f, maxVel=%f, accel=%f\n",
            functionName, pC_->portName, axisNo_, position, minVelocity, maxVelocity, acceleration );
  return asynSuccess;
}

asynStatus motorSimAxis::setVelocity(double velocity, double acceleration )
{
  route_pars_t pars;
  double deltaV = velocity - this->nextpoint_.axis[0].v;        
  double time;

  /* Check to see if in hard limits */
  if ((this->nextpoint_.axis[0].p > hiHardLimit_ && velocity > 0) ||
      (this->nextpoint_.axis[0].p < lowHardLimit_ && velocity < 0))
    return (asynError);

  routeGetParams( this->route_, &pars );
  if (acceleration != 0)
    pars.axis[0].Amax = fabs(acceleration);
  routeSetParams( this->route_, &pars );

  time = fabs(deltaV / pars.axis[0].Amax);

  this->endpoint_.axis[0].v = velocity;
  this->endpoint_.axis[0].p = (this->nextpoint_.axis[0].p +
                               time * ( this->nextpoint_.axis[0].v + 0.5 * deltaV));
  this->reroute_ = ROUTE_NEW_ROUTE;
  return (asynSuccess);
}


asynStatus motorSimAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards )
{
  asynStatus status = asynError;
  // static const char *functionName = "home";

  status = setVelocity((forwards? maxVelocity: -maxVelocity), acceleration );
  homing_ = 1;
  return status;
}


asynStatus motorSimAxis::moveVelocity(double minVelocity, double velocity, double acceleration )
{
  asynStatus status = asynError;
  // static const char *functionName = "moveVelocity";

  status = setVelocity(velocity, acceleration );
  return status;
}

asynStatus motorSimAxis::stop(double acceleration )
{
  // static const char *functionName = "moveVelocityAxis";

  setVelocity(0.0, acceleration );
  deferred_move_ = 0;
  return asynSuccess;
}

asynStatus motorSimAxis::setPosition(double position)
{
  enc_offset_ = position - nextpoint_.axis[0].p;
  return asynSuccess;
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
  epicsTimeStamp now;
  double delta;

  epicsTimeGetCurrent(&now);
  delta = epicsTimeDiffInSeconds(&now, &prevTime_);
  prevTime_ = now;

  pC_->lock();
  process(delta, moving);
  pC_->unlock();
  return(asynSuccess);
}


/** Process one iteration of an axis

  This routine takes a single axis and propogates its motion forward a given amount
  of time.

  \param pAxis  [in]   Pointer to axis information.
  \param delta  [in]   Time in seconds to propogate motion forwards.

  \return Integer indicating 0 (asynSuccess) for success or non-zero for failure. 
*/

void motorSimAxis::process(double delta, bool *moving)
{
  double lastpos;
  bool done = false;

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
  if ( nextpoint_.axis[0].p > hiHardLimit_ && nextpoint_.axis[0].v > 0 )
  {
    if (homing_)
      setVelocity(-endpoint_.axis[0].v, 0.0 );
    else
    {
      reroute_ = ROUTE_NEW_ROUTE;
      endpoint_.axis[0].p = hiHardLimit_;
      endpoint_.axis[0].v = 0.0;
    }
  }
  else if (nextpoint_.axis[0].p < lowHardLimit_ && nextpoint_.axis[0].v < 0)
  {
    if (homing_)
      setVelocity(-endpoint_.axis[0].v, 0.0 );
    else
    {
      reroute_ = ROUTE_NEW_ROUTE;
      endpoint_.axis[0].p = lowHardLimit_;
      endpoint_.axis[0].v = 0.0;
    }
  }

  if (nextpoint_.axis[0].v ==  0)
  {
    if (!deferred_move_)
    {
      done = true;
    }
  }
  else
    done = false;

  setDoubleParam (getMotorPositionIndex(),         (nextpoint_.axis[0].p+enc_offset_));
  setDoubleParam (getMotorEncoderPositionIndex(),  (nextpoint_.axis[0].p+enc_offset_));
  if (nextpoint_.axis[0].v != 0)
    getStatus()->setDirection((nextpoint_.axis[0].v >  0) ? PLUS:MINUS);
  getStatus()->setDoneMoving(done);
  *moving = !done;
  getStatus()->setHighLimitOn(nextpoint_.axis[0].p >= hiHardLimit_);
  getStatus()->setAtHome(nextpoint_.axis[0].p == home_);
  getStatus()->setMoving(!done);
  getStatus()->setLowLimitOn(nextpoint_.axis[0].p <= lowHardLimit_);
  callParamCallbacks();
}

asynStatus motorSimAxis::createParams()
{
  int status = asynSuccess;
  asynStatus retStatus = asynSuccess;

  status |= asynMotorAxis::createParams();
  status |= getStatus()->createParams();

  if (status != 0 )
      retStatus = asynError;
  return retStatus;
}

asynStatus motorSimAxis::postInitAxis()
{
  return pC_->setDoubleParam(getAxisIndex(), getMotorPositionIndex(), DEFAULT_START);
}

int motorSimAxis::getNumParams()
{
  return asynMotorAxis::getNumParams() + asynMotorStatus::getNumParams();
}

extern "C" int motorSimConfigAxis(const char *portName, int axis, int hiHardLimit, int lowHardLimit, int home, int start)
{
  motorSimControllerNode *pNode;
  static const char *fileName = "motorSimAxis";
  static const char *functionName = "motorSimConfigAxis";

  // Find this controller
  if (motorSimControllerListInitialized == false)
  {
    printf("%s:%s: ERROR, controller list not initialized\n", fileName, functionName);
    return(-1);
  }
  pNode = (motorSimControllerNode*)ellFirst(&motorSimControllerList);
  while (pNode)
  {
    if (strcmp(pNode->portName, portName) == 0)
    {
      printf("%s:%s: configuring controller %s axis %d\n", fileName, functionName, pNode->portName, axis); 
      static_cast<motorSimAxis*>(pNode->pController->getAxis(axis))->config(hiHardLimit, lowHardLimit, home, start);
      return(0);
    }
    pNode = (motorSimControllerNode*)ellNext((ELLNODE*)pNode);
  }
  printf("Controller not found\n");
  return(-1);
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

void motorSimConfigAxisCallFunc(const iocshArgBuf *args)
{
  motorSimConfigAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

const iocshFuncDef motorSimConfigAxisDef ={"motorSimConfigAxis",6,motorSimConfigAxisArgs};

static void motorSimAxisRegister(void)
{

  iocshRegister(&motorSimConfigAxisDef, motorSimConfigAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(motorSimAxisRegister);
}

