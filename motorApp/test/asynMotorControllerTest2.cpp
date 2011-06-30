/*************************************************************************\
* Copyright (c) 2009 UChicago Argonne LLC, as Operator of Argonne
*     National Laboratory.
* Copyright (c) 2002 The Regents of the University of California, as
*     Operator of Los Alamos National Laboratory.
* EPICS BASE is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
\*************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "epicsUnitTest.h"
#include "testMain.h"
#include "asynMotorController.h"
void tFindParam(const char* paramName, asynMotorController mc, 
  asynStatus expStatus)
{
  int index = 0;
  asynStatus status = asynSuccess;
  status = mc.findParam( paramName, &index);
  testOk(status == expStatus, "verify Parameter %s exists, value %d",
    paramName, index);
}


MAIN(asynMotorControllerTest2)
{
  const char DUMMY_PARAM[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  asynMotorController mc("testController", 3, 0, 0,0 , 0, 0, 0,0);
  int numTests = 18+1;
  testPlan(11);
  int index = 0;
  asynStatus status = asynSuccess;

  status = mc.findParam( motorMoveRelString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorMoveRelString, index);
   
  status = mc.findParam( motorMoveAbsString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorMoveAbsString, index);
   
  status = mc.findParam( motorMoveVelString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorMoveVelString, index);
   
  status = mc.findParam( motorHomeString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorHomeString, index);
   
  status = mc.findParam( motorStopString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorStopString, index);
   
  status = mc.findParam( motorVelocityString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorVelocityString, index);
   
  status = mc.findParam( motorVelBaseString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorVelBaseString, index);
   
  status = mc.findParam( motorAccelString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorAccelString, index);
   
  status = mc.findParam( motorPositionString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorPositionString, index);
   
  status = mc.findParam( motorEncoderPositionString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorEncoderPositionString, index);
   
  status = mc.findParam( motorDeferMovesString, &index);
  testOk(status == asynSuccess, "verify Parameter %s exists, value %d",
    motorDeferMovesString, index);
   
//  tFindParam(motorMoveRelString, mc, asynSuccess);
//  tFindParam(motorMoveAbsString, mc, asynSuccess);
//  tFindParam(motorMoveVelString, mc, asynSuccess);
//  tFindParam(motorHomeString, mc, asynSuccess);
//  tFindParam(motorStopString, mc, asynSuccess);
//  tFindParam(motorVelocityString, mc, asynSuccess);
//  tFindParam(motorVelBaseString, mc, asynSuccess);
//  tFindParam(motorAccelString, mc, asynSuccess);
//  tFindParam(motorPositionString, mc, asynSuccess);
//  tFindParam(motorEncoderPositionString, mc, asynSuccess);
//  tFindParam(motorDeferMovesString, mc, asynSuccess);
//  tFindParam(motorMoveAbsString, mc, asynSuccess);
//  tFindParam(motorMoveRelString, mc, asynSuccess);
//  tFindParam(motorMoveAbsString, mc, asynSuccess);
////  tFindParam(motorMoveRelString, mc, asynSuccess);
//  tFindParam(motorMoveAbsString, mc, asynSuccess);
//  tFindParam(motorMoveRelString, mc, asynSuccess);
//  tFindParam(motorMoveAbsString, mc, asynSuccess);
//  //tFindParam(DUMMY_PARAM, mc, asynParamNotFound);
  return testDone();

}
