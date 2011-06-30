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

#include "asynMotorController.h"
#include "epicsUnitTest.h"
#include "testMain.h"

void tFindParam(const char* paramName, asynMotorController mc, 

  asynStatus expStatus)
{
//  int index = 0;
//  asynStatus status = asynSuccess;
//  status = mc.findParam( paramName, &index);
//  testOk(status == expStatus, "verify Parameter %s exists, value %d",
//    paramName, index);
//testOk1(true);
}


MAIN(asynMotorControllerTest)
{
  const char DUMMY_PARAM[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM2[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM3[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM4[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM5[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM6[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM7[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM8[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM9[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  const char DUMMY_PARAM10[] = "HOW_COULD_THIS_BE_A_PARAMETER";
  asynMotorController mc("testController", 3, 0, 0,0 , 0, 0, 0,0);
  int numTests = 18+1;
  testPlan(3);
   
printf("hello\n");
  tFindParam(DUMMY_PARAM, mc, asynSuccess);
printf("hello\n");
//  tFindParam(DUMMY_PARAM2, mc, asynSuccess);
printf("hello\n");
  tFindParam(DUMMY_PARAM3, mc, asynSuccess);
printf("hello\n");
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
