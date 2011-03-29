/*
FILENAME...  motorSimController.h
USAGE...     Simulated Motor Support.

Based on drvMotorSim.c

Mark Rivers
March 28, 2010

*/


#include <epicsTime.h>
#include <epicsThread.h>

#include "asynMotorDriver.h"
#include "route.h"

#define NUM_SIM_CONTROLLER_PARAMS 0

class motorSimAxis : public asynMotorAxis
{
public:

  /* These are the functions we override from the base class */
  motorSimAxis(class motorSimController *pController, int axis, double lowLimit, double hiLimit, double home, double start);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(int *moving);
  motorSimController* getController();

  /* These are the methods that are new to this class */
  asynStatus config(int hiHardLimit, int lowHardLimit, int home, int start);
  asynStatus setVelocity(double velocity, double acceleration);
  void process(double delta );

private:
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

  /* These are the fucntions we override from the base class */
  motorSimController(const char *portName, int numAxes, int priority, int stackSize);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  motorSimAxis* getAxis(asynUser *pasynUser);
  motorSimAxis* getAxis(int axisNo);
  asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
  asynStatus triggerProfile(asynUser *pasynUser);

  /* These are the functions that are new to this class */
  void motorSimTask();  // Should be pivate, but called from non-member function

private:
  asynStatus processDeferredMoves();
  epicsThreadId motorThread_;
  epicsTimeStamp prevTime_;
  int movesDeferred_;
  
friend class motorSimAxis;
};
