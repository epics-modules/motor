/*
FILENAME...  motorSimAxis.h
USAGE...     motorSimAxis object definition.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$

Original Author: Mark Rivers, March 28, 2010

Based on drvMotorSim.c

*/

#include <epicsTime.h>

#include "asynMotorAxis.h"
#include "route.h"

class motorSimAxis : public asynMotorAxis
{
public:

  /* These are the pure virtual functions that must be implemented */
  motorSimAxis(class motorSimController *pController, int axis, double lowLimit, double hiLimit, double home, double start);
  asynStatus move(double position, bool relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus doDeferredMove();
  void axisReport(FILE *& fp, int & level);
  virtual asynStatus createParams();
  static int        getNumParams();

  /* These are the methods that are new to this class */
  asynStatus config(int hiHardLimit, int lowHardLimit, int home, int start);
  asynStatus setVelocity(double velocity, double acceleration);
  void process(double delta );

protected:
  virtual asynStatus postInitAxis();

private:
  asynMotorController *pC_;
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
  
};

