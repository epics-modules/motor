/*
FILENAME...   HXPDriver.h
USAGE...      Motor driver support for the Newport Hexapod controller.

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_HXP_AXES 6
#define HXP_POLL_TIMEOUT 2.0
#define HXP_MOVE_TIMEOUT 100000.0 // "Forever"

// No controller-specific parameters yet
#define NUM_HXP_PARAMS 0  

class HXPAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  HXPAxis(class HXPController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setClosedLoop(bool closedLoop);

private:
  HXPController *pC_;          /* Pointer to the asynMotorController to which this axis belongs.
                                      Abbreviated because it is used very frequently */
  int moveSocket_;
  int pollSocket_;
  char axisName_;
  char positionerName_[12];
  double encoderPosition_;
  double setpointPosition_;
  int axisStatus_;
  double mres_;
  int moving_;
  
friend class HXPController;
};

class HXPController : public asynMotorController {
public:
  HXPController(const char *portName, const char *HXPPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  void report(FILE *fp, int level);
  HXPAxis* getAxis(asynUser *pasynUser);
  HXPAxis* getAxis(int axisNo);

private:
  char *IPAddress_;
  int IPPort_;
  int pollSocket_;
  // only needed for profile moves
  //int moveSocket_;
  char firmwareVersion_[100];
  char *axisNames_;
  int HXPStatus_;
  int noDisableError_;

friend class HXPAxis;
};
