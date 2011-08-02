/*
FILENAME...  motorSimController.h
USAGE...     Simulated Motor Support.

Based on drvMotorSim.c

Mark Rivers
March 28, 2010

*/


#include <epicsTime.h>
#include <epicsThread.h>

#include "asynMotorController.h"

#define DEFAULT_LOW_LIMIT -10000
#define DEFAULT_HI_LIMIT   10000
#define DEFAULT_HOME       0
#define DEFAULT_START      0

#define NUM_SIM_CONTROLLER_PARAMS 0

class motorSimController : public asynMotorController {
public:

  /* These are the fucntions we override from the base class */
  motorSimController(const char *portName, int numAxes, int priority, int stackSize);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  void report(FILE *fp, int level);
  asynStatus profileMove(asynUser *pasynUser, int npoints, double positions[], double times[], int relative, int trigger);
  asynStatus triggerProfile(asynUser *pasynUser);
  bool areMovesDeferred();
  virtual int getNumParams();

  /* These are the functions that are new to this class */
  void motorSimTask();  // Should be pivate, but called from non-member function

protected:
  virtual asynStatus postInitDriver();

private:
  asynStatus processDeferredMoves();
  epicsThreadId motorThread_;
  epicsTimeStamp prevTime_;
  int movesDeferred_;
  bool initialized;
};


typedef struct motorSimControllerNode {
  ELLNODE node;
  const char *portName;
  motorSimController *pController;
} motorSimControllerNode;


