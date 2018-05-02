#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <string>
#include "luaEpics.h"

class epicsShareClass ScriptMotorAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  ScriptMotorAxis(class ScriptMotorController *pC, int axisNo, const char* script_file, const char* params);
  
  void reload(const char* script, const char* params);

  void report(FILE *fp, int level);
  
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);
  asynStatus setPGain(double pGain);
  asynStatus setIGain(double iGain);
  asynStatus setDGain(double dGain);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setEncoderRatio(double ratio);

  virtual epicsUInt32 setStatusParam(int index, int value);
  virtual void setPositionParam(int index, double value);
  
private:
  ScriptMotorController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  int axisIndex_;
  
  std::string params;

  lua_State* state;
  
  void initState(const char* script_file);
  void config(const char* params);
  void logError();
  
friend class ScriptMotorController;
};

class epicsShareClass ScriptMotorController : public asynMotorController {
public:
  ScriptMotorController(const char *asyn_port, int max_axes, const char* script_file, const char* params);

  virtual asynStatus setIntegerParam(int list, int function, int value);
  virtual asynStatus setDoubleParam(int list, int function, double value);
  
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  void report(FILE *fp, int level);
  ScriptMotorAxis* getAxis(asynUser *pasynUser);
  ScriptMotorAxis* getAxis(int axisNo);
  
  void configAxis(int axisNo, const char* params);
  
  void reload();

protected:
  int ScriptMotorReload;

private:
  std::string script;
  std::string init_params;

friend class ScriptMotorAxis;
};
