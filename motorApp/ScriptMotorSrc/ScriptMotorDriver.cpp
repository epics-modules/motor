#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "ScriptMotorDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)


/************************************************
 * These are the ScriptMotorController methods *
 ************************************************/


/** Creates a new ScriptMotorController object.
  * \param[in] asyn_port       The name of the asyn port that will be created for this driver
  * \param[in] serial_port     The name of the drvAsynSerialPort that was created previously to connect to the VirtualMotor controller 
  * \param[in] max_axes        The number of axes that this controller supports 
  * \param[in] script_file 
  * \param[in] params
  */
ScriptMotorController::ScriptMotorController(const char* asyn_port,
                                             int max_axes, 
                                             const char* script_file, 
                                             const char* params)
  :  asynMotorController(asyn_port, 
                         max_axes, 
                         1, // No. ScriptMotorController asyn parameters
                         0, // No. additional interfaces beyond those in base class
                         0, // No. additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, // Default priority
                         0) // Default stack size
{
  this->script = std::string(script_file);

  if (params)    { this->init_params = std::string(params); }
  else           { this->init_params = std::string(""); }

  this->createParam("RELOAD_SCRIPT", asynParamInt32, &this->ScriptMotorReload);

  for (int axis = 0; axis < max_axes; axis += 1) 
  {
    new ScriptMotorAxis(this, axis, script_file, params);
  }
  
  this->startPoller(this->movingPollPeriod_, this->idlePollPeriod_, this->forcedFastPolls_);
}

void ScriptMotorController::reload()
{
  this->lock();
  for (int index = 0; index < this->numAxes_; index += 1)
  {
    ScriptMotorAxis* axis = this->getAxis(index);
    axis->reload(this->script.c_str(), this->init_params.c_str());
  }
  this->unlock();

  printf("Controller %s reloaded %s.\n", this->portName, this->script.c_str());
}

/** Creates a new ScriptMotorController object.
  * Configuration command, called directly or from iocsh
  * \param[in] asyn_port          The name of the asyn port that will be created for this driver
  * \param[in] max_axes           The number of axes that this controller supports 
  * \param[in] script_file 
  * \param[in] params
  */
extern "C" int ScriptControllerConfig(const char* asyn_port,
                                           int max_axes, 
                                           const char* script_file, 
                                           const char* params)
{
  new ScriptMotorController(asyn_port, max_axes, script_file, params);
  return(asynSuccess);
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void ScriptMotorController::report(FILE *fp, int level)
{
  fprintf(fp, "Script Motor Controller driver %s\n", this->portName);
  fprintf(fp, "    numAxes=%d\n", numAxes_);
  fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
  fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

asynStatus ScriptMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;

  if (function == this->ScriptMotorReload)
  { 
    if (value == 1)   { this->reload(); }
    return asynSuccess;
  }
  else
  {
    return asynMotorController::writeInt32(pasynUser, value);
  }
}

asynStatus ScriptMotorController::setIntegerParam(int list, int function, int value)
{
  if (function >= this->motorStatusDirection_ && function <= this->motorStatusHomed_)
  {
    ScriptMotorAxis* axis = (ScriptMotorAxis*) this->getAxis(list);
    epicsUInt32 status = axis->setStatusParam(function, value);
    asynMotorController::setIntegerParam(list, this->motorStatus_, status);
  }
  
  return asynMotorController::setIntegerParam(list, function, value);
}

asynStatus ScriptMotorController::setDoubleParam(int list, int function, double value)
{
  if (function == this->motorPosition_ || function == this->motorEncoderPosition_)
  {  
    ScriptMotorAxis* axis = (ScriptMotorAxis*) this->getAxis(list);
    axis->setPositionParam(function, value);
  }
  
  return asynMotorController::setDoubleParam(list, function, value);
}

void ScriptMotorController::configAxis(int axisNo, const char* params)
{
  ScriptMotorAxis* axis = this->getAxis(axisNo);

  if (params)    { axis->params = std::string(params); }
  else           { axis->params = std::string(""); }

  axis->config(params);
}

/** Returns a pointer to an ScriptMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ScriptMotorAxis* ScriptMotorController::getAxis(asynUser *pasynUser)
{
  return static_cast<ScriptMotorAxis*>(asynMotorController::getAxis(pasynUser));
}


/** Returns a pointer to an ScriptMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
ScriptMotorAxis* ScriptMotorController::getAxis(int axisNo)
{
  return static_cast<ScriptMotorAxis*>(asynMotorController::getAxis(axisNo));
}


/******************************************
 * These are the ScriptMotorAxis methods *
 ******************************************/


/** Creates a new ScriptMotorAxis object.
  * \param[in] pC Pointer to the ScriptMotorController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
ScriptMotorAxis::ScriptMotorAxis(ScriptMotorController *pC, int axisNo, const char* script_file, const char* params)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  this->initState(script_file);
  this->config(params);
  
  int isnum;
  
  lua_getglobal(this->state, "MovingPollPeriod");
  double MovingPollPeriod = lua_tonumberx(this->state, -1, &isnum);  
  if (isnum)    { pC->movingPollPeriod_ = MovingPollPeriod; }
  lua_remove(this->state, -1);
  
  lua_getglobal(this->state, "IdlePollPeriod");
  double IdlePollPeriod = lua_tonumberx(this->state, -1, &isnum);
  if (isnum)    { pC->idlePollPeriod_ = IdlePollPeriod; }
  lua_remove(this->state, -1);
  
  lua_getglobal(this->state, "ForcedFastPolls");
  double ForcedFastPolls = lua_tonumberx(this->state, -1, &isnum);
  if (isnum)    { pC->forcedFastPolls_ = ForcedFastPolls; }
  lua_remove(this->state, -1);
  
  // Zero the encoder position (this only appears to be a problem on windows)
  setDoubleParam(pC_->motorEncoderPosition_, 0.0);

  // Make the changed parameters take effect
  callParamCallbacks();
}

void ScriptMotorAxis::initState(const char* script_file)
{
  this->state = luaL_newstate();
  int status = luaLoadScript(this->state, script_file);
  
  if (status) { printf("Error compiling script file: %s\n", script_file); }

  lua_pushstring(this->state, (const char*) this->pC_->portName);
  lua_setglobal(this->state, "DRIVER");
  
  lua_pushnumber(this->state, axisNo_);
  lua_setglobal(this->state, "AXIS");
}

void ScriptMotorAxis::reload(const char* script_file, const char* controller_params)
{
    this->initState(script_file);
    this->config(controller_params);
    this->config(this->params.c_str());
}

epicsUInt32 ScriptMotorAxis::setStatusParam(int index, int value)
{
  if (index >= pC_->motorStatusDirection_ && index <= pC_->motorStatusHomed_)
  {
    epicsUInt32 status = status_.status;
    int mask = 1 << (index - pC_->motorStatusDirection_);
    
    if (value) { status |= mask; }
    else       { status &= ~mask; }
    
    if (status != status_.status)
    {
      status_.status = status;
      statusChanged_ = 1;
    }
    
    return status;
  }
  
  return 0;
}

void ScriptMotorAxis::setPositionParam(int index, double value)
{
  if (index == pC_->motorPosition_)
  {
    if (value != status_.position)
    {
        statusChanged_ = 1;
        status_.position = value;
    }
  } 
  else if (index == pC_->motorEncoderPosition_)
  {
    if (value != status_.encoderPosition)
    {
        statusChanged_ = 1;
        status_.encoderPosition = value;
    }
  }
}


/*
 *
 */
extern "C" int ScriptAxisConfig(const char* ScriptMotorName, int axisNo, const char* params)
{
  static const char *functionName = "VirtualMotorCreateAxis";
 
  ScriptMotorController *pC = (ScriptMotorController*) findAsynPortDriver(ScriptMotorName);
  if (!pC) 
  {
    printf("Error port %s not found\n", ScriptMotorName);
    return asynError;
  }

  pC->lock();
    pC->configAxis(axisNo, params);
  pC->unlock();
  
  return asynSuccess;
}


/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void ScriptMotorAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "    Axis #%d\n", axisNo_);
    fprintf(fp, "        axisIndex_=%d\n", axisIndex_);
 }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


/*
 * move() is called by asynMotor device support when an absolute or a relative move is requested.
 * It can be called multiple times if BDST > 0 or RTRY > 0.
 *
 * Arguments in terms of motor record fields:
 *     position (steps) = RVAL = DVAL / MRES
 *     baseVelocity (steps/s) = VBAS / abs(MRES)
 *     velocity (step/s) = VELO / abs(MRES)
 *     acceleration (step/s/s) = (velocity - baseVelocity) / ACCL
 */
asynStatus ScriptMotorAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "ScriptMotorAxis::move";
  
  int result = lua_getglobal(this->state, "move");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, position);
    lua_pushboolean(this->state, relative);
    lua_pushnumber(this->state, minVelocity);
    lua_pushnumber(this->state, maxVelocity);
    lua_pushnumber(this->state, acceleration);
    
    if (lua_pcall(this->state, 5, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}


/*
 * home() is called by asynMotor device support when a home is requested.
 * Note: forwards is set by device support, NOT by the motor record.
 *
 * Arguments in terms of motor record fields:
 *     minVelocity (steps/s) = VBAS / abs(MRES)
 *     maxVelocity (step/s) = HVEL / abs(MRES)
 *     acceleration (step/s/s) = (maxVelocity - minVelocity) / ACCL
 *     forwards = 1 if HOMF was pressed, 0 if HOMR was pressed
 */

asynStatus ScriptMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  int result = lua_getglobal(this->state, "home");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, minVelocity);
    lua_pushnumber(this->state, maxVelocity);
    lua_pushnumber(this->state, acceleration);
    lua_pushboolean(this->state, forwards);
    
    if (lua_pcall(this->state, 4, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }

  return asynSuccess;
}



/*
 * moveVelocity() is called by asynMotor device support when a jog is requested.
 * If a controller doesn't have a jog command (or jog commands), this a jog can be simulated here.
 *
 * Arguments in terms of motor record fields:
 *     minVelocity (steps/s) = VBAS / abs(MRES)
 *     maxVelocity (step/s) = (jog_direction == forward) ? JVEL * DIR / MRES : -1 * JVEL * DIR / MRES
 *     acceleration (step/s/s) = JAR / abs(EGU)
 */
asynStatus ScriptMotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  int result = lua_getglobal(this->state, "moveVelocity");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, minVelocity);
    lua_pushnumber(this->state, maxVelocity);
    lua_pushnumber(this->state, acceleration);
    
    if (lua_pcall(this->state, 3, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}


/*
 * stop() is called by asynMotor device support whenever a user presses the stop button.
 * It is also called when the jog button is released.
 *
 * Arguments in terms of motor record fields:
 *     acceleration = ??? 
 */
asynStatus ScriptMotorAxis::stop(double acceleration)
{
  int result = lua_getglobal(this->state, "stop");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, acceleration);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}


/*
 * setPosition() is called by asynMotor device support when a position is redefined.
 * It is also required for autosave to restore a position to the controller at iocInit.
 *
 * Arguments in terms of motor record fields:
 *     position (steps) = DVAL / MRES = RVAL 
 */
asynStatus ScriptMotorAxis::setPosition(double position)
{
  int result = lua_getglobal(this->state, "setPosition");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, position);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}

asynStatus ScriptMotorAxis::setEncoderPosition(double position)
{
  int result = lua_getglobal(this->state, "setEncoderPosition");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, position);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}

asynStatus ScriptMotorAxis::setHighLimit(double highLimit)
{
  int result = lua_getglobal(this->state, "setHighLimit");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, highLimit);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}

asynStatus ScriptMotorAxis::setLowLimit(double lowLimit)
{
  int result = lua_getglobal(this->state, "setLowLimit");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, lowLimit);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}


asynStatus ScriptMotorAxis::setPGain(double PGain)
{
  int result = lua_getglobal(this->state, "setPGain");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, PGain);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}

asynStatus ScriptMotorAxis::setIGain(double IGain)
{
  int result = lua_getglobal(this->state, "setIGain");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, IGain);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}

asynStatus ScriptMotorAxis::setDGain(double DGain)
{
  int result = lua_getglobal(this->state, "setDGain");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, DGain);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}

/*
 * setClosedLoop() is called by asynMotor device support when a user enables or disables torque, 
 * usually from the motorx_all.adl, but only for drivers that set the following params to 1:
 *   pC->motorStatusGainSupport_
 *   pC->motorStatusHasEncoder_
 * What is actually implemented here varies greatly based on the specfics of the controller.
 * 
 * Arguments in terms of motor record fields:
 *     closedLoop = CNEN 
 */

asynStatus ScriptMotorAxis::setClosedLoop(bool closedLoop)
{
  int result = lua_getglobal(this->state, "setClosedLoop");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushboolean(this->state, (int) closedLoop);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}

asynStatus ScriptMotorAxis::setEncoderRatio(double EncoderRatio)
{
  int result = lua_getglobal(this->state, "setEncoderRatio");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {
    lua_pushnumber(this->state, EncoderRatio);
    
    if (lua_pcall(this->state, 1, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    // Do something with returned value
    
    lua_pop(this->state, 1);
  }
  
  return asynSuccess;
}



/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus ScriptMotorAxis::poll(bool *moving)
{ 
  int result = lua_getglobal(this->state, "poll");
  if (result != LUA_TFUNCTION)
  {
    // No function in script
  }
  else
  {    
    if (lua_pcall(this->state, 0, 1, 0))
    {
      this->logError();
      return asynError;
    }
    
    int rettype = lua_type(this->state, -1);
    
    if (rettype == LUA_TBOOLEAN)
    {
      if (lua_toboolean(this->state, -1))    { *moving = true; }
      else                                   { *moving = false; }
    }
    
    lua_pop(this->state, 1);
  }
    
  this->callParamCallbacks();
  return asynSuccess;
}

void ScriptMotorAxis::config(const char* params)
{
  luaLoadMacros(this->state, params);
}

void ScriptMotorAxis::logError()
{
  std::string err(lua_tostring(this->state, -1));
  lua_pop(this->state, 1);
  
  printf("%s\n", err.c_str());
}

void ScriptMotorReload(const char* port)
{
    ScriptMotorController* controller = (ScriptMotorController*) findAsynPortDriver(port);
    
    if (controller != NULL)    { controller->reload(); }
}


/** Code for iocsh registration */
static const iocshArg ScriptMotorReloadArg0 = {"Motor Port name", iocshArgString};

static const iocshArg* const ScriptMotorReloadArgs[] = {&ScriptMotorReloadArg0};

static const iocshFuncDef ScriptMotorReloadDef = {"ScriptMotorReload", 1, ScriptMotorReloadArgs};

static void ScriptMotorReloadCallFunc(const iocshArgBuf *args)
{
  ScriptMotorReload(args[0].sval);
}


static const iocshArg ScriptMotorCreateControllerArg0 = {"Motor Port name", iocshArgString};
static const iocshArg ScriptMotorCreateControllerArg1 = {"Number of axes", iocshArgInt};
static const iocshArg ScriptMotorCreateControllerArg2 = {"Control Script", iocshArgString};
static const iocshArg ScriptMotorCreateControllerArg3 = {"Parameters", iocshArgString};
static const iocshArg * const ScriptMotorCreateControllerArgs[] = {&ScriptMotorCreateControllerArg0,
                                                             &ScriptMotorCreateControllerArg1,
                                                             &ScriptMotorCreateControllerArg2,
                                                             &ScriptMotorCreateControllerArg3};
static const iocshFuncDef ScriptMotorCreateControllerDef = {"ScriptControllerConfig", 4, ScriptMotorCreateControllerArgs};
static void ScriptMotorCreateContollerCallFunc(const iocshArgBuf *args)
{
  ScriptControllerConfig(args[0].sval, args[1].ival, args[2].sval, args[3].sval);
}


static const iocshArg ScriptMotorCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg ScriptMotorCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg ScriptMotorCreateAxisArg2 = {"Parameters", iocshArgString};

static const iocshArg * const ScriptMotorCreateAxisArgs[] = {&ScriptMotorCreateAxisArg0,
                                                     &ScriptMotorCreateAxisArg1,
                                                     &ScriptMotorCreateAxisArg2};
                                                    
static const iocshFuncDef ScriptMotorCreateAxisDef = {"ScriptAxisConfig", 3, ScriptMotorCreateAxisArgs};
static void ScriptMotorCreateAxisCallFunc(const iocshArgBuf *args)
{
  ScriptAxisConfig(args[0].sval, args[1].ival, args[2].sval);
}



static void ScriptMotorRegister(void)
{
  iocshRegister(&ScriptMotorReloadDef,           ScriptMotorReloadCallFunc);
  iocshRegister(&ScriptMotorCreateControllerDef, ScriptMotorCreateContollerCallFunc);
  iocshRegister(&ScriptMotorCreateAxisDef,       ScriptMotorCreateAxisCallFunc);
}


extern "C" {
epicsExportRegistrar(ScriptMotorRegister);
}
