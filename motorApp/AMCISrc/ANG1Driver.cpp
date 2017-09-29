/*
FILENAME... ANG1Driver.cpp
USAGE...    Motor record driver support for the AMCI ANG1 stepper motor controller over Modbus/TCP.

Kurt Goetze

Based on the ACS MCB-4B Model 3 device driver written by Mark Rivers

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsString.h>

#include <asynInt32SyncIO.h>

#include "ANG1Driver.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

static const char *driverName = "ANG1MotorDriver";

/*** Input Registers ***/
#define STATUS_1   0
#define STATUS_2   1
#define POS_RD_UPR 2
#define POS_RD_LWR 3
#define EN_POS_UPR 4
#define EN_POS_LWR 5
#define EN_CAP_UPR 6
#define EN_CAP_LWR 7
#define MOT_CUR    8  // programmed motor current (x10) 
#define JERK_RD    9

/*** Output Registers ***/
#define CMD_MSW    0  // module 0 starts at register address 1024.  This is set in drvModbusAsynConfigure.
#define CMD_LSW    1
#define POS_WR_UPR 2
#define POS_WR_LWR 3
#define SPD_UPR    4
#define SPD_LWR    5
#define ACCEL      6
#define DECEL      7
// Not used must equal zero #define RESERVED 8
#define JERK       9

/** Constructor, Creates a new ANG1Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ANG1InPortName    The name of the drvAsynSerialPort that was created previously to connect to the ANG1 controller 
  * \param[in] ANG1OutPortName   The name of the drvAsynSerialPort that was created previously to connect to the ANG1 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
ANG1Controller::ANG1Controller(const char *portName, const char *ANG1InPortName, const char *ANG1OutPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_ANG1_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis, i;
  asynStatus status;
  ANG1Axis *pAxis;
  static const char *functionName = "ANG1Controller::ANG1Controller";
  
  inputDriver_ = epicsStrDup(ANG1InPortName);    // Set this before calls to create Axis objects
  
  // Create controller-specific parameters
  createParam(ANG1JerkString,         asynParamInt32,       &ANG1Jerk_);
  
  /* Connect to ANG1 controller */
  for (i=0; i<MAX_INPUT_REGS; i++) {
    status = pasynInt32SyncIO->connect(ANG1InPortName, i, &pasynUserInReg_[i], NULL);
  }
  for (i=0; i<MAX_OUTPUT_REGS; i++) {
    status = pasynInt32SyncIO->connect(ANG1OutPortName, i, &pasynUserOutReg_[i], NULL);
  }
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to ANG1 controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new ANG1Axis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new ANG1Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ANG1InPortName    The name of the drvAsynIPPPort that was created previously to connect to the ANG1 controller 
  * \param[in] ANG1OutPortName   The name of the drvAsynIPPPort that was created previously to connect to the ANG1 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int ANG1CreateController(const char *portName, const char *ANG1InPortName, const char *ANG1OutPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  ANG1Controller *pANG1Controller
    = new ANG1Controller(portName, ANG1InPortName, ANG1OutPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pANG1Controller = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void ANG1Controller::report(FILE *fp, int level)
{
  fprintf(fp, "ANG1 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an ANG1Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ANG1Axis* ANG1Controller::getAxis(asynUser *pasynUser)
{
//  ?  return static_cast<ANG1Axis*>(asynMotorController::getAxis(pANG1Axis methodsasynUser));
  return static_cast<ANG1Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an ANG1Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] No Axis index number. */
ANG1Axis* ANG1Controller::getAxis(int axisNo)
{
  return static_cast<ANG1Axis*>(asynMotorController::getAxis(axisNo));
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library (?)
  * If the function is ANG1Jerk_ it sets the jerk value in the controller.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * For all other functions it calls asynMotorController::writeInt32.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ANG1Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ANG1Axis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  /* Set the parameter and readback in the parameter library. */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  
  if (function == ANG1Jerk_)
  {
    // Jerk in units steps/sec/sec/sec (0 - 5000)
    printf("Jerk = %d\n", value);
	status = writeReg16(JERK, value, DEFAULT_CONTROLLER_TIMEOUT);

//    sprintf(outString_, "%s JOG JRK %f", pAxis->axisName_, value);
//    status = writeController();
    
  } else {
    /* Call base class method */
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

asynStatus ANG1Controller::writeReg16(int reg, int output, double timeout)
{
  asynStatus status;
  
  //printf("writeReg16: writing %d to register %d\n", output, reg);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"writeReg16: writing %d to register %d\n", output, reg);  
  status = pasynInt32SyncIO->write(pasynUserOutReg_[reg], output, timeout);
  epicsThreadSleep(0.01);
                                  
  return status ;
}

asynStatus ANG1Controller::writeReg32(int reg, int output, double timeout)
{
//.. break 32-bit integer into 2 pieces
//.. write the pieces into ANG1 registers

  asynStatus status;
  float fnum;
  int lower,upper;
  
  fnum = (output / 1000.0);
  upper = (int)fnum;
  fnum = fnum - upper;
  fnum = NINT(fnum * 1000);
  lower = (int)fnum;

//could write the words this way  
//  status = pasynInt32SyncIO->write(pasynUserOutReg_[reg], upper, timeout);
//  status = pasynInt32SyncIO->write(pasynUserOutReg_[reg+1], lower, timeout);
  
//or this way  
//  writeReg16(piece1 ie MSW ...
  status = writeReg16(reg, upper, DEFAULT_CONTROLLER_TIMEOUT);
  
//  writeReg16(piece2 ie LSW ...
  reg++;
  status = writeReg16(reg, lower, DEFAULT_CONTROLLER_TIMEOUT);

  return status ;
}

asynStatus ANG1Controller::readReg16(int reg, epicsInt32 *input, double timeout)
{
  asynStatus status;
  
  //printf("reg = %d\n", reg);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"readReg16 reg = %d\n", reg);
  status = pasynInt32SyncIO->read(pasynUserInReg_[reg], input, timeout);
                                  
  return status ;
}

asynStatus ANG1Controller::readReg32(int reg, epicsInt32 *combo, double timeout)
{
//.. read 2 16-bit words from ANG1 registers
//.. assemble 2 16-bit pieces into 1 32-bit integer

  asynStatus status;
//  float fnum;
  epicsInt32 lowerWord32, upperWord32;        // only have pasynInt32SyncIO, not pasynInt16SyncIO ,
  epicsInt16 lowerWord16, upperWord16;        // so we need to get 32-bits and cast to 16-bit integer
  
  //printf("calling readReg16\n");
  status = readReg16(reg, &upperWord32, timeout);    //get Upper Word
  upperWord16 = (epicsInt16)upperWord32;
  //printf("upperWord16: %d\n", upperWord16);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"readReg32 upperWord16: %d\n", upperWord16);

  // if status != 1 :
  reg++;
  status = readReg16(reg, &lowerWord32, timeout);  //get Lower Word
  lowerWord16 = (epicsInt16)lowerWord32;
  //printf("lowerWord16: %d\n", lowerWord16);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"readReg32 lowerWord16: %d\n", lowerWord16);
  
  *combo = NINT((upperWord16 * 1000) + lowerWord16);
  
  return status ;
}


// ANG1Axis methods Here
// These are the ANG1Axis methods

/** Creates a new ANG1Axis object.
  * \param[in] pC Pointer to the ANG1Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
ANG1Axis::ANG1Axis(ANG1Controller *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)	
{ 
  int status;
  
  //status = pasynInt32SyncIO->connect(myModbusInputDriver, 0, &pasynUserForceRead_, "MODBUS_READ");
  status = pasynInt32SyncIO->connect(pC_->inputDriver_, 0, &pasynUserForceRead_, "MODBUS_READ");
  if (status) {
    //printf("%s:%s: Error, unable to connect pasynUserForceRead_ to Modbus input driver %s\n", pC_->inputDriver_, pC_->functionName, myModbusInputDriver);
    printf("%s: Error, unable to connect pasynUserForceRead_ to Modbus input driver\n", pC_->inputDriver_);	
  }
  printf("ANG1Axis::ANG1Axis : pasynUserForceRead_->reason=%d\n", pasynUserForceRead_->reason);
  
  // set position to 0
  setPosition(0);
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void ANG1Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

// SET VEL & ACCEL
asynStatus ANG1Axis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "ANG1::sendAccelAndVelocity";

  // Send the velocity in egus
  //sprintf(pC_->outString_, "%1dVA%f", axisNo_ + 1, (velocity*stepSize_));
  //status = pC_->writeController();
  status = pC_->writeReg32(SPD_UPR, NINT(velocity), DEFAULT_CONTROLLER_TIMEOUT);

  // Send the acceleration in egus/sec/sec
  //printf("    velocity: %f\n", velocity);
  //printf("    acceleration: %f\n", acceleration);
  // ANG1 acceleration range 1 to 5000 steps/ms/sec
  // Therefore need to limit range received by motor record from 1000 to 5e6 steps/sec/sec
  if (acceleration < 1000) {
    // print message noting that accel has been capped low
    acceleration = 1000;
  }
  if (acceleration > 5000000) {
    // print message noting that accel has been capped high
    acceleration = 5000000;
  }
  // ANG1 acceleration units are steps/millisecond/second, so we divide by 1000 here
  status = pC_->writeReg16(ACCEL, NINT(acceleration/1000.0), DEFAULT_CONTROLLER_TIMEOUT);
  status = pC_->writeReg16(DECEL, NINT(acceleration/1000.0), DEFAULT_CONTROLLER_TIMEOUT);
  return status;
}

// MOVE
asynStatus ANG1Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int velo, distance, move_bit;
  
  printf(" ** ANG1Axis::move called, relative = %d\n", relative);

  status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  //velo = maxVelocity * SOME_SCALE_FACTOR
  velo = NINT(maxVelocity);
  if (relative) {
    printf(" ** relative move called\n");
    //status = pC_->writeReg32(SPD_UPR, velo, DEFAULT_CONTROLLER_TIMEOUT);
	//distance = position *  SOM_OTHER_SCALE_FACTOR;
	distance = NINT(position);
    status = pC_->writeReg32(POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x0;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x2;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
  } else {
    // absolute
    printf(" ** absolute move called\n");
    //status = pC_->writeReg32(SPD_UPR, velo, DEFAULT_CONTROLLER_TIMEOUT);
	//distance = position *  SOM_OTHER_SCALE_FACTOR;
	distance = NINT(position);
	printf(" ** distance = %d\n", distance);
    status = pC_->writeReg32(POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x0;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x1;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);	
  }
  // Delay the first status read, give the controller some time to return moving status
  epicsThreadSleep(0.05);
  return status;
}

// HOME (needs work)
asynStatus ANG1Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  int home_bit;
  // static const char *functionName = "ANG1Axis::home";

  //status = sendAccelAndVelocity(acceleration, maxVelocity);

  if (forwards) {
    printf(" ** HOMING FORWARDS **\n");
    home_bit = 0x20;
	status = pC_->writeReg16(CMD_MSW, home_bit, DEFAULT_CONTROLLER_TIMEOUT);
  } else {
    home_bit = 0x40;
    status = pC_->writeReg16(CMD_MSW, home_bit, DEFAULT_CONTROLLER_TIMEOUT);
  }
  return status;
}

// JOG
asynStatus ANG1Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int velo, distance, move_bit;
  static const char *functionName = "ANG1Axis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);

  velo = NINT(fabs(maxVelocity));
    
  status = sendAccelAndVelocity(acceleration, velo);

  /* ANG1 does not have jog command. Move 1 million steps */
  if (maxVelocity > 0.) {
    /* This is a positive move in ANG1 coordinates */
	//printf(" ** relative move (JOG pos) called\n");
	distance = 1000000;
    status = pC_->writeReg32(POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x0;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x2;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
  } else {
    /* This is a negative move in ANG1 coordinates */
    //printf(" ** relative move (JOG neg) called\n");
	distance = -1000000;
    status = pC_->writeReg32(POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x0;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x2;
    status = pC_->writeReg16(CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
  }
  // Delay the first status read, give the controller some time to return moving status
  epicsThreadSleep(0.05);
  return status;
}


// STOP
asynStatus ANG1Axis::stop(double acceleration)
{
  asynStatus status;
  int stop_bit;
  //static const char *functionName = "ANG1Axis::stop";
  
  printf("\n  STOP \n\n");
  
  stop_bit = 0x0;
  status = pC_->writeReg16(CMD_MSW, stop_bit, DEFAULT_CONTROLLER_TIMEOUT);

//  stop_bit = 0x10;      Immediate stop
  stop_bit = 0x4;      // Hold move
  status = pC_->writeReg16(CMD_MSW, stop_bit, DEFAULT_CONTROLLER_TIMEOUT);

  return status;
}

// SET
asynStatus ANG1Axis::setPosition(double position)
{
  asynStatus status;
  int set_position, set_bit;
  //static const char *functionName = "ANG1Axis::setPosition";

  //status = writeReg32(SPD_UPR, velo, DEFAULT_CONTROLLER_TIMEOUT);
  //distance = position *  SOM_OTHER_SCALE_FACTOR;
  set_position = NINT(position);
  
  status = pC_->writeReg32(POS_WR_UPR, set_position, DEFAULT_CONTROLLER_TIMEOUT);

  set_bit = 0x200;
  status = pC_->writeReg16(CMD_MSW, set_bit, DEFAULT_CONTROLLER_TIMEOUT);

  set_bit = 0x0;
  status = pC_->writeReg16(CMD_MSW, set_bit, DEFAULT_CONTROLLER_TIMEOUT);

  return status;
}

// ENABLE TORQUE
asynStatus ANG1Axis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  int enable  = 0x8000;
  int disable = 0x0000;
  int cmd;
  
  printf(" ** setClosedLoop called \n");
  if (closedLoop) {
    printf("setting enable %X\n", enable);
	// Let's reset errors first
	cmd = 0x0;
    status = pC_->writeReg16(CMD_MSW, cmd, DEFAULT_CONTROLLER_TIMEOUT);

	cmd = 0x400;
    status = pC_->writeReg16(CMD_MSW, cmd, DEFAULT_CONTROLLER_TIMEOUT);	

	cmd = 0x0;
    status = pC_->writeReg16(CMD_MSW, cmd, DEFAULT_CONTROLLER_TIMEOUT);	
    status = pC_->writeReg16(CMD_LSW, enable, DEFAULT_CONTROLLER_TIMEOUT);
	setIntegerParam(pC_->motorStatusPowerOn_, 1);
	
  } else {
    printf("setting disable %X\n", disable);
    status = pC_->writeReg16(CMD_LSW, disable, DEFAULT_CONTROLLER_TIMEOUT);
    setIntegerParam(pC_->motorStatusPowerOn_, 0);
  }

  return status;
}

// POLL
/** Polls the axis.
  * This function reads motor position, limit status, home status, and moving status
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus ANG1Axis::poll(bool *moving)
{ 
  int done;
  int limit;
  int enabled;
  double position;
  asynStatus status;
  epicsInt32 read_val;  // don't use a pointer here.  The _address_ of read_val should be passed to the read function.
  
  // Force a read operation
  //printf(" . . . . . Calling pasynInt32SyncIO->write\n");
  //printf("Calling pasynInt32SyncIO->write(pasynUserForceRead_, 1, TIMEOUT), pasynUserForceRead_->reason=%d\n", pasynUserForceRead_->reason);
  status = pasynInt32SyncIO->write(pasynUserForceRead_, 1, DEFAULT_CONTROLLER_TIMEOUT);
  //printf(" . . . . . status = %d\n", status);
  // if status goto end

  // Read the current motor position
  // 
  //readReg32(int reg, epicsInt32 *combo, double timeout)
  status = pC_->readReg32(POS_RD_UPR, &read_val, DEFAULT_CONTROLLER_TIMEOUT);
  printf("ANG1Axis::poll:  Motor position raw: %d\n", read_val);
  position = (double) read_val;
  setDoubleParam(pC_->motorPosition_, position);
  printf("ANG1Axis::poll:  Motor position: %f\n", position);

  // Read the moving status of this motor
  //
  status = pC_->readReg16(STATUS_1, &read_val, DEFAULT_CONTROLLER_TIMEOUT);
  //printf("status 1 is 0x%X\n", read_val);
  
  // Done logic
  done = ((read_val & 0x8) >> 3);  // status word 1 bit 3 set to 1 when the motor is not in motion.
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;
  printf("done is %d\n", done);
  
  // Read the limit status
  //
  status = pC_->readReg16(STATUS_2, &read_val, DEFAULT_CONTROLLER_TIMEOUT);
  printf("status 2 is 0x%X\n", read_val);  
  
  limit  = (read_val & 0x1);    // a cw limit has been reached
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  //printf("+limit %d\n", limit);
    if (limit) {   // reset error and set position so we can move off of the limit
    // Reset error
	setClosedLoop(1);
	// Reset position
	//printf(" Reset Position\n");
	setPosition(position);
  }

  limit  = (read_val & 0x2);    // a ccw limit has been reached
  setIntegerParam(pC_->motorStatusLowLimit_, limit);
  //printf("-limit %d\n", limit);
  if (limit) {   // reset error and set position so we can move off of the limit
    // Reset error
	setClosedLoop(1);
	// Reset position
	setPosition(position);
  }

  // test for home
  
  // Should be in init routine?  Allows CNEN to be used.
  setIntegerParam(pC_->motorStatusGainSupport_, 1);

  // Check for the torque status and set accordingly.
  enabled = (read_val & 0x8000);
  if (enabled)
    setIntegerParam(pC_->motorStatusPowerOn_, 1);
  else
    setIntegerParam(pC_->motorStatusPowerOn_, 0);
  
  // Notify asynMotorController polling routine that we're ready
  callParamCallbacks();

  return status;
}

/** Code for iocsh registration */
static const iocshArg ANG1CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ANG1CreateControllerArg1 = {"ANG1 In port name", iocshArgString};
static const iocshArg ANG1CreateControllerArg2 = {"ANG1 Out port name", iocshArgString};
static const iocshArg ANG1CreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg ANG1CreateControllerArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ANG1CreateControllerArg5 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const ANG1CreateControllerArgs[] = {&ANG1CreateControllerArg0,
                                                             &ANG1CreateControllerArg1,
                                                             &ANG1CreateControllerArg2,
                                                             &ANG1CreateControllerArg3,
                                                             &ANG1CreateControllerArg4,
															 &ANG1CreateControllerArg5,};
static const iocshFuncDef ANG1CreateControllerDef = {"ANG1CreateController", 6, ANG1CreateControllerArgs};
static void ANG1CreateContollerCallFunc(const iocshArgBuf *args)
{
  ANG1CreateController(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival, args[5].ival);
}

static void ANG1Register(void)
{
  iocshRegister(&ANG1CreateControllerDef, ANG1CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(ANG1Register);
}
