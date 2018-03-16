/*
FILENAME... ANF2Driver.cpp
USAGE...    Motor record driver support for the AMCI ANF2 stepper motor controller over Modbus/TCP.

Kevin Peterson

Based on the AMCI ANG1 Model 3 device driver written by Kurt Goetze

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

#include "ANF2Driver.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

static const char *driverName = "ANF2MotorDriver";

/** Constructor, Creates a new ANF2Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ANF2InPortName    The name of the drvAsynSerialPort that was created previously to connect to the ANF2 controller 
  * \param[in] ANF2OutPortName   The name of the drvAsynSerialPort that was created previously to connect to the ANF2 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
ANF2Controller::ANF2Controller(const char *portName, const char *ANF2InPortName, const char *ANF2OutPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_ANF2_PARAMS, 
                         asynInt32ArrayMask, // One additional interface beyond those in base class
                         asynInt32ArrayMask, // One additional callback interface beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int i, j;
  asynStatus status = asynSuccess;
  static const char *functionName = "ANF2Controller::ANF2Controller";
  
  // Keep track of the number of axes created, so the poller can wait for all the axes to be created before starting
  axesCreated_ = 0;
  
  inputDriver_ = epicsStrDup(ANF2InPortName);    // Set this before calls to create Axis objects
  
  // Create controller-specific parameters
  createParam(ANF2GetInfoString,         asynParamInt32,       &ANF2GetInfo_);
  
  if (numAxes > MAX_AXES) {
    numAxes = MAX_AXES;
  }
  
  for (j=0; j<numAxes; j++) {
    /* Connect to ANF2 controller */
    for (i=0; i<MAX_INPUT_REGS; i++) {
      status = pasynInt32SyncIO->connect(ANF2InPortName, i, &pasynUserInReg_[j][i], NULL);
    }
    for (i=0; i<MAX_OUTPUT_REGS; i++) {
      status = pasynInt32SyncIO->connect(ANF2OutPortName, i, &pasynUserOutReg_[j][i], NULL);
      // Maybe send the outputs with Array calls in the future
      //status = pasynInt32ArraySyncIO->connect(ANF2OutPortName, i, &pasynUserOutArrayReg_[j][i], NULL);
    }
  }
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to ANF2 controller\n",
      functionName);
  }

  /* Create the poller thread for this controller (do 2 forced-fast polls)
   * NOTE: at this point the axis objects don't yet exist, but the poller tolerates this */
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new ANF2Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ANF2InPortName    The name of the drvAsynIPPPort that was created previously to connect to the ANF2 controller 
  * \param[in] ANF2OutPortName   The name of the drvAsynIPPPort that was created previously to connect to the ANF2 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int ANF2CreateController(const char *portName, const char *ANF2InPortName, const char *ANF2OutPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  /*
  ANF2Controller *pANF2Controller
    = new ANF2Controller(portName, ANF2InPortName, ANF2OutPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pANF2Controller = NULL;
  */
  new ANF2Controller(portName, ANF2InPortName, ANF2OutPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void ANF2Controller::report(FILE *fp, int level)
{
  fprintf(fp, "ANF2 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
  fprintf(fp, "    axesCreated=%i\n", axesCreated_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an ANF2Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ANF2Axis* ANF2Controller::getAxis(asynUser *pasynUser)
{
//  ?  return static_cast<ANF2Axis*>(asynMotorController::getAxis(pANF2Axis methodsasynUser));
  return static_cast<ANF2Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an ANF2Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] No Axis index number. */
ANF2Axis* ANF2Controller::getAxis(int axisNo)
{
  return static_cast<ANF2Axis*>(asynMotorController::getAxis(axisNo));
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library (?)
  *
  * If the function is ANF2Jerk_ it sets the jerk value in the controller.
  * Calls any registered callbacks for this pasynUser->reason and address.
  *
  * For all other functions it calls asynMotorController::writeInt32.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ANF2Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ANF2Axis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";
  
  /* Set the parameter and readback in the parameter library. */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  
  if (function == ANF2GetInfo_)
  {
    // Only get info when value is 1
    if (value == 1) {
        printf("ANF2Controller:writeInt32: Getting info for axis = %d\n", pAxis->axisNo_);
        pAxis->getInfo();

    }
  } else {
  // Call base class method
    status = asynMotorController::writeInt32(pasynUser, value);
  }
  
  // Call base class method
  status = asynMotorController::writeInt32(pasynUser, value);
  
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

asynStatus ANF2Controller::writeReg16(int axisNo, int axisReg, int output, double timeout)
{
  asynStatus status;

  //printf("writeReg16: writing %d to register %d\n", output, axisReg);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"writeReg16: writing %d to register %d\n", output, axisReg);  
  status = pasynInt32SyncIO->write(pasynUserOutReg_[axisNo][axisReg], output, timeout);
  //status = pasynInt32ArraySyncIO->write(pasynUserOutArrayReg_[axisNo][axisReg], &output, 1, timeout);
  epicsThreadSleep(0.01);
                                  
  return status ;
}

// This could be useful in the future, but it isn't needed yet
/*asynStatus ANF2Controller::writeReg32Array(int axisNo, int axisReg, epicsInt32* output, int nElements, double timeout)
{
  asynStatus status;
  
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"writeReg32Array: writing %d elements starting at register %d\n", nElements, axisReg);  
  status = pasynInt32ArraySyncIO->write(pasynUserOutArrayReg_[axisNo][axisReg], output, nElements, timeout);
    
  return status ;
}*/

asynStatus ANF2Controller::writeReg32(int axisNo, int axisReg, int output, double timeout)
{
//.. break 32-bit integer into 2 pieces
//.. write the pieces into ANF2 registers

  asynStatus status;

  int lower,upper;
  
  // This is the way the ANG1 driver does it, and the code doesn't appear to work
  /*float fnum;

  fnum = (output / 1000.0);
  upper = (int)fnum;
  fnum = fnum - upper;
  fnum = NINT(fnum * 1000);
  lower = (int)fnum;*/
  
  upper = (output >> 16) & 0x0000FFFF;
  lower = output & 0x0000FFFF;

  printf("upper = 0x%x\t= %i\n", upper, upper);
  printf("lower = 0x%x\t= %i\n", lower, lower);

  //  writeReg16(piece1 ie MSW ...
  status = writeReg16(axisNo, axisReg, upper, DEFAULT_CONTROLLER_TIMEOUT);
  
  //  writeReg16(piece2 ie LSW ...
  axisReg++;
  status = writeReg16(axisNo, axisReg, lower, DEFAULT_CONTROLLER_TIMEOUT);

  // No breaking up the output value required when writing an array - maybe do this in the future
  //status = pasynInt32ArraySyncIO->write(pasynUserOutArrayReg_[axisNo][axisReg], &output, 2, timeout);

  return status ;
}

asynStatus ANF2Controller::readReg16(int axisNo, int axisReg, epicsInt32 *input, double timeout)
{
  asynStatus status;
  
  //printf("axisReg = %d\n", axisReg);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"readReg16 reg = %d\n", axisReg);
  status = pasynInt32SyncIO->read(pasynUserInReg_[axisNo][axisReg], input, timeout);
                                  
  return status ;
}

asynStatus ANF2Controller::readReg32(int axisNo, int axisReg, epicsInt32 *combo, double timeout)
{
//.. read 2 16-bit words from ANF2 registers
//.. assemble 2 16-bit pieces into 1 32-bit integer

  asynStatus status;
//  float fnum;
  epicsInt32 lowerWord32, upperWord32;        // only have pasynInt32SyncIO, not pasynInt16SyncIO ,
  epicsInt16 lowerWord16, upperWord16;        // so we need to get 32-bits and cast to 16-bit integer
  
  //printf("calling readReg16\n");
  status = readReg16(axisNo, axisReg, &upperWord32, timeout);    //get Upper Word
  upperWord16 = (epicsInt16)upperWord32;
  //printf("upperWord16: %d\n", upperWord16);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"readReg32 upperWord16: %d\n", upperWord16);

  // if status != 1 :
  axisReg++;
  status = readReg16(axisNo, axisReg, &lowerWord32, timeout);  //get Lower Word
  lowerWord16 = (epicsInt16)lowerWord32;
  //printf("lowerWord16: %d\n", lowerWord16);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"readReg32 lowerWord16: %d\n", lowerWord16);
  
  *combo = NINT((upperWord16 * 1000) + lowerWord16);
  
  return status ;
}


// ANF2Axis methods Here
// These are the ANF2Axis methods

/** Creates a new ANF2Axis object.
  * \param[in] pC Pointer to the ANF2Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
ANF2Axis::ANF2Axis(ANF2Controller *pC, const char *ANF2ConfName, int axisNo, epicsInt32 config)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)	
{ 
  int status;
  epicsInt32 configBits[2];
  
  axisNo_ = axisNo;
  //this->axisNo_ = axisNo;

  registers_[0] = 0x0;
  registers_[1] = 0x0;
  registers_[2] = 0x0;
  registers_[3] = 0x0;
  registers_[4] = 0x0;

  status = pasynInt32SyncIO->connect(pC_->inputDriver_, 0, &pasynUserForceRead_, "MODBUS_READ");
  if (status) {
    //printf("%s:%s: Error, unable to connect pasynUserForceRead_ to Modbus input driver %s\n", pC_->inputDriver_, pC_->functionName, myModbusInputDriver);
    printf("%s: Error, unable to connect pasynUserForceRead_ to Modbus input driver\n", pC_->inputDriver_);	
  }
  printf("ANF2Axis::ANF2Axis : pasynUserForceRead_->reason=%d\n", pasynUserForceRead_->reason);

  status = pasynInt32ArraySyncIO->connect(ANF2ConfName, 0, &pasynUserConfWrite_, NULL);
  if (status) {
    printf("%s: Error, unable to connect pasynUserConfWrite_ to Modbus input driver\n", ANF2ConfName);	
  }
  printf("ANF2Axis::ANF2Axis : pasynUserConfWrite_->reason=%d\n", pasynUserConfWrite_->reason);

  epicsThreadSleep(1.0);

  // Read data that is likely to be stale
  getInfo();

  // Send the configuration
  //status = pC_->writeReg32(axisNo_, CONFIG_MSW, config, DEFAULT_CONTROLLER_TIMEOUT);
 
  // Send the configuration (array) 
  // assemble the configuration bits; set the start speed to a non-zero value (100), which is required for the configuration to be accepted
  configBits[0] = config;
  configBits[1] = 0x00000064;
  
  registers_[0] = config;
  registers_[1] = 0x00000064;
  
  // Does the number of elements refer to the number of 16-bit elements?
  status = pasynInt32ArraySyncIO->write(pasynUserConfWrite_, configBits, 4, DEFAULT_CONTROLLER_TIMEOUT);
  // Mabye do it this way in the future
  //status = this->pC_->writeReg32Array(axisNo, CONFIG_MSW, configBits, 4, DEFAULT_CONTROLLER_TIMEOUT);
  // Write all the registers
  status = pasynInt32ArraySyncIO->write(pasynUserConfWrite_, registers_, 10, DEFAULT_CONTROLLER_TIMEOUT);

  // Delay
  epicsThreadSleep(1.0);

  // Read the configuration? Or maybe the command registers?
  getInfo();
  
  // set position to 0
  //setPosition(0);
  setPosition(1337);
  //setPosition(3141);
  
  // Delay
  epicsThreadSleep(1.0);
  
  // Read the command registers
  getInfo();
  
  // Tell the driver the axis has been created
  pC_->axesCreated_ += 1;
}

/*
 Configuration Bits:
 0x1  - Caputure Input (0 = Disabled, 1 = Enabled)
 0x2  - External Input (0 = Disabled, 1 = Enabled)
 0x4  - Home Input (0 = Disabled, 1 = Enabled)
 0x8  - 

 */

extern "C" asynStatus ANF2CreateAxis(const char *ANF2Name,  /* specify which controller by port name */
                         const char *ANF2ConfName,          /* specify which config port name */
                         int axis,                         /* axis number 0-1 */
                         const char *hexConfig)            /* desired configuration in hex */
{
  ANF2Controller *pC;
  epicsInt32 config;
  static const char *functionName = "ANF2CreateAxis";

  pC = (ANF2Controller*) findAsynPortDriver(ANF2Name);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, ANF2Name);
    return asynError;
  }

  errno = 0;
  config = strtoul(hexConfig, NULL, 16);
  if (errno != 0) {
    printf("%s:%s: Error invalid config=%s\n",
           driverName, functionName, hexConfig);
    return asynError;
  } else {
    printf("%s:%s: Config=>%s=%x\n",
           driverName, functionName, hexConfig, config);
  }
  
  pC->lock();
  new ANF2Axis(pC, ANF2ConfName, axis, config);
  pC->unlock();
  return asynSuccess;
}

void ANF2Axis::getInfo()
{
  asynStatus status;
  int i;
  epicsInt32 read_val;
  
  // For a read (not sure why this is necessary)
  status = pasynInt32SyncIO->write(pasynUserForceRead_, 1, DEFAULT_CONTROLLER_TIMEOUT);

  printf("Info for axis %i\n", axisNo_);

  for( i=0; i<MAX_INPUT_REGS; i++)
  {
    status = pC_->readReg16(axisNo_, i, &read_val, DEFAULT_CONTROLLER_TIMEOUT);
    printf("  status=%d, register=%i, val=0x%x\n", status, i, read_val);
  }
}


/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void ANF2Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
    fprintf(fp, "    this->axisNo_ %i\n", this->axisNo_);
    fprintf(fp, "    this->config_ %x\n", this->config_);
    fprintf(fp, "    config_ %x\n", config_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

// SET VEL & ACCEL
asynStatus ANF2Axis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "ANF2::sendAccelAndVelocity";

  // Send the velocity
  //status = pC_->writeReg32(axisNo_, SPD_UPR, NINT(velocity), DEFAULT_CONTROLLER_TIMEOUT);

  // Set the velocity register
  registers_[2] = NINT(velocity);

  // Send the acceleration
  // ANF2 acceleration range 1 to 5000 steps/ms/sec
  // Therefore need to limit range received by motor record from 1000 to 5e6 steps/sec/sec
  if (acceleration < 1000) {
    // print message noting that accel has been capped low
    //printf("Acceleration is < 1000: %lf\n", acceleration);
    acceleration = 1000;
  }
  if (acceleration > 5000000) {
    // print message noting that accel has been capped high
    //printf("Acceleration is > 5000: %lf\n", acceleration);
    acceleration = 5000000;
  }
  // ANF2 acceleration units are steps/millisecond/second, so we divide by 1000 here
  //status = pC_->writeReg16(axisNo_, ACCEL, NINT(acceleration/1000.0), DEFAULT_CONTROLLER_TIMEOUT);
  //status = pC_->writeReg16(axisNo_, DECEL, NINT(acceleration/1000.0), DEFAULT_CONTROLLER_TIMEOUT);
  
  // Set the accel/decel register
  registers_[3] = (NINT(acceleration/1000.0) << 16) | (NINT(acceleration/1000.0));
    
  return status;
}

// MOVE
asynStatus ANF2Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int distance, move_bit;
  
  printf(" ** ANF2Axis::move called, relative = %d, axisNo_ = %i\n", relative, this->axisNo_);

  status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  if (relative) {
    printf(" ** relative move called\n");
    //status = pC_->writeReg32(axisNo_, SPD_UPR, velo, DEFAULT_CONTROLLER_TIMEOUT);
    distance = NINT(position);
    //status = pC_->writeReg32(axisNo_, POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
    
    move_bit = 0x0;
    //status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
    
    move_bit = 0x2;
    //status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
  
    // Set position and cmd registers
    registers_[1] = NINT(position);
    registers_[0] = 0x2 << 16;
  
  } else {
    // absolute
    printf(" ** absolute move called\n");
    //status = pC_->writeReg32(axisNo_, SPD_UPR, velo, DEFAULT_CONTROLLER_TIMEOUT);
    distance = NINT(position);
    printf(" ** distance = %d\n", distance);
    //status = pC_->writeReg32(axisNo_, POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
    
    move_bit = 0x0;
    //status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
    
    move_bit = 0x1;
    //status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);	
  
    // Set position and cmd registers
    registers_[1] = NINT(position);
    registers_[0] = 0x1 << 16;
  
  }
  
  // The final registers are zero for absolute and relative moves
  registers_[4] = 0x0;
  
  // Write all the registers atomically IAMHERE
  // Does the number of elements refer to the number of 16-bit elements?
  status = pasynInt32ArraySyncIO->write(pasynUserConfWrite_, registers_, 10, DEFAULT_CONTROLLER_TIMEOUT);
  
  // Delay the first status read, give the controller some time to return moving status
  epicsThreadSleep(0.05);
  return status;
}

// HOME (needs work)
asynStatus ANF2Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  int home_bit;
  // static const char *functionName = "ANF2Axis::home";

  //status = sendAccelAndVelocity(acceleration, maxVelocity);

  if (forwards) {
    printf(" ** HOMING FORWARDS **\n");
    home_bit = 0x20;
	status = pC_->writeReg16(axisNo_, CMD_MSW, home_bit, DEFAULT_CONTROLLER_TIMEOUT);
  } else {
    home_bit = 0x40;
    status = pC_->writeReg16(axisNo_, CMD_MSW, home_bit, DEFAULT_CONTROLLER_TIMEOUT);
  }
  return status;
}

// JOG
asynStatus ANF2Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  int velo, distance, move_bit;
  static const char *functionName = "ANF2Axis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);

  velo = NINT(fabs(maxVelocity));
    
  status = sendAccelAndVelocity(acceleration, velo);

  /* ANF2 does not have jog command. Move 1 million steps */
  if (maxVelocity > 0.) {
    /* This is a positive move in ANF2 coordinates */
	//printf(" ** relative move (JOG pos) called\n");
	distance = 1000000;
    status = pC_->writeReg32(axisNo_, POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x0;
    status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x2;
    status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
  } else {
    /* This is a negative move in ANF2 coordinates */
    //printf(" ** relative move (JOG neg) called\n");
	distance = -1000000;
    status = pC_->writeReg32(axisNo_, POS_WR_UPR, distance, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x0;
    status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
	move_bit = 0x2;
    status = pC_->writeReg16(axisNo_, CMD_MSW, move_bit, DEFAULT_CONTROLLER_TIMEOUT);
  }
  // Delay the first status read, give the controller some time to return moving status
  epicsThreadSleep(0.05);
  return status;
}


// STOP
asynStatus ANF2Axis::stop(double acceleration)
{
  asynStatus status;
  int stop_bit;
  //static const char *functionName = "ANF2Axis::stop";
  
  printf("\n  STOP \n\n");
  
  stop_bit = 0x0;
  status = pC_->writeReg16(axisNo_, CMD_MSW, stop_bit, DEFAULT_CONTROLLER_TIMEOUT);

//  stop_bit = 0x10;      Immediate stop
  stop_bit = 0x4;      // Hold move
  status = pC_->writeReg16(axisNo_, CMD_MSW, stop_bit, DEFAULT_CONTROLLER_TIMEOUT);

  return status;
}

// SET
asynStatus ANF2Axis::setPosition(double position)
{
  asynStatus status;
  int set_position, set_bit;
  //static const char *functionName = "ANF2Axis::setPosition";

  //status = writeReg32(SPD_UPR, velo, DEFAULT_CONTROLLER_TIMEOUT);
  //distance = position *  SOM_OTHER_SCALE_FACTOR;
  set_position = NINT(position);
  
  status = pC_->writeReg32(axisNo_, POS_WR_UPR, set_position, DEFAULT_CONTROLLER_TIMEOUT);

  set_bit = 0x200;
  status = pC_->writeReg16(axisNo_, CMD_MSW, set_bit, DEFAULT_CONTROLLER_TIMEOUT);

  set_bit = 0x0;
  status = pC_->writeReg16(axisNo_, CMD_MSW, set_bit, DEFAULT_CONTROLLER_TIMEOUT);

  return status;
}

// ENABLE TORQUE
asynStatus ANF2Axis::setClosedLoop(bool closedLoop)
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
    status = pC_->writeReg16(axisNo_, CMD_MSW, cmd, DEFAULT_CONTROLLER_TIMEOUT);

	cmd = 0x400;
    status = pC_->writeReg16(axisNo_, CMD_MSW, cmd, DEFAULT_CONTROLLER_TIMEOUT);	

	cmd = 0x0;
    status = pC_->writeReg16(axisNo_, CMD_MSW, cmd, DEFAULT_CONTROLLER_TIMEOUT);	
    /*
    status = pC_->writeReg16(axisNo_, CMD_LSW, enable, DEFAULT_CONTROLLER_TIMEOUT);
	setIntegerParam(pC_->motorStatusPowerOn_, 1);
    */
	
  } else {
    printf("setting disable %X\n", disable);
    status = pC_->writeReg16(axisNo_, CMD_LSW, disable, DEFAULT_CONTROLLER_TIMEOUT);
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
asynStatus ANF2Axis::poll(bool *moving)
{ 
  int done;
  int limit;
  int enabled;
  double position;
  asynStatus status;
  epicsInt32 read_val;  // don't use a pointer here.  The _address_ of read_val should be passed to the read function.
  
  // Don't do any polling until ALL the axes have been created; this ensures that we don't interpret the configuration values as command values
  if (pC_->axesCreated_ != pC_->numAxes_) {
    *moving = false;
    return asynSuccess;
  }
  
  //getInfo();
  
  // Force a read operation
  //printf(" . . . . . Calling pasynInt32SyncIO->write\n");
  //printf("Calling pasynInt32SyncIO->write(pasynUserForceRead_, 1, TIMEOUT), pasynUserForceRead_->reason=%d\n", pasynUserForceRead_->reason);
  status = pasynInt32SyncIO->write(pasynUserForceRead_, 1, DEFAULT_CONTROLLER_TIMEOUT);
  //printf(" . . . . . status = %d\n", status);
  // if status goto end

  //getInfo();

  // Read the current motor position
  // 
  //readReg32(int reg, epicsInt32 *combo, double timeout)
  status = pC_->readReg32(axisNo_, POS_RD_UPR, &read_val, DEFAULT_CONTROLLER_TIMEOUT);
  //printf("ANF2Axis::poll:  Motor position raw: %d\n", read_val);
  position = (double) read_val;
  setDoubleParam(pC_->motorPosition_, position);
  //printf("ANF2Axis::poll:  Motor position: %f\n", position);

  // Read the moving status of this motor
  //
  status = pC_->readReg16(axisNo_, STATUS_1, &read_val, DEFAULT_CONTROLLER_TIMEOUT);
  //printf("status 1 is 0x%X\n", read_val);
  
  // Done logic
  done = ((read_val & 0x8) >> 3);  // status word 1 bit 3 set to 1 when the motor is not in motion.
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;
  //printf("done is %d\n", done);
  
  // Read the limit status
  //
  status = pC_->readReg16(axisNo_, STATUS_2, &read_val, DEFAULT_CONTROLLER_TIMEOUT);
  //printf("status 2 is 0x%X\n", read_val);  
  
  limit  = (read_val & 0x8);    // a cw limit has been reached
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  //printf("+limit %d\n", limit);
    if (limit) {   // reset error and set position so we can move off of the limit
    // Reset error
	setClosedLoop(1);
	// Reset position
	//printf(" Reset Position\n");
	setPosition(position);
  }

  limit  = (read_val & 0x10);    // a ccw limit has been reached
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
  // The ANG1 driver does the wrong thing for torque enable/disable
  //enabled = (read_val & 0x8000);
  enabled = 1;
  if (enabled)
    setIntegerParam(pC_->motorStatusPowerOn_, 1);
  else
    setIntegerParam(pC_->motorStatusPowerOn_, 0);
  
  // Notify asynMotorController polling routine that we're ready
  callParamCallbacks();

  return status;
}

/** Code for iocsh registration */

/* ANF2CreateController */
static const iocshArg ANF2CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ANF2CreateControllerArg1 = {"ANF2 In port name", iocshArgString};
static const iocshArg ANF2CreateControllerArg2 = {"ANF2 Out port name", iocshArgString};
static const iocshArg ANF2CreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg ANF2CreateControllerArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ANF2CreateControllerArg5 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const ANF2CreateControllerArgs[] = {&ANF2CreateControllerArg0,
                                                             &ANF2CreateControllerArg1,
                                                             &ANF2CreateControllerArg2,
                                                             &ANF2CreateControllerArg3,
                                                             &ANF2CreateControllerArg4,
							     &ANF2CreateControllerArg5,};
static const iocshFuncDef ANF2CreateControllerDef = {"ANF2CreateController", 6, ANF2CreateControllerArgs};
static void ANF2CreateControllerCallFunc(const iocshArgBuf *args)
{
  ANF2CreateController(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival, args[5].ival);
}


/* ANF2CreateAxis */
static const iocshArg ANF2CreateAxisArg0 = {"Port name", iocshArgString};
static const iocshArg ANF2CreateAxisArg1 = {"Config port name", iocshArgString};
static const iocshArg ANF2CreateAxisArg2 = {"Axis number", iocshArgInt};
static const iocshArg ANF2CreateAxisArg3 = {"Hex config", iocshArgString};
static const iocshArg * const ANF2CreateAxisArgs[] = {&ANF2CreateAxisArg0,
                                                             &ANF2CreateAxisArg1,
                                                             &ANF2CreateAxisArg2,
                                                             &ANF2CreateAxisArg3};
static const iocshFuncDef ANF2CreateAxisDef = {"ANF2CreateAxis", 4, ANF2CreateAxisArgs};
static void ANF2CreateAxisCallFunc(const iocshArgBuf *args)
{
  ANF2CreateAxis(args[0].sval, args[1].sval, args[2].ival, args[3].sval);
}


static void ANF2Register(void)
{
  iocshRegister(&ANF2CreateControllerDef, ANF2CreateControllerCallFunc);
  iocshRegister(&ANF2CreateAxisDef, ANF2CreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(ANF2Register);
}
