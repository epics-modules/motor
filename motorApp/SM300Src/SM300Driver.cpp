/*
Motor driver support for the SM300 controller.

Based on the SM100 Model 3 device driver 
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include <errlog.h>
#include "SM300Driver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new SM300Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SM300PortName     The name of the drvAsynSerialPort that was created previously to connect to the SM300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SM300Controller::SM300Controller(const char *portName, const char *SM300PortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod, double stepSize)
  : is_moving_(false),
	asynMotorController(portName, numAxes, NUM_SM300_PARAMS,
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  SM300Axis *pAxis;
  static const char *functionName = "SM300Controller::SM300Controller";

  /* Connect to SM300 controller */
  status = pasynOctetSyncIO->connect(SM300PortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to SM300 controller\n",
      functionName);
  }
  if (numAxes != 2) {
	  errlogPrintf("SM300: Driver is only setup for two axes X and Y!\n");
  }
  pAxis = new SM300Axis(this, 0, 'X');  
  pAxis = new SM300Axis(this, 1, 'Y');

  
  createParam(SM300ResetString, asynParamInt32, &reset_);
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Send a querry string to the controller and get a return value.
  * Querry string is prefeced with ACK STX and postfixed with EOT
  * return string ends with ETX
  * \param[in] querry the querry to send
  * \param[in] what form the querry reply has; true if the reply has EOT false for ETX with BCC
  * \returns success of write and read of querry string
  */
asynStatus SM300Controller::sendQuerry(const char * querry, bool hasEotEnding) {
	if (hasEotEnding) {
		setTerminationChars("\x04", 1, "\x04", 1);
	} 
	else {
		setTerminationChars("\x03", 1, "\x04", 1);
	}
	//send data format 2
	sprintf(this->outString_, "\x06\x02%s", querry);
	return this->writeReadController();
}

/** Send a command string to the controller.
* Command string is prefeced with ACK STX and postfixed with EOT
* \param[in] command the command to send
* \returns success of write and read of acknowledgement from controller
*/
asynStatus SM300Controller::sendCommand(const char * command) {
	setTerminationChars("\x06", 1, "\x04", 1);
	//send data format 2
	sprintf(this->outString_, "\x06\x02%s", command);
	return this->writeReadController();
}

/** 
  * Return true if the controller registering moving motors
  */
bool SM300Controller::is_moving() {
	return is_moving_;
}

/**
  * deal with db records being set which are integers
  * return status
  */
asynStatus SM300Controller::writeInt32(asynUser *pasynUser, epicsInt32 value) {
	asynStatus status;

	int function = pasynUser->reason;		//Function requested
	if (function == reset_) {
		if (value == 0) return asynSuccess;

		setTerminationChars("\x04", 1, "\x04", 1);

		//send empty string with ACK to clear the buffer		
		status = this->writeController();

		//set termination character is EOT without check sum *
		// when sedning send <CR> incase this mode is switched on
		setTerminationChars("\x06", 1, "\x04\x0D", 2);
		if (sendCommand("PEK0")) return status; 

		
		setTerminationChars("\x06", 1, "\x04", 1);
		// set achknowlegement on
		if (sendCommand("PEL1")) return status;
		// set linear interpolation
		if (sendCommand("B/ G01")) return status;
		// absoulute coordinates
		if (sendCommand("B/ G90")) return status;
		// switch off message from contrl unit when motor is in position or in error (will poll instead)
		if (sendCommand("PER0")) return status;

		//send data format 2 - 100s
		if (sendCommand("PXA2")) return status;
		if (sendCommand("PYA2")) return status;
		// Gear factor numerator
		if (sendCommand("PXB5")) return status;
		if (sendCommand("PYB1")) return status;
		// Gear factor denomenator
		if (sendCommand("PXC10")) return status;
		if (sendCommand("PYC10")) return status;
		// Drag Error
		if (sendCommand("PXD2500")) return status;
		if (sendCommand("PYD2500")) return status;
		// Start/stop ramp
		if (sendCommand("PXE100000")) return status;
		if (sendCommand("PYE25000")) return status;
		// KV factor oe feedback control amplification
		if (sendCommand("PXF1000")) return status;
		if (sendCommand("PYF1000")) return status;
		// Regulator factor A
		if (sendCommand("PXG0")) return status;
		if (sendCommand("PYG0")) return status;
		// +Software switch limit
		if (sendCommand("PXH+57000")) return status;
		if (sendCommand("PYH+64000")) return status;
		// -Software switch limit
		if (sendCommand("PXI-50")) return status;
		if (sendCommand("PYI-20")) return status;
		// maximum speed
		if (sendCommand("PXJ25000")) return status;
		if (sendCommand("PYJ25000")) return status;
		// direction and speed of homing procedure
		if (sendCommand("PXK-2500")) return status;
		if (sendCommand("PYK-7500")) return status;
		// standstill check
		if (sendCommand("PXL100")) return status;
		if (sendCommand("PYL100")) return status;
		// Inposition window
		if (sendCommand("PXM5")) return status;
		if (sendCommand("PYM5")) return status;
		// Distance from reference switch
		if (sendCommand("PXN1000")) return status;
		if (sendCommand("PYN5000")) return status;
		// Backlash compensation
		if (sendCommand("PXO0")) return status;
		if (sendCommand("PYO0")) return status;
		// Moving direction
		if (sendCommand("PXP0")) return status;
		if (sendCommand("PYP0")) return status;
		// Feed for axes
		if (sendCommand("BF15000")) return status;
		setIntegerParam(reset_, 0);
		callParamCallbacks();
	}
	else {
		status = asynMotorController::writeInt32(pasynUser, value);
	}
	return status;
}


/** Creates a new SM300Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SM300PortName       The name of the drvAsynIPPPort that was created previously to connect to the SM300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  * \param[in] eguPerStep        The stage resolution
  */
extern "C" int SM300CreateController(const char *portName, const char *SM300PortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod, const char *eguPerStep)
{
  double stepSize;
   
  stepSize = strtod(eguPerStep, NULL);
  new SM300Controller(portName, SM300PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., stepSize);
  //printf("\n *** SM300: stepSize=%f\n", stepSize);
  if (errno != 0) {
    printf("SM300: Error invalid steps per unit=%s\n", eguPerStep);
    return asynError;
  }
  
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SM300Controller::report(FILE *fp, int level)
{
  fprintf(fp, "SM300 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SM300Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SM300Axis* SM300Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<SM300Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SM300Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] No Axis index number. */
SM300Axis* SM300Controller::getAxis(int axisNo)
{
  return static_cast<SM300Axis*>(asynMotorController::getAxis(axisNo));
}

/** Set the termination characters on the output and input buffers
 * \param[in] eosIn end of string for input
 * \param[in] eosInlen length of end of string for input
 * \param[in] eosOut end of string for output
 * \param[in] eosOutlen length of end of string for output
 */
void SM300Controller::setTerminationChars(const char *eosIn, int eosInlen, const char *eosOut, int eosOutlen) {

	pasynOctetSyncIO->setOutputEos(this->pasynUserController_, eosOut, eosOutlen);
	pasynOctetSyncIO->setInputEos(this->pasynUserController_, eosIn, eosInlen);
}

/** Polls the axis.
 * This function reads moving status which is done on a controller
 */
asynStatus SM300Controller::poll()
{

	asynStatus comStatus;
	SM300Axis *axis;

	// Read the current status of the motor (at Poition, Not at position, Error)
	comStatus = this->sendQuerry("LM", true);
	if (comStatus) goto skip;

	if (strlen(this->inString_) < 3) {
		comStatus = asynError;
		errlogPrintf("SM300 poll: moving status return string is too short.\n");
		goto skip;
	}

	if (this->inString_[2] == 'P') {
		is_moving_ = false;
	}
	else if (this->inString_[2] == 'N') {
		is_moving_ = true;
	}
	else {
		is_moving_ = false;
		comStatus = asynError;
		errlogPrintf("SM300 poll: moving status returned error status.\n");
		goto skip;
	}
	for (int i=0; i < this->numAxes_; i++) {
		axis = this->getAxis(i);
		axis->setIntegerParam(motorStatusDone_, is_moving_ ? 0 : 1);
	}

skip:
	has_error_ = comStatus != asynSuccess;

	for (int i=0; i < this->numAxes_; i++) {
		axis = this->getAxis(i);
		axis->setIntegerParam(this->motorStatusProblem_, axis->has_error() || has_error_ ? 1 : 0);
		axis->callParamCallbacks();
	}
	callParamCallbacks();
	return comStatus ? asynError : asynSuccess;
}

/** Get an whether the controller has an error.
*/
bool SM300Controller::has_error() {
	return has_error_;
}


// These are the SM300Axis methods

/** Creates a new SM300Axis object.
  * \param[in] pC Pointer to the SM300Controller to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
SM300Axis::SM300Axis(SM300Controller *pC, int axisNo, char axisLabel)
	: asynMotorAxis(pC, axisNo),
	pC_(pC), axisLabel(axisLabel)
{
}

/** Get an whether the axis has an error.
*/
bool SM300Axis::has_error() {
	return has_error_;
}

/** Polls the axis.
* This function reads the position of the axis 
* \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false).
* \returns success of poll
*/
asynStatus SM300Axis::poll(bool *moving)
{
	char temp[MAX_CONTROLLER_STRING_SIZE];
	double position;
	asynStatus comStatus;

	*moving = pC_->is_moving();

	// Read the current motor position	
	sprintf(temp, "LI%c", this->axisLabel);
	comStatus = pC_->sendQuerry(temp, false);
	if (comStatus) goto skip;

	// The response string is of the form "\06\02%d"
	if (strlen(pC_->inString_) < 3) {
		comStatus = asynError;
		errlogPrintf("SM300 poll: position return string is too short.\n");
		goto skip;
	}

	char *stop_char;
	position = (strtod(&pC_->inString_[2], &stop_char) / 1.0);
	if (stop_char[0] != '\0') {
		errlogPrintf("SM300 poll: LI reply contains non number %s.\n", &pC_->inString_[2]);
		comStatus = asynError;
		goto skip;
	}
	setDoubleParam(pC_->motorPosition_, position);
	
skip:
	has_error_ = comStatus != asynSuccess;
	setIntegerParam(pC_->motorStatusProblem_, has_error_ || pC_->has_error() ? 1 : 0);
	callParamCallbacks();
	return comStatus ? asynError : asynSuccess;
}


/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void SM300Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_ + 1);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


/** Move the motor to an absolute location or by a relative amount (uses current location since the motor doesw not support a relative command).
* \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move
* by (if relative=1). Units=steps.
* \param[in] relative  Flag indicating relative move (1) or absolute move (0).
* \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
* \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
* \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus SM300Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus comStatus;
  char temp[MAX_CONTROLLER_STRING_SIZE];
  double move_to = position;

  //status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  if (relative==1) { //relative move
	  errlogPrintf("SM300 move: Can not do relative move with this motor.\n");
	  return asynError;
  } 

  // perform a stop so new positions can be set
  comStatus = pC_->sendQuerry("BSS", false);
  if (comStatus) goto skip;

  sprintf(temp, "B%c%.0f", axisLabel, round(move_to));
  comStatus = pC_->sendCommand(temp);
  
  if (comStatus) goto skip;
  comStatus = pC_->sendCommand("BSL");

skip:
  return comStatus;

}

asynStatus SM300Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  char temp[MAX_CONTROLLER_STRING_SIZE];
  sprintf(temp, "BR%c", axisLabel);
  return pC_->sendCommand(temp);
}

// Jog
asynStatus SM300Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
	errlogPrintf("Device does not support move velocity (jog)");
	return asynError;

}

/** Stop of motor axis moving (Stops all axes)
 * \returns send command status
*/
asynStatus SM300Axis::stop(double acceleration )
{
  asynStatus status;

  status = pC_->sendQuerry("BSS", false); 
  // ignore reply it has to be <STX>P<EOT>
  return status ? asynError : asynSuccess;
}

/** Set absolute position in hardware
  * Not supported
  */
asynStatus SM300Axis::setPosition(double position)
{
	errlogPrintf("Device does not support set position");
	return asynError;
}

/** Code for iocsh registration */
static const iocshArg SM300CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SM300CreateControllerArg1 = {"SM300 port name", iocshArgString};
static const iocshArg SM300CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SM300CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SM300CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg SM300CreateControllerArg5 = {"EGUs per step", iocshArgString};
static const iocshArg * const SM300CreateControllerArgs[] = {&SM300CreateControllerArg0,
                                                             &SM300CreateControllerArg1,
                                                             &SM300CreateControllerArg2,
                                                             &SM300CreateControllerArg3,
                                                             &SM300CreateControllerArg4,
															 &SM300CreateControllerArg5};
static const iocshFuncDef SM300CreateControllerDef = {"SM300CreateController", 6, SM300CreateControllerArgs};
static void SM300CreateContollerCallFunc(const iocshArgBuf *args)
{
  SM300CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval);
}

static void SM300Register(void)
{
  iocshRegister(&SM300CreateControllerDef, SM300CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(SM300Register);
}
