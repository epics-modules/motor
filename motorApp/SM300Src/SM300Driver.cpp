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

/** Creates a new SM300Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SM300PortName     The name of the drvAsynSerialPort that was created previously to connect to the SM300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SM300Controller::SM300Controller(const char *portName, const char *SM300PortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
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
  createParam(SM300ResetAndHomeString, asynParamInt32, &reset_and_home_);
  createParam(SM300DisconnectString, asynParamInt32, &disconnect_);  
  createParam(SM300ErrorCodeString, asynParamInt32, &error_code_);  

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Send a query string to the controller and get a return value.
  * query string is prefeced with ACK STX and postfixed with EOT
  * return string ends with ETX
  * \param[in] query the query to send
  * \param[in] what form the query reply has; true if the reply has EOT false for ETX with BCC
  * \returns success of write and read of query string
  */
asynStatus SM300Controller::sendQuery(const char * query, bool hasEotEnding) {
	if (hasEotEnding) {
		setTerminationChars("\x04", 1, "\x04", 1);
	} 
	else {
		setTerminationChars("\x03", 1, "\x04", 1);
	}
	//send data format 2
	sprintf(this->outString_, "\x06\x02%s", query);
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
  * \returns status
  */
asynStatus SM300Controller::writeInt32(asynUser *pasynUser, epicsInt32 value) {
	asynStatus status;

	int function = pasynUser->reason;		//Function requested
	if (function == reset_) {
		if (value == 0) return asynSuccess;
		status = perform_reset();
		callParamCallbacks();
	}
	else if (function == disconnect_) {
		setIntegerParam(disconnect_, 1);
		setTerminationChars("\x06", 1, "\x04", 1);
		sprintf(this->outString_, "\x06\x02%s", "M77");
		status = writeController();
		setIntegerParam(disconnect_, 0);
		callParamCallbacks();
	}
	else if (function == reset_and_home_) {
		setIntegerParam(reset_and_home_, 1);
		callParamCallbacks();
		perform_reset();
		for (int i = 0; i < numAxes_; i++) {
			getAxis(i)->home(0, 0, 0, 0);
		}
		setIntegerParam(reset_and_home_, 0);
		callParamCallbacks();
	}
	else {
		status = asynMotorController::writeInt32(pasynUser, value);
	}
	return status;
}

/**
 * Perform a reset of the controller sending it initiation parameters
 * \returns status
*/
asynStatus SM300Controller::perform_reset() {
	setIntegerParam(reset_, 1);
	callParamCallbacks();
	setTerminationChars("\x04", 1, "\x04", 1);

	//send empty string with ACK to clear the buffer		
	asynStatus status = this->writeController();

	//set termination character is EOT without check sum *
	// when sending send <CR> incase this mode is switched on
	setTerminationChars("\x06", 1, "\x04\x0D", 2);
	if (sendCommand("PEK0")) return status;

	setTerminationChars("\x06", 1, "\x04", 1);
	const char * commands[] = {
		// set achknowlegement on
		"PEL1",  
		// set linear interpolation
		"B/ G01", 
		// absoulute coordinates
	    "B/ G90", 
		// switch off message from contrl unit when motor is in position or in error (will poll instead)
		"PER0", 
		//send data format 2 - 100s
		"PXA2",
		"PYA2",
		// Gear factor numerator
	    "PXB5",
	    "PYB1",
	    // Gear factor denomenator
	    "PXC10",
	    "PYC10",
	    // Drag Error
	    "PXD2500",
	    "PYD2500",
	    // Start/stop ramp
	    "PXE100000",
	    "PYE25000",
	    // KV factor oe feedback control amplification
	    "PXF1000",
	    "PYF1000",
	    // Regulator factor A
	    "PXG0",
	    "PYG0",
	    // +Software switch limit
	    "PXH+57000",
	    "PYH+64000",
	    // -Software switch limit
	    "PXI-50",
	    "PYI-20",
	    // maximum speed
	    "PXJ25000",
	    "PYJ25000",
	    // direction and speed of homing procedure
	    "PXK-2500",
	    "PYK-7500",
	    // standstill check
	    "PXL100",
	    "PYL100",
	    // Inposition window
	    "PXM5",
	    "PYM5",
	    // Distance from reference switch
	    "PXN1000",
	    "PYN5000",
	    // Backlash compensation
	    "PXO0",
	    "PYO0",
	    // Moving direction
	    "PXP0",
	    "PYP0",
	    // Feed for axes
	    "BF15000"
	};

	int n_array = (sizeof(commands) / sizeof(const char *));
	for (int i = 0; i < n_array; i++) {
		if (sendCommand(commands[i])) return status;
	}

	setIntegerParam(reset_, 0);
	return asynSuccess;
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
                                   int movingPollPeriod, int idlePollPeriod)
{
  new SM300Controller(portName, SM300PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  //printf("\n *** SM300: stepSize=%f\n", stepSize);
  
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
	bool motorInError = false;

	// Read the current status of the motor (at Poition, Not at position, Error)
	comStatus = this->sendQuery("LM", true);
	if (comStatus) goto skip;

	if (strlen(this->inString_) < 3) {
		comStatus = asynError;
		errlogPrintf("SM300 poll: moving status return string is too short.\n");
		goto skip;
	}

	if (this->inString_[2] == 'P') {       // axis in Position
		is_moving_ = false;

		// Finished a home
		if (axis_x_homing_) {
			axis_x_homing_ = false;
			home_axis_x_ = false;
		}
		if (axis_y_homing_) {
			axis_y_homing_ = false;
			home_axis_y_ = false;
		}
		// If need to home to now
		if (home_axis_x_) {
			comStatus = sendCommand("BRX");
			axis_x_homing_ = true;
			if (comStatus) {
				errlogPrintf("SM300 poll: Failed to home x.\n");
			}
		} else if (home_axis_y_) {
			comStatus = sendCommand("BRY");
			axis_y_homing_ = true;
			if (comStatus) {
				errlogPrintf("SM300 poll: Failed to home y.\n");
			}
		}
	}
	else if (this->inString_[2] == 'N') {  // Not in position
		is_moving_ = true;
	}
	else {
		motorInError = true;
		is_moving_ = false;
		comStatus = asynError;
		errlogPrintf("SM300 poll: moving status returned errors status.\n");
	}
	for (int i=0; i < this->numAxes_; i++) {
		axis = this->getAxis(i);
		axis->setIntegerParam(motorStatusDone_, is_moving_ ? 0 : 1);
	}
	
	comStatus = sendQuery("LS10", false);
	if (comStatus) goto skip;

	if (strlen(this->inString_) < 3) {
		comStatus = asynError;
		errlogPrintf("SM300 error code: return string is too short.\n");
		goto skip;
	}
	int code = strtol(&this->inString_[2], NULL, 16);

	if (code != 0) {
		motorInError = true;
	}

    // special case for error in sending command
	if (code >= 0xF && code <= 0x14) {
		errlogPrintf("SM300 error code: CNC cmd error code, code = %.2x\n", code);
		code = 0xF;		
	}
	// speacial case for error in CNC interpreter
	if (code >= 0x1E && code <= 0x4F) {
		errlogPrintf("SM300 error code: CNC cmd error, code = %.2x\n", code);
		code = 0xE;		
	}
	
	setIntegerParam(error_code_, code);

skip:
	has_error_ = motorInError || comStatus != asynSuccess;

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
	comStatus = pC_->sendQuery(temp, false);
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

  sprintf(temp, "B%c%.0f", axisLabel, round(move_to));
  comStatus = pC_->sendCommand(temp);
  
  if (comStatus) goto skip;
  comStatus = pC_->sendCommand("BSL");

skip:
  return comStatus;

}

/** Home an axis
 *
 *  This stores the intention to home when the motor comes to a stop. The reason it is done this way is that if the motors is homing
 *	one axis it can not home any other axis. It wil home the next time the motor is stationary.
 * /param[in] axis the axis to home (X or Y)
*/
void SM300Controller::homeAxis(const char axis) {
	if (axis == 'X') {
		if (!axis_x_homing_) {
			home_axis_x_ = true;
		}
	}
	if (axis == 'Y') {
		if (!axis_y_homing_) {
			home_axis_y_ = true;
		}
	}
}

/** Home the axis 
 *  velocities and accelerations are ignored these are set by the controller.
 */
asynStatus SM300Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  pC_->homeAxis(axisLabel);
  return asynSuccess;
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
  // Motor can not be stopped using BSS because if it is then it immediately sets the position to 0 wherever the motor stops.
  return asynSuccess;
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
static const iocshArg * const SM300CreateControllerArgs[] = {&SM300CreateControllerArg0,
                                                             &SM300CreateControllerArg1,
                                                             &SM300CreateControllerArg2,
                                                             &SM300CreateControllerArg3,
                                                             &SM300CreateControllerArg4};
static const iocshFuncDef SM300CreateControllerDef = {"SM300CreateController", 5, SM300CreateControllerArgs};
static void SM300CreateContollerCallFunc(const iocshArgBuf *args)
{
  SM300CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void SM300Register(void)
{
  iocshRegister(&SM300CreateControllerDef, SM300CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(SM300Register);
}
