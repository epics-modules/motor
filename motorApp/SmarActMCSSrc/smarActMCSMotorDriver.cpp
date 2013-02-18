
/* Motor driver support for smarAct MCS RS-232 Controller   */

/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 9/11 */

#include <iocsh.h>

#include <asynOctetSyncIO.h>
#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <smarActMCSMotorDriver.h>
#include <errlog.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <exception>

#include <math.h>

#include <epicsString.h>
#include <epicsExport.h>

/* Static configuration parameters (compile-time constants) */
#undef  DEBUG

#define CMD_LEN 50
#define REP_LEN 50
#define DEFLT_TIMEOUT 2.0

#define HOLD_FOREVER 60000
#define HOLD_NEVER       0
#define FAR_AWAY     1000000000 /*nm*/

// Windows does not have rint()
#ifdef _WIN32
double rint(double x)
{
  //middle value point test
  if (ceil(x+0.5) == floor(x+0.5))
  {
    int a = (int)ceil(x);
    if (a%2 == 0) return ceil(x);
    else return floor(x);
  }
  else return floor(x+0.5);
}
#endif
  

/* The asyn motor driver apparently can't cope with exceptions */
#undef  ASYN_CANDO_EXCEPTIONS
/* Define this if exceptions should be thrown and it is OK to abort the application */
#undef  DO_THROW_EXCEPTIONS

#if defined(ASYN_CANDO_EXCEPTIONS) || defined(DO_THROW_EXCEPTIONS)
#define THROW_(e) throw e
#else
#define THROW_(e) epicsPrintf("%s\n",e.what());
#endif

enum SmarActMCSStatus {
	Stopped     = 0,
	Stepping    = 1,
	Scanning    = 2,
	Holding     = 3,
	Targeting   = 4,
	MoveDelay   = 5,
	Calibrating = 6,
	FindRefMark = 7,
	Locked      = 9
};

SmarActMCSException::SmarActMCSException(SmarActMCSExceptionType t, const char *fmt, ...)
	: t_(t)
{
va_list ap;
	if ( fmt ) {
		va_start(ap, fmt);
		epicsVsnprintf(str_, sizeof(str_), fmt, ap);
		va_end(ap);
	} else {
		str_[0] = 0;
	}
};

SmarActMCSException::SmarActMCSException(SmarActMCSExceptionType t, const char *fmt, va_list ap)
		: t_(t)
{
	epicsVsnprintf(str_, sizeof(str_), fmt, ap);
}

/* Parse reply from MCS and return the value converted to a number.
 *
 * If the string cannot be parsed, i.e., is not in the format
 *  ':' , <string_of_upper_case_letters> , <number1> , ',' , <number2>
 * 
 * then the routine returns '-1'.
 *
 * Otherwise, if <string_of_upper_case_letters> starts with 'E'
 * (which means an 'Error' code) then the (always non-negative)
 * error code is returned (may be zero in case of an 'acknowledgement'
 * message in synchronous mode).
 *
 * If the string is parsed successfully then <number2> is passed up
 * in *val_p.
 *
 * Hence - return code nonzero -> error (error code if > 0, parse error
 *                                otherwise)
 *       - return code zero    -> successful ACK or command reply; value
 *                                in *val_p.
 */
int
SmarActMCSController::parseReply(const char *reply, int *ax_p, int *val_p)
{
char cmd[10];
	if ( 3 != sscanf(reply, ":%10[A-Z]%i,%i", cmd, ax_p, val_p) )
		return -1;
	return 'E' == cmd[0] ? *val_p : 0;
}

SmarActMCSController::SmarActMCSController(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
	: asynMotorController(portName, numAxes,
	                      0, // parameters
	                      0, // interface mask
	                      0, // interrupt mask
	                      ASYN_CANBLOCK | ASYN_MULTIDEVICE,
	                      1, // autoconnect
	                      0,0) // default priority and stack size
	, asynUserMot_p_(0)
{
asynStatus       status;
char             junk[100];
size_t           got_junk;
int              eomReason;
pAxes_ = (SmarActMCSAxis **)(asynMotorController::pAxes_);

	status = pasynOctetSyncIO->connect(IOPortName, 0, &asynUserMot_p_, NULL);
	if ( status ) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
		          "SmarActMCSController:SmarActMCSController: cannot connect to MCS controller\n");
		THROW_(SmarActMCSException(MCSConnectionError, "SmarActMCSController: unable to connect serial channel"));
	}

	// slurp away any initial telnet negotiation; there is no guarantee that
	// the other end will not send some telnet chars in the future. The terminal
	// server should really be configured to 'raw' mode!
	pasynOctetSyncIO->read(asynUserMot_p_, junk, sizeof(junk), 2.0, &got_junk, &eomReason);
	if ( got_junk ) {
		epicsPrintf("SmarActMCSController(%s): WARNING - detected unexpected characters on link (%s); make sure you have a RAW (not TELNET) connection\n", portName, IOPortName);
	}

	pasynOctetSyncIO->setInputEos ( asynUserMot_p_, "\n", 1 );
	pasynOctetSyncIO->setOutputEos( asynUserMot_p_, "\n", 1 );

	// Create axes
/*	for ( ax=0; ax<numAxes; ax++ ) {
		//axis_p = new SmarActMCSAxis(this, ax);
		pAxes_[ax] = new SmarActMCSAxis(this, ax);
	}
*/
	// move to iocsh function smarActMCSCreateAxis()

	// FIXME the 'forcedFastPolls' may need to be set if the 'sleep/wakeup' feature
	//       of the sensor/readback is used.
	startPoller( movingPollPeriod, idlePollPeriod, 0 );

}

asynStatus
SmarActMCSController::sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, va_list ap)
{
char       buf[CMD_LEN];
size_t     nwrite;
int        eomReason;
asynStatus status;

	epicsVsnprintf(buf, sizeof(buf), fmt, ap);

	status = pasynOctetSyncIO->writeRead( asynUserMot_p_, buf, strlen(buf), rep, len, timeout, &nwrite, got_p, &eomReason);

	//asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "sendCmd()=%s", buf);

	return status;
}

asynStatus
SmarActMCSController::sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, ...)
{
va_list    ap;
asynStatus status;
	va_start(ap, fmt);
	status = sendCmd(got_p, rep, len, timeout, fmt, ap);
	va_end(ap);
	return status;
}


asynStatus SmarActMCSController::sendCmd(size_t *got_p, char *rep, int len, const char *fmt, ...)
{
va_list    ap;
asynStatus status;
	va_start(ap, fmt);
	status = sendCmd(got_p, rep, len, DEFLT_TIMEOUT, fmt, ap);
	va_end(ap);
	return status;
}

asynStatus SmarActMCSController::sendCmd(char *rep, int len, const char *fmt, ...)
{
va_list    ap;
asynStatus status;
size_t     got;
	va_start(ap, fmt);
	status = sendCmd(&got, rep, len, DEFLT_TIMEOUT, fmt, ap);
	va_end(ap);
	return status;
}

/* Obtain value of the 'motorClosedLoop_' parameter (which
 * maps to the record's CNEN field)
 */
int
SmarActMCSAxis::getClosedLoop()
{
int val;
	c_p_->getIntegerParam(axisNo_, c_p_->motorClosedLoop_, &val);
	return val;
}

SmarActMCSAxis::SmarActMCSAxis(class SmarActMCSController *cnt_p, int axis, int channel)
	: asynMotorAxis(cnt_p, axis), c_p_(cnt_p)
{
	int val;
	channel_ = channel;

	asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "SmarActMCSAxis::SmarActMCSAxis -- creating axis %u\n", axis);

	comStatus_ = getVal("GCLS",&vel_);
#ifdef DEBUG
	printf("GCLS %u returned %i\n", axis, comStatus_);
#endif
	if ( comStatus_ )
		goto bail;
	if ( (comStatus_ = getVal("GS", &val)) )
		goto bail;

	if ( Holding == val ) {
		// still holding? This means that - in a previous life - the
		// axis was configured for 'infinite holding'. Inherit this
		// (until the next 'move' command that is).
		///
		holdTime_ = HOLD_FOREVER;
	} else {
		// initial value from 'closed-loop' property
		holdTime_ = getClosedLoop() ? HOLD_FOREVER : 0;
	}

	if ( asynSuccess == getVal("GP",&val) ) {
		setIntegerParam(c_p_->motorStatusHasEncoder_, 1);
		setIntegerParam(c_p_->motorStatusGainSupport_, 1);
	}

bail:
	setIntegerParam(c_p_->motorStatusProblem_, comStatus_ ? 1 : 0 );
	setIntegerParam(c_p_->motorStatusCommsError_, comStatus_ ? 1 : 0 );

	callParamCallbacks();

	if ( comStatus_ ) {
		THROW_(SmarActMCSException(MCSCommunicationError, "SmarActMCSAxis::SmarActMCSAxis -- channel %u ASYN error %i", axis, comStatus_));
	}

}

/* Read a parameter from the MCS (nothing to do with asyn's parameter
 * library).
 *
 * parm_cmd: MCS command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the MCS
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus
SmarActMCSAxis::getVal(const char *parm_cmd, int *val_p)
{
char       rep[REP_LEN];
asynStatus st;
int        ax;

	//asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "getVal() cmd=:%s%u", parm_cmd, this->channel_);

	//st = c_p_->sendCmd(rep, sizeof(rep), ":%s%u", parm_cmd, this->axisNo_);
	st = c_p_->sendCmd(rep, sizeof(rep), ":%s%u", parm_cmd, this->channel_);
	if ( st )
		return st;
	return c_p_->parseReply(rep, &ax, val_p) ? asynError: asynSuccess;
}

asynStatus
SmarActMCSAxis::poll(bool *moving_p)
{
int                    val;
enum SmarActMCSStatus status;

	if ( (comStatus_ = getVal("GP", &val)) )
		goto bail;

	setDoubleParam(c_p_->motorEncoderPosition_, (double)val);
	setDoubleParam(c_p_->motorPosition_, (double)val);
#ifdef DEBUG
	printf("POLL (position %d)", val);
#endif

	if ( (comStatus_ = getVal("GS", &val)) )
		goto bail;

	status = (enum SmarActMCSStatus)val;

	switch ( status ) {
		default:
			*moving_p = false;
		break;

		/* If we use 'infinite' holding (until the next 'move' command)
		 * then the 'Holding' state must be considered 'not moving'. However,
		 * if we use a 'finite' holding time then we probably should consider
		 * the 'move' command incomplete until the holding time expires.
		 */
		case Holding:
			*moving_p = HOLD_FOREVER == holdTime_ ? false : true;
		break;

		case Stepping:
		case Scanning:
		case Targeting:
		case MoveDelay:
		case Calibrating:
		case FindRefMark:
			*moving_p = true;
		break;
	}

	setIntegerParam(c_p_->motorStatusDone_, ! *moving_p );


	/* Check if the sensor 'knows' absolute position and
	 * update the MSTA 'HOMED' bit.
	 */
	if ( (comStatus_ = getVal("GPPK", &val)) ) 
		goto bail;

	setIntegerParam(c_p_->motorStatusHomed_, val ? 1 : 0 );

#ifdef DEBUG
	printf(" status %u", status);
#endif

bail:
	setIntegerParam(c_p_->motorStatusProblem_,    comStatus_ ? 1 : 0 );
	setIntegerParam(c_p_->motorStatusCommsError_, comStatus_ ? 1 : 0 );
#ifdef DEBUG
	printf("\n");
#endif

	callParamCallbacks();

	return comStatus_;
}

asynStatus
SmarActMCSAxis::moveCmd(const char *fmt, ...)
{
int     val, ax;
char    rep[REP_LEN];
size_t  got;
double  tout = DEFLT_TIMEOUT;
va_list ap;

	va_start(ap, fmt);
	comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, fmt, ap);
	va_end(ap);

	if ( comStatus_ )
		goto bail;

	if ( c_p_->parseReply(rep, &ax, &val) ) {
		comStatus_ = asynError;
		goto bail;
	}

bail:
#ifdef DEBUG
	printf("\n");
#endif

	return comStatus_;
}

asynStatus
SmarActMCSAxis::setSpeed(double velocity)
{
long       vel;
asynStatus status;

	if ( (vel = (long)rint(fabs(velocity))) != vel_ ) {
		/* change speed */
		if ( asynSuccess == (status = moveCmd(":SCLS%u,%ld", channel_, vel)) ) {
			vel_ = vel;
		}
		return status;
	}
	return asynSuccess;
}

asynStatus  
SmarActMCSAxis::move(double position, int relative, double min_vel, double max_vel, double accel)
{
const char *fmt = relative ? ":MPR%u,%ld,%d" : ":MPA%u,%ld,%d";

#ifdef DEBUG
	printf("Move to %f (speed %f - %f)\n", position, min_vel, max_vel);
#endif

	if ( (comStatus_ = setSpeed(max_vel)) )
		goto bail;

	
	/* cache 'closed-loop' setting until next move */
	holdTime_  = getClosedLoop() ? HOLD_FOREVER : 0;

	comStatus_ = moveCmd(fmt, channel_, (long)rint(position), holdTime_);

bail:
	if ( comStatus_ ) {
		setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
	}
	return comStatus_;
}

asynStatus
SmarActMCSAxis::home(double min_vel, double max_vel, double accel, int forwards)
{

#ifdef DEBUG
	printf("Home %u\n", forwards);
#endif

	if ( (comStatus_ = setSpeed(max_vel)) )
		goto bail;

	/* cache 'closed-loop' setting until next move */
	holdTime_  = getClosedLoop() ? HOLD_FOREVER : 0;

	comStatus_ = moveCmd(":FRM%u,%u,%d,0", channel_, forwards ? 0 : 1, holdTime_);

bail:
	if ( comStatus_ ) {
		setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
	}
	return comStatus_;
}

asynStatus
SmarActMCSAxis::stop(double acceleration)
{
#ifdef DEBUG
	printf("Stop\n");
#endif
	comStatus_ = moveCmd(":S%u", channel_);

	if ( comStatus_ ) {
		setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
	}
	return comStatus_;
}

asynStatus
SmarActMCSAxis::setPosition(double position)
{
	comStatus_ = moveCmd(":SP%u,%d", channel_, (long)rint(position));
	if ( comStatus_ ) {
		setIntegerParam(c_p_->motorStatusProblem_,    1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
	}
	return comStatus_;
}

asynStatus
SmarActMCSAxis::moveVelocity(double min_vel, double max_vel, double accel)
{
long       speed   = (long)rint(fabs(max_vel));
long       tgt_pos = FAR_AWAY;

	/* No MCS command we an use directly. Just use a 'relative move' to
	 * very far target.
	 */

#ifdef DEBUG
	printf("moveVelocity (%f - %f)\n", min_vel, max_vel);
#endif

	if ( 0 == speed ) {
		/* Here we are in a dilemma. If we set the MCS' speed to zero
		 * then it will move at unlimited speed which is so fast that
		 * 'JOG' makes no sense.
		 * Just 'STOP' the motion - hope that works...
		 */
		setIntegerParam(c_p_->motorStop_, 1);
		callParamCallbacks();
		return asynSuccess;
	}

	if ( max_vel < 0 ) {
		tgt_pos = -tgt_pos; 
	}

	if ( (comStatus_ = setSpeed(max_vel)) )
		goto bail;

	comStatus_ = moveCmd(":MPR%u,%ld,0", channel_, tgt_pos);

bail:
	if ( comStatus_ ) {
		setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
	}
	return comStatus_;
}

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
static const iocshArg cc_a0 = {"Port name [string]",               iocshArgString};
static const iocshArg cc_a1 = {"I/O port name [string]",           iocshArgString};
static const iocshArg cc_a2 = {"Number of axes [int]",             iocshArgInt};
static const iocshArg cc_a3 = {"Moving poll period (s) [double]",  iocshArgDouble};
static const iocshArg cc_a4 = {"Idle poll period (s) [double]",    iocshArgDouble};

static const iocshArg * const cc_as[] = {&cc_a0, &cc_a1, &cc_a2, &cc_a3, &cc_a4};

static const iocshFuncDef cc_def = {"smarActMCSCreateController", sizeof(cc_as)/sizeof(cc_as[0]), cc_as};

extern "C" void *
smarActMCSCreateController(
	const char *motorPortName,
	const char *ioPortName,
	int         numAxes,
	double      movingPollPeriod,
	double      idlePollPeriod)
{
void *rval = 0;
	// the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
	// if constructing a controller (or axis) incurs an exception even if its
	// caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
	try {
#endif
		rval = new SmarActMCSController(motorPortName, ioPortName, numAxes, movingPollPeriod, idlePollPeriod);
#ifdef ASYN_CANDO_EXCEPTIONS
	} catch (SmarActMCSException &e) {
		epicsPrintf("smarActMCSCreateController failed (exception caught):\n%s\n", e.what());
		rval = 0;
	}
#endif

	return rval;
}

static void cc_fn(const iocshArgBuf *args)
{
	smarActMCSCreateController(
		args[0].sval,
		args[1].sval,
		args[2].ival,
		args[3].dval,
		args[4].dval);
}


static const iocshArg ca_a0 = {"Controller Port name [string]",    iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]",                iocshArgInt};
static const iocshArg ca_a2 = {"Channel [int]",                    iocshArgInt};

static const iocshArg * const ca_as[] = {&ca_a0, &ca_a1, &ca_a2};

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
/* smarActMCSCreateAxis called to create each axis of the smarActMCS controller*/
static const iocshFuncDef ca_def = {"smarActMCSCreateAxis", 3, ca_as};

extern "C" void *
smarActMCSCreateAxis(
	const char *controllerPortName,
	int        axisNumber,
	int        channel)
{
void *rval = 0;

SmarActMCSController *pC;
SmarActMCSAxis *pAxis;
asynMotorAxis *pAsynAxis;

	// the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
	// if constructing a controller (or axis) incurs an exception even if its
	// caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
	try {
#endif
//		rval = new SmarActMCSAxis(, axisNumber, channel);
		pC = (SmarActMCSController*) findAsynPortDriver(controllerPortName);
		if (!pC) {
			printf("smarActMCSCreateAxis: Error port %s not found\n", controllerPortName);
			rval = 0;
			return rval;
		}
		// check if axis number already exists
		pAsynAxis = pC->getAxis(axisNumber);
		if (pAsynAxis != NULL) { // axis already exists
			epicsPrintf("SmarActMCSCreateAxis failed:axis %u already exists\n", axisNumber);
#ifdef ASYN_CANDO_EXCEPTIONS
			THROW_(SmarActMCSException(MCSCommunicationError, "axis %u already exists", axisNumber));
#endif
			rval = 0;
			return rval;
		}
		pC->lock();
		pAxis = new SmarActMCSAxis(pC, axisNumber, channel);
		pAxis = NULL;
		pC->unlock();

#ifdef ASYN_CANDO_EXCEPTIONS
	} catch (SmarActMCSException &e) {
		epicsPrintf("SmarActMCSAxis failed (exception caught):\n%s\n", e.what());
		rval = 0;
	}
#endif

	return rval;
}

static void ca_fn(const iocshArgBuf *args)
{
	smarActMCSCreateAxis(
		args[0].sval,
		args[1].ival,
		args[2].ival);
}

static void smarActMCSMotorRegister(void)
{
  iocshRegister(&cc_def, cc_fn);  // smarActMCSCreateController
  iocshRegister(&ca_def, ca_fn);  // smarActMCSCreateAxis
}

extern "C" {
epicsExportRegistrar(smarActMCSMotorRegister);
}
