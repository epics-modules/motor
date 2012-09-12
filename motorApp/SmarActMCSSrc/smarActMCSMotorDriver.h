#ifndef SMARACT_MCS_MOTOR_DRIVER_H
#define SMARACT_MCS_MOTOR_DRIVER_H

/* Motor driver support for smarAct MCS RS-232 Controller   */

/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 9/11 */

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>

enum SmarActMCSExceptionType {
	MCSUnknownError,
	MCSConnectionError,
	MCSCommunicationError,
};

class SmarActMCSException : public std::exception {
public:
	SmarActMCSException(SmarActMCSExceptionType t, const char *fmt, ...);
	SmarActMCSException(SmarActMCSExceptionType t)
		: t_(t)
		{ str_[0] = 0; }
	SmarActMCSException()
		: t_(MCSUnknownError)
		{ str_[0] = 0; }
	SmarActMCSException(SmarActMCSExceptionType t, const char *fmt, va_list ap);
	SmarActMCSExceptionType getType()
		const { return t_; }
	virtual const char *what()
		const throw() {return str_ ;}
protected:
	char str_[100];	
	SmarActMCSExceptionType t_;
};


class SmarActMCSAxis : public asynMotorAxis
{
public:
	SmarActMCSAxis(class SmarActMCSController *cnt_p, int axis, int channel);
	asynStatus  poll(bool *moving_p);
	asynStatus  move(double position, int relative, double min_vel, double max_vel, double accel);
	asynStatus  home(double min_vel, double max_vel, double accel, int forwards);
	asynStatus  stop(double acceleration);
	asynStatus  setPosition(double position);
	asynStatus  moveVelocity(double min_vel, double max_vel, double accel);

	virtual asynStatus getVal(const char *parm, int *val_p);
	virtual asynStatus moveCmd(const char *cmd, ...);
	virtual int        getClosedLoop();

	int         getVel() const { return vel_; }

protected:
	asynStatus  setSpeed(double velocity);

private:
	SmarActMCSController   *c_p_;  // pointer to asynMotorController for this axis
	asynStatus             comStatus_;
	int                    vel_;
	unsigned               holdTime_;
	int                    channel_;

friend class SmarActMCSController;
};

class SmarActMCSController : public asynMotorController
{
public:
	SmarActMCSController(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, va_list ap);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, ...);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, const char *fmt, ...);
	virtual asynStatus sendCmd(char *rep, int len, const char *fmt, ...);

	static int parseReply(const char *reply, int *ax_p, int *val_p);

protected:
	SmarActMCSAxis **pAxes_;

private:
	asynUser *asynUserMot_p_;
friend class SmarActMCSAxis;
};

#endif // _cplusplus
#endif // SMARACT_MCS_MOTOR_DRIVER_H
