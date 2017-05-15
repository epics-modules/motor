/*
FILENAME...   TCPsimMotor.h
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"
/* Copied from motor.h */
#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)       /* Nearest integer. */

// No controller-specific parameters yet
#define NUM_VIRTUAL_MOTOR_PARAMS 0  

#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_WHEN_HOMING  (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

extern "C" {
  int TCPsimMotorCreateAxis(const char *TCPsimMotorName, int axisNo,
			     int axisFlags, const char *axisOptionsStr);
}

typedef struct {
  int          axis_status;
  int          motorPosition;
} st_axis_status_type;

class epicsShareClass TCPsimMotorAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  TCPsimMotorAxis(class TCPsimMotorController *pC, int axisNo,
		   int axisFlags, const char *axisOptionsStr);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);

private:
  TCPsimMotorController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  struct {
    st_axis_status_type lastpoll;
    struct {
      const char *externalEncoderStr;
      int axisFlags;
    } cfg;
  } drvlocal;

  void handleStatusChange(asynStatus status);

  asynStatus writeReadACK(void);
  asynStatus setValueOnAxis(const char* var);
  asynStatus setValueOnAxis(const char* var, const char *value);
  asynStatus setValueOnAxis(const char* var, int value);

  asynStatus getValueFromAxis(const char* var, unsigned, char *value);
  asynStatus getValueFromAxis(const char* var, int *value);

  asynStatus amplifierPercentage(int);
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);

  friend class TCPsimMotorController;
};

class epicsShareClass TCPsimMotorController : public asynMotorController {
public:
  TCPsimMotorController(const char *portName, const char *TCPsimMotorPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  asynStatus writeReadOnErrorDisconnect(void);
  asynStatus writeOnErrorDisconnect(void);

  TCPsimMotorAxis* getAxis(asynUser *pasynUser);
  TCPsimMotorAxis* getAxis(int axisNo);
  protected:
  void handleStatusChange(asynStatus status);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  friend class TCPsimMotorAxis;
};
