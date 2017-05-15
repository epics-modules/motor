#ifndef MOTOR_H
#define MOTOR_H

#define MAX_AXES 8
#define AXIS_CHECK_RETURN(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=MAX_AXES)) return;}
#define AXIS_CHECK_RETURN_ZERO(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=MAX_AXES)) return 0;}
#define AXIS_CHECK_RETURN_ERROR(_axis) {init_axis(_axis); if (((_axis) <= 0) || ((_axis) >=MAX_AXES)) return (-1);}

int getAxisDone(int axis_no);
int getAxisHome(int axis_no);
int getAxisHomed(int axis_no);

static void init_axis(int);
void hw_motor_init(int);
/* Where does the motor wake up after power-on */
void setMotorParkingPosition(int axis_no, double value);
void setMaxHomeVelocityAbs(int axis_no, double value);
void setMotorReverseERES(int axis_no, double value);


double getMotorVelocity(int axis_no);
int isMotorMoving(int axis_no);

double getLowSoftLimitPos(int axis_no);
void   setLowSoftLimitPos(int axis_no, double value);
void   enableLowSoftLimit(int axis_no, int value);
void   setLowHardLimitPos(int axis_no, double value);

double getHighSoftLimitPos(int axis_no);
void   setHighSoftLimitPos(int axis_no, double value);
void   enableHighSoftLimit(int axis_no, int value);
void   setHighHardLimitPos(int axis_no, double value);

double getMotorPos(int axis_no);
double getEncoderPos(int axis_no);

char getpLimits(int axis_no);
void setpLimits(int axis_no, char value);
int getNegLimitSwitch(int axis_no);
int getPosLimitSwitch(int axis_no);
int get_bError(int motor_axis_no);

/*
 * Movements
 */

int movePosition(int axis_no,
                 double position,
                 int relative,
                 double max_velocity,
                 double acceleration);

/*
 *  moveHome
 *  move until the home switch is hit
 *
 *  direction:    either <0 or >=0, from which direction the switch
 *                should be reached (Which means that we may run over
 *                the switch and return from the other side with min_velocity
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveHome(int axis_no,
             int direction,
             double max_velocity,
             double acceleration);

/*
 *  moveAbsolute: Move to absolute position
 *  limit switches shoud be obeyed (I think)
 *
 *  direction:    either <0 or >=0
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveAbsolute(int axis_no,
                 double position,
                 double max_velocity,
                 double acceleration);

/*
 *  moveRelative: Move to absolute position
 *  limit switches shoud be obeyed (I think)
 *
 *  direction:    either <0 or >=0
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveRelative(int axis_no,
                 double offset,
                 double max_velocity,
                 double acceleration);


/*
 *  moveVelocity: Same as JOG
 *  limit switches shoud be obeyed (I think)
 *
 *  axis_no       1..max
 *  direction:    either <0 or >=0
 *  max_velocity: >0 velocity after acceleration has been done
 *  acceleration: time in seconds to reach max_velocity
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int moveVelocity(int axis_no,
                 int direction,
                 double max_velocity,
                 double acceleration);

/*
 *  stop
 *  axis_no       1..max
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int motorStop(int axis_no);

/*
 *  amplifier power in percent 0..100
 *  axis_no       1..max
 *
 *  return value: 0 == OK,
 *                error codes and error handling needs to be defined
 */
int setAmplifierPercent(int axis_no, int percent);

#endif /* MOTOR_H */
