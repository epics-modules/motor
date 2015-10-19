#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include "sock-util.h"
#include "logerr_info.h"
#include "cmd_buf.h"
#include "hw_motor.h"
#include "cmd_TCPsim.h"

#define TCPSIM_SEND_NEWLINE  1
#define TCPSIM_SEND_OK       2
#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)       /* Nearest integer. */

typedef struct
{
  int    velocity;
} cmd_Motor_cmd_type;

static cmd_Motor_cmd_type cmd_Motor_cmd[MAX_AXES];

static void init_axis(int axis_no)
{
  static char init_done[MAX_AXES];
  const static double MRES = 0.001;
  double ReverseMRES = (double)1.0/MRES;

  hw_motor_init(axis_no);

  if (axis_no >= MAX_AXES || axis_no < 0) {
    return;
  }
  if (!init_done[axis_no]) {
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d\n",
            __FILE__, __FUNCTION__, __LINE__, axis_no);
    hw_motor_init(axis_no);
    setMotorParkingPosition(axis_no, 0); /* steps */
    setMaxHomeVelocityAbs(axis_no, 5 * ReverseMRES);
    setLowHardLimitPos(axis_no,  -10.0 * ReverseMRES);
    setHighHardLimitPos(axis_no, 160.0 * ReverseMRES);
    setAmplifierPercent(axis_no, 100);
    init_done[axis_no] = 1;
  }
}


static int handle_TCPSIM_cmd3(int motor_axis_no, const char *myarg_2)
{
  AXIS_CHECK_RETURN_ZERO(motor_axis_no);

  /* AB (stop) */
  if (0 == strcmp(myarg_2, "AB")) {
    motorStop(motor_axis_no);
    return TCPSIM_SEND_OK;
  }

  /* POS? */
  if (0 == strcmp(myarg_2, "POS?")) {
    cmd_buf_printf("%ld", NINT(getMotorPos(motor_axis_no)));
    return TCPSIM_SEND_NEWLINE;
  }
  /* ST? */
  if (0 == strcmp(myarg_2, "ST?")) {
    /* Status bits */
    #define STATUS_BIT_DIRECTION       0x1
    #define STATUS_BIT_DONE            0x2
    #define STATUS_BIT_MOVING          0x4
    #define STATUS_BIT_LIMIT_POS       0x8
    #define STATUS_BIT_LIMIT_NEG      0x10
  //#define STATUS_BIT_HOMING         0x20
    #define STATUS_BIT_HSIGNAL        0x40
    #define STATUS_BIT_HOMED          0x80
    #define STATUS_BIT_ERROR         0x100

    int axis_status = 0;
    if (getAxisDone(motor_axis_no)) axis_status       |= STATUS_BIT_DONE;
    if (isMotorMoving(motor_axis_no)) axis_status     |= STATUS_BIT_MOVING;
    if (getNegLimitSwitch(motor_axis_no)) axis_status |= STATUS_BIT_LIMIT_NEG;
    if (getPosLimitSwitch(motor_axis_no)) axis_status |= STATUS_BIT_LIMIT_POS;
    if (getAxisHome(motor_axis_no)) axis_status       |= STATUS_BIT_HSIGNAL;
    if (getAxisHomed(motor_axis_no)) axis_status      |= STATUS_BIT_HOMED;
    if (get_bError(motor_axis_no)) axis_status        |= STATUS_BIT_ERROR;

    cmd_buf_printf("%d", axis_status);
    return TCPSIM_SEND_NEWLINE;
  }

  return 0;
}

static int handle_TCPSIM_cmd4(int motor_axis_no,
                              const char *myarg_2,
                              const char *myarg_3)
{
  int value = 0;
  int nvals = -1;
  AXIS_CHECK_RETURN_ZERO(motor_axis_no);

  if (0 == strcmp(myarg_2, "MA")) {
    nvals = sscanf(myarg_3, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      movePosition(motor_axis_no,
                   (double)value,
                   0, /* int relative, */
                   cmd_Motor_cmd[motor_axis_no].velocity,
                   1.0 /*double acceleration */ );
      return TCPSIM_SEND_OK;
    }
  }
  if (0 == strcmp(myarg_2, "VEL")) {
    nvals = sscanf(myarg_3, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      cmd_Motor_cmd[motor_axis_no].velocity = value;
      return TCPSIM_SEND_OK;
    }
  }
  if (0 == strcmp(myarg_2, "HOM")) {
    nvals = sscanf(myarg_3, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      moveHome(motor_axis_no,
               value,
               cmd_Motor_cmd[motor_axis_no].velocity,
               1 /* double acceleration */);
      return TCPSIM_SEND_OK;
    }
  }
  if (0 == strcmp(myarg_2, "JOG")) {
    nvals = sscanf(myarg_3, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      int direction = 1;
      if (value < 0) {
        direction = 0;
        value = 0 - value;
      }
      moveVelocity(motor_axis_no,
                   direction,
                   (double)value,
                   1 /* double acceleration */);
      return TCPSIM_SEND_OK;
    }
  }
  if (0 == strcmp(myarg_2, "POW")) {
    nvals = sscanf(myarg_3, "%d", &value);
    LOGINFO5("%s/%s:%d res=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             nvals);
    if (nvals == 1) {
      if (!setAmplifierPercent(motor_axis_no, value)) {
        return TCPSIM_SEND_OK;
      }
    }
  }
  return 0;
}

int cmd_TCPsim(int argc, const char *argv[])
{
  int ret = 0;
  int axis_no = 0;
  LOGINFO5("%s/%s:%d argc=%d argv[0]=%s\n",
           __FILE__, __FUNCTION__, __LINE__,
           argc, argv[0]);
  /* We use a UNIX like counting:
     argc == 4
     argv[0]  "1:MOVE 2011" (non-UNIX: the whole command line)
     argv[1]  "1:MOVE"
     argv[2]  "2011"
  */
  if (argc >= 2) {
    int nvals;
    nvals = sscanf(argv[1], "%d", &axis_no);
    LOGINFO5("%s/%s:%d argv[1]=%s argc=%d nvals=%d axis_no=%d\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[1], argc, nvals, axis_no);
    if (nvals != 1 || axis_no < 1) {
      return 0; /* Not TCPSIM */
    }
  }

  if (argc == 4) {
    LOGINFO5("%s/%s:%d axis_no=%d argv[2]=%s argv[3]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             axis_no, argv[2], argv[3]);
    ret = handle_TCPSIM_cmd4(axis_no, argv[2], argv[3]);
  } else if (argc == 3) {
    LOGINFO5("%s/%s:%d argv[1]=%s argv[2]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[1], argv[2]);
    ret = handle_TCPSIM_cmd3(axis_no, argv[2]);
  }
  switch (ret) {
    case TCPSIM_SEND_OK:
      cmd_buf_printf("%s\n", "OK");
      break;
    case TCPSIM_SEND_NEWLINE:
      cmd_buf_printf("\n");
      break;
    default:
      LOGERR("%s/%s:%d argv[0]=%s\n",
             __FILE__, __FUNCTION__, __LINE__,
             argv[0]);
      ;
  }
  return ret;
}
/******************************************************************************/
