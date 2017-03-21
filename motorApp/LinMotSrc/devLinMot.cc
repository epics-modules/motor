#include <string.h>
#include <math.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvLinMot.h"
#include "epicsExport.h"

extern struct driver_table LinMot_access;

/* ----------------Create the dsets for devLinMot----------------- */
static struct driver_table *drvtabptr;
static long LinMot_init(void *);
static long LinMot_init_record(void *);
static long LinMot_start_trans(struct motorRecord *);
static RTN_STATUS LinMot_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS LinMot_end_trans(struct motorRecord *);

struct motor_dset devLinMot =
{
    {8, NULL, (DEVSUPFUN) LinMot_init, (DEVSUPFUN) LinMot_init_record, NULL},
    motor_update_values,
    LinMot_start_trans,
    LinMot_build_trans,
    LinMot_end_trans
};

extern "C" {epicsExportAddress(dset,devLinMot);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types LinMot_table[] = {
    MOTION,     /* MOVE_ABS */
    MOTION,     /* MOVE_REL */
    MOTION,     /* HOME_FOR */
    MOTION,     /* HOME_REV */
    IMMEDIATE,  /* LOAD_POS */
    IMMEDIATE,  /* SET_VEL_BASE */
    IMMEDIATE,  /* SET_VELOCITY */
    IMMEDIATE,  /* SET_ACCEL */
    IMMEDIATE,  /* GO */
    IMMEDIATE,  /* SET_ENC_RATIO */
    INFO,       /* GET_INFO */
    MOVE_TERM,  /* STOP_AXIS */
    VELOCITY,   /* JOG */
    IMMEDIATE,  /* SET_PGAIN */
    IMMEDIATE,  /* SET_IGAIN */
    IMMEDIATE,  /* SET_DGAIN */
    IMMEDIATE,  /* ENABLE_TORQUE */
    IMMEDIATE,  /* DISABL_TORQUE */
    IMMEDIATE,  /* PRIMITIVE */
    IMMEDIATE,  /* SET_HIGH_LIMIT */
    IMMEDIATE,  /* SET_LOW_LIMIT */
    VELOCITY,   /* JOG_VELOCITY */
    IMMEDIATE   /* SET_RESOLUTION */
};

static inline void Debug(int level, const char *format, ...) {
  #ifdef DEBUG
    if (level < devLinMotDebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
  #endif
}


static struct board_stat **LinMot_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for LinMot stepper motor */
static long LinMot_init(void *arg)
{
    return(OK);
}


/* initialize a record instance */
static long LinMot_init_record(void *arg)
{
    return(OK);
}


/* start building a transaction */
static long LinMot_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
static RTN_STATUS LinMot_end_trans(struct motorRecord *mr)
{
    return(OK);
}


/* add a part to the transaction */
static RTN_STATUS LinMot_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    return(OK);
}

