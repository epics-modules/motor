#define VERSION 3.00


#include    <string.h>
#include        "motorRecord.h"
#include        "motor.h"
#include        "motordevCom.h"
#include        "drvLinMot.h"
#include    "epicsExport.h"

#define STATIC static

extern struct driver_table LinMot_access;

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

/*----------------debugging-----------------*/
volatile int devLinMotDebug = 0;
extern "C" {epicsExportAddress(int, devLinMotDebug);}

static inline void Debug(int level, const char *format, ...) {
}

/* ----------------Create the dsets for devLinMot----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long LinMot_init(void *);
STATIC long LinMot_init_record(void *);
STATIC long LinMot_start_trans(struct motorRecord *);
STATIC RTN_STATUS LinMot_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS LinMot_end_trans(struct motorRecord *);
STATIC long VELO = 0;

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
    IMMEDIATE   /* SET_LOW_LIMIT */
};


static struct board_stat **LinMot_cards;

/* --------------------------- program data --------------------- */

/* initialize device support for LinMot stepper motor */
STATIC long LinMot_init(void *arg)
{
    return((long)0);
}


/* initialize a record instance */
STATIC long LinMot_init_record(void *arg)
{
    return((long)0);
}


/* start building a transaction */
STATIC long LinMot_start_trans(struct motorRecord *mr)
{
    return((long)0);
}


/* end building a transaction */
STATIC RTN_STATUS LinMot_end_trans(struct motorRecord *mr)
{
    return(OK);
    
}

/* add a part to the transaction */
STATIC RTN_STATUS LinMot_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    return (OK);
}