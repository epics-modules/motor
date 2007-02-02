/*
FILENAME...     motorUtil.cc
USAGE...        Motor Record Utility Support.

Version:        $Revision: 1.2 $
Modified By:    $Author: peterd $
Last Modified:  $Date: 2007-02-02 13:58:58 $
*/


/*
*  motorUtil_A.c -- version 1.0
*
*  Original Author: Kevin Peterson (kmpeters@anl.gov)
*  Date: December 11, 2002
*
*  UNICAT
*  Advanced Photon Source
*  Argonne National Laboratory
*
*  Current Author: Ron Sluiter
*/

#include <stdio.h>
#include <string.h>
#include <cadef.h>
#include <dbDefs.h>
#include <epicsString.h>
#include <epicsRingPointer.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <errlog.h>

#include <motor.h>

#define TIMEOUT 60 /* seconds */

/* ----- External Declarations ----- */
extern char **getMotorList();
/* ----- --------------------- ----- */

/* ----- Function Declarations ----- */
RTN_STATUS motorUtilInit(char *);
static int motorUtil_task(void *);
static chid getChID(char *);
static long pvMonitor(int, chid, int);
static void dmov_handler(struct event_handler_args);
static void allstop_handler(struct event_handler_args);
static void stopAll(chid, char *);
static int motorMovingCount();
static void moving(int, chid, short);
/* ----- --------------------- ----- */


typedef struct motor_pv_info
{
    char name[PVNAME_SZ];      /* pv names limited to 28 chars + term. in dbDefs.h */
    chid chid_dmov;     /* Channel id for <motor name>.DMOV */
    chid chid_stop;     /* Channel id for <motor name>.STOP */
    int in_motion;
    int index;          /* Call to ca_add_event() must have ptr to argument. */
} Motor_pv_info;


/* ----- Global Variables ----- */
int motorUtil_debug = 0;
int numMotors = 0;
/* ----- ---------------- ----- */

/* ----- Local Variables  ----- */
static Motor_pv_info *motorArray;
static char **motorlist = 0;
static char *vme;
static int old_numMotorsMoving = -1;
static short old_alldone_value = -1;
static chid chid_allstop, chid_moving, chid_alldone;
/* ----- ---------------- ----- */


RTN_STATUS motorUtilInit(char *vme_name)
{
    RTN_STATUS status = OK;
    
    if (strlen(vme) > PVNAME_SZ - 7 )
    {
        printf( "motorUtilInit: Prefix %s has more than %d characters. Exiting",
                vme_name, PVNAME_SZ - 7 );
        return ERROR;
    }

    vme = epicsStrDup(vme_name);

    epicsThreadCreate((char *) "motorUtil", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) motorUtil_task, (void *) NULL);
    return(status);
}


static int motorUtil_task(void *arg)
{
    char temp[PVNAME_STRINGSZ+5];
    int itera, status;

    SEVCHK(ca_context_create(ca_enable_preemptive_callback),
           "motorUtil: ca_context_create() error");

    motorlist = getMotorList();
    if (motorUtil_debug)
        errlogPrintf("There are %i motors\n", numMotors);
    
    if (numMotors > 0)
    {
        motorArray = (Motor_pv_info *) callocMustSucceed(numMotors,
                                   sizeof(Motor_pv_info), "motorUtil:init()");
   
        /* setup $(P)moving */
        strcpy(temp, vme);
        strcat(temp, "moving.VAL");
        chid_moving = getChID(temp);

        /* setup $(P)alldone */
        strcpy(temp, vme);
        strcat(temp, "alldone.VAL");
        chid_alldone = getChID(temp);

        /* loop over motors in motorlist and fill in motorArray */
        for (itera=0; itera < numMotors; itera++)
        {
            motorArray[itera].index = itera;

            /* Setup .DMOVs */
            strcpy(motorArray[itera].name, motorlist[itera]);
            strcpy(temp, motorlist[itera]);
            strcat(temp, ".DMOV");
            motorArray[itera].chid_dmov = getChID(temp);
            status = pvMonitor(1, motorArray[itera].chid_dmov, itera);
                    
            /* Setup .STOPs */
            strcpy(temp, motorlist[itera]);
            strcat(temp, ".STOP");
            motorArray[itera].chid_stop = getChID(temp);        
        }

        /* setup $(P)allstop */
        strcpy(temp, vme);
        strcat(temp, "allstop.VAL");
        chid_allstop = getChID(temp);
        status = pvMonitor(0, chid_allstop, -1);
    }
    
    epicsThreadSuspendSelf(); /* Wait Forever */
    return(ERROR);
}


static chid getChID(char *PVname)
{
    chid channelID = 0;
    int status;

    /* In R3.14 ca_create_channel() will replace ca_search_and_connect() */
    status = ca_search_and_connect(PVname, &channelID, 0, 0);
    status = ca_pend_io(TIMEOUT);

    if (status != ECA_NORMAL)
    {
        SEVCHK(status, "ca_search_and_connect");
        ca_task_exit(); /* this is serious */
        errlogPrintf("(motorUtil_A.c) getChID error: %i\n", status);
    }
    return channelID;
}


static long pvMonitor(int eventType, chid channelID, int motor_index)
{
    int status; 

    /* Create monitor */
    if (eventType)      /* moving() */
        status = ca_add_event(DBR_SHORT, channelID, &dmov_handler,
                              &(motorArray[motor_index].index), 0);
    else                /* stopAll() */
        status = ca_add_event(DBR_STRING, channelID, &allstop_handler, 0, 0);
    status = ca_pend_io(TIMEOUT);

    if (status != ECA_NORMAL)
    {
        SEVCHK(status, "ca_add_event");
        ca_task_exit(); /* this is serious */
        return status;
    }

    return ECA_NORMAL;
}


static void allstop_handler(struct event_handler_args args)
{
    stopAll(args.chid, (char *) args.dbr);
}


static void stopAll(chid callback_chid, char *callback_value)
{
    int itera, status = 0;
    short val = 1, release_val = 0;
    
    if (callback_chid != chid_allstop)
        errlogPrintf("callback_chid = %08x, chid_allstop = %08x\n",
               (unsigned int) callback_chid, (unsigned int) chid_allstop);
    
    if (strcmp(callback_value, "release") != 0)
    {
        /* if at least one motor is moving, then continue with stop all */
        if (motorMovingCount())
        {
            for(itera=0; itera < numMotors; itera++)
                ca_put(DBR_SHORT, motorArray[itera].chid_stop, &val);
            status = ca_flush_io(); 
        }

        /* reset allstop so that it may be called again */
        ca_put(DBR_SHORT, chid_allstop, &release_val);
        status = ca_flush_io();
        if (motorUtil_debug)
            errlogPrintf("reset allstop to \"release\"\n");
    }
    else if (motorUtil_debug)
	errlogPrintf("didn't need to reset allstop\n");
}


static void dmov_handler(struct event_handler_args args)
{
    moving(*((int *) args.usr), args.chid, *((short *) args.dbr));
}


static void moving(int callback_motor_index, chid callback_chid,
                   short callback_dmov)
{
    short new_alldone_value, done = 1, not_done = 0;
    int numMotorsMoving, status;

    if (motorUtil_debug)            
        errlogPrintf("%s is %s\n", motorArray[callback_motor_index].name,
               (callback_dmov) ? "STOPPED" : "MOVING");

    if (callback_dmov)                      
        motorArray[callback_motor_index].in_motion = 0;
    else
        motorArray[callback_motor_index].in_motion = 1;
    
    numMotorsMoving = motorMovingCount();

    new_alldone_value = (numMotorsMoving) ? 0 : 1;
    
    /* check to see if $(P)alldone needs to be updated */
    if (new_alldone_value != old_alldone_value)
    {
        /* give $(P)alldone the appropriate value */
        if (numMotorsMoving == 0)
        {
            if (motorUtil_debug)
                errlogPrintf("sending alldone = TRUE\n");

            ca_put(DBR_SHORT, chid_alldone, &done);
            old_alldone_value = new_alldone_value;
        }
        else
        {
            if (motorUtil_debug)
                errlogPrintf("sending alldone = FALSE\n");

            ca_put(DBR_SHORT, chid_alldone, &not_done);
            old_alldone_value = new_alldone_value;
        }
    }
    else if (motorUtil_debug)
	errlogPrintf("the alldone value remains the same.\n");

    /* check to see if $(P)moving needs to be updated */
    if (numMotorsMoving != old_numMotorsMoving)
    {
        if (motorUtil_debug)
            errlogPrintf("updating number of motors moving\n");

        /* give $(P)moving the appropriate value */
        ca_put(DBR_LONG, chid_moving, &numMotorsMoving);

        old_numMotorsMoving = numMotorsMoving;
    }
    else if (motorUtil_debug)
	errlogPrintf("the number of motors moving remains the same.\n");
    
    /* send the ca_puts */
    status = ca_flush_io();
}


static int motorMovingCount()
{
    int itera, in_motion_count=0;

    for (itera=0; itera < numMotors; itera++)
        in_motion_count += motorArray[itera].in_motion;

    return in_motion_count;
}


void listMovingMotors()
{
    int itera;
  
    errlogPrintf("\nThe following motors are moving:\n");
    
    for (itera=0; itera < numMotors; itera++)
        if (motorArray[itera].in_motion == 1)
            errlogPrintf("%s, index = %i\n", motorArray[itera].name,
                   motorArray[itera].index);
}


void printChIDlist()
{
    int itera;

    for (itera=0; itera < numMotors; itera++)
    {
        errlogPrintf("i = %i,\tname = %s\tchid_dmov = %08x\tchid_stop = \
               %08x\tin_motion = %i\tindex = %i\n", itera,
               motorArray[itera].name,
               (unsigned int) motorArray[itera].chid_dmov,
               (unsigned int) motorArray[itera].chid_stop, 
               motorArray[itera].in_motion, motorArray[itera].index);
    }
    
    errlogPrintf("chid_allstop = %08x\n", (unsigned int) chid_allstop);
    errlogPrintf("chid_alldone = %08x\n", (unsigned int) chid_alldone);
    errlogPrintf("chid_moving = %08x\n", (unsigned int) chid_moving);
}


extern "C"
{

static const iocshArg Arg = {"IOC name", iocshArgString};
static const iocshArg * const motorUtilArg[1]  = {&Arg};
static const iocshFuncDef motorUtilDef  = {"motorUtilInit", 1, motorUtilArg};

static void motorUtilCallFunc(const iocshArgBuf *args)
{
    motorUtilInit(args[0].sval);
}

static void motorUtilRegister(void)
{
    iocshRegister(&motorUtilDef,  motorUtilCallFunc);
}

epicsExportRegistrar(motorUtilRegister);

} // extern "C"

