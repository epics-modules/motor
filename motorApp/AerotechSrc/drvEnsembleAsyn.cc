/*
FILENAME... drvEnsembleAsyn.cc
USAGE...    Motor record asyn driver level support for Aerotech Ensemble.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
*      Original Author: Mark Rivers
*      Date: 07/28/09
*      Current Author: Aerotech, Inc.
*
-----------------------------------------------------------------------------
COPYRIGHT NOTICE
-----------------------------------------------------------------------------
Copyright (c) 2002 The University of Chicago, as Operator of Argonne
National Laboratory.
Copyright (c) 2002 The Regents of the University of California, as
Operator of Los Alamos National Laboratory.
Synapps Versions 4-5
and higher are distributed subject to a Software License Agreement found
in file LICENSE that is included with this distribution.
-----------------------------------------------------------------------------
* NOTES
* -----
* Verified with firmware:
*      - 4.01.000
*
* Modification Log:
* -----------------
*
* .01 07-28-09 cjb Initialized from drvMM4000Asyn.c (Newport)
* .02 03-02-10 rls Original check into version control.
* .03 03-16-10 rls Switch to C++ file.
* .04 04-07-10 rls Acknowledged fault before enabling drive.
* .05 04-19-10 rls Change back to reading EOT LS's and let record determine
*                  limit error condition.
* .06 04-21-10 rls Temporarily backing out previous change on EOT LS's; need
*                  more cnfg. info from controller on LS active high/low.
* .07 04-28-10 rls - IOC crashes at boot-up when Ensemble is power-ed off;
*                    error check that pController is not Null.
*                  - changed EOT LS read status from AXISFAULT to AXISSTATUS so
*                    LS status can be monitored independent of fault status.
* .08 06-03-10 rls - Set motorAxisHasEncoder based on CfgFbkPosType parameter.
* .09 09-13-10 rls - Bug fix from Wang Xiaoqiang (PSI); remove redundant EOS
*                    append in sendAndReceive().  Fixes comm. problem with
*                    Ensemble firmware 2.54.004 and above.
* .10 09-29-10 rls - Commented out home search until firmware upgrade that
*                    allows aborting home search from ASCII protocol.
* .11 06-21-11 rls - Bug fix for jog velocity and acceleration not converted
*                    from raw units to Ensemble user units when the
*                    PosScaleFactor parameter is not 1.
* .12 11-30-11 rls - Ensemble 4.x compatibility.
*                  - In order to support SCURVE trajectories; changed from
*                    MOVE[ABS/INC] to LINEAR move command. 
* .13 12-15-11 rls - Bug fix for jog not terminating; must check both 
*                    PLANESTATUS and AXISSTATUS for move_active.
* .14 12-22-11 rls - Restore home search; using HomeAsync.abx vendor program.
* .15 02-02-12 rls - Replace "stepSize > 0.0" test with ReverseDirec parameter.
*/


#include <stddef.h>
#include "epicsThread.h"
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "epicsFindSymbol.h"
#include "epicsTime.h"
#include "epicsThread.h"
#include "epicsEvent.h"
#include "epicsString.h"
#include "epicsMutex.h"
#include "ellLib.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"
#include "errlog.h"

#include "drvSup.h"
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"

#include "paramLib.h"
#include "drvEnsembleAsyn.h"
#include "ParameterId.h"
#include "epicsExport.h"

motorAxisDrvSET_t motorEnsemble = 
{
    14,
    motorAxisReport,            /**< Standard EPICS driver report function (optional) */
    motorAxisInit,              /**< Standard EPICS driver initialization function (optional) */
    motorAxisSetLog,            /**< Defines an external logging function (optional) */
    motorAxisOpen,              /**< Driver open function */
    motorAxisClose,             /**< Driver close function */
    motorAxisSetCallback,       /**< Provides a callback function the driver can call when the status updates */
    motorAxisSetDouble,         /**< Pointer to function to set a double value */
    motorAxisSetInteger,        /**< Pointer to function to set an integer value */
    motorAxisGetDouble,         /**< Pointer to function to get a double value */
    motorAxisGetInteger,        /**< Pointer to function to get an integer value */
    motorAxisHome,              /**< Pointer to function to execute a move to reference or home */
    motorAxisMove,              /**< Pointer to function to execute a position move */
    motorAxisVelocityMove,      /**< Pointer to function to execute a velocity mode move */
    motorAxisStop,              /**< Pointer to function to stop motion */
    motorAxisforceCallback,     /**< Pointer to function to request a poller status update */
    motorAxisProfileMove,       /**< Pointer to function to execute a profile move */
    motorAxisTriggerProfile     /**< Pointer to function to trigger a profile move */
};

extern "C" {
epicsExportAddress(drvet, motorEnsemble);
}

typedef struct
{
    epicsMutexId EnsembleLock;
    asynUser *pasynUser;
    int numAxes;
    double movingPollPeriod;
    double idlePollPeriod;
    epicsEventId pollEventId;
    epicsMutexId sendReceiveMutex;
    AXIS_HDL pAxis;  /* array of axes */
} EnsembleController;

typedef struct motorAxisHandle
{
    EnsembleController *pController;
    PARAMS params;
    double currentPosition;
    double stepSize;
    double homePreset;
    int homeDirection;
    int closedLoop;
    int axisStatus;
    int card;
    int axis;
    int maxDigits;
    motorAxisLogFunc print;
    void *logParam;
    epicsMutexId mutexId;
    Switch_Level swconfig;
    int lastFault;
    bool ReverseDirec;
} motorAxis;

typedef struct
{
    AXIS_HDL pFirst;
    epicsThreadId motorThread;
    motorAxisLogFunc print;
    void *logParam;
    epicsTimeStamp now;
} motorEnsemble_t;


extern "C" {
static int motorEnsembleLogMsg(void * param, const motorAxisLogMask_t logMask, const char *pFormat, ...);
}
static asynStatus sendAndReceive(EnsembleController *pController, char *outputString, char *inputString, int inputSize);

#define PRINT   (drv.print)
#define FLOW    motorAxisTraceFlow
#define TERROR   motorAxisTraceError
#define IODRIVER  motorAxisTraceIODriver

#define ENSEMBLE_MAX_AXES 10
#define BUFFER_SIZE 100 /* Size of input and output buffers */
#define TIMEOUT 2.0     /* Timeout for I/O in seconds */


/* Status byte bits */
#define ENABLED_BIT     0x00000001
#define IN_POSITION_BIT 0x00000004
#define IN_MOTION_BIT   0x00000008
#define DIRECTION_BIT   0x00000200
#define HOME_LIMIT_BIT  0x01000000
#define HOME_MARKER_BIT 0x02000000


/* Fault status bits */
#define  CW_FAULT_BIT   0x004
#define CCW_FAULT_BIT   0x008


/* The following should be defined to have the same value as
the Ensemble parameters specified */
#define ASCII_EOS_CHAR      '\n'  /* AsciiCmdEOSChar */
#define ASCII_EOS_STR       "\n"
#define ASCII_ACK_CHAR      '%'   /* AsciiCmdAckChar */
#define ASCII_NAK_CHAR      '!'   /* AsciiCmdNakChar */
#define ASCII_FAULT_CHAR    '#'   /* AsciiCmdFaultChar */
#define ASCII_TIMEOUT_CHAR  '$'   /* AsciiCmdTimeoutChar */

#define TCP_TIMEOUT 2.0
static motorEnsemble_t drv = {NULL, NULL, motorEnsembleLogMsg, 0, {0, 0}};
static int numEnsembleControllers;
/* Pointer to array of controller structures */
static EnsembleController *pEnsembleController=NULL;

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

static void motorAxisReportAxis(AXIS_HDL pAxis, int level)
{
    if (level > 0)
    {
        printf("Axis %d\n", pAxis->axis);
        printf("   axisStatus:  0x%x\n", pAxis->axisStatus);
        printf("   home preset: %f\n", pAxis->homePreset);
        printf("   step size:   %f\n", pAxis->stepSize);
        printf("   max digits:  %d\n", pAxis->maxDigits);
    }
}

static void motorAxisReport(int level)
{
    int i, j;

    for (i=0; i<numEnsembleControllers; i++)
    {
        if (level)
        {
            printf("    moving poll period: %f\n", pEnsembleController[i].movingPollPeriod);
            printf("    idle poll period: %f\n", pEnsembleController[i].idlePollPeriod);
        }
        for (j=0; j<pEnsembleController[i].numAxes; j++)
            motorAxisReportAxis(&pEnsembleController[i].pAxis[j], level);
    }   
}


static int motorAxisInit(void)
{
    int controller, axis;

    for (controller = 0; controller < numEnsembleControllers; controller++)
    {
        AXIS_HDL pAxis;
        for (axis = 0; axis < pEnsembleController[controller].numAxes; axis++)
        {
            pAxis = &pEnsembleController[controller].pAxis[axis];
            if (!pAxis->mutexId)
                break;
            epicsMutexLock(pAxis->mutexId);

            /*Set GAIN_SUPPORT on so that at least, CNEN functions. */
            motorParam->setInteger(pAxis->params, motorAxisHasClosedLoop, 1);

            motorParam->callCallback(pAxis->params);
            epicsMutexUnlock(pAxis->mutexId);
        }
    }
    return(MOTOR_AXIS_OK);
}

static int motorAxisSetLog( AXIS_HDL pAxis, motorAxisLogFunc logFunc, void * param )
{
    if (pAxis == NULL)
    {
        if (logFunc == NULL)
        {
            drv.print=motorEnsembleLogMsg;
            drv.logParam = NULL;
        }
        else
        {
            drv.print=logFunc;
            drv.logParam = param;
        }
    }
    else
    {
        if (logFunc == NULL)
        {
            pAxis->print=motorEnsembleLogMsg;
            pAxis->logParam = NULL;
        }
        else
        {
            pAxis->print=logFunc;
            pAxis->logParam = param;
        }
    }
    return (MOTOR_AXIS_OK);
}

static AXIS_HDL motorAxisOpen(int card, int axis, char * param)
{
    AXIS_HDL pAxis;

    if (card > numEnsembleControllers)
        return(NULL);
    if (axis > pEnsembleController[card].numAxes)
        return(NULL);
    pAxis = &pEnsembleController[card].pAxis[axis];
    return (pAxis);
}

static int motorAxisClose(AXIS_HDL pAxis)
{
    return (MOTOR_AXIS_OK);
}

static int motorAxisGetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int * value)
{
    if (pAxis == NULL || pAxis->params == NULL)
        return (MOTOR_AXIS_ERROR);
    else
        return (motorParam->getInteger(pAxis->params, (paramIndex) function, value));
}

static int motorAxisGetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double * value)
{
    if (pAxis == NULL || pAxis->params == NULL)
        return (MOTOR_AXIS_ERROR);
    else
        return (motorParam->getDouble(pAxis->params, (paramIndex) function, value));
}

static int motorAxisSetCallback(AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param)
{
    if (pAxis == NULL || pAxis->params == NULL)
        return (MOTOR_AXIS_ERROR);
    else
        return (motorParam->setCallback(pAxis->params, callback, param));
}

static int motorAxisSetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double value)
{
    asynStatus status = asynSuccess;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];

    if (pAxis == NULL || pAxis->pController == NULL)
      return(asynError);

    epicsMutexLock(pAxis->mutexId);

    switch (function)
    {
    case motorAxisPosition:
    {
        double offset = value * fabs(pAxis->stepSize);
        sprintf(outputBuff, "POSOFFSET SET @%d, %.*f", pAxis->axis, pAxis->maxDigits, offset);
        status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
        if (inputBuff[0] != ASCII_ACK_CHAR)
          status = asynError;
        break;
    }
    case motorAxisEncoderRatio:
    {
        PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: Ensemble does not support setting encoder ratio\n");
        break;
    }
    case motorAxisResolution:
    {
        /* we need to scale over a dozen other parameters if this changed in some cases, so the user should just use
         * the Configuration Manager to change this setting to ensure that this is done correctly */
        PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: Ensemble does not support setting motor resolution\n");
        break;
    }
    case motorAxisLowLimit:
    {
        PRINT(pAxis->logParam, TERROR, "Driver does not set Ensemble's Low Limit\n");
        break;
    }
    case motorAxisHighLimit:
    {
        PRINT(pAxis->logParam, TERROR, "Driver does not set Ensemble's High Limit\n");
        break;
    }
    case motorAxisPGain:
    {
        PRINT(pAxis->logParam, TERROR, "Ensemble does not support setting proportional gain\n");
        break;
    }
    case motorAxisIGain:
    {
        PRINT(pAxis->logParam, TERROR, "Ensemble does not support setting integral gain\n");
        break;
    }
    case motorAxisDGain:
    {
        PRINT(pAxis->logParam, TERROR, "Ensemble does not support setting derivative gain\n");
        break;
    }
    default:
        PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: unknown function %d\n", function);
        break;
    }
    if (status == asynSuccess )
    {
        motorParam->setDouble(pAxis->params, function, value);
        motorParam->callCallback(pAxis->params);
    }
    epicsMutexUnlock(pAxis->mutexId);

    return(status);
}

static int motorAxisSetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    int status, FaultStatus;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];

    if (pAxis == NULL || pAxis->pController == NULL)
        return (MOTOR_AXIS_ERROR);

    epicsMutexLock(pAxis->mutexId);

    switch (function)
    {
    case motorAxisClosedLoop:
        if (value == 0)
            sprintf(outputBuff, "DISABLE @%d", pAxis->axis);
        else
        {
            sprintf(outputBuff, "AXISFAULT @%d", pAxis->axis);
            ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));

            if (ret_status == 0 && inputBuff[0] == ASCII_ACK_CHAR && (FaultStatus = atoi(&inputBuff[1])))
            {
                PRINT(pAxis->logParam, TERROR, "motorAxisSetInteger: FAULTACK = %X\n", FaultStatus);
                sprintf(outputBuff, "FAULTACK @%d", pAxis->axis);
                ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
            }
            sprintf(outputBuff, "ENABLE @%d", pAxis->axis);
        }
        ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
        break;
    default:
        PRINT(pAxis->logParam, TERROR, "motorAxisSetInteger: unknown function %d\n", function);
        break;
    }
    if (ret_status != MOTOR_AXIS_ERROR)
    {
        status = motorParam->setInteger(pAxis->params, function, value);
        motorParam->callCallback(pAxis->params);
    }
    epicsMutexUnlock(pAxis->mutexId);
    return (ret_status);
}


static int motorAxisMove(AXIS_HDL pAxis, double position, int relative,
                         double min_velocity, double max_velocity, double acceleration)
{
    int ret_status;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    const char *moveCommand;
    bool posdir;
    int axis, maxDigits;

    if (pAxis == NULL || pAxis->pController == NULL)
        return (MOTOR_AXIS_ERROR);

    axis = pAxis->axis;
    maxDigits = pAxis->maxDigits;

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f\n",
          pAxis->card, axis, position, min_velocity, max_velocity, acceleration);

    posdir = false;

    if (relative)
    {
        if (position >= 0.0)
            posdir = true;
        else
            posdir = false;
        moveCommand = "INC";
    }
    else
    {
        if (position >= pAxis->currentPosition)
            posdir = true;
        else
            posdir = false;
        moveCommand = "ABS";
    }

    sprintf(outputBuff, "%s", moveCommand);
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    if (ret_status)
        return (MOTOR_AXIS_ERROR);

    if (acceleration > 0)
    { /* only use the acceleration if > 0 */
        sprintf(outputBuff, "RAMP RATE %.*f", maxDigits, acceleration * fabs(pAxis->stepSize));
        ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    }

    sprintf(outputBuff, "LINEAR @%d %.*f F%.*f", axis, maxDigits, position * fabs(pAxis->stepSize),
            maxDigits, max_velocity * fabs(pAxis->stepSize));

    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    if (ret_status)
        return (MOTOR_AXIS_ERROR);

    if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK)
    {
        motorParam->setInteger(pAxis->params, motorAxisDirection, (int) posdir);
        /* Ensure that the motor record's next status update sees motorAxisDone = False. */
        motorParam->setInteger(pAxis->params, motorAxisDone, 0);
        motorParam->callCallback(pAxis->params);
        epicsMutexUnlock(pAxis->mutexId);
    }

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    return (MOTOR_AXIS_OK);
}

static int motorAxisHome(AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards)
{
    int ret_status;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    epicsUInt32 hparam;
    int axis;

    if (pAxis == NULL || pAxis->pController == NULL)
        return (MOTOR_AXIS_ERROR);

    axis = pAxis->axis;
    
    PRINT(pAxis->logParam, FLOW, "motorAxisHome: set card %d, axis %d to home, forwards = %d\n",
          pAxis->card, axis, forwards);

    if (max_velocity > 0)
    {
        sprintf(outputBuff, "SETPARM @%d, %d, %.*f", axis, PARAMETERID_HomeSpeed, pAxis->maxDigits,
                max_velocity * fabs(pAxis->stepSize)); /* HomeFeedRate */
        ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    }

    if (acceleration > 0)
        sprintf(outputBuff, "SETPARM @%d, %d, %.*f", axis, PARAMETERID_HomeRampRate, pAxis->maxDigits,
                acceleration * fabs(pAxis->stepSize)); /* HomeAccelDecelRate */

    hparam = pAxis->homeDirection;
    if (forwards == 1)
        hparam |= 0x00000001;
    else
        hparam &= 0xFFFFFFFE;
    pAxis->homeDirection = hparam;

    sprintf(outputBuff, "SETPARM @%d, %d, %d", axis, PARAMETERID_HomeSetup, hparam); /* HomeDirection */
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));

    /* Set IGLOBAL(32) for one axis and IGLOBAL(33) for axis #; according to HomeAsync.ab protocol*/
    sprintf(outputBuff, "IGLOBAL(32) = 1");
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    sprintf(outputBuff, "IGLOBAL(33) = %d", axis);
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));

    sprintf(outputBuff,"PROGRAM RUN 5, \"HomeAsync.bcx\"");
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    if (ret_status)
        return(MOTOR_AXIS_ERROR);
    if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK)
    {
        motorParam->setInteger(pAxis->params, motorAxisDirection, forwards);
        /* Ensure that the motor record's next status update sees motorAxisDone = False. */
        motorParam->setInteger(pAxis->params, motorAxisDone, 0);
        motorParam->callCallback(pAxis->params);
        epicsMutexUnlock(pAxis->mutexId);
    }

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    return (MOTOR_AXIS_OK);
}


static int motorAxisVelocityMove(AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration)
{
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    int ret_status;

    if (pAxis == NULL || pAxis->pController == NULL)
        return(MOTOR_AXIS_ERROR);

    sprintf(outputBuff, "SETPARM @%d, %d, %.*f", pAxis->axis, PARAMETERID_DefaultRampRate,
            pAxis->maxDigits, acceleration * fabs(pAxis->stepSize));
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    sprintf(outputBuff, "FREERUN @%d %.*f", pAxis->axis, pAxis->maxDigits, velocity  * fabs(pAxis->stepSize));
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));

    if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK)
    {
        motorParam->setInteger(pAxis->params, motorAxisDirection, (velocity > 0.0 ? 1 : 0));
        /* Ensure that the motor record's next status update sees motorAxisDone = False. */
        motorParam->setInteger(pAxis->params, motorAxisDone, 0);
        motorParam->callCallback(pAxis->params);
        epicsMutexUnlock(pAxis->mutexId);
    }

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    return (ret_status);
}

static int motorAxisProfileMove(AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger)
{
    return (MOTOR_AXIS_ERROR);
}

static int motorAxisTriggerProfile(AXIS_HDL pAxis)
{
    return (MOTOR_AXIS_ERROR);
}

static int motorAxisStop(AXIS_HDL pAxis, double acceleration)
{
    int ret_status;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];

    if (pAxis == NULL || pAxis->pController == NULL)
        return (MOTOR_AXIS_ERROR);

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d\n",
          pAxis->card, pAxis->axis);

    /* we can't accurately determine which type of motion is occurring on the controller, so don't worry about the acceleration rate, just stop the motion on the axis */
    sprintf(outputBuff, "ABORT @%d", pAxis->axis);    
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    return (ret_status);
}

static int motorAxisforceCallback(AXIS_HDL pAxis)
{
    if (pAxis == NULL || pAxis->pController == NULL)
        return(MOTOR_AXIS_ERROR);

    PRINT(pAxis->logParam, FLOW, "motorAxisforceCallback: request card %d, axis %d status update\n",
          pAxis->card, pAxis->axis);

    /* Force a status update. */
    motorParam->forceCallback(pAxis->params);

    /* Send a signal to the poller task which will make it do a status update */
    epicsEventSignal(pAxis->pController->pollEventId);
    return(MOTOR_AXIS_OK);
}


static void EnsemblePoller(EnsembleController *pController)
{
    /* This is the task that polls the Ensemble */
    double timeout;
    AXIS_HDL pAxis;
    int status, itera, comStatus;
    Axis_Status axisStatus;
    bool anyMoving;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];

    timeout = pController->idlePollPeriod;
    epicsEventSignal(pController->pollEventId);  /* Force on poll at startup */

    while (1)
    {
        if (timeout != 0.)
            status = epicsEventWaitWithTimeout(pController->pollEventId, timeout);
        else
            status = epicsEventWait(pController->pollEventId);

        anyMoving = false;

        for (itera = 0; itera < pController->numAxes; itera++)
        {
            pAxis = &pController->pAxis[itera];
            if (!pAxis->mutexId)
                break;
            epicsMutexLock(pAxis->mutexId);
            sprintf(outputBuff, "AXISSTATUS(@%d)", pAxis->axis); 
            comStatus = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (comStatus != asynSuccess || strlen(inputBuff) <= 1)
            {
                PRINT(pAxis->logParam, TERROR, "EnsemblePoller: error reading status=%d\n", comStatus);
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
                epicsMutexUnlock(pAxis->mutexId);
                continue;
            }
            else
            {
                PARAMS params = pAxis->params;
                axisStatus.All = 0;
                if (inputBuff[0] != ASCII_ACK_CHAR)
                {
                    epicsMutexUnlock(pAxis->mutexId);
                    continue;
                }
                else
                {
                    int CW_sw_active, CCW_sw_active;
                    bool move_active;

                    motorParam->setInteger(params, motorAxisCommError, 0);
                    axisStatus.All = atoi(&inputBuff[1]);
                    
                    comStatus = sendAndReceive(pController, (char *) "PLANESTATUS(0)", inputBuff, sizeof(inputBuff));
                    move_active = (0x01 & atoi(&inputBuff[1])) ? true : false;
                    move_active |= axisStatus.Bits.move_active;
                    motorParam->setInteger(params, motorAxisDone, !move_active);
                    if (move_active)
                        anyMoving = true;

                    motorParam->setInteger(pAxis->params, motorAxisPowerOn, axisStatus.Bits.axis_enabled);
                    motorParam->setInteger(pAxis->params, motorAxisHomeSignal, axisStatus.Bits.home_limit);

                    if (pAxis->ReverseDirec == true)
                        motorParam->setInteger(pAxis->params, motorAxisDirection, axisStatus.Bits.motion_ccw);
                    else
                        motorParam->setInteger(pAxis->params, motorAxisDirection, !axisStatus.Bits.motion_ccw);
                    
                    CW_sw_active  = !(axisStatus.Bits.CW_limit  ^ pAxis->swconfig.Bits.CWEOTSWstate);
                    CCW_sw_active = !(axisStatus.Bits.CCW_limit ^ pAxis->swconfig.Bits.CCWEOTSWstate);
                
                    if (pAxis->ReverseDirec == false)
                    {
                        motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, CW_sw_active);
                        motorParam->setInteger(pAxis->params, motorAxisLowHardLimit,  CCW_sw_active);   
                    }
                    else
                    {
                        motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, CCW_sw_active);
                        motorParam->setInteger(pAxis->params, motorAxisLowHardLimit,  CW_sw_active);   
                    }
                }
                pAxis->axisStatus = axisStatus.All;
            }
            sprintf(outputBuff, "PFBKPROG(@%d)", pAxis->axis);
            comStatus = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (comStatus  != asynSuccess)
            {
                PRINT(pAxis->logParam, TERROR, "EnsemblePoller: error reading position=%d\n", comStatus);
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
                epicsMutexUnlock(pAxis->mutexId);
                continue;
            }
            else
            {
                double pfdbk;
                if (inputBuff[0] != ASCII_ACK_CHAR)
                    pfdbk = 0;
                else
                    pfdbk = atof(&inputBuff[1]);
                pAxis->currentPosition = pfdbk / fabs(pAxis->stepSize);
                motorParam->setDouble(pAxis->params, motorAxisPosition, pAxis->currentPosition);
                motorParam->setDouble(pAxis->params, motorAxisEncoderPosn, pAxis->currentPosition);
                PRINT(pAxis->logParam, IODRIVER, "EnsemblePoller: axis %d axisStatus=%x, position=%f\n", 
                      pAxis->axis, pAxis->axisStatus, pAxis->currentPosition);
            }
            sprintf(outputBuff, "AXISFAULT(@%d)", pAxis->axis);
            comStatus = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (comStatus != asynSuccess)
            {
                PRINT(pAxis->logParam, TERROR, "EnsemblePoller: error reading axisfault axis=%d error=%d\n", itera, comStatus);
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
                epicsMutexUnlock(pAxis->mutexId);
                continue;
            }
            else
            {
                if (inputBuff[0] != ASCII_ACK_CHAR)
                {
                    motorParam->setInteger(pAxis->params, motorAxisProblem, 1);
                    motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
                    epicsMutexUnlock(pAxis->mutexId);
                    continue;
                }
                else
                {
                    int axisFault;

                    axisFault = atoi(&inputBuff[1]);
                    if (axisFault == 0)
                    {
                        pAxis->lastFault = 0;
                        motorParam->setInteger(pAxis->params, motorAxisProblem, 0);
                    }
                    else if (axisFault != pAxis->lastFault)
                    {
                        pAxis->lastFault = axisFault;
                        motorParam->setInteger(pAxis->params, motorAxisProblem, 1);
                        PRINT(pAxis->logParam, TERROR, "EnsemblePoller: controller fault on axis=%d fault=0x%X\n", itera, axisFault);
                    }
                }
            }
            motorParam->callCallback(pAxis->params);
            epicsMutexUnlock(pAxis->mutexId);
        } /* Next axis */

        if (anyMoving == true)
            timeout = pController->movingPollPeriod;
        else
            timeout = pController->idlePollPeriod;
    } /* End while */
}

static int motorEnsembleLogMsg(void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{

    va_list     pvar;
    int         nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout,pFormat,pvar);
    va_end (pvar);
    printf("\n");
    return(nchar);
}

int EnsembleAsynSetup(int num_controllers)   /* number of Ensemble controllers in system.  */
{

    if (num_controllers < 1)
    {
        printf("EnsembleSetup, num_controllers must be > 0\n");
        return (MOTOR_AXIS_ERROR);
    }
    numEnsembleControllers = num_controllers;
    pEnsembleController = (EnsembleController *)calloc(numEnsembleControllers, sizeof(EnsembleController)); 
    return (MOTOR_AXIS_OK);
}

int EnsembleAsynConfig(int card,             /* Controller number */
                       const char *portName, /* asyn port name of serial or GPIB port */
                       int asynAddress,      /* asyn subaddress for GPIB */
                       int numAxes,          /* Number of axes this controller supports */
                       int movingPollPeriod, /* Time to poll (msec) when an axis is in motion */
                       int idlePollPeriod)   /* Time to poll (msec) when an axis is idle. 0 for no polling */

{
    AXIS_HDL pAxis;
    EnsembleController *pController;
    char threadName[20];
    int axis, status, digits, retry = 0;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    int numAxesFound;
    static char getparamstr[] = "GETPARM(@%d, %d)";

    if (numEnsembleControllers < 1)
    {
        printf("EnsembleConfig: no Ensemble controllers allocated, call EnsembleSetup first\n");
        return (MOTOR_AXIS_ERROR);
    }
    if ((card < 0) || (card >= numEnsembleControllers))
    {
        printf("EnsembleConfig: card must in range 0 to %d\n", numEnsembleControllers-1);
        return (MOTOR_AXIS_ERROR);
    }
    if ((numAxes < 1) || (numAxes > ENSEMBLE_MAX_AXES))
    {
        printf("EnsembleConfig: numAxes must in range 1 to %d\n", ENSEMBLE_MAX_AXES);
        return (MOTOR_AXIS_ERROR);
    }

    pController = &pEnsembleController[card];
    pController->pAxis = (AXIS_HDL) calloc(numAxes, sizeof(motorAxis));
    pController->numAxes = numAxes;
    pController->movingPollPeriod = movingPollPeriod/1000.;
    pController->idlePollPeriod = idlePollPeriod/1000.;

    pController->sendReceiveMutex = epicsMutexMustCreate();

    status = pasynOctetSyncIO->connect(portName, asynAddress, &pController->pasynUser, NULL);

    if (status != asynSuccess)
    {
        printf("EnsembleAsynConfig: cannot connect to asyn port %s\n", portName);
        return (MOTOR_AXIS_ERROR);
    }

    /* Set command End-of-string */
    pasynOctetSyncIO->setInputEos(pController->pasynUser, ASCII_EOS_STR, strlen(ASCII_EOS_STR));
    pasynOctetSyncIO->setOutputEos(pController->pasynUser, ASCII_EOS_STR, strlen(ASCII_EOS_STR));

    retry = 0;

    do
    {
        /* we only care if we get a response
        so we don't need to send a valid command */
        strcpy(outputBuff, "NONE");
        status = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));

        retry++;
    } while (status != asynSuccess && retry < 3);

    numAxesFound = 0;

    if (status != asynSuccess)
        return (MOTOR_AXIS_ERROR);
        
    /* Get the number of axes */
    for (axis = 0; axis < ENSEMBLE_MAX_AXES && numAxesFound < numAxes; axis++)
    {
        /* Does this axis actually exist? */
        sprintf(outputBuff, getparamstr, axis, PARAMETERID_AxisName);
        sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));

        /* We know the axis exists if we got an ACK response */
        if (inputBuff[0] == ASCII_ACK_CHAR)
        {
            pAxis = &pController->pAxis[numAxesFound];
            pAxis->pController = pController;
            pAxis->card = card;
            pAxis->axis = axis;
            pAxis->mutexId = epicsMutexMustCreate();
            pAxis->params = motorParam->create(0, MOTOR_AXIS_NUM_PARAMS);

            sprintf(outputBuff, getparamstr, axis, PARAMETERID_PositionFeedbackType);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR && atoi(&inputBuff[1]) > 0)
            {
              pAxis->closedLoop = 1;
              motorParam->setInteger(pAxis->params, motorAxisHasEncoder, 1);
            }

            sprintf(outputBuff, getparamstr, axis, PARAMETERID_CountsPerUnit);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->stepSize = 1 / atof(&inputBuff[1]);
            else
                pAxis->stepSize = 1;  
            digits = (int) -log10(fabs(pAxis->stepSize)) + 2;
            if (digits < 1)
                digits = 1;
            pAxis->maxDigits = digits;

            sprintf(outputBuff, getparamstr, axis, PARAMETERID_HomeOffset);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->homePreset = atof(&inputBuff[1]);

            sprintf(outputBuff, getparamstr, axis, PARAMETERID_HomeSetup);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->homeDirection = atoi(&inputBuff[1]);
            numAxesFound++;

            sprintf(outputBuff, getparamstr, axis, PARAMETERID_EndOfTravelLimitSetup);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->swconfig.All = atoi(&inputBuff[1]);

            /* Prevent ASCII interpreter from blocking during MOVEABS/INC commands. */
            sendAndReceive(pController, (char *) "WAIT MODE NOWAIT", inputBuff, sizeof(inputBuff));

            /* Set RAMP MODE to RATE. */
            sprintf(outputBuff, "RAMP MODE @%d RATE", axis);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));

            /* Get Reverse Direction indicator. */
            sprintf(outputBuff, getparamstr, axis, PARAMETERID_ReverseMotionDirection);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->ReverseDirec = (bool) atoi(&inputBuff[1]);
        }
    }

    pController->pollEventId = epicsEventMustCreate(epicsEventEmpty);

    /* Create the poller thread for this controller */
    epicsSnprintf(threadName, sizeof(threadName), "Ensemble:%d", card);
    epicsThreadCreate(threadName, epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) EnsemblePoller, (void *) pController);

    return (MOTOR_AXIS_OK);
}

static asynStatus sendAndReceive(EnsembleController *pController, char *outputBuff, char *inputBuff, int inputSize) 
{
    char outputCopy[BUFFER_SIZE];
    size_t nWriteRequested;
    size_t nWrite, nRead;
    int eomReason;
    asynStatus status;

    if (pController == NULL)
        return(asynError);

    strcpy(outputCopy, outputBuff);
    nWriteRequested=strlen(outputCopy);

    /* sendAndReceive is intended only for "fast" read-write operations (such as getting parameter/status values),
     * so we don't expect much latency on read/writes */
    epicsMutexLock(pController->sendReceiveMutex);

    status = pasynOctetSyncIO->writeRead(pController->pasynUser, 
             outputCopy, nWriteRequested, inputBuff, inputSize, TIMEOUT, &nWrite, &nRead, &eomReason);
    if (nWrite != nWriteRequested)
        status = asynError;
    if (status != asynSuccess)
        asynPrint(pController->pasynUser, ASYN_TRACE_ERROR,
                  "drvEnsembleAsyn:sendAndReceive error calling write, output=%s status=%d, error=%s\n",
                  outputCopy, status, pController->pasynUser->errorMessage);
    else
    {
        /* read until we have an ACK followed by a string (most likely will be the numeric value we're looking for) */
        while (status == asynSuccess && nRead > 1 && inputBuff[0] == ASCII_ACK_CHAR && inputBuff[1] == ASCII_EOS_CHAR)
            status = pasynOctetSyncIO->read(pController->pasynUser, inputBuff, inputSize, TIMEOUT, &nRead, &eomReason);
        if (status != asynSuccess)
            asynPrint(pController->pasynUser, ASYN_TRACE_ERROR,
                      "drvEnsembleAsyn:sendAndReceive error calling read, status=%d, error=%s\n",
                      status, pController->pasynUser->errorMessage);
    }
    epicsMutexUnlock(pController->sendReceiveMutex);
    return(status);
}

