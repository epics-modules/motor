/*
FILENAME... drvA3200Asyn.cc
USAGE...    Motor record asyn driver level support for Aerotech A3200.
 
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$ 
*/

/*
*      Original Author: Corey Bonnell
*      Date: 11/15/13
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
*      - 4.07.000
*
* Modification Log:
* -----------------
*
* .01 11-15-13 cjb Initialized from drvEnsembleAsyn.c (Aerotech)
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
#include "drvA3200Asyn.h"
#include "epicsExport.h"

/* NOTE: The following two files are copied from the A3200 C library include files.
* If changing the driver to target a different version of the A3200, copy the following two files from that version's C library include files */
#include "A3200CommonStructures.h"
#include "A3200ParameterId.h"

#define PRINT   (drv.print)
#define FLOW    motorAxisTraceFlow
#define TERROR   motorAxisTraceError
#define IODRIVER  motorAxisTraceIODriver

#define A3200_MAX_AXES 32
#define DRIVER_NAME "drvA3200Asyn"
#define BUFFER_SIZE 4096 /* Size of input and output buffers */
#define TIMEOUT 2.0     /* Timeout for I/O in seconds */

/* The following should be defined to have the same value as
the A3200 parameters specified */
#define ASCII_EOS_CHAR      '\n'  /* CommandTerminatingCharacter */
#define ASCII_EOS_STR       "\n"
#define ASCII_ACK_CHAR      '%'   /* CommandSuccessCharacter */
#define ASCII_NAK_CHAR      '!'   /* CommandInvalidCharacter */
#define ASCII_FAULT_CHAR    '#'   /* CommandFaultCharacter */

typedef union
{
    epicsUInt32 All;
    struct
    {
#ifdef MSB_First
        unsigned int na5                  :27;
        unsigned int EOTswitch            :1;
        unsigned int LIF481mode           :1;
        unsigned int CWEOTSWstate         :1;
        unsigned int CCWEOTSWstate        :1;
        unsigned int HomeSWstate          :1;
#else
        unsigned int HomeSWstate          :1;
        unsigned int CCWEOTSWstate        :1;
        unsigned int CWEOTSWstate         :1;
        unsigned int LIF481mode           :1;
        unsigned int EOTswitch            :1;
        unsigned int na5                  :27;
#endif
    } Bits;
} Switch_Level;

motorAxisDrvSET_t motorA3200 = 
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

extern "C"
{
    epicsExportAddress(drvet, motorA3200);
}

typedef struct
{
    epicsMutexId controllerLock;
    asynUser *pasynUser;
    int numAxes;
    double movingPollPeriod;
    double idlePollPeriod;
    epicsEventId pollEventId;
    epicsMutexId sendReceiveMutex;
    AXIS_HDL pAxis;  /* array of axes */
    epicsUInt32 taskNumber; /* the task number to use for motion commands */
} A3200Controller;

typedef struct motorAxisHandle
{
    A3200Controller *pController;
    PARAMS params;
    double currentCmdPos;
    double stepSize;
    double homePreset;
    int homeDirection;
    int closedLoop;
    int axisStatus;
    int card;
    int axis;
    char axisName[128];
    int maxDigits;
    motorAxisLogFunc print;
    void *logParam;
    epicsMutexId mutexId;
    Switch_Level swconfig;
    int lastFault;
    bool reverseDirec;
} motorAxis;

typedef struct
{
    AXIS_HDL pFirst;
    epicsThreadId motorThread;
    motorAxisLogFunc print;
    void *logParam;
    epicsTimeStamp now;
} motorA3200_t;

extern "C" { static int motorA3200LogMsg(void *, const motorAxisLogMask_t, const char *, ...); }
static asynStatus sendAndReceive(A3200Controller *, const char *, char *, size_t);

static motorA3200_t drv = { NULL, NULL, motorA3200LogMsg, 0, { 0, 0 } };
static int numA3200Controllers;
/* Pointer to array of controller structures */
static A3200Controller *pA3200Controller = NULL;

#define MAX(a, b) ((a)>(b) ? (a) : (b))
#define MIN(a, b) ((a)<(b) ? (a) : (b))

static void motorAxisReportAxis(AXIS_HDL pAxis, int level)
{
    if (level > 0)
    {
        printf("Axis %s\n", pAxis->axisName);
        printf("   axisStatus:  0x%x\n", pAxis->axisStatus);
        printf("   home preset: %f\n", pAxis->homePreset);
        printf("   step size:   %f\n", pAxis->stepSize);
        printf("   max digits:  %d\n", pAxis->maxDigits);
    }
}

static void motorAxisReport(int level)
{
    int i, j;

    for (i = 0; i < numA3200Controllers; i++)
    {
        if (level)
        {
            printf("    moving poll period: %f\n", pA3200Controller[i].movingPollPeriod);
            printf("    idle poll period: %f\n", pA3200Controller[i].idlePollPeriod);
        }
        for (j = 0; j < pA3200Controller[i].numAxes; j++)
        {
            motorAxisReportAxis(&pA3200Controller[i].pAxis[j], level);
        }
    }   
}


static int motorAxisInit(void)
{
    int controller, axis;

    for (controller = 0; controller < numA3200Controllers; controller++)
    {
        AXIS_HDL pAxis;
        for (axis = 0; axis < pA3200Controller[controller].numAxes; axis++)
        {
            pAxis = &pA3200Controller[controller].pAxis[axis];
            if (!pAxis->mutexId)
                break;
            epicsMutexLock(pAxis->mutexId);

            /*Set GAIN_SUPPORT on so that at least, CNEN functions. */
            motorParam->setInteger(pAxis->params, motorAxisHasClosedLoop, 1);

            motorParam->callCallback(pAxis->params);
            epicsMutexUnlock(pAxis->mutexId);
        }
    }
    return MOTOR_AXIS_OK;
}

static int motorAxisSetLog(AXIS_HDL pAxis, motorAxisLogFunc logFunc, void * param)
{
    if (pAxis == NULL)
    {
        if (logFunc == NULL)
        {
            drv.print =  motorA3200LogMsg;
            drv.logParam = NULL;
        }
        else
        {
            drv.print = logFunc;
            drv.logParam = param;
        }
    }
    else
    {
        if (logFunc == NULL)
        {
            pAxis->print = motorA3200LogMsg;
            pAxis->logParam = NULL;
        }
        else
        {
            pAxis->print = logFunc;
            pAxis->logParam = param;
        }
    }
    return MOTOR_AXIS_OK;
}

static AXIS_HDL motorAxisOpen(int card, int axis, char * param)
{
    AXIS_HDL pAxis;

    if (card >= numA3200Controllers)
        return NULL;
    if (axis >= pA3200Controller[card].numAxes)
        return NULL;

    pAxis = &pA3200Controller[card].pAxis[axis];
    return pAxis;
}

static int motorAxisClose(AXIS_HDL pAxis)
{
    return MOTOR_AXIS_OK;
}

static int motorAxisGetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int * value)
{
    if (pAxis == NULL || pAxis->params == NULL)
        return MOTOR_AXIS_ERROR;
    else
        return motorParam->getInteger(pAxis->params, (paramIndex) function, value);
}

static int motorAxisGetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double * value)
{
    if (pAxis == NULL || pAxis->params == NULL)
        return MOTOR_AXIS_ERROR;
    else
        return motorParam->getDouble(pAxis->params, (paramIndex) function, value);
}

static int motorAxisSetCallback(AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param)
{
    if (pAxis == NULL || pAxis->params == NULL)
        return MOTOR_AXIS_ERROR;
    else
        return motorParam->setCallback(pAxis->params, callback, param);
}

static int motorAxisSetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double value)
{
    asynStatus status = asynSuccess;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];

    if (pAxis == NULL || pAxis->pController == NULL)
        return asynError;

    epicsMutexLock(pAxis->mutexId);

    switch (function)
    {
        case motorAxisPosition:
            {
                double offset = value * fabs(pAxis->stepSize);
                sprintf(outputBuff, "POSOFFSET SET %s %.*f", pAxis->axisName, pAxis->maxDigits, offset);
                status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
                if (inputBuff[0] != ASCII_ACK_CHAR)
                    status = asynError;
                break;
            }
        case motorAxisEncoderRatio:
            {
                PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: A3200 does not support setting encoder ratio\n");
                break;
            }
        case motorAxisResolution:
            {
                /* we need to scale over a dozen other parameters if this changed in some cases, so the user should just use
                * the Configuration Manager to change this setting to ensure that this is done correctly */
                PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: A3200 does not support setting motor resolution\n");
                break;
            }
        case motorAxisLowLimit:
            {
                PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: Driver does not set A3200's Low Limit\n");
                break;
            }
        case motorAxisHighLimit:
            {
                PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: Driver does not set A3200's High Limit\n");
                break;
            }
        case motorAxisPGain:
            {
                PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: A3200 does not support setting proportional gain\n");
                break;
            }
        case motorAxisIGain:
            {
                PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: A3200 does not support setting integral gain\n");
                break;
            }
        case motorAxisDGain:
            {
                PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: A3200 does not support setting derivative gain\n");
                break;
            }
        default:
            PRINT(pAxis->logParam, TERROR, "motorAxisSetDouble: unknown function %d\n", function);
            break;
    }
    if (status == asynSuccess)
    {
        motorParam->setDouble(pAxis->params, function, value);
        motorParam->callCallback(pAxis->params);
    }
    epicsMutexUnlock(pAxis->mutexId);

    return status;
}

static int motorAxisSetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];

    if (pAxis == NULL || pAxis->pController == NULL)
        return MOTOR_AXIS_ERROR;

    epicsMutexLock(pAxis->mutexId);

    switch (function)
    {
        case motorAxisClosedLoop:
            if (value == 0)
                sprintf(outputBuff, "DISABLE %s", pAxis->axisName);
            else
            {
                if (pAxis->lastFault)
                {
                    sprintf(outputBuff, "FAULTACK %s", pAxis->axisName);
                    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
                }
                sprintf(outputBuff, "ENABLE %s", pAxis->axisName);
            }
            ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
            break;
        default:
            PRINT(pAxis->logParam, TERROR, "motorAxisSetInteger: unknown function %d\n", function);
            break;
    }
    if (ret_status != MOTOR_AXIS_ERROR)
    {
        motorParam->setInteger(pAxis->params, function, value);
        motorParam->callCallback(pAxis->params);
    }
    epicsMutexUnlock(pAxis->mutexId);
    return ret_status;
}


static int motorAxisMove(AXIS_HDL pAxis, double position, int relative,
                                                 double min_velocity, double max_velocity, double acceleration)
{
    int ret_status;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    const char *moveCommand;
    bool posdir;

    if (pAxis == NULL || pAxis->pController == NULL)
        return MOTOR_AXIS_ERROR;

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %s move to %f, min vel=%f, max_vel=%f, accel=%f\n",
          pAxis->card, pAxis->axisName, position, min_velocity, max_velocity, acceleration);

    if (relative)
    {
        posdir = position >= 0.0;
        moveCommand = "INCREMENTAL";
    }
    else
    {
        posdir = position >= pAxis->currentCmdPos;
        moveCommand = "ABSOLUTE";
    }

    ret_status = sendAndReceive(pAxis->pController, moveCommand, inputBuff, sizeof(inputBuff));
    if (ret_status)
        return MOTOR_AXIS_ERROR;

    if (acceleration > 0)
    { /* only use the acceleration if > 0 */
        sprintf(outputBuff, "RAMP RATE %.*f", pAxis->maxDigits, acceleration * fabs(pAxis->stepSize));
        ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    }

    sprintf(outputBuff, "LINEAR %s %.*f F%.*f", pAxis->axisName, pAxis->maxDigits, position * fabs(pAxis->stepSize),
            pAxis->maxDigits, max_velocity * fabs(pAxis->stepSize));

    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    if (ret_status)
        return MOTOR_AXIS_ERROR;

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

    return MOTOR_AXIS_OK;
}

static int motorAxisHome(AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards)
{
    int ret_status;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    epicsUInt32 hparam;
    int axis;

    if (pAxis == NULL || pAxis->pController == NULL)
        return MOTOR_AXIS_ERROR;

    axis = pAxis->axis;

    PRINT(pAxis->logParam, FLOW, "motorAxisHome: set card %d, axis %d to home, forwards = %d\n",
          pAxis->card, axis, forwards);

    if (max_velocity > 0)
    {
        sprintf(outputBuff, "HomeSpeed.%s = %.*f", pAxis->axisName, pAxis->maxDigits,
                max_velocity * fabs(pAxis->stepSize));
        ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    }

    if (acceleration > 0)
    {
        sprintf(outputBuff, "HomeRampRate.%s = %.*f", pAxis->axisName, pAxis->maxDigits,
                acceleration * fabs(pAxis->stepSize));
        ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    }
    hparam = pAxis->homeDirection;
    hparam = forwards ? 0x00000001 : 0x0;
    pAxis->homeDirection = hparam;

    sprintf(outputBuff, "HomeSetup.%s = %d", pAxis->axisName, hparam);
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));

    sprintf(outputBuff, "HOME %s", pAxis->axisName);
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

    return MOTOR_AXIS_OK;
}


static int motorAxisVelocityMove(AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration)
{
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    int ret_status;

    if (pAxis == NULL || pAxis->pController == NULL)
        return MOTOR_AXIS_ERROR;

    sprintf(outputBuff, "AbortDecelRate.%s = %.*f", pAxis->axisName, pAxis->maxDigits, acceleration * fabs(pAxis->stepSize));
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    sprintf(outputBuff, "RAMP RATE %s %.*f", pAxis->axisName, pAxis->maxDigits, acceleration * fabs(pAxis->stepSize));
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    sprintf(outputBuff, "FREERUN %s %.*f", pAxis->axisName, pAxis->maxDigits, velocity  * fabs(pAxis->stepSize));
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

    return ret_status;
}

static int motorAxisProfileMove(AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger)
{
    return MOTOR_AXIS_ERROR;
}

static int motorAxisTriggerProfile(AXIS_HDL pAxis)
{
    return MOTOR_AXIS_ERROR;
}

static int motorAxisStop(AXIS_HDL pAxis, double acceleration)
{
    int ret_status;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];

    if (pAxis == NULL || pAxis->pController == NULL)
        return MOTOR_AXIS_ERROR;

    PRINT(pAxis->logParam, FLOW, "Abort on card %d, axis %d\n", pAxis->card, pAxis->axis);

    /* we can't accurately determine which type of motion is occurring on the controller,
    * so don't worry about the acceleration rate, just stop the motion on the axis */
    sprintf(outputBuff, "ABORT %s", pAxis->axisName);
    ret_status = sendAndReceive(pAxis->pController, outputBuff, inputBuff, sizeof(inputBuff));
    return ret_status;
}

static int motorAxisforceCallback(AXIS_HDL pAxis)
{
    if (pAxis == NULL || pAxis->pController == NULL)
        return MOTOR_AXIS_ERROR;

    PRINT(pAxis->logParam, FLOW, "motorAxisforceCallback: request card %d, axis %d status update\n", pAxis->card, pAxis->axis);

    /* Force a status update. */
    motorParam->forceCallback(pAxis->params);

    /* Send a signal to the poller task which will make it do a status update */
    epicsEventSignal(pAxis->pController->pollEventId);
    return MOTOR_AXIS_OK;
}

static void A3200Poller(A3200Controller *pController)
{
    /* This is the task that polls the A3200 */
    double timeout;
    AXIS_HDL pAxis;
    int itera;
    bool anyMoving;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    const char* STATUS_FORMAT_STRING = "~STATUS (%s, AxisStatus) (%s, DriveStatus) (%s, AxisFault) (%s, ProgramPositionFeedback) (%s, ProgramPositionCommand) (%s, ProgramVelocityFeedback)";
    int status;
    int axis_status, drive_status, axis_fault;
    double pfbk, pcmd, vfbk;
    bool move_active;

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
            PARAMS params;

            pAxis = &pController->pAxis[itera];
            params = pAxis->params;

            if (!pAxis->mutexId)
                break;
            epicsMutexLock(pAxis->mutexId);
            sprintf(outputBuff, STATUS_FORMAT_STRING,
                    pAxis->axisName,
                    pAxis->axisName,
                    pAxis->axisName,
                    pAxis->axisName,
                    pAxis->axisName,
                    pAxis->axisName);

            status = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (status != asynSuccess)
            {
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
                epicsMutexUnlock(pAxis->mutexId);
                continue;
            }

            if (inputBuff[0] != ASCII_ACK_CHAR)
            {
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
                epicsMutexUnlock(pAxis->mutexId);
                continue;
            }

            sscanf(&inputBuff[1], "%d %d %d %lf %lf %lf", &axis_status, &drive_status, &axis_fault, &pfbk, &pcmd, &vfbk);

            motorParam->setInteger(params, motorAxisCommError, 0);

            move_active = drive_status & DRIVESTATUS_MoveActive;
            motorParam->setInteger(params, motorAxisDone, !move_active);
            if (move_active)
                anyMoving = true;

            motorParam->setInteger(pAxis->params, motorAxisPowerOn, (drive_status & DRIVESTATUS_Enabled) != 0);
            motorParam->setInteger(pAxis->params, motorAxisHomeSignal, (axis_status & AXISSTATUS_Homed) != 0);

            if (pAxis->reverseDirec == false)
            {
                motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, !((drive_status & DRIVESTATUS_CwEndOfTravelLimitInput) ^ pAxis->swconfig.Bits.CWEOTSWstate));
                motorParam->setInteger(pAxis->params, motorAxisLowHardLimit,  !((drive_status & DRIVESTATUS_CcwEndOfTravelLimitInput) ^ pAxis->swconfig.Bits.CCWEOTSWstate));
            }
            else
            {
                motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, !((drive_status & DRIVESTATUS_CcwEndOfTravelLimitInput) ^ pAxis->swconfig.Bits.CCWEOTSWstate));
                motorParam->setInteger(pAxis->params, motorAxisLowHardLimit,  !((drive_status & DRIVESTATUS_CwEndOfTravelLimitInput) ^ pAxis->swconfig.Bits.CWEOTSWstate));
            }
            pAxis->axisStatus = axis_status;

            pfbk /= fabs(pAxis->stepSize);
            motorParam->setDouble(pAxis->params, motorAxisEncoderPosn, pfbk);

            pcmd /= fabs(pAxis->stepSize);
            motorParam->setDouble(pAxis->params, motorAxisPosition, pcmd);
            pAxis->currentCmdPos = pcmd;

            PRINT(pAxis->logParam, IODRIVER, "A3200Poller: axis %s axisStatus=%x, position=%f\n",
                  pAxis->axisName, pAxis->axisStatus, pAxis->currentCmdPos);

            if (axis_fault && axis_fault != pAxis->lastFault)
            {
                PRINT(pAxis->logParam, TERROR, "A3200Poller: controller fault on axis=%s fault=0x%X\n", pAxis->axisName, axis_fault);
            }

            pAxis->lastFault = axis_fault;

            vfbk /= fabs(pAxis->stepSize);
            motorParam->setDouble(pAxis->params, motorAxisActualVel, vfbk);

            motorParam->callCallback(pAxis->params);
            epicsMutexUnlock(pAxis->mutexId);
        } /* Next axis */

        timeout = anyMoving ? pController->movingPollPeriod : pController->idlePollPeriod;
    } /* End while */
}

static int motorA3200LogMsg(void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{
    va_list pvar;
    int nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout, pFormat, pvar);
    va_end(pvar);
    printf("\n");
    return nchar;
}

int A3200AsynSetup(int num_controllers)   /* number of A3200 controllers in system.  */
{
    
    if (num_controllers < 1)
    {
        printf("A3200AsynSetup, num_controllers must be > 0\n");
        return MOTOR_AXIS_ERROR;
    }
    numA3200Controllers = num_controllers;
    pA3200Controller = (A3200Controller *)calloc(numA3200Controllers, sizeof(A3200Controller));
    if (pA3200Controller == NULL)
    {
        printf("A3200AsynSetup, could not allocate memory\n");
        return MOTOR_AXIS_ERROR;
    }
    return MOTOR_AXIS_OK;
}

int A3200AsynConfig(int card,             /* Controller number */
                                        const char *portName, /* asyn port name of serial or GPIB port */
                                        int asynAddress,      /* asyn subaddress for GPIB */
                                        int numAxes,         /* The number of axes that the driver controls */
                                        int taskNumber,       /* the task number to use for motion commands */
                                        int movingPollPeriod, /* Time to poll (msec) when an axis is in motion */
                                        int idlePollPeriod)   /* Time to poll (msec) when an axis is idle. 0 for no polling */

{
    A3200Controller *pController;
    char threadName[20];
    int axis, status, digits, retry = 0;
    char inputBuff[BUFFER_SIZE], outputBuff[BUFFER_SIZE];
    const char* GET_PARAM_FORMAT_STRING  = "%s.%s";

    if (numA3200Controllers < 1)
    {
        printf("A3200AsynConfig: no A3200 controllers allocated, call A3200 first\n");
        return MOTOR_AXIS_ERROR;
    }
    if ((card < 0) || (card >= numA3200Controllers))
    {
        printf("A3200AsynConfig: card must in range 0 to %d\n", numA3200Controllers - 1);
        return MOTOR_AXIS_ERROR;
    }

    if (numAxes < 1 || numAxes > A3200_MAX_AXES)
    {
        printf("A3200AsynConfig: numAxes must be in the range of 1 to %u\n", A3200_MAX_AXES);
        return MOTOR_AXIS_ERROR;
    }

    pController = &pA3200Controller[card];

    pController->numAxes = numAxes;
    pController->taskNumber = taskNumber;
    pController->movingPollPeriod = movingPollPeriod / 1000.;
    pController->idlePollPeriod = idlePollPeriod / 1000.;

    pController->sendReceiveMutex = epicsMutexMustCreate();

    status = pasynOctetSyncIO->connect(portName, asynAddress, &pController->pasynUser, NULL);

    if (status != asynSuccess)
    {
        printf("A3200AsynConfig: cannot connect to asyn port %s\n", portName);
        return MOTOR_AXIS_ERROR;
    }

    /* Set command End-of-string */
    pasynOctetSyncIO->setInputEos(pController->pasynUser, ASCII_EOS_STR, strlen(ASCII_EOS_STR));
    pasynOctetSyncIO->setOutputEos(pController->pasynUser, ASCII_EOS_STR, strlen(ASCII_EOS_STR));

    pController->pAxis = (AXIS_HDL) calloc(numAxes, sizeof(motorAxis));
    if (pA3200Controller->pAxis == NULL)
    {
        printf("A3200AsynConfig, could not allocate memory\n");
        return MOTOR_AXIS_ERROR;
    }

    retry = 0;

    do
    {
        sprintf(outputBuff, "~TASK %u", pController->taskNumber);
        status = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
        retry++;
    } while(status != asynSuccess && retry < 3);

    if (status != asynSuccess)
        return MOTOR_AXIS_ERROR;

    sendAndReceive(pController, "~STOPTASK", inputBuff, sizeof(inputBuff)); // reset the task

    /* Get axes info */
    for (axis = 0; axis < numAxes; axis++)
    {
        sprintf(outputBuff, "$strtask0 = GETPARMSTRING %d, PARAMETERID_AxisName", axis);
        sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));

        sendAndReceive(pController, "~GETVARIABLE $strtask0", inputBuff, sizeof(inputBuff));
        if (inputBuff[0] == ASCII_ACK_CHAR)
        {
            AXIS_HDL pAxis = &pController->pAxis[axis];
            pAxis->pController = pController;
            pAxis->card = card;
            pAxis->axis = axis;
            pAxis->mutexId = epicsMutexMustCreate();
            pAxis->params = motorParam->create(0, MOTOR_AXIS_NUM_PARAMS);
            strncpy(pAxis->axisName, &inputBuff[1], sizeof(pAxis->axisName) - 1);

            sprintf(outputBuff, GET_PARAM_FORMAT_STRING, "PositionFeedbackType", pAxis->axisName);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR && atoi(&inputBuff[1]) > 0)
            {
                pAxis->closedLoop = 1;
                motorParam->setInteger(pAxis->params, motorAxisHasEncoder, 1);
            }

            sprintf(outputBuff, GET_PARAM_FORMAT_STRING, "CountsPerUnit", pAxis->axisName);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->stepSize = 1 / atof(&inputBuff[1]);
            else
                pAxis->stepSize = 1;
            digits = (int) -log10(fabs(pAxis->stepSize)) + 2;
            pAxis->maxDigits = digits < 1 ? 1 : digits;

            sprintf(outputBuff, GET_PARAM_FORMAT_STRING, "HomeOffset", pAxis->axisName);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->homePreset = atof(&inputBuff[1]);

            sprintf(outputBuff,  GET_PARAM_FORMAT_STRING, "HomeSetup", pAxis->axisName);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->homeDirection = atoi(&inputBuff[1]) & 0x1;

            sprintf(outputBuff,  GET_PARAM_FORMAT_STRING, "EndOfTravelLimitSetup", pAxis->axisName);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->swconfig.All = atoi(&inputBuff[1]);

            /* Set RAMP MODE to RATE. */
            sprintf(outputBuff, "RAMP MODE RATE %s", pAxis->axisName);
            sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));

            /* Get Reverse Direction indicator. */
            sprintf(outputBuff,  GET_PARAM_FORMAT_STRING, "ReverseMotionDirection", pAxis->axisName);
            if (inputBuff[0] == ASCII_ACK_CHAR)
                pAxis->reverseDirec = (bool) atoi(&inputBuff[1]);
        }
    }

    sendAndReceive(pController, "~INITQUEUE", inputBuff, sizeof(inputBuff));

    pController->pollEventId = epicsEventMustCreate(epicsEventEmpty);

    /* Create the poller thread for this controller */
    epicsSnprintf(threadName, sizeof(threadName), "A3200:%d", card);
    epicsThreadCreate(threadName, epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) A3200Poller, (void *) pController);

    return MOTOR_AXIS_OK;
}

static asynStatus sendAndReceive(A3200Controller *pController, const char *outputBuff, char *inputBuff, size_t inputSize) 
{
    size_t nWriteRequested;
    size_t nWrite, nRead;
    int eomReason;
    asynStatus status;
    AXIS_HDL pAxis;

    if (pController == NULL)
        return asynError;

    pAxis = &pController->pAxis[0];
    nWriteRequested = strlen(outputBuff);

    /* sendAndReceive is intended only for "fast" read-write operations (such as getting parameter/status values),
    * so we don't expect much latency on read/writes */
    epicsMutexLock(pController->sendReceiveMutex);

    status = pasynOctetSyncIO->writeRead(pController->pasynUser, outputBuff, nWriteRequested,
                                         inputBuff, inputSize, TIMEOUT, &nWrite, &nRead, &eomReason);

    if (nWrite != nWriteRequested)
        status = asynError;
    else if (status == asynTimeout)
    {
        int retry = 1;

        while (retry <= 3 && status == asynTimeout)
        {
            PRINT(pAxis->logParam, TERROR, "%s:sendAndReceive: Retrying read, retry# = %d.\n", DRIVER_NAME, retry);
            status = pasynOctetSyncIO->read(pController->pasynUser, inputBuff, inputSize, TIMEOUT, &nRead, &eomReason);
            retry++;
        }
        if (retry > 3)
            PRINT(pAxis->logParam, TERROR,
                  "%s:sendAndReceive: Retries exhausted on response to command = %s.\n", DRIVER_NAME, outputBuff);
        else
            PRINT(pAxis->logParam, TERROR,
                  "%s:sendAndReceive: Retry succeeded for command = %s with response = %s\n", DRIVER_NAME, outputBuff, inputBuff);
    }

    if (status != asynSuccess)
        asynPrint(pController->pasynUser, ASYN_TRACE_ERROR,
                  "%s:sendAndReceive writeRead error, output=%s status=%d, error=%s\n", DRIVER_NAME,
                  outputBuff, status, pController->pasynUser->errorMessage);
    else
    {
        /* read until we have an ACK followed by a string (most likely will be the numeric value we're looking for) */
        while (status == asynSuccess && nRead > 1 && inputBuff[0] == ASCII_ACK_CHAR && inputBuff[1] == ASCII_EOS_CHAR)
            status = pasynOctetSyncIO->read(pController->pasynUser, inputBuff, inputSize, TIMEOUT, &nRead, &eomReason);
        if (status != asynSuccess)
            asynPrint(pController->pasynUser, ASYN_TRACE_ERROR,
                      "%s:sendAndReceive error calling read, status=%d, error=%s\n", DRIVER_NAME,
                      status, pController->pasynUser->errorMessage);
    }
    epicsMutexUnlock(pController->sendReceiveMutex);
    return status;
}

