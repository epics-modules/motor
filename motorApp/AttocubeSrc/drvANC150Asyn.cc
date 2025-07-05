/*
FILENAME...     drvANC150Asyn.cc
USAGE...        asyn motor driver support for attocube systems AG ANC150
                Piezo Step Controller.


*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 07/24/2008
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01 11-09-07 rls copied from drvMM4000Asyn.cc
 * .02 10-29-08 rls - added frequency and step mode to polling data.
 *                  - support for CNEN field.
 *                  - simulate trajectory positon so RDBL link updates.
 *                  - allow zero moves so motor record does not get stuck.
 * .03 11-19-08 rls - change input EOS to prompt ">".
 *                  - support enable/disable "torque".
 *                  - zero move bug fix.
 *                  - set firmwareVersion.
 * .04 02-18-09 rls Copied Matthew Pearson's (Diamond) fix on XPS for;
 *                  - idle polling interfering with setting position.
 *                  - auto save/restore not working.
 * .05 06-11-09 rls - Matthew Pearson's fix for record seeing motorAxisDone True
 *                  on 1st status update after a move; set motorAxisDone False
 *                  in motorAxisDrvSET_t functions that start motion
 *                  (motorAxisHome, motorAxisMove, motorAxisVelocityMove) and
 *                  force a status update with a call to callCallback().
 * .06 08-07-09 rls - bug fix for multi-axis not reading frequency.
 *                  - bug fix for set position not setting val/dval/rval.
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "epicsThread.h"
#include "epicsEvent.h"
#include "epicsMutex.h"
#include "epicsTime.h"
#include "epicsTimer.h"
#include "epicsString.h"
#include "iocsh.h"

#include "drvSup.h"
#include "asynOctetSyncIO.h"
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"
#include "paramLib.h"
#include "epicsExport.h"

/* End-of-string defines */
#define ANC150_OUT_EOS   "\r\n" /* Command */
#define ANC150_IN_EOS    "> "   /* Reply   */

#define NINT(f)	(long)((f)>0 ? (f)+0.5 : (f)-0.5)	/* Nearest integer. */

motorAxisDrvSET_t motorANC150 =
{
    14,
    motorAxisReport,        /**< Standard EPICS driver report function (optional) */
    motorAxisInit,          /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLog,        /**< Defines an external logging function (optional) */
    motorAxisOpen,          /**< Driver open function */
    motorAxisClose,         /**< Driver close function */
    motorAxisSetCallback,   /**< Provides a callback function the driver can call when the status updates */
    motorAxisSetDouble,     /**< Pointer to function to set a double value */
    motorAxisSetInteger,    /**< Pointer to function to set an integer value */
    motorAxisGetDouble,     /**< Pointer to function to get a double value */
    motorAxisGetInteger,    /**< Pointer to function to get an integer value */
    motorAxisHome,          /**< Pointer to function to execute a more to reference or home */
    motorAxisMove,          /**< Pointer to function to execute a position move */
    motorAxisVelocityMove,  /**< Pointer to function to execute a velocity mode move */
    motorAxisStop,          /**< Pointer to function to stop motion */
    motorAxisforceCallback  /**< Pointer to function to request a poller status update */
};

extern "C" {epicsExportAddress(drvet, motorANC150);}

typedef struct
{
    asynUser *pasynUser;
    int numAxes;
    char firmwareVersion[100];
    double movingPollPeriod;
    double idlePollPeriod;
    epicsEventId pollEventId;
    AXIS_HDL pAxis;             /* array of axes */
} ANC150Controller;

typedef struct motorAxisHandle
{
    ANC150Controller *pController;
    PARAMS params;
    double targetPosition;
    double currentPosition;
    double highLimit;
    double lowLimit;
    double homePreset;
    int axisStatus;
    int card;
    int axis;
    int maxDigits;
    motorAxisLogFunc print;
    void *logParam;
    bool moving_ind;        /* Moving indicator. */
    epicsMutexId mutexId;
    epicsTime *movetimer;   /* Moving timer. */
    double moveinterval;    /* Moving delta time (sec). */
    int frequency;
} motorAxis;

typedef struct
{
    AXIS_HDL pFirst;
    epicsThreadId motorThread;
    motorAxisLogFunc print;
    void *logParam;
    epicsTimeStamp now;
} motorANC150_t;

extern "C" {
static int motorANC150LogMsg(void *, const motorAxisLogMask_t, const char *, ...);
}
static int sendOnly(ANC150Controller *, char *);
static asynStatus sendAndReceive(ANC150Controller *, char *, char *, int);
static asynStatus getFreq(ANC150Controller *, int);
static bool stpMode(ANC150Controller *, int);

#define PRINT   (drv.print)
#define FLOW    motorAxisTraceFlow
#define IODRIVER  motorAxisTraceIODriver

#define ANC150_MAX_AXES 6
#define BUFFER_SIZE 100         /* Size of input and output buffers */
#define TIMEOUT 2.0             /* Timeout for I/O in seconds */

#define ANC150_HOME       0x20  /* Home LS. */
#define ANC150_LOW_LIMIT  0x10  /* Minus Travel Limit. */
#define ANC150_HIGH_LIMIT 0x08  /* Plus Travel Limit. */
#define ANC150_DIRECTION  0x04  /* Motor direction: 0 - minus; 1 - plus. */
#define ANC150_POWER_ON   0x02  /* Motor power 0 - ON; 1 - OFF. */
#define ANC150_MOVING     0x01  /* In-motion indicator. */


#define TCP_TIMEOUT 2.0
static motorANC150_t drv = {NULL, NULL, motorANC150LogMsg, 0, {0, 0}};
static int numANC150Controllers;
/* Pointer to array of controller strutures */
static ANC150Controller *pANC150Controller = NULL;

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

static void motorAxisReportAxis(AXIS_HDL pAxis, int level)
{
    if (level > 0)
    {
        printf("Axis %d\n", pAxis->axis);
        printf("   axisStatus:  0x%x\n", pAxis->axisStatus);
        printf("   high limit:  %f\n", pAxis->highLimit);
        printf("   low limit:   %f\n", pAxis->lowLimit);
        printf("   home preset: %f\n", pAxis->homePreset);
        printf("   max digits:  %d\n", pAxis->maxDigits);
    }
}

static void motorAxisReport(int level)
{
    int i, j;

    for (i = 0; i < numANC150Controllers; i++)
    {
        printf("Controller %d firmware version: %s\n", i, pANC150Controller[i].firmwareVersion);
        if (level)
        {
            printf("    model: attocube ANC 150\n");
            printf("    moving poll period: %f\n", pANC150Controller[i].movingPollPeriod);
            printf("    idle poll period: %f\n", pANC150Controller[i].idlePollPeriod);
            printf("Controller %d firmware version: %s\n", i, pANC150Controller[i].firmwareVersion);
        }
        for (j = 0; j < pANC150Controller[i].numAxes; j++)
        {
            motorAxisReportAxis(&pANC150Controller[i].pAxis[j], level);
        }
    }
}


static int motorAxisInit(void)
{
    return(MOTOR_AXIS_OK);
}

static int motorAxisSetLog(AXIS_HDL pAxis, motorAxisLogFunc logFunc, void *param)
{
    if (pAxis == NULL)
    {
        if (logFunc == NULL)
        {
            drv.print = motorANC150LogMsg;
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
            pAxis->print = motorANC150LogMsg;
            pAxis->logParam = NULL;
        }
        else
        {
            pAxis->print = logFunc;
            pAxis->logParam = param;
        }
    }
    return(MOTOR_AXIS_OK);
}

static AXIS_HDL motorAxisOpen(int card, int axis, char *param)
{
    AXIS_HDL pAxis;

    if (card > numANC150Controllers)
        return(NULL);
    if (axis > pANC150Controller[card].numAxes)
        return(NULL);
    pAxis = &pANC150Controller[card].pAxis[axis];
    return(pAxis);
}

static int motorAxisClose(AXIS_HDL pAxis)
{
    return(MOTOR_AXIS_OK);
}

static int motorAxisGetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int *value)
{
    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);
    else
    {
        return(motorParam->getInteger(pAxis->params, (paramIndex) function, value));
    }
}

static int motorAxisGetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double *value)
{
    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);
    else
    {
        return(motorParam->getDouble(pAxis->params, (paramIndex) function, value));
    }
}

static int motorAxisSetCallback(AXIS_HDL pAxis, motorAxisCallbackFunc callback, void *param)
{
    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);
    else
    {
        return(motorParam->setCallback(pAxis->params, callback, param));
    }
}

static int motorAxisSetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double value)
{
    int ret_status = MOTOR_AXIS_ERROR;

    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);
    else
    {
        epicsMutexLock(pAxis->mutexId);
        switch (function)
        {
        case motorAxisPosition:
        {
            pAxis->currentPosition = pAxis->targetPosition = value;
            ret_status = MOTOR_AXIS_OK;
            break;
        }
        case motorAxisEncoderRatio:
        {
            PRINT(pAxis->logParam, motorAxisTraceError,
                  "motorAxisSetDouble: ANC150 does not support setting encoder ratio\n");
            break;
        }
        case motorAxisResolution:
        {
            PRINT(pAxis->logParam, motorAxisTraceError,
                  "motorAxisSetDouble: ANC150 does not support setting resolution\n");
            break;
        }
        case motorAxisLowLimit:
        case motorAxisHighLimit:
            break;
        case motorAxisPGain:
        {
            PRINT(pAxis->logParam, motorAxisTraceError,
                  "ANC150 does not support setting proportional gain\n");
            break;
        }
        case motorAxisIGain:
        {
            PRINT(pAxis->logParam, motorAxisTraceError,
                  "ANC150 does not support setting integral gain\n");
            break;
        }
        case motorAxisDGain:
        {
            PRINT(pAxis->logParam, motorAxisTraceError,
                  "ANC150 does not support setting derivative gain\n");
            break;
        }
        default:
            PRINT(pAxis->logParam, motorAxisTraceError,
                  "motorAxisSetDouble: unknown function %d\n", function);
            break;
        }
        if (ret_status == MOTOR_AXIS_OK )
        {
            motorParam->setDouble(pAxis->params, function, value);
            motorParam->callCallback(pAxis->params);
        }
        epicsMutexUnlock(pAxis->mutexId);
    }
    return(ret_status);
}

static int motorAxisSetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    int status;
    char buff[20];

    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);

    switch (function)
    {
    case motorAxisClosedLoop:
        if (value == 0.0)
            sprintf(buff, "setm %d gnd", pAxis->axis + 1);
        else
            sprintf(buff, "setm %d stp", pAxis->axis + 1);

        ret_status = sendOnly(pAxis->pController, buff);
        break;

    default:
        PRINT(pAxis->logParam, motorAxisTraceError,
              "motorAxisSetInteger: unknown function %d\n", function);
        break;
    }
    if (ret_status != MOTOR_AXIS_ERROR)
        status = motorParam->setInteger(pAxis->params, function, value);
    return(ret_status);
}


static int motorAxisMove(AXIS_HDL pAxis, double position, int relative,
                         double min_velocity, double max_velocity, double acceleration)
{
    int status;
    long imove;
    char buff[100];
    const char *moveCommand;
    bool posdir;
    double fmove, ffrequency;

    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);

    PRINT(pAxis->logParam, FLOW,
          "Set card %d, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f\n",
          pAxis->card, pAxis->axis, position, min_velocity, max_velocity, acceleration);

    if (relative)
    {
        if (position >= 0.0)
            posdir = true;
        else
            posdir = false;
        pAxis->targetPosition += position;
        imove = NINT(position);
    }
    else
    {
        imove = NINT(position - pAxis->currentPosition);
        if (imove >= 0)
            posdir = true;
        else
            posdir = false;
        pAxis->targetPosition = position;
    }
    if (posdir == true)
        moveCommand = "stepu";
    else
        moveCommand = "stepd";

    pAxis->moving_ind = true;
    imove = abs(imove);

    fmove = (double) imove;
    ffrequency = (double) pAxis->frequency;
    pAxis->moveinterval = fmove / ffrequency;
    if (pAxis->moveinterval <= 0.0)
        pAxis->moveinterval = epicsThreadSleepQuantum();
    *pAxis->movetimer = epicsTime::getCurrent() + pAxis->moveinterval;

    sprintf(buff, "%s %d %ld", moveCommand, pAxis->axis + 1, imove);
    status = sendOnly(pAxis->pController, buff);
    if (status)
        return(MOTOR_AXIS_ERROR);

    if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK)
    {
        /* Set direction indicator. */
        motorParam->setInteger(pAxis->params, motorAxisDirection, posdir);

        /* Insure that the motor record's next status update sees motorAxisDone = False. */
        motorParam->setInteger(pAxis->params, motorAxisDone, 0);
        motorParam->callCallback(pAxis->params);
        epicsMutexUnlock(pAxis->mutexId);
    }

    /* Send a signal to the poller task which will make it do a poll, and
       switch to the moving poll rate. */
    epicsEventSignal(pAxis->pController->pollEventId);

    return(MOTOR_AXIS_OK);
}

static int motorAxisHome(AXIS_HDL pAxis, double min_velocity,
                         double max_velocity, double acceleration, int forwards)
{
    return(MOTOR_AXIS_ERROR);
}


static int motorAxisVelocityMove(AXIS_HDL pAxis, double min_velocity,
                                 double velocity, double acceleration)
{
    int status;

    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);

    /*
     * ANC150 does not have a jog command.  Simulate with move absolute to the
     * appropriate software limit. We can move to ANC150 soft limits. If the
     * record soft limits are set tighter than the ANC150 limits the record
     * will prevent JOG motion beyond its soft limits
     */
    if (velocity > 0.)
        status = motorAxisMove(pAxis, pAxis->highLimit, 0, min_velocity,
                               velocity, acceleration);
    else
        status = motorAxisMove(pAxis, pAxis->lowLimit, 0, min_velocity,
                               -velocity, acceleration);

    return(status);
}

static int motorAxisProfileMove(AXIS_HDL pAxis, int npoints, double positions[],
                                double times[], int relative, int trigger)
{
    return(MOTOR_AXIS_ERROR);
}

static int motorAxisTriggerProfile(AXIS_HDL pAxis)
{
    return(MOTOR_AXIS_ERROR);
}

static int motorAxisStop(AXIS_HDL pAxis, double acceleration)
{
    int status;
    char buff[100];

    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d to stop with accel=%f\n",
          pAxis->card, pAxis->axis, acceleration);

    sprintf(buff, "stop %d", pAxis->axis + 1);
    status = sendOnly(pAxis->pController, buff);
    if (status)
        return(MOTOR_AXIS_ERROR);
    
    /* Reset timer; indicating move is done. */
    *pAxis->movetimer = epicsTime::getCurrent();

    return(MOTOR_AXIS_OK);
}

static int motorAxisforceCallback(AXIS_HDL pAxis)
{
    if (pAxis == NULL)
        return(MOTOR_AXIS_ERROR);

    PRINT(pAxis->logParam, FLOW,
          "motorAxisforceCallback: request card %d, axis %d status update\n",
          pAxis->card, pAxis->axis);

    /* Force a status update. */
    motorParam->forceCallback(pAxis->params);

    /* Send a signal to the poller task which will make it do a status update */
    epicsEventSignal(pAxis->pController->pollEventId);
    return(MOTOR_AXIS_OK);
}


static void ANC150Poller(ANC150Controller *pController)
{
    /* This is the task that polls the ANC150 */
    double timeout;
    AXIS_HDL pAxis;
    asynStatus astatus;
    int status;
    int itera;
    int axisDone;
    int anyMoving;
    int forcedFastPolls = 0;

    timeout = pController->idlePollPeriod;
    epicsEventSignal(pController->pollEventId); /* Force on poll at startup */

    while (1)
    {
        if (timeout != 0.)
            status = epicsEventWaitWithTimeout(pController->pollEventId, timeout);
        else
            status = epicsEventWait(pController->pollEventId);

        if (status == epicsEventWaitOK)
        {
            /*
             * We got an event, rather than a timeout.  This is because other
             * software knows that an axis should have changed state (started
             * moving, etc.). Force a minimum number of fast polls, because the
             * controller status might not have changed the first few polls
             */
            forcedFastPolls = 0;
        }

        anyMoving = 0;
        for (itera = 0; itera < pController->numAxes; itera++)
        {
            double slewposition, proportion;

            pAxis = &pController->pAxis[itera];
            if (!pAxis->mutexId)
                break;
            epicsMutexLock(pAxis->mutexId);
            
            if (pAxis->moving_ind == true)
            {
                double time_remain = *pAxis->movetimer - epicsTime::getCurrent();
                double delta = pAxis->targetPosition - pAxis->currentPosition;

                axisDone = 0;
                anyMoving = 1;
                if (time_remain < 0.0)
                {
                    pAxis->moving_ind = false;
                    slewposition = pAxis->currentPosition = pAxis->targetPosition;
                }
                else
                {
                    proportion = 1.0 - (time_remain / pAxis->moveinterval);
                    slewposition = pAxis->currentPosition + (delta * proportion);
                }
            }
            else
            {
                axisDone = 1;
                slewposition = pAxis->currentPosition = pAxis->targetPosition;
            }

            motorParam->setInteger(pAxis->params, motorAxisDone, axisDone);
            if (pAxis->axisStatus & ANC150_HOME)
                motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 0);
            if (pAxis->axisStatus & ANC150_HIGH_LIMIT)
                motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 0);
            if (pAxis->axisStatus & ANC150_LOW_LIMIT)
                motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 0);

            motorParam->setDouble(pAxis->params, motorAxisPosition, slewposition);
            motorParam->setDouble(pAxis->params, motorAxisEncoderPosn, slewposition);
            PRINT(pAxis->logParam, IODRIVER, "ANC150Poller: axis %d axisStatus=%x, position=%f\n",
                  pAxis->axis, pAxis->axisStatus, slewposition);

            /* Update frequency. */
            astatus = getFreq(pController, itera);
            if (astatus == asynSuccess)
                motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
            else
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);

            /* Check power on. */
            if (stpMode(pController, itera))
                motorParam->setInteger(pAxis->params, motorAxisPowerOn, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisPowerOn, 0);

            motorParam->callCallback(pAxis->params);
            epicsMutexUnlock(pAxis->mutexId);

        }           /* Next axis */

        if (forcedFastPolls > 0)
        {
            timeout = pController->movingPollPeriod;
            forcedFastPolls--;
        }
        else if (anyMoving)
            timeout = pController->movingPollPeriod;
        else
            timeout = pController->idlePollPeriod;
    }
}

static int motorANC150LogMsg(void *param, const motorAxisLogMask_t mask,
                             const char *pFormat,...)
{

    va_list pvar;
    int nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout, pFormat, pvar);
    va_end(pvar);
    printf("\n");
    return(nchar);
}


int ANC150AsynSetup(int num_controllers)    /* number of ANC150 controllers in system.  */
{

    if (num_controllers < 1)
    {
        printf("ANC150Setup, num_controllers must be > 0\n");
        return(MOTOR_AXIS_ERROR);
    }
    numANC150Controllers = num_controllers;
    pANC150Controller = (ANC150Controller *) calloc(numANC150Controllers,
                                                    sizeof(ANC150Controller));
    return(MOTOR_AXIS_OK);
}


int ANC150AsynConfig(int card,              /* Controller number */
                     const char *portName,  /* asyn serial port name */
                     int numAxes,           /* Number of axes this controller supports */
                     int movingPollPeriod,  /* Time to poll (msec) when an axis is in motion */
                     int idlePollPeriod)    /* Time to poll (msec) when an axis is idle. 0 for no polling */

{
    AXIS_HDL pAxis;
    int axis;
    ANC150Controller *pController;
    char threadName[20];
    int status;
    int retry = 0;
    char inputBuff[BUFFER_SIZE];
    char outputBuff[BUFFER_SIZE];

    if (numANC150Controllers < 1)
    {
        printf("ANC150Config: no ANC150 controllers allocated, call ANC150Setup first\n");
        return(MOTOR_AXIS_ERROR);
    }
    if ((card < 0) || (card >= numANC150Controllers))
    {
        printf("ANC150Config: card must in range 0 to %d\n", numANC150Controllers - 1);
        return(MOTOR_AXIS_ERROR);
    }
    if ((numAxes < 1) || (numAxes > ANC150_MAX_AXES))
    {
        printf("ANC150Config: numAxes must in range 1 to %d\n", ANC150_MAX_AXES);
        return(MOTOR_AXIS_ERROR);
    }

    pController = &pANC150Controller[card];
    pController->numAxes = numAxes;
    pController->movingPollPeriod = movingPollPeriod / 1000.;
    pController->idlePollPeriod = idlePollPeriod / 1000.;

    status = pasynOctetSyncIO->connect(portName, 0, &pController->pasynUser, NULL);

    if (status != asynSuccess)
    {
        printf("ANC150AsynConfig: cannot connect to asyn port %s\n", portName);
        return(MOTOR_AXIS_ERROR);
    }

    /* Set command End-of-string */
    pasynOctetSyncIO->setInputEos(pController->pasynUser,  ANC150_IN_EOS,  strlen(ANC150_IN_EOS));
    pasynOctetSyncIO->setOutputEos(pController->pasynUser, ANC150_OUT_EOS, strlen(ANC150_OUT_EOS));

    do
    {
        pasynOctetSyncIO->flush(pController->pasynUser);
        status = sendAndReceive(pController, (char *) "ver", inputBuff, sizeof(inputBuff));
        if (status == asynSuccess && strncmp(inputBuff, "attocube", 8) == 0)
            strncpy(pController->firmwareVersion, &inputBuff[19], sizeof(inputBuff));
        else
            status = asynError;
        retry++;
        /* Return value is length of response string */
    } while (status != asynSuccess && retry < 3);

    if (status != asynSuccess)
        return(MOTOR_AXIS_ERROR);


    /* Don't initialize pAxis until all the error checks have passed;
     * prevents drvAsynMotorConfigure from crashing. */
    pController->pAxis = (AXIS_HDL) calloc(numAxes, sizeof(motorAxis));

    for (axis = 0; axis < numAxes; axis++)
    {
        pAxis = &pController->pAxis[axis];
        pAxis->pController = pController;
        pAxis->card = card;
        pAxis->axis = axis;
        pAxis->mutexId = epicsMutexMustCreate();
        pAxis->params = motorParam->create(0, MOTOR_AXIS_NUM_PARAMS);
        motorParam->setInteger(pAxis->params, motorAxisClosedLoop, 1);
        /* Set motorAxisHasClosedLoop on so the CNEN field works. */
        motorParam->setInteger(pAxis->params, motorAxisHasClosedLoop, 1);
        pAxis->currentPosition = 0.0;
        pAxis->movetimer = new epicsTime();
        pAxis->moving_ind = false;
        getFreq(pController, axis);
        sprintf(outputBuff, "setm %d stp", pAxis->axis + 1);
        status = sendOnly(pAxis->pController, outputBuff);
    }

    pController->pollEventId = epicsEventMustCreate(epicsEventEmpty);

    /* Create the poller thread for this controller */
    epicsSnprintf(threadName, sizeof(threadName), "ANC150:%d", card);
    epicsThreadCreate(threadName,
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) ANC150Poller, (void *) pController);

    return(MOTOR_AXIS_OK);
}

static int sendOnly(ANC150Controller * pController, char *outputBuff)
{
    char inputBuff[BUFFER_SIZE];
    int nRequested = strlen(outputBuff);
    size_t nActual, nRead;
    asynStatus status;
    int eomReason;

    status = pasynOctetSyncIO->writeRead(pController->pasynUser, outputBuff, nRequested,
                                         inputBuff, sizeof(inputBuff), TIMEOUT, &nActual,
                                         &nRead, &eomReason);
    
    if (nActual != (size_t) nRequested)
        status = asynError;

    if (status != asynSuccess)
    {
        asynPrint(pController->pasynUser, ASYN_TRACE_ERROR,
                  "drvANC150Asyn:sendOnly: error sending command %d, sent=%d, status=%d\n",
                  outputBuff, nActual, status);
    }

    return(status);
}


static asynStatus sendAndReceive(ANC150Controller *pController, char *outputBuff,
                                 char *inputBuff, int inputSize)
{
    int nWriteRequested = strlen(outputBuff);
    size_t nWrite, nRead;
    int eomReason;
    asynStatus status;
    char localbuf[BUFFER_SIZE];

    status = pasynOctetSyncIO->writeRead(pController->pasynUser, outputBuff,
                                         nWriteRequested, localbuf, BUFFER_SIZE,
                                         TIMEOUT, &nWrite, &nRead, &eomReason);

    if (status == asynSuccess)
    {
        if (nWrite != (size_t) nWriteRequested ||
            strncmp(outputBuff, localbuf, nWrite) != 0)
            status = asynError;
    }

    if (status == asynSuccess)
    {
        char *echo, *reply, *ack;

        if ((echo = strstr(localbuf, "\r\n")) == NULL)
            status = asynError;
        else if ((reply = strstr(&echo[2], "\r\n")) == NULL)
            status = asynError;
        else if ((ack = strstr(&reply[2], "\r\n")) == NULL)
            status = asynError;
        else
        {
            *reply = 0;        /* Terminate reply at \r. */
            strcpy(inputBuff, &echo[2]);
        }
    }

    if (status != asynSuccess)
    {
        asynPrint(pController->pasynUser, ASYN_TRACE_ERROR,
                  "drvANC150Asyn:sendAndReceive error calling writeRead, output=%s status=%d, error=%s\n",
                  outputBuff, status, pController->pasynUser->errorMessage);
    }

    return(status);
}

        
static asynStatus getFreq(ANC150Controller *pController, int axis)
{
    asynStatus status;
    char inputBuff[BUFFER_SIZE];
    char outputBuff[BUFFER_SIZE];
    AXIS_HDL pAxis;
    int savedfreq;

    sprintf(outputBuff, "getf %d", axis + 1);
    status = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));

    if (status != asynSuccess)
        return(status);
    else if (strncmp(inputBuff, "Axis not in computer control mode", 34) == 0)
        return(status);
    else
    {
        pAxis = &pController->pAxis[axis];
        savedfreq = pAxis->frequency;
        if (sscanf(inputBuff, "frequency = %d", &pAxis->frequency) != 1)
        {
            pAxis->frequency = savedfreq;
            pasynOctetSyncIO->flush(pController->pasynUser);
            return(asynError);
        }
        return(asynSuccess);
    }
}

        
static bool stpMode(ANC150Controller *pController, int axis)
{
    asynStatus status;
    char inputBuff[BUFFER_SIZE];
    char outputBuff[BUFFER_SIZE];
    size_t nRead;
    int eomReason;
    bool rtnstatus;

    sprintf(outputBuff, "getm %d", axis + 1);
    status = sendAndReceive(pController, outputBuff, inputBuff, sizeof(inputBuff));

    if (strncmp(inputBuff, "mode = stp", 11) == 0)
        rtnstatus = true;
    else if (strncmp(inputBuff, "mode = gnd", 11) == 0)
        rtnstatus = false;
    else if (strncmp(inputBuff, "Axis not in computer control mode", 34) == 0)
    {
        /* Eat the ERROR msg. */
        status = pasynOctetSyncIO->read(pController->pasynUser, inputBuff,
                                        sizeof(inputBuff), TIMEOUT, &nRead, &eomReason);
        rtnstatus = false;
    }
    else
    {
        pasynOctetSyncIO->flush(pController->pasynUser);
        rtnstatus = true;
    }
    return(rtnstatus);
}


extern "C"
{

// Setup arguments
    static const iocshArg setupArg0 = {"Maximum # of controllers", iocshArgInt};
// Config arguments
    static const iocshArg configArg0 = {"Card# being configured", iocshArgInt};
    static const iocshArg configArg1 = {"asyn port name", iocshArgString};
    static const iocshArg configArg2 = {"Number of Axes", iocshArgInt};
    static const iocshArg configArg3 = {"Moving poll rate", iocshArgInt};
    static const iocshArg configArg4 = {"Idle poll rate", iocshArgInt};

    static const iocshArg *const SetupArgs[1]  = {&setupArg0};
    static const iocshArg *const ConfigArgs[5] = {&configArg0, &configArg1, &configArg2,
                                                    &configArg3, &configArg4};

    static const iocshFuncDef setupANC150  = {"ANC150AsynSetup",  1, SetupArgs};
    static const iocshFuncDef configANC150 = {"ANC150AsynConfig", 5, ConfigArgs};

    static void setupANC150CallFunc(const iocshArgBuf *args)
    {
        ANC150AsynSetup(args[0].ival);
    }
    static void configANC150CallFunc (const iocshArgBuf *args)
    {
        ANC150AsynConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
    }

    static void ANC150Register(void)
    {
        iocshRegister(&setupANC150, setupANC150CallFunc);
        iocshRegister(&configANC150, configANC150CallFunc);
    }

    epicsExportRegistrar(ANC150Register);

} // extern "C"


