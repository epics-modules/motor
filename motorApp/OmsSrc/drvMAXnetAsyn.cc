/*
 * FILENAME...	drvMAXnetAsyn.cc
 * USAGE...	Asyn motor driver level support for OMS MAXnet.
 *
 *
 * Version:        $Revision$
 * Modified By:    $Author$
 * Last Modified:  $Date$
 * HeadURL:        $URL $
 *
 *      Author: Jens Eden (PTB)
 *      based on Newport drvMM4000 written by Mark Rivers
 *
Version:        1.1.0
Modified By:    je
Last Modified:
    2010/03/15
    2010/05/06    je    use asynOctetSyncIO instead of asynOctet for
                        all MAXnet commands;
 */


#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#include "drvMAXnetAsyn.h"
#include "paramLib.h"

#include "epicsFindSymbol.h"
#include "epicsTime.h"
#include "epicsThread.h"
#include "epicsEvent.h"
#include "epicsString.h"
#include "epicsStdio.h"
#include "epicsMutex.h"
#include "ellLib.h"
#include "iocsh.h"
#include "asynDriver.h"
#include "asynOctet.h"
#include "asynOctetSyncIO.h"

#include "drvSup.h"
#include "epicsExport.h"
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"

motorAxisDrvSET_t motorMAXnet =
  {
    15,
    motorAxisReport,            /**< Standard EPICS driver report function (optional) */
    motorAxisInit,              /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLog,            /**< Defines an external logging function (optional) */
    motorAxisOpen,              /**< Driver open function */
    motorAxisClose,             /**< Driver close function */
    motorAxisSetCallback,       /**< Provides a callback function the driver can call when the status updates */
    motorAxisSetDouble,         /**< Pointer to function to set a double value */
    motorAxisSetInteger,        /**< Pointer to function to set an integer value */
    motorAxisGetDouble,         /**< Pointer to function to get a double value */
    motorAxisGetInteger,        /**< Pointer to function to get an integer value */
    motorAxisHome,              /**< Pointer to function to execute a more to reference or home */
    motorAxisMove,              /**< Pointer to function to execute a position move */
    motorAxisVelocityMove,      /**< Pointer to function to execute a velocity mode move */
    motorAxisStop,               /**< Pointer to function to stop motion */
    motorAxisforceCallback      /**< Pointer to function to request a poller status update */
  };

epicsExportAddress(drvet, motorMAXnet);

/* typedef struct motorAxis * AXIS_ID; */
#define MAXnet_FIRMWARE_VERSION_LENGTH 80
#define MAXnet_MAX_BUFFERLENGTH MAXnet_FIRMWARE_VERSION_LENGTH

typedef struct {
    epicsMutexId mutexId;
    asynUser* pasynUser;
    asynUser* pasynUserSyncIO;
    asynOctet *pasynOctet;
    void* octetPvt;
    void* registrarPvt;
    int numAxes;
    int notificationCounter;
    epicsMutexId notificationMutex;
    char firmwareVersionString[MAXnet_FIRMWARE_VERSION_LENGTH];
    double movingPollPeriod;
    double idlePollPeriod;
    double timeout;
    epicsEventId pollEventId;
    char *portname;
    int portConnected;
    char outputBuffer[MAXnet_MAX_BUFFERLENGTH + 1];
    int outputSize;
    int fd;
    AXIS_HDL pAxis;  /* array of axes */
} MAXnetController;

typedef struct motorAxisHandle
{
    MAXnetController *pController;
    PARAMS params;
    double currentPosition;
    char axisChar;
    int moveDelay;
    int axisStatus;
    int modeStepper;            /* 1 = stepper; 0 = servo */
    int haveEncoder;
    int limitInvert;
    int card;
    int axis;
    motorAxisLogFunc print;
    void *logParam;
    epicsMutexId mutexId;
} motorAxis;

typedef struct
{
  AXIS_HDL pFirst;
  epicsThreadId motorThread;
  motorAxisLogFunc print;
  void *logParam;
  epicsTimeStamp now;
} motorMAXnet_t;


static int motorMAXnetLogMsg(void * param, const motorAxisLogMask_t logMask, const char *pFormat, ...);
#define PRINT   (pAxis->print)
#define FLOW    motorAxisTraceFlow
#define TRACEERROR   motorAxisTraceError
#define IODRIVER  motorAxisTraceIODriver

#define MAXnet_MAX_AXES 5

static motorMAXnet_t drv={ NULL, NULL, motorMAXnetLogMsg, 0, { 0, 0 } };
static int numMAXnetControllers;
/* Pointer to array of controller strutures */
static MAXnetController *pMAXnetController=NULL;

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

static asynStatus sendReceive(AXIS_HDL pAxis, char *outputBuff, int insertChar, char *inputBuff, int inputSize);
static asynStatus sendReceiveLock(MAXnetController *pController, char *outputBuff, char *inputBuff, int inputSize);
static asynStatus sendContr(MAXnetController *pController, const char *outputBuff);
static asynStatus sendLock(AXIS_HDL pAxis, char *outputBuff, int insertChar);
static asynStatus sendReceiveContrArray(MAXnetController*, char*, epicsInt32*);
static asynStatus sendReceiveContrStatus(MAXnetController *pController, char *outputBuff, char *inputBuff, int inputSize);

/* DEBUG */
volatile int motorMAXnetdebug = 0;
extern "C" {epicsExportAddress(int, motorMAXnetdebug);}

int getSubstring(unsigned int number, char* inputBuffer, char *outBuffer, unsigned int outBufferLen)
{
    /* get the numbered substring from a comma-separated string */
    /* numbering starts with 0 */
    /* other as strtok accept a sequence of commas ,,, as empty substrings */
    char *start, *end, *pos;
    unsigned int i, count;
    int doloop = 1;

    if (strlen(inputBuffer) < number) return 0;

    start = inputBuffer;
    end = start + strlen(inputBuffer);
    for (i=0; i<=number && doloop==1; ++i){
        pos = strchr(start, ',');
        if (pos == NULL) {
            doloop = 0;
            count = MIN(strlen(start), outBufferLen);
        }
        else {
            count = MIN((unsigned int)(pos-start), outBufferLen);
            *pos = '\0';
        }
        strncpy(outBuffer, start, count);
        outBuffer[count]='\0';
        start = pos+1;
        if (start >= end) doloop =0;
    }
    if (i-1 != number) return 0;
    return i-1;
}

static void connectCallback(asynUser *pasynUser, asynException exception)
{
    asynStatus status;
    int connected = 0;
    MAXnetController *pController = (MAXnetController *)pasynUser->userPvt;

    if (exception == asynExceptionConnect) {
        status = pasynManager->isConnected(pasynUser, &connected);
        if (connected){
            if (motorMAXnetdebug > 4) asynPrint(pasynUser, ASYN_TRACE_FLOW,
				"MAXnet connectCallback:  TCP-Port connected\n");
            pController->portConnected = 1;
        }
        else {
            if (motorMAXnetdebug > 3) asynPrint(pasynUser, ASYN_TRACE_FLOW,
				"MAXnet connectCallback:  TCP-Port disconnected\n");
            pController->portConnected = 0;
        }
    }
}

static void asynCallback(void *drvPvt, asynUser *pasynUser, char *data, size_t len, int eomReason)
{
    MAXnetController* pController = (MAXnetController*)drvPvt;

    /* If the string has a "%", it is an notification, increment counter,
     * send a signal to the poller task which will trigger a poll */

    if ((len >= 1) && (strchr(data, '%') != NULL)){
    	char* pos = strchr(data, '%');
        epicsEventSignal(pController->pollEventId);
    	while (pos != NULL){
            epicsMutexLock(pController->notificationMutex);
    		++pController->notificationCounter;
            epicsMutexUnlock(pController->notificationMutex);
           ++pos;
            pos = strchr(pos, '%');
    	}
    }
}

static void motorAxisReportAxis(AXIS_HDL pAxis, int level)
{
    if (level > 0)
    {
        printf("  Axis %d \n", pAxis->axis);
        printf("    axisStatus:%d\n", pAxis->axisStatus);
    }
}

static void motorAxisReport(int level)
{
    int i, j;

    for(i=0; i<numMAXnetControllers; i++) {
        printf("Controller %d firmware version: %s\n", i, pMAXnetController[i].firmwareVersionString);
        printf(" portname: %s\n", pMAXnetController[i].portname);
        for(j=0; j<pMAXnetController[i].numAxes; j++) {
           motorAxisReportAxis(&pMAXnetController[i].pAxis[j], level);
        }
    }
}

static int motorAxisTriggerProfile( AXIS_HDL pAxis)
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisProfileMove(AXIS_HDL pAxis, int one, double *two, double *three, int four, int five)
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisInit(void)
{
    int controller, axis;

    for (controller = 0; controller < numMAXnetControllers; controller++)
    {
        AXIS_HDL pAxis;
        for (axis = 0; axis < pMAXnetController[controller].numAxes; axis++)
        {
            pAxis = &pMAXnetController[controller].pAxis[axis];
            if (!pAxis->mutexId)
                break;
            epicsMutexLock(pAxis->mutexId);

            if (pAxis->modeStepper)
                motorParam->setInteger(pAxis->params, motorAxisHasClosedLoop, 0);
            else
                motorParam->setInteger(pAxis->params, motorAxisHasClosedLoop, 1);

            /* Set encoder present */
            if (pAxis->haveEncoder)
                motorParam->setInteger(pAxis->params, motorAxisHasEncoder, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisHasEncoder, 0);

            motorParam->callCallback(pAxis->params);
            epicsMutexUnlock(pAxis->mutexId);
        }
    }
 return MOTOR_AXIS_OK;
}

static int motorAxisSetLog( AXIS_HDL pAxis, motorAxisLogFunc logFunc, void * param )
{
    if (pAxis == NULL)
    {
        if (logFunc == NULL)
        {
            drv.print=motorMAXnetLogMsg;
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
            pAxis->print=motorMAXnetLogMsg;
            pAxis->logParam = NULL;
        }
        else
        {
            pAxis->print=logFunc;
            pAxis->logParam = param;
        }
    }
    return MOTOR_AXIS_OK;
}

static AXIS_HDL motorAxisOpen(int card, int axis, char * param)
{
  AXIS_HDL pAxis;

  if (card >= numMAXnetControllers) return(NULL);
  if (axis >= pMAXnetController[card].numAxes) return(NULL);
  pAxis = &pMAXnetController[card].pAxis[axis];
  return pAxis;
}

static int motorAxisClose(AXIS_HDL pAxis)
{
  return MOTOR_AXIS_OK;
}

static int motorAxisGetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int * value)
{
    if (pAxis == NULL)
        return MOTOR_AXIS_ERROR;
    else {
        epicsMutexLock(pAxis->mutexId);
        int retval=motorParam->getInteger(pAxis->params, (paramIndex) function, value);
        epicsMutexUnlock(pAxis->mutexId);
        return retval;
    }
}

static int motorAxisGetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double * value)
{
    if (pAxis == NULL)
        return MOTOR_AXIS_ERROR;
    else {
        epicsMutexLock(pAxis->mutexId);
        int retval=motorParam->getDouble(pAxis->params, (paramIndex) function, value);
        epicsMutexUnlock(pAxis->mutexId);
        return retval;
    }
}

static int motorAxisSetCallback(AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param)
{
    if (pAxis == NULL)
        return MOTOR_AXIS_ERROR;
    else{
        epicsMutexLock(pAxis->mutexId);
        int retval=motorParam->setCallback(pAxis->params, callback, param);
        epicsMutexUnlock(pAxis->mutexId);
        return retval;
    }
}

static int motorAxisSetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    asynStatus sendStatus = asynError;
    char buffer[50];

    if (pAxis == NULL)
        return MOTOR_AXIS_ERROR;

    switch (function)
    {
        case motorAxisPosition:
        {
            PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d set position to %f\n", pAxis->card, pAxis->axis, value);
            sprintf(buffer,"A? LP%d;",(int)(value));
            sendStatus = sendLock(pAxis, buffer, 1);
            break;
        }
        case motorAxisEncoderRatio:
        {
            PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d encoder ratio to %f\n", pAxis->card, pAxis->axis, value);
            sprintf(buffer,"A? ER%d,100000;",(int)(value*100000));
            sendStatus = sendLock(pAxis, buffer, 1);
            break;
        }
        case motorAxisResolution:
        case motorAxisLowLimit:
        case motorAxisHighLimit:
            // we do nothing
            ret_status = MOTOR_AXIS_OK;
            break;
        case motorAxisPGain:
        {
            if ((0.00 >= value) && (value < 32768.00)){
                PRINT(pAxis->logParam, FLOW, "MAXnet set card %d, axis %d proportional gain to %f\n", pAxis->card, pAxis->axis, value);
                sprintf(buffer,"A? KP%f;", value);
                sendStatus = sendLock(pAxis, buffer, 1);
            }
            else {
                PRINT(pAxis->logParam,TRACEERROR, "MAXnet proportional gain %f not in range 0.0 -> 32768.0\n", value);
            }
            break;
        }
        case motorAxisIGain:
        {
            if ((0.00 >= value) && (value < 32768.00)){
                PRINT(pAxis->logParam, FLOW, "MAXnet set card %d, axis %d integral gain to %f\n", pAxis->card, pAxis->axis, value);
                sprintf(buffer,"A? KI%f;", value);
                sendStatus = sendLock(pAxis, buffer, 1);
            }
            else {
                PRINT(pAxis->logParam,TRACEERROR, "MAXnet integral gain %f not in range 0.0 -> 32768.0\n", value);
            }
            break;
        }
        case motorAxisDGain:
        {
             if ((0.00 >= value) && (value < 32768.00)){
                PRINT(pAxis->logParam, FLOW, "MAXnet set card %d, axis %d derivative gain to %f\n", pAxis->card, pAxis->axis, value);
                sprintf(buffer,"A? KD%f;", value);
                sendStatus = sendLock(pAxis, buffer, 1);
            }
            else {
                PRINT(pAxis->logParam,TRACEERROR, "MAXnet derivative gain %f not in range 0.0 -> 32768.0\n", value);
            }
            break;
        }
        default:
            PRINT(pAxis->logParam,TRACEERROR, "motorAxisSetDouble: unknown function %d\n", function);
            break;
    }
    if (sendStatus == asynSuccess) ret_status = MOTOR_AXIS_OK;
    if (ret_status != MOTOR_AXIS_ERROR) {
        epicsMutexLock(pAxis->mutexId);
        motorParam->setDouble(pAxis->params, function, value);
        epicsMutexUnlock(pAxis->mutexId);
    }
    return ret_status;
}

static int motorAxisSetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    asynStatus sendStatus = asynError;
    char buffer[10];

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    switch (function) {
    case motorAxisClosedLoop:
        if (value) {
            PRINT(pAxis->logParam, FLOW, "motorAxisSetInteger set card %d, axis %d closed loop enable\n",
                      pAxis->card, pAxis->axis);
            sprintf(buffer, "A? HN");
            sendStatus = sendLock(pAxis, buffer, 1);
        } else {
            PRINT(pAxis->logParam, FLOW, "motorAxisSetInteger set card %d, axis %d closed loop disable\n",
                      pAxis->card, pAxis->axis);
            sprintf(buffer, "A? HF");
            sendStatus = sendLock(pAxis, buffer, 1);
        }
        break;
    default:
        PRINT(pAxis->logParam,TRACEERROR, "motorAxisSetInteger: unknown function %d\n", function);
        break;
    }
    if (sendStatus == asynSuccess) ret_status = MOTOR_AXIS_OK;
    if (ret_status != MOTOR_AXIS_ERROR) {
        epicsMutexLock(pAxis->mutexId);
        motorParam->setInteger(pAxis->params, function, value);
        epicsMutexUnlock(pAxis->mutexId);
    }
    return ret_status;
}


static int motorAxisMove(AXIS_HDL pAxis, double position, int relative,
                          double min_velocity, double max_velocity, double acceleration)
{
    int ret_status = MOTOR_AXIS_ERROR;
    asynStatus sendStatus = asynError;
    epicsInt32 minvelo, velo, acc, rela, pos;
    char *relabs[2] = {(char *) "MA", (char *) "MR"};
    char buff[100];

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    if (relative)
        rela = 1;
    else
        rela = 0;

    pos = (epicsInt32) (position + 0.5);
    if (abs(pos) > 67000000){
        PRINT(pAxis->logParam,TRACEERROR, "motorAxisMove: position out of range %f\n", position);
        return MOTOR_AXIS_ERROR;
    }

     velo = (epicsInt32) (max_velocity + 0.5);
    if (velo < 1) velo = 1;
    else if (velo > 4000000) velo = 4000000;

     minvelo = (epicsInt32) (min_velocity + 0.5);
    if (minvelo < 0) minvelo = 0;
    else if (minvelo >= velo) minvelo = velo - 1;

    acc = abs((epicsInt32) acceleration);
    if (acc > 8000000)
        acc = 8000000;
    else if (acc < 1)
        acc = 1;

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f\n",
          pAxis->card, pAxis->axis, position, min_velocity, max_velocity, acceleration);


    /* move to the specified position */
    sprintf(buff, "A? AC%d; VB%d; VL%d; %s%d; GO ID", acc, minvelo, velo, relabs[rela], pos);
    sendStatus = sendLock(pAxis, buff, 1);
    if (sendStatus == asynSuccess) ret_status = MOTOR_AXIS_OK;

     /* set the Move-Flag */
    epicsMutexLock(pAxis->mutexId);
    motorParam->setInteger(pAxis->params, motorAxisDone, 0);
    motorParam->callCallback(pAxis->params);
    epicsMutexUnlock(pAxis->mutexId);

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    return ret_status;
}

static int motorAxisHome(AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards)
{
    int ret_status = MOTOR_AXIS_ERROR;
    asynStatus sendStatus = asynError;
    char buff[60];
    char *direction[2] = {(char*) "HR", (char*) "HM"};
    epicsInt32 velo, acc, fw;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    if (forwards)
        fw = 1;
    else
        fw=0;

     velo = (epicsInt32) max_velocity;
    if (velo < 1) velo = 1;
    else if (velo > 1000000) velo = 1000000;

    acc = abs((epicsInt32) acceleration);
    if (acc > 8000000)
        acc = 8000000;
    else if (acc < 1)
        acc = 1;

    PRINT(pAxis->logParam, FLOW, "motorAxisHome: set card %d, axis %d to home\n",
          pAxis->card, pAxis->axis);

    /* do a home run and move to the home position */
    sprintf(buff, "A? AC%d; VL%d; %s; MA0 GO ID", acc, velo, direction[forwards]);
    sendStatus = sendLock(pAxis, buff, 1);
    if (sendStatus == asynSuccess) ret_status = MOTOR_AXIS_OK;

     /* set the Move-Flag */
    epicsMutexLock(pAxis->mutexId);
    motorParam->setInteger(pAxis->params, motorAxisDone, 0);
    motorParam->callCallback(pAxis->params);
    epicsMutexUnlock(pAxis->mutexId);

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    return ret_status;
}

/*
 *   min_velocity is not supported with jog commands
 */
static int motorAxisVelocityMove(AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration)
{
    int ret_status = MOTOR_AXIS_ERROR;
    asynStatus sendStatus = asynError;
    char buff[100];
    epicsInt32 velo, acc;

    if (pAxis == NULL) return ret_status;

    acc = (epicsInt32) acceleration;
    if (acc < 1) acc = 1;
    else if (acc > 8000000) acc = 8000000;

    velo = (epicsInt32) velocity;
    if (velo > 4000000) velo = 4000000;
    else if (velo < -4000000) velo = -4000000;

    PRINT(pAxis->logParam, FLOW, "motorAxisVelocityMove card %d, axis %d move velocity=%d, accel=%d\n",
          pAxis->card, pAxis->axis, velo, acc);

    epicsMutexLock(pAxis->mutexId);
    sprintf(buff, "A? AC%d; JG%d;", acc, velo);
    sendStatus = sendLock(pAxis, buff, 1);
    if (sendStatus == asynSuccess) ret_status = MOTOR_AXIS_OK;

     /* set the Move-Flag */
    motorParam->setInteger(pAxis->params, motorAxisDone, 0);
    motorParam->callCallback(pAxis->params);
    epicsMutexUnlock(pAxis->mutexId);

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    return ret_status;
}

static int motorAxisStop(AXIS_HDL pAxis, double acceleration)
{
    int ret_status = MOTOR_AXIS_ERROR;
    asynStatus sendStatus = asynError;
    int acc;
    char buff[50];

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d to stop with accel=%f\n",
          pAxis->card, pAxis->axis, acceleration);

    acc = (int)(fabs(acceleration)+0.5);
    if (acc > 8000000) acc=8000000;
    if (acc < 1) acc = 200000;

    sprintf(buff, "A? AC%d; ST ID;",acc);
    sendStatus = sendLock(pAxis, buff, 1);
    if (sendStatus == asynSuccess) ret_status = MOTOR_AXIS_OK;

    /* Send a signal to the poller task which will make it do a poll */
    epicsEventSignal(pAxis->pController->pollEventId);

    return ret_status;
}

static int motorAxisforceCallback(AXIS_HDL pAxis)
{
    if (pAxis == NULL)
        return (MOTOR_AXIS_ERROR);

    PRINT(pAxis->logParam, FLOW, "motorAxisforceCallback: request card %d, axis %d status update\n",
          pAxis->card, pAxis->axis);

    /* Force a status update. */
    motorParam->forceCallback(pAxis->params);

    /* Send a signal to the poller task which will make it do a status update */
    epicsEventSignal(pAxis->pController->pollEventId);
    return (MOTOR_AXIS_OK);
}

static void MAXnetPoller(MAXnetController *pController)
{
#define STATUSBUFFERLEN 5
    /* This is the task that polls the MAXnet */
    double timeout, timeToWait, pollWait;
    AXIS_HDL pAxis;
    int i, retry_count, substatus;
    // asynStatus status;
    int axisMoving, anyMoving;
    int forcedFastPolls=0;
    int sanityCounter = 0;
    epicsInt32  axisPosArr[MAXnet_MAX_AXES], encPosArr[MAXnet_MAX_AXES], veloArr[MAXnet_MAX_AXES], commandBufferSize[MAXnet_MAX_AXES];
    char inputBuffer[50];
    char statusBuffer[MAXnet_MAX_AXES*STATUSBUFFERLEN+2];
    char encStatusBuffer[MAXnet_MAX_AXES*6+2];
    char encBuffer[7];
    char outputBuffer[10];
    asynStatus status;
    size_t nRead;
    int eomReason = 0;
    unsigned int limitFlags;
    epicsTimeStamp now, loopStart;

    while(1) {

        anyMoving = 0;
        limitFlags =0;

        epicsTimeGetCurrent(&loopStart);

        /* read all axis status values and reset done-field
         * MDNN,MDNN,PNLN,PNNN,PNLN,PNNN,PNNN,PNNN<LF>*/
        retry_count=0;
        while ((sendReceiveContrStatus(pController, (char*) "AM;RI;", statusBuffer,
                               sizeof(statusBuffer)) != asynSuccess) && (retry_count < 10)){
            epicsThreadSleep(0.02);
            ++retry_count;
        }
        if (retry_count > 9){
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				      "MAXnetPoller: Unable to read axis status after %d tries\n", retry_count);
        }

        ++ sanityCounter;
        if (sanityCounter > 100){
            // check the remaining command buffer size, which must not overflow
            // the commands used below do not use the command buffer, it cannot overflow
            // we keep it as a remainder for future extensions
            sanityCounter = 0;
            if (sendReceiveContrArray(pController, (char*) "AM;RQC", commandBufferSize) != asynSuccess){
                asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
							"MAXnetPoller: Error executing MAXnet command: Report Command Queue (RCQ)\n");
            }
            for (i=0; i < pController->numAxes; i++) {
                pAxis = &pController->pAxis[i];
                if (motorMAXnetdebug > 3) asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_FLOW,
									"MAXnetPoller: Command buffer axis %c: %d\n", pAxis->axisChar, commandBufferSize[i]);

                // flush command buffer if size is below 100
                if (commandBufferSize[i] < 100) {
                    asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
                    		"MAXnetPoller: Caution: flushing command queue axis %c, remaining size %d\n", pAxis->axisChar, commandBufferSize[i]);
                    strcpy(outputBuffer,"A?;FL;");
                    sendLock(pAxis, (char*) outputBuffer, i);
                }
            }

        }

        if (sendReceiveContrArray(pController, (char*) "PP;", axisPosArr) != asynSuccess){
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				    "MAXnetPoller Error executing MAXnet command Axes Position (PP)\n");
        }
        if (sendReceiveContrArray(pController, (char*) "PE;", encPosArr) != asynSuccess){
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				    "MAXnetPoller Error executing MAXnet command Encoder Position (PE)\n");
        }
        if (sendReceiveContrArray(pController, (char*) "AM;RV;", veloArr) != asynSuccess){
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				    "MAXnetPoller Error executing MAXnet command Report Velocity (RV)\n");
        }
        if (sendReceiveLock(pController, (char*) "AM;EA;", encStatusBuffer, sizeof(encStatusBuffer)) != asynSuccess){
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				    "MAXnetPoller Error reading encoder status buffer %s\n", encStatusBuffer);
        }

        /* read all limits */
        if (sendReceiveLock(pController, (char*) "AM;QL;", inputBuffer, sizeof(inputBuffer))!= asynSuccess)
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				"MAXnetPoller error reading limits %s\n", inputBuffer);
        if (1 != sscanf(inputBuffer, "%x", &limitFlags))
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				"MAXnetPoller error converting limits: %s\n", inputBuffer);

        /* Lock all the controller's axes. */
        for (i = 0; i < pController->numAxes; i++)
        {
            pAxis = &pController->pAxis[i];
            epicsMutexLock(pAxis->mutexId);
        }

        for (i=0; i < pController->numAxes; i++) {
            pAxis = &pController->pAxis[i];

            /* if this axis has an encoder, read encoder status */
            if (pAxis->haveEncoder){
                if ((substatus = getSubstring(i, encStatusBuffer, encBuffer, sizeof(encBuffer)))){
                    if (substatus != i) PRINT(pAxis->logParam,TRACEERROR, "MAXnetPoller: error encoder: status %d, i %d\n", substatus, i);
                    if (encBuffer[2] == 'S')
                        motorParam->setInteger(pAxis->params, motorAxisFollowingError, 1);
                    else
                        motorParam->setInteger(pAxis->params, motorAxisFollowingError, 0);
                }
                motorParam->setInteger(pAxis->params, motorAxisEncoderPosn, encPosArr[i]);
            }

            /* check the done flag or current velocity */
            if (statusBuffer[i*STATUSBUFFERLEN + 1] == 'D'){
                   pAxis->moveDelay=0;
                motorParam->setInteger(pAxis->params, motorAxisDone, 1);
                motorParam->setInteger(pAxis->params, motorAxisMoving, 0);
            }
            else if (veloArr[i] == 0){
                /* keep a small delay here, to make sure that a motorAxisDone cycle is performed
                   even if the motor is on hard limits */
                motorParam->getInteger(pAxis->params, motorAxisMoving, &axisMoving);
                if (axisMoving){
                    pAxis->moveDelay++ ;
                    if (pAxis->moveDelay >= 5) {
                        motorParam->setInteger(pAxis->params, motorAxisDone, 1);
                        motorParam->setInteger(pAxis->params, motorAxisMoving, 0);
                        pAxis->moveDelay = 0;
                    }
                }
            }
            else {
                pAxis->moveDelay = 0;
                motorParam->setInteger(pAxis->params, motorAxisDone, 0);
                motorParam->setInteger(pAxis->params, motorAxisMoving, 1);
            }
            motorParam->getInteger(pAxis->params, motorAxisMoving, &axisMoving);
            anyMoving += axisMoving;

            /* check limits */
            if (((limitFlags & (1 << i)) > 0) ^ (pAxis->limitInvert))
                motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 0);
            if (((limitFlags & (1 << (i+8))) > 0) ^ (pAxis->limitInvert))
                motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 0);

            /* check home switch */
            if (statusBuffer[i*STATUSBUFFERLEN + 3] == 'H')
                motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 0);

            /* check direction */
            if (statusBuffer[i*STATUSBUFFERLEN] == 'P')
                motorParam->setInteger(pAxis->params, motorAxisDirection, 1);
            else
                motorParam->setInteger(pAxis->params, motorAxisDirection, 0);

            /* set positions */
            motorParam->setInteger(pAxis->params, motorAxisPosition, axisPosArr[i]);

            motorParam->callCallback(pAxis->params);

        } /* Next axis */

        /* UnLock all the controller's axes. */
        for (i = 0; i < pController->numAxes; i++)
        {
            pAxis = &pController->pAxis[i];
            epicsMutexUnlock(pAxis->mutexId);
        }

        if (forcedFastPolls > 0) {
            timeout = pController->movingPollPeriod;
            pollWait = pController->movingPollPeriod / 25.0;
            forcedFastPolls--;
        }
        else if (anyMoving) {
            timeout = pController->movingPollPeriod;
            pollWait = pController->movingPollPeriod / 25.0;
        }
        else {
            timeout = pController->idlePollPeriod;
            pollWait = pController->idlePollPeriod / 5.0;
        }

        /* wait here for the next poll and read while waiting to receive interrupt messages
           waiting may be interrupted by pollEvent */
        epicsTimeGetCurrent(&now);
        timeToWait = timeout - epicsTimeDiffInSeconds(&now, &loopStart);
        pasynManager->lockPort(pController->pasynUserSyncIO);
        pController->pasynOctet->flush(pController->octetPvt, pController->pasynUser);
        pasynManager->unlockPort(pController->pasynUserSyncIO);
        while ( timeToWait > 0){
            /* reading with bufferlength 0 and timeout 0.0 triggers a callback and
             * poll event if a notification is outstanding */
            pasynManager->lockPort(pController->pasynUserSyncIO);
            status = pController->pasynOctet->read(pController->octetPvt, pController->pasynUser, inputBuffer,
            		                                0, &nRead, &eomReason);
            pasynManager->unlockPort(pController->pasynUserSyncIO);
            if (epicsEventWaitWithTimeout(pController->pollEventId, pollWait) == epicsEventWaitOK) {
                if (motorMAXnetdebug > 3) asynPrint(pController->pasynUser, ASYN_TRACE_FLOW,
													"MAXnetPoller wakeup call received\n");
                forcedFastPolls = 10;
                break;
            }
            epicsTimeGetCurrent(&now);
            timeToWait = timeout - epicsTimeDiffInSeconds(&now, &loopStart);
        }
    } /* End while */
}

/*
    send a string to MAXnet Card
    If insertChar != 0 and 2nd Character is a "?", then replace it with axisChar
*/
static asynStatus sendLock(AXIS_HDL pAxis, char *outputBuff, int insertChar)
{
    asynStatus status;

    if (insertChar && (strncmp(outputBuff,"A? ",3)==0))
        outputBuff[1] = pAxis->axisChar;
    epicsMutexLock(pAxis->pController->mutexId);
    status = sendContr(pAxis->pController, outputBuff);
    epicsMutexUnlock(pAxis->pController->mutexId);
    return status;
}

static asynStatus sendContr(MAXnetController *pController, const char *outputBuff)
{
    size_t nActual = 0;
    asynStatus status;

    status = pasynOctetSyncIO->write(pController->pasynUserSyncIO, outputBuff,
                                          strlen(outputBuff), pController->timeout, &nActual);

    if (status != asynSuccess) {
        asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
                  "drvMAXnetAsyn:sendContr: error sending command %d, sent=%d, status=%d\n",
                  outputBuff, nActual, status);
    }
    return(status);
}

/*
    send a string to MAXnet Card and wait for the answer
    If insertChar != 0 and 2nd Character is a "?", then replace it with axisChar
*/
static asynStatus sendReceive(AXIS_HDL pAxis, char *outputBuff, int insertChar, char *inputBuff, int inputSize)
{
    if (insertChar && (strncmp(outputBuff,"A? ",3)==0))
        outputBuff[1] = pAxis->axisChar;
    return sendReceiveLock(pAxis->pController, outputBuff, inputBuff, inputSize);
}

/*
 * check if buffer is a notification messages with 13 chars ("%000 SSSSSSSS")
 */
static int isNotification (MAXnetController *pController, char *buffer) {

    char *inString;
    if ((inString = strstr(buffer, "000 0")) != NULL){
        if ((inString = strstr(buffer, "000 01")) != NULL){
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
                          "drvMAXnetAsyn:isNotification: CMD_ERR_FLAG received\n");
        }
        else {
            if (motorMAXnetdebug > 3) asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_FLOW,
                    "drvMAXnetAsyn:isNotification: Interrupt notification: %s\n", buffer);
            epicsEventSignal(pController->pollEventId);
        }
        return 1;
    }
    else
        return 0;
}

static asynStatus sendReceiveLock(MAXnetController *pController, char *outputBuff, char *inputBuff, int inputSize)
{
    char localBuffer[MAXnet_MAX_BUFFERLENGTH + 1] = "";
    size_t nRead, nWrite, bufferSize = inputSize;
    int eomReason = 0;
    asynStatus status = asynSuccess;
    char *outString;
    int errorCount = 10;

    if (bufferSize > MAXnet_MAX_BUFFERLENGTH) bufferSize = MAXnet_MAX_BUFFERLENGTH;

    epicsMutexLock(pController->mutexId);
    pasynManager->lockPort(pController->pasynUserSyncIO);

    /*
     * read the notification from input buffer
     */
    while ((pController->notificationCounter > 0) && errorCount){
    	status = pasynOctetSyncIO->read(pController->pasynUserSyncIO, localBuffer, bufferSize, 0.1, &nRead, &eomReason);
    	if (status == asynSuccess) {
    		localBuffer[nRead] = '\0';
            if (isNotification(pController, localBuffer) && (pController->notificationCounter > 0)) --pController->notificationCounter;
    	}
        else if (status == asynTimeout) {
            pController->notificationCounter = 0;
    	}
        else {
        	--errorCount;
        }
    }

	status = pasynOctetSyncIO->writeRead(pController->pasynUserSyncIO, outputBuff, strlen(outputBuff), localBuffer,
                                        bufferSize, pController->timeout, &nWrite, &nRead, &eomReason);
    if (status == asynSuccess) localBuffer[nRead] = '\0';

    // if input data is a notification read until expected data arrived
    while ((status == asynSuccess) && isNotification(pController, localBuffer)) {
        status = pasynOctetSyncIO->read(pController->pasynUserSyncIO, localBuffer,
                                             bufferSize, pController->timeout, &nRead, &eomReason);
        if (status == asynSuccess) localBuffer[nRead] = '\0';
    }
    pasynManager->unlockPort(pController->pasynUserSyncIO);

    if ((status == asynSuccess) && (eomReason == ASYN_EOM_EOS)) {
        // cut off a leading /006
        outString = strrchr(localBuffer, 6);
        if (outString == NULL) {
            outString = localBuffer;
        }
        else {
            ++outString;
        }

        // copy into inputBuffer
        strncpy(inputBuff, outString, inputSize);
        inputBuff[inputSize-1] = '\0';
    }

    epicsMutexUnlock(pController->mutexId);
    return status;
}

static asynStatus sendReceiveContrStatus(MAXnetController *pController, char *outputBuff, char *inputBuff, int inputSize)
{
    asynStatus status;
    status = sendReceiveLock(pController, outputBuff, inputBuff, inputSize);

    if (status == asynSuccess){
        if (!((inputBuff[0] == 'P') || (inputBuff[0] == 'M')))
            status = asynError;
        if (strlen(inputBuff) < (unsigned int)(pController->numAxes * 5 -1))
            status = asynError;
        if (status == asynError)
            if (motorMAXnetdebug > 0) asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				"drvMAXnetAsyn:sendReceiveContrStatus: corrupted status string %s\n", inputBuff);
    }
    return status;
}

static asynStatus sendReceiveContrArray(MAXnetController *pController, char* cmd, epicsInt32 positions[MAXnet_MAX_AXES])
{
    //  maximum length of the ascii position / velocity values is 11 chars + comma
    //  -2147483648 <-> 2147483647
    // we expect pController->numAxes values separated with commas
    // possible answers are "0,5000,0" ",,,," "0" ",,," (3 commas for 4 axes)
    #define MAXnet_MAXNUMBERLEN 12

    asynStatus status = asynSuccess;
    char inputBuffer[MAXnet_MAXNUMBERLEN*MAXnet_MAX_AXES +2] = "";
    char *start, *end, *stop;
    int i, intVal, again = 1;
    int count =0;

    status = sendReceiveLock(pController, cmd, inputBuffer, sizeof(inputBuffer));
    if ((status == asynSuccess) && (strlen(inputBuffer) >= (unsigned int)pController->numAxes -1)) {
        start = inputBuffer;
        stop = start + MIN(strlen(inputBuffer), sizeof(inputBuffer));
        for (i = 0; ((i < MAXnet_MAX_AXES) && again); ++i){
            if (*start == ','){
                positions[i] = 0;
                ++count;
                start += 1;
                if (*start == '\0'){
                    /* comma was last character */
                    ++count;
                    again=0;
                    if (i < MAXnet_MAX_AXES-1) positions[i+1] = 0;
                }
                continue;
            }
            intVal = strtol(start, &end, 10);
            if ((intVal == 0) && (start == end)) {
                again = 0;
            }
            else {
                positions[i] = intVal;
                ++count;
            }
            if (end >= stop) again = 0;
            start = end + 1;
        }
        if (count != pController->numAxes) {
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				"MAXnet sendReceiveContrArray: array string conversion error, count: %d, axes: %d, input: >%s<\n",count, pController->numAxes, inputBuffer);
            return asynError;
        }
    }
    else {
        if (status == asynSuccess){
            asynPrint(pController->pasynUserSyncIO, ASYN_TRACE_ERROR,
				"MAXnet sendReceiveContrArray: read string too short %d\n", strlen(inputBuffer));
            return asynError;
        }
    }
    return status;
}

static int motorMAXnetLogMsg(void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{

    va_list     pvar;
    int         nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout,pFormat,pvar);
    va_end (pvar);
    printf("\n");
    return(nchar);
}


int MAXnetSetup(int num_controllers)   /* number of MAXnet cards in system.  */
{

    if (num_controllers < 1) {
        printf("MAXnetSetup, num_controllers must be > 0\n");
        return MOTOR_AXIS_ERROR;
    }
    numMAXnetControllers = num_controllers;
    pMAXnetController = (MAXnetController *)calloc(numMAXnetControllers, sizeof(MAXnetController));
    return MOTOR_AXIS_OK;
}


int MAXnetConfig(int card,                   /* Controller number */
              const char *portName,     /* MAXnet Device name */
              int numAxes,                /* Number of axes this controller supports */
              int movingPollPeriod,     /* Time to poll (msec) when an axis is in motion */
              int idlePollPeriod,       /* Time to poll (msec) when an axis is idle. 0 for no polling */
              const char *initString)   /* Init String sent to card */
{
    AXIS_HDL pAxis;
    int axis, Firmwaremajor, Firmwareminor;
    int totalAxes, retry_count;
    asynStatus status;
    MAXnetController *pController;
    char threadName[20];
    char inputBuffer[160];
    char outputBuffer[10];
    char *p, *tokSave;
    char axisChrArr[8] = {'X','Y','Z','T','U','V','R','S'};
    asynInterface *pasynInterface;


    if (numMAXnetControllers < 1) {
        printf("MAXnetConfig: no MAXnet controllers allocated, call MAXnetSetup first\n");
        return MOTOR_AXIS_ERROR;
    }
    if ((card < 0) || (card >= numMAXnetControllers)) {
        printf("MAXnetConfig: card must be in range 0 to %d\n", numMAXnetControllers-1);
        return MOTOR_AXIS_ERROR;
    }
    if ((numAxes < 1) || (numAxes > MAXnet_MAX_AXES)) {
        printf("MAXnetConfig: numAxes must be in range 1 to %d\n", MAXnet_MAX_AXES);
        return MOTOR_AXIS_ERROR;
    }

    pController = &pMAXnetController[card];
    pController->pAxis = (AXIS_HDL) calloc(numAxes, sizeof(motorAxis));
    pController->numAxes = numAxes;
    pController->movingPollPeriod = ((double)movingPollPeriod)/1000.;
    pController->idlePollPeriod = ((double)idlePollPeriod)/1000.;
    pController->portname = epicsStrDup(portName);
    pController->mutexId = epicsMutexMustCreate();
    pController->notificationMutex = epicsMutexMustCreate();
    pController->notificationCounter = 0;
    pController->pasynUser = pasynManager->createAsynUser(0,0);
    pController->pasynUser->userPvt = pController;
    pController->pollEventId = epicsEventMustCreate(epicsEventEmpty);

    pController->pasynUserSyncIO = pasynManager->createAsynUser(0,0);
    pController->pasynUserSyncIO->userPvt = pController;

    status = pasynManager->connectDevice(pController->pasynUser,pController->portname,0);
    if(status != asynSuccess){
        printf("MAXnetConfig: can't connect to port %s: %s\n",pController->portname,pController->pasynUser->errorMessage);
        return MOTOR_AXIS_ERROR;
    }

    status =  pasynManager->exceptionCallbackAdd(pController->pasynUser, connectCallback);
    if(status != asynSuccess){
        printf("MAXnetConfig: can't set exceptionCallback for %s: %s\n",pController->portname,pController->pasynUser->errorMessage);
        return MOTOR_AXIS_ERROR;
    }
    /* set the connect flag */
    pasynManager->isConnected(pController->pasynUser, &pController->portConnected);

    pasynInterface = pasynManager->findInterface(pController->pasynUser,asynOctetType,1);
    if( pasynInterface == NULL) {
        printf("MAXnetConfig: %s driver not supported\n", asynOctetType);
        return MOTOR_AXIS_ERROR;
    }
    pController->pasynOctet = (asynOctet*)pasynInterface->pinterface;
    pController->octetPvt = pasynInterface->drvPvt;

    status = pasynOctetSyncIO->connect(pController->portname, 0, &pController->pasynUserSyncIO, NULL);

    /* flush any junk at input port - should be no data available */
    pasynOctetSyncIO->flush(pController->pasynUserSyncIO);

    pController->timeout = 2.0;
    pController->pasynUser->timeout = 0.0;

    // CAUTION firmware versions before 1.33.4 use for serial port \n, and for TCP port \n\r as InputEOS
    // Set inputEOS in st.cmd for old firmware versions
    status = pController->pasynOctet->setInputEos(pController->octetPvt, pController->pasynUser, "\n", 1);
    status = pasynOctetSyncIO->setInputEos(pController->pasynUserSyncIO, "\n", 1);
    if(status != asynSuccess){
        printf("MAXnetConfig: unable to set InputEOS %s: %s\n", pController->portname, pController->pasynUserSyncIO->errorMessage);
        return MOTOR_AXIS_ERROR;
    }

    status = pasynOctetSyncIO->setOutputEos(pController->pasynUserSyncIO, "\n", 1);
    if(status != asynSuccess){
        printf("MAXnetConfig: unable to set OutputEOS %s: %s\n",pController->portname,pController->pasynUserSyncIO->errorMessage);
        return MOTOR_AXIS_ERROR;
    }

    status = pController->pasynOctet->registerInterruptUser(pController->octetPvt,pController->pasynUser,asynCallback,pController,&pController->registrarPvt);
    if(status != asynSuccess) {
        printf("MAXnetConfig: registerInterruptUser failed - %s: %s\n",pController->portname,pController->pasynUser->errorMessage);
        return MOTOR_AXIS_ERROR;
    }

    /* get FirmwareVersion, try 3 times */
    retry_count = 0;
    status = asynError;
    while (status != asynSuccess && retry_count < 3){
        epicsThreadSleep(1.0);
        status = sendReceiveLock(pController, (char*) "WY", pController->firmwareVersionString, MAXnet_FIRMWARE_VERSION_LENGTH);
        retry_count++;
        printf("MAXnet Firmware Version %s\n", pController->firmwareVersionString);
    }
    if(status != asynSuccess) {
        printf("MAXnetConfig: unable to talk to controller at %s: %s\n",pController->portname,pController->pasynUserSyncIO->errorMessage);
        return MOTOR_AXIS_ERROR;
    }

    // TODO enhance command strings for fw-versions below 1.30
    if ((p = strstr(pController->firmwareVersionString, "ver:"))){
        retry_count = sscanf(p, "ver:%d.%d,", &Firmwaremajor, &Firmwareminor);
        if ((retry_count == 2) && (Firmwareminor < 30 )){
            printf("This Controllers Firmware Version %d.%d is not supported, version 1.30 or higher is mandatory\n", Firmwaremajor, Firmwareminor);
        }
    }
    if ((p == NULL) || (retry_count != 2)) {
        printf("MAXnetConfig: unable to retrieve Firmware version, version 1.30 or higher is mandatory\n");
    }

    /* get Positions of all axes */
    inputBuffer[0] = '\0';
    sendReceiveLock(pController, (char*) "AA RP", inputBuffer, sizeof(inputBuffer));

    /* The return string will tell us how many axes this controller has */
    for (totalAxes = 0, tokSave = NULL, p = epicsStrtok_r(inputBuffer, ",", &tokSave);
             p != 0; p = epicsStrtok_r(NULL, ",", &tokSave), totalAxes++);

    if (totalAxes != numAxes) {
        printf("MAXnetConfig: actual number of axes=%d <> numAxes=%d\n", totalAxes, numAxes);
        return MOTOR_AXIS_ERROR;
    }

    /* send InitString */
    if ((initString != NULL) && (strlen(initString) > 0)) {
        sendContr(pController, initString);
    }
    for (axis=0; axis < numAxes; axis++) {
        pAxis = &pController->pAxis[axis];
        pAxis->pController = pController;
        pAxis->axisChar = axisChrArr[axis];
        pAxis->card = card;
        pAxis->axis = axis;
        pAxis->mutexId = epicsMutexMustCreate();
        pAxis->params = motorParam->create(0, MOTOR_AXIS_NUM_PARAMS);

        /* Determine if encoder is present and if mode is stepper or servo. */
        strcpy(outputBuffer,"A? PS?");
        sendReceive(pAxis, outputBuffer, 1, inputBuffer, sizeof(inputBuffer));
        /*we expect any of "=O","=E","=M"  */
        if (inputBuffer[1] == 'O'){
            pAxis->modeStepper      = 1;
            pAxis->haveEncoder    = 0;
        }
        else if (inputBuffer[1] == 'M'){
            pAxis->modeStepper      = 0;
            pAxis->haveEncoder    = 1;
        }
        else if (inputBuffer[1] == 'E'){
            pAxis->modeStepper      = 1;
            pAxis->haveEncoder    = 1;
        }
        else
            printf("MAXnetConfig: error: unknown axis type! (%s)\n", inputBuffer);

        /* Determine limit true state high or low */
        /* CAUTION you need firmware version 1.30 or higher to do this */
        strcpy(outputBuffer,"A? LT?");
        sendReceive(pAxis, outputBuffer, 1, inputBuffer, sizeof(inputBuffer));
        /*we expect any of "=l" or "=h"  */
        if (inputBuffer[1] == 'l'){
            pAxis->limitInvert      = 1;
        }
        else if (inputBuffer[1] == 'h'){
            pAxis->limitInvert      = 0;
        }
        else
            printf("MAXnetConfig: error: unknown limit true state!\n");
    }

    /* Create the poller thread for this controller */
    epicsSnprintf(threadName, sizeof(threadName), "MAXnet:%d", card);
    epicsThreadCreate(threadName,
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) MAXnetPoller, (void *) pController);

    return MOTOR_AXIS_OK;

}

/*
 *  send a string to MAXnet card and show the answer
*/
void MAXnetSendRead(int card,               /* number of card */
                  const char *sendstr)      /* string to send*/
{
    char inputBuffer[130];
    MAXnetController *pController;
    asynStatus status;
    double timeout;

    if ((card < 0) || (card >= numMAXnetControllers)) {
        printf("MAXnetSendRead: card must be in range 0 to %d\n", numMAXnetControllers-1);
        return;
    }

    pController = &pMAXnetController[card];
    timeout = pController->timeout;
    // set a small timeout in case we don't get an answer
    pController->timeout = 0.1;
    status = sendReceiveLock(pController, (char*) sendstr, inputBuffer, sizeof(inputBuffer));
    // restore timeout
    pController->timeout = timeout;

    if (status == asynSuccess)
        printf("MAXnetSendRead:\n    sent: %s\n  answer: %s\n", sendstr, inputBuffer);
    else if (status == asynTimeout)
        printf("MAXnetSendRead: read timeout\n");
    else
        printf("MAXnetSendRead: error\n");

    return;
}

/* Code for iocsh registration */

/* MAXnetSetup */
static const iocshArg MAXnetSetupArg0 = {"Number of MAXnet controllers", iocshArgInt};
static const iocshArg * const MAXnetSetupArgs[1] =  {&MAXnetSetupArg0};
static const iocshFuncDef setupMAXnet = {"MAXnetSetup", 1, MAXnetSetupArgs};
static void setupMAXnetCallFunc(const iocshArgBuf *args)
{
    MAXnetSetup(args[0].ival);
}


/* MAXnetConfig */
static const iocshArg MAXnetConfigArg0 = {"card being configured", iocshArgInt};
static const iocshArg MAXnetConfigArg1 = {"asyn port name", iocshArgString};
static const iocshArg MAXnetConfigArg2 = {"number of axes", iocshArgInt};
static const iocshArg MAXnetConfigArg3 = {"moving poll rate", iocshArgInt};
static const iocshArg MAXnetConfigArg4 = {"idle poll rate", iocshArgInt};
static const iocshArg MAXnetConfigArg5 = {"initstring", iocshArgString};
static const iocshArg * const MAXnetConfigArgs[6] = {&MAXnetConfigArg0,
                                                  &MAXnetConfigArg1,
                                                  &MAXnetConfigArg2,
                                                  &MAXnetConfigArg3,
                                                  &MAXnetConfigArg4,
                                                  &MAXnetConfigArg5};
static const iocshFuncDef configMAXnet = {"MAXnetConfig", 6, MAXnetConfigArgs};
static void configMAXnetCallFunc(const iocshArgBuf *args)
{
    MAXnetConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval);
}

// MAXnetSendRead
static const iocshArg sendReadArg0 = {"Card to sent string to", iocshArgInt};
static const iocshArg sendReadArg1 = {"MAX command string", iocshArgString};
static const iocshArg * const OmssendReadArgs[2] = {&sendReadArg0, &sendReadArg1};
static const iocshFuncDef sendReadMAXnet = {"MAXnetSendRead", 2, OmssendReadArgs};
static void sendReadMAXnetCallFunc(const iocshArgBuf *args)
{
    MAXnetSendRead(args[0].ival, args[1].sval);
}

static void OmsMAXnetRegister(void)
{

    iocshRegister(&setupMAXnet,      setupMAXnetCallFunc);
    iocshRegister(&configMAXnet,     configMAXnetCallFunc);
    iocshRegister(&sendReadMAXnet,   sendReadMAXnetCallFunc);
}

epicsExportRegistrar(OmsMAXnetRegister);
