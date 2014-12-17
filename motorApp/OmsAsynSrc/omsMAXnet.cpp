/*
FILENAME...     omsMAXnet.cpp
USAGE...        Pro-Dex OMS MAXnet asyn motor controller support

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *  Created on: 10/2010
 *      Author: eden
 */

#include <string.h>

#include "asynOctetSyncIO.h"
#include "omsMAXnet.h"

#ifdef __GNUG__
    #ifdef      DEBUG
        #define Debug(l, f, args...) {if (l & motorMAXnetdebug) \
                                  errlogPrintf(f, ## args);}
    #else
        #define Debug(l, f, args...)
    #endif
#else
    #define Debug
#endif

static const char *driverName = "omsMAXnetDriver";
volatile int motorMAXnetdebug = 0;
extern "C" {epicsExportAddress(int, motorMAXnetdebug);}

#define MAXnet_MAX_BUFFERLENGTH 250

static void connectCallback(asynUser *pasynUser, asynException exception)
{
    asynStatus status;
    int connected = 0;
    omsMAXnet* pController = (omsMAXnet*)pasynUser->userPvt;

    if (exception == asynExceptionConnect) {
        status = pasynManager->isConnected(pasynUser, &connected);
        if (connected){
            if (motorMAXnetdebug & 8) asynPrint(pasynUser, ASYN_TRACE_FLOW,
                "MAXnet connectCallback:  TCP-Port connected\n");
            pController->portConnected = 1;
        }
        else {
            if (motorMAXnetdebug & 4) asynPrint(pasynUser, ASYN_TRACE_FLOW,
                "MAXnet connectCallback:  TCP-Port disconnected\n");
            pController->portConnected = 0;
        }
    }
}

void omsMAXnet::asynCallback(void *drvPvt, asynUser *pasynUser, char *data, size_t len, int eomReason)
{
    omsMAXnet* pController = (omsMAXnet*)drvPvt;

/* If the string has a "%", it is a notification, increment counter and
 * send a signal to the poller task which will trigger a poll */

    if ((len >= 1) && (strchr(data, '%') != NULL)){
        char* pos = strchr(data, '%');
        epicsEventSignal(pController->pollEventId_);
        while (pos != NULL){
            Debug(2, "omsMAXnet::asynCallback: %s (%d)\n", data, len);
            pController->notificationMutex->lock();
            ++pController->notificationCounter;
            pController->notificationMutex->unlock();
            ++pos;
            pos = strchr(pos, '%');
        }
    }
}

omsMAXnet::omsMAXnet(const char* portName, int numAxes, const char* serialPortName, const char* initString, int priority, int stackSize)
    : omsBaseController(portName, numAxes, priority, stackSize, 0){

    asynStatus status;
    asynInterface *pasynInterface;

    controllerType = epicsStrDup("MAXnet");

    notificationMutex = new epicsMutex();
    notificationCounter = 0;
    useWatchdog = true;
    char eosstring[5];
    int eoslen=0;

    serialPortName = epicsStrDup(serialPortName);

    pasynUserSerial = pasynManager->createAsynUser(0,0);
    pasynUserSerial->userPvt = this;

    status = pasynManager->connectDevice(pasynUserSerial,serialPortName,0);
    if(status != asynSuccess){
        printf("MAXnetConfig: can't connect to port %s: %s\n",serialPortName,pasynUserSerial->errorMessage);
        return;
    }

    status =  pasynManager->exceptionCallbackAdd(pasynUserSerial, connectCallback);
    if(status != asynSuccess){
        printf("MAXnetConfig: can't set exceptionCallback for %s: %s\n",serialPortName,pasynUserSerial->errorMessage);
        return;
    }
    /* set the connect flag */
    pasynManager->isConnected(pasynUserSerial, &portConnected);

    pasynInterface = pasynManager->findInterface(pasynUserSerial,asynOctetType,1);
    if( pasynInterface == NULL) {
        printf("MAXnetConfig: %s driver not supported\n", asynOctetType);
        return;
    }
    pasynOctetSerial = (asynOctet*)pasynInterface->pinterface;
    octetPvtSerial = pasynInterface->drvPvt;

    status = pasynOctetSyncIO->connect(serialPortName, 0, &pasynUserSyncIOSerial, NULL);
    if(status != asynSuccess){
        printf("MAXnetConfig: can't connect pasynOctetSyncIO %s: %s\n",serialPortName,pasynUserSyncIOSerial->errorMessage);
        return;
    }

    /* flush any junk at input port - should be no data available */
    pasynOctetSyncIO->flush(pasynUserSyncIOSerial);

    timeout = 2.0;
    pasynUserSerial->timeout = 0.0;

    // to override default setting, set input and output EOS in st.cmd
    if (pasynOctetSyncIO->getInputEos(pasynUserSyncIOSerial, eosstring, 5, &eoslen) == asynSuccess) {
        if (eoslen == 0)
            if (pasynOctetSyncIO->setInputEos(pasynUserSyncIOSerial, "\n\r", 2) != asynSuccess)
                printf("MAXnetConfig: unable to set InputEOS %s: %s\n", serialPortName, pasynUserSyncIOSerial->errorMessage);
    }
    if (pasynOctetSyncIO->getOutputEos(pasynUserSyncIOSerial, eosstring, 5, &eoslen) == asynSuccess) {
        if (eoslen == 0)
            if (pasynOctetSyncIO->setOutputEos(pasynUserSyncIOSerial, "\n", 1) != asynSuccess)
                printf("MAXnetConfig: unable to set OutputEOS %s: %s\n", serialPortName, pasynUserSyncIOSerial->errorMessage);
    }

    void* registrarPvt= NULL;
    status = pasynOctetSerial->registerInterruptUser(octetPvtSerial, pasynUserSerial, omsMAXnet::asynCallback, this, &registrarPvt);
    if(status != asynSuccess) {
        printf("MAXnetConfig: registerInterruptUser failed - %s: %s\n",serialPortName,pasynUserSerial->errorMessage);
        return;
    }

    /* get FirmwareVersion */
    if(getFirmwareVersion() != asynSuccess) {
        printf("MAXnetConfig: unable to talk to controller at %s: %s\n",serialPortName,pasynUserSyncIOSerial->errorMessage);
        return;
    }
    if (fwMinor < 30 ){
        printf("This Controllers Firmware Version %d.%d is not supported, version 1.30 or higher is mandatory\n", fwMajor, fwMinor);
    }

    if( Init(initString, 0) != asynSuccess) {
        printf("MAXnetConfig: unable to talk to controller at %s: %s\n",serialPortName,pasynUserSyncIOSerial->errorMessage);
        return;
    }
}

// poll the serial port for notification messages while waiting
epicsEventWaitStatus omsMAXnet::waitInterruptible(double timeout)
{
    double pollWait, timeToWait = timeout;
    asynStatus status;
    size_t nRead;
    char inputBuff[1];
    int eomReason = 0;
    epicsTimeStamp starttime;
    epicsEventWaitStatus waitStatus = epicsEventWaitTimeout;
    epicsTimeGetCurrent(&starttime);

    if (timeout == idlePollPeriod_)
        pollWait = idlePollPeriod_ / 5.0;
    else
        pollWait = movingPollPeriod_ / 20.0;

    pasynManager->lockPort(pasynUserSerial);
    pasynOctetSerial->flush(octetPvtSerial, pasynUserSerial);
    pasynManager->unlockPort(pasynUserSerial);
    while ( timeToWait > 0){
        /* reading with bufferlength 0 and timeout 0.0 triggers a callback and
         * poll event, if a notification is outstanding. One character will be read. */
        if (enabled) {
            pasynManager->lockPort(pasynUserSerial);
            status = pasynOctetSerial->read(octetPvtSerial, pasynUserSerial, inputBuff,
                                                0, &nRead, &eomReason);
            pasynManager->unlockPort(pasynUserSerial);
        }
        if (epicsEventWaitWithTimeout(pollEventId_, pollWait) == epicsEventWaitOK) {
            waitStatus = epicsEventWaitOK;
            break;
        }
        epicsTimeGetCurrent(&now);
        timeToWait = timeout - epicsTimeDiffInSeconds(&now, &starttime);
    }
    return waitStatus;
}

asynStatus omsMAXnet::sendOnly(const char *outputBuff)
{
    size_t nActual = 0;
    asynStatus status;

    if (!enabled) return asynError;

    status = pasynOctetSyncIO->write(pasynUserSyncIOSerial, outputBuff,
                             strlen(outputBuff), timeout, &nActual);

    if (status != asynSuccess) {
        asynPrint(pasynUserSyncIOSerial, ASYN_TRACE_ERROR,
                  "drvMAXnetAsyn:sendOnly: error sending command %s, sent=%d, status=%d\n",
                  outputBuff, nActual, status);
    }
    Debug(4, "omsMAXnet::sendOnly: wrote: %s \n", outputBuff);
    return(status);
}

asynStatus omsMAXnet::sendReceive(const char *outputBuff, char *inputBuff, unsigned int inputSize)
{
    char localBuffer[MAXnet_MAX_BUFFERLENGTH + 1] = "";
    size_t nRead=0, nReadnext=0, nWrite=0;
    size_t bufferSize = MAXnet_MAX_BUFFERLENGTH;
    int eomReason = 0;
    asynStatus status = asynSuccess;
    char *outString = localBuffer;
    int errorCount = 10;

    if (!enabled) return asynError;

    /*
     * read the notification from input buffer
     */
    while ((notificationCounter > 0) && errorCount){
        status = pasynOctetSyncIO->read(pasynUserSyncIOSerial, localBuffer, bufferSize, 0.001, &nRead, &eomReason);
        while ((status == asynSuccess) && !(eomReason & ASYN_EOM_EOS)) {
            status = pasynOctetSyncIO->read(pasynUserSyncIOSerial, localBuffer+nRead,
                                                 bufferSize-nRead, timeout, &nReadnext, &eomReason);
            nRead += nReadnext;
        }
        localBuffer[nRead] = '\0';
        outString = localBuffer;
    	while (*outString == 6) ++outString;
        if (status == asynSuccess) {
            if (isNotification(outString) && (notificationCounter > 0)) {
                --notificationCounter;
            }
        }
        else if (status == asynTimeout) {
            notificationCounter = 0;
        }
        else {
            --errorCount;
        }
    }

    Debug(4, "omsMAXnet::sendReceive: write: %s \n", outputBuff);
    nRead=0;
    nReadnext=0;
    status = pasynOctetSyncIO->writeRead(pasynUserSyncIOSerial, outputBuff, strlen(outputBuff), localBuffer,
                                        bufferSize, timeout, &nWrite, &nRead, &eomReason);

    while ((status == asynSuccess) && !(eomReason & ASYN_EOM_EOS)) {
        status = pasynOctetSyncIO->read(pasynUserSyncIOSerial, localBuffer+nRead,
                                             bufferSize-nRead, timeout, &nReadnext, &eomReason);
        nRead += nReadnext;
    }
    localBuffer[nRead] = '\0';
	// cut off a leading /006
    outString = localBuffer;
	while (*outString == 6) ++outString;

	// if input data is a notification read until expected data arrived
    while ((status == asynSuccess) && isNotification(outString)) {
        nRead=0;
        nReadnext=0;
        status = pasynOctetSyncIO->read(pasynUserSyncIOSerial, localBuffer,
                                             bufferSize, timeout, &nRead, &eomReason);
        while ((status == asynSuccess) && !(eomReason & ASYN_EOM_EOS)) {
            status = pasynOctetSyncIO->read(pasynUserSyncIOSerial, localBuffer+nRead,
                                                 bufferSize-nRead, timeout, &nReadnext, &eomReason);
            nRead += nReadnext;
        }
        localBuffer[nRead] = '\0';
    	// cut off a leading /006
        outString = localBuffer;
    	while (*outString == 6) ++outString;
    	if (notificationCounter > 0) --notificationCounter;
    }

	// copy into inputBuffer
	strncpy(inputBuff, outString, inputSize);
	inputBuff[inputSize-1] = '\0';

    Debug(4, "omsMAXnet::sendReceive: read: %s \n", inputBuff);

    return status;
}

/*
 * check if buffer is a notification messages with 13 chars ("%000 SSSSSSSS")
 * (first character may miss
 */
int omsMAXnet::isNotification (char *buffer) {

    const char* functionName="isNotification";
    char *inString;
    if ((inString = strstr(buffer, "000 0")) != NULL){
        if ((inString = strstr(buffer, "000 01")) != NULL){
            printf("%s:%s:%s: CMD_ERR_FLAG received\n", driverName, functionName, portName);
        }
        else {
            Debug(2,"%s:%s:%s: Interrupt notification: %s\n",
            		driverName, functionName, portName, buffer);
            epicsEventSignal(pollEventId_);
        }
        return 1;
    }
    else
        return 0;
}

extern "C" int omsMAXnetConfig(
              const char *portName,      /* MAXnet Motor Asyn Port name */
              int numAxes,               /* Number of axes this controller supports */
              const char *serialPortName,/* MAXnet Serial Asyn Port name */
              int movingPollPeriod,      /* Time to poll (msec) when an axis is in motion */
              int idlePollPeriod,        /* Time to poll (msec) when an axis is idle. 0 for no polling */
              const char *initString)    /* Init String sent to card */
{
    // for now priority and stacksize are hardcoded here, should they be configurable in omsMAXnetConfig?
    int priority = epicsThreadPriorityMedium;
    int stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
    omsMAXnet *pController = new omsMAXnet(portName, numAxes, serialPortName, initString, priority, stackSize);
    pController->startPoller((double)movingPollPeriod, (double)idlePollPeriod, 10);
    return(asynSuccess);
}

/* Code for iocsh registration */

/* omsMAXnetConfig */
static const iocshArg omsMAXnetConfigArg0 = {"asyn motor port name", iocshArgString};
static const iocshArg omsMAXnetConfigArg1 = {"number of axes", iocshArgInt};
static const iocshArg omsMAXnetConfigArg2 = {"asyn serial/tcp port name", iocshArgString};
static const iocshArg omsMAXnetConfigArg3 = {"moving poll rate", iocshArgInt};
static const iocshArg omsMAXnetConfigArg4 = {"idle poll rate", iocshArgInt};
static const iocshArg omsMAXnetConfigArg5 = {"initstring", iocshArgString};
static const iocshArg * const omsMAXnetConfigArgs[6] = {&omsMAXnetConfigArg0,
                                                  &omsMAXnetConfigArg1,
                                                  &omsMAXnetConfigArg2,
                                                  &omsMAXnetConfigArg3,
                                                  &omsMAXnetConfigArg4,
                                                  &omsMAXnetConfigArg5 };
static const iocshFuncDef configOmsMAXnet = {"omsMAXnetConfig", 6, omsMAXnetConfigArgs};
static void configOmsMAXnetCallFunc(const iocshArgBuf *args)
{
    omsMAXnetConfig(args[0].sval, args[1].ival, args[2].sval, args[3].ival, args[4].ival, args[5].sval);
}

static void OmsMAXnetAsynRegister(void)
{
    iocshRegister(&configOmsMAXnet,     configOmsMAXnetCallFunc);
}

epicsExportRegistrar(OmsMAXnetAsynRegister);

