/* drvXPSAsynAux.c */

/* This driver implements "auxilliary" commands for the XPS controller, i.e.
 * commands beyond those for the motor record.  These include support for
 * analog and digital I/O. */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <cantProceed.h> /* !! for callocMustSucceed() */

#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <errlog.h>
#include <iocsh.h>

#include <asynDriver.h>
#include <asynFloat64.h>
#include <asynUInt32Digital.h>
#include <asynDrvUser.h>

#include <XPS_C8_drivers.h>

typedef struct {
    char *portName;
    int socketID;
    epicsMutexId lock;
    epicsEventId pollerEventId;
    double pollerTimeout;
    asynInterface common;
    asynInterface float64;
    void *float64InterruptPvt;
    asynInterface uint32D;
    void *uint32DInterruptPvt;
    asynInterface drvUser;
    asynUser *pasynUser;
} drvXPSAsynAuxPvt;

typedef enum {
    analogInput,
    analogOutput,
    analogGain,
    binaryInput,
    binaryOutput
} drvXPSAsynAuxCommand;

typedef struct {
    drvXPSAsynAuxCommand command;
    char *commandString;
} drvXPSAsynAuxCommandStruct;

static drvXPSAsynAuxCommandStruct drvXPSAsynAuxCommands[] = {
    {analogInput,  "ANALOG_INPUT"},
    {analogOutput, "ANALOG_OUTPUT"},
    {analogGain,   "ANALOG_GAIN"},
    {binaryInput,  "BINARY_INPUT"},
    {binaryOutput, "BINARY_OUTPUT"}
};

#define TCP_TIMEOUT 1.0

#define MAX_ANALOG_INPUTS   4
#define MAX_ANALOG_OUTPUTS  4
#define MAX_DIGITAL_INPUTS  4
#define MAX_DIGITAL_OUTPUTS 3

static char *analogInputNames[MAX_ANALOG_INPUTS] = {
    "GPIO2.ADC1", /* Analog Input # 1 of the I/O board connector # 2 */
    "GPIO2.ADC2", /* Analog Input # 2 of the I/O board connector # 2 */
    "GPIO2.ADC3", /* Analog Input # 3 of the I/O board connector # 2 */
    "GPIO2.ADC4", /* Analog Input # 4 of the I/O board connector # 2 */
};
static char *analogOutputNames[MAX_ANALOG_OUTPUTS] = {
    "GPIO2.DAC1", /* Analog Output # 1 of the I/O board connector # 2 */
    "GPIO2.DAC2", /* Analog Output # 2 of the I/O board connector # 2 */
    "GPIO2.DAC3", /* Analog Output # 3 of the I/O board connector # 2 */
    "GPIO2.DAC4", /* Analog Output # 4 of the I/O board connector # 2 */
};
static char *digitalInputNames[MAX_DIGITAL_INPUTS] = {
    "GPIO1.DI", /* Digital Input of the I/O board connector # 1 (8 bits) */
    "GPIO2.DI", /* Digital Input of the I/O board connector # 2 (6 bits) */
    "GPIO3.DI", /* Digital Input of the I/O board connector # 3 (6 bits) */
    "GPIO4.DI", /* Digital Input of the I/O board connector # 4 (16 bits) */
};
static char *digitalOutputNames[MAX_DIGITAL_OUTPUTS] = {
    "GPIO1.DO", /* Digital Output of the I/O board connector # 1 (8 bits) */
    "GPIO3.DO", /* Digital Output of the I/O board connector # 3 (6 bits) */
    "GPIO4.DO", /* Digital Output of the I/O board connector # 4 (16 bits) */
};

/* These functions are used by the interfaces */
static asynStatus readFloat64        (void *drvPvt, asynUser *pasynUser,
                                     epicsFloat64 *value);
static asynStatus writeFloat64       (void *drvPvt, asynUser *pasynUser,
                                     epicsFloat64 value);
static asynStatus readUInt32D       (void *drvPvt, asynUser *pasynUser,
                                     epicsUInt32 *value, epicsUInt32 mask);
static asynStatus writeUInt32D      (void *drvPvt, asynUser *pasynUser,
                                     epicsUInt32 value, epicsUInt32 mask);
static asynStatus drvUserCreate     (void *drvPvt, asynUser *pasynUser,
                                     const char *drvInfo,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType    (void *drvPvt, asynUser *pasynUser,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy    (void *drvPvt, asynUser *pasynUser);

static void report                  (void *drvPvt, FILE *fp, int details);
static asynStatus connect           (void *drvPvt, asynUser *pasynUser);
static asynStatus disconnect        (void *drvPvt, asynUser *pasynUser);


static asynCommon drvXPSAsynAuxCommon = {
    report,
    connect,
    disconnect
};

static asynFloat64 drvXPSAsynAuxFloat64 = {
    writeFloat64,
    readFloat64
};

static asynUInt32Digital drvXPSAsynAuxUInt32D = {
    writeUInt32D,
    readUInt32D,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

static asynDrvUser drvXPSAsynAuxDrvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};

static void XPSAuxPoller(drvXPSAsynAuxPvt *pPvt);


int XPSAuxConfig(const char *portName, /* asyn port name */
                 const char *ip,       /* XPS IP address or IP name */
                 int port,             /* IP port number that XPS is listening on */
                 int pollPeriod)       /* Time to poll (msec) analog and digital inputs */

{
    drvXPSAsynAuxPvt *pPvt;
    asynStatus status;
    epicsThreadId threadId;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "XPSAuxConfig");
    pPvt->portName = epicsStrDup(portName);
    pPvt->lock = epicsMutexCreate();
    pPvt->pollerEventId = epicsEventCreate(epicsEventEmpty);

    pPvt->socketID = TCP_ConnectToServer((char *)ip, port, TCP_TIMEOUT);
    if (pPvt->socketID < 0) {
        printf("drvXPSAsynAuxConfig: error calling TCP_ConnectToServer\n");
        return -1;
    }
    pPvt->pollerTimeout = pollPeriod/1000.;

    /* Link with higher level routines */
    pPvt->common.interfaceType = asynCommonType;
    pPvt->common.pinterface  = (void *)&drvXPSAsynAuxCommon;
    pPvt->common.drvPvt = pPvt;
    pPvt->float64.interfaceType = asynFloat64Type;
    pPvt->float64.pinterface  = (void *)&drvXPSAsynAuxFloat64;
    pPvt->float64.drvPvt = pPvt;
    pPvt->uint32D.interfaceType = asynUInt32DigitalType;
    pPvt->uint32D.pinterface  = (void *)&drvXPSAsynAuxUInt32D;
    pPvt->uint32D.drvPvt = pPvt;
    pPvt->drvUser.interfaceType = asynDrvUserType;
    pPvt->drvUser.pinterface  = (void *)&drvXPSAsynAuxDrvUser;
    pPvt->drvUser.drvPvt = pPvt;
    status = pasynManager->registerPort(portName,
                                        ASYN_MULTIDEVICE, /*is multiDevice*/
                                        1,  /*  autoconnect */
                                        0,  /* medium priority */
                                        0); /* default stack size */
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig ERROR: Can't register port\n");
        return -1;
    }
    status = pasynManager->registerInterface(portName,&pPvt->common);
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig ERROR: Can't register common.\n");
        return -1;
    }
    status = pasynFloat64Base->initialize(pPvt->portName,&pPvt->float64);
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig ERROR: Can't register float64\n");
        return -1;
    }
    status = pasynManager->registerInterruptSource(pPvt->portName, &pPvt->float64,
                                                   &pPvt->float64InterruptPvt);
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig ERROR: Can't register float64 interrupts\n");
        return -1;
    }

    status = pasynUInt32DigitalBase->initialize(pPvt->portName,&pPvt->uint32D);
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig ERROR: Can't register uint32D\n");
        return -1;
    }
    status = pasynManager->registerInterruptSource(pPvt->portName, &pPvt->uint32D,
                                                   &pPvt->uint32DInterruptPvt);
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig ERROR: Can't register uint32D interrupts\n");
        return -1;
    }

    status = pasynManager->registerInterface(portName,&pPvt->drvUser);
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig ERROR: Can't register drvUser.\n");
        return -1;
    }

    /* Create asynUser for debugging */
    pPvt->pasynUser = pasynManager->createAsynUser(0, 0);

    /* Connect to device */
    status = pasynManager->connectDevice(pPvt->pasynUser, portName, -1);
    if (status != asynSuccess) {
        errlogPrintf("XPSAuxConfig, connectDevice failed\n");
        return -1;
    }
    threadId = epicsThreadCreate("XPSAuxPoller", epicsThreadPriorityMedium,
                                  epicsThreadGetStackSize(epicsThreadStackMedium),
                                 (EPICSTHREADFUNC)XPSAuxPoller,
                                  pPvt);
    if (threadId == NULL) {
        errlogPrintf("XPSAuxConfig, epicsThreadCreate failed\n");
        return(-1);
    }

    return 0;
}

static asynStatus readFloat64(void *drvPvt, asynUser *pasynUser,
                              epicsFloat64 *value)
{
    drvXPSAsynAuxPvt *pPvt = (drvXPSAsynAuxPvt *)drvPvt;
    int channel;
    drvXPSAsynAuxCommand command = pasynUser->reason;
    char *GPIOName;
    int status;

    pasynManager->getAddr(pasynUser, &channel);

    switch(command) {
    case analogInput:
        if ((channel < 0) || (channel >= MAX_ANALOG_INPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::readFloat64 channel out of range=%d",
                          channel);
            return(asynError);
        }
        GPIOName = analogInputNames[channel];
        break;
    case analogOutput:
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::readFloat64 channel out of range=%d",
                          channel);
            return(asynError);
        }
        GPIOName = analogOutputNames[channel];
        break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvXPSAsynAux::readFloat64 invalid command=%d",
                      command);
        return(asynError);
    }
    status = GPIOAnalogGet(pPvt->socketID, 1, GPIOName, value);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvXPSAsynAux::readFloat64 error calling GPIOAnalogGet=%d",
                      status);
        return(asynError);
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvXPSAsynAux::readFloat64, value=%f\n", *value);
    return(asynSuccess);
}


static asynStatus writeFloat64(void *drvPvt, asynUser *pasynUser,
                               epicsFloat64 value)
{
    drvXPSAsynAuxPvt *pPvt = (drvXPSAsynAuxPvt *)drvPvt;
    int channel;
    drvXPSAsynAuxCommand command = pasynUser->reason;
    char *GPIOName;
    int status;

    pasynManager->getAddr(pasynUser, &channel);

    switch(command) {
    case analogOutput:
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::writeFloat64 channel out of range=%d",
                          channel);
            return(asynError);
        }
        GPIOName = analogOutputNames[channel];
        break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvXPSAsynAux::writeFloat64 invalid command=%d",
                      command);
        return(asynError);
    }
    status = GPIOAnalogSet(pPvt->socketID, 1, GPIOName, &value);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvXPSAsynAux::writeFloat64 error calling GPIOAnalogSet=%d",
                      status);
        return(asynError);
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvXPSAsynAux::writeFloat64, value=%f\n", value);
    return(asynSuccess);
}

static asynStatus readUInt32D(void *drvPvt, asynUser *pasynUser,
                              epicsUInt32 *value, epicsUInt32 mask)
{
    drvXPSAsynAuxPvt *pPvt = (drvXPSAsynAuxPvt *)drvPvt;
    int channel;
    drvXPSAsynAuxCommand command = pasynUser->reason;
    int status;
    unsigned short rawValue;

    pasynManager->getAddr(pasynUser, &channel);

    switch(command) {
    case binaryInput:
        if ((channel < 0) || (channel >= MAX_DIGITAL_INPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::readUInt32D readBi channel out of range=%d",
                          channel);
            return(asynError);
        }
        status = GPIODigitalGet(pPvt->socketID, digitalInputNames[channel], &rawValue);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::readUInt32D error calling GPIODigitalGet=%d",
                          status);
            return(asynError);
        }
        *value = rawValue & mask;
        break;
    case binaryOutput:
        if ((channel < 0) || (channel >= MAX_DIGITAL_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::readUInt32D readBo channel out of range=%d",
                          channel);
            return(asynError);
        }
        status = GPIODigitalGet(pPvt->socketID, digitalOutputNames[channel], &rawValue);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::readUInt32D error calling GPIODigitalGet=%d",
                          status);
            return(asynError);
        }
        *value = rawValue & mask;
        break;
    case analogGain:
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::readUInt32D channel out of range=%d",
                          channel);
            return(asynError);
        }
        status = GPIOAnalogGainGet(pPvt->socketID, 1, analogInputNames[channel], (int*)value);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::writeUInt32D error calling GPIOAnalogGainSet=%d",
                          status);
            return(asynError);
        }
        break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvXPSAsynAux::readUInt32D invalid command=%d",
                      command);
        return(asynError);
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "drvXPSAsynAux::readUInt32D, value=%x\n", *value);
    return(asynSuccess);
}

static asynStatus writeUInt32D(void *drvPvt, asynUser *pasynUser,
                               epicsUInt32 value, epicsUInt32 mask)
{
    drvXPSAsynAuxPvt *pPvt = (drvXPSAsynAuxPvt *)drvPvt;
    int channel;
    drvXPSAsynAuxCommand command = pasynUser->reason;
    char *GPIOName;
    int status;

    pasynManager->getAddr(pasynUser, &channel);

    switch(command) {
    case binaryOutput:
        if ((channel < 0) || (channel >= MAX_DIGITAL_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::writeUInt32D channel out of range=%d",
                          channel);
            return(asynError);
        }
        GPIOName = digitalOutputNames[channel];
        status = GPIODigitalSet(pPvt->socketID, GPIOName, mask, value);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::writeUInt32D error calling GPIODigitalSet=%d",
                          status);
            return(asynError);
        }
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "drvXPSAsynAux::writeUInt32D, set binary output value=%d\n", value);
        break;
    case analogGain:
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::writeUInt32D channel out of range=%d",
                          channel);
            return(asynError);
        }
        GPIOName = analogInputNames[channel];
        status = GPIOAnalogGainSet(pPvt->socketID, 1, GPIOName, (int*)&value);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "drvXPSAsynAux::writeUInt32D error calling GPIOAnalogGainSet=%d",
                          status);
            return(asynError);
        }
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "drvXPSAsynAux::writeUInt32D, set gain value=%d\n", value);
        break;
    default:
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvXPSAsynAux::writeUInt32D invalid command=%d",
                      command);
        return(asynError);
    }
    return(asynSuccess);
}

static void XPSAuxPoller(drvXPSAsynAuxPvt *pPvt)
{
    char analogNames[100] = "";
    double analogValues[MAX_ANALOG_INPUTS];
    unsigned short digitalValues[MAX_DIGITAL_INPUTS];
    unsigned short digitalValuesPrev[MAX_DIGITAL_INPUTS];
    ELLLIST *pclientList;
    interruptNode *pnode;
    asynUInt32DigitalInterrupt *pUInt32DigitalInterrupt;
    asynFloat64Interrupt *pfloat64Interrupt;
    int firstTime = 1;
    int i;
    int status;
    asynUser *pasynUser;
    int addr, reason, mask, changedBits;

    /* Build strings with the names of the analog and digital inputs */
    for (i=0; i<MAX_ANALOG_INPUTS; i++) {
        strcat(analogNames, analogInputNames[i]);
        strcat(analogNames, ";");
    }

    while(1) {
        status = epicsEventWaitWithTimeout(pPvt->pollerEventId, pPvt->pollerTimeout);
        status = GPIOAnalogGet(pPvt->socketID, MAX_ANALOG_INPUTS, analogNames, analogValues);
        if (status) {
            asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR,
                      "drvXPSAsynAux::XPSAuxPoller error calling GPIOAnalogGet=%d\n", status);
        }
        for (i=0; i<MAX_DIGITAL_INPUTS; i++) {
            status = GPIODigitalGet(pPvt->socketID, digitalInputNames[i], &digitalValues[i]);
            if (status) {
                asynPrint(pPvt->pasynUser, ASYN_TRACE_ERROR,
                          "drvXPSAsynAux::XPSAuxPoller error calling GPIODigitalGet=%d\n", status);
            }
        }

        /* Call back any clients who have registered for callbacks on changed digital bits */
        pasynManager->interruptStart(pPvt->uint32DInterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            pUInt32DigitalInterrupt = pnode->drvPvt;
            pasynUser = pUInt32DigitalInterrupt->pasynUser;
            pasynManager->getAddr(pasynUser, &addr);
            reason = pasynUser->reason;
            mask = pUInt32DigitalInterrupt->mask;
            changedBits = digitalValues[addr] ^ digitalValuesPrev[addr];
            if (firstTime) changedBits = 0xffff;
            if ((mask & changedBits) && (reason == binaryInput)) {
                pUInt32DigitalInterrupt->callback(pUInt32DigitalInterrupt->userPvt, pasynUser,
                                                  mask & digitalValues[addr]);
            }
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->uint32DInterruptPvt);
        for (i=0; i<MAX_DIGITAL_INPUTS; i++) {
            digitalValuesPrev[i] = digitalValues[i];
        }

        /* Pass float64 interrupts for analog inputs*/
        pasynManager->interruptStart(pPvt->float64InterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            pfloat64Interrupt = pnode->drvPvt;
            addr = pfloat64Interrupt->addr;
            reason = pfloat64Interrupt->pasynUser->reason;
            if (reason == analogInput) {
                pfloat64Interrupt->callback(pfloat64Interrupt->userPvt,
                                            pfloat64Interrupt->pasynUser,
                                            analogValues[addr]);
            }
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->float64InterruptPvt);
        firstTime = 0;
    }
}


/* asynDrvUser routines */
static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
                                const char *drvInfo,
                                const char **pptypeName, size_t *psize)
{
    int i;
    char *pstring;
    int ncommands = sizeof(drvXPSAsynAuxCommands)/sizeof(drvXPSAsynAuxCommands[0]);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvXPSAsynAux::drvUserCreate, drvInfo=%s, pptypeName=%p, psize=%p, pasynUser=%p\n", 
              drvInfo, pptypeName, psize, pasynUser);

    for (i=0; i < ncommands; i++) {
        pstring = drvXPSAsynAuxCommands[i].commandString;
        if (epicsStrCaseCmp(drvInfo, pstring) == 0) {
            break;
        }
    }
    if (i < ncommands) {
        pasynUser->reason = drvXPSAsynAuxCommands[i].command;
        if (pptypeName) {
            *pptypeName = epicsStrDup(pstring);
        }
        if (psize) {
            *psize = sizeof(drvXPSAsynAuxCommands[i].command);
        }
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "drvXPSAsynAux::drvUserCreate, command=%d string=%s\n", 
                  pasynUser->reason, pstring);
        return(asynSuccess);
    } else {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "drvXPSAsynAux::drvUserCreate, unknown command=%s", drvInfo);
        return(asynError);
    }
}

static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
                                 const char **pptypeName, size_t *psize)
{
    drvXPSAsynAuxCommand command = pasynUser->reason;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvXPSAsynAux::drvUserGetType entered");

    *pptypeName = NULL;
    *psize = 0;
    if (pptypeName)
        *pptypeName = epicsStrDup(drvXPSAsynAuxCommands[command].commandString);
    if (psize) *psize = sizeof(command);
    return(asynSuccess);
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvXPSAsynAux::drvUserDestroy, drvPvt=%p, pasynUser=%p\n",
              drvPvt, pasynUser);

    return(asynSuccess);
}

/* asynCommon routines */

/* Report  parameters */
static void report(void *drvPvt, FILE *fp, int details)
{
    drvXPSAsynAuxPvt *pPvt = (drvXPSAsynAuxPvt *)drvPvt;
    interruptNode *pnode;
    ELLLIST *pclientList;

    fprintf(fp, "Port: %s\n", pPvt->portName);
    if (details >= 1) {
        /* Report uint32D interrupts */
        pasynManager->interruptStart(pPvt->uint32DInterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynUInt32DigitalInterrupt *puint32DInterrupt = pnode->drvPvt;
            fprintf(fp, "    int32 callback client address=%p, addr=%d, reason=%d\n",
                    puint32DInterrupt->callback, puint32DInterrupt->addr,
                    puint32DInterrupt->pasynUser->reason);
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->uint32DInterruptPvt);

        /* Report float64 interrupts */
        pasynManager->interruptStart(pPvt->float64InterruptPvt, &pclientList);
        pnode = (interruptNode *)ellFirst(pclientList);
        while (pnode) {
            asynFloat64Interrupt *pfloat64Interrupt = pnode->drvPvt;
            fprintf(fp, "    float64 callback client address=%p, addr=%d, reason=%d\n",
                    pfloat64Interrupt->callback, pfloat64Interrupt->addr,
                    pfloat64Interrupt->pasynUser->reason);
            pnode = (interruptNode *)ellNext(&pnode->node);
        }
        pasynManager->interruptEnd(pPvt->float64InterruptPvt);
    }
}

/* Connect */
static asynStatus connect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionConnect(pasynUser);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "drvXPSAsynAux::connect, pasynUser=%p\n", pasynUser);
    return(asynSuccess);
}

/* Disconnect */
static asynStatus disconnect(void *drvPvt, asynUser *pasynUser)
{
    pasynManager->exceptionDisconnect(pasynUser);
    return(asynSuccess);
}

static const iocshArg configArg0 = { "portName",iocshArgString};
static const iocshArg configArg1 = { "IP address",iocshArgString};
static const iocshArg configArg2 = { "IP port",iocshArgInt};
static const iocshArg configArg3 = { "polling period",iocshArgInt};
static const iocshArg * const configArgs[4] = {&configArg0,
                                               &configArg1,
                                               &configArg2,
                                               &configArg3};
static const iocshFuncDef configFuncDef = {"XPSAuxConfig",4,configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    XPSAuxConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

void drvXPSAsynAuxRegister(void)
{
    iocshRegister(&configFuncDef,configCallFunc);
}

epicsExportRegistrar(drvXPSAsynAuxRegister);
