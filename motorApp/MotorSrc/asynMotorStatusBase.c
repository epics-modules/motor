/*  asynMotorStatusBase.c */
/***********************************************************************
* Copyright (c) 2002-6 The University of Chicago, as Operator of Argonne
* National Laboratory, and the Regents of the University of
* California, as Operator of Los Alamos National Laboratory, and
* Berliner Elektronenspeicherring-Gesellschaft m.b.H. (BESSY),
* and Diamond Light Source Ltd.
* asynDriver is distributed subject to a Software License Agreement
* found in file LICENSE that is included with this distribution.
***********************************************************************/

/*  20-June-2006 Peter Denison
*/

#include <epicsTypes.h>
#include <cantProceed.h>

#define epicsExportSharedSymbols
#include <shareLib.h>
#include "asynDriver.h"
#include "asynMotorStatus.h"

static asynStatus initialize(const char *portName, asynInterface *pmotorStatusInterface);

static asynMotorStatusBase motorStatusBase = {initialize};
epicsShareDef asynMotorStatusBase *pasynMotorStatusBase = &motorStatusBase;

static asynStatus writeDefault(void *drvPvt, asynUser *pasynUser,
                               struct MotorStatus *value);
static asynStatus readDefault(void *drvPvt, asynUser *pasynUser,
                              struct MotorStatus *value);
static asynStatus registerInterruptUser(void *drvPvt,asynUser *pasynUser,
                               interruptCallbackMotorStatus callback, void *userPvt,
                               void **registrarPvt);
static asynStatus cancelInterruptUser(void *drvPvt, asynUser *pasynUser,
                               void *registrarPvt);


asynStatus initialize(const char *portName, asynInterface *pdriver)
{
    asynMotorStatus *pasynMotorStatus = (asynMotorStatus *)pdriver->pinterface;

    if(!pasynMotorStatus->write) pasynMotorStatus->write = writeDefault;
    if(!pasynMotorStatus->read) pasynMotorStatus->read = readDefault;
    if(!pasynMotorStatus->registerInterruptUser)
        pasynMotorStatus->registerInterruptUser = registerInterruptUser;
    if(!pasynMotorStatus->cancelInterruptUser)
        pasynMotorStatus->cancelInterruptUser = cancelInterruptUser;
    return pasynManager->registerInterface(portName,pdriver);
}

static asynStatus writeDefault(void *drvPvt, asynUser *pasynUser,
    struct MotorStatus *value)
{
    const char *portName;
    asynStatus status;
    int        addr;
    
    status = pasynManager->getPortName(pasynUser,&portName);
    if(status!=asynSuccess) return status;
    status = pasynManager->getAddr(pasynUser,&addr);
    if(status!=asynSuccess) return status;
    epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
        "write is not supported\n");
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
        "%s %d write is not supported\n",portName,addr);
    return asynError;
}

static asynStatus readDefault(void *drvPvt, asynUser *pasynUser,
    struct MotorStatus *value)
{
    const char *portName;
    asynStatus status;
    int        addr;
    
    status = pasynManager->getPortName(pasynUser,&portName);
    if(status!=asynSuccess) return status;
    status = pasynManager->getAddr(pasynUser,&addr);
    if(status!=asynSuccess) return status;
    epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
        "read is not supported\n");
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
        "%s %d read is not supported\n",portName,addr);
    return asynError;
}


static asynStatus registerInterruptUser(void *drvPvt,asynUser *pasynUser,
      interruptCallbackMotorStatus callback, void *userPvt,void **registrarPvt)
{
    const char    *portName;
    asynStatus    status;
    int           addr;
    interruptNode *pinterruptNode;
    void          *pinterruptPvt;
    asynMotorStatusInterrupt *pasynMotorStatusInterrupt;

    status = pasynManager->getPortName(pasynUser,&portName);
    if(status!=asynSuccess) return status;
    status = pasynManager->getAddr(pasynUser,&addr);
    if(status!=asynSuccess) return status;
    status = pasynManager->getInterruptPvt(pasynUser, asynMotorStatusType,
                                           &pinterruptPvt);
    if(status!=asynSuccess) return status;
    pinterruptNode = pasynManager->createInterruptNode(pinterruptPvt);
    if(status!=asynSuccess) return status;
    pasynMotorStatusInterrupt = pasynManager->memMalloc(
                                             sizeof(asynMotorStatusInterrupt));
    pinterruptNode->drvPvt = pasynMotorStatusInterrupt;
    pasynMotorStatusInterrupt->pasynUser =
                       pasynManager->duplicateAsynUser(pasynUser, NULL, NULL);
    pasynMotorStatusInterrupt->addr = addr;
    pasynMotorStatusInterrupt->callback = callback;
    pasynMotorStatusInterrupt->userPvt = userPvt;
    *registrarPvt = pinterruptNode;
    asynPrint(pasynUser,ASYN_TRACE_FLOW,
        "%s %d registerInterruptUser\n",portName,addr);
    return pasynManager->addInterruptUser(pasynUser,pinterruptNode);
}

static asynStatus cancelInterruptUser(void *drvPvt, asynUser *pasynUser,void *registrarPvt)
{
    interruptNode *pinterruptNode = (interruptNode *)registrarPvt;
    asynStatus    status;
    const char    *portName;
    int           addr;
    asynMotorStatusInterrupt *pasynMotorStatusInterrupt =
                  (asynMotorStatusInterrupt *)pinterruptNode->drvPvt;
    
    status = pasynManager->getPortName(pasynUser,&portName);
    if(status!=asynSuccess) return status;
    status = pasynManager->getAddr(pasynUser,&addr);
    if(status!=asynSuccess) return status;
    asynPrint(pasynUser,ASYN_TRACE_FLOW,
        "%s %d cancelInterruptUser\n",portName,addr);
    status = pasynManager->removeInterruptUser(pasynUser,pinterruptNode);
    pasynManager->freeAsynUser(pasynMotorStatusInterrupt->pasynUser);
    pasynManager->memFree(pasynMotorStatusInterrupt, sizeof(asynMotorStatusInterrupt));
    return status;
}
