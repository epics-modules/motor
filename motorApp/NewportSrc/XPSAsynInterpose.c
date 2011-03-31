#include <stdio.h>
#include <cantProceed.h>
#include <epicsString.h>
#include <errlog.h>
#include <iocsh.h>
#include <asynDriver.h>
#include <asynDrvUser.h>
#include <XPSAsynInterpose.h>
#include <epicsExport.h>

typedef struct {
    XPSCommand command;
    char *commandString;
} XPSCommandStruct;

static XPSCommandStruct XPSCommands[XPS_NUM_PARAMS] = {
    {minJerkTime,     "XPS_MIN_JERK"},
    {maxJerkTime,     "XPS_MAX_JERK"},
    {XPSStatus,       "XPS_STATUS"}
};

typedef struct {
    char *portName;
    asynInterface drvUser;
    asynDrvUser *drvUserPrev;
    void *drvUserPrevPvt;
} XPSInterposePvt;

static asynStatus drvUserCreate     (void *drvPvt, asynUser *pasynUser,
                                     const char *drvInfo,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType    (void *drvPvt, asynUser *pasynUser,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy    (void *drvPvt, asynUser *pasynUser);



static asynDrvUser XPSDrvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};

int XPSInterpose(const char *portName)
{
    XPSInterposePvt *pPvt;
    asynInterface *drvUserPrev;
    asynStatus status;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "XPSInterpose");
    pPvt->portName = epicsStrDup(portName);

    pPvt->drvUser.interfaceType = asynDrvUserType;
    pPvt->drvUser.pinterface  = (void *)&XPSDrvUser;
    pPvt->drvUser.drvPvt = pPvt;

    status = pasynManager->interposeInterface(portName, -1, &pPvt->drvUser, &drvUserPrev);
    if (status != asynSuccess) {
        errlogPrintf("XPSInterpose ERROR: calling interpose interface.\n");
        return -1;
    }
    pPvt->drvUserPrev = drvUserPrev->pinterface;
    pPvt->drvUserPrevPvt = drvUserPrev->drvPvt;
    return(asynSuccess);
}


/* asynDrvUser routines */
static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
                                const char *drvInfo,
                                const char **pptypeName, size_t *psize)
{
    XPSInterposePvt *pPvt = (XPSInterposePvt *) drvPvt;
    int i;
    char *pstring;
    int ncommands = sizeof(XPSCommands)/sizeof(XPSCommands[0]);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "XPSInterpose::drvUserCreate, drvInfo=%s, pptypeName=%p, psize=%p, pasynUser=%p\n", 
               drvInfo, pptypeName, psize, pasynUser);

    for (i=0; i < ncommands; i++) {
        pstring = XPSCommands[i].commandString;
        if (epicsStrCaseCmp(drvInfo, pstring) == 0) {
            break;
        }
    }
    if (i < ncommands) {
        pasynUser->reason = XPSCommands[i].command;
        if (pptypeName) {
            *pptypeName = epicsStrDup(pstring);
        }
        if (psize) {
            *psize = sizeof(XPSCommands[i].command);
        }
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "XPSInterpose::drvUserCreate, command=%s\n", pstring);
        return(asynSuccess);
    } else {
        /* This command is not recognized, call the previous driver's routine */
        return(pPvt->drvUserPrev->create(pPvt->drvUserPrevPvt, pasynUser, drvInfo, pptypeName, psize));
    }
}

static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
                                 const char **pptypeName, size_t *psize)
{
    XPSInterposePvt *pPvt = (XPSInterposePvt *) drvPvt;
    XPSCommand command = pasynUser->reason;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "XPSInterpose::drvUserGetType entered");

    if ((command >= minJerkTime) &&
        (command <= maxJerkTime)) { 
        *pptypeName = NULL;
        *psize = 0;
        if (pptypeName)
            *pptypeName = epicsStrDup(XPSCommands[command].commandString);
        if (psize) *psize = sizeof(command);
        return(asynSuccess);
    } else {
        return(pPvt->drvUserPrev->getType(pPvt->drvUserPrevPvt, pasynUser,
                                                 pptypeName, psize));
    }
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
    XPSInterposePvt *pPvt = (XPSInterposePvt *) drvPvt;

    return(pPvt->drvUserPrev->destroy(pPvt->drvUserPrevPvt, pasynUser));
}

static const iocshArg XPSInterposeArg0 = {"Port Name", iocshArgString};
static const iocshArg * const XPSInterposeArgs[1] = {&XPSInterposeArg0};
 
static const iocshFuncDef XPSInterposeDef = {"XPSInterpose", 1, XPSInterposeArgs};

static void XPSInterposeCallFunc(const iocshArgBuf *args)
{
    XPSInterpose(args[0].sval);
}


static void XPSInterposeRegister(void)
{
    iocshRegister(&XPSInterposeDef,  XPSInterposeCallFunc);
}

epicsExportRegistrar(XPSInterposeRegister);
