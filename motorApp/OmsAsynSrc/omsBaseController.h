/*
FILENAME...     omsBaseController.h
USAGE...        Pro-Dex OMS asyn motor base controller support

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *  Created on: 06/2012
 *      Author: eden
 */

#ifndef OMSBASECONTROLLER_H_
#define OMSBASECONTROLLER_H_

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <iocsh.h>
#include <errlog.h>
#include "asynMotorController.h"
#include "omsBaseAxis.h"
#include <epicsExport.h>

#define OMS_MAX_AXES 8
#define OMSBASE_MAXNUMBERLEN 12
#define OMSINPUTBUFFERLEN OMSBASE_MAXNUMBERLEN * OMS_MAX_AXES + 2

class omsBaseController : public asynMotorController {
public:
    omsBaseController(const char *portName, int numAxes, int priority, int stackSize, int extMotorParams);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void report(FILE *fp, int level);
    virtual asynStatus sendReceive(const char*, char*, unsigned int ) = 0;
    virtual asynStatus sendOnly(const char *outputBuff) = 0;
    void omsPoller();
    virtual asynStatus startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls);
    static void callPoller(void*);
    static void callShutdown(void *ptr){((omsBaseController*)ptr)->shutdown();};
    void shutdown();

protected:
    virtual asynStatus writeOctet(asynUser *, const char *, size_t, size_t *);
    virtual asynStatus getFirmwareVersion();
    virtual asynStatus Init(const char*, int);
    virtual asynStatus sanityCheck();
    asynStatus sendReceiveLock(const char*, char*, unsigned int );
    asynStatus sendOnlyLock(const char *);
    virtual omsBaseAxis* getAxis(asynUser *pasynUser);
    virtual omsBaseAxis* getAxis(int);
    asynStatus getAxesArray(char*, int positions[OMS_MAX_AXES]);
    virtual asynStatus getAxesPositions(int positions[OMS_MAX_AXES]);
    virtual asynStatus getAxesStatus(char *, int, bool *);
    virtual asynStatus getEncoderPositions(epicsInt32 encPosArr[OMS_MAX_AXES]);
    virtual asynStatus getClosedLoopStatus(int clstatus[OMS_MAX_AXES]);
    virtual epicsEventWaitStatus waitInterruptible(double timeout);
    virtual bool watchdogOK();
    char* getPortName(){return portName;};
    bool firmwareMin(int, int, int);
    static omsBaseController* findController(const char*);
    static ELLLIST omsControllerList;
    static int omsTotalControllerNumber;
    char* controllerType;
    int fwMajor, fwMinor, fwRevision;
    epicsTimeStamp now;
    char* portName;
    bool useWatchdog;
    bool enabled;
    int numAxes;

private:
    asynStatus sendReplace(omsBaseAxis*, char*);
    asynStatus sendReceiveReplace(omsBaseAxis*, char *, char *, int);
    asynStatus getSubstring(unsigned int , char* , char *, unsigned int);
    int sanityCounter;
    epicsThreadId motorThread;
    char inputBuffer[OMSINPUTBUFFERLEN];
    char pollInputBuffer[OMSINPUTBUFFERLEN];
    omsBaseAxis** pAxes;
    int controllerNumber;
    epicsMutex *baseMutex;
    int sendReceiveIndex;
    int sendIndex;
    int receiveIndex;
    int pollIndex;
    int priority, stackSize;

    friend class omsBaseAxis;
};

typedef struct omsBaseNode {
    ELLNODE node;
    const char *portName;
    omsBaseController* pController;
} omsBaseNode;

#endif /* OMSBASE_H_ */
