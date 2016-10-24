/*
FILENAME...     omsMAXnet.h
USAGE...        Pro-Dex OMS MAXnet asyn motor controller support

*/

/*
 *  Created on: 10/2010
 *      Author: eden
 */

#ifndef OMSMAXNET_H_
#define OMSMAXNET_H_

#include "omsBaseController.h"

class omsMAXnet : public omsBaseController {
public:
    omsMAXnet(const char* , int , const char*, const char*, int , int );
    static void asynCallback(void*, asynUser*, char *, size_t, int);
    int portConnected;
    int notificationCounter;
    epicsMutex* notificationMutex;
    epicsEventWaitStatus waitInterruptible(double);
    asynStatus sendReceive(const char *, char *, unsigned int );
    asynStatus sendOnly(const char *);
    virtual bool resetConnection();

private:
    int isNotification (char *);
    asynUser* pasynUserSerial;
    asynUser* pasynUserSyncIOSerial;
    asynOctet *pasynOctetSerial;
    void* octetPvtSerial;
    double timeout;
};

#endif /* OMSMAXNET_H_ */
