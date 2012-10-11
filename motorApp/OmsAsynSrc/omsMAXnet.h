/*
FILENAME...     omsMAXnet.h
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

private:
    int isNotification (char *);
    asynUser* pasynUserSerial;
    asynUser* pasynUserSyncIOSerial;
    asynOctet *pasynOctetSerial;
    void* octetPvtSerial;
    char* serialPortName;
    double timeout;
};

#endif /* OMSMAXNET_H_ */
