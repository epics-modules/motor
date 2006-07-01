/*  asynMotorStatus.h */
/***********************************************************************
* Copyright (c) 2002-6 The University of Chicago, as Operator of Argonne
* National Laboratory, and the Regents of the University of
* California, as Operator of Los Alamos National Laboratory, and
* Berliner Elektronenspeicherring-Gesellschaft m.b.H. (BESSY),
* and Diamond Light Source Ltd.
* asynDriver is distributed subject to a Software License Agreement
* found in file LICENSE that is included with this distribution.
***********************************************************************/

/*    20-June-2006 Peter Denison

*/

#ifndef asynMotorStatusH
#define asynMotorStatusH

#include <asynDriver.h>
#include <epicsTypes.h>
#include <shareLib.h>

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* The set of information that needs to be passed back from a motor as an
   atomic unit. The values here need to be consistent with the latest move
   (i.e. status = done only when position = demand position etc.) */
typedef struct MotorStatus {
    double position;
    double encoder_posn;
    double velocity;
    epicsUInt32 status;
} MotorStatus;

typedef void (*interruptCallbackMotorStatus)(
              void *userPvt, asynUser *pasynUser,
              struct MotorStatus *value);
typedef struct asynMotorStatusInterrupt {
    asynUser *pasynUser;
    int addr;
    interruptCallbackMotorStatus callback;
    void *userPvt;
} asynMotorStatusInterrupt;

#define asynMotorStatusType "asynMotorStatus"

typedef struct asynMotorStatus {
    asynStatus (*write)(void *drvPvt, asynUser *pasynUser,
                       struct MotorStatus *value);
    asynStatus (*read)(void *drvPvt, asynUser *pasynUser,
                       struct MotorStatus *value);
    asynStatus (*registerInterruptUser)(void *drvPvt, asynUser *pasynUser,
             interruptCallbackMotorStatus callback, void *userPvt,
             void **registrarPvt);
    asynStatus (*cancelInterruptUser)(void *drvPvt, asynUser *pasynUser,
             void *registrarPvt);
} asynMotorStatus;

#define asynMotorStatusBaseType "asynMotorStatusBase"

typedef struct asynMotorStatusBase {
    asynStatus (*initialize)(const char *portName,
                            asynInterface *pmotorInterface);
} asynMotorStatusBase;
epicsShareExtern asynMotorStatusBase *pasynMotorStatusBase;

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* asynMotorStatusH */
