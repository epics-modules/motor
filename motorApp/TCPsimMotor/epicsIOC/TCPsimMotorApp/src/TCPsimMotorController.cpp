/*
FILENAME... TCPsimMotorController.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "TCPsimMotor.h"

/** Creates a new TCPsimMotorController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName     The name of the drvAsynSerialPort that was created previously to connect to the TCPsimMotor controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
TCPsimMotorController::TCPsimMotorController(const char *portName, const char *MotorPortName, int numAxes,
                                               double movingPollPeriod,double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_VIRTUAL_MOTOR_PARAMS,
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  /* Connect to TCPsimMotor controller */
  status = pasynOctetSyncIO->connect(MotorPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "cannot connect to motor controller\n");
  }
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new TCPsimMotorController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName  The name of the drvAsynIPPPort that was created previously to connect to the TCPsimMotor controller
  * \param[in] numAxes           The number of axes that this controller supports (0 is not used)
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int TCPsimMotorCreateController(const char *portName, const char *MotorPortName, int numAxes,
                                            int movingPollPeriod, int idlePollPeriod)
{
  new TCPsimMotorController(portName, MotorPortName, 1+numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
asynStatus TCPsimMotorController::writeReadOnErrorDisconnect(void)
{
  size_t nwrite = 0;
  asynStatus status;
  int eomReason;
  size_t outlen = strlen(outString_);
  size_t nread;
  status = pasynOctetSyncIO->writeRead(pasynUserController_, outString_, outlen,
                                       inString_, sizeof(inString_),
                                       DEFAULT_CONTROLLER_TIMEOUT,
                                       &nwrite, &nread, &eomReason);
  if (status == asynTimeout) {
    asynInterface *pasynInterface = NULL;
    asynCommon     *pasynCommon = NULL;
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s status=asynTimeout (%d)\n",
              outString_, (int)status);
    pasynInterface = pasynManager->findInterface(pasynUserController_,
                                                 asynCommonType,
                                                 0 /* FALSE */);
    if (pasynInterface) {
      pasynCommon = (asynCommon *)pasynInterface->pinterface;
      status = pasynCommon->disconnect(pasynInterface->drvPvt,
                                       pasynUserController_);
      handleStatusChange(asynError);
      if (status != asynSuccess) {
        asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s status=%s (%d)\n",
                  outString_, pasynManager->strStatus(status), (int)status);
      }
    } else {
      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "pasynInterface=%p pasynCommon=%p\n",
                pasynInterface, pasynCommon);
    }
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

asynStatus TCPsimMotorController::writeOnErrorDisconnect(void)
{
  size_t nwrite = 0;
  asynStatus status;
  size_t outlen = strlen(outString_);
  status = pasynOctetSyncIO->write(pasynUserController_, outString_, outlen,
                                   DEFAULT_CONTROLLER_TIMEOUT, &nwrite);
  if (status == asynTimeout) {
    asynInterface *pasynInterface = NULL;
    asynCommon     *pasynCommon = NULL;
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s status=asynTimeout (%d)\n",
              outString_, (int)status);
    pasynInterface = pasynManager->findInterface(pasynUserController_,
                                                 asynCommonType,
                                                 0 /* FALSE */);
    if (pasynInterface) {
      pasynCommon = (asynCommon *)pasynInterface->pinterface;
      status = pasynCommon->disconnect(pasynInterface->drvPvt,
                                       pasynUserController_);
      handleStatusChange(asynError);
      if (status != asynSuccess) {
        asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s status=%s (%d)\n",
                  outString_, pasynManager->strStatus(status), (int)status);
      }
    } else {
      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "pasynInterface=%p pasynCommon=%p\n",
                pasynInterface, pasynCommon);
    }
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

void TCPsimMotorController::handleStatusChange(asynStatus status)
{
  int i;
  for (i=0; i<numAxes_; i++) {
    TCPsimMotorAxis *pAxis=getAxis(i);
    if (!pAxis) continue;
    pAxis->handleStatusChange(status);
  }
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void TCPsimMotorController::report(FILE *fp, int level)
{
  fprintf(fp, "Twincat motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an TCPsimMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
TCPsimMotorAxis* TCPsimMotorController::getAxis(asynUser *pasynUser)
{
  return static_cast<TCPsimMotorAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an TCPsimMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
TCPsimMotorAxis* TCPsimMotorController::getAxis(int axisNo)
{
  return static_cast<TCPsimMotorAxis*>(asynMotorController::getAxis(axisNo));
}


asynStatus TCPsimMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  TCPsimMotorAxis *pAxis;
  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;

  (void)pAxis->setIntegerParam(function, value);
  return asynMotorController::writeInt32(pasynUser, value);
}

/** Code for iocsh registration */
static const iocshArg TCPsimMotorCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg TCPsimMotorCreateControllerArg1 = {"EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg TCPsimMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg TCPsimMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg TCPsimMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const TCPsimMotorCreateControllerArgs[] = {&TCPsimMotorCreateControllerArg0,
                                                             &TCPsimMotorCreateControllerArg1,
                                                             &TCPsimMotorCreateControllerArg2,
                                                             &TCPsimMotorCreateControllerArg3,
                                                             &TCPsimMotorCreateControllerArg4};
static const iocshFuncDef TCPsimMotorCreateControllerDef = {"TCPsimMotorCreateController", 5, TCPsimMotorCreateControllerArgs};
static void TCPsimMotorCreateContollerCallFunc(const iocshArgBuf *args)
{
  TCPsimMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


/* TCPsimMotorCreateAxis */
static const iocshArg TCPsimMotorCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg TCPsimMotorCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg TCPsimMotorCreateAxisArg2 = {"axisFlags", iocshArgInt};
static const iocshArg TCPsimMotorCreateAxisArg3 = {"axisOptionsStr", iocshArgString};
static const iocshArg * const TCPsimMotorCreateAxisArgs[] = {&TCPsimMotorCreateAxisArg0,
							      &TCPsimMotorCreateAxisArg1,
							      &TCPsimMotorCreateAxisArg2,
							      &TCPsimMotorCreateAxisArg3};
static const iocshFuncDef TCPsimMotorCreateAxisDef = {"TCPsimMotorCreateAxis", 4, TCPsimMotorCreateAxisArgs};
static void TCPsimMotorCreateAxisCallFunc(const iocshArgBuf *args)
{
  TCPsimMotorCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void TCPsimMotorControllerRegister(void)
{
  iocshRegister(&TCPsimMotorCreateControllerDef, TCPsimMotorCreateContollerCallFunc);
  iocshRegister(&TCPsimMotorCreateAxisDef,       TCPsimMotorCreateAxisCallFunc);
}

extern "C" {
  epicsExportRegistrar(TCPsimMotorControllerRegister);
}
