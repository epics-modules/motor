/*
FILENAME...     omsBaseAxis.cpp
USAGE...        Pro-Dex OMS asyn motor base axes support

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/


/*
 *  Created on: 6/2012
 *      Author: eden
 *
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "omsBaseAxis.h"
#include "omsBaseController.h"

static const char *driverName = "omsBaseAxisDriver";

omsBaseAxis::omsBaseAxis(omsBaseController *pController, int axis, char axisChar)
    : asynMotorAxis(pController, axis), axisChar(axisChar)
{
    pC_ = pController;
    stepper = 1;
    invertLimit = 0;
}

asynStatus omsBaseAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
//    omsBaseAxis *pAxis = this->getAxis(pasynUser);
    static const char *functionName = "moveAxis";

    asynStatus status = asynError;
    epicsInt32 minvelo, velo, acc, rela, pos;
    char *relabs[2] = {(char *) "MA", (char *) "MR"};
    char buff[100];

    if (relative)
        rela = 1;
    else
        rela = 0;

    pos = (epicsInt32) (position + 0.5);
    if (abs(pos) > 67000000){
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s:%s axis %d position out of range %f\n",
              driverName, functionName, pC_->portName, axisNo_, position);
        return status;
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

    /* move to the specified position */
    sprintf(buff, "A%1c AC%d; VB%d; VL%d; %s%d; GO ID", axisChar, acc, minvelo, velo, relabs[rela], pos);
    status = pC_->sendOnlyLock(buff);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set driver %s, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f",
        driverName, functionName, pC_->portName, axisNo_, position, min_velocity, max_velocity, acceleration );

    return status;
}

asynStatus omsBaseAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards )
{
    static const char *functionName = "homeAxis";

    asynStatus status = asynError;
    char buff[60];
    char *direction[2] = {(char*) "HR", (char*) "HM"};
    epicsInt32 velo, acc, fw = 0;

    if (forwards) fw = 1;

    velo = (epicsInt32) max_velocity;
    if (velo < 1) velo = 1;
    else if (velo > 1000000) velo = 1000000;

    acc = abs((epicsInt32) acceleration);
    if (acc > 8000000)
        acc = 8000000;
    else if (acc < 1)
        acc = 1;

    /* do a home run and move to the home position */
    sprintf(buff, "A%1c AC%d; VL%d; %s; MA0 GO ID", axisChar, acc, velo, direction[forwards]);
    status = pC_->sendOnlyLock(buff);

    homing = 1;

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set driver %s, axis %d to home %s, min vel=%f, max_vel=%f, accel=%f",
        driverName, functionName, pC_->portName, axisNo_, (forwards?"FORWARDS":"REVERSE"), min_velocity, max_velocity, acceleration );

    return status;
}

asynStatus omsBaseAxis::doMoveToHome()
{
    static const char *functionName = "doMoveToHome";
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
        "%s:%s: This function is not yet implemented for axis %d\n",
        driverName, functionName, axisNo_);
    return asynError;
}

asynStatus omsBaseAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration )
{
    asynStatus status = asynError;
    static const char *functionName = "moveVelocityAxis";

    char buff[100];
    epicsInt32 velo, acc;

    acc = (epicsInt32) acceleration;
    if (acc < 1) acc = 1;
    else if (acc > 8000000) acc = 8000000;

    velo = (epicsInt32) maxVelocity;
    if (velo > 4000000) velo = 4000000;
    else if (velo < -4000000) velo = -4000000;

    sprintf(buff, "A%1c AC%d; JG%d;", axisChar, acc, velo);
    status = pC_->sendOnlyLock(buff);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: Set port %s, axis %d move with velocity of %f, acceleration=%f",
        driverName, functionName, pC_->portName, axisNo_, maxVelocity, acceleration );

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    return status;
}

asynStatus omsBaseAxis::stop(double acceleration )
{
    asynStatus status = asynError;
    static const char *functionName = "stopAxis";
    int acc;
    char buff[50];

    acc = (int)(fabs(acceleration)+0.5);
    if (acc > 8000000) acc=8000000;
    if (acc < 1) acc = 200000;

    sprintf(buff, "A%1c AC%d; ST ID;", axisChar, acc);
    status = pC_->sendOnlyLock(buff);

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: port %s, set axis %d to stop with accel=%f\n",
        driverName, functionName, pC_->portName, axisNo_, axisNo_, acceleration );

    return status;
}

/** Set the current position of the motor.
 * \param[in] position The new absolute motor position that should be set in the hardware. Units=steps.
 */
asynStatus omsBaseAxis::setPosition(double position)
{
    static const char *functionName = "setPosition";
    asynStatus status = asynError;
    char buff[20];
    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
              "%s:%s:%s axis %d set position to %f\n",
              driverName, functionName, pC_->portName, axisNo_, position);
    sprintf(buff,"A%1c LP%d;", axisChar, (int)(position));
    status = pC_->sendOnlyLock(buff);

  return status;
}

/** we need to implement this, because we need to use the motorUpdateStatus_ function
 * in asynMotorController, because we cannot access statusChanged_ (shouldn't be private)
 * ignore moving flag, since we have our own poller.
 */
asynStatus omsBaseAxis::poll(bool *moving)
{
    epicsEventSignal(pC_->pollEventId_);
    return asynSuccess;
}
