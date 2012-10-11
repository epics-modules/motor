/*
FILENAME...     omsBaseAxis.h
USAGE...        Pro-Dex OMS asyn motor base axes support

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *  Created on: 06/2012
 *      Author: eden
 */

#ifndef OMSBASEAXIS_H_
#define OMSBASEAXIS_H_

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class omsBaseController;

class omsBaseAxis : public asynMotorAxis
{
public:
    omsBaseAxis(class omsBaseController *, int, char );
    virtual asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
    virtual asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
    virtual asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    virtual asynStatus stop(double acceleration);
    virtual asynStatus doMoveToHome();
    virtual asynStatus setPosition(double position);
    virtual asynStatus poll(bool *moving);

    int getAxis(){return axisNo_;};
    int isStepper(){return stepper;};
    void setStepper(int val){stepper=val;};
    int getLimitInvert(){return invertLimit;};
    void setLimitInvert(int val){invertLimit=val;};
    int card;
    int moveDelay;
    char axisChar;
    bool homing;


private:
    omsBaseController *pC_;
    int stepper;
    int invertLimit;

friend class omsBaseController;
};

#endif /* OMSBASEAXIS_H_ */
