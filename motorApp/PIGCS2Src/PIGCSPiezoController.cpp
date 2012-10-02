/*
 * PIGCSPiezoController
 *
 *  Created on: 15.12.2010
 *      Author: sra
 */
 
/*
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

#include "PIGCSPiezoController.h"
#include "PIasynAxis.h"
#include <math.h>
#include <stdlib.h>

//#undef asynPrint
//#define asynPrint(user,reason,format...) 0


asynStatus PIGCSPiezoController::getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl)
{
    asynStatus status = getMoving(pAxis, moving);
    if (status != asynSuccess)
    {
    	return status;
    }

   	homing = 0;
   	negLimit = 0;
   	posLimit = 0;

    return status;
}

asynStatus PIGCSPiezoController::getReferencedState(PIasynAxis* pAxis)
{
	pAxis->m_homed = 1;
	return asynSuccess;
}

