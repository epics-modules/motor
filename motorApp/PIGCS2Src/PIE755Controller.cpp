/*
 * PIE755Controller
 *
 *      Author: sra
 */
 
/*
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

#include "PIE755Controller.h"
#include "PIasynAxis.h"
#include <stdlib.h>

#undef asynPrint
#define asynPrint(user,reason,format...) 0


asynStatus PIE755Controller::getStatus(PIasynAxis* pAxis, int& homing, int& moving, int& negLimit, int& posLimit, int& servoControl)
{
	int busy;
	asynStatus status = getBusy(pAxis, busy);
    if (status != asynSuccess)
    {
    	return status;
    }

    negLimit = 0;
   	posLimit = 0;
   	homing = busy;
   	if (busy)
   	{
   		moving = busy;
   		return status;
   	}

	status = getMoving(pAxis, moving);
    if (status != asynSuccess)
    {
    	return status;
    }

    return status;
}
