/*
FILENAME...	asynPortDriverExt.cpp
USAGE...	

Version:	$Revision$
Modified By:	$Author$
Last Modified:	$Date$
*/

#include "asynPortDriverExt.h"

asynPortDriverExt::asynPortDriverExt(asynMotorController *pC): pC_(pC)
{
}

/** This is a convenience method for calling port driver createParam() method.
This allows us to avoid reference to the portDriver.
*/
asynStatus asynPortDriverExt::createParam(const char *name, asynParamType type, int *index)
{
  return(pC_->createParam(name, type, index));
}

