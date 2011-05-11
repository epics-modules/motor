/*
FILENAME...	asynPortDriverExt.h
USAGE...	

Version:	$Revision$
Modified By:	$Author$
Last Modified:	$Date$
*/

#ifndef asynPortDriverExt_H
#define asynPortDriverExt_H

#include "asynMotorController.h"

class asynPortDriverExt
{
public:
  asynPortDriverExt(asynMotorController *pC);
  static int        getNumParams() {return 0;};
  virtual asynStatus createParams() = 0;
  asynStatus         createParam(const char *name, asynParamType type, int *index);

protected:
  asynMotorController *pC_;
};

#endif /*asynPortDriverExt_H */

