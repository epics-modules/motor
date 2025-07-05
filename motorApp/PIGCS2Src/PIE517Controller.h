/*
FILENAME...     PIGCScontroller.h

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************


Original Author: Steffen Rau 
Created: 15.12.2010
*/

#ifndef PIE517CONTROLLER_H_
#define PIE517CONTROLLER_H_

#include "PIGCSPiezoController.h"

/**
 * class representing PI E-517 and E-545 controllers.
 *
 * Basically these controllers have the same functionality as a
 * digital piezo controller. Specialties are "velocity control" mode
 * and the "online" state. Output channels must be set to "online" mode
 * before commands are accepted over the interface. This is done in
 * init()
 */
class PIE517Controller : public PIGCSPiezoController
{
public:
	PIE517Controller(PIInterface* pInterface, const char* szIDN)
	: PIGCSPiezoController(pInterface, szIDN)
	{
	}
	~PIE517Controller() {}

	virtual asynStatus init(void);
	virtual asynStatus haltAxis(PIasynAxis* pAxis)
	{
		// E-517 does support HLT with single axis
		return PIGCSController::haltAxis(pAxis);
	}

private:
    asynStatus setOnline(int outputChannel, int onlineState);
    asynStatus getNrOutputChannels();

	int m_nrOutputChannels;

};

#endif /* PIE517CONTROLLER_H_ */
