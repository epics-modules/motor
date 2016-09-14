IdlePollPeriod = 1.00
MovingPollPeriod = 0.25

lastPos = 0
targetPos = 0

function move(position, relative, minVel, maxVel, accel)
	local MRES = asyn.getDoubleParam( DRIVER, AXIS, "MOTOR_REC_RESOLUTION")
	
	if (relative) then
		local prev = asyn.getDoubleParam( DRIVER, AXIS, "MOTOR_POSITION")
		targetPos = prev + (position * MRES)
	else
		targetPos = (position * MRES)
	end
	
	epics.put(DRIVE_PV, targetPos)
	
	if (position > 0) then
		asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_DIRECTION", 1)
	else
		asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_DIRECTION", 0)
	end
	
	asyn.callParamCallbacks(DRIVER, AXIS)
end


function poll()
	local MRES = asyn.getDoubleParam( DRIVER, AXIS, "MOTOR_REC_RESOLUTION")
	
	if (MRES == 0.0) then
		return true
	end
	
	local curr = epics.get(READBACK_PV)
	
	asyn.setDoubleParam( DRIVER, AXIS, "MOTOR_POSITION", curr / MRES)
	
	local done = 0
	local moving = 1
	
	if (curr == targetPos) then
		done = 1
		moving = 0
	elseif (math.abs(curr - targetPos) <= MRES and lastPos == curr) then
		done = 1
		moving = 0
	end
	
	lastPos = curr
	
	asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_DONE", done)
	asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_MOVING", moving)
	
	asyn.callParamCallbacks(DRIVER, AXIS)
	
	return (done ~= 1)
end


function stop(acceleration)
	local curr = asyn.getDoubleParam( DRIVER, AXIS, "MOTOR_POSITION")
	
	epics.put(DRIVE_PV, curr)
	targetPos = curr
end
