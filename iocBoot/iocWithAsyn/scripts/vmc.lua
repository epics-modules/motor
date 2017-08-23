IdlePollPeriod   = 1.0
MovingPollPeriod = 0.25
ForcedFastPolls  = 2

InTerminator  = "\r\n"
OutTerminator = "\r\n"

function sendAccelAndVelocity(minVel, maxVel, accel)
	asyn.writeread( string.format( "%d BAS %f", AXIS + 1, minVel) , PORT);
	asyn.writeread( string.format( "%d VEL %f", AXIS + 1, maxVel) , PORT);
	asyn.writeread( string.format( "%d ACC %f", AXIS + 1, accel ) , PORT);
end


function move(position, relative, minVel, maxVel, accel)
	sendAccelAndVelocity( minVel, maxVel, accel)

	if (relative) then
		asyn.writeread( string.format( "%d MR %d", AXIS + 1, math.floor(position) ) , PORT)
	else
		asyn.writeread( string.format( "%d MV %d", AXIS + 1, math.floor(position) ) , PORT)
	end
end


function moveVelocity(minVel, maxVel, accel)
	sendAccelAndVelocity( minVel, maxVel, accel)
	
	asyn.writeread( string.format( "%d JOG %f", AXIS + 1, maxVel) , PORT);
end


function poll()
	asyn.write( string.format( "%d POS?", AXIS + 1) , PORT)

	asyn.setDoubleParam( DRIVER, AXIS, "MOTOR_POSITION", tonumber( asyn.read(PORT) ) )
	
	asyn.write( string.format( "%d ST?", AXIS + 1) , PORT)

	local status = tonumber( asyn.read(PORT) )
	
	local direction  = (status & 1)
	local done       = (status & 2)  >> 1
	local limit_high = (status & 8)  >> 3
	local limit_low  = (status & 16) >> 4
	
	asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_DIRECTION",  direction)
	asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_DONE",       done)
	asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_MOVING",     done ~ 1)
	asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_HIGH_LIMIT", limit_high)
	asyn.setIntegerParam( DRIVER, AXIS, "MOTOR_STATUS_LOW_LIMIT",  limit_low)
	
	asyn.callParamCallbacks(DRIVER, AXIS)
	
	return (done ~= 1)
end	


function stop(acceleration)
	asyn.writeread( string.format( "%d AB", AXIS + 1) , PORT);
end


function setPosition(position)
	asyn.writeread( string.format( "%d POS %d", AXIS + 1, math.floor(position) ) , PORT);
end
