# motor


This is a fork of
https://github.com/epics-modules/motor/

# Changes done by ESS, the most important ones

## v6.9.7-ESS

### Updates from upstream, motor R7.0

####  3b3625b7c84 "Remove the config from controller code"
    When most (configuration) values can be read from the controller,
    this was possible to push those values into the motorRecord fields.
    (For example the default velocity was copied into VELO)
    
    Remove all this logic from the record and asyn device support.
    If needed, the driver can set up ai records which can be forwarded
    into the record fields with help of ao records.
    
    The only parameters that are left are the "read only softlimits".
    They are needed to restrict DHLM and DLLM inside the working range
    of the axis.

#### db759583 "motorRecord.cc: Correct SPDB handling when MRES == 1.0"

#### ae5e1fae "Add a SPAM field"
    Make the enable/disable of printouts more fine-grained,
    now we use a bit mask in the new introduced SPAM field:
    
     1 STOP record stops motor, motor reports stopped
     2 MIP changes,  may be retry, doRetryOrDone, delayReq/Ack
     3 Record init, UDF changes, values from controller
     4 LVIO, recalcLVIO
     5 postProcess
     6 do_work()
     7 special()
     8 Process begin/end

#### 36177f7b "motorRecord.cc: Add ACCS field"
    The problem:
    When a user changes the velocity by writing to the VELO field,
    but forgets to write to the ACCL field, the acceleration will
    change and may be too high.
    (Note: Many users are not aware of this side-effect).
    
    Solution:
    Introduce a field called ACCS, "Acceleration in seconds^2".
    
    Once the field is written, the ACCL field is updated and the
    acceleration is "locked".
    It is locked for changes of VELO.
    It can be be unlocked by writing to ACCL.
    
    If the acceleration is locked (or not) is stored in the field ACCU,
    which can be either "motorACCSused_Accl" or "motorACCSused_Accs".
    
    And in this sense ACCU is not a lock, but just keeps track which
    of the accelation fields ACCL or ACCS has been updated last, and
    should be used in the next movement.
    In any case an update to ACCS updated ACCL and the other way around.
    Thanks to Mark Rivers for this nice idea.

#### a01e46ef "Don't allow DHLM=DLLM=0 when read-only limits are set"

####  b912ecc7 "Move EGU"
    Make it possible to forward movements (absolute/relative) from the
    record into the model 3 driver:
    - Add a new device support function, moveEGU
    - Add it to the device suppport table for asyn motors, entry #9
    - Use writeGenericPointer in asynMotorController to receive
      the new command.
    - Add moveEGU() in asynMotorAxis, this function can be overloaded
      by a driver.
    - Add moveVeloEGU() in asynMotorAxis, this function can be overloaded
      by a driver.

## v6.9.6-ESS

### Updates from upstream, motor R6.11

Beside many bugfixes and improvements, here some high lights:

#### 8eacc2f0 "Adapt motorRecord.cc to base-3.16 with typed rset"

#### ee9fca1d "Fixes for problems that showed up in base 7.0. -
     It was using the value of mr->[link]->value.constantStr
     and mr->[link]->value.pv_link.pvname without checking the
     link types.   This was incorrect, and led to errors because
     constantStr no longer has a defined value if the link type
     is not CONSTANT.   Changed so that it only accesses these
     union fields if the link is the correct type.
     - It did not check if pv_link.pvname was NULL before calling
       CA functions, though now that we check the link type perhaps
       this cannot occur.
     - It did not lock the motor record before accessing the fields.
     - Added Debug calls to show the link types and values."

#### 95c0a4ca "Don't set "stop" field true if driver returns RA_PROBLEM true.
     It is the driver's responsiblity to stop a motor if the condition
     that results in the RA_PROBLEM bit being set doesn't result
     in the motor automatically stopping."


#### 7493d50b "If URIP=Yes and reading RDBL causes LINK error,
     do not start a new target position move."

#### 8eacc2f0 "Adapt motorRecord.cc to base-3.16 with typed rset"

#### 689a813a "Merge branch 'add_SPDB_v1'",
     Rename SDBD into SPDB

#### f3d3d2cb 'Add Adjust after homed'
    When homing against a limit switch, we find the motor position
    most often outside the valid soft limit range.
    Make it possible to move the motor inside the allowed range,
    by settting the "motorFlagsAdjAfterHomed_" parameter to 1.

#### 9c2c0c6f "STOP when hitting a limit switch is configurable"
    
    When a limit switch is hit when traveling into the commanded direction,
    the motorRecord sends a STOP.
    There are cases, when this is not desired:
    - The IOC is running while the motion controller is commissioned.
      The  motor is moved using an engineering tool,
      The motorRecord does not have a valid "commanded direction".
      The motion controller will stop the motor, when a limit switch is
      hit, and this shouldbe tested manually.
    - The motor is homed against a limit switch.
      The controller will stop the motor, and then move it slowly away from
      the switch.
      Once the switch releases, the current position is latched.
    
    Note:
    Different controllers and use cases may need different behaviour(s),
    so more commits and changes may come in the future.

#### Late connect
    Improve the situation when an IOC is started a long time before the
    controller "goes online".
    The basic idea is to keep the record in "UDF".
    Different commits like
    84d0d09b "devMotorAsyn: Keep record in longer in UDF if needed"
    
    When the communication with an asyn motor has not be established when the
    record is iniitialized, keep the record in UDF.
    (The record processing can not wait here)
