# motor
[![Build Status](https://github.com/epics-modules/motor/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-modules/motor/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-modules/motor.png)](https://travis-ci.org/epics-modules/motor)-->


This is a fork of
https://github.com/epics-modules/motor/

# Changes to upstream motor, the most important ones

## v7.2.7-ESS, based on R7-3-1, and latest master.
###  Bug fixes
#### 7f5aa6177: motorRecord: another update after LS error: Endless loop
###  Improvements
#### 31b2549ed: asynMotorController.cpp: Avoid useless power on
#### 1b2a4e42b: motorRecord: another update after LS error: Better debug
#### 144773d22: motorRecord: Do not do retries after slipstall
## v7.2.6-ESS, based on R7-3-1, and latest master.
###  Bug fixes
#### 34cb92d68: asynMotorController: Handle CommsError in readGenericPointer()
#### 781661d64: asynMotorController: failure of poll() polls axes
###  Improvements
#### 3c1e9b569: motorRecord.cc: Enhance debug in process(begin)
#### 4128dc16e: asynMotorController: Factor out pollAutoPowerMayBeOff()
## v7.2.5-ESS, based on R7-3-1, see release notes from there
###  Bug fix
#### c1a1d794b: motorRecord:Fix hang when RTRY == 0: SET_LAST_VALS_FROM_VALS
## v7.2.4-ESS, based on R7-3-1, see release notes from there
###  Bug fixes
#### Update ci-scripts
#### a4a6dbddf: motorRecord.cc: Post ueip if reset to false when encoder missing
#### 7fbdf4b99: docs/motorRecord.html: CNEN is implementation specific
#### df0132e73: docs/motorRecord.html: Document HOMF and HOMR better
#### ea594ed6e: motorRecord: Fix init_re_init and neverPolled when URIP is used
#### 85f370eda: docs/index.html: Use ESS specific information
#### 8938b6146: motorRecord.cc: blink DMOV when jogging does not start
#### c1a1d794b: motorRecord:Fix hang when RTRY == 0: SET_LAST_VALS_FROM_VALS
## v7.2.3-ESS, based on R7-3-1, see release notes from there
###  Bug fixes
#### ff7ad73fc:motorRecord.html: Document bit 6 and 7 of .MFLG field
###  Improvements
#### 2dc310171: (and some more commits) Add MINP for soft motors
#### 2e89b5527: (and some more from upstream) ...add raw softlimts RLLM, RHLM needed when MRES changes
## v7.0.9-ESS, based on R7-2-2
###  Bug fixes
#### 2420d29d2: motorRecord: Reset JOGR/JOGF when LLS/HLS is active
#### f3f8b3b27: motorRecord.cc: doBackLash may hung up when on LS
###  Improvements
#### 40537595b: Add MOTOR_FLAGS_NO_TWEAK_ON_LS
#### 738314c57: asynMotorController: Do not move, when power on fails
####  1c6e4884: motorRecord.cc: Suppress LVIO if not homed
## v7.0.8-ESS, based on R7-2-2 + latest master
###  Bug fixes
#### 3358621ef: motorRecord: Do not do a retry when RA_PROBLEM is set
#### 41183c9c8: motorRecord.cc: Do not move if VAL - RBV == RDBD
#### 524696a8d: Fix negative backlash with relative moves in a negative direction
#### f149e8d7a: motorRecord: retry-relative follows motor/master (mostly)
#### c90d6a780: motorRecord.cc: Fix for SPMG=Pause and CALLBACK_DATA
#### 24ef7b207: motorRecord.cc: Fixes for NTM
### Improvements
#### 4c7949067: motorRecord: Do not home when both limit switches are active
#### bd7e007e8: asynMotorAxis.cpp: Distinguish PowerOff vs PowerOff Auto
#### 47bbd9354: motorRecord: loosen the close enough calculation
#### 7d181592d: asynMotorController.cpp: make autoPowerOn() more rebust
#### 6cc882fc3: motorRecord.cc/motorDevSup.h: Use CARD as axis number in printing
#### 700d424f4: motorRecord: fall back to ACCS if JAR is 0.0
#### 073ae178e: MotorRecord: Add NTM UPDATE
## v7.0.7-ESS, based on R7-2-2
###  Bug fixes
#### 50d99515e: motorRecord: Correct printing of special(DBF_MENU)
#### 240d12c9d: motorRecord.cc: Fix for SPMG=Pause and CALLBACK_DATA
#### fc8d9e956: motorRecord.cc: Fixes for NTM
#### 1f5666358: mototRecord.cc: Fixes for NTM
#### 1557ee7f6: motorRecord.cc: If controller uses EGU, keep VELO when UREV changes
### Improvements
#### 8cb38dbba: Remove non-up-to-date motor_release-ESS.html
#### 982ef46ee: MotorRecord: Add NTM UPDATE
#### d1a597ffa: motorRecord.cc: More debug logs around SPMG
#### 8efb77707: motorRecord.dbd: .SPMG is special SPC_MOD
## v7.0.6-ESS, based on R7-2-2
###  Bug fixes
#### fb6027cbb: asynMotorController.cpp: NULL pointer check for pasynUserController_
#### b47408f37: motorRecord.cc: Do not move further on limit switch
#### ab535ead6: motorRecord/maybeRetry(): Do not retry if limit swicth is active
#### 41c2bc5cc: Fix NTM=1 RTRY=1 bug: Motor stopped, but never moved
#### fc1bda9c0: motordrvCom.h needs shareLib.h
### Improvements
#### 8bfdd9d50: motorRecord.cc: More printing in doDVALchangedOrNOTdoneMoving()
#### 6c7833aac: motorRecord.cc: More printing around last VAL
#### ef91f0a0f: motorRecord.cc: More printing around commandedDval
## v7.0.5-ESS
###  Bug fixes
####  8e1047d1: motorRecord: JOGF=1 and writing to e.g. LLM stopped
####  380b35b0: asynMotorAxis.cpp: Only update motorStatus_ if needed
####  0bb7c2a3: devMotorAsyn.c: Handle MF_DRIVER_USES_EGU in position restore
####  e0db111f: motorRecord.cc: Fix printing in special() for links
####  173355c5: Add shareLib.h
### Improvements
####  c7b58b60: Save-Restore: Add restore_needed
####  1ac030da: devMotorAsyn.c: Introduce priv.saveRestore
####  4e67cc7a: asynMotorController::readGenericPointer: Improve asynError
####  c4eb8bc0: asynMotorAxis.cpp: Add positionWritten
####  afa150b6: devMotorAsyn.c: Rename pasynUserSync into pasynUserSyncFloat64
####  c3d59ab8: Merge 'github/asynMotor-remove-initEvent' into ess-master
####  c966d921: Add devPositionRestoreNeeded() and use it
####  49da10eb: motorRecord.cc: Improve prints about neverPolled   
####  6b982fe0: Make flags in MFLG field more usable 
####  855f0678: motorRecord.cc: Sanitize ERES after MRES
####  1764b559: motorRecord.cc: Add one more debug in init_re_init() 
####  74e65c7d: devMotorAsyn.c: Add debug print around RSTM
####  debba9e2: asynMotorController.cpp: autopwerOff and no jog
####  5bea53be: asynMotorController.cpp: Handle status in writeInt32()
####  6c7ef0cb: motorRecord.cc: Remove printing of RA_DIRECTION
####  166d19ae: motorRecord.cc: Reduce spam printing of RA_DIRECTION
####  0e87222a: motorRecord.cc: Introduce mr_stops_on_ls_activated()
####  f213c1d5: motorRecord.cc: Less LS special handling with cdir
####  b81edb83: motorRecord.cc: Cleanup around clear_xx_buttons
####  90643807: devMotorAsyn: remove initEvent in init_record()

## v7.0.4-ESS

###  Bug fixes
####  ca0747aa: Fixes around load_pos related to MRES
####  7a25fed5: devMotorAsyn.c: Initialize pmr->flags early 
####  46dfadb1: motorRecord.cc: Correct handling of RRES
####  5d825112: motorRecord.cc: clear_buttons() when hitting a limit switch
####  44a1ad3a: motorRecord.cc: Reset JOGF/JOGR in motor-has-stopped

### Improvements
####  ff79e4c5: motorRecord.cc: More printout levels; devSupGetInfo()
####  8d6e0da0: asynMotorController.cpp: Handle status of poll()
####  d8a43e9b: motorRecord.cc: Debug backlash and retry count
####  153b8616: motorRecord.cc: print value in special()
####  54f86b90: Improve printing of MIP
## v7.0.3-ESS

###  Bug fixes
####  54c4286b: LOAD_POS: Ignore MRES when driver uses EGU
####  146f43ee: Improve handling of ramp-down after stop
####  146f43ee: Record recognizes motor stop while jogging


### Improvements

#### 19dc53c1: devMotorAsyn: Introduce Debug prints

## v7.0.2-ESS

###  Update to the latest motor/master, 2020-08-06, Includes upstream R7-2-1

###  Bug fixes

####  7546277f: JVEL while jogging, MF_DRIVER_USES_EGU is used
####  c92efe25: Late connect: Don't mis-use UDF, use neverPolled

### Improvements

####  RSTM field (Restore mode)
    The RSTM field is in upstram master,
    but today not part of an official release

####  b64af248: .SPAM field: Initialize it in motorRecord.dbd
    The motorRecord state machine is sometimes hard to follow.
    In order to make sure that we can follow all the transitions
    (especially when the driver reports "DONE") there is a new logging
    from inside the motorRecord itself.
    A simplified version of a move triggered by writing to the .VAL field
    of the PV "IOC:m1" looks like this:
    2020/06/26 13:29:15.370 [motorRecord.cc:1833 IOC:m1] doRetryOrDone dval=158.9 rdbd=0.1
    2020/06/26 13:29:15.370 [motorRecord.cc:1853 IOC:m1] mipSetBit 'Mo' old='' new='Mo'
    2020/06/26 13:29:15.375 [motorRecord.cc:1462 IOC:m1] msta.Bits.RA_DONE=0
    2020/06/26 13:29:15.487 [motorRecord.cc:1442 IOC:m1] msta.Bits.RA_MOVING=1
    2020/06/26 13:29:18.119 [motorRecord.cc:1442 IOC:m1] msta.Bits.RA_MOVING=0
    2020/06/26 13:29:18.119 [motorRecord.cc:1462 IOC:m1] msta.Bits.RA_DONE=1
    2020/06/26 13:29:18.119 [motorRecord.cc:1574 IOC:m1] motor has stopped drbv=158.970
    2020/06/26 13:29:18.119 [motorRecord.cc:1231 IOC:m1] maybeRetry: close enough; rdbd=0.1diff=-0.07 mip=0x20('Mo')
    2020/06/26 13:29:18.119 [motorRecord.cc:1234 IOC:m1] mipClrBit '...' old='Mo' new=''

    If you don't want these printouts, set the .SPAM field to produce less loggings

#### d7b19882: motorRecord.cc: Print the date and time in Debug()

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

#### Controller uses EGU
    Modern controller use engineering units, so that the IOC uses e.g.
    "mm" instead of "steps" when communicating positions.
    The motorRecord insists somewhat to use steps.
    As a result, both MRES must be set to some value, and then
    the controller must have a "scale factor", "stepsPerUnit" or the like.
    Solution:
      setIntegerParam(pC_->motorFlagsDriverUsesEGU_,1);
    Note that there is still a need to set either MRES or SPDB to a useful
    value, otherwise the motorRecord will refuse to move distances below
    1.0 mm.

#### Read-only soft limits from controller
    The model 1 and model 2 drivers can refuse to accept soft-limits.
    The soft limits in the motorRecord stay always inside the physical
    driving range of the axis.
    Example: The controller has a driving range 0..175 mm
    The user tries to set the DHLM field to 180mm.
    The device support in the driver is allowed to refuse the change,
    so that the DHLM field stays ar 175.
    Similar for DLLM, and the user coordinate-ish LLH and LLM
    This has never been implemented for model 3 drivers.
    If available, the "read only softlimits" can be written
    into the parameter library like this:
      pAxis->setDoubleParam(motorHighLimitRO_, limitHigh);
      pAxis->setDoubleParam(motorLowLimitRO_,  limitLow);
    And device support will forward them to the motorRecord.

#### Homing ignores limit switches
    The motorRecord assumes that there is a "home switch",
    which needs to be activated.
    The user must know, where the motor is, and decide if she wants
    to use HOMF or HOMR to initiate the homing.
    This does not work well if
    - (a) The limit switch itself is the "home switch"
    - (b) The homing sequence is like this
          "go to the low limit switch, activate it, stop, reverse the
          direction and search for the home switch.
    In case of (a) will the motorRecord send a stop to the controller,
    which may abort the homeing and leaves the axis in an unusable state.
    In case of (b) the same problem occurs.
    Solution:
      setIntegerParam(pC_->motorFlagsNoStopOnLS_, 1);
    There is another, sometime annoying, thing:
    When we have a controller, that can do a homing even when a limit
    switch is active, the motorRecord will ignore HOMF when HLS set.
    It silently ignores HOMR when LLS is active.
    So again, the user must choose between HOMR/HOMF.
    Solution:
      setIntegerParam(pC_->motorFlagsHomeOnLs_, 1);

#### Enhanced auto-power-on
    There are 2 improvements in the new "auto power on mode 2"
    The old auto power on had 0 (=off) and 1 (=on).
    The new mode "2" has two more features:
    - setting a power off timout < 0.0 will leave the axis
      powered on.
    - The fixed power on delay can be shortened, if the axis
      reports "power on achieved" earlier.
      A better documentation of this feature is needed.

#### All MRES related calculation are in motorDevSup.c
    The code in the motorRecord was full of DVAL/RVAL calculations,
    dividing by MRES was needed everywhere.
    This version uses dial coordinates in the motorRecord.cc
    and let motorDevSup.c convert them into "raw" values. 

#### Compiler warnings removed
    Removed all compiler warnings in the motor module.
    (Those that my compilers showed)

