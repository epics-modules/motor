#/ Controller version  = 5.00
#/ Date = 08/08/2006 17:03
#/ User remarks = #0
! X-AXIS
! Ibex dual rotary stage, 4xHR2 Nanomotiion motors per axis, AB5 amplifier, firmware 3.2
! Renishaw encoders, 20 micron pitch, X2000 interpolator, 1 count = 10 nm, 15 counts = 1 micro-radian
! ACS-Tech80 SpiPlus PCI-4 controller, firmware 5.0
!Note: Set dcEnable flag in AUTOEXEC routine to select AB2 or AB5 amplifier
!Note: VEL and ACC are commented out for the AC mode as they are set from EPICS MEDM screens.
!Note: VEL and ACC are DEFINED for the DC mode as they are not defined from EPICS MEDM screens
!ACC is defined in the homing routines as they are defined from EPICS during Homing but VEL is defined.

GLOBAL acpar(8)
GLOBAL dcpar(8)
GLOBAL dcEnable(8)
GLOBAL INT home_F(8)
GLOBAL INT home_R(8)
GLOBAL target_pos(8)
GLOBAL jog_vel(8)
GLOBAL Done(8)
GLOBAL opReq(8)

! Each AXIS has a copy of these functions in a corresponding buffer 
LOCAL Axis
LOCAL Buffer

IF     opReq(Axis) = 1
    call ABS_MOVE
ELSEIF opReq(Axis) = 2
    call REL_MOVE 
ELSEIF opReq(Axis) = 3
    call JOG_MOVE  
ELSEIF opReq(Axis) = 4
    call HOME_F
ELSEIF opReq(Axis) = 5
    call HOME_R
END

opReq(Axis) = 0; Done(Axis) = 1

STOP

!Powerup Routine
AUTOEXEC:

!Buffer = sysinfo(3)
Buffer = 0
Axis = Buffer

! Set AC/DC Switching Amplifier (AB2) enable flag 
! Enable = 1, Disable = 0
dcEnable(Axis) = 0

DISP "Buffer 0: Axis:", Axis

! Set all axis configuration parmeters
call AXIS_CFG
acpar(Axis)=1; TILL acpar(Axis)=0
if dcEnable(Axis) > 0
    dcpar(Axis)=1; TILL dcpar(Axis)=0
else
    acpar(Axis)=1; TILL acpar(Axis)=0
end

STOP

!THESE PROGRAMS ARE SMALL ROUTINES THAT ARE REQUIRED FOR CO-ORDINATING MOVEMENT WITH EPICS. THIS IS ESPECIALLY NEEDED
!BECAUSE OF THE AC AND DC MODE SWITCHING NEEDED for AB2 amplifier BEFORE AND AFTER EVERY MOVE.
!Written by Joe Sullivan (BCDA) and Suresh (8-ID) (March 2006)

ABS_MOVE:
! Switch to AC(Servo) Mode
acpar(Axis)=1; TILL acpar(Axis)=0
PTP(Axis),target_pos(Axis)
TILL ^MST(Axis).#MOVE
! Switch to DC(Position) Mode
if dcEnable(Axis) > 0
    dcpar(Axis)=1; TILL dcpar(Axis)=0
end
RET

REL_MOVE:
acpar(Axis)=1; TILL acpar(Axis)=0
PTP/r(Axis),target_pos(Axis)
TILL ^MST(Axis).#MOVE
if dcEnable(Axis) > 0
    dcpar(Axis)=1; TILL dcpar(Axis)=0
end
RET


JOG_MOVE:
acpar(Axis)=1; TILL acpar(Axis)=0
JOG/v(Axis),jog_vel(Axis)
TILL ^MST(Axis).#MOVE
if dcEnable(Axis) > 0
    dcpar(Axis)=1; TILL dcpar(Axis)=0
end
RET


!These program are for homing X and was written by Aaron Dietrich, ACS-Tech80 and modified by Suresh, 8-ID (March 2006)
!EPICS calls these routines when you Home from the MEDM screen.
!HOME_R homes in the -ve direction  and HOME_F homes in the +ve direction
!If you desire to home in the same direction due to constraints, change the last 2 lines
!in this program accordingly.
!Note: this program is written for AB2 amplifier. If using with AB5, comment out acpar and dcpar lines
!and rest should be fine.


HOME_R:
! Load tuned parameters for AC mode for AB2 Amplifier
!disable this when using AB5 amplifier
acpar(Axis)=1; TILL acpar(Axis)=0

DISP "Homing in negative direction is in PROGRESS ......"
ENABLE (Axis)
FDEF(Axis).#CL = 0  !disable default response of Current Limit fault
FDEF(Axis).#CPE = 0  !disable default response of Critical Postion Error fault
FDEF(Axis).#SRL = 0  !disable default response of Software Right Limit fault
FDEF(Axis).#SLL = 0  !disable default response of Software Left Limit fault

JOG (Axis), -
TILL (ABS(PE(Axis))>10000)
JOG (Axis), +
IST(Axis).#IND = 0  !enables the index capture
TILL IST(Axis).#IND = 1  !wait till index capture happens
KILL (Axis)
SET FPOS(Axis)=FPOS(Axis)-IND(Axis)  !this sets index position as 0 reference point
PTP (Axis), 0
TILL ^MST(Axis).#MOVE

FDEF(Axis).#CL = 1  !re-enable default response of Current Limit fault
FDEF(Axis).#CPE = 1  !re-enable default response of Critical Postion Error fault
FDEF(Axis).#SRL = 1  !re-enable default response of Software Right Limit fault
FDEF(Axis).#SLL = 1  !re-enable default response of Software Left Limit fault

!Put the stage in DC Mode
if dcEnable(Axis) > 0
    dcpar(Axis)=1; TILL dcpar(Axis)=0
end
!DISABLE (Axis)
call AXIS_CFG
DISP "Homing is DONE ......"
home_R(Axis) = 0;
RET


HOME_F:
! Load tuned parameters for AC mode for AB2 Amplifier
!disable this when using AB5 amplifier
acpar(Axis)=1; TILL acpar(Axis)=0

DISP "Homing in positive direction is in PROGRESS ......"
ENABLE (Axis)
FDEF(Axis).#CL = 0  !disable default response of Current Limit fault
FDEF(Axis).#CPE = 0  !disable default response of Critical Postion Error fault
FDEF(Axis).#SRL = 0  !disable default response of Software Right Limit fault
FDEF(Axis).#SLL = 0  !disable default response of Software Left Limit fault

JOG (Axis), +
TILL (ABS(PE(Axis))>10000)
JOG (Axis), -
IST(Axis).#IND = 0  !enables the index capture
TILL IST(Axis).#IND = 1  !wait till index capture happens
KILL (Axis)
SET FPOS(Axis)=FPOS(Axis)-IND(Axis)  !this sets index position as 0 reference point
PTP (Axis), 0
TILL ^MST(Axis).#MOVE

FDEF(Axis).#CL = 1  !re-enable default response of Current Limit fault
FDEF(Axis).#CPE = 1  !re-enable default response of Critical Postion Error fault
FDEF(Axis).#SRL = 1  !re-enable default response of Software Right Limit fault
FDEF(Axis).#SLL = 1  !re-enable default response of Software Left Limit fault

!Put the stage in DC Mode
if dcEnable(Axis) > 0
    dcpar(Axis)=1; TILL dcpar(Axis)=0
end
!DISABLE (Axis)
call AXIS_CFG
DISP "Homing is DONE ......"
home_F(Axis) = 0;
RET


!THESE PROGRAMS TUNE PARAMETERS SAVED FOR AB2 and AB5 AND THE GOOD
!STAGE TO BE USED IN THE MONO IN 8-ID

!MOSTLY WE WILL NEVER USE AB5 AMPLIFIER AS THE MOTOR IS ALWAYS SERVOING IN THIS MODE
!AND IS NOT GOOD FOR VACUUM USE AS THE MOTOR GETS HOT AND CANNOT BE ENABLED FOR MORE THAN
!10 MINUTES OR SO. IN AB2, DEADBAND IS ACTIVE AND SO THE MOTOR IS BASICALLY DISABLED
!ONCE THE STAGE IS WITHIN THE DEADBAND MAX WHICH IS LIKE 10 COUNTS IN THE CURRENT TUNING
!CONFIGURATION AND IS ABOUT 100 nm.
!In the DC mode, the motor can be kept enabled forever in vacuum and holds position to within 1 count
!which is currently 10 nm.
!Written by Suresh (8-ID) (March 2006)

!Note: VEL and ACC are commented out for the AC mode as they are set from EPICS MEDM screens.
!Note: VEL and ACC are DEFINED for the DC mode as they are not defined from EPICS MEDM screens

!To switch from DC to AC you must disable the motor
AC_TUNED_PAR: ! FOR LONG MOVE
! Switching only neccessary if DC mode enabled
if dcEnable(Axis) > 0
  DISABLE(Axis);
  !SET DC_MODE to 0 and SET Nanomotion bit to 1 resp.
  MFLAGS(Axis).30 = 0;
  MFLAGS(Axis).7 = 1;
  SLPKP(Axis)=300;
  SLVKP(Axis)=20;
  SLVKI(Axis)= 1600;
  SLFRC(Axis)=17;
end

!VEL(Axis)= 102400 * 1.0; !defined from EPICS
!ACC(Axis)=VEL(Axis)*10;  !defined from EPICS
DEC(Axis)=ACC(Axis);
KDEC(Axis)=DEC(Axis)*1E4;
JERK(Axis)=ACC(Axis)*1E3;

if dcEnable(Axis) > 0
  ENABLE(Axis);
end
acpar(Axis)=0;
RET


DC_TUNED_PAR:
!SET DC_MODE to 1 and SET Nanomotion bit to 0 resp.
VEL(Axis)=15;
ACC(Axis)=VEL(Axis)*10;
MFLAGS(Axis).30 = 1;
MFLAGS(Axis).7 =0;
SLPKP(Axis)=2500;
SLVKP(Axis)=20;
SLVKI(Axis)= 9000;
SLFRC(Axis)=0;
DEC(Axis)=ACC(Axis);
KDEC(Axis)=DEC(Axis);
JERK(Axis)=ACC(Axis)*10;
dcpar(Axis)=0;
RET

! *********************
! Axis Configuration
! *********************
AXIS_CFG:
DISABLE (Axis)
! Configuration Parameters
MFLAGS(Axis).#OPEN = 0
DCOM(Axis)= 0 
MFLAGS(Axis).12=0 ! Encoder Direction, 0 = direct, 1 = invereted
MFLAGS(Axis).13=0 ! Driver Output, 0 = direct, 1 = inverted
SLCPRD(Axis) = 1E9  
! Safety Parameters
XVEL(Axis) = 2E8  ! Maximum Velocity
XCURV(Axis) = 75  ! Maximum motor command during motion, 100% = 10V
XCURI(Axis) = 75  ! Maximum motor command during rest, 100% = 10V
XRMS(Axis) = 50   ! Maximum RMS of motor command. Do not exceed 75%
XRMST(Axis)= 3230 ! RMS time constant
CERRI(Axis) = 1E4 ! Critical position error at rest
CERRV(Axis) = 1E5 ! Critical position error at CV
CERRA(Axis) = 1E5 ! Critical position error at Accel
FMASK(Axis).#CPE = 1! Enable critical position error protection
FMASK(Axis).#CL = 1 ! Enable current limit protection
!LEFT_LIM = 3E6 
!RIGHT_LIM = -3E6
! Nanomotion Parameters
MFLAGS(Axis).7 = 0! Disable Enable Nanomotion mode, since we are using an AB5 amplifier
SETTLE(Axis)=5    ! Require motor to be within TARGRAD consecutively for 5 msec before declaring end of move.
TARGRAD(Axis)= 10 ! Target radius = 50 nm
SLDZMAX(Axis)= 0  ! Dead Zone Maximum 
SLDZMIN(Axis)= 0  ! Dead Zone Minimum
SLZFF(Axis) = 0   ! Zero feed forward at 0 counts before target
SLFRC(Axis) = 0   ! friction offset compensation
SLFRCD(Axis) = 0  ! Dynamic friction offset compensation
SLIOFFS(Axis) = 0 ! DAC offset
SLVSOFD(Axis) = 0.707 ! Low pass filter damping
! Default Motion Parameters
VEL(Axis)= 1E4    ! Velocity
ACC(Axis)= 1E5    ! Accel
DEC(Axis)= 1E5    ! Decel
KDEC(Axis)= 1E9   ! Kill decel 
JERK(Axis)= 1E8 !  
! Servo Parameters, relaexed
SLPKP(Axis)=40   ! KP (POSITION GAIN)
SLVKP(Axis)=2000   ! KV (VELOCITY GAIN)
SLVKI(Axis)=500  ! KI (INTEGRATOR GAIN)
SLVLI(Axis)=50    ! Anti windup integrator limit, % of 100%
SLVSOF(Axis)=1000  ! Low pass filter bandwidth
SLAFF(Axis) = 0   ! Accel feed forward
MFLAGS(Axis).14 = 0! Enable notch filter
RET

ON home_R(Axis)=1; CALL HOME_R; RET
ON home_F(Axis)=1; CALL HOME_F; RET
ON acpar(Axis)=1;  CALL AC_TUNED_PAR; RET
ON dcpar(Axis)=1;  CALL DC_TUNED_PAR; RET


#8
!THIS BUFFER STOPS MOTION PROGRAMS ON A PER AXIS BASES AND RETURNS TO DC MODE
!Done(Axis) Flag is used by EPICS to check when op operation is complete
!Written by Joe Sullivan (BCDA) and Suresh (8-ID) (March 2006)

GLOBAL stop_all(8)
GLOBAL Done(8)
GLOBAL acpar(8)
GLOBAL dcpar(8)
GLOBAL dcEnable(8)
LOCAL Axis

STOP

STOP_MOVE:

! Stop corresponding buffer
stop Axis

! Halt motor motion and wait until done
HALT(Axis)
TILL ^MST(Axis).#MOVE

!Test that buffer and autoroutine are stopped (with timeout)
TILL ^PST(Axis).#RUN & ^PST(Axis).#AUTO,500

!clear the AC and DC Flags to make sure they are cleared
acpar(Axis)=0;dcpar(Axis)=0;

! Switch to DC Mode
if dcEnable(Axis) > 0
    dcpar(Axis)=1; TILL dcpar(Axis)=0
end

! Clear flags
stop_all(Axis) = 0; Done(Axis) = 1;

RET

ON stop_all(0)=1; Axis=0;CALL STOP_MOVE; RET
ON stop_all(1)=1; Axis=1;CALL STOP_MOVE; RET
ON stop_all(2)=1; Axis=2;CALL STOP_MOVE; RET
ON stop_all(3)=1; Axis=3;CALL STOP_MOVE; RET
ON stop_all(4)=1; Axis=4;CALL STOP_MOVE; RET
ON stop_all(5)=1; Axis=5;CALL STOP_MOVE; RET
ON stop_all(6)=1; Axis=6;CALL STOP_MOVE; RET
ON stop_all(7)=1; Axis=7;CALL STOP_MOVE; RET









