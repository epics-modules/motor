#/ Controller version  = 4.50
#/ Date = 04/26/2006 16:41
#/ User remarks = hello
#0
!Note: Each motor needs all the programs in BUFFER 0 copied into its corresponding buffer. 
!Note: ie: motor 1 - buffer #0 , motor 2 - buffer #1 ..... etc. 
!Note: VEL and ACC are commented out for the AC mode as they are set from EPICS MEDM screens.
!Note: VEL and ACC are DEFINED for the DC mode as they are not defined from EPICS MEDM screens
!ACC is defined in the homing routines as they are defined from EPICS during Homing but VEL is defined.

GLOBAL acpar(4)
GLOBAL dcpar(4)
GLOBAL INT home_F(4)
GLOBAL INT home_R(4)
GLOBAL target_pos(4)
GLOBAL jog_vel(4)
GLOBAL Done(4)
GLOBAL opReq(4)

! Each AXIS has a copy of these functions in a corresponding buffer 
LOCAL Axis
Axis = 0

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

!THESE PROGRAMS ARE SMALL ROUTINES THAT ARE REQUIRED FOR CO-ORDINATING MOVEMENT WITH EPICS. THIS IS ESPECIALLY NEEDED
!BECAUSE OF THE AC AND DC MODE SWITCHING NEEDED for AB2 amplifier BEFORE AND AFTER EVERY MOVE.
!Written by Joe Sullivan (BCDA) and Suresh (8-ID) (March 2006)

ABS_MOVE:
! Switch to AC(Servo) Mode
acpar(Axis)=1; TILL acpar(Axis)=0
PTP(Axis),target_pos(Axis)
TILL ^MST(Axis).#MOVE
! Switch to DC(Position) Mode
dcpar(Axis)=1; TILL dcpar(Axis)=0
RET

REL_MOVE:
acpar(Axis)=1; TILL acpar(Axis)=0
PTP/r(Axis),target_pos(Axis)
TILL ^MST(Axis).#MOVE
dcpar(Axis)=1; TILL dcpar(Axis)=0
RET


JOG_MOVE:
acpar(Axis)=1; TILL acpar(Axis)=0
JOG/v(Axis),jog_vel(Axis)
TILL ^MST(Axis).#MOVE
dcpar(Axis)=1; TILL dcpar(Axis)=0
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
!VEL(Axis) = 102400*2 !2 mm/sec  !defined from EPICS
ACC(Axis) = VEL(Axis)*10
DEC(Axis) = ACC(Axis)
KDEC(Axis) = DEC(Axis)*2
JERK(Axis) = ACC(Axis)*20

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
dcpar(Axis)=1; TILL dcpar(Axis)=0
!DISABLE (Axis)
DISP "Homing is DONE ......"
home_R(Axis) = 0;
RET


HOME_F:
! Load tuned parameters for AC mode for AB2 Amplifier
!disable this when using AB5 amplifier
acpar(Axis)=1; TILL acpar(Axis)=0

!VEL(Axis) = 102400*2 !2 mm/sec !defined from EPICS
ACC(Axis) = VEL(Axis)*10 
DEC(Axis) = ACC(Axis)*0.5
KDEC(Axis) = DEC(Axis)
JERK(Axis) = ACC(Axis)*10

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
dcpar(Axis)=1; TILL dcpar(Axis)=0
!DISABLE (Axis)
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
  !!!!     BLOCK       !All commands between BLOCK and END will be executed in one controller cycle (1 msec)
DISABLE(Axis);
SLCPRD(Axis)=1E9; !Set this parameter for Nanomotion with High res. on sin/cos encoder to over come a bug in ACS related to Commutating motors
!VEL(Axis)= 102400 * 1.0; !defined from EPICS
!ACC(Axis)=VEL(Axis)*10;  !defined from EPICS
!SET DC_MODE to 0 and SET Nanomotion bit to 1 resp.
MFLAGS(Axis).30 = 0;
MFLAGS(Axis).7 = 1;
XVEL(Axis)=2.048E7;
SLPKP(Axis)=300;
SLVKP(Axis)=20;
SLVKI(Axis)= 1600;
SLFRC(Axis)=17;
SLDZMIN(Axis)=2;
SLDZMAX(Axis)=10;
TARGRAD(Axis) = SLDZMAX(Axis);
SETTLE(Axis) = 10;
DEC(Axis)=ACC(Axis)*0.5;
KDEC(Axis)=DEC(Axis);
JERK(Axis)=ACC(Axis)*10;
  !!!!     END         !All commands between BLOCK and END will be executed in one controller cycle (1 msec)
ENABLE(Axis);
acpar(Axis)=0;
RET


DC_TUNED_PAR:
!SET DC_MODE to 1 and SET Nanomotion bit to 0 resp.
   !!!!     BLOCK        !All commands between BLOCK and END will be executed in one controller cycle (1 msec)
VEL(Axis)=15;
ACC(Axis)=VEL(Axis)*10;
MFLAGS(Axis).30 = 1;
MFLAGS(Axis).7 =0;
XVEL(Axis)=2.048E7;
SLPKP(Axis)=2500;
SLVKP(Axis)=20;
SLVKI(Axis)= 9000;
SLFRC(Axis)=0;
SETTLE(Axis) = 10;
DEC(Axis)=ACC(Axis);
KDEC(Axis)=DEC(Axis);
JERK(Axis)=ACC(Axis)*10;
!SLDZMIN(Axis)=2;SLDZMAX(Axis)=10;TARGRAD(Axis) = SLDZMAX(Axis);
  !!!!       END        !All commands between BLOCK and END will be executed in one controller cycle (1 msec)
dcpar(Axis)=0;
RET


ON home_R(0)=1; Axis = 0; CALL HOME_R; RET
ON home_F(0)=1; Axis = 0; CALL HOME_F; RET
ON acpar(0)=1;Axis=0;CALL AC_TUNED_PAR; RET
ON dcpar(0)=1;Axis=0;CALL DC_TUNED_PAR; RET





#4
!THIS BUFFER STOPS MOTION PROGRAMS ON A PER AXIS BASES AND RETURNS TO DC MODE
!Done(Axis) Flag is used by EPICS to check when op operation is complete
!Written by Joe Sullivan (BCDA) and Suresh (8-ID) (March 2006)

GLOBAL stop_all(4)
GLOBAL Done(4)
GLOBAL acpar(4)
GLOBAL dcpar(4)
LOCAL INT Axis
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
acpar(0)=0;dcpar(0)=0;

! Switch to DC Mode
dcpar(Axis)=1; TILL dcpar(Axis)=0

! Clear flags
stop_all(Axis) = 0; Done(Axis) = 1;

RET

ON stop_all(0)=1; Axis=0;CALL STOP_MOVE; RET






