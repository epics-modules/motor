/* EnsembleTrajectoryScan.h
 * 
 * This file is included in EnsembleTrajectoryScan.st.
 */


/* State codes for Build, Read and Execute. Careful, these must match the
 * corresponding MBBI records, but there is no way to check this */
#define BUILD_STATE_DONE            0
#define BUILD_STATE_BUSY            1
#define READ_STATE_DONE             0
#define READ_STATE_BUSY             1
#define EXECUTE_STATE_DONE          0
#define EXECUTE_STATE_MOVE_START    1
#define EXECUTE_STATE_EXECUTING     2
#define EXECUTE_STATE_FLYBACK       3

/* Status codes for Build, Execute and Read */
#define STATUS_UNDEFINED 0
#define STATUS_SUCCESS   1
#define STATUS_FAILURE   2
#define STATUS_ABORT     3
#define STATUS_TIMEOUT   4
#define STATUS_WARNING   5

/* Time modes */
#define TIME_MODE_TOTAL         0
#define TIME_MODE_PER_ELEMENT   1

/* Move modes */
#define MOVE_MODE_RELATIVE   0
#define MOVE_MODE_ABSOLUTE   1
#define MOVE_MODE_HYBRID     2

/* The maximum number of axes per controller.  If this is changed from 8
 * then many assign statements in this file must be changed */
#define MAX_AXES 8

#define MSGSIZE 40

/* Define PVs */
int     debugLevel;  assign debugLevel   to "{P}{R}DebugLevel.VAL"; 
monitor debugLevel;
int     numAxes;     assign numAxes      to "{P}{R}NumAxes.VAL"; 
monitor numAxes;
int     nelements;   assign nelements    to "{P}{R}Nelements.VAL"; 
monitor nelements;
int     npulses;     assign npulses      to "{P}{R}Npulses.VAL"; 
monitor npulses;
int     startPulses; assign startPulses  to "{P}{R}StartPulses.VAL"; 
monitor startPulses;
int     endPulses;   assign endPulses    to "{P}{R}EndPulses.VAL"; 
monitor endPulses;
int     nactual;     assign nactual      to "{P}{R}Nactual.VAL"; 
int     moveMode;    assign moveMode     to "{P}{R}MoveMode.VAL";    
monitor moveMode;
double  time;        assign time         to "{P}{R}Time.VAL";      
monitor time;
double  timeScale;   assign timeScale    to "{P}{R}TimeScale.VAL"; 
monitor timeScale;
int     timeMode;    assign timeMode     to "{P}{R}TimeMode.VAL";    
monitor timeMode;
double  accel;       assign accel        to "{P}{R}Accel.VAL";     
monitor accel;
int     build;       assign build        to "{P}{R}Build.VAL";     
monitor build;
int     buildState;  assign buildState   to "{P}{R}BuildState.VAL"; 
int     buildStatus; assign buildStatus  to "{P}{R}BuildStatus.VAL"; 
string  buildMessage;assign buildMessage to "{P}{R}BuildMessage.VAL";
int     simMode;     assign simMode      to "{P}{R}SimMode.VAL";   
monitor simMode;
int     execute;     assign execute      to "{P}{R}Execute.VAL";   
monitor execute;
int     execState;   assign execState    to "{P}{R}ExecState.VAL";  
monitor execState; 
int     execStatus;  assign execStatus   to "{P}{R}ExecStatus.VAL";   
string  execMessage; assign execMessage  to "{P}{R}ExecMessage.VAL";
int     abort;       assign abort        to "{P}{R}Abort.VAL";   
monitor abort;
int     readback;    assign readback     to "{P}{R}Readback.VAL";   
monitor readback;
int     readState;   assign readState    to "{P}{R}ReadState.VAL";   
int     readStatus;  assign readStatus   to "{P}{R}ReadStatus.VAL";   
string  readMessage; assign readMessage  to "{P}{R}ReadMessage.VAL";
double  timeTrajectory[MAX_ELEMENTS];
assign  timeTrajectory to  "{P}{R}TimeTraj.VAL"; 
monitor timeTrajectory;
string  trajectoryFile; assign trajectoryFile to "{P}{R}TrajectoryFile.VAL";
monitor trajectoryFile;

/*** BEGIN: Specific to MAX_trajectoryScan.st ***/

/* time since trajectory start */
double  elapsedTime; assign elapsedTime  to "{P}{R}ElapsedTime.VAL";      

/* Number (0..15) of general purpose I/O bit to be used for output pulses (e.g., to trigger detector). */
int     outBitNum;   assign outBitNum    to "{P}{R}OutBitNum.VAL"; 
monitor outBitNum;

/* Number (0..15) of general purpose I/O bit to be used for input start-trajectory pulse. */
int     inBitNum;    assign inBitNum     to "{P}{R}InBitNum.VAL"; 
monitor inBitNum;

/* time in seconds from trajectory start */
double  realTimeTrajectory[MAX_ELEMENTS];
assign  realTimeTrajectory to "{P}{R}realTimeTrajectory.VAL"; 

/* raw values read from the controller, generally while a trajectory is executing */
int  motorCurrentRaw[MAX_AXES];
int  motorCurrentVRaw[MAX_AXES];
int  motorCurrentARaw[MAX_AXES];

/* EPICS motor record resolution */
double  epicsMotorMres[MAX_AXES]; 
assign  epicsMotorMres to {"","","","","","","",""};
monitor epicsMotorMres;

/* EPICS motor-controller card number.  (For now, we talk to the controller via drvMAX.cc) */
int  epicsMotorCard[MAX_AXES]; 
assign  epicsMotorCard to {"","","","","","","",""};
monitor epicsMotorCard;

/* EPICS motor record soft limits */
double  epicsMotorHLM[MAX_AXES]; 
assign  epicsMotorHLM to {"","","","","","","",""};
monitor epicsMotorHLM;
double  epicsMotorLLM[MAX_AXES]; 
assign  epicsMotorLLM to {"","","","","","","",""};
monitor epicsMotorLLM;

double  motorMinSpeed[MAX_AXES]; 
assign  motorMinSpeed to
        {"{P}{R}M1MinSpeed.VAL",
         "{P}{R}M2MinSpeed.VAL",
         "{P}{R}M3MinSpeed.VAL",
         "{P}{R}M4MinSpeed.VAL",
         "{P}{R}M5MinSpeed.VAL",
         "{P}{R}M6MinSpeed.VAL",
         "{P}{R}M7MinSpeed.VAL",
         "{P}{R}M8MinSpeed.VAL"};

double  motorMaxSpeed[MAX_AXES]; 
assign  motorMaxSpeed to
        {"{P}{R}M1MaxSpeed.VAL",
         "{P}{R}M2MaxSpeed.VAL",
         "{P}{R}M3MaxSpeed.VAL",
         "{P}{R}M4MaxSpeed.VAL",
         "{P}{R}M5MaxSpeed.VAL",
         "{P}{R}M6MaxSpeed.VAL",
         "{P}{R}M7MaxSpeed.VAL",
         "{P}{R}M8MaxSpeed.VAL"};

/*** END: Specific to MAX_trajectoryScan.st ***/

/*** BEGIN: Specific to MAX_trajectoryScan.st and EnsembleTrajectoryScan.st ***/

double  motorStart[MAX_AXES]; 
assign  motorStart to
        {"{P}{R}M1Start.VAL",
         "{P}{R}M2Start.VAL",
         "{P}{R}M3Start.VAL",
         "{P}{R}M4Start.VAL",
         "{P}{R}M5Start.VAL",
         "{P}{R}M6Start.VAL",
         "{P}{R}M7Start.VAL",
         "{P}{R}M8Start.VAL"};

/*** END: Specific to MAX_trajectoryScan.st and EnsembleTrajectoryScan.st ***/

/*** BEGIN: Specific to EnsembleTrajectoryScan.st ***/

int pulseDir;
assign pulseDir to "{P}{R}PulseDir";
monitor pulseDir;

double pulseLenUS;
assign pulseLenUS to "{P}{R}PulseLenUS";
monitor pulseLenUS;

int pulseSrc;
assign pulseSrc to "{P}{R}PulseSrc";
monitor pulseSrc;

/*** END: Specific to EnsembleTrajectoryScan.st ***/

int     moveAxis[MAX_AXES]; 
assign  moveAxis     to
        {"{P}{R}M1Move.VAL",
         "{P}{R}M2Move.VAL",
         "{P}{R}M3Move.VAL",
         "{P}{R}M4Move.VAL",
         "{P}{R}M5Move.VAL",
         "{P}{R}M6Move.VAL",
         "{P}{R}M7Move.VAL",
         "{P}{R}M8Move.VAL"};
monitor moveAxis;

double  motorTrajectory[MAX_AXES][MAX_ELEMENTS]; 
assign  motorTrajectory to
        {"{P}{R}M1Traj.VAL",
         "{P}{R}M2Traj.VAL",
         "{P}{R}M3Traj.VAL",
         "{P}{R}M4Traj.VAL",
         "{P}{R}M5Traj.VAL",
         "{P}{R}M6Traj.VAL",
         "{P}{R}M7Traj.VAL",
         "{P}{R}M8Traj.VAL"};
monitor motorTrajectory;

double  motorReadbacks[MAX_AXES][MAX_PULSES]; 
assign  motorReadbacks to
        {"{P}{R}M1Actual.VAL",
         "{P}{R}M2Actual.VAL",
         "{P}{R}M3Actual.VAL",
         "{P}{R}M4Actual.VAL",
         "{P}{R}M5Actual.VAL",
         "{P}{R}M6Actual.VAL",
         "{P}{R}M7Actual.VAL",
         "{P}{R}M8Actual.VAL"};

double  motorError[MAX_AXES][MAX_PULSES]; 
assign  motorError  to
        {"{P}{R}M1Error.VAL",
         "{P}{R}M2Error.VAL",
         "{P}{R}M3Error.VAL",
         "{P}{R}M4Error.VAL",
         "{P}{R}M5Error.VAL",
         "{P}{R}M6Error.VAL",
         "{P}{R}M7Error.VAL",
         "{P}{R}M8Error.VAL"};

double  motorCurrent[MAX_AXES]; 
assign  motorCurrent to
        {"{P}{R}M1Current.VAL",
         "{P}{R}M2Current.VAL",
         "{P}{R}M3Current.VAL",
         "{P}{R}M4Current.VAL",
         "{P}{R}M5Current.VAL",
         "{P}{R}M6Current.VAL",
         "{P}{R}M7Current.VAL",
         "{P}{R}M8Current.VAL"};

double  motorMDVS[MAX_AXES]; 
assign  motorMDVS   to
        {"{P}{R}M1MDVS.VAL",
         "{P}{R}M2MDVS.VAL",
         "{P}{R}M3MDVS.VAL",
         "{P}{R}M4MDVS.VAL",
         "{P}{R}M5MDVS.VAL",
         "{P}{R}M6MDVS.VAL",
         "{P}{R}M7MDVS.VAL",
         "{P}{R}M8MDVS.VAL"};
monitor motorMDVS;

double  motorMDVA[MAX_AXES]; 
assign  motorMDVA   to
        {"{P}{R}M1MDVA.VAL",
         "{P}{R}M2MDVA.VAL",
         "{P}{R}M3MDVA.VAL",
         "{P}{R}M4MDVA.VAL",
         "{P}{R}M5MDVA.VAL",
         "{P}{R}M6MDVA.VAL",
         "{P}{R}M7MDVA.VAL",
         "{P}{R}M8MDVA.VAL"};

int     motorMDVE[MAX_AXES]; 
assign  motorMDVE   to
        {"{P}{R}M1MDVE.VAL",
         "{P}{R}M2MDVE.VAL",
         "{P}{R}M3MDVE.VAL",
         "{P}{R}M4MDVE.VAL",
         "{P}{R}M5MDVE.VAL",
         "{P}{R}M6MDVE.VAL",
         "{P}{R}M7MDVE.VAL",
         "{P}{R}M8MDVE.VAL"};

double  motorMVA[MAX_AXES]; 
assign  motorMVA    to
        {"{P}{R}M1MVA.VAL",
         "{P}{R}M2MVA.VAL",
         "{P}{R}M3MVA.VAL",
         "{P}{R}M4MVA.VAL",
         "{P}{R}M5MVA.VAL",
         "{P}{R}M6MVA.VAL",
         "{P}{R}M7MVA.VAL",
         "{P}{R}M8MVA.VAL"};

int     motorMVE[MAX_AXES]; 
assign  motorMVE    to
        {"{P}{R}M1MVE.VAL",
         "{P}{R}M2MVE.VAL",
         "{P}{R}M3MVE.VAL",
         "{P}{R}M4MVE.VAL",
         "{P}{R}M5MVE.VAL",
         "{P}{R}M6MVE.VAL",
         "{P}{R}M7MVE.VAL",
         "{P}{R}M8MVE.VAL"};

double  motorMAA[MAX_AXES]; 
assign  motorMAA    to
        {"{P}{R}M1MAA.VAL",
         "{P}{R}M2MAA.VAL",
         "{P}{R}M3MAA.VAL",
         "{P}{R}M4MAA.VAL",
         "{P}{R}M5MAA.VAL",
         "{P}{R}M6MAA.VAL",
         "{P}{R}M7MAA.VAL",
         "{P}{R}M8MAA.VAL"};

int     motorMAE[MAX_AXES]; 
assign  motorMAE    to
        {"{P}{R}M1MAE.VAL",
         "{P}{R}M2MAE.VAL",
         "{P}{R}M3MAE.VAL",
         "{P}{R}M4MAE.VAL",
         "{P}{R}M5MAE.VAL",
         "{P}{R}M6MAE.VAL",
         "{P}{R}M7MAE.VAL",
         "{P}{R}M8MAE.VAL"};

/* We don't assign the EPICS motors here because there may be fewer than 
 * MAX_AXES actually in use. */
double  epicsMotorPos[MAX_AXES]; 
assign  epicsMotorPos to {"","","","","","","",""};
monitor epicsMotorPos;

int  epicsMotorDir[MAX_AXES]; 
assign  epicsMotorDir to {"","","","","","","",""};
monitor epicsMotorDir;

double  epicsMotorOff[MAX_AXES]; 
assign  epicsMotorOff to {"","","","","","","",""};
monitor epicsMotorOff;

double  epicsMotorDone[MAX_AXES]; 
assign  epicsMotorDone to {"","","","","","","",""};
monitor epicsMotorDone;

double  epicsMotorVELO[MAX_AXES]; 
assign  epicsMotorVELO to {"","","","","","","",""};
monitor epicsMotorVELO;

double  epicsMotorVMAX[MAX_AXES]; 
assign  epicsMotorVMAX to {"","","","","","","",""};
monitor epicsMotorVMAX;

double  epicsMotorVMIN[MAX_AXES]; 
assign  epicsMotorVMIN to {"","","","","","","",""};
monitor epicsMotorVMIN;

double  epicsMotorACCL[MAX_AXES]; 
assign  epicsMotorACCL to {"","","","","","","",""};
monitor epicsMotorACCL;


evflag buildMon;        sync build      buildMon;
evflag executeMon;      sync execute    executeMon;
evflag execStateMon;    sync execState  execStateMon;
evflag abortMon;        sync abort      abortMon;
evflag readbackMon;     sync readback   readbackMon;
evflag nelementsMon;    sync nelements  nelementsMon;
evflag motorMDVSMon;    sync motorMDVS  motorMDVSMon;

/* This is going to get messy.  We need to use Ensemble commands like "VELOCITY" and "PVT",
 * But they aren't available via the ASCII interface we're using to send commands.  So we
 * use ASCII-legal commands to tell an Aerobasic program the commands we want to execute,
 * and have the AeroBasic program execute those commands.
 */

/* defines for IGLOBAL values to tell AeroBasic program which command to invoke */
#define cmdDONE					0
#define cmdVELOCITY_ON			1
#define cmdVELOCITY_OFF			2
#define cmdHALT					3
#define cmdSTART				4
#define cmdPVT_INIT_TIME_ABS	5
#define cmdPVT_INIT_TIME_INC	6
#define cmdPVT1					7	/* PVT command for one motor (PVT i1 d1, d2, TIME d3) */
#define cmdPVT2					8	/* PVT command for two motors (PVT i1 d1, d2 i2 d3, d4 TIME d5)*/
#define cmdPVT3					9
#define cmdPVT4					10
#define cmdABORT				11
#define cmdSTARTABORT			12
#define cmdSCOPEBUFFER			13
#define cmdSCOPEDATA			14
#define cmdSCOPESTATUS			15
#define cmdSCOPETRIG			16	/* if IGLOBAL(iarg1Var)==1, stop scope */
#define cmdSCOPETRIGPERIOD		17
#define cmdDRIVEINFO			18	/* per Aerotech email, but not found in 4.02.004 doc */
#define cmdLINEAR				19	/* LINEAR @IGLOBAL(iarg1Var) DGLOBAL(darg1Var) FDGLOBAL(darg1Var) */
#define cmdDATAACQ_TRIG			20	/* DATAACQ @IGLOBAL(iarg1Var) TRIGGER IGLOBAL(1arg2Var) */
#define cmdDATAACQ_INP			21	/* DATAACQ @IGLOBAL(iarg1Var) INPUT IGLOBAL(1arg2Var) */
#define cmdDATAACQ_ON			22	/* DATAACQ @IGLOBAL(iarg1Var) ON IGLOBAL(1arg2Var) */
#define cmdDATAACQ_OFF			23	/* DATAACQ @IGLOBAL(iarg1Var) OFF */
#define cmdDATAACQ_READ			24	/* DATAACQ @IGLOBAL(iarg1Var) READ IGLOBAL(1arg2Var), IGLOBAL(1arg3Var) */
#define cmdDOTRAJECTORY			25	/* for motor IGLOBAL(iarg1Var); 3*IGLOBAL(iarg2Var) DGLOBALs (PVT) have been loaded */

#define cmdVar		45
#define iarg1Var	46
#define iarg2Var	47
#define iarg3Var	48
#define iarg4Var	49
#define darg1Var	1
#define darg2Var	2
#define darg3Var	3
#define numIArg		44
#define numDArg		43
#define pvtWaitMSVar	42

/* Numerical values for first arg for cmdSCOPEDATA */
#define sd_PositionCommand	0
#define sd_PositionFeedback	1
#define sd_ExternalPosition	2
#define sd_AxisFault		3
#define sd_AxisStatus		4
#define sd_AnalogInput0		5
#define sd_AnalogInput1		6
#define sd_AnalogOutput0	7
#define sd_AnalogOutput1	8
#define sd_DigitalInput0	9
#define sd_DigitalInput1	10
#define sd_DigitalOutput0	11
#define sd_DigitalOutput1	12
#define sd_CurrentCommand	13
#define sd_CurrentFeedback	14
#define sd_OptionalData1	15
#define sd_OptionalData2	16
#define sd_ProgramCounter	17
