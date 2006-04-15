program XPS_trajectoryScan("P=13BMC:,R=traj1,M1=m33,M2=m34,M3=m35,M4=m36,M5=m37,M6=m38,M7=m39,M8=m40")

/*  
 *  This was adapted from trajectoryScan.st written by Mark Rivers!
 *
 *  This sequencer program works with trajectoryScan.db.  It implements
 *  coordinated trajectory motion with the Newport XPS-C8 motor controller.
 *  It can be used with the Newport General Purpose Diffractometer or with any
 *  other set of motors connected to that controller.
 *  
 *  This inforces a minimum trajectory element period of 0.1s because the SNL
 *  must be processed during each element period for the gathering to perform
 *  correctly
 *
 *  This has been converted to use the Asyn XPS driver. 
 *
 *  Jon Kelly
 *  24th May 2005
 *
 * Modifications:
*/
/* typrdef to remove warnings from xps_c8_driver.h */
%%typedef int bool;

%% #include <string.h>
%% #include <stdio.h>
%% #include "xps_c8_driver.h"
%% #include "Socket.h"
%% #include <hostLib.h>
%% #include <remLib.h>
%% #include <netDrv.h>


/* This program must be compiled with the recursive option */
option +r;

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

/* Time modes */
#define TIME_MODE_TOTAL         0
#define TIME_MODE_PER_ELEMENT   1

/* Move modes */
#define MOVE_MODE_RELATIVE   0
#define MOVE_MODE_ABSOLUTE   1
#define MOVE_MODE_HYBRID     2


/* Maximum number of motors on MM4005 */
#define MAX_AXES 8

/* Maximum # of trajectory elements.  The number of points XPS can accept
 * is almost unlimited because the data is stored in a file but  the channel 
 * access limit with a double data type is 2000.
 */
#define MAX_ELEMENTS 2000

/* Maximum # of output pulses.  At the moment the pulses are defined by a
 * timer which is synchronised with the trajectory points so the max number
 * of pulses equals the max number of elements.
 */
#define MAX_PULSES 2000

/* Polling interval for waiting for motors to reach their targets */
#define POLL_INTERVAL 0.1

/* getSocket timeout*/
#define SOCKET_TIMEOUT 	1

/* Used within the exec state as a timeout within the while loops which wait for
 * the ss xpsTrajectoryRun to catch up */
#define COUNT_TIMEOUT	100000

/* If the trajectory element period is bellow this value the state program 
 * will not be able to keep up! */
#define MINPERIOD 0.1


/* XPS specific PVs */
string groupName;  assign groupName 	to "{P}{R}GroupName.VAL";
string ipAddress;  assign ipAddress 	to "{P}{R}IP.VAL";
string asynPort;   assign asynPort      to "{P}{R}AsynPort.VAL";
int    xpsPort;	   assign xpsPort	to "{P}{R}XPSPort.VAL";
string axisName[8];	    
		   assign axisName 	to 
						{"{P}{R}Axis1.VAL",
					 	 "{P}{R}Axis2.VAL",
					 	 "{P}{R}Axis3.VAL",
					 	 "{P}{R}Axis4.VAL",
					 	 "{P}{R}Axis5.VAL",
					 	 "{P}{R}Axis6.VAL",
					 	 "{P}{R}Axis7.VAL",
					 	 "{P}{R}Axis8.VAL"};


/* Define PVs */
int    numAxes;     assign numAxes      to "{P}{R}NumAxes.VAL"; 
                                                        monitor numAxes;
int    nelements;   assign nelements    to "{P}{R}Nelements.VAL"; 
                                                        monitor nelements;
int    npulses;     assign npulses      to "{P}{R}Npulses.VAL"; 
                                                        monitor npulses;

int    moveMode;    assign moveMode     to "{P}{R}MoveMode.VAL";    
                                                        monitor moveMode;
double time;        assign time         to "{P}{R}Time.VAL";      
                                                        monitor time;
double timeScale;   assign timeScale    to "{P}{R}TimeScale.VAL"; 
                                                        monitor timeScale;
int    timeMode;    assign timeMode     to "{P}{R}TimeMode.VAL";    
                                                        monitor timeMode;
double accel;       assign accel        to "{P}{R}Accel.VAL";     
                                                        monitor accel;

int    build;       assign build        to "{P}{R}Build.VAL";     
                                                        monitor build;
int    buildState;  assign buildState   to "{P}{R}BuildState.VAL"; 
int    buildStatus; assign buildStatus  to "{P}{R}BuildStatus.VAL"; 
string buildMessage;assign buildMessage to "{P}{R}BuildMessage.VAL";

int    simMode;     assign simMode      to "{P}{R}SimMode.VAL";   
                                                        monitor simMode;
int    execute;     assign execute      to "{P}{R}Execute.VAL";   
                                                        monitor execute;
int    execState;   assign execState    to "{P}{R}ExecState.VAL";
							monitor execState;   
int    execStatus;  assign execStatus   to "{P}{R}ExecStatus.VAL";   
string execMessage; assign execMessage  to "{P}{R}ExecMessage.VAL";
int    abort;       assign abort        to "{P}{R}Abort.VAL";   
                                                        monitor abort;

int    detOn;       assign detOn        to "{P}{R}DetOn.PROC";
int    detOff;      assign detOff       to "{P}{R}DetOff.PROC";

int    readback;    assign readback     to "{P}{R}Readback.VAL";   
                                                        monitor readback;
int    readState;   assign readState    to "{P}{R}ReadState.VAL";   
int    readStatus;  assign readStatus   to "{P}{R}ReadStatus.VAL";   
string readMessage; assign readMessage  to "{P}{R}ReadMessage.VAL";

double timeTrajectory[MAX_ELEMENTS];
                    assign timeTrajectory to  "{P}{R}TimeTraj.VAL"; 
                                                        monitor timeTrajectory;

int    moveAxis[MAX_AXES]; 
                    assign moveAxis     to
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
                    assign motorTrajectory to
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

double  epicsMotorPos[MAX_AXES]; 
                    assign  epicsMotorPos  to
                                           {"{P}{M1}.VAL",
                                            "{P}{M2}.VAL",
                                            "{P}{M3}.VAL",
                                            "{P}{M4}.VAL",
                                            "{P}{M5}.VAL",
                                            "{P}{M6}.VAL",
                                            "{P}{M7}.VAL",
                                            "{P}{M8}.VAL"};
                                                        monitor epicsMotorPos;

double  epicsMotorDir[MAX_AXES]; 
                    assign  epicsMotorDir  to
                                           {"{P}{M1}.DIR",
                                            "{P}{M2}.DIR",
                                            "{P}{M3}.DIR",
                                            "{P}{M4}.DIR",
                                            "{P}{M5}.DIR",
                                            "{P}{M6}.DIR",
                                            "{P}{M7}.DIR",
                                            "{P}{M8}.DIR"};
                                                        monitor epicsMotorDir;
double  epicsMotorOff[MAX_AXES]; 
                    assign  epicsMotorOff  to
                                           {"{P}{M1}.OFF",
                                            "{P}{M2}.OFF",
                                            "{P}{M3}.OFF",
                                            "{P}{M4}.OFF",
                                            "{P}{M5}.OFF",
                                            "{P}{M6}.OFF",
                                            "{P}{M7}.OFF",
                                            "{P}{M8}.OFF"};
                                                        monitor epicsMotorOff;
double  epicsMotorDone[MAX_AXES]; 
                    assign  epicsMotorDone  to
                                           {"{P}{M1}.DMOV",
                                            "{P}{M2}.DMOV",
                                            "{P}{M3}.DMOV",
                                            "{P}{M4}.DMOV",
                                            "{P}{M5}.DMOV",
                                            "{P}{M6}.DMOV",
                                            "{P}{M7}.DMOV",
                                            "{P}{M8}.DMOV"};
                                                        monitor epicsMotorDone;

evflag buildMon;        sync build      buildMon;
evflag executeMon;      sync execute    executeMon;
evflag abortMon;        sync abort      abortMon;
evflag readbackMon;     sync readback   readbackMon;
evflag nelementsMon;    sync nelements  nelementsMon;
evflag execStateMon;    sync execState  execStateMon;

int status;
int i;
int j;
int k;
int anyMoving;
int ncomplete;
int nextElement;
int nextra;
int dir;
int    pollSocket;
int    abortSocket;
int    socket;
int    positionSocket;
int    xpsStatus;
int    count;
double dtime;
double minPeriod;
double posActual;
double posTheory;
double expectedTime;
double initialPos[MAX_AXES];
double trajVel;
double velocityTrajectory[MAX_AXES][MAX_ELEMENTS];
string trajFileName;

%%char divisor[] = "1";	/* gather/pulse every # traj elements */
%%char GPIOname[] = "GPIO4.DO";	/* DB15 Connector */
%%char pulsemask[] = "63"; /* Defines which pins are pulsed, 63 in base 2 ->00011111 */
%%char strNpulses[10];
%%char strElementNumber[10];
%%char nullchar[] = " ";

/* All PVs which will be accessed in local C functions need to have their index
 * extracted with pvIndex() */
int motorCurrentIndex[MAX_AXES];
int epicsMotorDoneIndex[MAX_AXES];

/* Note, this should be time_t, but SNL doesn't understand that.  This is
 * the defininition in vxWorks. */
unsigned long startTime;  

/* Define escaped C functions at end of file */
%% static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos);
%% static int getMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitMotors(SS_ID ssId, struct UserVar *pVar);
%% static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);

%% static int getSocket(SS_ID ssId, struct UserVar *pVar);
%% static int trajectoryExecute(SS_ID ssId, struct UserVar *pVar);
%% static int buildAndVerify(SS_ID ssId, struct UserVar *pVar);
%% static int currentElement(SS_ID ssId, struct UserVar *pVar);
%% static int getGroupStatus(SS_ID ssId, struct UserVar *pVar);
%% static int readBackError(SS_ID ssId, struct UserVar *pVar);
%% static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar);


ss xpsTrajectoryScan {

    /* Initialize things when first starting */
    state init {
        when() {
            
	   /* Get the values from the fields*/ 
	   pvGet(groupName);
	   pvGet(ipAddress);
	   pvGet(xpsPort);
	   pvGet(asynPort);
	   xpsStatus = 0;
	   ncomplete = 1;
	   for (i=0; i<8; ++i) 
	       pvGet(axisName[i]);
	    
	   /* Set the arrays to zero */
	   for (i=0; i<MAX_ELEMENTS; i++) {
               for (j=0; j<MAX_AXES; j++) {
	       motorTrajectory[j][i] = 0.0;
	       }
	    /* Velocity at each pulse/trajectory point */
	        velocityTrajectory[0][i] = 0.0;
	    }

	    /* setup a simple 10pt traj for test 
	    for (i=2; i<=8; i++) {
	    motorTrajectory[0][i] = 2;
	    }
	    motorTrajectory[1][0] = 0.1;
	    motorTrajectory[1][1] = 0.2;
	    motorTrajectory[1][2] = 0.5;
	    motorTrajectory[1][3] = 1;
	    motorTrajectory[1][4] = 1.8;
	    motorTrajectory[1][5] = 3;
	    motorTrajectory[1][6] = 2;
	    motorTrajectory[1][7] = 1;
	    motorTrajectory[1][8] = 0;
	    
	    motorTrajectory[2][1] = 0.5;
	    motorTrajectory[2][2] = 1;
	    motorTrajectory[2][3] = 0.5;
	    motorTrajectory[2][4] = 0;
	    motorTrajectory[2][5] = 0.5;
	    motorTrajectory[2][6] = 1;
	    motorTrajectory[2][7] = 0.5;
	    motorTrajectory[2][8] = 0;*/
	    
	   
	    /* Define the file name used within the XPS */
	    strcpy (trajFileName,"TrajectoryScan.trj"); 
	    
	    /* Force numAxes to be <= MAX_AXES */
            if (numAxes > MAX_AXES) numAxes = MAX_AXES;
            
	    /* Ask XPS for a socket 1 to drive 1 to poll*/
	    %%pVar->socket = getSocket(ssId, pVar);
	    %%pVar->pollSocket = getSocket(ssId, pVar);
	    %%pVar->abortSocket = getSocket(ssId, pVar);
	    %%pVar->positionSocket = getSocket(ssId, pVar);
	    
            for (j=0; j<numAxes; j++) {
                motorCurrentIndex[j]   =  pvIndex(motorCurrent[j]);
                epicsMotorDoneIndex[j] =  pvIndex(epicsMotorDone[j]);
            }

            /* Clear all event flags */
            efClear(buildMon);
            efClear(executeMon);
            efClear(abortMon);
            efClear(readbackMon);
            efClear(nelementsMon);
        } state monitor_inputs
    }

    /* Monitor inputs which control what to do (Build, Execute, Read) */
    state monitor_inputs {
        when(efTestAndClear(buildMon) && (build==1)) {
        } state build

        when(efTest(executeMon) && (execute==1) 
				&& (buildStatus == STATUS_SUCCESS)){
        } state execute

        when(efTestAndClear(readbackMon) && (readback==1)
				&& (execStatus == STATUS_SUCCESS)) {
        } state readback

    }


    /* Build and verify trajectory */
    state build {
        when() {
	    /* Set busy flag while building */
            buildState = BUILD_STATE_BUSY;
            pvPut(buildState);
            buildStatus=STATUS_UNDEFINED;
            pvPut(buildStatus);
            /* SPEC adds up the total time  */
            
	    /* If total time mode, calc time per element and write,
	       else use the array timeTraj as written by SPEC */
	    if (timeMode == TIME_MODE_TOTAL) {
                dtime = time/nelements;
		for (i=0; i<nelements; i++) timeTrajectory[i] = dtime;
                pvPut(timeTrajectory);
	    }
	    
            /* Compute expected time for trajectory & check element period */
            expectedTime=0;
            minPeriod = 100000;
	    for (i=0; i<nelements; i++) {
                expectedTime += timeTrajectory[i];
		
		if (minPeriod > timeTrajectory[i]) /* Used in build function */
		    minPeriod = timeTrajectory[i];
	    }
	    
	    time = expectedTime;
	    pvPut(time);
	    
	    %%buildAndVerify(ssId, pVar);
	     
	    /* Export values to PVs */
	    for (j=0; j<numAxes; j++) {       
                    pvPut(motorMVA[j]);
		    pvPut(motorMAA[j]);
	    } 
           /* Clear busy flag, post status */
            buildState = BUILD_STATE_DONE;
            pvPut(buildState);
            pvPut(buildStatus);
            pvPut(buildMessage);
            /* Clear build command, post.  This is a "busy" record, don't want
             * to do this until build is complete. */
            build=0;
            pvPut(build);
        } state monitor_inputs
    }


    state execute {
        when () {
	    
	    %%waitMotors(ssId, pVar);
	    
            /* Get the initial positions of the motors */
            for (j=0; j<numAxes; j++) initialPos[j] = epicsMotorPos[j];
            /* Move to start position if required */
            if (moveMode == MOVE_MODE_ABSOLUTE) {
                for (j=0; j<numAxes; j++) {
                    if (!moveAxis[j]) continue;
                    epicsMotorPos[j] = motorTrajectory[j][0];
                    pvPut(epicsMotorPos[j]);
                }
                %%waitEpicsMotors(ssId, pVar);
            }
	    
	    %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
	    /* Set busy flag */
            execState = EXECUTE_STATE_MOVE_START;
            pvPut(execState);
            /* Set status to INVALID */
            execStatus = STATUS_UNDEFINED;
            pvPut(execStatus);
	    strcpy(execMessage, " ");
	    	    
	    count = 0;
	    /* Wait until ss xpsTrajectoryRun has called getGroupStatus 
	       before this state calls the same function */

	    while (execState != EXECUTE_STATE_EXECUTING && 
	    	   count < COUNT_TIMEOUT ){
	        epicsThreadSleep(0.1);
	 	count++;
	    }
	    if (count == COUNT_TIMEOUT) {
		strcpy(execMessage, "Exec Timeout!");
		pvPut(execMessage);
		execStatus = STATUS_ABORT;
		pvPut(execStatus);
	    }	    
	    count = 0;
	    /* Wait until ss xpsTrajectoryRun has started the traj scan */
	    while ( xpsStatus != 45 && 
	        count < COUNT_TIMEOUT && 
	        execStatus != STATUS_FAILURE ){
		/*epicsThreadSleep(0.1);*/
	        count++;
	        %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
	    }
	    if (count == COUNT_TIMEOUT) {
		strcpy(execMessage, "Exec Timeout!");
		pvPut(execMessage);
		execStatus = STATUS_ABORT;
		pvPut(execStatus);
	    }
            readStatus=STATUS_UNDEFINED;
            pvPut(readStatus);
	    /* Start the detector */
	    detOn = 1;
            pvPut(detOn);
	    
	    /* Get start time of execute */	    
	    startTime = time(0);
	    
	    /* Add Event for 2nd element because the first element has started so it would
	       not be added by the execState=EXEC code */
	    %%pVar->status = EventAdd(pVar->pollSocket,\
				     pVar->axisName[0],\
			      "PVT.ElementNumberStart",\
					  	   "2",\
				 	     "DOPulse",\
					      GPIOname,\
					     pulsemask,\
					      nullchar);
	    if (status != 0) 
                printf("Error EventAdd ElementNumberStart %i element 1\n",status);
	    
        } state wait_execute
    }


    /* Wait for trajectory to complete */
    state wait_execute {
        when (execStatus == STATUS_ABORT) {
            /* The trajectory_abort state set has detected an abort. It has
             * already posted the status and message.  Don't execute flyback
             * return to top */
            execState = EXECUTE_STATE_DONE;
            pvPut(execState);
            /* Clear execute command, post.  This is a "busy" record, don't
             * want to do this until execution is complete. */
            execute=0;
            pvPut(execute);
	    efClear(executeMon);
        } state monitor_inputs

        when(execState==EXECUTE_STATE_EXECUTING) {
            /* Get the current motor positions, post them */
            %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
            for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);

            %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
	    
	    nextElement = ncomplete + 1;            
	    %%pVar->ncomplete = currentElement(ssId, pVar);
	    
	    /* Define the Pulse event once the previous one has been called */
	    if(ncomplete == nextElement && xpsStatus == 45) { /* Just started next element */
	    
	        sprintf(strElementNumber,"%i",(ncomplete + 1));	/* Convert to a string */
	    
		%%pVar->status = EventAdd(pVar->pollSocket,\
					 pVar->axisName[0],\
				  "PVT.ElementNumberStart",\
					  strElementNumber,\
						 "DOPulse",\
						  GPIOname,\
						 pulsemask,\
						  nullchar);
	        if (status != 0) 
                    printf("Error EventAdd ElementNumberStart %i element %i\n",status,i);
		
	    }
            if(xpsStatus == 45)
	        sprintf(execMessage, "Executing element %d/%d", 
                                        ncomplete, nelements);
            pvPut(execMessage);
            %%pVar->xpsStatus = getGroupStatus(ssId, pVar); 
            /* 12 = ready from move */
	    if (xpsStatus == 12) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_SUCCESS;
                strcpy(execMessage, " ");
            }
            /* See if the elapsed time is more than expected, time out */
            if (difftime(time(0), startTime) > (expectedTime+10)) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_TIMEOUT;
                strcpy(execMessage, "Timeout");
            }
            /* see if the xps group status reflects an error. */
	    /* 45 = performing a trajectory, <10 = notinitialised due to major error */
	    if (xpsStatus < 10) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_FAILURE;
                strcpy(execMessage,"XPS Status Error");
            }
        } state wait_execute

        when(execState==EXECUTE_STATE_FLYBACK) {
            /* Stop the detector */
            detOff = 1;
            pvPut(detOff);
            pvPut(execState);
            pvPut(execStatus);
            pvPut(execMessage);
	    
	    /* Make sure the motors have stopped */
	    %%waitMotors(ssId, pVar);
	    %%waitEpicsMotors(ssId, pVar);	    
            
	    /* Get the current motor positions, post them */
            %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
            for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
            for (j=0; j<numAxes; j++) {
                epicsMotorPos[j] = motorCurrent[j];
                pvPut(epicsMotorPos[j]);
            }
            %%waitEpicsMotors(ssId, pVar);
            
	    execState = EXECUTE_STATE_DONE;
            pvPut(execState);
            /* Clear execute command, post.  This is a "busy" record, don't
             * want to do this until execution is complete. */
            execute=0;
	    pvPut(execute);
	    efClear(executeMon);
           
        } state monitor_inputs
    }
    /* Read back actual positions */
    state readback {
        when() {
            /* Set busy flag */
            readState = READ_STATE_BUSY;
            pvPut(readState);
            readStatus=STATUS_UNDEFINED;
            pvPut(readStatus);
            /* Erase the readback and error arrays */
            for (j=0; j<numAxes; j++) {
                for (i=0; i<MAX_PULSES; i++) {
                    motorReadbacks[j][i] = 0.;
                    motorError[j][i] = 0.;
		    
                }
            }
            
	    %%readBackError(ssId, pVar);
            
	    /* Post the readback and error arrays */
            for (j=0; j<numAxes; j++) {
                pvPut(motorReadbacks[j]);
                pvPut(motorError[j]);
		pvPut(motorTrajectory[j]);
            }
            /* Clear busy flag */
            readState = READ_STATE_DONE;
            pvPut(readState);
            /* For now we are not handling read errors */
            readStatus = STATUS_SUCCESS;  
            pvPut(readStatus);
            strcpy(readMessage, " ");
            pvPut(readMessage);
            /* Clear readback command, post.  This is a "busy" record, don't
             * want to do this until readback is complete. */
            readback=0;
            pvPut(readback);
        } state monitor_inputs
    }
}


/* This state set simply monitors the abort input.  It is a separate state set
 * so that it is always active, no matter what the state of the trajectoryScan
 * state set. It also uses a separate socket. */
ss xpsTrajectoryAbort {
    state monitorAbort {
        when (efTestAndClear(abortMon) && (abort==1) &&
					(execState==EXECUTE_STATE_EXECUTING)) {

            execStatus = STATUS_ABORT;
            pvPut(execStatus);
            strcpy(execMessage, "Motion aborted");
            pvPut(execMessage);
	    
	    %%trajectoryAbort(ssId, pVar);
	    
            /* Clear abort command, post.  This is a "busy" record, don't
             * want to do this until abort command has been sent. */
            abort=0;
            pvPut(abort);
	    efClear(abortMon);
        } state monitorAbort
    }
}
/* This state was required because the trajectory execute call did not return
 * until the end of the trajectory. So to poll during the scan we call the 
 * scan from a separate state set.  Now the driver has been converted to Asyn the
 * socket no longer hangs so this SS could be recombined. */
ss xpsTrajectoryRun {
    state asyncExecute {
        when (efTest(execStateMon) && (execState == EXECUTE_STATE_MOVE_START)){
	
	efClear(executeMon);
	/*%%pVar->xpsStatus = getGroupStatus(ssId, pVar);*/
	execState = EXECUTE_STATE_EXECUTING;
	pvPut(execState);

	/* if ready to move */
	if (xpsStatus > 9 && xpsStatus < 20) {
		
	/* Call the C function from here so that the main program can poll */
	%%trajectoryExecute(ssId, pVar);
	} else {
        execStatus = STATUS_FAILURE;
	pvPut(execStatus);
	}
	
	} state asyncExecute
    }
}
/* This state set polls every second to keep the motor current position medm 
 * screen up to date when a scan is not running */
ss xpsTrajectoryPosition {
    state positionUpdate {
    	when (delay (1) && (execState == EXECUTE_STATE_DONE)){
	%%getMotorPositions(ssId, pVar, pVar->motorCurrent);
        for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
	} state positionUpdate
    }
}

%{

/* getMotorPositions returns the positions of each motor */
static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos)
{
    int status;

    /* Read the current positions of all the axes */
    
    status = GroupPositionCurrentGet(pVar->positionSocket,
    			pVar->groupName,pVar->numAxes,pos);
    if (status != 0)
        	printf(" Error performing GroupPositionCurrentGet%i\n",status);
    return(0);
}


/* Returns 0 when no motors are moving */
static int getMotorMoving(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int moving=0;
    int groupStatus;

    /* Read the current status of the group */

    status = GroupStatusGet(pVar->pollSocket,pVar->groupName,&groupStatus);
    if (status != 0)
        printf(" Error performing GroupStatusGet %i\n",status);
	
    if (groupStatus > 42)
        moving = 1;

    return(moving);
}

/* getEpicsMotorMoving returns the EPICS moving status of each motor, packed into 
 * a single int.  Bit 0 = motor 1, bit 1 = motor 2, etc. 0=not moving, 1=moving.
 * If the entire int is 0 then no motors are moving */
static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar)
{
    int j;
    int result=0, mask=0x01;

    for (j=0; j<pVar->numAxes; j++) {
        seq_pvGet(ssId, pVar->epicsMotorDoneIndex[j], 0);
        if (pVar->epicsMotorDone[j] == 0) result |= mask;
        mask = mask << 1;
    }
    return(result);
}

/* waitMotors waits for all motors to stop moving.  It reads and posts the
 * motor positions during each loop. */
static int waitMotors(SS_ID ssId, struct UserVar *pVar)
{
    int j;

    /* Logic is that we always want to post position motor positions 
     * after the end of move is detected. */
    while(getMotorMoving(ssId, pVar)) {
        /* Get the current motor positions, post them */
        getMotorPositions(ssId, pVar, pVar->motorCurrent);
        for (j=0; j<pVar->numAxes; j++) seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
        epicsThreadSleep(POLL_INTERVAL);
    }
    getMotorPositions(ssId, pVar, pVar->motorCurrent);
    for (j=0; j<pVar->numAxes; j++) seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
    return(0);
}

/* waitEpicsMotors waits for all motors to stop moving using the EPICS motor
 * records..  It reads and posts the motor positions during each loop. */
static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar)
{
    int j;

    /* Logic is that we always want to post position motor positions 
     * after the end of move is detected. */
    while(getEpicsMotorMoving(ssId, pVar)) {
        /* Get the current motor positions, post them */
        for (j=0; j<pVar->numAxes; j++) {
            pVar->motorCurrent[j] = pVar->epicsMotorPos[j];
            seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
        }
        epicsThreadSleep(POLL_INTERVAL);
    }
    for (j=0; j<pVar->numAxes; j++) {
        pVar->motorCurrent[j] = pVar->epicsMotorPos[j];
        seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
    }
    return(0);
}
/* Function to ask the XPS for a socket this requires Socket.h */
static int getSocket(SS_ID ssId, struct UserVar *pVar)
{
    int sock = 0;
    
	/* With the asyn driver we pass the asyn port name not the ip */
	
	sock = TCP_ConnectToServer(pVar->asynPort,0,SOCKET_TIMEOUT);
	if (sock < 0) 
            printf(" Error TCP_ConnectToServer %i\n",sock);
	return (sock);
}
/* Function to setup the gathering, events and start the trajectory */
static int trajectoryExecute(SS_ID ssId, struct UserVar *pVar)
{
/* Configure Gathering */
	int status,lastXpsStatus;
	int j;
	char *gatheringdata;
	char elp[] = ".ExternalLatchPosition";
	char nullchar[] = " ";
	int nloops = 1;	 /* Number of trajectory loops performed */
	char divisor[] = "1";	/* gather/pulse every # traj elements */
	char GPIOname[] = "GPIO4.DO";	/* DB15 Connector */
	char pulsemask[] = "63"; /* Defines which pins are pulsed, 63 in base 2 ->00011111 */
	char strNpulses[10];
	int servoPulses;
	
	/* Calc the time between pulses to set timer */
	servoPulses = (int) (pVar->time/pVar->npulses * 1e4);
	
	sprintf(strNpulses,"%i",(pVar->npulses+1));	/* Convert to a string */

	/* create a pointer to a char array 300 chars long */
	gatheringdata = (char *) calloc (sizeof (char), 300);
	
	/* write list of gathering parameters */
	for (j=0; j<pVar->numAxes; j++) {
	    strcat (gatheringdata, pVar->axisName[j]);
	    strcat (gatheringdata, elp);
	    if (j < (pVar->numAxes - 1))
	        strcat (gatheringdata, ",");
	}
	
	/* Define what is to be saved in the GatheringExternal.dat */
	status = GatheringExternalConfigurationSet(pVar->socket,1,gatheringdata);
	if (status != 0)
            printf(" Error performing GatheringConfigurationSet%i\n",status);

	/* Start the position capture/pulse when the trajectory starts */    
	status = EventAdd(pVar->socket,pVar->axisName[0],"PVT.TrajectoryStart",nullchar,
				"ExternalGatheringRun",strNpulses,divisor,nullchar);
	if (status != 0)
                printf(" Error EventAdd(socket,positioner,PVT.TrajectoryStart %i\n",status);
	
	 /* Do pulse to capture the initial position */ 
	status = EventAdd(pVar->socket,pVar->axisName[0],"PVT.TrajectoryStart",nullchar,
					"DOPulse",GPIOname,pulsemask,nullchar);
	if (status != 0)
      	     printf(" Error performing EventAdd PVT.TrajectoryStart%i\n",status);
        
	
	status = MultipleAxesPVTExecution(pVar->socket,pVar->groupName,
							"TrajectoryScan.trj",nloops);
	/* status -27 means the trajectory was aborted -22 is because the 
	   asyn driver attempted to resend the command */
	if ((status != 0) && (status != -27) && (status != -22))
       	    printf(" Error performing MultipleAxesPVTExecution\%i\n",status);
	
	/* The asyn driver does not let the sockets hang while the trajectory is 
	   performed so I have added a while loop and lastXpsStatus to remove 
	   the odd miss status reads */
	
	do {
	    epicsThreadSleep(0.2);
	    lastXpsStatus = pVar->xpsStatus; 
	    status = GroupStatusGet(pVar->socket,pVar->groupName,&pVar->xpsStatus);
            if (status != 0)
                printf(" Error performing GroupStatusGet %i\n",status);

	} while ((pVar->xpsStatus > 42) || (lastXpsStatus != pVar->xpsStatus)); /* Moving */
	
	/* Do pulse to capture the final position */ 
	status = EventAdd(pVar->socket,pVar->axisName[0],"Immediate",nullchar,
					"DOPulse",GPIOname,pulsemask,nullchar);
	if (status != 0)
      	    printf(" Error performing EventAdd Immediate %i\n",status);
        
	/* Save the gathered data to a file */	
	status = GatheringExternalStopAndSave(pVar->socket);
	
	/* status -30 means gathering not started i.e. aborted before the end of
	1 trajectory element */
	if ((status != 0) && (status != -30))
            printf(" Error performing GatheringExternalStopAndSave\%i\n",status);
	
	
	free (gatheringdata);   
	return (0);
}
/* Function to build install and verify trajectory */ 
static int buildAndVerify(SS_ID ssId, struct UserVar *pVar)
{
	char vxWorksString[100];
	FILE *trajFile;
	int i,j,status;
	double trajStep;
	double trajVel;
	double maxp;
	double minp;
	char outputFilename[100] = " ";
	double P0,P1,T0,T1;
	
	/* Set the velocity at each trajectory point 
	   Loop an extra 1 to add a zero vel element at the end 
	   of the scan */
	
	pVar->timeTrajectory[pVar->nelements] = pVar->timeTrajectory[pVar->nelements-1];
	
	
	for (i=0; i<(pVar->nelements); i++) {
            for (j=0; j<pVar->numAxes; j++) {
	    
	    if (pVar->timeTrajectory[i] == 0 && i > 0) {  
	        pVar->timeTrajectory[i] = pVar->timeTrajectory[i-1];
	    }
	    
	    
	    P0 = pVar->motorTrajectory[j][i];
	    P1 = pVar->motorTrajectory[j][i+1];
	    T0 = pVar->timeTrajectory[i];
	    T1 = pVar->timeTrajectory[i+1];
 
	    
	    /* Average either side of the point */
	    pVar->velocityTrajectory[j][i] = (P0 + P1)/(T0 + T1);
	    
	    if (i == (pVar->nelements-1)) /* Set final velocity to zero */
	        pVar->velocityTrajectory[j][i] = 0.0;
	    
	    
	    if (pVar->motorTrajectory[j][i] == 0) /* To prevent the motor reversing */
	    	pVar->velocityTrajectory[j][i] = 0;
	    
	    if (pVar->motorTrajectory[j][i+1] == 0) /* prevent the motor over shooting */
	        pVar->velocityTrajectory[j][i] = 0;
	    
	    
	   /* Impliment the accel time */
	   if ( i > 0 ) { /* i.e. not 1st point */
	     	/* If the motor is slowing down to zero vel */
		if (pVar->velocityTrajectory[j][i-1] > 0 && 
		     pVar->velocityTrajectory[j][i] == 0 &&
		     pVar->timeTrajectory[i] < pVar->accel) {
		     pVar->timeTrajectory[i] = pVar->accel;
		}
		/* If the motor is accelerating from zero */
		if (pVar->velocityTrajectory[j][i-1] == 0 && 
		     pVar->velocityTrajectory[j][i] > 0 &&
		     pVar->timeTrajectory[i] < pVar->accel) {
		     pVar->timeTrajectory[i] = pVar->accel;
		}   
	    }
	    if ( i == 0 ) { /* i.e. 1st point */
	        /* If the motor is accelerating from zero */
		
		if (pVar->velocityTrajectory[j][i] > 0 &&
		     pVar->timeTrajectory[i] < pVar->accel) {
		     pVar->timeTrajectory[i] = pVar->accel;
		}   
	    }
	    /*printf("i %i,V %f, P0 %f, P1 %f, T0 %f, T1 %f\n",
	    		i,pVar->velocityTrajectory[j][i],P0,P1,T0,T1);*/
	    }
	 }
	

	strcpy (vxWorksString,"XPS1:/Admin/Public/Trajectories/");
	strcat (vxWorksString,pVar->trajFileName);
	
	/* logon to the xps and open a trajectory file to write to
	 * using vxWorks specific functions */	    
	hostAdd ("XPS1",pVar->ipAddress);
	netDevCreate ("XPS1:", "XPS1", 1);
	remCurIdSet("Administrator", "Administrator");
	trajFile =  fopen (vxWorksString, "w");

			
        /* Define each element in trajectory file TrajectoryScan.trj
	 * time,  pos,vel,  pos,vel,  pos,vel, etc */
  	for (i=0; i<(pVar->nelements); i++) {
            for (j=0; j<pVar->numAxes; j++) {
                    
                if (pVar->moveMode == MOVE_MODE_RELATIVE) {
                    trajStep = pVar->motorTrajectory[j][i];
                } else {
                    trajStep = pVar->motorTrajectory[j][i+1] - pVar->motorTrajectory[j][i];
                }
		trajVel = pVar->velocityTrajectory[j][i];
                
		if (!(pVar->moveAxis[j])) {
		    trajStep = 0.0;	/* Axis turned off*/
		    trajVel = 0.0;
		}
		
		if(j == (pVar->numAxes-1)) 
		   fprintf(trajFile,"%f,%f \n",trajStep,trajVel);
		 
		if (j == 0)   
	           fprintf(trajFile,"%f, %f,%f, ",pVar->timeTrajectory[i],trajStep,trajVel);
		 
		if (j > 0 && j < (pVar->numAxes-1))
		   fprintf(trajFile,"%f,%f, ",trajStep,trajVel);
		
             }    
      	}
            
	 fclose (trajFile); 	/* This is where the ftp actualy takes place */
	   
	 /* Verify trajectory, call xps driver function from xps_c8_driver.h */
	    
         status = MultipleAxesPVTVerification(pVar->socket,pVar->groupName,pVar->trajFileName);
	 
	if (status == 0)
	    strcpy(pVar->buildMessage, " ");	    
	if (status == -69)
	    strcpy(pVar->buildMessage, "Accel Too High");
	if (status == -68)
	    strcpy(pVar->buildMessage, "Vel Too High");
	if (status == -70)
	    strcpy(pVar->buildMessage, "Final Vel Non Zero");	     
	 
	if (((status > -68) || (status < -70)) && status != 0){
	     /* Don't read Verify results*/
            printf(" Error performing MultipleAxesPVTVerification%i\n Return\n",status); 
	}
	
	if (pVar->minPeriod < MINPERIOD) {
	    strcpy(pVar->buildMessage, "Element Period too short");
	    status = -99;
	}
		
        /* Read dynamic parameters*/
	 
	pVar->buildStatus = STATUS_FAILURE;
	 
	if (status == 0){
            pVar->buildStatus = STATUS_SUCCESS;
	}    
	/* You cann't read the max vel and accel if the verification failed */
	for (j=0; j<pVar->numAxes; j++) {
            if (pVar->buildStatus == STATUS_SUCCESS) {
	        status = MultipleAxesPVTVerificationResultGet(pVar->socket,pVar->axisName[j],
		         outputFilename,&minp, &maxp, &pVar->motorMVA[j], &pVar->motorMAA[j]);
		if (status != 0){
        	   printf(" Error performing MultipleAxesPVTVerificationResultGet %i\n",status); 
	        }
		  
		} else {
		pVar->motorMVA[j] = 0;
		pVar->motorMAA[j] = 0;     
	        }   
	 } 
	       
	return (0); 
}

/* Function returns the current trajectory element*/ 
static int currentElement(SS_ID ssId, struct UserVar *pVar)
{
	int status;
	int number;
	char fileName[100];
	strcpy (fileName,pVar->trajFileName); 
	
	status = MultipleAxesPVTParametersGet(pVar->pollSocket,
		pVar->groupName,fileName,&number);

	return (number);
}

static int getGroupStatus(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int groupStatus;

    /* Read the current status of the group */

    status = GroupStatusGet(pVar->pollSocket,pVar->groupName,&groupStatus);
    if (status != 0)
        printf(" Error performing GroupStatusGet %i\n",status);
    return(groupStatus);
}

/* Function to load the GatheringExternal.dat file which was written
 * by the XPS after the trajectory was performed and read back the
 * actual motor positions and calculate the position errors */ 
static int readBackError(SS_ID ssId, struct UserVar *pVar)
{
    char vxWorksString[100];
    char buffer[100];
    FILE *trajFile;
    FILE *gatheringFile;
    int i,j;
    double trajStep;
    double trajTime;
    double posTheory[8];
    
    trajStep = 0.0;
    for (j=0; j<pVar->numAxes; ++j)
    	posTheory[j] = 0.0;
    
    strcpy (vxWorksString,"XPS1:/Admin/Public/Trajectories/");
    strcat (vxWorksString,pVar->trajFileName);
    
    /* log-on to the xps and open a trajectory file to read*/	    
    hostAdd ("XPS1",pVar->ipAddress);
    netDevCreate ("XPS1:", "XPS1", 1);
    remCurIdSet("Administrator", "Administrator");
    trajFile =  fopen (vxWorksString, "r");
    gatheringFile = fopen ("XPS1:/Admin/Public/GatheringExternal.dat", "r");
    
    
    /* Read 1st 2 lines which only contain the axis names*/
    for(i=0; i<2; ++i){
        fgets (buffer, 1000, gatheringFile);
	    /*printf("Line %i of GatheringEx = %s\n",i,buffer);*/
	}
    
    /* loop for 1 + nelements due to the extra starting point in the gathered data */
    for (i=0; i<(pVar->nelements+1); ++i){
    	for (j=0; j<pVar->numAxes; ++j){
	      
	    fscanf(gatheringFile,"%lf",&pVar->motorReadbacks[j][i]);
		
	    if(i > 0){  /* Don't read the traj file on the first loop because the gathered file
	    		   has 1 extra line */   
	        if(j == (pVar->numAxes-1)) {
		    if(fscanf(trajFile," %lf,%*f",&trajStep) != 1) printf("trajerror\n");}
		 
	        if (j == 0){   
	            if(fscanf(trajFile,"%lf, %lf,%*f,",&trajTime,&trajStep) != 2) printf("trajerror\n");}
		 
	        if (j > 0 && j < (pVar->numAxes-1)){
		    if(fscanf(trajFile," %lf,%*f,",&trajStep) != 1) printf("trajerror\n");}
	    } 
	    if(i == 0) {
		 /* Start the pos theory at the actual start position i.e. zero error*/
		 posTheory[j] = pVar->motorReadbacks[j][i];
		 /*printf("posTheory = %f ",posTheory[j]);*/
	    }
	    else {
		posTheory[j]+= trajStep;
	        
	    }
	    /* Write over the actual trajectory data which was sent to the XPS to make 
	       the medm graph more readable */
	    pVar->motorTrajectory[j][i] = posTheory[j];
	         
	    pVar->motorError[j][i] = posTheory[j] - pVar->motorReadbacks[j][i];
	    /*printf("i=%i J=%i ReadBack=%f	motorError=%f trajStep=%f\n",
		 	i,j,pVar->motorReadbacks[j][i],pVar->motorError[j][i],trajStep);*/
	    }
	  }
	
	
	
    fclose (trajFile); 	
    fclose (gatheringFile); 

    return (0);
}
/* Function aborts the trajectory/motion */ 
static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar)
{
	int status;
	
	status = GroupMoveAbort(pVar->abortSocket,pVar->groupName);
	if (status != 0)
           printf(" Error performing GroupMoveAbort %i\n",status);
	   
	return (0);
}


}%















