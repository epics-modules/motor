program MM4005_trajectoryScan("P=13IDC:,R=traj1,M1=M1,M2=M2,M3=M3,M4=M4,M5=M5,M6=M6,M7=M7,M8=M8,PORT=serial1")

/*  This sequencer program works with trajectoryScan.db.  It implements
 *  coordinated trajectory motion with the Newport MM4005 motor controller.
 *  It can be used with the Newport General Purpose Diffractometer or with any
 *  other set of motors connected to that controller.
 *
 *  Mark Rivers
 *  August 12, 2000
 *
 * Modifications:
 *  August 21, 2000 MLR Added +r option, so that the flag is just for this
 *                      module, rather than requiring it to be in Makefile.Vx
 *  Dec. 5, 2001 MLR    Fixed bug in computing following error
 *  Dec. 7, 2001 MLR    Fixed bug in converting from EPICS to MM4000 units when
 *                      building trajectory.
 *  Oct. 28, 2002 MLR   Fixed bug in reporting updated trajectory positions after padding
 *                      Was not a problem in executing trajectory, only in values reported
 *                      to EPICS
 *  Dec. 18, 2003 MLR   Changed flyback (now a misnomer) so it does EPICS move to current motor positions,
 *                      not start positions.  Start position move did not work because motors already thought
 *                      they were there (if no encoder error) and motor record did nothing.
 *  Feb. 12, 2004 MLR   Added delay after starting trajectory execution.  This is an attempt to
 *                      eliminate problem where MM4000 seems to sometimes ignore TP command right
 *                      after trajectory execution starts.  Also, first MS sometimes comes back saying
 *                      motors are not moving yet, so software thinks trajectory is done.
 *                      Converted to OSI
 *                      Converted to new version of seq which changes call to seq_pvPut
 *                      Changed flyback (now a misnomer) so it does EPICS move to current motor positions,
 *                      not start positions.  Start position move did not work because motors already thought
 *                      they were there (if no encoder error) and motor record did nothing.
 *  Oct.  1, 2004 MLR   Changed from using asynRecord (previously generic serial record) to using asynOctetSyncIO calls.
 *                      This fixed bugs in I/O due to timing, and is simpler and cleaner.
 *  Feb.  6, 2005 MLR   Changes for last version of asyn; terminators done differently
 *  Mar. 28, 2005 MLR   Increased number of pulses; added readback during trajectory download to prevent buffer overflow
 *  Apr. 29, 2005 MLR   Changed timeout to 30 seconds - necessary?
 *  Apr. 12, 2006 MLR   Changed from strtok_r to epicsStrtok_r
*/

%% #include <string.h>
%% #include <stdio.h>
%% #include <epicsString.h>
%% #include <asynOctetSyncIO.h>

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

/* Maximum # of trajectory elements.  The MM4005 allows 2000, and this is also
 * the channel access limit with a double data type.  However this uses 
 * a lot of memory, the variable motorTrajectory uses MAX_AXES*MAX_ELEMENTS*8 
 * bytes in this SNL program (up to 128KB). Similar memory will be required 
 * for the records in the database. Restrict to 1000 for now.
 */
#define MAX_ELEMENTS 2000

/* Maximum # of output pulses.  The MM4005 allows 2000, and this is also
 * the channel access limit with a double data type.  However this uses 
 * a lot of memory, the variables motorActual and motorError each use
 * MAX_AXES*MAX_PULSES*8 bytes (up to 256KB total). Similar memory will be 
 * required for the records in the database. Restrict to 1000 for now.
 */
#define MAX_PULSES 2000

/* Maximum size of string to/from MM4005, typically for TQ command. */
#define MAX_MM4000_STRING 256

/* Maximum size of string in EPICS string PVs.  This is defined in 
 * epicsTypes.h, but in order to include that file it must be escaped, and then
 * SNL compiler gives a warning. */
#define MAX_STRING_SIZE 40

/* Time for each "padding" trajectory element added to trajectory because it
 * is not a multiple of 4 elements */
#define PAD_TIME 0.1

/* Polling interval for waiting for motors to reach their targets */
#define POLL_INTERVAL 0.1

/* Define PVs */
int    numAxes;     assign numAxes      to "{P}{R}NumAxes.VAL"; 
                                                        monitor numAxes;
int    nelements;   assign nelements    to "{P}{R}Nelements.VAL"; 
                                                        monitor nelements;
int    npulses;     assign npulses      to "{P}{R}Npulses.VAL"; 
                                                        monitor npulses;
int    startPulses; assign startPulses  to "{P}{R}StartPulses.VAL"; 
                                                        monitor startPulses;
int    endPulses;   assign endPulses    to "{P}{R}EndPulses.VAL"; 
                                                        monitor endPulses;
int    nactual;     assign nactual      to "{P}{R}Nactual.VAL"; 

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
evflag motorMDVSMon;    sync motorMDVS  motorMDVSMon;

char stringOut[MAX_MM4000_STRING];
char stringIn[MAX_MM4000_STRING];
char *asynPort;
char *pasynUser;   /* This is really asynUser* */
int status;
int i;
int j;
int k;
double delay;
int anyMoving;
int ncomplete;
int nextra;
int npoints;
int dir;
double dtime;
double dpos;
double posActual;
double posTheory;
double expectedTime;
double initialPos[MAX_AXES];
char *p;
char *tok_save;

/* All PVs which will be accessed in local C functions need to have their index
 * extracted with pvIndex() */
int motorCurrentIndex[MAX_AXES];
int epicsMotorDoneIndex[MAX_AXES];

/* Note, this should be time_t, but SNL doesn't understand that.  This is
 * the defininition in vxWorks. */
unsigned long startTime;  

/* Define escaped C functions at end of file */
%% static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command);
%% static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command);
%% static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos);
%% static int getMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitMotors(SS_ID ssId, struct UserVar *pVar);
%% static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);

ss trajectoryScan {

    /* Initialize things when first starting */
    state init {
        when() {
            /* Force numAxes to be <= MAX_AXES */
            if (numAxes > MAX_AXES) numAxes = MAX_AXES;
            asynPort = macValueGet("PORT");
            %%pVar->status = pasynOctetSyncIO->connect(pVar->asynPort, 0,
            %%                                     (asynUser **)&pVar->pasynUser,
            %%                                     NULL);
            if (status != 0) {
               printf("trajectoryScan error in pasynOctetSyncIO->connect\n");
               printf("   status=%d, port=%s\n", status, asynPort);
            }
            for (j=0; j<numAxes; j++) {
                motorCurrentIndex[j]   =  pvIndex(motorCurrent[j]);
                epicsMotorDoneIndex[j] =  pvIndex(epicsMotorDone[j]);
            }
            /* Read the maximum allowable speed error between blocks */
            for (j=0; j<numAxes; j++) {
                sprintf(stringOut, "%dGC?", j+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                /* Parse the return string which is of the form 1GCxxx */
                motorMDVS[j] = atof(stringIn+3);
                pvPut(motorMDVS[j]);
            }
            /* Clear all event flags */
            efClear(buildMon);
            efClear(executeMon);
            efClear(abortMon);
            efClear(readbackMon);
            efClear(nelementsMon);
            efClear(motorMDVSMon);
        } state monitor_inputs
    }

    /* Monitor inputs which control what to do (Build, Execute, Read) */
    state monitor_inputs {
        when(efTestAndClear(buildMon) && (build==1)) {
        } state build

        when(efTestAndClear(executeMon) && (execute==1)) {
        } state execute

        when(efTestAndClear(readbackMon) && (readback==1)) {
        } state readback

        when(efTestAndClear(nelementsMon) && (nelements>=1)) {
            /* If nelements changes, then change endPulses to this value,
             * since this is what the user normally wants.  endPulses can be
             * changed again after changing nelements if this is desired. */
            endPulses = nelements;
            pvPut(endPulses);
        } state monitor_inputs

        when(efTestAndClear(motorMDVSMon)) {
            /* One of the motorMDVS values has changed.  The event flag is on
             * the array, so we can't tell which one.  No harm in writing all
             * the values to the MM4005. */
            for (j=0; j<numAxes; j++) {
                sprintf(stringOut, "%dGC%f", j+1, motorMDVS[j]);
                %%writeOnly(ssId, pVar, pVar->stringOut);
            }
        } state monitor_inputs
    }


    /* Build and verify trajectory */
    state build {
        when() {
            /* Set busy flag while building */
            buildState = BUILD_STATE_BUSY;
            pvPut(buildState);
            buildStatus=STATUS_UNDEFINED;
            pvPut(buildStatus);
            /* Initialize new trajectory */
            strcpy(stringOut, "NC");
            %%writeOnly(ssId, pVar, pVar->stringOut);
            /* Define which motors are to be moved */
            for (i=0; i<numAxes; i++) {
                sprintf(stringOut, "%dDC%d", i+1, moveAxis[i]);
                %%writeOnly(ssId, pVar, pVar->stringOut);
            }
            /* Set acceleration time */
            sprintf(stringOut, "UC%f", accel);
            %%writeOnly(ssId, pVar, pVar->stringOut);
            /* If time mode is TIME_MODE_TOTAL then construct timeTrajectory 
             * and post it */
            if (timeMode == TIME_MODE_TOTAL) {
                dtime = time/nelements;
                for (i=0; i<nelements; i++) timeTrajectory[i] = dtime;
                pvPut(timeTrajectory);
            }

            /* Make sure number of trajectory elements is a multiple of 4.
             * If not, pad with up to 3 entries of PAD_TIME duration each.
             * Continue the trajectory at the same velocity. 
             * Change nelements and post new value */
            if (moveMode == MOVE_MODE_RELATIVE) {
                npoints=nelements;
            } else {
                npoints=nelements-1;
            }
            nextra = (npoints % 4);
            if (nextra != 0) {
                nextra = 4-nextra;
                /* Compute the increment to move the motors during these
                 * padding elements, keeping velocity constant */
                for (i=0; i<nextra; i++) {
                    timeTrajectory[npoints+i] = PAD_TIME;
                    for (j=0; j<numAxes; j++) {
                        if (!moveAxis[j]) continue;
                        if (moveMode == MOVE_MODE_RELATIVE) {
                            motorTrajectory[j][nelements+i] = 
                                        motorTrajectory[j][nelements-1] *
                                        PAD_TIME / timeTrajectory[nelements-1];
                        } else {
                            dpos = (motorTrajectory[j][nelements-1] -
                                            motorTrajectory[j][nelements-2]) *
                                        PAD_TIME / timeTrajectory[nelements-2];
                            motorTrajectory[j][nelements+i] = 
                                        motorTrajectory[j][nelements-1] +
                                                                dpos*(i+1);
                        }
                    }
                }
                nelements += nextra;
                npoints += nextra;
                pvPut(nelements);
                pvPut(timeTrajectory);
                /* Post the new trajectory position arrays */
                for (j=0; j<numAxes; j++) {
                   pvPut(motorTrajectory[j]);
                }
            }
            /* Compute expected time for trajectory */
            expectedTime=0;
            for (i=0; i<npoints; i++) 
                                expectedTime += timeTrajectory[i];
            /* Define each element in trajectory */
            for (i=0; i<npoints; i++) {
                sprintf(buildMessage, "Building element %d/%d", i+1, nelements);
                pvPut(buildMessage);
                sprintf(stringOut, "%dDT%f", i+1, timeTrajectory[i]);
                %%writeOnly(ssId, pVar, pVar->stringOut);
                for (j=0; j<numAxes; j++) {
                    if (!moveAxis[j]) continue;
                    if (moveMode == MOVE_MODE_RELATIVE) {
                        dpos = motorTrajectory[j][i];
                    } else {
                        dpos = motorTrajectory[j][i+1] - motorTrajectory[j][i];
                    }
                    /* Convert from user units to MM4000 units */
                    if (epicsMotorDir[j] == 0) dir=1; else dir=-1;
                    dpos = dpos*dir;
                    sprintf(stringOut, "%dDX%f", j+1, dpos);
                    %%writeOnly(ssId, pVar, pVar->stringOut);
                }
                /* The following command is intended to prevent buffer overflow in
                 * the MM4005 by reading introducing a delay (reading status) when downloading
                 * many-element trajectories */
                if (((i+1) % 20) == 0) %%writeRead(ssId, pVar, "TB");
            }
            /* Define pulse output for trajectory */
            if (npulses > 0) {
                /* Check validity, modify values if necessary */
                if (startPulses < 1) startPulses = 1;
                if (startPulses > npoints) startPulses = npoints;
                pvPut(startPulses);
                if (endPulses < startPulses) endPulses = startPulses;
                if (endPulses > npoints) endPulses = npoints;
                pvPut(endPulses);
                /* There seems to be a bug in the MM4005, it puts out one fewer
                 * pulse than requested.  Add one */
                sprintf(stringOut, "MB%d,ME%d,MN%d", 
                                startPulses, endPulses, npulses+1);
                %%writeOnly(ssId, pVar, pVar->stringOut);
            }
            /* Verify trajectory */
            strcpy(buildMessage, "Verifying trajectory");
            pvPut(buildMessage);
            strcpy(stringOut, "VC");
            %%writeOnly(ssId, pVar, pVar->stringOut);
            /* Read error code back from MM4000 */
            %%writeRead(ssId, pVar, "TB");
            /* Set status and message string */
            if (stringIn[2] == '@') {
                buildStatus = STATUS_SUCCESS;
                strcpy(buildMessage, " ");
            } else {
                buildStatus = STATUS_FAILURE;
                strncpy(buildMessage, stringIn, MAX_STRING_SIZE-1);
            }
            /* Read dynamic parameters, post them */
            for (j=0; j<numAxes; j++) {
                p = stringIn;
                /* This query can only be done for axes which are active in the
                 * trajectory */
                if (!moveAxis[j]) continue;
                /* We could query all parameters with one nRc, but the parsing
                 * is a pain, much simpler to query one at a time */
                /* Maximum speed change element and value */
                sprintf(stringOut, "%dRC1", j+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                motorMDVE[j] = atoi(p+3);
                pvPut(motorMDVE[j]);
                sprintf(stringOut, "%dRC2", j+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                motorMDVA[j] = atof(p+3);
                pvPut(motorMDVA[j]);
                /* Maximum velocity element and value */
                sprintf(stringOut, "%dRC3", j+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                motorMVE[j] = atoi(p+3);
                pvPut(motorMVE[j]);
                sprintf(stringOut, "%dRC4", j+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                motorMVA[j] = atof(p+3);
                pvPut(motorMVA[j]);
                /* Maximum acceleration element and value */
                sprintf(stringOut, "%dRC5", j+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                motorMAE[j] = atoi(p+3);
                pvPut(motorMAE[j]);
                sprintf(stringOut, "%dRC6", j+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                motorMAA[j] = atof(p+3);
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
            /* Set busy flag */
            execState = EXECUTE_STATE_MOVE_START;
            pvPut(execState);
            /* Set status to INVALID */
            execStatus = STATUS_UNDEFINED;
            pvPut(execStatus);
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
            /* Start the detector */
            detOn = 1;
            pvPut(detOn);
            /* Send the execute command, along with simulation mode and time
             * scaling factor */
            sprintf(stringOut, "LS,%dEC%f",simMode,timeScale);
            %%writeOnly(ssId, pVar, pVar->stringOut);
            /* Get start time of execute */
            startTime = time(0);
            execState = EXECUTE_STATE_EXECUTING;
            pvPut(execState);
            /* This is an attempt to fix the problem of TP sometimes not responding */
            epicsThreadSleep(0.1);
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
        } state monitor_inputs

        when(execState==EXECUTE_STATE_EXECUTING) {
            /* Get the current motor positions, post them */
            %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
            for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
            /* Send XC1 command, read last trajectory element done */
            %%writeRead(ssId, pVar, "XC1");
            /* Parse response, which is of the form XCnnnn */
            ncomplete = atoi(&stringIn[2]);
            sprintf(execMessage, "Executing element %d/%d", 
                                        ncomplete, nelements);
            pvPut(execMessage);
            %%pVar->anyMoving = getMotorMoving(ssId, pVar);
            if (!anyMoving) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_SUCCESS;
                strcpy(execMessage, " ");
            }
            /* See if the elapsed time is more than twice expected, time out */
            if (difftime(time(0), startTime) > expectedTime*timeScale*2.) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_TIMEOUT;
                strcpy(execMessage, "Timeout");
            }
            /* Send TB command, read any error messages */
            %%writeRead(ssId, pVar, "TB");
            /* Parse the return string, of form "TBx message". If 'x' is '@'
               then there is no error, else stop with error code */
            if (stringIn[2] != '@') {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_FAILURE;
                strncpy(execMessage, stringIn, MAX_STRING_SIZE-1);
            }
        } state wait_execute

        when(execState==EXECUTE_STATE_FLYBACK) {
            /* Stop the detector */
            detOff = 1;
            pvPut(detOff);
            pvPut(execState);
            pvPut(execStatus);
            pvPut(execMessage);
            /* Get the current motor positions, post them */
            %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
            for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
            for (j=0; j<numAxes; j++) {
                if (!moveAxis[j]) continue;
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
            /* Read the actual number of trace points */
            %%writeRead(ssId, pVar, "NQ");
            /* Parse response, which is of the form NQnnnn */
            nactual = atoi(&stringIn[2]);
            pvPut(nactual);
            /* Read actual positions */
            for (i=0; i<nactual; i++) {
                sprintf(readMessage, "Reading point %d/%d", i+1, nactual);
                pvPut(readMessage);
                sprintf(stringOut, "%dTQ", i+1);
                %%writeRead(ssId, pVar, pVar->stringOut);
                /* Parse the return string which is of the form 
                 * 15TQ,1TH2.7,1TP2.65,2TH3.1,2TP3.1 ... */
                tok_save = 0;
                /* Skip the first token, which is nnTQ */
                p = epicsStrtok_r(stringIn, ",", &tok_save);
                for (j=0; (j<numAxes && p!=0); j++) {
                    p = epicsStrtok_r(0, ",", &tok_save);
                    posTheory = atof(p+3);
                    p = epicsStrtok_r(0, ",", &tok_save);
                    if (epicsMotorDir[j] == 0) dir=1; else dir=-1;
                    posActual = atof(p+3);
                    motorError[j][i] = posActual-posTheory;
                    /* Convert from MM4000 units to user units */
                    posActual = posActual*dir + epicsMotorOff[j];
                    motorReadbacks[j][i] = posActual;
                }
            }
            /* Post the readback and error arrays */
            for (j=0; j<numAxes; j++) {
                pvPut(motorReadbacks[j]);
                pvPut(motorError[j]);
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
 * state set. If an abort is received it sends the "AB" command to the MM4005, 
 * sets the execStatus to STATUS_ABORT and writes a message to execMessage */
ss trajectoryAbort {
    state monitorAbort {
        when (efTestAndClear(abortMon) && (abort==1)) {
            /* Send AB command */
            strcpy(stringOut,"AB");
            %%writeOnly(ssId, pVar, pVar->stringOut);
            execStatus = STATUS_ABORT;
            pvPut(execStatus);
            strcpy(execMessage, "Motion aborted");
            pvPut(execMessage);
            /* Clear abort command, post.  This is a "busy" record, don't
             * want to do this until abort command has been sent. */
            abort=0;
            pvPut(abort);
        } state monitorAbort
    }
}

%{

/* writeOnly sends a command to the MM4005 */
static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command)
{
    asynStatus status;
    int nwrite;
    char buffer[MAX_MM4000_STRING];

    /* Copy command so we can add terminator */
    strncpy(buffer, command, MAX_MM4000_STRING-3);
    strcat(buffer, "\r");
    status = pasynOctetSyncIO->write((asynUser *)pVar->pasynUser, buffer,
                               strlen(buffer), 1.0, &nwrite);
    return(status);
}

/* writeRead sends a command to the MM4005 and reads the response
 * It also writes the response string to another PV so it can be displayed. */
static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command)
{
    asynStatus status;
    int nwrite, nread;
    int eomReason;
    char buffer[MAX_MM4000_STRING];

    /* Copy command so we can add terminator */
    strncpy(buffer, command, MAX_MM4000_STRING-3);
    strcat(buffer, "\r");
    /* Use 30 second timeout, some commands take a long time to reply */
    status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, buffer,
                               strlen(buffer), pVar->stringIn, MAX_MM4000_STRING, 
                               30.0, &nwrite, &nread, &eomReason);
    return(status);
}


/* getMotorPositions returns the positions of each motor */
static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos)
{
    char *p, *tok_save;
    int j;
    int dir;

    /* Read the current positions of all the axes */
    writeRead(ssId, pVar, "TP");
    /* Parse the return string which is of the form 
     * 1TP2.65,2TP3.1 ... */
    tok_save = 0;
    p = epicsStrtok_r(pVar->stringIn, ",", &tok_save);
    for (j=0; (j<pVar->numAxes && p!=0); j++) {
        if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1;
        pos[j] = atof(p+3)*dir + pVar->epicsMotorOff[j];
        p = epicsStrtok_r(0, ",", &tok_save);
    }
    return(0);
}


/* getMotorMoving returns the moving status of each motor, packed into a single
 * int.  Bit 0 = motor 1, bit 1 = motor 2, etc. 0=not moving, 1=moving.
 * If the entire int is 0 then no motors are moving */
static int getMotorMoving(SS_ID ssId, struct UserVar *pVar)
{
    char *p, *tok_save;
    int j;
    int result=0, mask=0x01;

    /* Read the current status of all the axes */
    writeRead(ssId, pVar, "MS");
    /* Parse the return string which is of the form 
     * 1MSA,2MS@ ... */
    tok_save = 0;
    p = epicsStrtok_r(pVar->stringIn, ",", &tok_save);
    for (j=0; (j<pVar->numAxes && p!=0); j++) {
        /* The low order bit in the status byte is the MOVING bit */
        if (*(p+3) & 0x01) result |= mask;
        mask = mask << 1;
        p = epicsStrtok_r(0, ",", &tok_save);
    }
    return(result);
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

}%
