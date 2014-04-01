program EnsembleTrajectoryScan("P=13IDC:,R=traj1,M1=M1,M2=M2,M3=M3,M4=M4,M5=M5,M6=M6,M7=M7,M8=M8,PORT=serial1")

/* sample program invocation:
 * dbLoadRecords("$(MOTOR)/motorApp/Db/trajectoryScan.db","P=xxx:,R=traj1:,NAXES=2,NELM=100,NPULSE=100")
 * ...
 * iocInit()
 * ...
 * seq &EnsembleTrajectoryScan, "P=xxx:,R=traj1:,M1=m1,M2=m2,M3=m3,M4=m4,M5=m5,M6=m6,M7=m7,M8=m8,PORT=none"
 *
 * NAXES must be less than or equal to the value of MAX_AXES specified in EnsembleTrajectoryScan.h
 * NELM must be less than or equal to the value of MAX_ELEMENTS specified below
 * NPULSE must be less than or equal to the value of MAX_AXES specified below
 */

/*  This sequencer program works with trajectoryScan.db.  It implements
 *  coordinated trajectory motion with an Aerotech Ensemble motor controller.
 *
 *  Tim Mooney -- based on MM4000_trajectoryScan.st by Mark Rivers.
 *
 *  NOTE that this software is not ready for multiple-motor trajectories, even though
 *  some of the code is.
 */

%% #include <stdlib.h>
%% #include <string.h>
%% #include <ctype.h>
%% #include <stdio.h>
%% #include <math.h>
%% #include <epicsString.h>
%% #include <epicsStdio.h>
%% #include <asynOctetSyncIO.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) > (b) ? (b) : (a))
#define NINT(f)  (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/* This program must be compiled with the recursive option */
option +r;

/* Maximum # of trajectory elements.  The variable motorTrajectory
 * uses MAX_AXES*MAX_ELEMENTS*8 bytes in this SNL program (up to 128KB).
 * Similar memory will be required for the records in the database.
 * (Note that currently MAX_AXES is fixed at 8, in trajectoryScan.h.)
 */
#define MAX_ELEMENTS 1000

/* Maximum # of output pulses.  For now, we emit a pulse at the beginning of
 * every trajectory element.
 */
#define MAX_PULSES 10000

/* Note that MAX_ELEMENTS, and MAX_PULSES must be defined before including
 * trajectoryScan.h, which defines MAX_AXES. */
#include "EnsembleTrajectoryScan.h"

/* Maximum size of string messages we'll be sending to the Ensemble controller */
#define MAX_MESSAGE_STRING 100

/* Buffer sizes */
#define NAME_LEN 100

/* Maximum size of string in EPICS string PVs.  This is defined in 
 * epicsTypes.h, but in order to include that file it must be escaped, and then
 * SNL compiler gives a warning. */
#define MAX_STRING_SIZE 40

/* Polling interval in seconds for waiting for motors to reach their targets */
#define POLL_INTERVAL (1/5.)
#define READ_INTERVAL (1/10.)

char stringOut[MAX_MESSAGE_STRING];
char stringIn[MAX_MESSAGE_STRING];
char stringLast[MAX_MESSAGE_STRING];
char abortCommand[MAX_MESSAGE_STRING];
char *asynPort;
char *pasynUser;   /* This is really asynUser* */
int status;
int i;
int j;
int k;
int n;
int anyMoving;
int npoints;
double dtime;
double dpos;
double posActual;
double expectedTime;
double initialPos[MAX_AXES];
char macroBuf[NAME_LEN];
char motorName[NAME_LEN];
int initStatus;
int limitViolation;
int loadingTrajectory;

/* All PVs which will be accessed in local C functions need to have their index
 * extracted with pvIndex() */
int motorCurrentIndex[MAX_AXES];
int epicsMotorDoneIndex[MAX_AXES];
int elapsedTimeIndex;

#define ABORT_STATE_NONE 0
#define ABORT_STATE_NOTED 1
#define ABORT_STATE_SENT 2
#define ABORT_STATE_DONE 3
int abortState;

%%epicsTimeStamp startTime;
%%epicsTimeStamp currTime;
%%epicsTimeStamp lastPollTime;

%%epicsTimeStamp pvtTime;
%%epicsTimeStamp lastPvtTime;

/* Define escaped C functions at end of file */
%% static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command);
%% static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command, char *reply);
%% static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos);
%% static int getMotorMoving(SS_ID ssId, struct UserVar *pVar, int movingMask);
%% static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);
%% static int buildTrajectory(SS_ID ssId, struct UserVar *pVar, double *timeTrajectory,
%% double *motorTrajectory, int epicsMotorDir, int moveMode, int npoints, int npulses, double motorOffset,
%% double motorResolution, double *velocity);
%% static int loadTrajectory(SS_ID ssId, struct UserVar *pVar);
%% static int userToRaw(double user, double off, int dir, double res);
%% static double rawToUser(int raw, double off, int dir, double res);
%% static int sendReceiveCommand(SS_ID ssId, struct UserVar *pVar, char *cmd, char *callerReply, int checkProgram);
%% static double trajEval(SS_ID ssId, struct UserVar *pVar, double *motorReadbacks, int scopeDataIntervalMS, int nScopeDataPoints,
%% 	double *motorTrajectory, double *realTimeTrajectory, int npoints, double *motorError);
%% int writeDoubleAndCheck(SS_ID ssId, struct UserVar *pVar, int n, double d);
%% int writeIntAndCheck(SS_ID ssId, struct UserVar *pVar, int n, int ival);
double position[MAX_AXES][MAX_ELEMENTS];
double velocity[MAX_AXES][MAX_ELEMENTS];
double motorStart[MAX_AXES];
double motorCurr[MAX_AXES];
double motorPosOffset[MAX_AXES];

/* variables for constructing trajectory commands */
int movingMask;
int numGlobalDoubles;

/* temporary variables to hold mav speed and acceleration for a motor */
double vmax;
double amax;
double d, dlast;

/* variables to control acquisition of scope data */
#define GLOBALINDEXSTART 50
#define USE_SCOPE 1
#if USE_SCOPE
int nScopeDataPoints;
int scopeDataIntervalMS;
#endif

#define USE_DATAACQ 0
#if USE_DATAACQ
int nAcqDataPoints;
#endif

/* Ensemble parameter numbers */
#define VelocityCommandThreshold_PARM_NUM 35
#define ReverseMotionDirection_PARM_NUM 1
#define HomePositionSet_PARM_NUM 166

%%int encoder_runs_backwards=0;

ss EnsembleTrajectoryScan {

	/* Initialize things when first starting */
	state init {
		when() {
			initStatus = STATUS_UNDEFINED;
			/* Force numAxes to be <= MAX_AXES */
			if (numAxes > MAX_AXES) numAxes = MAX_AXES;
			for (i=0; i<numAxes; i++) {
				sprintf(macroBuf, "M%d", i+1);

				sprintf(motorName, "%s%s.VAL", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorPos[i], motorName);
				pvMonitor(epicsMotorPos[i]);

				sprintf(motorName, "%s%s.DIR", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorDir[i], motorName);
				pvMonitor(epicsMotorDir[i]);

				sprintf(motorName, "%s%s.OFF", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorOff[i], motorName);
				pvMonitor(epicsMotorOff[i]);

				sprintf(motorName, "%s%s.DMOV", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorDone[i], motorName);
				pvMonitor(epicsMotorDone[i]);

				sprintf(motorName, "%s%s.MRES", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorMres[i], motorName);
				pvMonitor(epicsMotorMres[i]);

				sprintf(motorName, "%s%s.CARD", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorCard[i], motorName);
				pvMonitor(epicsMotorCard[i]);

				sprintf(motorName, "%s%s.HLM", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorHLM[i], motorName);
				pvMonitor(epicsMotorHLM[i]);

				sprintf(motorName, "%s%s.LLM", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorLLM[i], motorName);
				pvMonitor(epicsMotorLLM[i]);

				sprintf(motorName, "%s%s.VELO", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorVELO[i], motorName);
				pvMonitor(epicsMotorVELO[i]);

				sprintf(motorName, "%s%s.VMAX", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorVMAX[i], motorName);
				pvMonitor(epicsMotorVMAX[i]);

				sprintf(motorName, "%s%s.VBAS", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorVMIN[i], motorName);
				pvMonitor(epicsMotorVMIN[i]);

				sprintf(motorName, "%s%s.ACCL", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorACCL[i], motorName);
				pvMonitor(epicsMotorACCL[i]);

			}
			asynPort = macValueGet("PORT");
			%%pVar->status = pasynOctetSyncIO->connect(pVar->asynPort, 0, (asynUser **)&pVar->pasynUser, NULL);
			if (status != 0) {
			   printf("trajectoryScan error in pasynOctetSyncIO->connect\n");
			   printf("   status=%d, port=%s\n", status, asynPort);
			}
			for (j=0; j<numAxes; j++) {
				motorCurrentIndex[j]   =  pvIndex(motorCurrent[j]);
				epicsMotorDoneIndex[j] =  pvIndex(epicsMotorDone[j]);
				motorPosOffset[j] = 0.;
			}
			elapsedTimeIndex = pvIndex(elapsedTime);
			abortState = ABORT_STATE_NONE;

			/* Clear all event flags */
			efClear(buildMon);
			efClear(executeMon);
			efClear(abortMon);
			efClear(readbackMon);
			efClear(nelementsMon);
			efClear(motorMDVSMon); /* we don't use this */
			if (initStatus == STATUS_UNDEFINED) initStatus = STATUS_SUCCESS;
			loadingTrajectory = 0;

			/* PARAMETERID_GlobalDoubles is 125 */
			%%pVar->status = writeRead(ssId, pVar, "getparm(125)", pVar->stringIn);
			if (stringIn[0] == '%') {
				numGlobalDoubles = atol(&stringIn[1]);
			} else {
				/* try again */
				%%pVar->status = writeRead(ssId, pVar, "getparm(@0,125)", pVar->stringIn);
				if (stringIn[0] == '%') {
					numGlobalDoubles = atol(&stringIn[1]);
				} else {
					printf("Can't read number of global doubles.  Motor controller problem?\n");
					numGlobalDoubles = 0;
				}
			}
		} state monitor_inputs
	}

	/* Monitor inputs which control what to do (Build, Execute, Read) */
	state monitor_inputs {
		when(efTestAndClear(buildMon) && (build==1) && (initStatus == STATUS_SUCCESS)) {
		} state build

		when(efTestAndClear(executeMon) && (execute==1) && ((buildStatus == STATUS_SUCCESS) || (buildStatus == STATUS_WARNING))) {
		} state execute

		when(efTestAndClear(readbackMon) && (readback==1) /*&& (execStatus == STATUS_SUCCESS)*/) {
			if (debugLevel>1) printf("readback command detected\n");
		} state readback

		when(efTestAndClear(nelementsMon) && (nelements>=1)) {
			/* If nelements changes, then change endPulses to this value,
			 * since this is what the user normally wants.  endPulses can be
			 * changed again after changing nelements if this is desired. */
			if (nelements > (numGlobalDoubles/3 - 3)) {
				/* The Ensemble has an array of doubles we use to send the trajectory.  The number of
				 * elements in that array must be configured using the "Configuration Manager".
				 */ 
				nelements = numGlobalDoubles/3 - 3;
				pvPut(nelements);
			}
			endPulses = nelements;
			pvPut(endPulses);
		} state monitor_inputs
	}

	/* Build trajectory */
	state build {
		when() {
			/* Set busy flag while building */
			buildState = BUILD_STATE_BUSY;
			pvPut(buildState);
			buildStatus=STATUS_UNDEFINED;
			pvPut(buildStatus);
			epicsSnprintf(buildMessage, MSGSIZE, "Building...");
			pvPut(buildMessage);

			buildStatus = STATUS_SUCCESS; /* presume we'll be successful */
			/* Initialize new trajectory */
			/* If time mode is TIME_MODE_TOTAL then construct timeTrajectory and post it */
			if (timeMode == TIME_MODE_TOTAL) {
				dtime = time/(nelements-1);
				for (i=0; i<nelements; i++) timeTrajectory[i] = dtime;
				pvPut(timeTrajectory);
			} else {
				for (i=0, time=0.; i<nelements; i++) {
					time += timeTrajectory[i];
				}
				pvPut(time);
			}

			npoints = nelements;

			/* calculate time from beginning of motion at which motor should reach each trajectory point */			
			realTimeTrajectory[0] = 0.;
			for (i=1; i<npoints; i++) {
				realTimeTrajectory[i] = realTimeTrajectory[i-1] + timeTrajectory[i];
			}
			for (i=0; i<npoints; i++) realTimeTrajectory[i] *= timeScale;

			/* For MEDM plotting */
			for (; i<MAX_ELEMENTS; i++) realTimeTrajectory[i] = realTimeTrajectory[i-1];
			pvPut(realTimeTrajectory);

			/* Calculate velocities and accelerations for trajectories.   Build abort command, in case we need it. */
			n = sprintf(abortCommand, "ABORT");
			for (j=0; j<MAX_AXES; j++) {
				if (moveAxis[j]) {
					%%buildTrajectory(ssId, pVar, pVar->realTimeTrajectory, pVar->motorTrajectory[pVar->j],
					%%	pVar->epicsMotorDir[pVar->j], pVar->moveMode, pVar->npoints, pVar->npulses,
					%%	pVar->epicsMotorOff[pVar->j], pVar->epicsMotorMres[pVar->j], pVar->velocity[pVar->j]);
					n += sprintf(&abortCommand[n], " @%d", j);
				}
			}

			/* Compute expected time for trajectory.  This includes timeScale factor. */
			expectedTime = realTimeTrajectory[npoints-1];

			/* Check trajectories against motor soft limits */
			limitViolation = 0;
			for (j=0; j<numAxes && !limitViolation; j++) {
				if (moveAxis[j]) {
					vmax = epicsMotorVMAX[j];
					if (fabs(vmax) < .001) vmax = epicsMotorVELO[j];
					if (debugLevel>1) printf("vmax=%f\n", vmax);
					amax = vmax/epicsMotorACCL[j];
					for (k=0; k<npoints && !limitViolation; k++) {
						posActual = motorTrajectory[j][k];
						if (moveMode != MOVE_MODE_ABSOLUTE) posActual += epicsMotorPos[j];
						limitViolation |= (posActual > epicsMotorHLM[j]) || (posActual < epicsMotorLLM[j]);
                		if (limitViolation) {
							epicsSnprintf(buildMessage, MSGSIZE, "Limit: m%d at pt. %d (%f)", j+1, k+1, posActual);
						}
						if (velocity[j][k] > vmax) {
							limitViolation |= 1;
							epicsSnprintf(buildMessage, MSGSIZE, "V limit: m%d at pt. %d (%f)", j+1, k+1,
								velocity[j][k]);
						}
						if (k > 1) {
							dtime = realTimeTrajectory[k]-realTimeTrajectory[k-1];
							d = (velocity[j][k] - velocity[j][k-1]) / dtime;
							if (fabs(d) > amax) {
								limitViolation |= 1;
								epicsSnprintf(buildMessage, MSGSIZE, "A limit: m%d at pt. %d (%f)", j+1, k+1, d);
							}
							if ((dtime < .03) && (buildStatus != STATUS_WARNING)) {
								buildStatus = STATUS_WARNING;
								epicsSnprintf(buildMessage, MSGSIZE, "!! deltaT(=%.3f) < .03 at pt. %d", dtime, k+1);
							}
						}
					}
				}
			}
			if (limitViolation) {
				buildStatus = STATUS_FAILURE;
			}

			/* Set status and message string */

			/* Clear busy flag, post status */
			buildState = BUILD_STATE_DONE;
			pvPut(buildState);
			pvPut(buildStatus);
			pvPut(buildMessage);
			/* Clear build command, post.  This is a "busy" record, don't want
			 * to do this until build is complete. */
			build=0;
			pvPut(build);
			if (buildStatus == STATUS_SUCCESS) {
				epicsSnprintf(buildMessage, MSGSIZE, "Done");
				pvPut(buildMessage);
			}
		} state monitor_inputs
	}


	state execute {
		when () {
			execState = EXECUTE_STATE_MOVE_START;
			pvPut(execState);
			abortState = ABORT_STATE_NONE;
			execStatus = STATUS_UNDEFINED;
			pvPut(execStatus);
			/* Erase the readback and error arrays */
			for (j=0; j<numAxes; j++) {
				for (i=0; i<MAX_PULSES; i++) {
					motorReadbacks[j][i] = 0.;
					motorError[j][i] = 0.;
				}
			}
			/* Get the initial (pretrajectory) and start (preaccel) positions of the motors */
			for (j=0; j<numAxes; j++) {
				initialPos[j] = epicsMotorPos[j];
				motorStart[j] = motorTrajectory[j][0] - (velocity[j][0] * epicsMotorACCL[j]) / 2;
				if (moveMode != MOVE_MODE_ABSOLUTE) motorStart[j] += epicsMotorPos[j];
			}
			/* Move to start position */
			for (j=0; j<numAxes; j++) {
				if (moveAxis[j]) {
					epicsMotorPos[j] = motorStart[j];
					pvPut(epicsMotorPos[j]);
					if (debugLevel > 0) printf("\nstate execute: moving motor %d to %f\n", j, epicsMotorPos[j]);
				}
			}
			%%waitEpicsMotors(ssId, pVar);

			/* Check that motors went to start positions */
			%%getMotorPositions(ssId, pVar, pVar->motorCurr);
			for (j=0; j<numAxes; j++) {
				if (moveAxis[j]) {
					if (fabs(motorCurr[j] - motorStart[j]) > .01) {
						printf("state execute: motor %d didn't move to start\n", j);
						if (execStatus != STATUS_ABORT) {
							execStatus = STATUS_ABORT;
							%%pVar->status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn);
							abortState = ABORT_STATE_SENT;
							if (debugLevel) printf("abort: sent command '%s'\n", abortCommand);
						}
					}
				}
			}

			if (execStatus != STATUS_ABORT) {

				%%getMotorPositions(ssId, pVar, pVar->motorStart);
				%%epicsTimeGetCurrent(&lastPollTime);

				/* Get start time of execute */
				elapsedTime = 0.;
				pvPut(elapsedTime);
				%%epicsTimeGetCurrent(&startTime);
				execState = EXECUTE_STATE_EXECUTING;
				pvPut(execState);
				for (j=0, movingMask = 0; j<numAxes; j++) {
					if (moveAxis[j]) movingMask |= (1<<j);
				}
				/* The trajectory will usually begin executing while it is being loaded,
				 * because the Ensemble has only a 14-point FIFO for trajectory commands.
				 */
				 for (i=0; i<10; i++) {
					%%pVar->status = loadTrajectory(ssId, pVar);
					if (status == -2) {
						printf("Trying trajectory again\n");
					} else {
						break;
					}
				}
			}
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
			execute = 0;
			pvPut(execute);
		} state monitor_inputs

		when ((execState==EXECUTE_STATE_EXECUTING) && delay(0.1)) {

			/* Get the current motor positions, post them */
			%%epicsTimeGetCurrent(&currTime);
			%%pVar->dtime = epicsTimeDiffInSeconds(&currTime, &lastPollTime);
			if (dtime > POLL_INTERVAL) {
				%%pVar->elapsedTime = epicsTimeDiffInSeconds(&currTime, &startTime);
				pvPut(elapsedTime);
				%%epicsTimeGetCurrent(&lastPollTime);
				%%getMotorPositions(ssId, pVar, pVar->motorCurrent);
				for (j=0; j<numAxes; j++) {
					if (moveAxis[j]) {
						pvPut(motorCurrent[j]);
					}
				}
				%%pVar->anyMoving = getMotorMoving(ssId, pVar, pVar->movingMask);
				if (anyMoving == 0) {
					execState = EXECUTE_STATE_FLYBACK;
					execStatus = STATUS_SUCCESS;
					strcpy(execMessage, " ");
				}
				/* See if the elapsed time is more than twice expected, time out */
				if (elapsedTime > expectedTime*2.) {
					execState = EXECUTE_STATE_FLYBACK;
					execStatus = STATUS_TIMEOUT;
					strcpy(execMessage, "Timeout");
	
					/* abort motion of selected axes */
					%%pVar->status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn);
					abortState = ABORT_STATE_SENT;
					if (debugLevel) printf("timeout: sent command '%s'\n", abortCommand);
					strcpy(stringOut, "PROGRAM STOP 1");
					%%pVar->status = writeRead(ssId, pVar, pVar->stringOut, pVar->stringIn);

					%%waitEpicsMotors(ssId, pVar); /* wait until all motors are done */
					abortState = ABORT_STATE_DONE;
				}
			}
			/* Check for errors while trajectories are in progress */
		} state wait_execute

		when (execState==EXECUTE_STATE_FLYBACK) {
			if (debugLevel>2) printf("flyback.\n");
			pvPut(elapsedTime);
			pvPut(execState);
			pvPut(execStatus);
			pvPut(execMessage);
			/* Turn PSO off */
			if ((outBitNum >= 0) && (outBitNum <= 15)) {
				strcpy(stringOut, "PSOCONTROL @0 OFF");
				%%pVar->status = writeRead(ssId, pVar, pVar->stringOut, pVar->stringIn);
			}
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
		entry {
			if (debugLevel) printf("readback: entry\n");
		}
		when() {
			/* Set busy flag */
			readState = READ_STATE_BUSY;
			pvPut(readState);
			readStatus=STATUS_UNDEFINED;
			pvPut(readStatus);

#if USE_SCOPE
			%%pVar->status = sendReceiveCommand(ssId, pVar, "SCOPETRIG STOP", pVar->stringIn, 1);
			strcpy(stringLast, "");
			if (debugLevel) printf("state readback: motorPosOffset[0]=%f\n", motorPosOffset[0]);
			for (i=0; i<nScopeDataPoints; i++) {
				if (execStatus == STATUS_ABORT) break;
				sprintf(stringOut, "SCOPEDATA %d %d", sd_PositionFeedback, i);
				%%pVar->status = sendReceiveCommand(ssId, pVar, pVar->stringOut, pVar->stringIn, 0);
				sprintf(stringOut, "DGLOBAL(%d)", darg1Var);
				%%pVar->status = writeRead(ssId, pVar, pVar->stringOut, pVar->stringIn);
				if (stringIn[0] == '%') {
					if (i==0) {
						/* read until we get the same thing twice */
						for (j=0; j<10 ; j++) {
							strcpy(stringLast, stringIn);
							sprintf(stringOut, "SCOPEDATA %d %d", sd_PositionFeedback, i);
							%%pVar->status = sendReceiveCommand(ssId, pVar, pVar->stringOut, pVar->stringIn, 0);
							sprintf(stringOut, "DGLOBAL(%d)", darg1Var);
							%%pVar->status = writeRead(ssId, pVar, pVar->stringOut, pVar->stringIn);
							if (strcmp(stringIn, stringLast) == 0) break;
							if (debugLevel) printf("**%d\n", i);
						}
						if (j==10) printf("abandoned read of point %d\n", i);
					} else {
						/* make sure we didn't just get last point's data again */
						for (j=0; j<10 && (strcmp(stringIn, stringLast) == 0); j++) {
							if (debugLevel) printf("**%d\n", i);
							sprintf(stringOut, "SCOPEDATA %d %d", sd_PositionFeedback, i);
							%%pVar->status = sendReceiveCommand(ssId, pVar, pVar->stringOut, pVar->stringIn, 0);
							sprintf(stringOut, "DGLOBAL(%d)", darg1Var);
							%%pVar->status = writeRead(ssId, pVar, pVar->stringOut, pVar->stringIn);
						}
						strcpy(stringLast, stringIn);
						if (j==10) printf("abandoned read of point %d\n", i);
					}
					k = atoi(&stringIn[1]);
					if (encoder_runs_backwards) k *= -1;
					motorReadbacks[0][i] = k * epicsMotorMres[0] + motorPosOffset[0];
					motorError[0][i] = motorReadbacks[0][i] - motorTrajectory[0][i];
				}
			}
			for (; i<MAX_PULSES; i++) {
				motorReadbacks[0][i] = 0;
				motorError[0][i] = 0;
			}
			%%pVar->d = trajEval(ssId, pVar, pVar->motorReadbacks[0], pVar->scopeDataIntervalMS, pVar->nScopeDataPoints,
			%%pVar->motorTrajectory[0], pVar->realTimeTrajectory, pVar->npoints, pVar->motorError[0]);
#endif

#if USE_DATAACQ
			nAcqDataPoints = 1 + endPulses - startPulses;
			sprintf(stringOut, "DATAACQ_READ @0 %d %d", GLOBALINDEXSTART, nAcqDataPoints);
			%%pVar->status = sendReceiveCommand(ssId, pVar, pVar->stringOut, pVar->stringIn, 1);
			%%pVar->status = sendReceiveCommand(ssId, pVar, "DATACQ_OFF @0", pVar->stringIn, 0);
			for (i=0; i<nAcqDataPoints; i++) {
				sprintf(stringOut, "IGLOBAL(%d)", i+GLOBALINDEXSTART);
				%%pVar->status = writeRead(ssId, pVar, pVar->stringOut, pVar->stringIn);
				if (stringIn[0] == '%') {
					motorReadbacks[0][i] = atoi(&stringIn[1]) * epicsMotorMres[0];
					motorError[0][i] = motorReadbacks[0][i] - motorTrajectory[0][i];
				}
			}
			for (; i<MAX_PULSES; i++) {
				motorReadbacks[0][i] = 0;
				motorError[0][i] = 0;
			}
#endif

            /* Post the readback arrays */
            /*for (j=0; j<numAxes; j++) {*/ 
            for (j=0; j<MAX_AXES; j++) {
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
 * state set. If an abort is received it tells the controller to abort, 
 * sets the execStatus to STATUS_ABORT and writes a message to execMessage */
ss trajectoryAbort {
	state monitorAbort {
		when (efTest(abortMon) && (abort==1) && delay(0.1)) {
			if (debugLevel) printf("trajectoryAbort: loadingTrajectory=%d\n", loadingTrajectory);
			abortState = ABORT_STATE_NOTED;
			if (!loadingTrajectory) {
				efClear(abortMon);
				%%pVar->status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn);
				if (debugLevel) printf("trajectoryAbort: sent command '%s'\n", abortCommand);
				abortState = ABORT_STATE_SENT;

				execStatus = STATUS_ABORT;
				pvPut(execStatus);
				strcpy(execMessage, "Abort command sent");
				pvPut(execMessage);
				pvPut(elapsedTime);
				/* Clear abort command, post.  This is a "busy" record, don't
				 * want to do this until abort command has been sent. */
				abort=0;
				pvPut(abort);
			}
		} state monitorAbort
	}
}

/***********************************************************************************************************/
/* C functions */
%{

/* writeOnly sends a command to the Ensemble controller */
static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command)
{
	asynStatus status=0;
	int debug_out=0;
	size_t nwrite;
	char buffer[MAX_MESSAGE_STRING];

	/* Copy command so we can add terminator */
	strncpy(buffer, command, MAX_MESSAGE_STRING-3);
	strcat(buffer, "\n");
	if (!(pVar->simMode)) {
		status = pasynOctetSyncIO->write((asynUser *)pVar->pasynUser, buffer,
							   strlen(buffer), 1.0, &nwrite);
	}
	if (pVar->execState==EXECUTE_STATE_EXECUTING)
		debug_out = (pVar->debugLevel >= 7);
	else
		debug_out = (pVar->debugLevel >= 2);
	if (pVar->simMode || debug_out) printf("    writeOnly:command='%s'\n", command);
	return(status);
}


/* writeRead sends a command to the Ensemble controller and reads the response into
 * the global character buffer, stringIn.
 */
static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command, char *reply)
{
	asynStatus status=0;
	char buffer[MAX_MESSAGE_STRING];
	size_t nwrite, nread;
	int eomReason;

	strncpy(buffer, command, MAX_MESSAGE_STRING-3);

	/*strcat(buffer, "\n");*/
	/* Use 30 second timeout, some commands take a long time to reply */
	if (!(pVar->simMode)) {
		status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, buffer,
							   strlen(buffer), reply, MAX_MESSAGE_STRING, 
							   30.0, &nwrite, &nread, &eomReason);
		if (status) printf("writeRead: pasynOctetSyncIO->writeRead returned %d\n", status);
	}
	if (pVar->simMode || (pVar->debugLevel >= 3)) {
		if (buffer[strlen(buffer)-1] == '\n') buffer[strlen(buffer)-1] = '\0';
		printf("    writeRead:command='%s', reply='%s'\n", buffer, reply);
	}
	return(status);
}


/* getMotorPositions returns the positions of each motor */
#define ASCII_ACK_CHAR '%'
static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos)
{
    int j, status;
    int dir, rawP;
	double rawF;
    char inputBuff[MAX_MESSAGE_STRING], outputBuff[MAX_MESSAGE_STRING];
	size_t nwrite, nread;
	int eomReason;

	for (j=0; j<pVar->numAxes; j++) {
		sprintf(outputBuff, "PFBKPROG(@%d)", j);
		status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, outputBuff,
							   strlen(outputBuff), inputBuff, MAX_MESSAGE_STRING, 
							   30.0, &nwrite, &nread, &eomReason);
		if (status) printf("getMotorPositions: pasynOctetSyncIO->writeRead returned %d\n", status);
		if (inputBuff[0] != ASCII_ACK_CHAR)
			rawF = 0;
		else
			rawF = atof(&inputBuff[1]);
		rawP = rawF / fabs(pVar->epicsMotorMres[j]);
        if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1;
		pos[j] = rawToUser(rawP, pVar->epicsMotorOff[j], dir, pVar->epicsMotorMres[j]);
    }

	if (pVar->debugLevel > 2) {
		printf("pos[0]=%.2f\n", pos[0]);
	}
	/*epicsThreadSleep(READ_INTERVAL);*/
	return(0);
}


/* getMotorMoving returns 1 if any of the motors in movingMask are moving */
#define MAX_DONE_TRY 1
static int getMotorMoving(SS_ID ssId, struct UserVar *pVar, int movingMask)
{
    char inputBuff[MAX_MESSAGE_STRING], outputBuff[MAX_MESSAGE_STRING];
	size_t nwrite, nread;
	int eomReason;
	int move_active;
	int i, count, status;

	sprintf(outputBuff, "PLANESTATUS(0)");

	for (i=0, count=0; i<MAX_DONE_TRY; i++) {
		status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, outputBuff,
							   strlen(outputBuff), inputBuff, MAX_MESSAGE_STRING, 
							   30.0, &nwrite, &nread, &eomReason);
		if (status) printf("getMotorMoving: pasynOctetSyncIO->writeRead returned %d\n", status);
		if (strlen(inputBuff) > 3) printf("getMotorMoving: PLANESTATUS(0) returned '%s'\n", inputBuff);
		if (pVar->debugLevel > 2) {
			printf("getMotorMoving: inputBuff='%s'\n", inputBuff);
		}
		move_active = atoi(&inputBuff[1]);
		if (move_active) {
			return(1);
		} else {
			count++;
			if (count>2) break;
		}
	}

	return(0);
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

/* waitEpicsMotors waits for all motors to stop moving using the EPICS motor
 * records..  It reads and posts the motor positions during each loop. */
static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar)
{
	int j;

	/* Logic is that we always want to post position motor positions 
	 * after the end of move is detected. */
	while (getEpicsMotorMoving(ssId, pVar)) {
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

/* Calculate velocities suitable for Ensemble PVT commands.
 * We're given x(t) in the form x[i], t[i].  We need to calculate v(x) that will produce x(t).
 * Assume someone else will take care of accelerating onto, and decelerating off of trajectory.
 */
#define VELOCITY_LINEAR 1
static int buildTrajectory(SS_ID ssId, struct UserVar *pVar, double *realTimeTrajectory,
	double *motorTrajectory, int epicsMotorDir, int moveMode, int npoints, int npulses, double motorOffset,
	double motorResolution, double *velocity)
{
	double dp, dt, maxV=0;
	int i, status;
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING];

	sprintf(stringOut, "GETPARM(%d)", VelocityCommandThreshold_PARM_NUM);
	status = writeRead(ssId, pVar, stringOut, reply);

	if (reply[0] == '%') {
		maxV = atof(&reply[1]);
	} else {
		printf("buildTrajectory: Can't read velocityCommandThreshold parameter\n");
		return(-1);
	}


#if VELOCITY_LINEAR
	for (i=0; i<npoints; i++) {
		if (i==0) {
			dp = motorTrajectory[1] - motorTrajectory[0];
			dt = realTimeTrajectory[1] - realTimeTrajectory[0];
			velocity[i] = dp/dt;
		} else {
			dp = motorTrajectory[i] - motorTrajectory[i-1];
			dt = realTimeTrajectory[i] - realTimeTrajectory[i-1];
			velocity[i] = 2*dp/dt - velocity[i-1];
		}
		if (pVar->debugLevel > 2) {
			printf("point %d: pos=%f, vel=%f, time=%f\n", i, motorTrajectory[i], velocity[i], realTimeTrajectory[i]);
		}
	}
#else
	for (i=0; i<npoints; i++) {
		if (i==0) {
			dp = motorTrajectory[1] - motorTrajectory[0];
			dt = realTimeTrajectory[1] - realTimeTrajectory[0];
			velocity[i] = dp/dt;
		} else if (i < npoints-1) {
			dp = motorTrajectory[i+1] - motorTrajectory[i-1];
			dt = realTimeTrajectory[i+1] - realTimeTrajectory[i-1];
			velocity[i] = dp/dt;
		} else {
			dp = motorTrajectory[i] - motorTrajectory[i-1];
			dt = realTimeTrajectory[i] - realTimeTrajectory[i-1];
			velocity[i] = 2*dp/dt - velocity[i-1];
		}
		if (pVar->debugLevel > 2) {
			printf("point %d: pos=%f, vel=%f, time=%f\n", i, motorTrajectory[i], velocity[i], realTimeTrajectory[i]);
		}
	}
#endif

	for (i=0; i<npoints; i++) {
		if (fabs(velocity[i]) > maxV) {
			printf("velocity > limit %f at point %d: vel=%f\n", maxV, i, velocity[i]);
			return(-1);
		}
	}

	return(0);
}

static int userToRaw(double user, double off, int dir, double res) {
	return (NINT((user-off)*dir/res));
}

static double rawToUser(int raw, double off, int dir, double res) {
	/*printf("rawToUser: raw=%d, off=%f, dir=%d, res=%f, user=%f\n", raw, off, dir, res, raw*res*dir+off);*/
	return (raw*res*dir+off);
}


#define WAITLOOPS 300
#define WAITTIME 0.01

/* Many of the Ensemble commands needed for trajectories are not supported via the ASCII
 * interface, but must be executed from a precompiled AeroBasic program, which loadTrajectory()
 * has already started for us with the command "PROGRAM RUN 1 doCommand.bcx".  We get the
 * Aerobasic program to execute for us by setting Ensemble global variables to specify the
 * command and any arguments, and waiting for the AeroBasic program to notice, execute, and
 * reply by setting the command global variable back to 0.
 */
int sendReceiveCommand(SS_ID ssId, struct UserVar *pVar, char *cmd, char *callerReply, int checkProgram) {
	int i, i1, i2, i3, i4, cmdNum;
	double d1, d2, d3;
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING];
	int status, saveDebug;

	if (pVar->debugLevel > 1) {
		printf("sendReceiveCommand: command='%s'\n", cmd);
	}
	if (checkProgram) {
		/* Caller doesn't know whether or not the Aerobasic "doCommand" program is running.  Check. */
		strcpy(stringOut, "TASKSTATE(1)");
		status = writeRead(ssId, pVar, stringOut, reply);
		status = atoi(&reply[1]);
		if (reply[0] == '!') {
			printf("sendReceiveCommand: 'TASKSTATE(1)' returned error.\n");
			return(-1);
		}
		switch (status) {
		case 0:
			if (strlen(reply) == 1) {
				printf("sendReceiveCommand: TASKSTATE(1)' returned '%s', which I'm ignoring\n", reply);
			} else {
				printf("sendReceiveCommand: TASKSTATE(1)' returned '%s'\n", reply);
				return(-1);
			}
			break;
		case 3: /* running */
			break;
		case 1:	/* idle */
		case 2: /* ready but not running */
		case 4: /* paused */
		case 5: /* done */
			strcpy(stringOut, "PROGRAM RUN 1, \"doCommand.bcx\"");
			status = writeRead(ssId, pVar, stringOut, reply);
			epicsThreadSleep(.1);
			break;
		case 6: /* in error */
			strcpy(stringOut, "PROGRAM STOP 1");
			status = writeRead(ssId, pVar, stringOut, reply);
			epicsThreadSleep(.1);
			strcpy(stringOut, "PROGRAM RUN 1, \"doCommand.bcx\"");
			status = writeRead(ssId, pVar, stringOut, reply);
			epicsThreadSleep(.1);
			break;
		default:
			/* do nothing */
			break;
		}
	}

	/* Nobody wants to see debug output from all these IGLOBAL commands, so depress the debugLevel. */
	saveDebug = pVar->debugLevel;
	pVar->debugLevel = MAX(0, pVar->debugLevel - 1);

	cmdNum = cmdDONE;
	if (strncmp(cmd, "VELOCITY ON", strlen("VELOCITY ON")) == 0) {
		cmdNum = cmdVELOCITY_ON;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdVELOCITY_ON);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "DOTRAJECTORY", strlen("DOTRAJECTORY")) == 0) {
		cmdNum = cmdDOTRAJECTORY;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdDOTRAJECTORY);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "VELOCITY OFF", strlen("VELOCITY OFF")) == 0) {
		cmdNum = cmdVELOCITY_OFF;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdVELOCITY_OFF);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "HALT", strlen("HALT")) == 0) {
		cmdNum = cmdHALT;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdHALT);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "STARTABORT", strlen("STARTABORT")) == 0) {
		cmdNum = cmdSTARTABORT;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSTARTABORT);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "START", strlen("START")) == 0) {
		cmdNum = cmdSTART;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSTART);
		if (callerReply != NULL) {
			status = writeRead(ssId, pVar, stringOut, reply);
		} else {
			status = writeOnly(ssId, pVar, stringOut);
		}
	} else if (strncmp(cmd, "PVT INIT TIME ABS", strlen("PVT INIT TIME ABS")) == 0) {
		cmdNum = cmdPVT_INIT_TIME_ABS;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdPVT_INIT_TIME_ABS);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "PVT INIT TIME INC", strlen("PVT INIT TIME INC")) == 0) {
		cmdNum = cmdPVT_INIT_TIME_INC;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdPVT_INIT_TIME_INC);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "PVT1", strlen("PVT1")) == 0) {
		cmdNum = cmdPVT1;
		sscanf(cmd, "PVT1 %d %lf,%lf TIME %lf", &i1, &d1, &d2, &d3);
		status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
		status = writeDoubleAndCheck(ssId, pVar, darg1Var, d1);
		status = writeDoubleAndCheck(ssId, pVar, darg2Var, d2);
		status = writeDoubleAndCheck(ssId, pVar, darg3Var, d3);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdPVT1);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "ABORT", strlen("ABORT")) == 0) {
		cmdNum = cmdABORT;
		i1 = i2 = i3 = i4 = -1;
		i = sscanf(cmd, "ABORT @%d @%d @%d @%d", &i1, &i2, &i3, &i4);
		status = writeIntAndCheck(ssId, pVar, numIArg, i);
		status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
		status = writeIntAndCheck(ssId, pVar, iarg2Var, i2);
		status = writeIntAndCheck(ssId, pVar, iarg3Var, i3);
		status = writeIntAndCheck(ssId, pVar, iarg4Var, i4);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdABORT);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "SCOPEBUFFER", strlen("SCOPEBUFFER")) == 0) {
		cmdNum = cmdSCOPEBUFFER;
		i1 = 0;
		i = sscanf(cmd, "SCOPEBUFFER %d", &i1);
		status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSCOPEBUFFER);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "SCOPEDATA", strlen("SCOPEDATA")) == 0) {
		cmdNum = cmdSCOPEDATA;
		i1 = i2 = 0;
		i = sscanf(cmd, "SCOPEDATA %d %d", &i1, &i2);
		status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
		status = writeIntAndCheck(ssId, pVar, iarg2Var, i2);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSCOPEDATA);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "SCOPESTATUS", strlen("SCOPESTATUS")) == 0) {
		cmdNum = cmdSCOPESTATUS;
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSCOPESTATUS);
		status = writeRead(ssId, pVar, stringOut, reply);
		/* AeroBasic passes status back in IGLOBAL(iarg1Var) */
	} else if (strncmp(cmd, "SCOPETRIG STOP", strlen("SCOPETRIG STOP")) == 0) {
		cmdNum = cmdSCOPETRIG;
		status = writeIntAndCheck(ssId, pVar, iarg1Var, 1);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSCOPETRIG);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "SCOPETRIGPERIOD", strlen("SCOPETRIGPERIOD")) == 0) {
		cmdNum = cmdSCOPETRIGPERIOD;
		/* Note period is in ms, though several faster rates are possible */
		i1 = 0;
		i = sscanf(cmd, "SCOPETRIGPERIOD %d", &i1);
		status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSCOPETRIGPERIOD);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "SCOPETRIG", strlen("SCOPETRIG")) == 0) {
		cmdNum = cmdSCOPETRIG;
		status = writeIntAndCheck(ssId, pVar, iarg1Var, 0);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSCOPETRIG);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "LINEAR", strlen("LINEAR")) == 0) {
		cmdNum = cmdLINEAR;
		i1 = 0;
		d1 = d2 = 0;
		i = sscanf(cmd, "LINEAR @%d %lf F%lf", &i1, &d1, &d2);
		status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
		status = writeDoubleAndCheck(ssId, pVar, darg1Var, d1);
		status = writeDoubleAndCheck(ssId, pVar, darg2Var, d2);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdLINEAR);
		status = writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "DATAACQ", strlen("DATAACQ")) == 0) {
		if (strncmp(cmd, "DATAACQ_TRIG", strlen("DATAACQ_TRIG")) == 0) {
			cmdNum = cmdDATAACQ_TRIG;
			i1 = i2 = 0;
			i = sscanf(cmd, "DATAACQ_TRIG @%d %d", &i1, &i2);
			status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
			status = writeIntAndCheck(ssId, pVar, iarg2Var, i2);
			sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdDATAACQ_TRIG);
			status = writeRead(ssId, pVar, stringOut, reply);
		} else if (strncmp(cmd, "DATAACQ_INP", strlen("DATAACQ_INP")) == 0) {
			cmdNum = cmdDATAACQ_INP;
			i1 = i2 = 0;
			i = sscanf(cmd, "DATAACQ_INP @%d %d", &i1, &i2);
			status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
			status = writeIntAndCheck(ssId, pVar, iarg2Var, i2);
			sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdDATAACQ_INP);
			status = writeRead(ssId, pVar, stringOut, reply);
		} else if (strncmp(cmd, "DATAACQ_ON", strlen("DATAACQ_ON")) == 0) {
			cmdNum = cmdDATAACQ_ON;
			i1 = i2 = 0;
			i = sscanf(cmd, "DATAACQ_ON @%d %d", &i1, &i2);
			status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
			status = writeIntAndCheck(ssId, pVar, iarg2Var, i2);
			sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdDATAACQ_ON);
			status = writeRead(ssId, pVar, stringOut, reply);
		} else if (strncmp(cmd, "DATAACQ_OFF", strlen("DATAACQ_OFF")) == 0) {
			cmdNum = cmdDATAACQ_OFF;
			i1 = 0;
			i = sscanf(cmd, "DATAACQ_OFF @%d", &i1);
			status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
			sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdDATAACQ_OFF);
			status = writeRead(ssId, pVar, stringOut, reply);
		} else if (strncmp(cmd, "DATAACQ_READ", strlen("DATAACQ_READ")) == 0) {
			cmdNum = cmdDATAACQ_READ;
			i1 = i2 = i3 = 0;
			i = sscanf(cmd, "DATAACQ_READ @%d %d %d", &i1, &i2, &i3);
			status = writeIntAndCheck(ssId, pVar, iarg1Var, i1);
			status = writeIntAndCheck(ssId, pVar, iarg2Var, i2);
			status = writeIntAndCheck(ssId, pVar, iarg3Var, i3);
			sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdDATAACQ_READ);
			status = writeRead(ssId, pVar, stringOut, reply);
		} else {
			cmdNum = -1;
			printf("sendReceiveCommand: unexpected command: '%s'\n", cmd);
			return(-1);
		}
	}

	/* Wait for IGLOBAL(cmdVar) to be reset to zero, but don't wait for the "DOTRAJECTORY" command,
	 * because it doesn't complete until the trajectory has been executed.
	 */
	i = 0;
	if (cmdNum != cmdDOTRAJECTORY) {
		sprintf(stringOut, "IGLOBAL(%d)", cmdVar);
		for (i=0; i<WAITLOOPS; i++) {
			status = writeRead(ssId, pVar, stringOut, reply);
			if (reply[0] == '%') {
				if (atoi(&reply[1]) == -cmdNum) break;
			} 
			epicsThreadSleep(WAITTIME);
		}
	}
	pVar->debugLevel = saveDebug;

	if (i>WAITLOOPS-1) {
		printf("sendReceiveCommand: timeout (%.2f s) executing '%s'; reply='%s'\n",
			WAITLOOPS*WAITTIME, cmd, reply);
		if (callerReply != NULL) {
			printf("... reply='%s'\n", reply);
		}
		/* clear cmdVar */
		status = writeIntAndCheck(ssId, pVar, cmdVar, 0);
		return(-1);
	}
	return(0);
}

int writeDoubleAndCheck(SS_ID ssId, struct UserVar *pVar, int n, double d) {
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING];
	double dCheck = -d;
	int i, status;
	
	for (i=0; i<10; i++) {
		sprintf(stringOut, "DGLOBAL(%d) = %f", n, d);
		status = writeRead(ssId, pVar, stringOut, reply);
		if (status) return(-1);
		sprintf(stringOut, "DGLOBAL(%d)", n);
		status = writeRead(ssId, pVar, stringOut, reply);
		if (status) return(-1);
		dCheck = atof(&reply[1]);
		if (fabs(d-dCheck) < .0001) break;
		printf("writeDoubleAndCheck: fix dglobal %d (%f != %f)\n", n, d, dCheck);
	}
	if (i==10) return(-1);
	return(0);
}

int writeIntAndCheck(SS_ID ssId, struct UserVar *pVar, int n, int ival) {
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING];
	int iCheck = -ival;
	int i, status;
	
	for (i=0; i<10; i++) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", n, ival);
		status = writeRead(ssId, pVar, stringOut, reply);
		if (status) return(-1);
		sprintf(stringOut, "IGLOBAL(%d)", n);
		status = writeRead(ssId, pVar, stringOut, reply);
		if (status) return(-1);
		iCheck = atol(&reply[1]);
		if (ival == iCheck) break;
		printf("writeIntAndCheck: fix iglobal %d (%d != %d)\n", n, ival, iCheck);
	}
	if (i==10) return(-1);
	return(0);
}


/* Not all Ensemble controllers support PSOARRAY commands.  There doesn't seem to be a command
 * that allows software to determine whether or not PSOARRAY commands are supported.
 */
#define PSO_DISTANCE_ARRAY 0

/* For trajectories of unlimited length, PVT commands can be executed via sendReceiveCommand(),
 * but this has not been reliable enough, so if PVT_BY_CONTROLLER, we send the whole trajectory
 * to the controller via DGLOBAL variables.
 */
#define PVT_BY_CONTROLLER 1

/**************************************************************************************/
static int loadTrajectory(SS_ID ssId, struct UserVar *pVar) {
	int i, j, n, status;
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING];
	double position, p, v, t, dp, dtime;
	int iGlobalIndex;
	int nPulses = 1 + pVar->endPulses - pVar->startPulses;
	int intPosition;
	double accelDist, accelTime, decelDist, decelTime, positionLast;
	double home_position_set=0.;
	double posfbkprog, posfbkcal;

	pVar->loadingTrajectory = 1;
	iGlobalIndex = GLOBALINDEXSTART;

	sprintf(stringOut, "getparm(@0,%d)", ReverseMotionDirection_PARM_NUM);
	status = writeRead(ssId, pVar, stringOut, reply);
	encoder_runs_backwards = atoi(&reply[1]);
	if (pVar->debugLevel > 2) printf("loadTrajectory: encoder_runs_backwards=%d\n", encoder_runs_backwards);

	sprintf(stringOut, "getparm(@0,%d)", HomePositionSet_PARM_NUM);
	status = writeRead(ssId, pVar, stringOut, reply);
	home_position_set = atof(&reply[1]);
	if (pVar->debugLevel > 2) printf("loadTrajectory: home_position_set=%f\n", home_position_set);

	status = writeIntAndCheck(ssId, pVar, pvtWaitMSVar, 50);

	/* There may be a position offset in place.  If so, scope data will need to be corrected. */
	/* This code doesn't seem to get at the difference between scope positions and commanded positions. */
	for (j=0; j<MAX_AXES; j++) {
		sprintf(stringOut, "pfbkprog(@%d)", j);
		status = writeRead(ssId, pVar, stringOut, reply);
		posfbkprog = atof(&reply[1]);
		sprintf(stringOut, "pfbkcal(@%d)", j);
		status = writeRead(ssId, pVar, stringOut, reply);
		posfbkcal = atof(&reply[1]);
		pVar->motorPosOffset[j] = posfbkprog - posfbkcal;
	}

#if PSO_DISTANCE_ARRAY

	/* Program PSO array output */
	for (j=0; j<1; j++) {

		/* Calc accel portion of trajectory.  Note epicsMotorACCL is accel time. */
		accelTime = pVar->epicsMotorACCL[j];
		accelDist = (pVar->velocity[j][0] * pVar->epicsMotorACCL[j]) / 2;

		decelTime = pVar->epicsMotorACCL[j];
		decelDist = (pVar->velocity[j][pVar->npoints - 1] * pVar->epicsMotorACCL[j]) / 2;

		if (pVar->moveAxis[j]) {
			positionLast = pVar->motorStart[j];
			for (i=0; i<pVar->npoints; i++) {
				/* enable pulses if we're within user specified range */
				if ((pVar->outBitNum >= 0) && (pVar->outBitNum <= 15)) {
					if (i >= pVar->startPulses && i <= pVar->endPulses) {
						/* Add this pulse-out location to the list */
						position = pVar->motorTrajectory[j][i];
						intPosition = NINT((position - positionLast)/pVar->epicsMotorMres[j]);
						status = writeIntAndCheck(ssId, pVar, iGlobalIndex++, abs(intPosition));
						positionLast += intPosition*pVar->epicsMotorMres[j]; /* position of last pulse */
					}
				}

			}
		}
	}
	/* Copy from IGLOBAL variables into the drive array */
	sprintf(stringOut, "PSOARRAY @0,%d,%d", GLOBALINDEXSTART, nPulses);
	status = writeRead(ssId, pVar, stringOut, reply);
	/* Send a PSO pulse at every trajectory point. */
	strcpy(stringOut, "PSODISTANCE @0 ARRAY"); /* use IGLOBAL array */
	status = writeRead(ssId, pVar, stringOut, reply);
#endif

	/* digital I/O commands */
	if ((pVar->outBitNum >= 0) && (pVar->outBitNum <= 15)) {
/* first trajectory doesn't work.  Try commenting this out.*/
#if 0
		strcpy(stringOut, "PSOCONTROL @0 RESET");
		status = writeRead(ssId, pVar, stringOut, reply);
#endif
		/*strcpy(stringOut, "PSOOUTPUT @0 CONTROL 1");*/
		/*status = writeRead(ssId, pVar, stringOut, reply);*/

		/* (total time, on time) in microseconds */
		/* strcpy(stringOut, "PSOPULSE @0 TIME 50,25"); */
		sprintf(stringOut, "PSOPULSE @0 TIME %f,%f", pVar->pulseLenUS*1.5, pVar->pulseLenUS);
		status = writeRead(ssId, pVar, stringOut, reply);
		strcpy(stringOut, "PSOOUTPUT @0 PULSE");
		status = writeRead(ssId, pVar, stringOut, reply);

		/* some controllers need to track source 3 instead of 1. */
		/* strcpy(stringOut, "PSOTRACK @0 INPUT 1"); */
		sprintf(stringOut, "PSOTRACK @0 INPUT %d", pVar->pulseSrc);
		status = writeRead(ssId, pVar, stringOut, reply);

		/* encoder direction in which PSO pulses will be sent */
		sprintf(stringOut, "PSOTRACK @0 DIRECTION %d", pVar->pulseDir);
		status = writeRead(ssId, pVar, stringOut, reply);

		/* Not all Ensemble controllers support the "PSODISTANCE X ARRAY" command */
#if PSO_DISTANCE_ARRAY
		/* Send a PSO pulse at every trajectory point. */
		strcpy(stringOut, "PSODISTANCE @0 ARRAY"); /* use IGLOBAL array */
		status = writeRead(ssId, pVar, stringOut, reply);
#else
		/* Just send fixed-distance pulses.  For now, don't even worry about StartPulses, EndPulses */
		dp = fabs(pVar->motorTrajectory[0][pVar->npoints-1] - pVar->motorTrajectory[0][0]) / pVar->npulses;
		sprintf(stringOut, "PSODISTANCE @0 FIXED %f UNITS", dp);
		status = writeRead(ssId, pVar, stringOut, reply);
#endif

		strcpy(stringOut, "PSOCONTROL @0 ARM");
		status = writeRead(ssId, pVar, stringOut, reply);
	}


	/* trajectory commands */
	getMotorPositions(ssId, pVar, pVar->motorCurrent);

	strcpy(stringOut, "TASKSTATE(1)");
	status = writeRead(ssId, pVar, stringOut, reply);
	status = atoi(&reply[1]);
	if ((reply[0] == '!') || ((status != 1) && (status != 3) && (status != 5))) {
		strcpy(stringOut, "PROGRAM STOP 1");
		status = writeRead(ssId, pVar, stringOut, reply);
		strcpy(stringOut, "TASKSTATE(1)");
		status = writeRead(ssId, pVar, stringOut, reply);
	}

	status = writeIntAndCheck(ssId, pVar, cmdVar, 0);
	strcpy(stringOut, "PROGRAM RUN 1, \"doCommand.bcx\"");
	status = writeRead(ssId, pVar, stringOut, reply);
	epicsThreadSleep(.1);

#if USE_SCOPE
	/* Program SCOPE commands */
	pVar->nScopeDataPoints = pVar->npoints * 2.2;
	if (pVar->nScopeDataPoints > MAX_PULSES) pVar->nScopeDataPoints = MAX_PULSES;
	pVar->scopeDataIntervalMS = NINT(1000 * (pVar->time*1.1)/pVar->nScopeDataPoints);
	if (pVar->scopeDataIntervalMS < 1) pVar->scopeDataIntervalMS = 1;
	sprintf(stringOut, "SCOPEBUFFER %d", pVar->nScopeDataPoints);
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1);
	sprintf(stringOut, "SCOPETRIGPERIOD %d", pVar->scopeDataIntervalMS);
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
#endif

#if USE_DATAACQ
	/* Program data-acquisition commands */
	status = sendReceiveCommand(ssId, pVar, "DATAACQ_TRIG @0 2", reply, 0); /* trigger on PSO */
	status = sendReceiveCommand(ssId, pVar, "DATAACQ_INP @0 0", reply, 0); /* acquire encoder */
	sprintf(stringOut, "DATAACQ_ON @0 %d", pVar->npoints); /* acquire pVar->npoints data points */
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
#endif

#if PVT_BY_CONTROLLER

	/* Get motor number into Ensemble's IGLOBAL(iarg1Var) for use by DOTRAJECTORY command. */
	for (j=0; j<MAX_AXES; j++) {
		if (pVar->moveAxis[j]) break;
	}
	status = writeIntAndCheck(ssId, pVar, iarg1Var, j);

	/* Calc accel portion of trajectory.  Note epicsMotorACCL is accel time. */
	accelTime = pVar->epicsMotorACCL[j];
	accelDist = (pVar->velocity[j][0] * pVar->epicsMotorACCL[j]) / 2;

	decelTime = pVar->epicsMotorACCL[j];
	decelDist = (pVar->velocity[j][pVar->npoints - 1] * pVar->epicsMotorACCL[j]) / 2;

	n = 0;
	status = 0;
	status += writeDoubleAndCheck(ssId, pVar, n++, pVar->motorTrajectory[j][0]);
	status += writeDoubleAndCheck(ssId, pVar, n++, pVar->velocity[j][0]);
	status += writeDoubleAndCheck(ssId, pVar, n++, accelTime);
	if (status) goto bad;

	for (i=1; i<pVar->npoints; i++) {
		p = pVar->motorTrajectory[j][i];
		v = pVar->velocity[j][i];
		t = accelTime + pVar->realTimeTrajectory[i];
		status += writeDoubleAndCheck(ssId, pVar, n++, p);
		status += writeDoubleAndCheck(ssId, pVar, n++, v);
		status += writeDoubleAndCheck(ssId, pVar, n++, t);
		if (status) goto bad;
	}

	/* add extra trajectory point to reduce end transient */
	p = 2*pVar->motorTrajectory[j][pVar->npoints-1] - pVar->motorTrajectory[j][pVar->npoints-2];
	v = pVar->velocity[j][pVar->npoints-1];
	t = accelTime + 2*pVar->realTimeTrajectory[pVar->npoints-1] - pVar->realTimeTrajectory[pVar->npoints-2];
	status += writeDoubleAndCheck(ssId, pVar, n++, p);
	status += writeDoubleAndCheck(ssId, pVar, n++, v);
	status += writeDoubleAndCheck(ssId, pVar, n++, t);
	if (status) goto bad;

	p = p + decelDist*.9;
	v = pVar->velocity[j][pVar->npoints-1]*.1;
	t = t + decelTime*.9;
	status += writeDoubleAndCheck(ssId, pVar, n++, p);
	status += writeDoubleAndCheck(ssId, pVar, n++, v);
	status += writeDoubleAndCheck(ssId, pVar, n++, t);
	if (status) goto bad;

	p = p + decelDist*.1;
	v = 0.;
	t = t + decelTime*.1;
	status += writeDoubleAndCheck(ssId, pVar, n++, p);
	status += writeDoubleAndCheck(ssId, pVar, n++, v);
	status += writeDoubleAndCheck(ssId, pVar, n++, t);
	if (status) goto bad;

	/* Set Ensemble's IGLOBAL(iarg2Var) to actual number of trajectory points, including accel/decel */
	status = writeIntAndCheck(ssId, pVar, iarg2Var, n/3);

#if USE_SCOPE
	status = sendReceiveCommand(ssId, pVar, "SCOPETRIG", reply, 0);
#endif

	epicsTimeGetCurrent(&startTime);
	sprintf(stringOut, "DOTRAJECTORY");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);

#else

#if USE_SCOPE
	status = sendReceiveCommand(ssId, pVar, "SCOPETRIG", reply, 0);
#endif
	/* Program motion */
	status = writeIntAndCheck(ssId, pVar, pvtWaitMSVar, 30);

	strcpy(stringOut, "VELOCITY ON");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
	if (status) goto bad;

	sprintf(stringOut, "PVT INIT TIME ABS");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
	if (status) goto bad;


	for (j=0; j<MAX_AXES; j++) {

		/* Calc accel portion of trajectory.  Note epicsMotorACCL is accel time. */
		accelTime = pVar->epicsMotorACCL[j];
		accelDist = (pVar->velocity[j][0] * pVar->epicsMotorACCL[j]) / 2;

		decelTime = pVar->epicsMotorACCL[j];
		decelDist = (pVar->velocity[j][pVar->npoints - 1] * pVar->epicsMotorACCL[j]) / 2;

		if (pVar->moveAxis[j]) {

			sprintf(stringOut, "PVT1 %d %f, %f TIME %f",
				j, pVar->motorTrajectory[j][0], pVar->velocity[j][0], accelTime);
			status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
			epicsTimeGetCurrent(&lastPvtTime);
			if (status) {
				/* try again */
				printf("loadTrajectory: first PVT command returned error\n");
				strcpy(stringOut, "TASKSTATE(1)");
				status = writeRead(ssId, pVar, stringOut, reply);
				printf("...TASKSTATE(1) returns '%s'\n", reply);
				strcpy(stringOut, "TASKERROR(1)");
				status = writeRead(ssId, pVar, stringOut, reply);
				printf("...TASKERROR(1) returns '%s'\n", reply);
				strcpy(stringOut, "PLANESTATUS(0)");
				status = writeRead(ssId, pVar, stringOut, reply);
				printf("PLANESTATUS(0) returns '%s'\n", reply);
				strcpy(stringOut, "AXISSTATUS(@0)");
				status = writeRead(ssId, pVar, stringOut, reply);
				printf("AXISSTATUS(@0) returns '%s'\n", reply);
				strcpy(stringOut, "VELOCITY OFF");
				status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1);
				strcpy(stringOut, "STARTABORT");
				status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1);
				status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn);

				strcpy(stringOut, "acknowledgeall");
				status = writeRead(ssId, pVar, stringOut, reply);
				printf("...acknowledgeall returns '%s'\n", reply);
				printf("...retrying first PVT command again\n", reply);

				strcpy(stringOut, "VELOCITY ON");
				status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
				sprintf(stringOut, "PVT INIT TIME ABS");
				status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
				sprintf(stringOut, "PVT1 %d %f, %f TIME %f",
					j, pVar->motorTrajectory[j][0], pVar->velocity[j][0], accelTime);
				status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
				epicsTimeGetCurrent(&lastPvtTime);
				if (status) {
					goto bad;
				}
			}

			for (i=1; i<pVar->npoints; i++) {
				/* Because trajectories > 14 points can't be fully preloaded, but must be loaded
				 * while they are executing, we have to handle aborts and user notification here.
				 */
				if (pVar->abortState != ABORT_STATE_NONE) goto abort;

				p = pVar->motorTrajectory[j][i];

				t = accelTime + pVar->realTimeTrajectory[i];
				sprintf(stringOut, "PVT1 %d %f, %f TIME %f", j, p, pVar->velocity[j][i], t);
				status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);

				if (pVar->debugLevel > 1) {
					epicsTimeGetCurrent(&pvtTime);
					dtime = epicsTimeDiffInSeconds(&pvtTime, &lastPvtTime);
					printf("PVT at dt=%.2f %s\n", dtime, (i>14 && dtime<.1) ? "***" : "");
					epicsTimeGetCurrent(&lastPvtTime);
				}
				if (status) {
					goto bad;
				}
				epicsTimeGetCurrent(&currTime);
				dtime = epicsTimeDiffInSeconds(&currTime, &lastPollTime);
				if (dtime > POLL_INTERVAL) {
					pVar->elapsedTime = epicsTimeDiffInSeconds(&currTime, &startTime);
					seq_pvPut(ssId, pVar->elapsedTimeIndex, 0);
					epicsTimeGetCurrent(&lastPollTime);
					getMotorPositions(ssId, pVar, pVar->motorCurrent);
					for (k=0; k<pVar->numAxes; k++) {
						if (pVar->moveAxis[k]) {
							seq_pvPut(ssId, pVar->motorCurrentIndex[k], 0);
						}
					}
				}
			}

			/* add extra trajectory point to reduce end transient */
			p = 2*pVar->motorTrajectory[j][pVar->npoints-1] - pVar->motorTrajectory[j][pVar->npoints-2];

			t = accelTime + 2*pVar->realTimeTrajectory[pVar->npoints-1] - pVar->realTimeTrajectory[pVar->npoints-2];

			sprintf(stringOut, "PVT1 %d %f, %f TIME %f", j, p, pVar->velocity[j][pVar->npoints-1], t);
			status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
			if (status) goto bad;

			p = p + decelDist*.9;

			t = t + decelTime*.9;

			sprintf(stringOut, "PVT1 %d %f, %f TIME %f", j, p, pVar->velocity[j][pVar->npoints-1]*.1, t);
			status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
			if (status) goto bad;

			strcpy(stringOut, "VELOCITY OFF");
			status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
			if (status) goto bad;

			p = p + decelDist*.1;

			t = t + decelTime*.1;

			sprintf(stringOut, "PVT1 %d %f, %f TIME %f", j, p, 0., t);
			status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0);
			if (status) goto bad;
		}
	}

#endif

	if (pVar->abortState != ABORT_STATE_NONE) goto abort;
	
	strcpy(stringOut, "TASKSTATE(1)");
	status = writeRead(ssId, pVar, stringOut, reply);
	if (pVar->debugLevel) printf("TASKSTATE = '%s', ", reply);
	strcpy(stringOut, "TASKERROR(1)");
	status = writeRead(ssId, pVar, stringOut, reply);
	if (pVar->debugLevel) printf("TASKERROR = '%s', ", reply);
	for (i=0; i<50; i++) {
		strcpy(stringOut, "PLANESTATUS(0)");
		status = writeRead(ssId, pVar, stringOut, reply);
		if (reply[1] == '3') break;
		epicsThreadSleep(.1);
	}
	if (i>=50) {
		printf("loadTrajectory: trajectory didn't start. (PLANESTATUS = '%s')\n", reply);
		strcpy(stringOut, "VELOCITY OFF");
		status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1);
		/*strcpy(stringOut, "STARTABORT");*/
		/*status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1);*/
		status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn);
		return(-2);
	}
	if (pVar->debugLevel) printf("PLANESTATUS = '%s'\n", reply);
	pVar->loadingTrajectory = 0;
	if (pVar->debugLevel) printf("loadTrajectory: normal exit after %.1f s wait\n", i/10.);
	return(0);

bad:
	printf("loadTrajectory: error\n");
	/* Find out why it failed. */
	strcpy(stringOut, "TASKSTATE(1)");
	status = writeRead(ssId, pVar, stringOut, reply);
	printf("TASKSTATE(1) returns '%s'\n", reply);
	strcpy(stringOut, "TASKERROR(1)");
	status = writeRead(ssId, pVar, stringOut, reply);
	printf("TASKERROR(1) returns '%s'\n", reply);
	strcpy(stringOut, "PLANESTATUS(0)");
	status = writeRead(ssId, pVar, stringOut, reply);
	printf("PLANESTATUS(0) returns '%s'\n", reply);
	strcpy(stringOut, "AXISSTATUS(@0)");
	status = writeRead(ssId, pVar, stringOut, reply);
	printf("AXISSTATUS(@0) returns '%s' (%x)\n", reply, atoi(&reply[1]));
	pVar->loadingTrajectory = 0;
	/* return(-1); */
	printf("loadTrajectory: aborting because of error\n");

abort:
	printf("loadTrajectory: aborted\n");
	strcpy(stringOut, "VELOCITY OFF");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1);
	strcpy(stringOut, "STARTABORT");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1);
	status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn);
	pVar->abortState = ABORT_STATE_SENT;
	pVar->loadingTrajectory = 0;
	if ((pVar->outBitNum >= 0) && (pVar->outBitNum <= 15)) {
		strcpy(stringOut, "PSOCONTROL @0 OFF");
		status = writeRead(ssId, pVar, stringOut, reply);
	}
	return(-1);
}

double eval_fStart(double fStart, double *motorReadbacks, double scopeDataInterval, int nScopeDataPoints,
	double *motorTrajectory, double *realTimeTrajectory, int npoints);
int  parabola(double *x, double *y, int npts, double *xFit);

static double trajEval(SS_ID ssId, struct UserVar *pVar, double *motorReadbacks, int scopeDataIntervalMS, int nScopeDataPoints,
	double *motorTrajectory, double *realTimeTrajectory, int npoints, double *motorError) {

	int i, rStart, lower, iThis;
	double fStart, fThis, frac, interp, scopeDataInterval = scopeDataIntervalMS*.001;
	double p;

	/* for optimizing fStart */
	double fTry, x[10], y[10];
	int nTry;

	/* fStart is the fractional index for which motorReadbacks[fStart] == motorTrajectory[0].  Find it. */
	lower = motorReadbacks[0] < motorTrajectory[0];
	for (rStart=0; rStart < nScopeDataPoints/2; rStart++) {
		if (pVar->debugLevel>5) printf("trajEval: motorReadbacks[rStart]=%f, motorTrajectory[0]=%f\n", motorReadbacks[rStart], motorTrajectory[0]);
		if ((motorReadbacks[rStart] > motorTrajectory[0]) == lower) break;
	}
	if (rStart>1 && fabs(motorReadbacks[rStart] - motorReadbacks[rStart-1]) > 1.e-3) {
		frac = (motorTrajectory[0] - motorReadbacks[rStart-1]) / (motorReadbacks[rStart] - motorReadbacks[rStart-1]);
	} else {
		frac = 0.;
	}
	fStart = 0.;
	frac = 0.;
	if (rStart>0) {
		fStart = (rStart-1) + frac;
	}
	if (pVar->debugLevel>5) printf("trajEval: rStart=%d, frac=%f, fStart=%f\n",  rStart, frac, fStart);

	/* Try to improve fStart */
	fTry = MAX(0,fStart-2);
	nTry = 5;
	for (i=0; i<nTry; i++, fTry += 1) {
		x[i] = fTry;
		y[i] = eval_fStart(fTry, motorReadbacks, scopeDataInterval, nScopeDataPoints, motorTrajectory, realTimeTrajectory, npoints);
	}
	i = parabola(x, y, nTry, &fTry);
	if (i==0) fStart = fTry;
	if (pVar->debugLevel>5) printf("trajEval: fStart=%f\n", fStart);

	/* Calculate SUM(motorReadbacks[t+fStart] - motorTrajectory[t]) */
	for (i=0; i<npoints; i++) {
		fThis = fStart + (realTimeTrajectory[i] - realTimeTrajectory[0])/scopeDataInterval;
		iThis = floor(fThis);
		if (iThis+1 < nScopeDataPoints) {
			if (pVar->debugLevel>5) printf("trajEval: iThis=%d, motorReadbacks[iThis]=%f\n", iThis, motorReadbacks[iThis]);
			p = fThis-iThis;
			if ((iThis > 0) && (iThis+2 < nScopeDataPoints)) {
				/* Lagrange 4-point interpolation */
				interp = (-p*(p-1)*(p-2)/6)*motorReadbacks[iThis-1] +
						((p*p-1)*(p-2)/2)*motorReadbacks[iThis] +
						(-p*(p+1)*(p-2)/2)*motorReadbacks[iThis+1] +
						(p*(p*p-1)/6)*motorReadbacks[iThis+2];
			} else if ((iThis > 0) && (iThis+1 < nScopeDataPoints)) {
				/* Lagrange 3-point interpolation */
				interp = (p*(p-1)/2)*motorReadbacks[iThis-1] + (1-p*p)*motorReadbacks[iThis] + (p*(p+1)/2)*motorReadbacks[iThis+1];
			} else {
				/* linear interpolation */
				interp = motorReadbacks[iThis] + (motorReadbacks[iThis+1] - motorReadbacks[iThis])*p;
			}
			if (pVar->debugLevel>5) printf("trajEval: fThis=%f, interp=%f, motorTrajectory[%d]=%f, \n", fThis, interp, i, motorTrajectory[i]);
			motorReadbacks[i] = interp;
			motorError[i] = interp - motorTrajectory[i];
		} else {
			motorReadbacks[i] = 0;
			motorError[i] = 0;
		}
	}
	for(; i<nScopeDataPoints; i++) {
		motorReadbacks[i] = 0;
		motorError[i] = 0;
	}
	return(0.);
}

double eval_fStart(double fStart, double *motorReadbacks, double scopeDataInterval, int nScopeDataPoints,
	double *motorTrajectory, double *realTimeTrajectory, int npoints) {

	double fThis, p, interp, chisq = 0;
	int i, iThis, numChi = 0;

	/* Calculate SUM(motorReadbacks[t+fStart] - motorTrajectory[t]) */
	for (i=0; i<npoints; i++) {
		fThis = fStart + (realTimeTrajectory[i] - realTimeTrajectory[0])/scopeDataInterval;
		iThis = floor(fThis);
		if (iThis+1 < nScopeDataPoints) {
			/* printf("eval_fStart: iThis=%d, motorReadbacks[iThis]=%f\n", iThis, motorReadbacks[iThis]);*/
			p = fThis-iThis;
			if ((iThis > 0) && (iThis+2 < nScopeDataPoints)) {
				/* Lagrange 4-point interpolation */
				interp = (-p*(p-1)*(p-2)/6)*motorReadbacks[iThis-1] +
						((p*p-1)*(p-2)/2)*motorReadbacks[iThis] +
						(-p*(p+1)*(p-2)/2)*motorReadbacks[iThis+1] +
						(p*(p*p-1)/6)*motorReadbacks[iThis+2];
			} else if ((iThis > 0) && (iThis+1 < nScopeDataPoints)) {
				/* Lagrange 3-point interpolation */
				interp = (p*(p-1)/2)*motorReadbacks[iThis-1] + (1-p*p)*motorReadbacks[iThis] + (p*(p+1)/2)*motorReadbacks[iThis+1];
			} else {
				/* linear interpolation */
				interp = motorReadbacks[iThis] + (motorReadbacks[iThis+1] - motorReadbacks[iThis])*p;
			}
			/* printf("eval_fStart: fThis=%f, interp=%f, motorTrajectory[%d]=%f, \n", fThis, interp, i, motorTrajectory[i]);*/
			chisq += (interp - motorTrajectory[i]) * (interp - motorTrajectory[i]);
			numChi++;
		}
	}
	return(chisq/numChi);
}

#define SMALL 1e-8
#define LARGE 1e10
int  parabola(double *x, double *y, int npts, double *xFit) {

	double xf, yf;
	double xx, yy, xi, xi2, xi3, xi4, yi, yixi, yixi2;
	int i;
	double numer, denom;
	double xmin=LARGE, xmax=-LARGE;

	/* make sums */
	xi = yi = yixi = xi2 = yixi2 = xi3 = xi4 = 0.0;
	for (i=0; i<npts; i++) {
		xmin = MIN(xmin, x[i]);
		xmax = MAX(xmax, x[i]);
		xi	  += (xx = x[i]);
		yi	  += (yy = y[i]);
		yixi  += yy * xx;
		xi2   += (xx *=x[i]);
		yixi2 += yy * xx;
		xi3   += (xx *= x[i]);
		xi4   += xx * x[i];
	}

	/* make averages */
	xi    /= npts;
	xi2   /= npts;
	xi3   /= npts;
	xi4   /= npts;
	yi    /= npts;
	yixi  /= npts;
	yixi2 /= npts;

	/* x coordinate of minimum of parabola */
	numer =    yi*(xi2*xi3 - xi*xi4) + yixi*(xi4 - xi2*xi2) + yixi2*(xi*xi2 - xi3);
	denom = 2*(yi*(xi2*xi2 - xi*xi3) + yixi*(xi3 -  xi*xi2) + yixi2*(xi*xi  - xi2));
	/* printf("	calculating xf: npts=%d; numer=%.3g; denom=%.3g\n", npts, numer, denom); */
	if (fabs(denom) < SMALL) return(-1);
	xf = numer / denom;
	if ((xf < xmin) || (xf > xmax)) return(-1);	/* Don't extrapolate */

	numer = (xf*xf - 2*xf*xi + xi2) * (yixi2 - yi*xi2);
	denom = 2*xf*(xi*xi2 - xi3) - xi2*xi2 + xi4;
	yf = yi - numer/denom;
	if (fabs(denom) < SMALL) return(-1);

	*xFit = xf;
	return(0);
}

}%
