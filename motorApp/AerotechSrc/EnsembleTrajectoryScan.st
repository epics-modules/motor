program EnsembleTrajectoryScan("P=13IDC:,R=traj1,M1=M1,M2=M2,M3=M3,M4=M4,M5=M5,M6=M6,M7=M7,M8=M8,PORT=serial1")

/* sample program invocation:
 * dbLoadRecords("$(MOTOR)/motorApp/Db/trajectoryScan.db","P=xxx:,R=traj1:,NAXES=2,NELM=100,NPULSE=100")
 * ...
 * iocInit()
 * ...
 * seq &EnsembleTrajectoryScan, "P=xxx:,R=traj1:,M1=m1,M2=m2,M3=m3,M4=m4,M5=m5,M6=m6,M7=m7,M8=m8,PORT=none"
 */

/*  This sequencer program works with trajectoryScan.db.  It implements
 *  coordinated trajectory motion with an Aerotech Ensemble motor controller.
 *
 *  Tim Mooney -- based on MM4000_trajectoryScan.st by Mark Rivers.
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
#define MAX_PULSES 1000

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
char sbuf[MAX_MESSAGE_STRING];
char stringIn[MAX_MESSAGE_STRING];
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
%% static int sendReceiveCommand(SS_ID ssId, struct UserVar *pVar, char *cmd, char *callerReply);
%% static int testAbort(SS_ID ssId, struct UserVar *pVar);

double position[MAX_AXES][MAX_ELEMENTS];
double velocity[MAX_AXES][MAX_ELEMENTS];
double motorStart[MAX_AXES];
double motorCurr[MAX_AXES];
double dbuf[MAX_PULSES];

/* variables for constructing trajectory commands */
int movingMask;

/* temporary variables to hold mav speed and acceleration for a motor */
double vmax;
double amax;
double d;

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
		} state monitor_inputs
	}

	/* Monitor inputs which control what to do (Build, Execute, Read) */
	state monitor_inputs {
		when(efTestAndClear(buildMon) && (build==1) && (initStatus == STATUS_SUCCESS)) {
		} state build

		when(efTestAndClear(executeMon) && (execute==1) && ((buildStatus == STATUS_SUCCESS) || (buildStatus == STATUS_WARNING))) {
		} state execute

		when(efTestAndClear(readbackMon) && (readback==1) /*&& (execStatus == STATUS_SUCCESS)*/) {
		} state readback

		when(efTestAndClear(nelementsMon) && (nelements>=1)) {
			/* If nelements changes, then change endPulses to this value,
			 * since this is what the user normally wants.  endPulses can be
			 * changed again after changing nelements if this is desired. */
			endPulses = nelements;
			pvPut(endPulses);
		} state monitor_inputs

		when(efTestAndClear(motorMDVSMon)) {
			/* We don't use this. */
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

			/* calculate time at which motor should reach each trajectory point */			
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
					amax = vmax/epicsMotorACCL[j];
					for (k=0; k<npoints && !limitViolation; k++) {
						posActual = motorTrajectory[j][k];
						if (moveMode != MOVE_MODE_ABSOLUTE) posActual += epicsMotorPos[j];
						limitViolation |= (posActual > epicsMotorHLM[j]) || (posActual < epicsMotorLLM[j]);
                		if (limitViolation) {
							epicsSnprintf(buildMessage, MSGSIZE, "Limit: m%d at pt. %d (%f)", j+1, k+1, posActual);
						}
						if (velocity[j][k]*epicsMotorMres[j] > vmax) {
							limitViolation |= 1;
							epicsSnprintf(buildMessage, MSGSIZE, "V limit: m%d at pt. %d (%f)", j+1, k+1,
								velocity[j][k]*epicsMotorMres[j]);
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
			/* Set busy flag */
			execState = EXECUTE_STATE_MOVE_START;
			pvPut(execState);
			abortState = ABORT_STATE_NONE;
			/* Set status to INVALID */
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
							%%writeOnly(ssId, pVar, pVar->abortCommand);
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
				%%loadTrajectory(ssId, pVar);
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

		when (execState==EXECUTE_STATE_EXECUTING) {

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
					%%writeOnly(ssId, pVar, pVar->abortCommand);
					abortState = ABORT_STATE_SENT;
					if (debugLevel) printf("timeout: sent command '%s'\n", abortCommand);
					strcpy(stringOut, "PROGRAM STOP 5");
					%%writeOnly(ssId, pVar, pVar->stringOut);

					%%waitEpicsMotors(ssId, pVar); /* wait until all motors are done */
					abortState = ABORT_STATE_DONE;
				}
			}
			/* Check for errors while trajectories are in progress */
		} state wait_execute

		when (execState==EXECUTE_STATE_FLYBACK) {
			if (debugLevel) printf("\nflyback.\n");
			pvPut(elapsedTime);
			pvPut(execState);
			pvPut(execStatus);
			pvPut(execMessage);
			/* Get the current motor positions, post them */
			%%getMotorPositions(ssId, pVar, pVar->motorCurrent);
			for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
			for (j=0; j<numAxes; j++) {
				if (moveAxis[j]) {
					epicsMotorPos[j] = motorCurrent[j];
					pvPut(epicsMotorPos[j]);
				}
			}
			%%waitEpicsMotors(ssId, pVar);
			if (debugLevel) printf("\n...flyback done\n");

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
				%%writeOnly(ssId, pVar, pVar->abortCommand);
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
			if (debugLevel==3) {
				printf("trajectoryAbort: calling testAbort\n");
				%%testAbort(ssId, pVar);
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

	strcat(buffer, "\n");
	/* Use 30 second timeout, some commands take a long time to reply */
	if (!(pVar->simMode)) {
		status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, buffer,
							   strlen(buffer), reply, MAX_MESSAGE_STRING, 
							   30.0, &nwrite, &nread, &eomReason);
	}
	if (pVar->simMode || (pVar->debugLevel >= 2)) {
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
		if (inputBuff[0] != ASCII_ACK_CHAR)
			rawF = 0;
		else
			rawF = atof(&inputBuff[1]);
		rawP = rawF / fabs(pVar->epicsMotorMres[j]);
        if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1;
		pos[j] = rawToUser(rawP, pVar->epicsMotorOff[j], dir, pVar->epicsMotorMres[j]);
    }

	if (pVar->debugLevel > 2) {
		printf("\npos[0]=%f", pos[0]);
	}
	/*epicsThreadSleep(READ_INTERVAL);*/
	return(0);
}


/* getMotorMoving returns 1 if any of the motors in movingMask are moving */
static int getMotorMoving(SS_ID ssId, struct UserVar *pVar, int movingMask)
{
    char inputBuff[MAX_MESSAGE_STRING], outputBuff[MAX_MESSAGE_STRING];
	size_t nwrite, nread;
	int eomReason;
	int move_active;
	int status;

	sprintf(outputBuff, "PLANESTATUS(0)");
	status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, outputBuff,
						   strlen(outputBuff), inputBuff, MAX_MESSAGE_STRING, 
						   30.0, &nwrite, &nread, &eomReason);
	if (pVar->debugLevel > 2) {
		printf("\ngetMotorMoving: inputBuff='%s'", inputBuff);
	}
	move_active = 0x01 & atoi(&inputBuff[1]);

	if (move_active) return(1);
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
#define VELOCITY_LINEAR 0
static int buildTrajectory(SS_ID ssId, struct UserVar *pVar, double *realTimeTrajectory,
	double *motorTrajectory, int epicsMotorDir, int moveMode, int npoints, int npulses, double motorOffset,
	double motorResolution, double *velocity)
{
	double dp, dt;
	int i;

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
		if (pVar->debugLevel > 0) {
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
		if (pVar->debugLevel > 0) {
			printf("point %d: pos=%f, vel=%f, time=%f\n", i, motorTrajectory[i], velocity[i], realTimeTrajectory[i]);
		}
	}
#endif

	return(0);
}

static int userToRaw(double user, double off, int dir, double res) {
	return (NINT((user-off)*dir/res));
}

static double rawToUser(int raw, double off, int dir, double res) {
	/*printf("rawToUser: raw=%d, off=%f, dir=%d, res=%f, user=%f\n", raw, off, dir, res, raw*res*dir+off);*/
	return (raw*res*dir+off);
}

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

#define WAITLOOPS 100
#define WAITTIME 0.01

/* Many of the Ensemble commands needed for trajectories are not supported via the ASCII
 * interface, but must be executed from a precompiled AeroBasic program, which loadTrajectory()
 * has already started for us with the command "PROGRAM RUN 5 doCommand.bcx".  We get the
 * Aerobasic program to execute for us by setting Ensemble global variables to specify the
 * command and any arguments, and waiting for the AeroBasic program to notice, execute, and
 * reply by setting the command global variable back to 0.
 */
int sendReceiveCommand(SS_ID ssId, struct UserVar *pVar, char *cmd, char *callerReply) {
	int i, i1, i2, i3, i4;
	double d1, d2, d3, d4, d5, d6, d7, d8, d9;
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING];
	int status, saveDebug;

	strcpy(stringOut, "TASKSTATE(5)");
	writeRead(ssId, pVar, stringOut, reply);
	status = atoi(&reply[1]);
	if (reply[0] == '!') {
		printf("sendReceiveCommand: 'TASKSTATE(5)' returned error.\n");
		return(-1);
	}
	switch (status) {
	case 0:
		printf("sendReceiveCommand: Can't run AeroBasic program\n");
		return(-1);
	case 3: /* running */
		break;
	case 1:	/* idle */
	case 2: /* ready but not running */
	case 4: /* paused */
	case 5: /* done */
		strcpy(stringOut, "PROGRAM RUN 5, \"doCommand.bcx\"");
		writeRead(ssId, pVar, stringOut, reply);
		epicsThreadSleep(.1);
		break;
	case 6: /* in error */
		strcpy(stringOut, "PROGRAM STOP 5");
		writeRead(ssId, pVar, stringOut, reply);
		epicsThreadSleep(.1);
		strcpy(stringOut, "PROGRAM RUN 5, \"doCommand.bcx\"");
		writeRead(ssId, pVar, stringOut, reply);
		epicsThreadSleep(.1);
		break;
	default:
		/* do nothing */
		break;
	}

	if (pVar->debugLevel > 0) {
		printf("command: '%s'\n", cmd);
	}

	/* Nobody wants to see debug output from all these IGLOBAL commands, so depress the debugLevel. */
	saveDebug = pVar->debugLevel;
	pVar->debugLevel = pVar->debugLevel - 1;

	if (strncmp(cmd, "VELOCITY ON", strlen("VELOCITY ON")) == 0) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdVELOCITY_ON);
		writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "VELOCITY OFF", strlen("VELOCITY OFF")) == 0) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdVELOCITY_OFF);
		writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "HALT", strlen("HALT")) == 0) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdHALT);
		writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "STARTABORT", strlen("STARTABORT")) == 0) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSTARTABORT);
		writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "START", strlen("START")) == 0) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdSTART);
		if (callerReply != NULL) {
			writeRead(ssId, pVar, stringOut, reply);
		} else {
			writeOnly(ssId, pVar, stringOut);
		}
	} else if (strncmp(cmd, "PVT INIT TIME ABS", strlen("PVT INIT TIME ABS")) == 0) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdPVT_INIT_TIME_ABS);
		writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "PVT INIT TIME INC", strlen("PVT INIT TIME INC")) == 0) {
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdPVT_INIT_TIME_INC);
		writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "PVT1", strlen("PVT1")) == 0) {
		sscanf(cmd, "PVT1 %d %lf,%lf TIME %lf", &i1, &d1, &d2, &d3);
		sprintf(stringOut, "IGLOBAL(%d) = %d", iarg1Var, i1);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "DGLOBAL(%d) = %f", darg1Var, d1);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "DGLOBAL(%d) = %f", darg2Var, d2);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "DGLOBAL(%d) = %f", darg3Var, d3);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdPVT1);
		writeRead(ssId, pVar, stringOut, reply);
	} else if (strncmp(cmd, "ABORT", strlen("ABORT")) == 0) {
		i1 = i2 = i3 = i4 = -1;
		i = sscanf(cmd, "ABORT @%d @%d @%d @%d", &i1, &i2, &i3, &i4);
		sprintf(stringOut, "IGLOBAL(%d) = %d", numIArg, i);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "IGLOBAL(%d) = %d", iarg1Var, i1);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "IGLOBAL(%d) = %d", iarg2Var, i2);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "IGLOBAL(%d) = %d", iarg3Var, i3);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "IGLOBAL(%d) = %d", iarg4Var, i4);
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "IGLOBAL(%d) = %d", cmdVar, cmdABORT);
		writeRead(ssId, pVar, stringOut, reply);
	}

	/* Wait for IGLOBAL(cmdVar) to be reset to zero, but don't wait for the "START" command,
	 * because it doesn't complete until the trajectory is finished.
	 */
	i = 0;
	if (strncmp(cmd, "START", strlen("START")) != 0) {
		sprintf(stringOut, "IGLOBAL(%d)", cmdVar);
		writeRead(ssId, pVar, stringOut, reply);
		for (i=0; i<WAITLOOPS && (strncmp(reply, "%0", strlen("%0")) != 0); i++) {
			epicsThreadSleep(WAITTIME);
			writeRead(ssId, pVar, stringOut, reply);
		}
	}
	pVar->debugLevel = saveDebug;

	if (i>WAITLOOPS-1) {
		printf("sendReceiveCommand: timeout (%.2f s) executing '%s'; reply='%s'\n",
			WAITLOOPS*WAITTIME, cmd, reply);
		if (callerReply != NULL) {
			printf("... reply='%s'\n", reply);
		}
		return(-1);
	}
	return(0);
}

#define PSO_DISTANCE_ARRAY 0
/**************************************************************************************/
static int loadTrajectory(SS_ID ssId, struct UserVar *pVar) {
	int i, j, k, status;
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING];
	double position, dp, dt, dtime;
	int dir;
	int iGlobalIndex, iGlobalIndexStart = 50;
	int nPulses = 1 + pVar->endPulses - pVar->startPulses;
	int intPosition, intPositionLast;
	double accelDist, accelTime, decelDist, decelTime;

	pVar->loadingTrajectory = 1;
	iGlobalIndex = iGlobalIndexStart;

	/* digital I/O commands */
	if ((pVar->outBitNum >= 0) && (pVar->outBitNum <= 15)) {
		strcpy(stringOut, "PSOCONTROL @0 RESET");
		writeRead(ssId, pVar, stringOut, reply);

		/*strcpy(stringOut, "PSOOUTPUT @0 CONTROL 1");*/
		/*writeRead(ssId, pVar, stringOut, reply);*/

		/* (total time, on time) in microseconds */
		/* strcpy(stringOut, "PSOPULSE @0 TIME 50,25"); */
		strcpy(stringOut, "PSOPULSE @0 TIME 5000,2500");
		writeRead(ssId, pVar, stringOut, reply);
		strcpy(stringOut, "PSOOUTPUT @0 PULSE");
		writeRead(ssId, pVar, stringOut, reply);

		/* some controllers need to track source 3 instead of 1.  I think 3 is the interpolated analog sin/cos */
		strcpy(stringOut, "PSOTRACK @0 INPUT 1");
		/* strcpy(stringOut, "PSOTRACK @0 INPUT 3"); */
		writeRead(ssId, pVar, stringOut, reply);

		/* Not all Ensemble controllers support the "PSODISTANCE X ARRAY" command */
#if PSO_DISTANCE_ARRAY
		/* Send a PSO pulse at every trajectory point. */
		strcpy(stringOut, "PSODISTANCE @0 ARRAY"); /* use IGLOBAL array */
		writeRead(ssId, pVar, stringOut, reply);
		sprintf(stringOut, "PSOARRAY @0,%d,%d", iGlobalIndexStart, nPulses);
		writeRead(ssId, pVar, stringOut, reply);
#else
		/* Just send fixed-distance pulses.  For now, don't even worry about StartPulses, EndPulses */
		dp = (pVar->motorTrajectory[0][pVar->npoints-1] - pVar->motorTrajectory[0][0]) / pVar->npoints;
		sprintf(stringOut, "PSODISTANCE @0 FIXED %f UNITS", dp);
		writeRead(ssId, pVar, stringOut, reply);
#endif
		/* When motor is in taxi position, issue "PSOCONTROL @0 ARM" */
	}

	/* trajectory commands */
	getMotorPositions(ssId, pVar, pVar->motorCurrent);

	if ((pVar->outBitNum >= 0) && (pVar->outBitNum <= 15)) {
		strcpy(stringOut, "PSOCONTROL @0 ARM");
		writeRead(ssId, pVar, stringOut, reply);
	}

#if PSO_DISTANCE_ARRAY
	/* Program PSO array output */
	intPositionLast = 0;
	for (j=0; j<1; j++) {
		/* Calc accel portion of trajectory.  Note epicsMotorACCL is accel time. */
		accelTime = pVar->epicsMotorACCL[j];
		accelDist = (pVar->velocity[j][0] * pVar->epicsMotorACCL[j]) / 2;

		decelTime = pVar->epicsMotorACCL[j];
		decelDist = (pVar->velocity[j][pVar->npoints - 1] * pVar->epicsMotorACCL[j]) / 2;

		if (pVar->moveAxis[j]) {
			for (i=0; i<pVar->npoints; i++) {
				position = pVar->motorTrajectory[j][i] + accelDist;
				/* enable pulses iferror: two or more data types in declaration specifiers we're within user specified range */
				if ((pVar->outBitNum >= 0) && (pVar->outBitNum <= 15)) {
					if (i >= pVar->startPulses && i <= pVar->endPulses) {
						/* Add this pulse-out location to the list */
						intPosition = NINT(((position - pVar->motorStart[j]) + accelDist)/pVar->epicsMotorMres[j]);
						sprintf(stringOut, "IGLOBAL(%d) = %d", iGlobalIndex++, intPosition-intPositionLast);
						writeRead(ssId, pVar, stringOut, reply);
						intPositionLast = intPosition;
					}
				}

			}
		}
	}
#endif

	strcpy(stringOut, "TASKSTATE(5)");
	writeRead(ssId, pVar, stringOut, reply);
	status = atoi(&reply[1]);
	if ((reply[0] == '!') || ((status != 1) && (status != 2) && (status != 5))) {
		strcpy(stringOut, "PROGRAM STOP 5");
		writeRead(ssId, pVar, stringOut, reply);
		strcpy(stringOut, "TASKSTATE(5)");
		writeRead(ssId, pVar, stringOut, reply);
	}

	sprintf(stringOut, "IGLOBAL(%d)=0", cmdVar);
	writeRead(ssId, pVar, stringOut, reply);
	strcpy(stringOut, "PROGRAM RUN 5, \"doCommand.bcx\"");
	writeRead(ssId, pVar, stringOut, reply);
	epicsThreadSleep(.1);
	strcpy(stringOut, "PVT INIT TIME INC");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply);
	if (status) goto bad;
	/* Don't start until I tell you to start */
	strcpy(stringOut, "HALT");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply);
	if (status) goto bad;
	strcpy(stringOut, "VELOCITY ON");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply);
	if (status) goto bad;
	/* Program motion */
	for (j=0; j<MAX_AXES; j++) {

		/* Calc accel portion of trajectory.  Note epicsMotorACCL is accel time. */
		accelTime = pVar->epicsMotorACCL[j];
		accelDist = (pVar->velocity[j][0] * pVar->epicsMotorACCL[j]) / 2;

		decelTime = pVar->epicsMotorACCL[j];
		decelDist = (pVar->velocity[j][pVar->npoints - 1] * pVar->epicsMotorACCL[j]) / 2;

		if (pVar->moveAxis[j]) {

			sprintf(stringOut, "PVT1 %d %f, %f TIME %f", j, accelDist, pVar->velocity[j][0], accelTime);
			status = sendReceiveCommand(ssId, pVar, stringOut, reply);
			if (status) goto bad;

			for (i=1; i<pVar->npoints; i++) {
				/* Because trajectories > 14 points can't be fully preloaded, but must be loaded
				 * while they are executing, we have to handle aborts and user notification here.
				 */
				if (pVar->abortState != ABORT_STATE_NONE) goto abort;
				dp = pVar->motorTrajectory[j][i] - pVar->motorTrajectory[j][i-1];
				dt = pVar->realTimeTrajectory[i] - pVar->realTimeTrajectory[i-1];
				sprintf(stringOut, "PVT1 %d %f, %f TIME %f", j, dp, pVar->velocity[j][i], dt);
				status = sendReceiveCommand(ssId, pVar, stringOut, reply);
				if (status) goto bad;
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
			strcpy(stringOut, "VELOCITY OFF");
			status = sendReceiveCommand(ssId, pVar, stringOut, reply);
			if (status) goto bad;
			sprintf(stringOut, "PVT1 %d %f, %f TIME %f", j, decelDist, 0., decelTime);
			status = sendReceiveCommand(ssId, pVar, stringOut, reply);
			if (status) goto bad;
		}
	}
	/*epicsThreadSleep(5.);*/
	strcpy(stringOut, "START");
	/* Tell sendReceiveCommand not to wait for reply by setting reply pointer to NULL. */
	status = sendReceiveCommand(ssId, pVar, stringOut, NULL);
	if (status) goto bad;

	if (pVar->abortState != ABORT_STATE_NONE) goto abort;
	pVar->loadingTrajectory = 0;
	return(0);

bad:
	printf("loadTrajectory: error\n");
	pVar->loadingTrajectory = 0;
	/* return(-1); */
	printf("loadTrajectory: aborting because of error\n");

abort:
	printf("loadTrajectory: aborted\n");
	strcpy(stringOut, "VELOCITY OFF");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply);
	strcpy(stringOut, "STARTABORT");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply);
	writeOnly(ssId, pVar, pVar->abortCommand);
	pVar->abortState = ABORT_STATE_SENT;
	pVar->loadingTrajectory = 0;
	return(-1);
}

static int testAbort(SS_ID ssId, struct UserVar *pVar) {
	int j, n, status;
	char stringOut[MAX_MESSAGE_STRING], reply[MAX_MESSAGE_STRING], abortCmd[MAX_MESSAGE_STRING]="";

	n = sprintf(abortCmd, "ABORT");
	for (j=0; j<MAX_AXES; j++) {
		if (pVar->moveAxis[j]) {
			n += sprintf(&abortCmd[n], " @%d", j);
		}
	}

	strcpy(stringOut, "enable x");
	writeRead(ssId, pVar, stringOut, reply);
	strcpy(stringOut, "acknowledgeall");
	writeRead(ssId, pVar, stringOut, reply);

	strcpy(stringOut, "TASKSTATE(5)");
	writeRead(ssId, pVar, stringOut, reply);
	printf("loadTrajectory: TASKSTATE(5) reply='%s'\n", reply);

	sprintf(stringOut, "IGLOBAL(%d)=0", cmdVar);
	writeRead(ssId, pVar, stringOut, reply);
	strcpy(stringOut, "PROGRAM RUN 5, \"doCommand.bcx\"");
	writeRead(ssId, pVar, stringOut, reply);
	epicsThreadSleep(.1);

	strcpy(stringOut, "VELOCITY OFF");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply);
	/*strcpy(stringOut, "STARTABORT");*/
	strcpy(stringOut, "START");
	status = sendReceiveCommand(ssId, pVar, stringOut, reply);
	writeOnly(ssId, pVar, abortCmd);



	pVar->abortState = ABORT_STATE_SENT;
	return(0);
}

}%
