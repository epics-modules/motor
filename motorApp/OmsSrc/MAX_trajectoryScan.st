program MAX_trajectoryScan("P=13IDC:,R=traj1,M1=M1,M2=M2,M3=M3,M4=M4,M5=M5,M6=M6,M7=M7,M8=M8,PORT=serial1")

/* sample program invocation:
 * dbLoadRecords("$(MOTOR)/motorApp/Db/trajectoryScan.db","P=xxx:,R=traj1:,NAXES=2,NELM=100,NPULSE=100")
 * ...
 * iocInit()
 * ...
 * seq &MAX_trajectoryScan, "P=xxx:,R=traj1:,M1=m1,M2=m2,M3=m3,M4=m4,M5=m5,M6=m6,M7=m7,M8=m8,PORT=none"
 */

/*  This sequencer program works with trajectoryScan.db.  It implements
 *  coordinated trajectory motion with an OMS (Prodex) MAXV motor controller.
 *  Eventually I hope to generalize to a MAXnet controller.  (More precisely,
 *  I hope to generalize to an asyn-based solution which could work for either.)
 *
 *  Tim Mooney -- based on MM4000_trajectoryScan.st by Mark Rivers.
 */

%% #include <string.h>
%% #include <ctype.h>
%% #include <stdio.h>
%% #include <math.h>
%% #include <epicsString.h>
%% #include <asynOctetSyncIO.h>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) > (b) ? (b) : (a))
#define NINT(f)  (int)((f)>0 ? (f)+0.5 : (f)-0.5)

#define DEBUG_VA 10
/* This program must be compiled with the recursive option */
option +r;

/* Until I get an asyn driver I can use, I'll test by writing/reading
 * directly to/from drvMaxv.cc's send_mess()/recv_mess() functions.
 */
#define USE_ASYN 0

#if USE_ASYN
#else
int cardNumber;

/* send_mess:
 *		If name is an axis name, command is prefixed by single-axis command, such as "AX ".
 *		If name is null, command is sent without modification.
 *	return value: {OK = 0, ERROR = 1}
 */
%%extern int MAXV_send_mess(int cardNumber, char const *message, char *name);

/* recv_mess:
 * amount: -1 means flush and discard; other values specify number of messages to read
 * 
 */
%%extern int MAXV_recv_mess(int cardNumber, char *message, int amount);

%%extern int MAXV_getPositions(int card, epicsInt32 *positions, int nPositions);
#endif

/* Maximum # of trajectory elements.  The MAXV allows something like 2550 for
 * a trajectory preloaded into the controller (unlimited if you're willing to
 * write elements while the trajectory is running).  For now, we limit the number
 * of elements to 1000.  This uses a lot of memory, the variable motorTrajectory
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
#include "MAX_trajectoryScan.h"

/* Maximum size of string messages we'll be sending to the MAX controller */
#define MAX_MESSAGE_STRING 100

/* Buffer sizes */
#define NAME_LEN 100

/* Maximum size of string in EPICS string PVs.  This is defined in 
 * epicsTypes.h, but in order to include that file it must be escaped, and then
 * SNL compiler gives a warning. */
#define MAX_STRING_SIZE 40

/* Polling interval in seconds for waiting for motors to reach their targets */
#define POLL_INTERVAL (1/10.)
#define READ_INTERVAL (1/60.)

char stringOut[MAX_MESSAGE_STRING];
char sbuf[MAX_MESSAGE_STRING];
char stringIn[MAX_MESSAGE_STRING];
char *asynPort;
char *pasynUser;   /* This is really asynUser* */
int status;
int i;
int j;
int k;
int n;
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
char macroBuf[NAME_LEN];
char motorName[NAME_LEN];
char *p;
char *tok_save;
int currPulse;
double frac;
double deltaV;
double v;
double vO;
int vOverride;
int vOverridePrev;
double vOverrideFactor;
double lastPollTime;
int lastRealTimePoint;
int doPoll;

/* All PVs which will be accessed in local C functions need to have their index
 * extracted with pvIndex() */
int motorCurrentIndex[MAX_AXES];
int epicsMotorDoneIndex[MAX_AXES];

/* Note, this should be time_t, but SNL doesn't understand that.  This is
 * the defininition in vxWorks. */
unsigned long startTime;  
%%epicsTimeStamp eStartTime;

/* Define escaped C functions at end of file */
%% static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command);
%% static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command, char *reply);
%% static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos, epicsInt32 *raw, double *dtime);
%% static int getMotorPositionsRB(SS_ID ssId, struct UserVar *pVar, double *pos, epicsInt32 *rawP, int *rawV, int *rawA, double *dtime);
%% static int getMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);
%%static int buildTrajectory(SS_ID ssId, struct UserVar *pVar, double *timeTrajectory,
%%	double *motorTrajectory, double epicsMotorDir, int moveMode, int npoints, int npulses, double motorResolution,
%%	int *position, int *velocity, int *acceleration);
%%static int getStarted(SS_ID ssId, struct UserVar *pVar);

/* Numerical Recipes spline routines */
%% static int spline(double *x, double *y, int n, double *y2);
%% static int splint(double *xa, double *ya, int n, double x, double *y);

int position[MAX_AXES][MAX_ELEMENTS];
int velocity[MAX_AXES][MAX_ELEMENTS];
int acceleration[MAX_AXES][MAX_ELEMENTS];
int motorStartRaw[MAX_AXES];

/*** variables for digital I/O ***/
/* detector trigger (e.g., MCS channel advance) */
/*int outBitNum; This is now a PV */
int onMask;
int offMask;
int outMask;
/* trajectory-start signal */
/*int inBitNum; This is now a PV */
/* variables for constructing trajectory commands */
int segment_accel;
int segment_decel;
int segment_v_start;
int segment_v_end;
char absRel;
int firstTask;
int movingMask;
/* variables for splitting a segment */
int p1;
int v1;
int do_split;
double t_v0;
double p1_double;
int waitingForTrigger;

ss maxTrajectoryScan {

	/* Initialize things when first starting */
	state init {
		when() {
			cardNumber = -2;
			initStatus = STATUS_UNDEFINED;
			absRel='A';
			/* Force numAxes to be <= MAX_AXES */
			if (numAxes > MAX_AXES) numAxes = MAX_AXES;
			for (i=0; i<numAxes; i++) {
				sprintf(macroBuf, "M%d", i+1);
				sprintf(motorName, "%s%s.VAL", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorPos[i], motorName);
				sprintf(motorName, "%s%s.DIR", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorDir[i], motorName);
				sprintf(motorName, "%s%s.OFF", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorOff[i], motorName);
				sprintf(motorName, "%s%s.DMOV", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorDone[i], motorName);
				sprintf(motorName, "%s%s.MRES", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorMres[i], motorName);
				sprintf(motorName, "%s%s.CARD", macValueGet("P"), macValueGet(macroBuf));
				pvAssign(epicsMotorCard[i], motorName);
				if (cardNumber == -2) {
					cardNumber = epicsMotorCard[i];
				} else {
					if (cardNumber != epicsMotorCard[i]) {
						printf("MAX_trajectoryScan: motors not on same card: %d %d\n", cardNumber, epicsMotorCard[i]);
						initStatus = STATUS_FAILURE;
					}
				}
			}
#if USE_ASYN
			asynPort = macValueGet("PORT");
			%%pVar->status = pasynOctetSyncIO->connect(pVar->asynPort, 0, (asynUser **)&pVar->pasynUser, NULL);
			if (status != 0) {
			   printf("trajectoryScan error in pasynOctetSyncIO->connect\n");
			   printf("   status=%d, port=%s\n", status, asynPort);
			}
#endif
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
			efClear(motorMDVSMon); /* we don't use this */
			if (initStatus == STATUS_UNDEFINED) initStatus = STATUS_SUCCESS;
		} state monitor_inputs
	}

	/* Monitor inputs which control what to do (Build, Execute, Read) */
	state monitor_inputs {
		when(efTestAndClear(buildMon) && (build==1) && (initStatus == STATUS_SUCCESS)) {
		} state build

		when(efTestAndClear(executeMon) && (execute==1) && (buildStatus == STATUS_SUCCESS)) {
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
			/* Initialize new trajectory */
			/* If time mode is TIME_MODE_TOTAL then construct timeTrajectory and post it */
			if (timeMode == TIME_MODE_TOTAL) {
				dtime = time/(nelements-1);
				for (i=0; i<nelements; i++) timeTrajectory[i] = dtime;
				pvPut(timeTrajectory);
			}

			if (moveMode == MOVE_MODE_RELATIVE) {
				npoints = nelements;
			} else {
				npoints = nelements-1;
			}

			/* calculate time at which motor should reach each trajectory point */
			realTimeTrajectory[0] = 0.;
			for (i=1; i<npoints; i++) {
				realTimeTrajectory[i] = realTimeTrajectory[i-1] + timeTrajectory[i];
			}
			for (i=0; i<npoints; i++) realTimeTrajectory[i] *= timeScale;

			/* For MEDM plotting */
			for (; i<MAX_ELEMENTS; i++) realTimeTrajectory[i] = realTimeTrajectory[i-1];
			pvPut(realTimeTrajectory);


			/* Calculate velocities and accelerations for trajectories. */
			for (j=0; j<MAX_AXES; j++) {
				if (moveAxis[j]) {
					%%buildTrajectory(ssId, pVar, pVar->realTimeTrajectory, pVar->motorTrajectory[pVar->j],
					%%	pVar->epicsMotorDir[pVar->j], pVar->moveMode, pVar->npoints, pVar->npulses,
					%%	pVar->epicsMotorMres[pVar->j],
					%%	pVar->position[pVar->j], pVar->velocity[pVar->j], pVar->acceleration[pVar->j]);
				}
			}

			/* Compute expected time for trajectory.  This includes timeScale factor. */
			expectedTime = realTimeTrajectory[npoints-1];

			/*** load trajectory into controller. ***/
			sprintf(stringOut, "AM;");	/* multitasking mode */
			%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);

			/* digital I/O commands */
			if (outBitNum >= 0) {
				onMask = 1<<outBitNum;
				offMask = 0;
				outMask = 1<<outBitNum;
				/*sprintf(stringOut, "IO%d,1;", outBitNum);*/	/* set bit as output */
				sprintf(stringOut, "BD%04x;", outMask);	/* set bit as output */
				%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
				sprintf(stringOut, "BL%d;", outBitNum);	/* set output bit low */
				%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
			}
			if (inBitNum >= 0) {
				sprintf(stringOut, "IO%d,0;", inBitNum);	/* set bit as input */
				%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
			}

			/* trajectory commands */
			absRel = (moveMode == MOVE_MODE_ABSOLUTE) ? 'A' : 'R';

			/* clear motor queue */
			sprintf(stringOut, "AM; SI");
			for (j=0; j<MAX_AXES; j++) {
				if (moveAxis[j]) strcat(stringOut, "1");
				if (j<(MAX_AXES-1)) strcat(stringOut, ",");
			}
			strcat(stringOut, ";");
			%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);

			/* Get update rate */
			sprintf(stringOut, "AX; #UR?;");
			%%if (pVar->simMode==0) writeRead(ssId, pVar, pVar->stringOut, pVar->stringOut);
			if (debugLevel > 0) printf("Update rate='%s'\n", stringOut);

			/* Set update rate (kludge: use npulses to specify this while I'm debugging.) */
			if ((npulses==1024) || (npulses==2048) || (npulses==4096) || (npulses==8192)) {
				sprintf(stringOut, "AX; #UR%d;", npulses);
				%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
			}

			for (j=0, firstTask=1; j<MAX_AXES; j++) {
				if (moveAxis[j]) {
					
					/* we may need current raw positions to mock up relative mode */
					%%epicsTimeGetCurrent(&eStartTime); /* not actually the start time, we just need a value */
					%%getMotorPositions(ssId, pVar, pVar->motorCurrent, pVar->motorCurrentRaw, &(pVar->dtime));

					/* output bit */
					if (firstTask && (outBitNum >= 0)) {
						/* Tell controller to output a pulse at the beginning of every trajectory segment. */
						sprintf(stringOut, "AM; VIO[%d]%04x,%04x,%04x;", j+1, onMask, offMask, outMask);
						%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
					} else {
						/* Tell controller NOT to output a pulse at the beginning of every trajectory segment. */
						sprintf(stringOut, "AM; VIO[%d];", j+1);
						%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
					}

					/* done flag and interrupt */
					sprintf(stringOut, "AM; VID[%d]1;", j+1);
					%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);

					/* Don't start until I tell you to start */
					sprintf(stringOut, "AM; VH[%d]0;", j+1);
					%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);

					/* Arm the trajectories to start on an input trigger bit.
					 * If no input trigger bit, then start now.
					 */
					if (inBitNum >= 0) {
						/* Wait for input bit to go high before processing any more commands. */
						sprintf(stringOut, "AM; SW%d;", inBitNum);
						%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
					}


					for (i=0; i<npoints; i++) {
						if (acceleration[j][i] > 0) {
							segment_accel = acceleration[j][i];
							segment_decel = acceleration[j][i];
						} else {
							segment_accel = -acceleration[j][i];
							segment_decel = -acceleration[j][i];
						}
						if (segment_accel < 1) segment_accel = 1;
						if (segment_accel > 8000000) segment_accel = 8000000;
						if (segment_decel < 1) segment_decel = 1;
						if (segment_decel > 8000000) segment_decel = 8000000;

						segment_v_start = (i==0)? velocity[j][0]:velocity[j][i-1];
						segment_v_end = velocity[j][i];

						/* If velocity goes through zero during this segment, we'll need to split the segment. */
						do_split = (segment_v_start>0) != (segment_v_end>0);
						do_split = do_split && (abs(segment_v_start)>2) && (abs(segment_v_end)>2);
						do_split = do_split && (i>0);
						if (do_split) {
							/* time from the beginning of the trajectory segment at which the velocity reaches zero.
							 * Coded in two lines because SNL is too stupid to understand a cast:
							 * t_v0 = (double)(-segment_v_start) / acceleration[j][i];
							 */
							t_v0 = -segment_v_start;
							t_v0 = t_v0/acceleration[j][i];

							if ((t_v0 < .005) || (((realTimeTrajectory[i] - realTimeTrajectory[i-1]) - t_v0) < .005)) {
								/* Don't split very near either end of segment. */
								if (debugLevel > 0) printf("declined to split segment at t=%f\n", t_v0);
								do_split = 0;
							} else {
								v1 = 0;
								p1_double = position[j][i-1] + segment_v_start*t_v0 + 0.5 * acceleration[j][i]*t_v0*t_v0;
								%% pVar->p1 = NINT(pVar->p1_double);
								if (debugLevel > 0) printf("split segment at t=%f, x=%d\n", t_v0, p1);
							}
						}

						segment_v_start = abs(segment_v_start); 
						segment_v_end = abs(segment_v_end); 

						if (segment_v_start < 1) segment_v_start = 1;
						if (segment_v_start > 4194303) segment_v_start = 4194303;
						if (segment_v_end < 0) segment_v_end = 0;
						if (segment_v_end > 4194303) segment_v_end = 4194303;

						/* BUG in MAXV: doesn't do relative trajectories right. Try to work around. */
						if (moveMode != MOVE_MODE_ABSOLUTE) position[j][i] += motorCurrentRaw[j];

						if (do_split) {
							/* we have to split this segment into two where velocity goes through zero. */
							n = sprintf(stringOut, "AM; VA[%d]%d;", j+1, segment_accel);
							n += sprintf(&stringOut[n], "VV[%d]%d,%d;", j+1, segment_v_start, v1);
							n += sprintf(&stringOut[n], "VP[%d]", j+1);
							for (k=0; k<j; k++) {strcat(stringOut, ","); n++;}
							n += sprintf(&(stringOut[n]), "%d", p1);
							for (k=j+1; k<MAX_AXES; k++) {strcat(stringOut, ","); n++;}
							strcat(stringOut, ";");
							%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);

							n = sprintf(stringOut, "AM; VA[%d]%d;", j+1, segment_accel);
							if (i<npoints-1) {
								n += sprintf(&stringOut[n], "VV[%d]%d;", j+1, segment_v_end);
							} else {
								n += sprintf(&stringOut[n], "VV[%d]%d,%d;", j+1, 1, segment_v_end);
							}
							n += sprintf(&stringOut[n], "VP[%d]", j+1);
							for (k=0; k<j; k++) {strcat(stringOut, ","); n++;}
							n += sprintf(&(stringOut[n]), "%d", position[j][i]);
							for (k=j+1; k<MAX_AXES; k++) {strcat(stringOut, ","); n++;}
							strcat(stringOut, ";");
							%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
						} else {
							n = sprintf(stringOut, "AM; VA[%d]%d;", j+1, segment_accel);
							if (i<npoints-1) {
								n += sprintf(&stringOut[n], "VV[%d]%d;", j+1, segment_v_end);
							} else {
								n += sprintf(&stringOut[n], "VV[%d]%d,%d;", j+1, segment_v_start, segment_v_end);
							}
							n += sprintf(&stringOut[n], "VP[%d]", j+1);
							for (k=0; k<j; k++) {strcat(stringOut, ","); n++;}
							n += sprintf(&(stringOut[n]), "%d", position[j][i]);
							for (k=j+1; k<MAX_AXES; k++) {strcat(stringOut, ","); n++;}
							strcat(stringOut, ";");
							%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
						}
					}
					sprintf(stringOut, "AM; VE[%d];", j+1);
					%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
					firstTask = 0;
				}
			}


			/* Set status and message string */
			buildStatus = STATUS_SUCCESS;

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
			/* Erase the readback and error arrays */
			for (j=0; j<numAxes; j++) {
				for (i=0; i<MAX_PULSES; i++) {
					motorReadbacks[j][i] = 0.;
					motorError[j][i] = 0.;
				}
			}
			currPulse = 0;
			/* Get the initial positions of the motors */
			for (j=0; j<numAxes; j++) initialPos[j] = epicsMotorPos[j];
			/* Move to start position if required */
			if (moveMode == MOVE_MODE_ABSOLUTE) {
				for (j=0; j<numAxes; j++) {
					if (moveAxis[j]) {
						epicsMotorPos[j] = motorTrajectory[j][0];
						pvPut(epicsMotorPos[j]);
					}
				}
				%%waitEpicsMotors(ssId, pVar);
			}

			%%getMotorPositions(ssId, pVar, pVar->motorCurrent, pVar->motorStartRaw, &(pVar->dtime));
			n = sprintf(stringOut, "AM;"); /* Axis multitasking mode */
			for (j=0; j<MAX_AXES; j++) {
				if (moveAxis[j]) {
					n += sprintf(&(stringOut[n]), "VO[%d]=100;", j+1); /* no velocity override */
				}
			}
			%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);

			n = sprintf(stringOut, "AM;"); /* Axis multitasking mode */
			for (j=0; j<MAX_AXES; j++) {
				if (moveAxis[j]) {
					n += sprintf(&(stringOut[n]), "VG[%d];", j+1); /* GO! */
				}
			}
			%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);

			/* Get start time of execute */
			elapsedTime = 0.;
			pvPut(elapsedTime);
			startTime = time(0);
			%%epicsTimeGetCurrent(&eStartTime);
			execState = EXECUTE_STATE_EXECUTING;
			pvPut(execState);
			lastPollTime = -POLL_INTERVAL;
			lastRealTimePoint = 0;
			/* KLUDGE: coopt variable 'accel' as test velocity-override factor */
			vOverrideFactor = accel;
			waitingForTrigger = (inBitNum >= 0);
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

		when (execState==EXECUTE_STATE_EXECUTING) {

			if (waitingForTrigger) {
				%%getMotorPositions(ssId, pVar, pVar->motorCurrent, pVar->motorCurrentRaw, &(pVar->dtime));
				for (j=0; j<numAxes; j++) {
					if (moveAxis[j] && (motorStartRaw[j] != motorCurrentRaw[j])) waitingForTrigger = 0;
				}
				if (waitingForTrigger) {
					%%pVar->waitingForTrigger = (getStarted(ssId, pVar) ? 0 : 1);
					startTime = time(0);
					%%epicsTimeGetCurrent(&eStartTime);
				}
			}
			
			if (!waitingForTrigger) {
				/* Get the current motor positions, post them */
				if (debugLevel < DEBUG_VA) {
					%%getMotorPositions(ssId, pVar, pVar->motorCurrent, pVar->motorCurrentRaw, &(pVar->dtime));
				} else {
					%%getMotorPositionsRB(ssId, pVar, pVar->motorCurrent, pVar->motorCurrentRaw, pVar->motorCurrentVRaw, pVar->motorCurrentARaw, &(pVar->dtime));
				}
				elapsedTime = dtime;
	
				doPoll = (dtime - lastPollTime) > POLL_INTERVAL;
				if (doPoll) pvPut(elapsedTime);
				for (j=0, movingMask = 0; j<numAxes; j++) {
					if (moveAxis[j]) {
						pvPut(motorCurrent[j]);
						movingMask |= (1<<j);
						/* MAXV has no readback function, so we read while it's moving. */
						if (currPulse < MAX_PULSES-1) {
							motorReadbacks[j][currPulse] = motorCurrent[j];
							motorError[j][currPulse] = dtime;
							if (debugLevel >= 10) printf("wait_execute: motor %d: rb=%f, t=%f\n",
								j, motorReadbacks[j][currPulse], motorError[j][currPulse]);
							if (j==0 && (debugLevel >= DEBUG_VA)) {
								motorReadbacks[j+1][currPulse] = motorCurrentRaw[j];
								motorReadbacks[j+2][currPulse] = motorCurrentVRaw[j];
								motorReadbacks[j+3][currPulse] = motorCurrentARaw[j];
								motorReadbacks[j+4][currPulse] = dtime;
							}
						}
						/*** compare current time, position with desired trajectory ***/
						/* bracket dtime in the interval [realTimeTrajectory[i], realTimeTrajectory[i+1]] */
						for (i=lastRealTimePoint; (i<npoints-1) && (dtime>0.) && (dtime > realTimeTrajectory[i]); i++);
						i--;
						if (i<0) i = 0;
						if (doPoll && (i > 2) && (i < npoints-2) && (vOverrideFactor >= .01)) {
							if (debugLevel >= 10) printf("wait_execute: time=%f, i=%d, realTimeTrajectory[i]=%f\n",
								dtime, i, realTimeTrajectory[i]);
							frac = (dtime - realTimeTrajectory[i]) / (realTimeTrajectory[i+1] - realTimeTrajectory[i]);
							posTheory = motorTrajectory[j][i] + frac * (motorTrajectory[j][i+1] - motorTrajectory[j][i]);
							dpos = motorCurrent[j] - posTheory;
							if (debugLevel >= 10) printf("   wait_execute: actual=%.2f, ideal=%.2f, err=%.2f\n",
								motorCurrent[j], posTheory, dpos);
							/* dp/dt */
							v = (motorReadbacks[j][currPulse] - motorReadbacks[j][currPulse-1]) /
								(motorError[j][currPulse] - motorError[j][currPulse-1]);
							/* change in speed needed to make up the position in a time equal to the length of the current
							 * segment */
							deltaV = dpos / (realTimeTrajectory[i+1] - realTimeTrajectory[i]);
							vO = (1-(deltaV/v)*vOverrideFactor) * 100;
							%%pVar->vOverride = NINT(pVar->vO);
							if (vOverride<80) vOverride=80;
							if (vOverride>120) vOverride=120;
							if (debugLevel >= 10) printf("   wait_execute: v=%.2f, dV=%.2f, vOverride=%.2f (%d)\n",
								v, deltaV, vO, vOverride);
							/* legal range of vOverride is [0, 200] */
							sprintf(stringOut, "AM; VO[%d]=%d;", j+1, vOverride);
							%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
							if (debugLevel >= 2) printf(", 'VO[%d]=%3d'", j+1, vOverride);
						}
					}
	
				}
				++currPulse;
				lastRealTimePoint = i;
				if (doPoll) {
					lastPollTime = dtime;
					%%pVar->anyMoving = getMotorMoving(ssId, pVar);
					if (debugLevel >= 10) printf("movingMask=0x%x, anyMoving=0x%x\n", movingMask, anyMoving);
					if (!(anyMoving&movingMask)) {
						execState = EXECUTE_STATE_FLYBACK;
						execStatus = STATUS_SUCCESS;
						strcpy(execMessage, " ");
					}
					/* See if the elapsed time is more than twice expected, time out */
					if (difftime(time(0), startTime) > expectedTime*2.) {
						execState = EXECUTE_STATE_FLYBACK;
						execStatus = STATUS_TIMEOUT;
						strcpy(execMessage, "Timeout");
	
						for (j=0; j<MAX_AXES; j++) {
							if (moveAxis[j]) {
								sprintf(stringOut, "AM; VH[%d]1;", j+1);
								%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
							}
						}
						/* SHOULD PROBABLY WAIT FOR MOTORS TO DECELERATE TO A STOP BEFORE KILLING */
						/* kill selected axes and flush queues */
						sprintf(stringOut, "KS");
						for (j=0; j<MAX_AXES; j++) {
							if (moveAxis[j]) strcat(stringOut, "1");
							if (j<(MAX_AXES-1)) strcat(stringOut, ",");
						}
						strcat(stringOut, ";");
						if (debugLevel) printf("timeout: sending command '%s'\n", stringOut);
						%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
	
						%%waitEpicsMotors(ssId, pVar); /* wait until all motors are done */
					}
				}
				/* Check for errors while trajectories are in progress */
			} /* if (!waitingForTrigger) */
		} state wait_execute

		when (execState==EXECUTE_STATE_FLYBACK) {
			if (debugLevel) printf("\nflyback...");
			pvPut(elapsedTime);
			pvPut(execState);
			pvPut(execStatus);
			pvPut(execMessage);
			/* Get the current motor positions, post them */
			%%getMotorPositions(ssId, pVar, pVar->motorCurrent, pVar->motorCurrentRaw, &(pVar->dtime));
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
#if 1
			/* During trajectory execution, time and motor position were accumulated into
			 * motorError[j] and motorReadbacks[j], respectively.  Interpolate motorReadbacks[j]
			 * to get readbacks at the times implied by realTimeTrajectory, so they can be plotted
			 * on the same axis with motorTrajectory[j].  This algorithm converts in place, which
			 * assumes we have more readbacks than trajectory segments ((i-1)>k).
			 */
            for (j=0; j<numAxes; j++) {
				if (debugLevel >= 10) printf("state readback: motor %d\n", j);
				for (k=0, i=0; k<npoints; k++) {
					while ((motorError[j][i] < realTimeTrajectory[k]) && (i < MAX_PULSES-1)) i++;
					if ((i>0) && (fabs(motorError[j][i] - motorError[j][i-1]) > 1e-6)) {
						frac = (realTimeTrajectory[k] - motorError[j][i-1])/(motorError[j][i] - motorError[j][i-1]);
						motorReadbacks[j][k] = motorReadbacks[j][i-1] + frac * (motorReadbacks[j][i] - motorReadbacks[j][i-1]);
					} else {
						motorReadbacks[j][k] = motorReadbacks[j][i];
					}
					if (debugLevel >= 10) printf("state readback: rb=%f, t=%f\n", motorReadbacks[j][k], realTimeTrajectory[k]);

				}
				for (; k<MAX_PULSES; k++) motorReadbacks[j][k] = 0.;
				/* calculate error, ignoring last (deceleration) point */
				for (k=0; k<npoints-1; k++) {
					motorError[j][k] =  motorReadbacks[j][k] - motorTrajectory[j][k];
				}
				for (; k<MAX_PULSES; k++) motorError[j][k] = motorError[j][k-1];
			}
			for (k=currPulse; k<MAX_PULSES; k++){
				motorReadbacks[1][k] = motorReadbacks[1][k-1];
				motorReadbacks[2][k] = motorReadbacks[2][k-1];
				motorReadbacks[3][k] = motorReadbacks[3][k-1];
				motorReadbacks[4][k] = motorReadbacks[4][k-1];
			}
#endif
            /* Post the readback and error arrays */
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
 * state set. If an abort is received it sends the "KS" command to the MAX controller, 
 * sets the execStatus to STATUS_ABORT and writes a message to execMessage */
ss trajectoryAbort {
	state monitorAbort {
		when (efTestAndClear(abortMon) && (abort==1)) {
#if 0
			sprintf(stringOut, "SA;"); /* Stop all motors, and flush all queues. */
			%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
#else
			for (j=0; j<MAX_AXES; j++) {
				if (moveAxis[j]) {
					sprintf(stringOut, "AM; VH[%d]1;", j+1);
					%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
				}
			}
			/* SHOULD PROBABLY WAIT FOR MOTORS TO DECELERATE TO A STOP BEFORE KILLING */
			/* kill selected axes and flush queues */
			sprintf(stringOut, "KS");
			for (j=0; j<MAX_AXES; j++) {
				if (moveAxis[j]) strcat(stringOut, "1");
				if (j<(MAX_AXES-1)) strcat(stringOut, ",");
			}
			strcat(stringOut, ";");
			if (debugLevel) printf("abort: sending command '%s'\n", stringOut);
			%%if (pVar->simMode==0) writeOnly(ssId, pVar, pVar->stringOut);
#endif

			execStatus = STATUS_ABORT;
			pvPut(execStatus);
			strcpy(execMessage, "Motion aborted");
			pvPut(execMessage);
			pvPut(elapsedTime);
			/* Clear abort command, post.  This is a "busy" record, don't
			 * want to do this until abort command has been sent. */
			abort=0;
			pvPut(abort);
		} state monitorAbort
	}
}

/* C functions */
%{

/* writeOnly sends a command to the MAX controller */
static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command)
{
	asynStatus status;
	int debug_out=0;
#if USE_ASYN
	size_t nwrite;
	char buffer[MAX_MESSAGE_STRING];

	/* Copy command so we can add terminator */
	strncpy(buffer, command, MAX_MESSAGE_STRING-3);
	strcat(buffer, "\r");
	status = pasynOctetSyncIO->write((asynUser *)pVar->pasynUser, buffer,
							   strlen(buffer), 1.0, &nwrite);
#else
	status = (asynStatus) MAXV_send_mess(pVar->cardNumber, command, (char *) NULL);
#endif
	if (pVar->execState==EXECUTE_STATE_EXECUTING)
		debug_out = (pVar->debugLevel >= 7);
	else
		debug_out = (pVar->debugLevel >= 2);
	if (debug_out) printf("    writeOnly:command='%s'\n", command);
	return(status);
}


/* writeRead sends a command to the MAX controller and reads the response into
 * the global character buffer, stringIn.
 */
static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command, char *reply)
{
	asynStatus status;
#if USE_ASYN
	size_t nwrite, nread;
	int eomReason;
	char buffer[MAX_MESSAGE_STRING];

	/* Copy command so we can add terminator */
	strncpy(buffer, command, MAX_MESSAGE_STRING-3);
	strcat(buffer, "\r");
	/* Use 30 second timeout, some commands take a long time to reply */
	status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, buffer,
							   strlen(buffer), reply, MAX_MESSAGE_STRING, 
							   30.0, &nwrite, &nread, &eomReason);
#else
	status = (asynStatus) MAXV_send_mess(pVar->cardNumber, command, (char *) NULL);
	status |= (asynStatus) MAXV_recv_mess(pVar->cardNumber, reply, 1);
#endif
	if (pVar->debugLevel >= 10) {
		printf("    writeRead:command='%s', reply='%s'\n", command, reply);
	}
	return(status);
}

/* getMotorPositions returns the positions of each motor */
static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos, epicsInt32 *rawP, double *dt)
{
    int j;
    int dir;
	epicsTimeStamp currtime;
	char vBuf[MAX_MESSAGE_STRING], aBuf[MAX_MESSAGE_STRING];

	MAXV_getPositions(pVar->cardNumber, rawP, pVar->numAxes);

	epicsTimeGetCurrent(&currtime);
	*dt = epicsTimeDiffInSeconds(&currtime, &eStartTime);
    for (j=0; j<pVar->numAxes; j++) {
        if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1;
        pos[j] = rawP[j]*dir*pVar->epicsMotorMres[j] + pVar->epicsMotorOff[j];
    }

	if ((pVar->execState == EXECUTE_STATE_EXECUTING) && (pVar->debugLevel >= 2)) {
		writeRead(ssId, pVar, "VRV[1];", vBuf);
		writeRead(ssId, pVar, "VRC[1];", aBuf);
		printf("\ndt=%6.3f, p=%7d, v=%7.0f, a=%7.0f",
			*dt, rawP[0], atof(&(vBuf[1])), atof(&(aBuf[1])));
		if (pVar->debugLevel >= 10) printf("\n");
	} else if (pVar->debugLevel >= 1) {
		printf("\ndt=%6.3f, p=%7d", *dt, rawP[0]);
	}
	epicsThreadSleep(READ_INTERVAL);
	return(0);
}

/* getMotorPositions returns the positions of each motor, and maybe velocity and acceleration */
static int getMotorPositionsRB(SS_ID ssId, struct UserVar *pVar, double *pos, epicsInt32 *rawP, int *rawV, int *rawA, double *dt)
{
    int j;
    int dir;
	epicsTimeStamp currtime;
	char vBuf[MAX_MESSAGE_STRING], aBuf[MAX_MESSAGE_STRING];

	MAXV_getPositions(pVar->cardNumber, rawP, pVar->numAxes);

	epicsTimeGetCurrent(&currtime);
	*dt = epicsTimeDiffInSeconds(&currtime, &eStartTime);

    for (j=0; j<pVar->numAxes; j++) {
        if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1;
        pos[j] = rawP[j]*dir*pVar->epicsMotorMres[j] + pVar->epicsMotorOff[j];
    }

    /* Read the velocity and acceleration for task 1 */
	if ((pVar->execState == EXECUTE_STATE_EXECUTING) && (pVar->debugLevel >= DEBUG_VA)) {
		writeRead(ssId, pVar, "VRV[1];", vBuf);
		writeRead(ssId, pVar, "VRC[1];", aBuf);
		rawV[0] = atol(&(vBuf[1]));
		rawA[0] = atol(&(aBuf[1]));
		printf("getMotorPositionsRB: dt=%6.3f, p=%7d, v=%7d, a=%7d\n", *dt, rawP[0], rawV[0], rawA[0]);
		if (pVar->debugLevel >= 10) printf("\n");
	} else if (pVar->debugLevel >= 1) {
		printf("getMotorPositionsRB: dt=%6.3f, p=%7d\n", *dt, rawP[0]);
	}
	epicsThreadSleep(READ_INTERVAL);

	return(0);
}


/* getMotorMoving returns the moving status of each motor, packed into a single
 * int.  Bit 0 = motor 1, bit 1 = motor 2, etc. 0=not moving, 1=moving.
 * If the entire int is 0 then no motors are moving */
static int getMotorMoving(SS_ID ssId, struct UserVar *pVar)
{
	int i, mask=1, result=0;
	char s[MAX_MESSAGE_STRING];

	for (i=0; i==0;) {
		/* Read the current status of all the axes */
		writeRead(ssId, pVar, "QI", pVar->stringIn);
		strcpy(s, pVar->stringIn);
		writeRead(ssId, pVar, "QI", pVar->stringIn);
		if (strcmp(s, pVar->stringIn) != 0) {
			if (pVar->debugLevel >= 10) {
				printf("getMotorMoving: inconsistent replies:\n");
				printf("r1:'%s', r2:'%s'\n", s, pVar->stringIn);
			}
		} else {
			i = 1;
		}
	}
	/* Parse the return string which is of the form 
	 * MDNN,MDNN,PNLN,PNNN,PNLN,PNNN,PNNN,PNNN,<LF>
	 * The second character of each status word is 'D' (done) or 'N' (not done)
	 */

	for (i=1; i<37; i+=5, mask<<=1) {
		if (pVar->stringIn[i] == 'N') result |= mask;
	}

	pVar->stringIn[40] = '\0';
	if (pVar->debugLevel >= 10) {
		printf("getMotorMoving: reply = '%s', movingMask = %2x\n", pVar->stringIn, result);
	}
	return(result);
}

static int getStarted(SS_ID ssId, struct UserVar *pVar) {
	int i, bits, mask;
	char c;

	writeRead(ssId, pVar, "BX", pVar->stringIn);
	for (i=0, bits=0; i<4; i++) {
		bits <<= 4;
		c = pVar->stringIn[i];
		/* convert hex character to number */
		if (isdigit((int)c)) {
			c = c - '0';
		} else if (isxdigit((int)c)) {
			if (islower((int)c)) {
				c = (c - 'a') + 10;
			} else {
				c = (c - 'A') + 10;
			}
		}
		bits |= c;
	}
	mask = 1 << (pVar->inBitNum);
	if (pVar->debugLevel >= 5)
		printf("\ngetStarted: reply='%s', bits=0x%x, mask=0x%x",  pVar->stringIn, bits, mask);
	return (bits & mask);
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

/* Calculate velocities and accelerations suitable for MAX variable velocity contouring commands.
 * We're given x(t) in the form x[i], t[i].  We need to calculate v(x) and a(x) that will produce x(t).
 */

double	y2[MAX_ELEMENTS], v_out[MAX_ELEMENTS], a_out[MAX_ELEMENTS], calcMotorTrajectory[MAX_ELEMENTS];
static int buildTrajectory(SS_ID ssId, struct UserVar *pVar, double *realTimeTrajectory,
	double *motorTrajectory, double epicsMotorDir, int moveMode, int npoints, int npulses, double motorResolution,
	int *position, int *velocity, int *acceleration)
{
	double dp, dt, v_ideal, v_lin, v_spline, accel_p, accel_v, time;
	double x0;
	double delta, yy0, yy1;
	int i, np;

	spline(realTimeTrajectory, motorTrajectory, npoints, y2);

	calcMotorTrajectory[0] = motorTrajectory[0];
	v_out[0] = 0;
	if (pVar->debugLevel >= 7) {
		printf("###:%8s %8s %7s %8s %8s %8s %8s %8s\n",
			"pos", "calcPos", "dp", "t", "v_ideal", "accel_p", "accel_v", "accel_s");
	}
	for (i=1; i<npoints; i++) {
		/*dp = motorTrajectory[i]-motorTrajectory[i-1];*/
		/* Don't assume we achieved exactly the desired [i-1] position. */
		dp = motorTrajectory[i]-calcMotorTrajectory[i-1];
		dt = realTimeTrajectory[i]-realTimeTrajectory[i-1];
		/* the acceleration that will get us to the desired position */
		accel_p = 2*(dp - v_out[i-1]*dt)/(dt*dt);
		if (i < npoints-1) {
			/* the ideal velocity at motorTrajectory[i] */
			/* linear interpolation */
			v_lin = (motorTrajectory[i+1]-motorTrajectory[i-1])/(realTimeTrajectory[i+1]-realTimeTrajectory[i-1]);

			/* spline calculation */
			delta = (realTimeTrajectory[i+1] - realTimeTrajectory[i-1])/10.;
			splint(realTimeTrajectory, motorTrajectory, npoints, realTimeTrajectory[i]-delta, &yy0);
			splint(realTimeTrajectory, motorTrajectory, npoints, realTimeTrajectory[i]+delta, &yy1);
			v_spline = (yy1-yy0)/(2*delta);

			if (pVar->debugLevel >= 10) printf("v_lin=%f, v_spline=%f\n", v_lin, v_spline);

			/* the acceleration that will get us to the ideal velocity */
			if (1) {
				v_ideal = v_spline;
			} else {
				v_ideal = v_lin;
			}
			accel_v = (v_ideal - v_out[i-1])/dt;

			/* compromise between desired position and ideal velocity */
			if ((pVar->endPulses != 0) && (i > 2)) {
				np = abs(pVar->endPulses);
				if (pVar->endPulses > 0) {
					if (pVar->endPulses == 999) {
						a_out[i-1] = (y2[i-1]+y2[i])/2;
					} else if (pVar->endPulses == 888) {
						a_out[i-1] = (accel_p + accel_v + (y2[i-1]+y2[i])/2)/3;
					} else {
						a_out[i-1] = (np*accel_p + accel_v)/(np+1);
					}
				} else {
					a_out[i-1] = (accel_p + np*accel_v)/(np+1);
				}
			} else {
				a_out[i-1] = (accel_p + accel_v)/2;
			}
		} else {
			v_ideal = 0.;
			accel_v = (v_ideal - v_out[i-1])/dt;
			a_out[i-1] = accel_p;
		}
		if (pVar->debugLevel >= 7) {
			printf("%3d:%8.2f %8.2f %7.2f %8.3f %8.3f %8.3f %8.3f %8.3f\n",
				i, motorTrajectory[i-1], calcMotorTrajectory[i-1], dp, realTimeTrajectory[i-1], v_ideal, accel_p, accel_v, (y2[i-1]+y2[i])/2);
		}
		v_out[i] = v_out[i-1] + a_out[i-1]*dt;
		calcMotorTrajectory[i] = calcMotorTrajectory[i-1] + v_out[i-1]*dt + .5 * a_out[i-1]*dt*dt;
	}
	a_out[npoints-1] = a_out[npoints-2];
	
	if (pVar->debugLevel >= 7) {
		printf("buildTrajectory:\n");
		printf("%10s %10s %10s %10s %10s\n", "realTime", "motorTraj", "calcTraj", "v_out", "a_out");
		for (i=0; i<npoints; i++) {
			printf("%10.2f %10.5f %10.5f %10.5f %10.5f\n",
				realTimeTrajectory[i], motorTrajectory[i], calcMotorTrajectory[i], v_out[i], a_out[i]);
		}
	}

	/* Translate into MAX commands */
	v_out[0] = 0;
	if (pVar->debugLevel >= 1) {
		printf("motor resolution %f\n", motorResolution);
		printf("%10s %10s %10s %10s %10s\n", "time", "position", "calcpos", "velocity", "acceleration");
	}
	for (i=0; i<npoints; i++) {
		if (i < npoints-1) {
			time = realTimeTrajectory[i+1];
			dt = realTimeTrajectory[i+1] - realTimeTrajectory[i];
			position[i] = NINT(calcMotorTrajectory[i+1]/motorResolution);
			velocity[i] = NINT(v_out[i+1]/motorResolution);
			acceleration[i] = NINT(a_out[i]/motorResolution);
		} else {
			time = realTimeTrajectory[i];
			dt = realTimeTrajectory[i] - realTimeTrajectory[i-1];
			position[i] = NINT(calcMotorTrajectory[i]/motorResolution);
			velocity[i] = 0;
			acceleration[i] = NINT(a_out[i]/motorResolution);
		}
		if (i>0) {
			x0 = position[i-1] + velocity[i-1]*dt + .5 * acceleration[i]*dt*dt;
		} else {
			x0 = .5 * acceleration[i]*dt*dt;
		}
		if (pVar->debugLevel >= 1) printf("%10.2f %10d %10d %10d %10d\n", time, position[i], NINT(x0), velocity[i], acceleration[i]);
	}

	return(0);
}

/* Numerical recipes spline routines */
double u[MAX_ELEMENTS+1];

static int spline(double *x, double *y, int n, double *y2)
{
	int i, k;
	double p, qn, sig, un;

	/* convert from c array to fortran array */
	x--; y--; y2--;

	y2[1] = u[1] = 0.0;
	for (i=2; i<=n-1; i++) {
		sig = (x[i]-x[i-1])/(x[i+1]-x[i-1]);
		p = sig*y2[i-1]+2.0;
		y2[i] = (sig-1.0)/p;
		u[i] = (y[i+1]-y[i])/(x[i+1]-x[i]) - (y[i]-y[i-1])/(x[i]-x[i-1]);
		u[i] = (6.0*u[i]/(x[i+1]-x[i-1])-sig*u[i-1])/p;
	}
	qn = un = 0.0;
	y2[n] = (un-qn*u[n-1])/(qn*y2[n-1]+1.0);
	for (k=n-1; k>=1; k--)
		y2[k] = y2[k]*y2[k+1]+u[k];
	return(0);
}

static int splint(double *xa, double *ya, int n, double x, double *y)
{
	int klo,khi,k;
	double h,b,a;

	/* convert from c array to fortran array */
	xa--; ya--;

	klo = 1;
	khi = n;
	while (khi-klo > 1) {
		k = (khi+klo) >> 1;
		if (xa[k] > x) khi = k;
		else klo = k;
	}
	h = xa[khi]-xa[klo];
	if (h == 0.0) {
		printf("Bad XA input to routine SPLINT");
		return(-1);
	}
	a = (xa[khi]-x)/h;
	b = (x-xa[klo])/h;
	*y = a*ya[klo]+b*ya[khi]+((a*a*a-a)*y2[klo]+(b*b*b-b)*y2[khi])*(h*h)/6.0;
	return(0);
}

}%
