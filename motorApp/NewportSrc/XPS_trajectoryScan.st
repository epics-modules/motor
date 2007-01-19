program XPS_trajectoryScan("P=13BMC:,R=traj1,IPADDR=164.54.160.34,PORT=5001,"
                           "USERNAME=Administrator,PASSWORD=Administrator,"
                           "M1=m1,M2=m2,M3=m3,M4=m4,M5=m5,M6=m6,M7=m7,M8=m8,"
                           "GROUP=g1,"
                           "P1=p1,P2=p2,P3=p3,P4=p4,P5=p5,P6=p6,P7=p7,P8=p8")

/*  
 *  This sequencer program works with trajectoryScan.db.  It implements
 *  coordinated trajectory motion with the Newport XPS-C8 motor controller.
 *  It can be used with the Newport General Purpose Diffractometer or with any
 *  other set of motors connected to that controller.
 *  
 *  Original author: Jon Kelly, based on Mark Rivers' version for the MM4000.
 *  Current author:  Mark Rivers
 *
 * Modifications:
*/

%% #include <string.h>
%% #include <stdio.h>
%% #include <math.h>
%% #include "XPS_C8_drivers.h"
%% #include "Socket.h"
%% #include "xps_ftp.h"


/* This program must be compiled with the recursive option */
option +r;

/* Maximum # of trajectory elements.  The number of points XPS can accept
 * is almost unlimited because the data is stored in a file but the channel 
 * access limit with a double data type is 2000 on R3.13 clients.
 */
#define MAX_ELEMENTS 2000

/* Maximum # of output pulses.  At the moment the pulses are defined by a
 * timer which is synchronised with the trajectory points so the max number
 * of pulses equals the max number of elements.
 */
#define MAX_PULSES 2000

/* Note that MAX_ELEMENTS, and MAX_PULSES must be defined before
 * including the trajectoryScan.h */
#include "trajectoryScan.h"

/* Buffer sizes */
#define NAME_LEN 100

/* Polling interval for waiting for motors to reach their targets */
#define POLL_INTERVAL 0.1

/* Socket timeouts */
#define POLL_TIMEOUT 1.0
#define DRIVE_TIMEOUT 100000.  /* Forever */
#define ABORT_TIMEOUT 10.

/* Used within the exec state as a timeout within the while loops which wait for
 * the ss xpsTrajectoryRun to catch up */
#define COUNT_TIMEOUT  100000

/* The maximum size of the item names in gathering, e.g. "GROUP2.POSITIONER1.CurrentPosition" */
#define MAX_GATHERING_AXIS_STRING 60
/* Total length of gathering string */
#define MAX_GATHERING_STRING MAX_AXES*MAX_GATHERING_AXIS_STRING

/* Constants used for FTP to the XPS */
#define TRAJECTORY_DIRECTORY "/Admin/public/Trajectories"
#define TRAJECTORY_FILE "TrajectoryScan.trj"
#define GATHERING_DIRECTORY "/Admin/public/"
#define GATHERING_FILE "Gathering.dat"

int status;
int i;
int j;
int k;
int anyMoving;
int ncomplete;
int nextra;
int dir;
int pollSocket;
int driveSocket;
int abortSocket;
int positionSocket;
int xpsStatus;
int count;
double dtime;
double posActual;
double posTheory;
double expectedTime;
double initialPos[MAX_AXES];
double trajVel;
double preDistance[MAX_AXES];
double postDistance[MAX_AXES];
double pulseTime;
double pulsePeriod;
char groupName[NAME_LEN];
char xpsAddress[NAME_LEN];
char *axisName[MAX_AXES];
char macroBuf[NAME_LEN];
char motorName[NAME_LEN];
char userName[NAME_LEN];
char password[NAME_LEN];
int  xpsPort;

/* Define PVs */

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

%% static int getSocket(SS_ID ssId, struct UserVar *pVar, double timeout);
%% static void trajectoryExecute(SS_ID ssId, struct UserVar *pVar);
%% static void buildAndVerify(SS_ID ssId, struct UserVar *pVar);
%% static int currentElement(SS_ID ssId, struct UserVar *pVar);
%% static int getGroupStatus(SS_ID ssId, struct UserVar *pVar);
%% static void readBackError(SS_ID ssId, struct UserVar *pVar);
%% static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar);


ss xpsTrajectoryScan {

    /* Initialize things when first starting */
    state init {
        when() {
            /* Get the values from the macro parameteters */ 
            strcpy(groupName, macValueGet("GROUP"));
            strcpy(xpsAddress, macValueGet("IPADDR"));
            strcpy(userName, macValueGet("USERNAME"));
            strcpy(password, macValueGet("PASSWORD"));
            xpsPort = atoi(macValueGet("PORT"));
            for (i=0; i<numAxes; i++) {
                axisName[i] = malloc(NAME_LEN);
                sprintf(macroBuf, "P%d", i+1);
                sprintf(axisName[i], "%s.%s", groupName, macValueGet(macroBuf));
                sprintf(macroBuf, "M%d", i+1);
                sprintf(motorName, "%s%s.VAL", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorPos[i], motorName);
                sprintf(motorName, "%s%s.DIR", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorDir[i], motorName);
                sprintf(motorName, "%s%s.OFF", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorOff[i], motorName);
                sprintf(motorName, "%s%s.DMOV", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorDone[i], motorName);
            }
            xpsStatus = 0;
            ncomplete = 1;
        
            /* Set the arrays to zero */
            for (i=0; i<MAX_ELEMENTS; i++) {
                for (j=0; j<MAX_AXES; j++) {
                    motorTrajectory[j][i] = 0.0;
                }
            }

            /* Force numAxes to be <= MAX_AXES */
            if (numAxes > MAX_AXES) numAxes = MAX_AXES;
            
            /* Get sockets for communicating with XPS */
            %%pVar->pollSocket = getSocket(ssId, pVar, POLL_TIMEOUT);
            %%pVar->driveSocket = getSocket(ssId, pVar, DRIVE_TIMEOUT);
            %%pVar->abortSocket = getSocket(ssId, pVar, ABORT_TIMEOUT);
        
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

        when(efTestAndClear(executeMon) && (execute==1) 
                && (buildStatus == STATUS_SUCCESS)){
        } state execute

        when(efTestAndClear(readbackMon) && (readback==1)
                && (execStatus == STATUS_SUCCESS)) {
        } state readback

        when(efTestAndClear(nelementsMon) && (nelements>=1)) {
            /* If nelements changes, then change endPulses to this value,
             * since this is what the user normally wants.  endPulses can be
             * changed again after changing nelements if this is desired. */
            endPulses = nelements;
            pvPut(endPulses);
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
            
            /* If total time mode, calc time per element and write,
               else use the array timeTraj  */
            if (timeMode == TIME_MODE_TOTAL) {
                dtime = time/nelements;
                for (i=0; i<nelements; i++) timeTrajectory[i] = dtime;
                pvPut(timeTrajectory);
            }
        
            /* Compute expected time for trajectory & check element period */
            expectedTime=0;
            for (i=0; i<nelements; i++) {
                expectedTime += timeTrajectory[i];
            }
        
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
            /* Move to start position if required.
             * Subtract distance of initial acceleration element */
            if (moveMode == MOVE_MODE_ABSOLUTE) {
                for (j=0; j<numAxes; j++) {
                    if (moveAxis[j]) {
                        epicsMotorPos[j] = motorTrajectory[j][0] - preDistance[j];
                        pvPut(epicsMotorPos[j]);
                    }
                }
            } else {
                /* Backup by distance of initial acceleration element */
                for (j=0; j<numAxes; j++) {
                    if (moveAxis[j]) {
                        epicsMotorPos[j] = epicsMotorPos[j] - preDistance[j];
                        pvPut(epicsMotorPos[j]);
                    }
                }
            }
            %%waitEpicsMotors(ssId, pVar);
        
            /* Set status to INVALID */
            execStatus = STATUS_UNDEFINED;
            pvPut(execStatus);
            strcpy(execMessage, " ");
 
           /* Define pulse output for trajectory */
            if (npulses > 0) {
                /* Check validity, modify values if necessary */
                if (startPulses < 1) startPulses = 1;
                if (startPulses > nelements) startPulses = nelements;
                pvPut(startPulses);
                if (endPulses < startPulses) endPulses = startPulses;
                if (endPulses > nelements) endPulses = nelements;
                pvPut(endPulses);

                /* The XPS can only output pulses at a fixed period, not a fixed
                 * distance along the trajectory.  
                 * The trajectory elements where pulses start and stop are
                 * defined with the PVs startPulses and endPulses.
                 * Compute the time between pulses as the total time over which pulses 
                 * should be output divided by the number of pulses to be output. */
                pulseTime=0;
                for (i=startPulses; i<=endPulses; i++) {
                    pulseTime += timeTrajectory[i-1];
                }
                pulsePeriod = pulseTime/npulses;
            } else {
                pulsePeriod = 0.;
            }
 
            %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
            /* Setting execState here will cause the xpsTrajectoryRun SS to wake up */
            execState = EXECUTE_STATE_MOVE_START;
            pvPut(execState);
                
            count = 0;
            while (execState != EXECUTE_STATE_EXECUTING && 
                   count < COUNT_TIMEOUT ) {
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
            while (xpsStatus != 45 && 
                   count < COUNT_TIMEOUT && 
                   execStatus != STATUS_FAILURE ) {
                epicsThreadSleep(0.1);
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
            /* Get the current motor positions, post them */
            %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
            for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);

            %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
            if (xpsStatus == 45) {
                %%pVar->ncomplete = currentElement(ssId, pVar);
                sprintf(execMessage, "Executing element %d/%d", 
                        ncomplete, nelements);
                pvPut(execMessage);
            }
            else if (xpsStatus == 12) {
                /* 12 = ready from move */
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_SUCCESS;
                strcpy(execMessage, " ");
            }
            else if (xpsStatus < 10) {
                /* The xps group status reflects an error. */
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_FAILURE;
                sprintf(execMessage,"XPS Status Error=%d", xpsStatus);
            }
            /* See if the elapsed time is more than expected, time out */
            if (difftime(time(0), startTime) > (expectedTime+10)) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_TIMEOUT;
                strcpy(execMessage, "Timeout");
            }
        } state wait_execute

        when (execState==EXECUTE_STATE_FLYBACK) {
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
 * state set. */
ss xpsTrajectoryAbort {
    state monitorAbort {
        when ((efTestAndClear(abortMon)) && (abort==1) &&
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
        } state monitorAbort
    }
}

/* This state was required because the trajectory execute call does not return
 * until the end of the trajectory. So to poll during the scan we call the 
 * scan from a separate state set. */
ss xpsTrajectoryRun {
    state asyncExecute {
        when (efTestAndClear(execStateMon) && (execState == EXECUTE_STATE_MOVE_START)) {
            /*%%pVar->xpsStatus = getGroupStatus(ssId, pVar);*/
            execState = EXECUTE_STATE_EXECUTING;
            pvPut(execState);

            /* If ready to move */
            if (xpsStatus > 9 && xpsStatus < 20) {
                /* Start the detector */
                detOn = 1;
                pvPut(detOn);
                /* Get start time of execute */        
                startTime = time(0);
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
        when (delay (1) && (execState == EXECUTE_STATE_DONE)) {
            %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
            for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
        } state positionUpdate
    }
}


/*  C functions  */
%{

/* getMotorPositions returns the positions of each motor */
static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos)
{
    int status;

    /* Read the current positions of all the axes */
    
    status = GroupPositionCurrentGet(pVar->positionSocket,
                                     pVar->groupName,pVar->numAxes,pos);
    if (status != 0)
        printf("Error performing GroupPositionCurrentGet%i\n", status);
  
    return(status);
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
        printf("Error performing GroupStatusGet %i\n",status);
    
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
static int getSocket(SS_ID ssId, struct UserVar *pVar, double timeout)
{
    int sock = 0;
    
    sock = TCP_ConnectToServer(pVar->xpsAddress, pVar->xpsPort, timeout);
    if (sock < 0) 
            printf("Error TCP_ConnectToServer, status=%d\n",sock);
    return (sock);
}


/* Function to setup the gathering, events and start the trajectory */
static void trajectoryExecute(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int j;
    char buffer[MAX_GATHERING_STRING];
    int eventId;
    
    /* Configure Gathering */
    /* Reset gathering.  
     * This must be done because GatheringOneData just appends to in-memory list */    
    status = GatheringReset(pVar->pollSocket);
    if (status != 0) {
        printf("Error performing GatheringReset, status=%d\n",status);
        return;
    }

    /* Write list of gathering parameters */
    strcpy(buffer, "");
    for (j=0; j<pVar->numAxes; j++) {
        strcat (buffer, pVar->axisName[j]);
        strcat (buffer, ".SetpointPosition;");
        strcat (buffer, pVar->axisName[j]);
        strcat (buffer, ".CurrentPosition;");
        strcat (buffer, pVar->axisName[j]);
        strcat (buffer, ".CurrentVelocity;");
        if (j < (pVar->numAxes - 1)) strcat (buffer, ";");
    }
    
    /* Define what is to be saved in the GatheringExternal.dat.  
     * 3 pieces of information per axis. */
    status = GatheringConfigurationSet(pVar->pollSocket, pVar->numAxes*3, buffer);
    if (status != 0)
            printf("Error performing GatheringConfigurationSet, status=%d, buffer=%s\n",
                   status, buffer);

    /* Define trajectory output pulses. 
     * startPulses and endPulses are defined as 1=first real element, need to add
     * 1 to skip the acceleration element.  */

    status = MultipleAxesPVTPulseOutputSet(pVar->pollSocket, pVar->groupName,
                                           pVar->startPulses+1, 
                                           pVar->endPulses+1, 
                                           pVar->pulsePeriod);

    /* Define trigger */
    sprintf(buffer, "Always;%s.PVT.TrajectoryPulse", pVar->groupName);
    status = EventExtendedConfigurationTriggerSet(pVar->pollSocket, 2, buffer, 
                                                  "", "", "", "");
    if (status != 0) {
        printf("Error performing EventExtendedConfigurationTriggerSet, status=%d, buffer=%s\n", 
               status, buffer);
        return;
    }

    /* Define action */
    status = EventExtendedConfigurationActionSet(pVar->pollSocket, 1, 
                                                 "GatheringOneData", 
                                                 "", "", "", "");
    if (status != 0) {
        printf("Error performing EventExtendedConfigurationActionSet, status=%d\n",
               status);
        return;
    }

    /* Start gathering */
    status= EventExtendedStart(pVar->pollSocket, &eventId);
    if (status != 0) {
        printf("Error performing EventExtendedStart, status=%d\n",status);
        return;
    }

    status = MultipleAxesPVTExecution(pVar->driveSocket,pVar->groupName,
                                      TRAJECTORY_FILE, 1);
    /* status -27 means the trajectory was aborted */
    if ((status != 0) && (status != -27))
        printf("Error performing MultipleAxesPVTExecution, status=%d\n", status);
    
    /* Remove the event */
    status = EventExtendedRemove(pVar->pollSocket, eventId);
    if (status != 0) {
        printf("Error performing ExtendedEventRemove, status=%d\n",status);
        return;
    }
        
    /* Save the gathered data to a file */    
    status = GatheringStopAndSave(pVar->pollSocket);
    
    /* status -30 means gathering not started i.e. aborted before the end of
       1 trajectory element */
    if ((status != 0) && (status != -30))
        printf("Error performing GatheringStopAndSave, status=%d\n", status);
    
    return;
}


/* Function to build, install and verify trajectory */ 
static void buildAndVerify(SS_ID ssId, struct UserVar *pVar)
{
    FILE *trajFile;
    int i, j, status;
    double trajStep;
    double trajVel;
    double maxp;
    double minp;
    double P0, P1, T0, T1;
    int ftpSocket;
    char fileName[NAME_LEN];
    double distance;
    double preSign[MAX_AXES], postSign[MAX_AXES];
    double maxVelocity[MAX_AXES], maxAcceleration[MAX_AXES];
    double minJerkTime[MAX_AXES], maxJerkTime[MAX_AXES];
    double preTimeMax, postTimeMax;
    double preVelocity[MAX_AXES], preAccelTime[MAX_AXES];
    double postVelocity[MAX_AXES], postAccelTime[MAX_AXES];
    
    /* We create trajectories with an extra element at the beginning and at the end.
     * The distance and time of the first element is defined so that the motors will
     * accelerate from 0 to the velocity of the first "real" element at their 
     * maximum allowed acceleration.
     * Similarly, the distance and time of last element is defined so that the 
     * motors will decelerate from the velocity of the last "real" element to 0 
     * at the maximum allowed acceleration.

    /* Compute the velocity of each motor during the first real trajectory element, 
     * and the time required to reach this velocity. */
    preTimeMax = 0.;
    postTimeMax = 0.;
    for (j=0; j<pVar->numAxes; j++) {
        status = PositionerSGammaParametersGet(pVar->pollSocket, pVar->axisName[j], 
                                               &maxVelocity[j], &maxAcceleration[j],
                                               &minJerkTime[j], &maxJerkTime[j]);
        if (status != 0) {
            printf("Error calling positionerSGammaParametersSet, status=%d\n", 
                   status);
        }
        distance = pVar->motorTrajectory[j][1] - pVar->motorTrajectory[j][0];
        if (distance >= 0.) preSign[j] = 1.0; else preSign[j] = -1.0;
        preVelocity[j] = distance/pVar->timeTrajectory[0];
        preAccelTime[j] = fabs(preVelocity[j]) / maxAcceleration[j];
        if (preAccelTime[j] > preTimeMax) preTimeMax = preAccelTime[j];
        distance = pVar->motorTrajectory[j][pVar->nelements-1] - 
                   pVar->motorTrajectory[j][pVar->nelements-2];
        if (distance >= 0.) postSign[j] = 1.0; else postSign[j] = -1.0;
        postVelocity[j] = distance/pVar->timeTrajectory[pVar->nelements-1];
        postAccelTime[j] = fabs(postVelocity[j]) / maxAcceleration[j];
        if (postAccelTime[j] > postTimeMax) postTimeMax = postAccelTime[j];
    }

    /* Compute the distance that each motor moves during its acceleration phase.  i
     * Only move it this far. */
    for (j=0; j<pVar->numAxes; j++) {
        pVar->preDistance[j] = preSign[j] * (0.5 * maxAcceleration[j] * 
                                             preAccelTime[j] * preAccelTime[j]);
        pVar->postDistance[j] = postSign[j] * (0.5 * maxAcceleration[j] * 
                                             postAccelTime[j] * postAccelTime[j]);
    }

    /* Create the trajectory file */
    trajFile =  fopen (TRAJECTORY_FILE, "w");

    /* Create the initial acceleration element */
    fprintf(trajFile,"%f", preTimeMax);
    for (j=0; j<pVar->numAxes; j++) 
        fprintf(trajFile,", %f, %f", pVar->preDistance[j], preVelocity[j]);
    fprintf(trajFile,"\n");
    
    for (i=0; i<pVar->nelements-1; i++) {
        T0 = pVar->timeTrajectory[i];
        T1 = pVar->timeTrajectory[i+1];
        for (j=0; j<pVar->numAxes; j++) {
            P0 = pVar->motorTrajectory[j][i];
            P1 = pVar->motorTrajectory[j][i+1];
        
            if (pVar->moveMode == MOVE_MODE_RELATIVE) {
                trajStep = pVar->motorTrajectory[j][i];
            } else {
                trajStep = pVar->motorTrajectory[j][i+1] - 
                           pVar->motorTrajectory[j][i];
            }
                
            /* Average either side of the point? */
            trajVel = trajStep / T0;
            if (!(pVar->moveAxis[j])) {
                trajStep = 0.0;    /* Axis turned off*/
                trajVel = 0.0;
            }
        
            if (j == 0) fprintf(trajFile,"%f", pVar->timeTrajectory[i]);
            fprintf(trajFile,", %f, %f",trajStep,trajVel);
            if (j == (pVar->numAxes-1)) fprintf(trajFile,"\n");
        }    
    }

    /* Create the final acceleration element. Final velocity must be 0. */
    fprintf(trajFile,"%f", postTimeMax);
    for (j=0; j<pVar->numAxes; j++) 
        fprintf(trajFile,", %f, %f", pVar->postDistance[j], 0.);
    fprintf(trajFile,"\n");
    fclose (trajFile);
    
    /* FTP the trajectory file from the local directory to the XPS */
    status = ftpConnect(pVar->xpsAddress, pVar->userName, pVar->password, &ftpSocket);
    if (status != 0) {
        printf("Error calling ftpConnect, status=%d\n", status);
        return;
    }
    status = ftpChangeDir(ftpSocket, TRAJECTORY_DIRECTORY);
    if (status != 0) {
        printf("Error calling ftpChangeDir, status=%d\n", status);
        return;
    }
    status = ftpStoreFile(ftpSocket, TRAJECTORY_FILE);
    if (status != 0) {
        printf("Error calling ftpStoreFile, status=%d\n", status);
        return;
    }
    status = ftpDisconnect(ftpSocket);
    if (status != 0) {
        printf("Error calling ftpDisconnect, status=%d\n", status);
        return;
    }
       
    /* Verify trajectory */
    status = MultipleAxesPVTVerification(pVar->pollSocket, pVar->groupName,
                                         TRAJECTORY_FILE);
     
    pVar->buildStatus = STATUS_FAILURE;
    if (status == 0) {
        strcpy(pVar->buildMessage, " ");
        pVar->buildStatus = STATUS_SUCCESS;
    }        
    else if (status == -69)
        strcpy(pVar->buildMessage, "Acceleration Too High");
    else if (status == -68)
        strcpy(pVar->buildMessage, "Velocity Too High");
    else if (status == -70)
        strcpy(pVar->buildMessage, "Final Velocity Non Zero");         
    else if (status == -75)
        strcpy(pVar->buildMessage, "Negative or Null Delta Time");         
    else
        sprintf(pVar->buildMessage, "Unknown trajectory verify error=%d", status);  
     
    /* Read dynamic parameters*/
     
    if (status == 0){
        pVar->buildStatus = STATUS_SUCCESS;
    }    
    if (1) {  /* We may need to test for status here */
        for (j=0; j<pVar->numAxes; j++) {
            status = MultipleAxesPVTVerificationResultGet(pVar->pollSocket,
                         pVar->axisName[j], fileName, &minp, &maxp, 
                         &pVar->motorMVA[j], &pVar->motorMAA[j]);
            if (status != 0) {
                printf("Error performing MultipleAxesPVTVerificationResultGet for axis %s, status=%d\n",
                       pVar->axisName[j], status); 
            }
        }
    } else {
        for (j=0; j<pVar->numAxes; j++) {
            pVar->motorMVA[j] = 0;
            pVar->motorMAA[j] = 0;     
        }
    }
           
    return; 
}


/* Function returns the current trajectory element*/ 
static int currentElement(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int number;
    char fileName[100];
    
    status = MultipleAxesPVTParametersGet(pVar->pollSocket,
                 pVar->groupName, fileName, &number);
    if (status != 0)
        printf("Error performing MultipleAxesPVTParametersGet, status=%d\n", 
               status);
    return (number);
}

static int getGroupStatus(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int groupStatus;

    /* Read the current status of the group */

    status = GroupStatusGet(pVar->pollSocket,pVar->groupName,&groupStatus);
    if (status != 0)
        printf("Error performing GroupStatusGet, status=%d\n", status);
    return(groupStatus);
}


/* Function to load the GatheringExternal.dat file which was written
 * by the XPS after the trajectory was performed and read back the
 * actual motor positions and calculate the position errors */ 
static void readBackError(SS_ID ssId, struct UserVar *pVar)
{
    char buffer[MAX_GATHERING_STRING];
    FILE *gatheringFile;
    int i,j;
    int nitems;
    double setpointPosition, actualPosition, actualVelocity;
    int ftpSocket;
    int status;
    
    /* FTP the gathering file from the XPS to the local directory */
    status = ftpConnect(pVar->xpsAddress, pVar->userName, pVar->password, &ftpSocket);
    if (status != 0) {
        printf("Error calling ftpConnect, status=%d\n", status);
        return;
    }
    status = ftpChangeDir(ftpSocket, GATHERING_DIRECTORY);
    if (status != 0) {
        printf("Error calling ftpChangeDir, status=%d\n", status);
        return;
    }
    status = ftpRetrieveFile(ftpSocket, GATHERING_FILE);
    if (status != 0) {
        printf("Error calling ftpRetrieveFile, status=%d\n", status);
        return;
    }
    status = ftpDisconnect(ftpSocket);
    if (status != 0) {
        printf("Error calling ftpDisconnect, status=%d\n", status);
        return;
    }

    gatheringFile = fopen(GATHERING_FILE, "r");
    
    /* Read 1st 2 lines which only contain the axis names*/
    for (i=0; i<2; i++) {
        fgets (buffer, MAX_GATHERING_STRING, gatheringFile);
    }
    
    for (i=0; i<pVar->nelements; i++) {
        /* There is a bug in the current V2.0.1 firmware that puts 2 lines in the 
         * Gathering.dat file when there should be 1.  
         * Skip the first one.  
         * THIS fgets NEEDS TO BE REMOVED WHEN V2.1 FIRMWARE IS RELEASED.
         */
        fgets (buffer, MAX_GATHERING_STRING, gatheringFile);
        for (j=0; j<pVar->numAxes; j++) {
            /* Note the trailing white space in this format is required to make
             * fscanf read the newline */
            nitems = fscanf(gatheringFile, "%lf %lf %lf ", 
                            &setpointPosition, &actualPosition, &actualVelocity);
            if (nitems != 3) {
                printf("Error reading Gathering.dat file, nitems=%d, should be 3\n",
                       nitems);
                goto done;
            }
            pVar->motorReadbacks[j][i] = actualPosition;
            pVar->motorError[j][i] = actualPosition - setpointPosition;
        }
    }
    
    done:
    fclose (gatheringFile); 

    return;
}

/* Function aborts the trajectory/motion */ 
static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    
    status = GroupMoveAbort(pVar->abortSocket,pVar->groupName);
    if (status != 0)
        printf("Error performing GroupMoveAbort, status=%d\n", status);
       
    return (0);
}

}%
