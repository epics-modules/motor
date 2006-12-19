/* Prog to test the XPS position gathering and trigger during a trajectory scan */
/**/
#include <stdio.h>
#include <time.h>
#include "XPS_C8_drivers.h"
#include "Socket.h"
#include "xps_ftp.h"

#define XPS_ADDRESS "164.54.160.34"
#define NUM_ELEMENTS 10
#define NUM_AXES 2
#define PULSE_TIME 1.0
#define POLL_TIMEOUT 1.0
#define DRIVE_TIMEOUT 100.0
#define USERNAME "Administrator"
#define PASSWORD "Administrator"
#define TRAJECTORY_DIRECTORY "/Admin/public/Trajectories"
#define TRAJECTORY_FILE "test_trajectory"
#define GATHERING_DIRECTORY "/Admin/public/"
#define GATHERING_FILE "Gathering.dat"

void xps_gathering()
{
    int status,poll_socket,drive_socket,end=0;
    int ftpSocket;
    char *gatheringData = "GROUP2.POSITIONER1.CurrentPosition;GROUP2.POSITIONER2.CurrentPosition";
            
                        
    char *positioner[NUM_AXES] = {"GROUP2.POSITIONER1", "GROUP2.POSITIONER2"};
    char group[] = "GROUP2";
    char outputfilename[500];
    double maxp,minp,maxv,maxa;
    char buffer[100];
    double motorReadbacks[NUM_AXES][NUM_ELEMENTS+1];
    double motorError[NUM_AXES][NUM_ELEMENTS+1];
    double trajTime = 0,trajStep = 0;
    int i,j;
    int groupStatus;
    double posTheory[NUM_AXES];
    FILE *gatheringFile, *trajFile;
    int eventID;
    time_t start_time, end_time;

    poll_socket  = TCP_ConnectToServer(XPS_ADDRESS, 5001, POLL_TIMEOUT);
    drive_socket = TCP_ConnectToServer(XPS_ADDRESS, 5001, DRIVE_TIMEOUT);

    status = GroupStatusGet(poll_socket, group, &groupStatus);
    printf("Initial group status=%d\n", groupStatus);
    /* If group not initialized, then initialize it */
    if (groupStatus >= 0 && groupStatus <= 9) {
        printf("Calling GroupInitialize ...\n");
        status = GroupInitialize(drive_socket, group);
        if (status) {
            printf("Error calling GroupInitialize error=%d\n", status);
            return;
        }
        printf("Calling GroupHomeSearch ...\n");
        status = GroupHomeSearch(drive_socket, group);
        if (status) {
            printf("Error calling GroupHomeSearch error=%d\n", status);
            return;
        }
    }

    printf("FTPing trajectory file to XPS ...\n");
    /* FTP the trajectory file from the local directory to the XPS */
    status = ftpConnect(XPS_ADDRESS, USERNAME, PASSWORD, &ftpSocket);
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
   
    /* Define trajectory output pulses */
     printf("Defining output pulses ...\n");
    status = MultipleAxesPVTPulseOutputSet(poll_socket, group, 1, NUM_ELEMENTS-1, PULSE_TIME);
 
    /*************************** Verify trajectory **********************/
    printf("Verifying trajectory ...\n");
    status = MultipleAxesPVTVerification(drive_socket, group, TRAJECTORY_FILE);
    if (status != 0) {
        printf(" Error performing MultipleAxesPVTVerification, status=%d\n",status);
        end = 1;
    }    

    printf("Reading verify results ...\n");
    printf(" MultipleAxesPVTVerificationResultGet\n");                
    for (i=0; i<NUM_AXES; i++) {
        status = MultipleAxesPVTVerificationResultGet(poll_socket,positioner[i], outputfilename,
                        &minp, &maxp, &maxv, &maxa);
        printf(" positioner 1%d, status %d, Max. pos. %g, Min. pos. %g, Max. vel. %g, Max. accel. %g\n",
                i, status, maxp, minp, maxv, maxa);
    }

    if (end == 1) return;    
 
   /***********************Configure Gathering and Timer******************/


    printf("Reseting gathering ...\n");
    status = GatheringReset(poll_socket);
    if (status != 0) {
        printf(" Error performing GatheringReset, status=%d\n",status);
        return;
    }

    printf("Defining gathering ...\n");
    status = GatheringConfigurationSet(poll_socket, 2, gatheringData);
    if (status != 0) {
        printf(" Error performing GatheringConfigurationSet, status=%d\n",status);
        return;
    }

/*
    printf("Gathering one data set manually ...\n");
    status = GatheringDataAcquire(poll_socket);
    if (status != 0) {
        printf(" Error performing GatheringDataAcquire, status=%d\n",status);
        return;
    }
*/
    printf("Defining trigger ...\n");
    status = EventExtendedConfigurationTriggerSet(poll_socket, 2, 
                                                  "Always;GROUP2.PVT.TrajectoryPulse",
                                                  "","","","");
    if (status != 0) {
        printf(" Error performing EventExtendedConfigurationTriggerSet, status=%d\n",status);
        return;
    }

    printf("Defining action ...\n");
    status = EventExtendedConfigurationActionSet(poll_socket, 1, "GatheringOneData","","","","");
    if (status != 0) {
        printf(" Error performing EventExtendedConfigurationActionSet, status=%d\n",status);
        return;
    }

    printf("Starting gathering ...\n");
    status= EventExtendedStart(poll_socket, &eventID);
    if (status != 0) {
        printf(" Error performing EventExtendedStart, status=%d\n",status);
        return;
    }


    /********************* Run traj ****************************/
    status = GroupStatusGet(poll_socket, group, &groupStatus);
    if (status != 0) {
        printf(" Error performing GroupStatusGet, status=%d\n",status);
        return;
    }
    printf("Group status before executing trajectory=%d\n", groupStatus);
    printf("Executing trajectory ...\n");
    start_time = time(NULL);
    status = MultipleAxesPVTExecution(drive_socket, group, TRAJECTORY_FILE, 1);
    end_time = time(NULL);
    printf("Time to execute trajectory=%f\n", difftime(end_time, start_time));
    if (status != 0) {
        printf(" Error performing MultipleAxesPVTExecution, status=%d\n",status);    
        return;
    }
    status = GroupStatusGet(poll_socket, group, &groupStatus);
    if (status != 0) {
        printf(" Error performing GroupStatusGet, status=%d\n",status);
        return;
    }
    printf("Group status after executing trajectory=%d\n", groupStatus);

    /* Remove the event */
    printf("Removing event ...\n");
    status = EventExtendedRemove(poll_socket, eventID);
    if (status != 0) {
        printf(" Error performing ExtendedEventRemove, status=%d\n",status);
        return;
    }

    /***********************Save the gathered data ***************/
    printf("Stopping gathering ...\n");
    start_time = time(NULL);
    status = GatheringStopAndSave(drive_socket);
    end_time = time(NULL);
    printf("Time to save gathering data=%f\n", difftime(end_time, start_time));
    if (status != 0) {
        printf(" Error performing GatheringExternalStopAndSave, status=%d\n",status);
        return;
    }
    status = GroupStatusGet(poll_socket, group, &groupStatus);
    if (status != 0) {
        printf(" Error performing GroupStatusGet, status=%d\n",status);
        return;
    }
    printf("Group status after stopping gathering=%d\n", groupStatus);

    /* FTP the gathering file from the XPS to the local directory */
    printf("FTPing gathering file from XPS ...\n");
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
   
    printf("Returning before processing gathering file for now\n");
    return;
 
    printf("Opening gathering file ...\n");
    gatheringFile = fopen("GATHERING_FILE", "r");
    printf("Opening trajectory file ...\n");
    trajFile = fopen("TRAJECTORY_FILE", "r");

    for (i=0; i<NUM_ELEMENTS; ++i) {
        for (j=0; j<NUM_AXES; ++j) {
            motorReadbacks[j][i] = 0;
            motorError[j][i] = 0;
        }
    }

    /* Read 1st 2 lines */
    for (i=0; i<2; i++) {
        fgets (buffer, 1000, gatheringFile);
        printf("Line %d of GatheringEx = %s\n",i,buffer);
    }
    for (i=0; i<NUM_ELEMENTS; i++) {
        for (j=0; j<NUM_AXES; j++) {
            fscanf(gatheringFile,"%lf",&motorReadbacks[j][i]);
             
            if( j == (NUM_AXES-1)) {
                if (fscanf(trajFile," %lf,%*f", &trajStep) != 1) printf("trajerror\n");
            }
         
            if (j == 0) {   
                if (fscanf(trajFile,"%lf,%lf,%*f,", &trajTime, &trajStep) != 2) printf("trajerror\n");
            }
        
            if (j > 0 && j < (NUM_AXES-1)) {
                if (fscanf(trajFile," %lf,%*f,", &trajStep) != 1) printf("trajerror\n");
            }
        
            if (i == 0) {
                posTheory[j] = motorReadbacks[j][i];
                printf("posTheory = %f ",posTheory[j]);
            }
            else {
                posTheory[j]+= trajStep;
            }
             
            motorError[j][i] = posTheory[j] - motorReadbacks[j][i];
            printf("i=%d J=%d ReadBack=%f    motorError=%f trajStep%f\n",
                    i,j,motorReadbacks[j][i],motorError[j][i],trajStep);
        }
    }
    
    fclose (trajFile);
    fclose (gatheringFile);
        
    return;
}

