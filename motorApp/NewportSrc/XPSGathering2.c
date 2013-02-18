/* Program to test the XPS position gathering and trigger during a trajectory scan */
/**/
#include <stdio.h>
#include <time.h>
#include "XPS_C8_drivers.h"
#include "Socket.h"
#include "xps_ftp.h"
#define epicsExportSharedSymbols
#include <shareLib.h>

#define XPS_ADDRESS "164.54.160.124"
#define NUM_TRAJECTORY_ELEMENTS 6
#define NUM_GATHERING_POINTS 100
#define NUM_GATHERING_ITEMS 2
#define NUM_AXES 6
#define PULSE_TIME 0.01
#define POLL_TIMEOUT 1.0
#define DRIVE_TIMEOUT 100.0
#define USERNAME "Administrator"
#define PASSWORD "Administrator"
#define TRAJECTORY_DIRECTORY "/Admin/public/Trajectories"
#define TRAJECTORY_FILE "TrajectoryScan.trj"
#define BUFFER_SIZE 32767

int main(int argc, char *argv[])
{
    int status,poll_socket,drive_socket,end=0;
    SOCKET ftpSocket;
    char *gatheringData = "GROUP1.PHI.SetpointPosition;"
                          "GROUP1.PHI.CurrentPosition;"
                          "GROUP1.KAPPA.SetpointPosition;"
                          "GROUP1.KAPPA.CurrentPosition;"
                          "GROUP1.OMEGA.SetpointPosition;"
                          "GROUP1.OMEGA.CurrentPosition;"
                          "GROUP1.PSI.SetpointPosition;"
                          "GROUP1.PSI.CurrentPosition;"
                          "GROUP1.2THETA.SetpointPosition;"
                          "GROUP1.2THETA.CurrentPosition;"
                          "GROUP1.NU.SetpointPosition;"
                          "GROUP1.NU.CurrentPosition";
    char *positioner[NUM_AXES] = {"GROUP1.PHI", "GROUP1.KAPPA", "GROUP1.OMEGA", "GROUP1.PSI", "GROUP1.2THETA", "GROUP1.NU"};
    char group[] = "GROUP1";
    char outputfilename[500];
    double maxp, minp, maxv, maxa;
    char buffer[BUFFER_SIZE];
    int currentSamples, maxSamples;
    int i;
    int groupStatus;
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
            return status;
        }
        printf("Calling GroupHomeSearch ...\n");
        status = GroupHomeSearch(drive_socket, group);
        if (status) {
            printf("Error calling GroupHomeSearch error=%d\n", status);
            return status;
        }
    }

    printf("FTPing trajectory file to XPS ...\n");
    /* FTP the trajectory file from the local directory to the XPS */
    status = ftpConnect(XPS_ADDRESS, USERNAME, PASSWORD, &ftpSocket);
    if (status != 0) {
        printf("Error calling ftpConnect, status=%d\n", status);
        return status;
    }
    status = ftpChangeDir(ftpSocket, TRAJECTORY_DIRECTORY);
    if (status != 0) {
        printf("Error calling ftpChangeDir, status=%d\n", status);
        return status;
    }
    status = ftpStoreFile(ftpSocket, TRAJECTORY_FILE);
    if (status != 0) {
        printf("Error calling ftpStoreFile, status=%d\n", status);
        return status;
    }
   
    /* Define trajectory output pulses */
     printf("Defining output pulses ...\n");
    status = MultipleAxesPVTPulseOutputSet(poll_socket, group, 2, 
                                           NUM_TRAJECTORY_ELEMENTS-1, PULSE_TIME);
 
    /*************************** Verify trajectory **********************/
    printf("Verifying trajectory ...\n");
    status = MultipleAxesPVTVerification(drive_socket, group, TRAJECTORY_FILE);
    if (status != 0) {
        printf("Error performing MultipleAxesPVTVerification, status=%d\n",status);
        end = 1;
    }    

    printf("Reading verify results ...\n");
    printf(" MultipleAxesPVTVerificationResultGet\n");                
    for (i=0; i<NUM_AXES; i++) {
        status = MultipleAxesPVTVerificationResultGet(poll_socket,positioner[i], 
                                                      outputfilename,
                                                      &minp, &maxp, &maxv, &maxa);
        printf(" positioner %d, status %d, Max. pos. %g, Min. pos. %g, Max. vel. %g, Max. accel. %g\n",
                i, status, maxp, minp, maxv, maxa);
    }

    if (end == 1) return -1;    
 
   /***********************Configure Gathering and Timer******************/
    printf("Reseting gathering ...\n");
    status = GatheringReset(poll_socket);
    if (status != 0) {
        printf("Error performing GatheringReset, status=%d\n",status);
        return status;
    }

    printf("Defining gathering ...\n");
    status = GatheringConfigurationSet(poll_socket, NUM_AXES*NUM_GATHERING_ITEMS, gatheringData);
    if (status != 0) {
        printf("Error performing GatheringConfigurationSet, status=%d\n",status);
        return status;
    }

    printf("Defining trigger ...\n");
    status = EventExtendedConfigurationTriggerSet(poll_socket, 2, 
                                                  "Always;GROUP1.PVT.TrajectoryPulse",
                                                  "","","","");
    if (status != 0) {
        printf("Error performing EventExtendedConfigurationTriggerSet, status=%d\n",
               status);
        return status;
    }

    printf("Defining action ...\n");
    status = EventExtendedConfigurationActionSet(poll_socket, 1, "GatheringOneData",
                                                 "", "", "", "");
    if (status != 0) {
        printf("Error performing EventExtendedConfigurationActionSet, status=%d\n",
               status);
        return status;
    }

    printf("Starting gathering ...\n");
    status= EventExtendedStart(poll_socket, &eventID);
    if (status != 0) {
        printf("Error performing EventExtendedStart, status=%d\n", status);
        return status;
    }


    /********************* Run trajectory ****************************/
    status = GroupStatusGet(poll_socket, group, &groupStatus);
    if (status != 0) {
        printf("Error performing GroupStatusGet, status=%d\n", status);
        return status;
    }
    printf("Group status before executing trajectory=%d\n", groupStatus);
    printf("Executing trajectory ...\n");
    start_time = time(NULL);
    status = MultipleAxesPVTExecution(drive_socket, group, TRAJECTORY_FILE, 1);
    end_time = time(NULL);
    printf("Time to execute trajectory=%f\n", difftime(end_time, start_time));
    if (status != 0) {
        printf("Error performing MultipleAxesPVTExecution, status=%d\n", status);    
        return status;
    }
    status = GroupStatusGet(poll_socket, group, &groupStatus);
    if (status != 0) {
        printf("Error performing GroupStatusGet, status=%d\n", status);
        return status;
    }
    printf("Group status after executing trajectory=%d\n", groupStatus);

    /* Remove the event */
    printf("Removing event ...\n");
    status = EventExtendedRemove(poll_socket, eventID);
    if (status != 0) {
        printf("Error performing ExtendedEventRemove, status=%d\n", status);
        return status;
    }

    /***********************Save the gathered data ***************/
    printf("Stopping gathering ...\n");
    start_time = time(NULL);
    status = GatheringStop(drive_socket);
    end_time = time(NULL);
    printf("Time to stop gathering=%f\n", difftime(end_time, start_time));
    if (status != 0) {
        printf("Error performing GatheringExternalStop, status=%d\n", status);
        return status;
    }
    status = GroupStatusGet(poll_socket, group, &groupStatus);
    if (status != 0) {
        printf("Error performing GroupStatusGet, status=%d\n", status);
        return status;
    }
    printf("Group status after stopping gathering=%d\n", groupStatus);

    /* Read the number of lines of gathering */
    status = GatheringCurrentNumberGet(drive_socket, &currentSamples, &maxSamples);
    if (status != 0) {
        printf("Error calling GatherCurrentNumberGet, status=%d\n", status);
        return status;
    }
    if (currentSamples != NUM_GATHERING_POINTS) {
        printf("readGathering: warning, NUM_GATHERING_POINTS=%d, currentSamples=%d\n", 
            NUM_GATHERING_POINTS, currentSamples);
    }
    status = GatheringDataMultipleLinesGet(drive_socket, 0, currentSamples, buffer);
    printf("GatheringDataMultipleLinesGet, status=%d, currentSamples=%d\n", status, currentSamples);
    printf("Buffer length=%ld, buffer=\n%s\n", (long)strlen(buffer), buffer);    
    return 0;
}

