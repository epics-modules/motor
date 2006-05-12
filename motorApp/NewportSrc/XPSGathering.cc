/* Prog to test the XPS position gathering and trigger during a trajectory scan*/
/**/
#include <stdio.h>
/*#include <stdbool.h>*/
#include "XPS_C8_drivers.h"
#include "Socket.h"
#include <hostLib.h>
#include <remLib.h>
#include <netDrv.h>
#define TIMEOUT 		1
#define pline	printf("-Debug line %i\n",__LINE__);
#define BUFFER_SIZE 500;


/* require xps_c8_driver.cpp with 23rd May bug fix for the
 GatheringExternalConfigurationSet command*/


/* Function to ask the XPS for a socket */
int getsocket(void)
{
	char ipchar[] = "164.54.160.124";
	int socket = 0;
	int port = 5001;

	socket = TCP_ConnectToServer(ipchar,port,TIMEOUT);
	if (socket < 0) 
            printf(" Error TCP_ConnectToServer\n");
	return (socket);
}

void xpsgathering(int inttrajelementperiod)
{
	int status,socket,end =0;
	char *gatheringdata= 	"GROUP1.PHI.ExternalLatchPosition,"
				"GROUP1.KAPPA.ExternalLatchPosition,"
				"GROUP1.OMEGA.ExternalLatchPosition,"
				"GROUP1.PSI.ExternalLatchPosition,"
				"GROUP1.2THETA.ExternalLatchPosition,"
				"GROUP1.NU.ExternalLatchPosition";
			
						
	char positioner[] = "GROUP1.PHI";
	char group[] = "GROUP1";
	char nullchar[] = "";
	char GPIOname[] = "GPIO4.DO";	/* DB15 Connector */
	char pulsemask[] = "63"; /* Defines which pins are pulsed, 63 in base 2 ->111111 */
	/* The trigger pin6 is connected to GPIO4.DO pin 27 (output 1) */
/*	char element[250];*/
	char inputfilename[] = "testtrajectory";
	char outputfilename[500];
	char timer1[250] = "Timer1";
	double maxp,minp,maxv,maxa;
	char trajelementperiod[250] ;	
/*	int inttrajelementperiod = 20000;*/	/* Time defined in trajectory file between moves *1e4 */
	char numtrajelements[] = "10";		/* Number of elements in traj scan*/
	char eventlist[260];
	
	printf("You Input %g Seconds For The Period\n",(double) inttrajelementperiod/10000);
	
	sprintf(trajelementperiod,"%i",inttrajelementperiod);	/* Convert to a string */

	socket = getsocket();
	
	/*************************** Verfy trajectory **********************/
	status = MultipleAxesPVTVerification(socket,group,inputfilename);
	if (status != 0){
        	printf(" Error performing MultipleAxesPVTVerification%i\n Return\n",status);
		end = 1; /* Don't Verify */
		}	

	/*printf("xpsgathering, socket=%d, timer=%s, period=%d\n",
	        socket, timer1, inttrajelementperiod);*/
			
	status = MultipleAxesPVTVerificationResultGet(socket,positioner,outputfilename,
					&minp, &maxp, &maxv, &maxa);
	printf(" MultipleAxesPVTVerificationResultGet\n");				
	printf(" status %i, ptrmaxp %g, ptrminp %g, ptrmaxv %g, ptrmaxa %g\n outputfilename=%s\n",
			status,maxp, minp, maxv, maxa,outputfilename);

	/*printf("xpsgathering, socket=%d, timer=%s, period=%d \n",
	        socket, timer1, inttrajelementperiod);*/	
	if (end == 1)
		return;	
	/***********************Configure Gathering and Timer******************/

	/* The "1" signifies one type of data collected */

printf(" socket=%i, gathering=%s\n",socket,gatheringdata);

	status = GatheringExternalConfigurationSet(socket,1,gatheringdata);

	if (status != 0)
        	printf(" Error performing GatheringConfigurationSet%i\n",status);
	

	status = TimerSet(socket,timer1,inttrajelementperiod);

	if (status != 0)
        	printf(" Error performing TimerSet %i\n",status);

	/**************************Add Events *****************************/	    

	status = EventAdd(socket,positioner,timer1,nullchar,
			"DOPulse",GPIOname,pulsemask,nullchar);
	if (status != 0)
      	    printf(" Error performing EventAdd(socket,positioner,Timer1=%i\n",status);
		
	status = EventAdd(socket,positioner,"PVT.TrajectoryStart",nullchar,
			"ExternalGatheringRun",numtrajelements,"1",nullchar);
	if (status != 0)
            printf(" Error performing EventAdd(socket,positioner,PVT.TrajectoryStart %i\n",status);
	
	status = EventGet(socket,positioner,eventlist);
	printf(" EventGet: status=%i list=%s\n",status,eventlist);
		
	/********************* Run traj ****************************/
	printf("socket=%i group=%s file=%s\n",socket,group,inputfilename);

	status = MultipleAxesPVTExecution(socket,group,inputfilename,1);
	if (status != 0)
        	printf(" Error performing MultipleAxesPVTExecution\%i\n",status);	
	
	/***********************Save the gathered data ***************/
	status = GatheringExternalStopAndSave(socket);
	if (status != 0)
        	printf(" Error performing GatheringExternalStopAndSave\%i\n",status);
	
	/*******************Tidy-up the Event List and stop timer1 **************/
	status = EventRemove(socket,positioner,timer1,"0");
	if (status != 0)
      	    printf(" Error performing EventRemove(socket,positioner,Timer1)=%i\n",status);

	status = EventRemove(socket,positioner,"PVT.TrajectoryStart",nullchar);
	if (status != 0)
      	    printf(" Error performing EventRemove(socket,positioner,PVT.TrajectoryStart)=%i\n",status);		

	
/********************************************************************/	
	
	FILE *trajFile;
	FILE *gatheringFile;
	char buffer[100];
	double motorReadbacks[6][20];
	double motorError[6][20];
	double trajTime = 0,trajStep = 0;
	int numAxes = 6;
	int i,j,npoints = 10;
	double posTheory[8];
	hostAdd ("XPS1","164.54.160.124");
	netDevCreate ("XPS1:", "XPS1", 1);
	remCurIdSet("Administrator", "Administrator");
	trajFile =  fopen ("XPS1:/Admin/Public/Trajectories/testtrajectory", "r");
	gatheringFile = fopen ("XPS1:/Admin/Public/GatheringExternal.dat", "r");

	for (i=0; i<npoints; ++i){
	     for (j=0; j<numAxes; ++j){
		motorReadbacks[j][i] = 0;
		motorError[j][i] = 0;
		}
	    posTheory[j] = 0;
	}

	/* Read 1st 2 lines */
	for(i=0; i<2; ++i){
	    fgets (buffer, 1000, gatheringFile);
	    printf("Line %i of GatheringEx = %s\n",i,buffer);
	 }
	 for (i=0; i<npoints; ++i){
	     for (j=0; j<numAxes; ++j){
	         if(j == (numAxes-1)) {
		     fscanf(gatheringFile,"%lf",&motorReadbacks[j][i]);}
		  else{   
	             fscanf(gatheringFile,"%lf",&motorReadbacks[j][i]);}

		     
		 if(j == (numAxes-1)) {
		     if(fscanf(trajFile," %lf,%*f",&trajStep) != 1) printf("trajerror\n");}
		 
		 if (j == 0){   
	             if(fscanf(trajFile,"%lf, %lf,%*f,",&trajTime,&trajStep) != 2) printf("trajerror\n");}
		 
		 if (j > 0 && j < (numAxes-1)){
		     if(fscanf(trajFile," %lf,%*f,",&trajStep) != 1) printf("trajerror\n");}
		 
		 if(i == 0) {
		     posTheory[j] = motorReadbacks[j][i];
		     printf("posTheory = %f ",posTheory[j]);}
		 else{
		     posTheory[j]+= trajStep;}
		     
		 motorError[j][i] = posTheory[j] - motorReadbacks[j][i];
		 printf("i=%i J=%i ReadBack=%f	motorError=%f trajStep%f\n",
		 	i,j,motorReadbacks[j][i],motorError[j][i],trajStep);
	     }
	  }
	
	
	
	fclose (trajFile); 	/* This is where the ftp actualy takes place */
	fclose (gatheringFile); 	/* This is where the ftp actualy takes place */	
/**********************************************************************/
		
	return;
}
