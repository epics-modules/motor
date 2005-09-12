/* This script was addapted from the Newport Socket.cpp code
   to provide TCP/IP sockets for the XPSC8 motion controller
   using EPICS asynOctetSyncIO.cc
   
   By Jon Kelly July 2005
*/

/* includes */

#ifdef vxWorks
#else
#define TRUE 1
#define FALSE 0
typedef int BOOL;
#endif

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "asynDriver.h"
#include "asynOctetSyncIO.h"
#include        <epicsThread.h>
#include        <epicsMutex.h>
#include        <epicsExport.h>
#include        <epicsString.h>


/* defines */

#define MAX_MSG_SIZE       256
#define TIMEOUT            0.2
#define MAX_RETRY          5
#define CONNREFUSED        -1
#define CREATESOCKETFAILED -2
#define OPTREFUSED         -3

#define MAX_NB_SOCKETS     50

/* global variables */

BOOL    UsedSocket[MAX_NB_SOCKETS] = { FALSE };
double  TimeoutSocket[MAX_NB_SOCKETS];
int     ErrorSocket[MAX_NB_SOCKETS];

/* Pointer to the connection info for each socket 
   the asynUser structure is defined in asynDriver.h */
static struct asynUser *pasynUserArray[MAX_NB_SOCKETS];

asynStatus status;

/***************************************************************************************/
#define DEBUG

#ifdef __GNUG__
    #ifdef	DEBUG
        volatile int asynXPSC8Debug = 10;
	#define Debug(l, f, args...) { if(l<=asynXPSC8Debug) printf(f,## args); }
	epicsExportAddress(int, asynXPSC8Debug);
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif


/***************************************************************************************/
/* The drvXPSC8.cc must assign the aysn port string to the IP field 
   and replace the xps port int with the asyn addr int		*/
int ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut)
{

    int SocketIndex = 0;
    const char *asynPort = (const char *)Ip_Address;
    int asynAddr;
    asynAddr = Ip_Port;

    /* Select a free socket */
    while ((UsedSocket[SocketIndex] == TRUE) && (SocketIndex < MAX_NB_SOCKETS))
	{
		SocketIndex++;
	}

    if (SocketIndex == MAX_NB_SOCKETS)
        return -1;

    ErrorSocket[SocketIndex] = 0;


    /* These asyn functions are defined in asynOctetSyncIO.c */
    status = pasynOctetSyncIO->connect(asynPort,asynAddr,&pasynUserArray[SocketIndex],NULL);
    
    if (status != asynSuccess )
    {
        ErrorSocket[SocketIndex] = CREATESOCKETFAILED;
	printf("Error Socket connect failed asynStatus=%i  \n",status);
        return -1;
    }

    
    /* Use default timeout macro if one is not given 
    if (TimeOut > 0) TimeoutSocket[SocketIndex] = TimeOut;
    else             TimeoutSocket[SocketIndex] = TIMEOUT;*/
    
    UsedSocket[SocketIndex] = TRUE;
    TimeoutSocket[SocketIndex] = TIMEOUT; /* Ignore the requested timeout */
    return SocketIndex;
}

/***************************************************************************************/
void SetTCPTimeout(int SocketIndex, double TimeOut)
{
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
       if (TimeOut > 0) TimeoutSocket[SocketIndex] = TimeOut;
    }
}

/***************************************************************************************/
void SendAndReceive (int SocketIndex, char *buffer, char *valueRtrn)
{
    int nbytesOut = 0; 
    int nbytesIn = 0;
    int eomReason;
    /*clock_t start, finish;
    double timeEllapse = 0.0;*/
    char *output, *input;
    int writeReadStatus = -99;
    int loop = 0, bufferLength;

    /* Check to see if the Socket is valid! */
    
    bufferLength = strlen(buffer);
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        if (bufferLength > MAX_MSG_SIZE)
        {
	    /* Error string too long */
	    printf("Error buffer>Max SendAndRecieve Out=%s In=%s\n",valueRtrn,buffer);
	    printf("asynStatus=%i  ",status);
	    strcpy(valueRtrn,"-3");
	    return;
	}
    	
	while ((writeReadStatus < 0) && (loop < MAX_RETRY))
	{
	    status = pasynOctetSyncIO->writeRead(pasynUserArray[SocketIndex],
						(char const *)buffer, 
						bufferLength,
						valueRtrn,
						MAX_MSG_SIZE,
						TimeoutSocket[SocketIndex],
						&nbytesOut,
						&nbytesIn,
						&eomReason);
	
	    Debug(12,"buffer %s\n bufferlen %i\n valueRtrn %s\n",buffer,bufferLength,valueRtrn);
	    Debug(12,"nbytesOut %i nbytesIn %i eomReason %i\n",nbytesOut,nbytesIn,eomReason); 
	    output = valueRtrn;
	    if (output != NULL) sscanf (output, "%i", &writeReadStatus);
	    
	    /* If the XPS returns a status <0 it is probably still confused from a 
	     * sendOnly so re-send the command */
	     
	    if (writeReadStatus < 0){
	        Debug(10,"Address %x\n",pasynUserArray[SocketIndex]);
	        Debug(1,"Error WriteReadStatus = %i Retry ",writeReadStatus);
		Debug(2,"Sock=%i Tout=%g In=%s Out=%s \n  Command sent again! loop %i\n",SocketIndex,
					TimeoutSocket[SocketIndex],valueRtrn,buffer,loop);
	    } else {
	    
	    /* If the XPS returns a non error status check to see if the data
	     * is what you expected because it frequently returns the answere
	     * to the previous call! */

	        output = valueRtrn;
		input = buffer;
		strchr (output, ',');
	        while ( (input=strchr (input, '*')) != NULL ) { /* Number required */
		    input++; /* Move ptr past * */
		    if ((output=strchr (output, ',')) == NULL) { /* No comma after number */
		        Debug(2,"Error Not enough numbers in=%s out=%s\n",valueRtrn,buffer);
		        writeReadStatus = -98;
		    }
		    output++; /* move ptr past , */
		Debug(15,"Look for number in return string %s\n",output);
		}
	    }
	    loop++;
	}
	
	if ( status != asynSuccess ) 
    	{
	    /* Asyn command error */
	    Debug(5,"Error buffer %s\n bufferlen %i\n valueRtrn %s\n",buffer,bufferLength,valueRtrn);
	    Debug(5,"Error nbytesOut %i nbytesIn %i eomReason %i\n",nbytesOut,nbytesIn,eomReason); 
	    
	    printf("Error SendAndRecieve read In=%s Out=%s nbytesOut=%i",valueRtrn,buffer,nbytesOut);
	    printf(" asynStatus=%i \n",status);
	    return;
	}
    
    } else {
        /* Not allowed action socket was not created */
	printf("Error Socket Invalid SendAndRecieve In=%s Out=%s\n",valueRtrn,buffer);
	printf("asynStatus=%i  ",status);
	strcpy(valueRtrn,"-22");
	return;
    }
    return;
}
/***************************************************************************************/
void SendOnly (int SocketIndex, char *buffer, char *valueRtrn)
{
    char dummyReturn[MAX_MSG_SIZE];
    int nbytesOut = 0; 
    int nbytesIn = 0;
    int eomReason;
    char const dummyBuffer[40] = "FirmwareVersionGet (char *)";
    char *output;
    int writeReadStatus = -99;
    int loop = 0;
    
    /* Valid Socket? */
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        if (strlen(buffer) > MAX_MSG_SIZE)
        {
	    /* Error string too long */
	    strcpy(valueRtrn,"-3");
	    return;
	}

Above_Write:
	status = pasynOctetSyncIO->write(pasynUserArray[SocketIndex],(char const *)buffer, 
				  	strlen(buffer),TimeoutSocket[SocketIndex], &nbytesIn);

	if ((status != asynSuccess) || (nbytesIn <= 0))
    	{
	    printf("****Error SendOnly status=%i",status);
	    /* Error during write message */
	    strcpy(valueRtrn,"-72");
	    return;
	}

/* When a send only is performed but the XPS is expecting to give a response it
 * becomes confused for a period of time.  A writeRead is sent and the readback 
 * checked until the XPS returns a sensible value */

Above_Dummy_writeRead:	 
	 status = pasynOctetSyncIO->writeRead(pasynUserArray[SocketIndex],
						(char const *)dummyBuffer, 
						strlen(dummyBuffer),
						dummyReturn,
						MAX_MSG_SIZE,
						TimeoutSocket[SocketIndex],
						&nbytesOut,
						&nbytesIn,
						&eomReason);

	 output = dummyReturn;
	 if (output != NULL) sscanf (output, "%i", &writeReadStatus);
	 else Debug(5,"SendOnly Dummywriteread Output Null\n");
	    
	 if (writeReadStatus < 0 && output != NULL){
	        Debug(2,"SendOnly DumyWriteReadStatus = %i Retry ",writeReadStatus);
		Debug(3,"Sock=%i Timeout=%g DumyReturn=%s \n DummyIn=%s OrigCommand=%s\n"\
		" Dummy sent again\n",SocketIndex,TimeoutSocket[SocketIndex],
							dummyReturn,dummyBuffer,buffer);
	 } 
	 loop++;
	 if (writeReadStatus == -3 && loop < MAX_RETRY) /* Command Not executed by XPS so retry */
	     goto Above_Write;
	 if (writeReadStatus < 0 && loop < MAX_RETRY) /* The XPS is still confused so re-send*/
	     goto Above_Dummy_writeRead;
	    
    } else {
        /* Not allowed action */
	strcpy(valueRtrn,"-22");
        return;
    }
    strcpy(valueRtrn,"0");
    return;
}

/***************************************************************************************/
void CloseSocket(int SocketIndex)
{
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        status = pasynOctetSyncIO->disconnect(pasynUserArray[SocketIndex]);
	
	TimeoutSocket[SocketIndex] = TIMEOUT;
	ErrorSocket[SocketIndex] = 0;
	UsedSocket[SocketIndex] = FALSE;
    }
}

/***************************************************************************************/
void CloseAllSockets(void)
{
	int i;
    for (i = 0; i < MAX_NB_SOCKETS; i++)
    {
	if (UsedSocket[i] == TRUE)
	{
            status = pasynOctetSyncIO->disconnect(pasynUserArray[i]);
		    
	    TimeoutSocket[i] = TIMEOUT;
	    ErrorSocket[i] = 0;
	    UsedSocket[i] = FALSE;
	}
    }
}

/***************************************************************************************/
void ResetAllSockets(void)
{
	int i;
    for (i = 0; i < MAX_NB_SOCKETS; i++)
    {
		if (UsedSocket[i] == TRUE)
		    UsedSocket[i] = FALSE;
    }
}


/***************************************************************************************/
char * GetError(int SocketIndex)
{
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        switch (ErrorSocket[SocketIndex])
        {
        case CONNREFUSED:         return("The attempt to connect was rejected.");
        case CREATESOCKETFAILED:  return("Create Socket failed.");
        case OPTREFUSED:          return("SetSockOption() Refused.");
        }
    }
    return("");
}
