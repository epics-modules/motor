/* This script was adapted from the Newport Socket.cpp code
   to provide TCP/IP sockets for the XPSC8 motion controller
   using EPICS asynOctetSyncIO.cc
   
   By Jon Kelly July 2005
   Re-written by Mark Rivers March 2006
*/

/* includes */

#ifdef vxWorks
#else
#define TRUE 1
#define FALSE 0
typedef int BOOL;
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <asynDriver.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <drvAsynIPPort.h>
#include <epicsExport.h>


/* The maximum number of sockets to XPS controllers.  The driver uses
 * one socket per motor plus one per controller, so a maximum of 9 per controller.
 * This number is used to create an array because the Newport code uses integers for
 * the socket ID. */
#define MAX_SOCKETS       1000
#define PORT_NAME_SIZE     100
#define ERROR_STRING_SIZE  100
#define DEFAULT_TIMEOUT    0.2
#define XPS_TERMINATOR     ",EndOfAPI"

#define MAX_RETRIES 2

static int  nextSocket = 0;

/* Pointer to the connection info for each socket 
   the asynUser structure is defined in asynDriver.h */
typedef struct {
    asynUser *pasynUser;
    asynUser *pasynUserCommon;
    double timeout;
    char errorString[ERROR_STRING_SIZE];
    int connected;
    epicsMutexId mutexId;
} socketStruct;
static socketStruct socketStructs[MAX_SOCKETS];


/***************************************************************************************/
int ConnectToServer(char *IpAddress, int IpPort, double timeout)
{

    char portName[PORT_NAME_SIZE];
    char ipString[PORT_NAME_SIZE];
    asynUser *pasynUser, *pasynUserCommon;
    socketStruct *psock;
    int status;

    if (nextSocket >= MAX_SOCKETS) {
        printf("ConnectToServer: too many open sockets, max=%d\n", MAX_SOCKETS);
        return -1;
    }
    /* Create a new asyn port */
    epicsSnprintf(ipString, PORT_NAME_SIZE, "%s:%d TCP", IpAddress, IpPort);
    epicsSnprintf(portName, PORT_NAME_SIZE, "%s:%d:%d", IpAddress, IpPort, nextSocket);
    /* Create port with autoConnect and noProcessEos options */
    drvAsynIPPortConfigure(portName, ipString, 0, 0, 1);

    /* Connect to driver with asynOctet interface */
    status = pasynOctetSyncIO->connect(portName, 0, &pasynUser, NULL);
    if (status != asynSuccess) {
        printf("ConnectToServer, error calling pasynOctetSyncIO->connect %s\n", pasynUser->errorMessage);
        return -1;
    }
    psock = &socketStructs[nextSocket];
    psock->pasynUser = pasynUser;

    /* Connect to driver with asynCommon interface */
    status = pasynCommonSyncIO->connect(portName, 0, &pasynUserCommon, NULL);
    if (status != asynSuccess) {
        printf("ConnectToServer, error calling pasynCommonSyncIO->connect %s\n", 
               pasynUserCommon->errorMessage);
        return -1;
    }
    psock->pasynUserCommon = pasynUserCommon;
    
    /* Create a mutex to prevent more than 1 thread using socket at once
     * Normally the SyncIO-.writeRead function takes care of this, but for long responses
     * we can't use a single write/read operation */
    psock->mutexId = epicsMutexMustCreate();

    psock->timeout = timeout;
    psock->connected = 1;
    strcpy(psock->errorString, "");

    nextSocket++;
    return nextSocket-1;
}

/***************************************************************************************/
void SetTCPTimeout(int SocketIndex, double TimeOut)
{
    if ((SocketIndex < 0) || (SocketIndex >= nextSocket)) {
        printf("SetTCPTimeout, SocketIndex=%d, must be >=0 and < %d\n", SocketIndex, nextSocket);
        return;
    }
    socketStructs[SocketIndex].timeout = TimeOut;
}


/***************************************************************************************/
void SendAndReceive (int SocketIndex, char buffer[], char valueRtrn[], int returnSize)
{
    size_t nbytesOut; 
    size_t nbytesIn;
    int eomReason;
    int bufferLength;
    socketStruct *psock;
    int status;
    int retries;
    int errStat;
    size_t nread;

    /* Check to see if the Socket is valid! */
    
    bufferLength = (int)strlen(buffer);
    if ((SocketIndex < 0) || (SocketIndex >= nextSocket)) {
        printf("SendAndReceive: invalid SocketIndex %d\n", SocketIndex);
        strcpy(valueRtrn,"-22");
        return;
    }
    psock = &socketStructs[SocketIndex];
    if (!psock->connected) {
        printf("SendAndReceive: socket not connected %d\n", SocketIndex);
        strcpy(valueRtrn,"-22");
        return;
    }

    epicsMutexMustLock(psock->mutexId);
    /* If timeout > 0. then we do a write read.  If < 0. then write. */

    if (psock->timeout > 0.0) {
        status = pasynOctetSyncIO->writeRead(psock->pasynUser,
                                             (char const *)buffer, 
                                             bufferLength,
                                             valueRtrn,
                                             returnSize,
                                             psock->timeout,
                                             &nbytesOut,
                                             &nbytesIn,
                                             &eomReason);
        if ( status != asynSuccess ) {
            asynPrint(psock->pasynUser, ASYN_TRACE_ERROR,
                      "SendAndReceive error calling writeRead, output=%s status=%d, error=%s\n",
                      buffer, status, psock->pasynUser->errorMessage);
        }
        asynPrint(psock->pasynUser, ASYN_TRACEIO_DRIVER,
                  "SendAndReceive, sent: '%s', received: '%s'\n",
                  buffer, valueRtrn);
        nread = nbytesIn;
        /* Loop until we the response contains ",EndOfAPI" or we get an error */
        while ((status==asynSuccess) && 
               (strcmp(valueRtrn + nread - strlen(XPS_TERMINATOR), XPS_TERMINATOR) != 0)) {
            status = pasynOctetSyncIO->read(psock->pasynUser,
                                            &valueRtrn[nread],
                                            returnSize-nread,
                                            psock->timeout,
                                            &nbytesIn,
                                            &eomReason);
            asynPrint(psock->pasynUser, ASYN_TRACEIO_DRIVER,
                  "SendAndReceive, received: nread=%d, returnSize-nread=%ld, nbytesIn=%d\n",
                      (int)nread, (long)(returnSize-nread), (int)nbytesIn);
            nread += nbytesIn;
        }
    } else {
        /* This is typically used for the "Move" commands, and we don't want to wait for the response */
        /* Fake the response by putting "-1" (for error) or "0" (for success) in the return string */
        for (retries=0; retries<MAX_RETRIES; retries++) {
            status = pasynOctetSyncIO->writeRead(psock->pasynUser,
                                                 (char const *)buffer, 
                                                 bufferLength,
                                                 valueRtrn,
                                                 returnSize,
                                                 -psock->timeout,
                                                 &nbytesOut,
                                                 &nbytesIn,
                                                 &eomReason);
            if (status == asynError) {
                asynPrint(psock->pasynUser, ASYN_TRACE_ERROR,
                          "SendAndReceive error calling write, output=%s status=%d, error=%s\n",
                          buffer, status, psock->pasynUser->errorMessage);
                strcpy(valueRtrn, "-1");
                break;
            }
            asynPrint(psock->pasynUser, ASYN_TRACEIO_DRIVER,
                      "SendAndReceive, sent: '%s'\n", buffer);    
            /* A timeout is OK */
            if (status == asynTimeout) {
                asynPrint(psock->pasynUser, ASYN_TRACEIO_DRIVER,
                          "SendAndReceive, timeout on read\n");
                strcpy(valueRtrn, "0");
                break;
            } else {
                asynPrint(psock->pasynUser, ASYN_TRACEIO_DRIVER,
                          "SendAndReceive, read: '%s'\n", valueRtrn);
                errStat = atoi(valueRtrn);
                if (errStat == 0) break;    /* All done */
                if (errStat == -1) continue; /* Error that previous command not complete */
                asynPrint(psock->pasynUser, ASYN_TRACE_ERROR,
                          "SendAndReceive unexpected response =%s\n",
                          valueRtrn);
                break;
            }
        }
        if (retries == MAX_RETRIES) strcpy(valueRtrn, "0");
    }
    epicsMutexUnlock(psock->mutexId);
}


/***************************************************************************************/
int ReadXPSSocket (int SocketIndex, char valueRtrn[], int returnSize, double timeout)
{
    size_t nbytesIn;
    int eomReason;
    socketStruct *psock;
    int status;
    size_t nread=0;

    /* Check to see if the Socket is valid! */
    if ((SocketIndex < 0) || (SocketIndex >= nextSocket)) {
        printf("ReadXPSSocket: invalid SocketIndex %d\n", SocketIndex);
        strcpy(valueRtrn,"-22");
        return -1;
    }
    psock = &socketStructs[SocketIndex];
    if (!psock->connected) {
        printf("ReadXPSSocket: socket not connected %d\n", SocketIndex);
        strcpy(valueRtrn,"-22");
        return -1;
    }

    /* Loop until we the response contains ",EndOfAPI" or we get an error */
    do {
        status = pasynOctetSyncIO->read(psock->pasynUser,
                                        &valueRtrn[nread],
                                        returnSize-nread,
                                        timeout,
                                        &nbytesIn,
                                        &eomReason);
        asynPrint(psock->pasynUser, ASYN_TRACEIO_DRIVER,
              "ReadXPSSocket, received: nread=%d, returnSize-nread=%ld, nbytesIn=%d\n",
                  (int)nread, (long)(returnSize-nread), (int)nbytesIn);
        nread += nbytesIn;
    } while ((status==asynSuccess) && 
             (strcmp(valueRtrn + nread - strlen(XPS_TERMINATOR), XPS_TERMINATOR) != 0));
    return (int)nread;
}


/***************************************************************************************/
void CloseSocket(int SocketIndex)
{
    socketStruct *psock;
    asynUser *pasynUser;
    int status;

    if ((SocketIndex < 0) || (SocketIndex >= nextSocket)) {
        printf("CloseSocket: invalid SocketIndex %d\n", SocketIndex);
        return;
    }
    psock = &socketStructs[SocketIndex];
    pasynUser = psock->pasynUserCommon;
    status = pasynCommonSyncIO->disconnectDevice(pasynUser);
    if (status != asynSuccess ) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "CloseSocket: error calling pasynCommonSyncIO->disconnect, status=%d, %s\n",
                  status, pasynUser->errorMessage);
        return;
    }
    psock->connected = 0;
}

/***************************************************************************************/
void closeXPSSockets(void)
{
    int i;

    for (i=0; i<nextSocket; i++) {
        if (socketStructs[i].connected) CloseSocket(i);
    }
}

/***************************************************************************************/
char * GetError(int SocketIndex)
{
    if ((SocketIndex < 0) || (SocketIndex >= nextSocket)) {
        printf("GetError: invalid SocketIndex %d\n", SocketIndex);
        return (char *)"Invalid socket";
    }
    return socketStructs[SocketIndex].errorString;
}

/***************************************************************************************/
void strncpyWithEOS(char * szStringOut, const char * szStringIn, int nNumberOfCharToCopy, int nStringOutSize)
{
        if (nNumberOfCharToCopy < nStringOutSize)
        {
                strncpy (szStringOut, szStringIn, nNumberOfCharToCopy);
                szStringOut[nNumberOfCharToCopy] = '\0';
        }
        else
        {
                strncpy (szStringOut, szStringIn, nStringOutSize - 1);
                szStringOut[nStringOutSize - 1] = '\0';
        }
}
