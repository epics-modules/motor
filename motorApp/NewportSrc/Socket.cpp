/*
DESCRIPTION
This module implements a client which periodically
sends a request to the slave spawned by a concurrent
server.  This code illustrates how many clients
to be handled in parallel if the server has a
concurrent architecture.
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
#include <osiSock.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <time.h>

/* defines */

#define MAX_MSG_SIZE       80
#define TIMEOUT             0.5
#define SIZEBUFFER        256
#define CONNREFUSED        -1
#define CREATESOCKETFAILED -2
#define OPTREFUSED         -3

#define MAX_NB_SOCKETS     40
#define SOCKET_TIMEOUT	   100

/* typedefs */

typedef int SOCK_FD;

/* global variables */

SOCK_FD sockFd[MAX_NB_SOCKETS];
BOOL    UsedSocket[MAX_NB_SOCKETS] = { FALSE };
double  TimeoutSocket[MAX_NB_SOCKETS];
int     ErrorSocket[MAX_NB_SOCKETS];




/***************************************************************************************/
int ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut)
{
/*	printf("Socket.cpp ConnectToServer: Top/n");*/
    int flag = 1;
    u_long srvInet;
    struct sockaddr_in srvAddr;
    int SocketIndex = 0;

	int    status;
	char * optval=0;
	osiSocklen_t  optlen;
	status = getsockopt (SocketIndex,IPPROTO_TCP,TCP_NODELAY, optval, &optlen);

	/* Select a free socket */
	while ((UsedSocket[SocketIndex] == TRUE) && (SocketIndex < MAX_NB_SOCKETS))
	{
		SocketIndex++;
	}

	if (SocketIndex == MAX_NB_SOCKETS)
        return -1;

    /* Create socket */
    ErrorSocket[SocketIndex] = 0;
    if ((sockFd[SocketIndex] = socket (PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
    {
        ErrorSocket[SocketIndex] = CREATESOCKETFAILED;
        return -1;
    }

    /* Convert the server's IP address from ASCII dot notation to */
    /* a network byte-ordered integer */
    srvInet = inet_addr (Ip_Address);

    /* Initialize servers address */
    memset ((char *)&srvAddr, 0, sizeof(srvAddr));
    srvAddr.sin_family      = AF_INET;	
    srvAddr.sin_port        = htons (Ip_Port);
    srvAddr.sin_addr.s_addr = srvInet;

    if (setsockopt (sockFd[SocketIndex], 
                    IPPROTO_TCP, 
                    TCP_NODELAY,
                    (char *)&flag, 
                    (int)sizeof(flag)) != 0 )
    {
        ErrorSocket[SocketIndex] = OPTREFUSED;
        return -1;
    }

    /* Connect to server */
    if (connect(sockFd[SocketIndex], (sockaddr *)&srvAddr, sizeof(srvAddr)) < 0)
    {
        ErrorSocket[SocketIndex] = CONNREFUSED;
        return -1;
    }
    
    if (TimeOut > 0) TimeoutSocket[SocketIndex] = TimeOut;
    else             TimeoutSocket[SocketIndex] = TIMEOUT;
    UsedSocket[SocketIndex] = TRUE;
    
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
    char pBuf[SIZEBUFFER]={'\0'};
    int  iRvcd=0;
    clock_t start, finish;
    double timeEllapse = 0.0;
    
	
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        /* Send String to controller and wait for response */
	
	/*printf("Timeout: %lf, CLOCKS_PER_SEC %i ",TimeoutSocket[SocketIndex],CLOCKS_PER_SEC);*/
        
	write (sockFd[SocketIndex], buffer, strlen(buffer) + 1);
	/*printf("SendAndRecieve after write\n");*/
        /* Read error ? */
/*        iRvcd = read (sockFd[SocketIndex], pBuf, SIZEBUFFER);*/
        start = clock();
/*	printf("SendAndRecieve after read\n");*/
	
        while ((iRvcd <= 0) && (timeEllapse < SOCKET_TIMEOUT))
        {
            
	    /*printf("SendAndRecieve above read\n");*/
	    iRvcd = read (sockFd[SocketIndex], pBuf, SIZEBUFFER);
            finish = clock();
            timeEllapse = (double)(finish - start) / CLOCKS_PER_SEC;
	    
            /* waiting ... */
        }
    }
    if (iRvcd > 0) pBuf[iRvcd] = '\0';
    strcpy(valueRtrn, pBuf);
}
/***************************************************************************************/
void SendOnly (int SocketIndex, char *buffer, char *valueRtrn)
{
    char pBuf[SIZEBUFFER]={'\0'};
    int  iRvcd=0;
    clock_t start, finish;
    double timeEllapse = 0.0;
	
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        /* Send String to controller and wait for response */

        write (sockFd[SocketIndex], buffer, strlen(buffer) + 1);
/*	printf("---------------------SendOnly after write\n");*/
        /* Read error ? */
/*        iRvcd = read (sockFd[SocketIndex], pBuf, SIZEBUFFER);
        start = clock();
	printf("SendAndRecieve after read\n");
	
        while ((iRvcd <= 0) && (timeEllapse < TimeoutSocket[SocketIndex]))
        {
            iRvcd = read (sockFd[SocketIndex], pBuf, SIZEBUFFER);
            finish = clock();
            timeEllapse = (double)(finish - start) / CLOCKS_PER_SEC;
            
        }*/
    }
    /*if (iRvcd > 0) pBuf[iRvcd] = '\0';*/
    pBuf[iRvcd] = '\0';
    strcpy(valueRtrn, pBuf);
}

/***************************************************************************************/
void CloseSocket(int SocketIndex)
{
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        close (sockFd[SocketIndex]);
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
            close (sockFd[i]);
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
