/*///////////////////////////////////////////////////////////////////////////////
* Socket.cpp
*/
#include "Socket.h" 

#define TIMEOUT 300
#define SMALL_BUFFER_SIZE 256
#define MAX_NB_SOCKETS  100

void Delay(double timedelay);

CAsyncSocket m_sConnectSocket[MAX_NB_SOCKETS];
BOOL         UsedSocket[MAX_NB_SOCKETS] = { FALSE};
double       TimeoutSocket[MAX_NB_SOCKETS];
int          NbSockets = 0;

/***************************************************************************************/
int ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut)
{
	int flag = 1;
	int socketID = 0;
	DWORD sockPendingFlag = 1;

	if (!AfxSocketInit())
	{
		AfxMessageBox("Fatal Error: MFC Socket initialization failed");
		return -1;
	}
	/* Select a socket number */
	if (NbSockets < MAX_NB_SOCKETS)
	{
		while ((UsedSocket[socketID] == TRUE) && (socketID < MAX_NB_SOCKETS))
			socketID++;

		if (socketID == MAX_NB_SOCKETS)
			return -1;
	}
	else
		return -1;
	UsedSocket[socketID] = TRUE;
	NbSockets++;

	/* Socket creation */
	if ((m_sConnectSocket[socketID].Create() == 0)
		|| (m_sConnectSocket[socketID].SetSockOpt(TCP_NODELAY,(char *)&flag,(int)sizeof( flag ),IPPROTO_TCP) == 0))
	{
		UsedSocket[socketID] = FALSE;
		NbSockets--;
		return -1;
	}

	/* Connect */
	if (m_sConnectSocket[socketID].Connect(Ip_Address,Ip_Port) == 0)
	{
		int SocketError = m_sConnectSocket[socketID].GetLastError();
		if (SocketError != WSAEWOULDBLOCK)
		{
			UsedSocket[socketID] = FALSE;
			NbSockets--;
			return -1;
		}
	}

	/* Set timeout */
	if (TimeOut > 0)
	{
		if (TimeOut < 1e-3)
			TimeoutSocket[socketID] = 1e-3;
		else
			TimeoutSocket[socketID] = TimeOut;
	}
	else
		TimeoutSocket[socketID]	= TIMEOUT;

	/* Socket array */
	struct fd_set *Sockets = new struct fd_set;
	FD_ZERO(Sockets);
	FD_SET(m_sConnectSocket[socketID], Sockets);

	/* Time structure */
	struct timeval *TimeOutStruct = new struct timeval;
	TimeOutStruct->tv_sec = (long) TimeoutSocket[socketID];
	TimeOutStruct->tv_usec = (long) ((TimeoutSocket[socketID] - (long)TimeoutSocket[socketID])* 1e9);

	/* Checking connection is ok */
	int SelectReturn = select(0, NULL, Sockets, NULL, TimeOutStruct);
	if (SelectReturn == SOCKET_ERROR 
		|| SelectReturn == 0)
	{
		UsedSocket[socketID] = FALSE;
		NbSockets--;
		return -1;
	}

	/* Force no pending */
	if (m_sConnectSocket[socketID].IOCtl(FIONBIO,&sockPendingFlag) == 0)
	{
		m_sConnectSocket[socketID].Close();
		UsedSocket[socketID] = FALSE;
		NbSockets--;
		return -1;
	}
	
	/* Delay unless -1 return for the first API */
	Sleep(10);

	/* Return socket ID */
	return socketID;
}

/***************************************************************************************/
void SetTCPTimeout(int SocketIndex, double TimeOut)
{
	if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
	{
		if (TimeOut > 0)
		{
			if (TimeOut < 1e-3)
				TimeoutSocket[SocketIndex] = 1e-3;
			else
				TimeoutSocket[SocketIndex] = TimeOut;
		}
	}
}

/***************************************************************************************/
void SendAndReceive(int socketID, char sSendString[], char sReturnString[], int iReturnStringSize)
{
	char    sSocketBuffer[SMALL_BUFFER_SIZE + 1] = {'\0'};
	int     iReceiveByteNumber = 0;
	int     iErrorNo = 0;
	fd_set  readFds;
	int     iSelectStatus;
	double  dTimeout;
	struct timeval cTimeout;
	clock_t start, stop;

	if ((socketID >= 0) && (socketID < MAX_NB_SOCKETS) && (UsedSocket[socketID] == TRUE))
	{
		/* Clear receive buffer */
		do
		{
			iReceiveByteNumber = m_sConnectSocket[socketID].Receive(sSocketBuffer,SMALL_BUFFER_SIZE);
		}
		while (iReceiveByteNumber != SOCKET_ERROR);
		sReturnString[0] = '\0';

		/* Send String to controller and wait for response */
		m_sConnectSocket[socketID].Send(sSendString,strlen(sSendString));

		/* Get reply with timeout */
		dTimeout = TimeoutSocket[socketID]; 
		do
		{
			/* Get time */
			start = clock();

			/* Check reply */
			iReceiveByteNumber = m_sConnectSocket[socketID].Receive(sSocketBuffer,SMALL_BUFFER_SIZE);
			iErrorNo = GetLastError() & 0xffff;
			
			/* Wait for reply */
			if ((iReceiveByteNumber == SOCKET_ERROR) && (iErrorNo == WSAEWOULDBLOCK))
			{
				FD_ZERO(&readFds);
				FD_SET(m_sConnectSocket[socketID].m_hSocket, &readFds);
				cTimeout.tv_sec = (long)dTimeout;
				cTimeout.tv_usec = (long)((dTimeout - (long)dTimeout) * 1e6);
				iSelectStatus = select(FD_SETSIZE, (fd_set *)&readFds, (fd_set *) NULL, (fd_set *) NULL, &cTimeout);
				if ((iSelectStatus > 0) && (FD_ISSET(m_sConnectSocket[socketID].m_hSocket, &readFds)))
					iReceiveByteNumber = m_sConnectSocket[socketID].Receive(sSocketBuffer,SMALL_BUFFER_SIZE);
				else
				{
					iErrorNo = GetLastError() & 0xffff;
					sprintf(sSocketBuffer,"-2,%s,EndOfAPI",sSendString);
					strncpyWithEOS(sReturnString, sSocketBuffer, strlen(sSocketBuffer), iReturnStringSize);
					iReceiveByteNumber = SOCKET_ERROR;
				}
			}

			/* Concatenation */
			if ((iReceiveByteNumber >= 0) && (iReceiveByteNumber <= SMALL_BUFFER_SIZE))
			{
				sSocketBuffer[iReceiveByteNumber] = '\0';
				strncat(sReturnString, sSocketBuffer, iReturnStringSize - strlen(sReturnString) - 1);
			}

			/* Calculate new timeout */
			stop = clock();
			dTimeout = dTimeout - (double)(stop - start) / CLOCKS_PER_SEC;
			if (dTimeout < 1e-3)
				dTimeout = 1e-3;
		}
		while ((iReceiveByteNumber != SOCKET_ERROR) && (strstr(sReturnString, "EndOfAPI") == NULL));
	}
	else
		sReturnString[0] = '\0';

	return;
}

/***************************************************************************************/
void CloseSocket(int socketID)
{
	if ((socketID >= 0) && (socketID < MAX_NB_SOCKETS))
	{
		if (UsedSocket[socketID] == TRUE)
		{
			m_sConnectSocket[socketID].Close();
			UsedSocket[socketID] = FALSE;
			TimeoutSocket[socketID] = TIMEOUT;
			NbSockets--;
		}
	}
}

/***************************************************************************************/
char * GetError(int socketID)
{
	if ((socketID >= 0) && (socketID < MAX_NB_SOCKETS))
	{
		int error = m_sConnectSocket[socketID].GetLastError();
		switch (error)
		{
		case WSANOTINITIALISED:
			return (_T("A successful AfxSocketInit must occur before using this API."));
		case WSAENETDOWN:
			return(_T("The Windows Sockets implementation detected that the network subsystem failed."));
		case WSAEADDRINUSE:
			return(_T("The specified address is already in use."));
		case WSAEINPROGRESS:
			return(_T("A blocking Windows Sockets call is in progress."));
		case WSAEADDRNOTAVAIL:
			return(_T("The specified address is not available from the local machine."));
		case WSAEAFNOSUPPORT:
			return(_T("Addresses in the specified family cannot be used with this socket."));
		case WSAECONNREFUSED:
			return(_T("The attempt to connect was rejected."));
		case WSAEDESTADDRREQ:
			return(_T("A destination address is required."));
		case WSAEFAULT:
			return(_T("The nSockAddrLen argument is incorrect."));
		case WSAEINVAL:
			return(_T("Invalid host address."));
		case WSAEISCONN:
			return(_T("The socket is already connected."));
		case WSAEMFILE:
			return(_T("No more file descriptors are available."));
		case WSAENETUNREACH:
			return(_T("The network cannot be reached from this host at this time."));
		case WSAENOBUFS:
			return(_T("No buffer space is available. The socket cannot be connected."));
		case WSAENOTSOCK:
			return(_T("The descriptor is not a socket."));
		case WSAETIMEDOUT:
			return(_T("Attempt to connect timed out without establishing a connection."));
		case WSAEWOULDBLOCK:
			return(_T("The socket is marked as nonblocking and the connection cannot be completed immediately."));
		case WSAEPROTONOSUPPORT:
			return(_T("The specified port is not supported."));
		case WSAEPROTOTYPE:
			return(_T("The specified port is the wrong type for this socket."));
		case WSAESOCKTNOSUPPORT:
			return(_T("The specified socket type is not supported in this address family."));
		}
		return(_T(""));
	}
	else
		return(_T(""));
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

