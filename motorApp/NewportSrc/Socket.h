/*///////////////////////////////////////////////////////////////////////////////
 * Socket.h
 */

#ifdef _WIN
#include <afxsock.h>		// MFC socket extensions
#endif

int  ConnectToServer (char *Ip_Address, int Ip_Port, double TimeOut);
void SetTCPTimeout (int SocketID, double Timeout);
void SendAndReceive(int socketID, char sSendString[], char sReturnString[], int iReturnStringSize);
void CloseSocket (int SocketID);
char * GetError (int SocketID);
void strncpyWithEOS(char * szStringOut, const char * szStringIn, int nNumberOfCharToCopy, int nStringOutSize);
