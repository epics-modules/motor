
int    ConnectToServer (char *Ip_Address, int Ip_Port, double Timeout);
void   SetTCPTimeout(int SocketIndex, double TimeOut);
void   SendAndReceive (int SocketIndex, char *buffer, char *valueRtrn);
void   SendOnly (int SocketIndex, char *buffer, char *valueRtrn);
void   CloseSocket (int SocketIndex);
void   CloseAllSockets (void);
void   ResetAllSockets (void);
char * GetError (int SocketIndex);
