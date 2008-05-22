/********************************************************************
 *  ftp.c : source file for FTP functions for XPS motion Controller *
 *                                                                  *
 *  Authors : Max Lekeux                                            *
 *                                                                  *
 *  Modifications history :                                         *
 *       - 23aug06,ML : creation                                    *
 *                                                                  *
 *  Warning : These functions were created to work with the XPS FTP *
 *            server. They prove the possibility to use standard    *
 *            FTP functions. They are developped to be              *
 *            cross-platform (Linux, Unix, Windows, VxWorks)        *
 *                                                                  *
 *  Warning for Visual C++ : don't forget to add wsock32.lib to the *
 *            link. To do so add it in Menu Project->Settings       *
 *            Tab Link -> Objects/Library modules                   *
 ********************************************************************/


/******[ includes ]**************************************************/
#include "xps_ftp.h"

/* local functions */
static int code(char*);
static int sendFtpCommandAndReceive (int, char*, char*);
static int getPort (int, char*);
static void printRecv (char*, int);


/******[ ftpConnect ]************************************************/
int ftpConnect (char* ip, char* login, char* password, int* socketFD)
{
  char command[COMMAND_SIZE];
  char returnString[RETURN_SIZE];
  struct sockaddr_in sockAddr;
  int sockFD, receivedBytes;

  memset(&sockAddr, 0, sizeof(sockAddr));

  sockFD = socket(AF_INET, SOCK_STREAM, 0);

  sockAddr.sin_family = AF_INET;
  sockAddr.sin_port = htons(21);
  sockAddr.sin_addr.s_addr = inet_addr(ip);

  if (connect(sockFD, (struct sockaddr *)&sockAddr, sizeof(sockAddr)) < 0) 
    return -1;

  do {
    receivedBytes = recv(sockFD, returnString, RETURN_SIZE, 0);
  }
  while (strchr(returnString,'\n')==NULL);

  /* login */
  sprintf(command, "USER %s", login);
  if (-1 == sendFtpCommandAndReceive (sockFD, command, returnString))
    return -2;
    
  sprintf(command, "PASS %s", password);
  if (-1 == sendFtpCommandAndReceive (sockFD, command, returnString))
    return -3;

  sprintf(command, "PASV");
  sendFtpCommandAndReceive (sockFD, command, returnString);
  sprintf(command, "TYPE I");
  sendFtpCommandAndReceive (sockFD, command, returnString);

  *socketFD = sockFD;

  return 0;
}


/******[ ftpDisconnect ]*********************************************/
int ftpDisconnect (int socketFD)
{
#ifdef _WIN32
  return closesocket(socketFD);
#else
  return close(socketFD);
#endif
}


/******[ ftpChangeDir ]**********************************************/
int ftpChangeDir (int socketFD, char* destination)
{
  char command[COMMAND_SIZE];
  char returnString[RETURN_SIZE];
  
  sprintf(command, "CWD %s", destination);
  if (-1 == sendFtpCommandAndReceive (socketFD, command, returnString))
    return -1;
  
  return 0;
}


/******[ ftpRetrieveFile ]*******************************************/
int ftpRetrieveFile(int socketFD, char *filename)
{
  int port_rcv, socketFDReceive, i;
  struct sockaddr_in adr_rcv;
  char ip[IP_SIZE];
  char command[COMMAND_SIZE];
  char returnString[RETURN_SIZE];
  FILE *file;

  memset(&adr_rcv, 0, sizeof(adr_rcv));
     
  port_rcv = getPort(socketFD, ip); 
  
  socketFDReceive = socket (AF_INET, SOCK_STREAM, 0);
  
  adr_rcv.sin_family = AF_INET;
  adr_rcv.sin_addr.s_addr = inet_addr(ip);
#ifdef _WIN32
  adr_rcv.sin_port = htons((u_short)port_rcv);
#else
  adr_rcv.sin_port = htons(port_rcv);
#endif

  if (0 > connect (socketFDReceive, (struct sockaddr *) &adr_rcv, sizeof(adr_rcv)))
    { 
      fprintf(stderr,"Cound not connect to FTP server to retrieve file %s\n", filename);
      return -1;
      
    }
  
  /* send command */
  sprintf(command, "RETR %s", filename);
  if (-1 == sendFtpCommandAndReceive (socketFD, command, returnString))
    return -1;

  /* Check there is no problem with file */
  if (code(returnString) == 550)
	return -1;

  /* open file on localhost */
  if (NULL == (file = fopen(filename, "wb")))
    {
      fprintf(stderr,"Cound not open file \"%s\" for writing on local host\n", filename);
      return -1;
    }

  do
    {
      i = recv(socketFDReceive,returnString, RETURN_SIZE,0);
      fwrite(returnString, sizeof(char), i, file);
    }
  while(i!=0);
   
#ifdef _WIN32
  closesocket(socketFDReceive);
#else
  close(socketFDReceive);
#endif

  fclose(file);

  i = recv(socketFD, returnString, RETURN_SIZE, 0);     /* read "226 Transfer complete." */

#ifdef DEBUG
  printf(" -> ");
  printRecv(returnString, i);
#endif

  return 0;
}


/******[ ftpStoreFile ]**********************************************/
int ftpStoreFile(int socketFD, char *filename)
{
  int port_snd, socketFDSend, i;
  struct sockaddr_in adr_snd;
  char ip[IP_SIZE];
  char command[COMMAND_SIZE];
  char returnString[RETURN_SIZE];
  FILE *file;

  memset(&adr_snd, 0, sizeof(adr_snd));
     
  port_snd = getPort(socketFD, ip); 
  
  socketFDSend = socket (AF_INET, SOCK_STREAM, 0);
  
  adr_snd.sin_family = AF_INET;
  adr_snd.sin_addr.s_addr = inet_addr(ip);
#ifdef _WIN32
  adr_snd.sin_port = htons((u_short)port_snd);
#else
  adr_snd.sin_port = htons(port_snd);
#endif

  if (0 > connect (socketFDSend, (struct sockaddr *) &adr_snd, sizeof(adr_snd)))
    { 
      fprintf(stderr,"Cound not connect to FTP server to retrieve file %s\n", filename);
      return -1;
      
    }
  
  /* send command */
  sprintf(command, "STOR %s", filename);
  if (-1 == sendFtpCommandAndReceive (socketFD, command, returnString))
    return -1;

  /* open file on localhost */
  if (NULL == (file = fopen(filename, "rb")))
    {
      fprintf(stderr,"Cound not open file \"%s\" for reading on local host\n", filename);
      return -1;
    }

  do
    {
	  i = fread(returnString, sizeof(char), RETURN_SIZE, file);
      send(socketFDSend,returnString, i, 0);
    }
  while(i != 0);
   
#ifdef _WIN32
  closesocket(socketFDSend);
#else
  close(socketFDSend);
#endif

  fclose(file);

  i = recv(socketFD, returnString, RETURN_SIZE, 0);     /* read "226 Transfer complete." */

#ifdef DEBUG
  printf(" -> ");
  printRecv(returnString, i);
#endif

  return 0;
}


/******[ code ]******************************************************/
static int code (char *str)
{
  char tmp[3];
  strncpy(tmp, str, 3);
  return atoi(tmp);
}


/******[ sendFtpCommandAndReceive ]**********************************/
static int sendFtpCommandAndReceive (int socketFD, char* command, char* returnString)
{
	int receivedBytes;
	char str_rec[RETURN_SIZE];
  
#ifdef DEBUG
	printf("%s\n", command);
#endif

	sprintf(command, "%s\n", command);

	send (socketFD, command, strlen(command), 0);
	receivedBytes = recv(socketFD, str_rec, RETURN_SIZE, 0);
   
#ifdef DEBUG
	printf(" -> ");
	printRecv (str_rec, receivedBytes);  
#endif

	str_rec[receivedBytes] = '\0';

	switch (code(str_rec))
    {
	case 530:
		fprintf(stderr, "Connection error\n"); return -1;
	case 421:
		fprintf(stderr, "Connection time out\n"); return -1; 
	default:
		break;
    }

	if (str_rec[3] == '-')        /* Reading a "-" in third position means that */
    {                           /* the server wants to informations with one packet */  
                                /* for each line. */ 
		char tmp_code[3];  
		int i, j, found_end = 0;      
      
		strncpy(tmp_code, str_rec, 3);
		while (!found_end)        /* It is the end if we receive the code followed by a space */ 
		{
			j = 0;
			while ((j+4) < strlen(str_rec))
			{
				if (tmp_code[0] == str_rec[j])     /* search the first letter of the code */
				{
					if ((tmp_code[1] == str_rec[j+1]) 
						&& (tmp_code[2] == str_rec[j+2]) 
						&& (str_rec[j+3] == ' ')) 
					{                            /* compare the rest + space */
						found_end = 1;
						break;
					}
					else
						j++;
				}
				else
					j++;
			}

			if ((j+4) >= strlen(str_rec)) {       /* Last line not found yet, keep going */
				i = recv(socketFD, str_rec, RETURN_SIZE, 0);
				str_rec[i] = '\0';
			}
		}
    }

	strncpy(returnString, str_rec, RETURN_SIZE);
	return 1;
}


/******[ getPort ]***************************************************/
static int  getPort (int socketFD, char* ip)
{
  char command[COMMAND_SIZE], returnString[RETURN_SIZE], tmp[RETURN_SIZE];
  int count, i, j, port;

  strcpy(command, "PASV");
  sendFtpCommandAndReceive (socketFD, command, returnString);
  
  i = 27;
  count = 0;
  while (count < 4)
    {
      if (returnString[i] == ',')
	{
	  count++;
	  if (count < 4)
	    ip[i++-27] = '.';
	}
      else
	{
	  ip[i-27] = returnString[i];
	  i++;
	}
    }
  ip[i-27] = '\0';

  j = ++i;
  while (returnString[i] != ',') 
    {
      tmp[i-j] = returnString[i];
      i++;
    }
  
  tmp[i-j] = '\0';
  port = (atoi (tmp) * 256);
  
  j = ++i;
  while (returnString[i] != ')')
   {
      tmp[i-j] = returnString[i];
      i++;
    }
  
  tmp[i-j] = '\0';
  port += atoi (tmp);
 
  return port;
}


/******[ printRecv ]**************************************************/
static void printRecv (char *str, int i)   
{
  int j;
  for (j = 0; j < i; j++)
    printf("%c", str[j]); 
}
