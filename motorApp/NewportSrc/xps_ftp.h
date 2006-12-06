/********************************************************************
 *  ftp.h : header file for FTP functions                           *
 *                                                                  *
 *  Authors : Max Lekeux                                            *
 *                                                                  *
 *  Modifications :                                                 *
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

/* #define VXWORKS */

/******[ includes ]**************************************************/
#ifdef _WIN32
#include <winsock.h>
#include <windows.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#ifdef vxWorks
#include <sockLib.h>
#include "inetLib.h"
#endif
#endif 

#include <string.h>
#include <stdio.h>
#include <stdlib.h>


/******[ defines ]***************************************************/
/* #define DEBUG */

#define IP_SIZE      16    /* size of an IP address */
#define NAME_SIZE    128   /* size for login, password */
#define COMMAND_SIZE 256   /* size of a FTP command string */
#define RETURN_SIZE  1500  /* size of a return (size of standard IP package) */
#define PATH_SIZE    256   /* size of path */


/******[ global variables ]******************************************/

int ftpChangeDir (int, char*);

/******[ prototypes ]************************************************/
/* FTP commands */
int ftpConnect (char*, char*, char*, int*);
int ftpDisconnect (int);
int ftpChangeDir (int, char*);
int ftpRetreaveFile (int, char*);
int ftpStoreFile(int, char*);

/* localy used functions */
int code(char*);
int sendFtpCommandAndReceive (int, char*, char*);
int getPort (int, char*);
void printRecv (char*, int);
