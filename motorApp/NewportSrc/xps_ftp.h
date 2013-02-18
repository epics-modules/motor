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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <osiSock.h>
#include <epicsExport.h>


/******[ defines ]***************************************************/
/* #define DEBUG */

#define IP_SIZE      16    /* size of an IP address */
#define NAME_SIZE    128   /* size for login, password */
#define COMMAND_SIZE 256   /* size of a FTP command string */
#define RETURN_SIZE  1500  /* size of a return (size of standard IP package) */
#define PATH_SIZE    256   /* size of path */

#ifdef __cplusplus
extern "C" {
#endif

/******[ prototypes ]************************************************/
/* FTP commands */
epicsShareFunc int ftpConnect (char*, char*, char*, SOCKET*);
epicsShareFunc int ftpDisconnect (SOCKET);
epicsShareFunc int ftpChangeDir (SOCKET, char*);
epicsShareFunc int ftpRetrieveFile (SOCKET, char*);
epicsShareFunc int ftpStoreFile(SOCKET, char*);

#ifdef __cplusplus
}
#endif
