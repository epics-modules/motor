/* This function alows you to run a tcl script from within the 
    Public/Scripts directory of the Newport XPS with the ip
    address contained in the string below.
    
    ->tclcall(tclScriptName.tcl,arbProcessName,tclScriptArgs)
    
    
    
    By Jon Kelly 2005					    */


#include <stdio.h>
#define  epicsExportSharedSymbols
#include <shareLib.h>
#include "tclCall.h"
#include "XPS_C8_drivers.h"
#include "Socket.h"

#define TIMEOUT 		1

static int getsocket(void);

epicsShareFunc void tclcall(char const *name,char const *taskName,char const *args){

	int status = 0;
	int socket;
	
	socket = getsocket();
	status = TCLScriptExecute(socket,(char *)name,
			(char*)taskName,(char *)args);
	
	printf("TCL Call Status %i\n",status);
	if (status < 0) {
	   printf("Error Name called %s, Task Name %s, Args %s\n",\
	   					name,taskName,args);
	}  					
	
	return;
}
		
static int getsocket(void)
{
	char ipchar[] = "164.54.160.124";
	int socket = 0;
	int port = 5001;

	printf("XPS ip = %s\n",ipchar);
	
	socket = TCP_ConnectToServer(ipchar,port,TIMEOUT);
	if (socket < 0) 
            printf(" Error TCP_ConnectToServer\n");
	return (socket);
}
