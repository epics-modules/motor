/* File: drvXPSC8.h          */

/* Device Driver Support definitions for motor
 *
 *      Original Author: Jon Kelly
 *
 */

#ifndef INCdrvXPSC8h
#define INCdrvXPSC8h 1

#include "motordrvCom.h"

#define XPSC8_END_OF_RUN_MINUS	0x80000100
#define XPSC8_END_OF_RUN_PLUS	0x80000200
#define XPSC8_NUM_CHANNELS	8
#define XPSC8_COMM_ERR		-1
#define TIMEOUT 		1
#define NOTREF 			42

/*------ This defines the XPSC8 specific property structure */
struct XPSC8axis
{
    int socket;
    int pollsocket;		/* This is the same for all motors to poll!*/
    double currentposition[1];	/* the XPS commands want an array */
    double setpointposition[1];
    double targetposition[1];
    double velocity;
    double accel;
    double minjerktime;	/* for the SGamma function */
    double maxjerktime;    
    double jogvelocity;
    double jogaccel;
    double maxlimit;
    double minlimit;
    double resolution;

    char *ip;
    char *positionername;  /* read in using NameConfig*/
    char *groupname;	   
    int groupstatus;
    int positionererror;
    int moving;		/* 1 = moving! */
    int status;	/* Included to fit in with the camac driver template 
    		Not a value from the XPS controller!*/

};

struct XPSC8controller{
    epicsMutexId XPSC8Lock; 
    struct XPSC8axis axis[XPSC8_NUM_CHANNELS];
    };

/* Global function, used by both driver and device support  */


#endif  /* INCdrvXPSC8h */
