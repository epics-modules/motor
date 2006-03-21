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
#define XPSC8_NUM_CHANNELS	8	/* Number of axes in one XPS */
#define XPSC8_MAX_CONTROLLERS	2	/* Set to diffractometer number*/
#define XPSC8_COMM_ERR		-1
#define XPSC8_MAX_WAIT		120	/* Cue timeout limit 1min 120*0.5 sec*/
#define XPSC8_QUE_PAUSE_READY	0.2	/* Time the driver waits for further
					   comands to group together before 
					   performing a move from stationary*/
#define XPSC8_QUE_PAUSE_MOVING	0.1	/* Poll time the driver waits to check if
					   the motor is still moving after a 
					   queue has been started*/
					   
#define TIMEOUT 		0.5	/* TCP/IP timeout */
#define NOTREF 			42

/*------ This defines the XPSC8 specific property structure */
struct XPSC8axis
{
    int socket;			/* Each Axis has its own drive socket */
    int pollsocket;		/*socket for drvXPSC8 thread polling */
    int devpollsocket;		/*socket for devXPSC8 thread polling */
    double currentposition;	/* the XPS commands want an array */
    double setpointposition;	/* but the values are then assigned */
    double targetposition;	/* to these variables. */
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

    int groupnumber;	/* (0-7) This axis is a member of which group?*/
    int axisingroup;	/* (0-7) eg the 4th axis in the 2nd group */
};

struct XPSC8group{	/* This structure is for each group not axis */
    double positionarray[XPSC8_NUM_CHANNELS];	/* Used to store queued moves */
    int groupsize;		/* (1-8) Number of members of a group */
    int queuesize;		/* Number of axes in the queue */
    int queueflag;		/* 1 = move/wait loop needs to be started */ 
    };

struct XPSC8controller{
    epicsMutexId XPSC8Lock; 
    struct XPSC8axis axis[XPSC8_NUM_CHANNELS];	/* maximum of 8 axes */
    struct XPSC8group group[XPSC8_NUM_CHANNELS]; /* maximum of 8 groups */
    };

/* Global function, used by both driver and device support  */


#endif  /* INCdrvXPSC8h */
