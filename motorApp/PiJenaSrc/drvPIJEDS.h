/* File: drvPIJEDS.h             */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Ron Sluiter
 *      Current Author: Joe Sullivan
 *      Date: 09/20/2005
 *
 * Modification Log:
 * -----------------
 * .00  09/13/2006  jps  copied from drvPIC838.h
 */

#ifndef	INCdrvPIJEDSh
#define	INCdrvPIJEDSh 1

#include "motordrvCom.h"
#include "asynDriver.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"

#define COMM_TIMEOUT 2		/* Timeout in seconds. */
#define EDS_MAX_RES 3		/* Position resolution (significant digits) */

#define EDS_OUT_EOS  "\r"        /* Command End-Of-Line = CR (0x13) */
#define EDS_IN_STX   "\023"      /* Reply Start (frame character) */
#define EDS_IN_ETX   "\021"      /* Reply End (framing Character)  */

#define EDS_VERSION            1959    /* Driver tested against EDS DSM V1.959 */
#define EDS_MAX_MOTORS           6     /* maximum motors per controller */
#define PIJEDS_NUM_CARDS	10     /* Maximum number of support controllers */

struct PIJEDScontroller
{
    asynUser *pasynUser;	/* asynUser structure */
    int asyn_address;		/* Use for GPIB or other address with asyn */
    CommStatus status;		/* Controller communication status. */
    double drive_resolution[EDS_MAX_MOTORS];
    bool versionSupport;        /* Track supported Versions - include in Report */
    char asyn_port[80];		/* asyn port name */
};

/* Types of position feedback */
#define EDS_FDBK_NONE 0
#define EDS_FDBK_STRAIN 1
#define EDS_FDBK_CAPACITIVE 2      /* Typical */
#define EDS_FDBK_INDUCTIVE 3

/* Controller Status Word */
typedef union
{
    epicsUInt16 All;
    struct
    {
#ifdef MSB_First
        unsigned int nabyte             :8; /* 8-15 */
        unsigned int closeLoop		:1; /* 7 - Closed Loop Mode */
        unsigned int piezoDecontrol     :1; /* 6 - Piezo Voltage Decontroled */
	unsigned int na5		:1; /* 5 - NA. */
        unsigned int openLoop		:1; /* 4 -  Open Loop Mode */
        unsigned int na3		:1; /* 3  NA */
        unsigned int fdbkType	        :2; /* 1&2 - Feedback Type - EDS_FDBK_nnnnn */
        unsigned int motorExist   	:1; /* 0 - Motor plugged into controller */

#else
        unsigned int motorExist   	:1; /* 0 - Motor plugged into controller */
        unsigned int fdbkType	        :2; /* 1&2 - Feedback Type - EDS_FDBK_nnnnn */
        unsigned int na3		:1; /* 3  NA */
        unsigned int openLoop		:1; /* 4 -  Open Loop Mode */
	unsigned int na5		:1; /* 5 - NA. */
        unsigned int piezoDecontrol     :1; /* 6 - Piezo Voltage Decontroled */
        unsigned int closeLoop		:1; /* 7 - Closed Loop Mode */
        unsigned int nabyte             :8; /* 8-15 */

#endif
    } Bits;                                
} EDS_Status_Reg;


/* Function prototypes. */
extern RTN_STATUS PIJEDSSetup(int, int);
extern RTN_STATUS PIJEDSConfig(int, const char *, int);

#endif	/* INCdrvPIJEDSh */
