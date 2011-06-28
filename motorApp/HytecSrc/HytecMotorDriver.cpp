/********************************************************************************/
/*    H   H Y   Y TTTTT EEEEE  CCC       HYTEC ELECTRONICS LTD                  */
/*    H   H  Y Y    T   E     C          5 Cradock Road,                        */
/*    HHHHH   Y     T   EEE   C          Reading, Berks.    Tel: 0118 9757770	*/
/*    H   H   Y     T   E     C          RG2 0JT            Fax: 0118 9757566	*/
/*    H   H   Y     T   EEEEE  CCC       Web: www.hytec-electronics.co.uk       */
/********************************************************************************/
/********************************************************************************/
/*    _____________________________________________________________________     */
/*    | H Y T E C   8 6 0 1   S T E P   M O T E R   A s y n   D r i v e r |     */
/*    ---------------------------------------------------------------------     */
/*                                                                              */
/*  Source file name      :-      HytecMotorDriver.c                            */
/*                                                                              */
/*  Initial creation date :-      29-Mar-2011                                   */
/*                                                                              */
/*  Original Developers   :-      Jim Chen. Hytec Electronics Ltd               */
/*                                                                              */
/********************************************************************************/
/*                                                                              */
/*  Description :- This is the "model 3" asyn motor driver for Hytec 8601  		*/
/*                 Stepper Motor IP module. The code is based on original   	*/
/*  			   Hytec drvHy8601asyn.c driver and also Mark Rivers' 			*/
/*				   ACRMotorDriver. 					                          	*/
/*                                                                              */
/*                                                                              */
/*  (C)2011 Hytec Electronics Ltd.                                              */
/*                                                                              */
/********************************************************************************/
/*                                                                              */
/*  Revision history: (comment and initial revisions)                           */
/*                                                                              */
/*  vers.       revised              modified by         date                   */
/*  -----       -----------        ----------------   ---------------           */
/*  2.0         Continued version  		Jim Chen       	 29/03/2011             */
/*  	        The main contents of this driver are the same as the original   */
/*  			Hytec drvHy8601asyn.c driver but in "model 3" form as defined   */
/*              by Mark Rivers. Hence it starts from version 2.0.				*/
/*  2.1         New interfaces 			Jim Chen       	 04/04/2011             */
/*  	        This version follows the asyn motor model 3 latest interface	*/
/*				changes that include:											*/
/*				a).New asynMotorAxis base class									*/
/*				b).Moves the axis specific functions from the motor controller 	*/
/*				   class to individual axis class								*/
/*				c).Changes asynMotorDriver.cpp name to asynMotorController.cpp.	*/
/*  2.2         Bugs fix 				Jim Chen       	 14/04/2011             */
/*  	        Fixed setPosition bug											*/
/*				Added firmware version parameter								*/
/*  2.3         Bugs fix 				Jim Chen       	 21/04/2011             */
/*  	        Fixed detecting encoder logic wrong bug							*/
/*  2.4         Bugs fix 				Jim Chen       	 24/06/2011             */
/*  	        Fixed wrong ioc shell command function name						*/
/*                                                                              */
/********************************************************************************/

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <epicsFindSymbol.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <ellLib.h>
#include <epicsMessageQueue.h>

#include <drvSup.h>
#include <epicsExport.h>
#include <devLib.h>
#include <drvIpac.h>
#include <iocsh.h>

#include "HytecMotorDriver.h"

static void ISR_8601(int pDrv);
static void intQueuedTask( void *pDrv );

#define MAX_MESSAGES  100           /* maximum number of messages */
#define ENCODER_HARD_RATIO    0.25 	/* encoder hardware ratio, it is 1/4 since the quadrature encoder */


// Constructor
HytecMotorController::HytecMotorController(const char *portName, int numAxes, 
			double movingPollPeriod, double idlePollPeriod, int cardnum, 
			int ip_carrier, int ipslot, int vector, int useencoder,
	        double encoderRatio0, double encoderRatio1, double encoderRatio2,
	        double encoderRatio3)
    :   asynMotorController(portName, numAxes, NUM_HYTEC_PARAMS, 
            asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask, 
            asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
            1, // autoconnect
            0, 0)  // Default priority and stack size
{
    int axis;
	double encoderRatio[4];
    asynStatus status;
    HytecMotorAxis *pAxis;
    static const char *functionName = "HytecMotorController";

	encoderRatio[0] = encoderRatio0;
	encoderRatio[1] = encoderRatio1;
	encoderRatio[2] = encoderRatio2;
	encoderRatio[3] = encoderRatio3;

    // Check the validity of the arguments
    // Is Interrupt Vector Valid...
    if (vector <= 0 || vector > 255) 
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: illegal interrupt vector number. must be in range [1..255]\n",
            driverName, functionName);
//	   printf("ERROR! illegal interrupt vector number %d !\n", vector);
    }

    if (numAxes < 1 ) numAxes = 1;
    this->numAxes = numAxes;
    this->card  = cardnum;
    this->ip_carrier = ip_carrier;
    this->ipslot = ipslot;
    this->vector = vector & 0xFF;
    this->ip_carrier = ip_carrier;
	this->useencoder = useencoder;
    
    // Create controller-specific parameters
    createParam(HytecPowerControlString,	asynParamInt32,  &this->HytecPowerControl_);
    createParam(HytecBrakeControlString,    asynParamInt32,  &this->HytecBrakeControl_);
    createParam(HytecFirmwareString,   		asynParamInt32,  &this->HytecFWVersion_);
    createParam(HytecMoveAllString,    		asynParamInt32,  &this->HytecMoveAll_);

    if ((status = SetupCard()) != asynSuccess) 
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: setup motor IP card failed.\n",
            driverName, functionName);
    }

    for (axis=0; axis<numAxes && status == asynSuccess; axis++ ) 
    {
    	pAxis = new HytecMotorAxis(this, axis, encoderRatio[axis], vector & 0xFF);
    }
    
	// create a queue task for dealing interrupts
    this->intMsgQId = epicsMessageQueueCreate(MAX_MESSAGES, (this->numAxes + 2)*sizeof(int));
    if (epicsThreadCreate("drvHy8601intQueuedTask",
                         epicsThreadPriorityLow,
                         epicsThreadGetStackSize(epicsThreadStackMedium),
                         (EPICSTHREADFUNC)intQueuedTask,
                         (void *) this) == NULL)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: drvHy8601intQueuedTask epicsThreadCreate failure.\n",
            driverName, functionName);
    }

  	startPoller(movingPollPeriod, idlePollPeriod, 2);
}

// set up IP card
asynStatus HytecMotorController::SetupCard()
{
    char *regbase;
	int st;
    static const char *functionName = "SetupCard";

    /* Register the card in A16 address space */
    regbase=(char*)ipmBaseAddr( this->ip_carrier,
			 this->ipslot,
			 ipac_addrIO);

    /* If failed to register card... */
    if (regbase == NULL) 
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: Cannot register Hy8601 A16 device.\n",
            driverName, functionName);
		return asynError;
    }

    /* Memory Check OK, so check PROM */
    /* JSC. checkprom returns 1 = ok, 0 = failed. */
    if (checkprom((char*)regbase+PROM_OFFS, PROM_MODEL)!=1) 
    {
	   return asynError;
    }

    /* Setup Interrupts. THIS SEEMS NOT RIGHT. 8601 HAS 4 AXIS AND EACH ONE HAS A VECTOR SO 4 ISRs ARE NEEDED?????????????? */
    st=ipmIntConnect(this->ip_carrier,
			 this->ipslot,
			 this->vector,
			 &ISR_8601,
			 (int)this);
      
    /* If Interrupts NOT Setup OK */
    if (st!=OK) 
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: intConnect ERROR.\n",
            driverName, functionName);
	   return asynError;
    }

    /* Setup Carrier Card */
    ipmIrqCmd(this->ip_carrier,
	      this->ipslot,
	      0,
	      ipac_irqEnable);

    this->regbase = regbase;
    return asynSuccess;
}

/**********************************************************

    NAME:	checkprom()

    Description:

    Function -	
	1. Check for "VITA4 " string. 
	2. Check for Hytec ID.
	3. Check for IP Model (0x8601).
	4. Display Debug Information.
	5. Return whether Check PROM Successful.
 
    Parameters:

    Inputs:         
	
	pr       - The PROM base address (1st address of PROM).
	expmodel - Expected IP Model as a Hex Value (i.e. 0x8601) 
	verbose  - TRUE = Display Debug Information as you go.

    Outputs:        None.
    Return:      (ishytec && ismodel)   

	TRUE  - Everything OK.
	FALSE - Either the Hytec ID or IP Model number is wrong.


    Notes:

    Programmer(s):  Darrell Nineham
                    Walter Scott            
                    Steve Hunt

**********************************************************

Revision History
----------------

Date            By      Version     Descripition
----            --      -------     ------------
29-MAR-2011     JSC     1.1.1.0     Intial Version.
    
**********************************************************/
int HytecMotorController::checkprom(char *pr,int expmodel)
{
	#define PR_ID    0x06
	#define PR_MODN  0x0a
	#define PR_REVN  0x0c
	#define PR_DRID1 0x10
	#define PR_DRID2 0x12
	#define PR_FLAGS 0x14
	#define PR_BYTES 0x16
	#define PR_SERN  0x1a
	#define HYTECID  0x00800300

	char*	hytecstr="  (HyTec Electronics Ltd., Reading, UK)";
	char	lstr[7];
	int		manid;
	epicsUInt16	modelnum;
	int		ishytec,ismodel,strok, ver;
  
	/* Begin */

	/* Debug - Display the passed Address */ 
	printf("PROM CONTENTS AT: %p\n",pr);
	/* Read the first 6 Characters from the passed Address */ 
	strncpy(lstr, (char*)pr, 6);
	/* Convert it to a String by Null terminating it */
	lstr[6]=0;

	/* Compare to Expected String (i.e. "VITA4 ") */
	strok = (strcmp(lstr, IP_DETECT_STR) == 0);

	printf("PROM header: '%6s'\n",lstr);
	/* Set manid to PROM Address of the Hytec ID */
	manid    = *((int*)(pr+PR_ID));
	/* Set ishytec to TRUE if Hytec ID is as expected */
	ishytec  = (manid ==HYTECID);
	/* Set modelnum to PROM Address of the IP Model */
	modelnum = *((epicsUInt16*)(pr+PR_MODN));
	/* Set ismodel to TRUE if IP Model is as expected */
	ismodel  = (modelnum==expmodel);
	
	/* Display Assorted Information from Reading the PROM */
	printf("PROM manufacturer ID: 0x%08X",manid);
	if (ishytec) printf(hytecstr);

	printf("\nPROM model #: 0x%04hx, rev. 0x%04hx, serial # %hu\n",
	modelnum,
	*((short int*)(pr+PR_REVN)),
	*((short int*)(pr+PR_SERN)));
    
	printf("PROM driver ids: 0x%04hx, 0x%04hx\n",
	*((short int*)(pr+PR_DRID1)),
	*((short int*)(pr+PR_DRID2)));

	ver = *((short int*)(pr+PR_REVN));
	ver = ver/4096*1000 + (ver % 4096)/256*100 + ((ver % 4096) % 256)/16*10 + ((ver % 4096) % 256) % 16;
    setIntegerParam( HytecFWVersion_, ver);
   
	printf("PROM flags: 0x%04hx\nPROM number of bytes used: 0x%04hx (%hd)\n\n",
	*((short int*)(pr+PR_FLAGS)),
	*((short int*)(pr+PR_BYTES)),
	*((short int*)(pr+PR_BYTES)));
	

	/* Debug - Print error message if string is NOT as expected */
	if (!strok)
	{
		printf("PROM INVALID PROM HEADER; EXPECTED '%s'\n",
		       IP_DETECT_STR);
	}
	
	/* Debug - Print error message if Hytec ID is NOT as expected */
	if (!ishytec)
	{
		printf("PROM UNSUPPORTED MANUFACTURER ID;\nPROM EXPECTED 0x%08X, %s\n",
		HYTECID,hytecstr);
	}
		
	/* Debug - Print error message if IP Model is NOT as expected */
	if(!ismodel)
	{
		printf("PROM UNSUPPORTED BOARD MODEL NUMBER: EXPECTED 0x%04hx\n",
		expmodel);
	}
	
	return (/* SCO strok && */ ismodel);
}

// report
void HytecMotorController::report(FILE *fp, int level)
{
    int axis;
    HytecMotorAxis *pAxis;

    fprintf(fp, "Hytec motor driver %s, numAxes=%d\n", this->portName, this->numAxes);

    if (level > 0) {
        for (axis=0; axis<this->numAxes; axis++) {
            pAxis = getAxis(axis);
            fprintf(fp, "  axis %d\n"
                        "    encoder ratio = %f\n", 
                pAxis->axisNo_, pAxis->encoderRatio);

        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

// get axis
HytecMotorAxis * HytecMotorController::getAxis(asynUser *pasynUser)
{
  	return dynamic_cast<HytecMotorAxis*>(asynMotorController::getAxis(pasynUser));
}
    
HytecMotorAxis * HytecMotorController::getAxis(int axisNo)
{
  	return dynamic_cast<HytecMotorAxis*>(asynMotorController::getAxis(axisNo));
}
    
// Int32 read interface
asynStatus HytecMotorController::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    HytecMotorAxis *pAxis = getAxis(pasynUser);
	double dvalue;
	bool moving;
//    static const char *functionName = "readInt32";
    
    if ((function == motorPosition_) || (function == motorEncoderPosition_))
    {
		//update all values and status
		pAxis->poll(&moving);		
	    getDoubleParam(pAxis->axisNo_, function, &dvalue);
	    *value = (int)dvalue;
	}else if(function == HytecFWVersion_)
	{
	    getIntegerParam( HytecFWVersion_, value);
	}else
        // Call base class call its method (if we have our parameters check this here)
   	    status = asynPortDriver::readInt32(pasynUser, value);    
    return status;	
}

// Int32 write interface
asynStatus HytecMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    HytecMotorAxis *pAxis = getAxis(pasynUser);
//    static const char *functionName = "writeInt32";
    
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(pAxis->axisNo_, function, value);
    
    if(function == HytecPowerControl_)
	{
		if(value)
	   		CSR_SET(pAxis->chanbase, CSR_AUX1);
		else
       		CSR_CLR(pAxis->chanbase, CSR_AUX1);
	} else if(function == HytecBrakeControl_)
	{
		if(value)
       		CSR_CLR(pAxis->chanbase, CSR_AUX2);
		else
	   		CSR_SET(pAxis->chanbase, CSR_AUX2);
	} else if(function == HytecMoveAll_)
	{
		
	} else
        /* Call base class call its method (if we have our parameters check this here) */
        status = asynMotorController::writeInt32(pasynUser, value);
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axisNo_);
    return status;
}

// Float64 write interface. This is currently useless.
asynStatus HytecMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    HytecMotorAxis *pAxis = getAxis(pasynUser);
//    static const char *functionName = "writeFloat64"; 
    
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(pAxis->axisNo_, function, value);
	
	/*
	 * The following part are not implemented in this driver but is retained here
	 * in case of future requirements.
	 */
//    if(function == motorResolution_)
//	{
	    /* Calculate the resolution. it is initialised as 1.0. 
            previous_resolution = pAxis->resolution;
            pAxis->resolution = value;
            pAxis->softLowLimit = pAxis->resolution * pAxis->softLowLimit / previous_resolution;     
            pAxis->softHighLimit = pAxis->resolution * pAxis->softHighLimit / previous_resolution;  */
//	}
//    else if(function == motorEncRatio_)
//	{	
	    /* Calculate the encoder ratio and store. Note, encoder ratio is initialised as 1/4 since 
		 * the hardware design counts 4 of each encoder pulse. encoderRatio is not used for 8601 
		 * after consulting to Ron Sluiter on 21/01/2001 
            pAxis->encoderRatio = ENCODER_HARD_RATIO * value; */
//    }
//    else if(function == motorLowLimit_)
//	{
    	// Calculate the soft low limit to engineering unit. softLowLimit is not used for 8601 
        //pAxis->softLowLimit = pAxis->resolution * value;
//	}
//    else if(function == motorHighLimit_)
//	{
    	// Calculate the soft high limit to engineering unit. softHighLimit is not used for 8601 
        //pAxis->softHighLimit = pAxis->resolution * value;
//	}
//	else
	    /* Call base class call its method (if we have our parameters check this here) */
    	status = asynMotorController::writeFloat64(pasynUser, value);
    
    /* Do callbacks so higher layers see any changes */
    pAxis->callParamCallbacks();
    return status;
}

// ISR8601 Routines to handle interrupts
static void ISR_8601(int pDrv)
{
    HytecMotorController *pController = (HytecMotorController*) pDrv;
	HytecMotorAxis * pAxis;
	volatile char * chanbase;
	epicsMessageQueueId	qID;
    int data[4];
    int axis;
	int numOfAxes;

	numOfAxes = pController->getNumAxes();
    for (axis = 0; axis < numOfAxes; axis++) 
    {	
		pAxis = pController->getAxis(axis);
		chanbase = pAxis->getChanbase();
        data[axis] = GET_REG(chanbase, REG_CSR);
        SET_REG(chanbase, REG_INTMASK, 0);             /* disable interrupt */
    }

    /* wake up INT queue */
	qID = pController->getINTMsgQID();
    if (epicsMessageQueueTrySend(qID, data, sizeof(data)) == 0)
        pController->increaseMsgSent();
    else
        pController->increaseMsgFail();
}

// Interrupt queue task
static void intQueuedTask( void *pDrv )
{
    HytecMotorController *pController = (HytecMotorController*) pDrv;
	HytecMotorAxis * pAxis;
	epicsMessageQueueId	qID;
    int data[4];
    int axis;
	int numOfAxes, ipslot, ipcarrier;

	qID = pController->getINTMsgQID();
	numOfAxes = pController->getNumAxes();
	ipslot = pController->getIPSlot();
	ipcarrier = pController->getIPCarrier();

    while(1) 
    {
        /* Wait for event from interrupt routine */
        epicsMessageQueueReceive(qID, data, sizeof(data));
                 
        for (axis = 0; axis < numOfAxes; axis++) 
        {
			pAxis = pController->getAxis(axis);
           	pController->drvHy8601GetAxisStatus( pAxis, data[axis] );
        }

        /* re-enable interrupt. for Linux IOCs */
        ipmIrqCmd(ipcarrier, ipslot, 0, ipac_irqEnable);
    }
}

// Interrupt queue task updates 
void HytecMotorController::drvHy8601GetAxisStatus( HytecMotorAxis *pAxis, int csr_data )
{
    this->lock();

    setIntegerParam( pAxis->axisNo_, motorStatusDone_, 		((csr_data & CSR_DONE) != 0 ) );
    setIntegerParam( pAxis->axisNo_, motorStatusHighLimit_, ((csr_data & CSR_MAXLMT) != 0 ) );
    setIntegerParam( pAxis->axisNo_, motorStatusHome_,    	((csr_data & CSR_HOMELMT) != 0 ) );

    if (csr_data & CSR_HOMELMT) CSR_CLR(pAxis->chanbase, CSR_HOMESTOP);                     /* after home limit is reported, clear Stop at home */

    setIntegerParam( pAxis->axisNo_, motorStatusProblem_,   ((csr_data & CSR_DRVSTAT) != 0) );
    setIntegerParam( pAxis->axisNo_, motorStatusLowLimit_,  ((csr_data & CSR_MINLMT) != 0) );
    callParamCallbacks(pAxis->axisNo_);

    this->unlock();
}

int HytecMotorController::getNumAxes()
{
	return numAxes;
}

int HytecMotorController::getIPCarrier()
{
	return ip_carrier;
}

int HytecMotorController::getIPSlot()
{
	return ipslot;
}

epicsMessageQueueId HytecMotorController::getINTMsgQID()
{
	return intMsgQId;
}

void HytecMotorController::increaseMsgSent()
{
	messagesSent++;
}

void HytecMotorController::increaseMsgFail()
{
	messagesFailed++;
}


/**************************************************************************************
 *
 *    							Motor Axis Methods
 *
 *************************************************************************************/
// Motor Axis methods
HytecMotorAxis::HytecMotorAxis(HytecMotorController *pC, int axisNo, double ratio, int vector)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), vector(vector), encoderRatio(ratio)
{
	InitialiseAxis();
}

int HytecMotorAxis::getVector()
{
	return vector;
}

volatile char *HytecMotorAxis::getChanbase()
{
	return chanbase;
}


// initialise axis
asynStatus HytecMotorAxis::InitialiseAxis()
{
    volatile char *chanbase;

    // Required for the motor axis infrastructure
	this->absPosition = 0;											// initialise the software counter to 0 to be in line with the reset which clears the absolute position 
	this->desiredMove = 0;											// clear desired position 
    this->resolution = 1.0;                             			// initial resolution is 1 
    this->softLowLimit = 0.0;                           			// initialise soft low limit to 0 
    this->softHighLimit = 0.0;                          			// initialise soft high limit to 0 
	this->times = 0;

    // Initialisation specific to this model of controller
    chanbase = this->chanbase = pC_->regbase + REG_BANK_OFFS + this->axisNo_ * REG_BANK_SZ;
                        
    // Reset Card 
    CSR_SET(chanbase, CSR_RESET);
    CSR_CLR(chanbase, CSR_RESET);

	/* Note: the useencoder in configure overrides the encoder detection. So if the user decides not 
	 * to use encoder by setting the useencoder param to 0 in the configure call, even there is 
	 * an encoder present, the software will not use the encoder. If the useencoder is 1, then
	 * driver will use encoder but if the encoder is not present, no value will be written to the position register */

	this->useencoder = ((pC_->useencoder & (1<<this->axisNo_)) == 0) ? 0 : 1;
	
   /* check use encoder set by the configure */
    if(this->useencoder)
        CSR_SET(chanbase, CSR_ENCODUSE);

    /* enable interrupt, set vector */
    /* Note, all 4 axis of 8601 have been set the same vector. 
     * It is the ISR's responsibility to check which axis generates the interrupt at runtime.
     * If the mask of individual axis needs to be set differently, extra interface should 
     * be created, but they should still share the same ISR routine. */
    CSR_SET(chanbase, CSR_INTEN);
    SET_REG(chanbase,REG_INTVECTOR,this->vector);

    CSR_SET(chanbase, CSR_CRASHSTOP);
    epicsThreadSleep( 0.1 );
    CSR_CLR(chanbase, CSR_CRASHSTOP);

    return asynSuccess;
}	

// Move axis
asynStatus HytecMotorAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
//    static const char *functionName = "moveAxis";
    asynStatus status = asynSuccess;
    double result;
    int currpos;

	if (min_velocity != 0) 
   	{
       	/* Set start speed register, min_velocity is steps/s */
       	SET_REG(this->chanbase, REG_STARTSTOPSPD, (int)floor(min_velocity));
   	}
   	if (max_velocity != 0) 
   	{
       	/* Set high speed register, max_velocity is steps/s */
       	SET_REG(this->chanbase, REG_HIGHSPD, (int)floor(max_velocity));
   	}
   	if (acceleration >= 64) 
   	{
       	/* Set ramp register, acceleration is steps/s/s */
       	SET_REG(this->chanbase, REG_RAMPRATE, (int)floor(acceleration));
   	}

   	if (relative) 
   	{
       	/* Write to step counter register abs(position)*/
       	SET_REG(this->chanbase, REG_STEPCNTLO, (abs((int)floor(position))) & 0xFFFF);
       	SET_REG(this->chanbase, REG_STEPCNTHI, (abs((int)floor(position))) >> 16); 
       	/* Write dir flag based on relative position */
       	if (position >= 0) 
       	{
	      	CSR_SET(this->chanbase, CSR_DIRECTION);
       	}else 
       	{
	      	CSR_CLR(this->chanbase, CSR_DIRECTION);
       	}
	   	this->desiredMove = position;												/* store desired position */
   	}else 
   	{
       	if(!this->useencoder) 									/* if encoder is not used, read the absolute position from register */
	   	{
	       	currpos = GET_REG(this->chanbase, REG_CURRPOSHI) << 16;
	       	currpos += GET_REG(this->chanbase, REG_CURRPOSLO);
	   	}
       	else                     								/* otherwise, current position is from the software variable */
           	currpos = (int)this->absPosition; 

	   	/* calculate steps to move */
      	result = position - currpos;
	   	this->desiredMove = result;												/* store desired position */

       	/* Write to step counter abs(result) */
       	SET_REG(this->chanbase, REG_STEPCNTLO, (abs((int)floor(result))) & 0xFFFF);
       	SET_REG(this->chanbase, REG_STEPCNTHI, (abs((int)floor(result))) >> 16); 
       	/* Write dir flag based on result */
       	if (result >= 0) 
       	{
	      	CSR_SET(this->chanbase, CSR_DIRECTION);
       	}else 
       	{
	      	CSR_CLR(this->chanbase, CSR_DIRECTION);
       	}
   	}

/* int csr = GET_REG(this->chanbase, REG_CSR); 
 printf("rel=%d askp=%d currp=%d, absp=%d  res=%d, csr=%x acce=%d minsp=%d maxsp=%d\n", relative, (int)position, (int)currpos, (int)this->absPosition, (int)result, csr, (int)floor(acceleration),  (int)floor(min_velocity),  (int)floor(max_velocity));*/  /* for debug  */

   	/* Write GO bit */
   	this->times = 0;											/* clear done bit flag at the beginning of every move */
   	CSR_SET(this->chanbase, CSR_GO);
   	SET_REG(this->chanbase,REG_INTMASK,DONE_INT);             	/* to enable interrupt */

	pC_->lock();
   	setIntegerParam(pC_->motorStatusDone_, 0);
   	callParamCallbacks();
	pC_->unlock();

    return status;
}

// Move home
asynStatus HytecMotorAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards)
{
    asynStatus status = asynSuccess;
//    static const char *functionName = "homeAxis";

   	if (min_velocity != 0)  
   	{
       	/* Set start speed register, min_velocity is steps/s */
       	SET_REG(this->chanbase, REG_STARTSTOPSPD, (int)floor(min_velocity));
   	}
   	if (max_velocity != 0) 
   	{
       	/* Set high speed register, max_velocity is steps/s */
       	SET_REG(this->chanbase, REG_HIGHSPD, (int)floor(max_velocity));
   	}
   	if (acceleration != 0) 
   	{
       	/* Set ramp register, acceleration is steps/s/s */
       	SET_REG(this->chanbase, REG_RAMPRATE, (int)floor(acceleration));
   	}
   	/* Write direction bit */
   	if (forwards) 
   	{
       	CSR_SET(this->chanbase, CSR_DIRECTION);
   	}else 
   	{
       	CSR_CLR(this->chanbase, CSR_DIRECTION);
   	}
   	/* Write SH bit and start moving towards home */
   	CSR_SET(this->chanbase, CSR_HOMESTOP | CSR_JOG);
   	SET_REG(this->chanbase,REG_INTMASK,DONE_INT);             	/* to enable interrupt */

	pC_->lock();
   	setIntegerParam(pC_->motorStatusDone_, 0);
   	callParamCallbacks();
	pC_->unlock();

    return status;
}

// move velocity
asynStatus HytecMotorAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration)
{
    asynStatus status = asynSuccess;
//    static const char *functionName = "moveVelocityAxis";

   	if (min_velocity != 0) 
   	{
       	/* Set start speed register, min_velocity is steps/s */
       	SET_REG(this->chanbase, REG_STARTSTOPSPD, (int)floor(min_velocity));
   	}
   	if (max_velocity != 0) 
   	{
       	/* Set high speed register, velocity is steps/s */
       	SET_REG(this->chanbase, REG_HIGHSPD, (int)floor(max_velocity));
   	}
   	if (acceleration != 0) 
   	{
       	/* XXXX: set ramp register, acceleration is steps/s/s */
       	SET_REG(this->chanbase, REG_RAMPRATE, (int)floor(acceleration));
   	}
   	/* Write direction bit */
   	if (max_velocity >= 0) 
   	{
       	CSR_SET(this->chanbase, CSR_DIRECTION);
   	} else 
   	{
       	CSR_CLR(this->chanbase, CSR_DIRECTION);
   	}
   	/* Set Jog flag */
   	CSR_SET(this->chanbase, CSR_JOG);
   	SET_REG(this->chanbase,REG_INTMASK,DONE_INT);             /* to enable interrupt */

	pC_->lock();
   	setIntegerParam(pC_->motorStatusDone_, 0);
   	callParamCallbacks();
	pC_->unlock();

    return status;
}

// stop axis
asynStatus HytecMotorAxis::stop(double acceleration )
{
    asynStatus status = asynSuccess;
//    static const char *functionName = "stopAxis";
    int value;
	unsigned int acce;

   	acce = ((unsigned int)floor(acceleration));
   	if (acce != 0) 
    {
       	value = GET_REG(this->chanbase, REG_RAMPRATE);
       	if((acce > 64) || (acce <= 0xFFFF)) 
           	value = acce;

    	/* Set ramp register, acceleration is steps/s/s */
       	SET_REG(this->chanbase, REG_RAMPRATE, value);
       	/* Clear JOG and GO bits */
       	CSR_CLR(this->chanbase, CSR_JOG | CSR_GO);
    }else
	{
   	    /* Write abort bit to 1 */
   	   	CSR_SET(this->chanbase, CSR_CRASHSTOP);
       	CSR_CLR(this->chanbase, CSR_CRASHSTOP);
	}

    return status;
}

// timer task updates
asynStatus HytecMotorAxis::poll(bool *moving)
{
    double position, error=0.0, stepMoved;
    epicsUInt16 csr;
    asynStatus status = asynSuccess;

    pC_->lock();

    /* Get position register reading */
   	position = GET_REG(this->chanbase, REG_CURRPOSHI) << 16;
   	position += GET_REG(this->chanbase, REG_CURRPOSLO);

	/* For calculating absolute position if encoder is used */
   	csr = GET_REG(this->chanbase, REG_CSR);
   	error = GET_REG(this->chanbase, REG_STEPCNTHI) << 16;
   	error += GET_REG(this->chanbase, REG_STEPCNTLO);
	
	if(this->desiredMove < 0)										/* move reversely */
	{
		if(error >= 0)
		{
			stepMoved = (this->desiredMove + error) > 0 ? this->desiredMove + error : -(this->desiredMove + error);
			this->desiredMove = -error;
		}
		else
		{
			stepMoved = (this->desiredMove) > 0 ? this->desiredMove : -(this->desiredMove);
			this->desiredMove = 0;
		}
	}
	else															/* move forward */
	{
		if(error >= 0)
		{
			stepMoved = this->desiredMove - error;
			this->desiredMove = error;
		}
		else
		{
			stepMoved = this->desiredMove;
			this->desiredMove = 0;
		}
	}

    if((csr & CSR_ENCODDET) && this->useencoder) 														/* use encoder */
	{
       	setDoubleParam( pC_->motorEncoderPosition_, floor(position*this->encoderRatio+0.5) );
		if(csr & CSR_DIRECTION)													/* forward */
		{
			if(!(csr & CSR_DONE)) 
				this->absPosition += stepMoved;
			else
			{
				if(this->times == 0)
				{
					this->absPosition += stepMoved;
					this->times = 1;											/* set done bit hit the first time */
				}
   				SET_REG(this->chanbase, REG_STEPCNTHI, 0);
   				SET_REG(this->chanbase, REG_STEPCNTLO, 0);
			}
	        setDoubleParam( pC_->motorPosition_, floor(this->absPosition+0.5) );
		}else																	/* reversed */
		{
			if(!(csr & CSR_DONE)) 
				this->absPosition -= stepMoved;
			else
			{
				if(this->times == 0)
				{
					this->absPosition -= stepMoved;
					this->times = 1;											/* set done bit hit the first time */
				}
   				SET_REG(this->chanbase, REG_STEPCNTHI, 0);
   				SET_REG(this->chanbase, REG_STEPCNTLO, 0);
			}
	        setDoubleParam( pC_->motorPosition_, floor(this->absPosition-0.5) );			
		}
	}else																		/* do not use encoder */
	{
       	setDoubleParam( pC_->motorEncoderPosition_, floor(position+0.5) );
        setDoubleParam( pC_->motorPosition_, floor(position+0.5) );
	}

/* //debug
int acc= GET_REG(this->chanbase, REG_RAMPRATE);
int lsp= GET_REG(this->chanbase, REG_STARTSTOPSPD);
int hsp=GET_REG(this->chanbase, REG_HIGHSPD);
if(this->axisNo_ == 0)
printf("%d: moved=%d, error=%d, desired=%d absp=%d csr=%x\n",this->axisNo_, (int)stepMoved, (int)error , (int)this->desiredMove, (int)this->absPosition, (int)csr); 
*/
    setIntegerParam( pC_->motorStatusDirection_,     ((csr & CSR_DIRECTION) != 0 ) );
    setIntegerParam( pC_->motorStatusHasEncoder_,    ((csr & CSR_ENCODDET) != 0) );

	*moving = (csr & CSR_DONE) != 0 ? true : false;
    setIntegerParam( pC_->motorStatusDone_,          ((csr & CSR_DONE) != 0 ) );			
    setIntegerParam( pC_->motorStatusHighLimit_, 	((csr & CSR_MAXLMT) != 0 ) );
    setIntegerParam( pC_->motorStatusHome_,    		((csr & CSR_HOMELMT) != 0 ) );
    setIntegerParam( pC_->motorStatusProblem_,       ((csr & CSR_DRVSTAT) != 0) );
    setIntegerParam( pC_->motorStatusLowLimit_,  	((csr & CSR_MINLMT) != 0) );

    callParamCallbacks();
    pC_->unlock();
    return status;
}

asynStatus HytecMotorAxis::setPosition(double position)
{
  	asynStatus status = asynSuccess;

    if(this->useencoder) 																/* use encoder */
		this->absPosition = position;												  	/* update the software absolute position, note if encoder is not present, this is not used */
	else
	{
  		SET_REG(this->chanbase, REG_CURRPOSLO, ((int)floor(position)) & 0xFFFF);
  		SET_REG(this->chanbase, REG_CURRPOSHI, ((int)floor(position)) >> 16);
	}

  	return status;
}



/**
 * Configure the Hy8601 card
 * 
 * This simply creates a HytecMotorController object and 
 * the constructor does everything for initialisation.
 * 
 * @param portName   asyn port name
 * @param numAxes    number of axis on the IP card
 * @param movingPollPeriod  The time in ms between polls when any axis is moving
 * @param idlePollPeriod    The time in ms between polls when no axis is moving 
 * @param cardnum    Arbitrary card number to assign to this controller
 * @param ip_carrier which previously configured IP carrier in the IOC
 * @param ipslot     which IP Slot on carrier card (0=A etc.) 
 * @param vector     which Interrupt Vector (0 - Find One ?) 
 *                   8601 interrupt mask is always set to 0x2000. Even though the logical AND of bits in this 
 *                   register and those in the CSR will cause the channel and then the IP module 
 *                   to generate an interrupt on IRQ0 as the graph below
 *
 *                   bit 15  14  13   12  11  10  9   8   7   6   5    4     3      2      1     0
 *                       * | * | DN | * | * | * | * | * | * | * | * | FLT | HLIM | +LIM | -LIM | * | 
 *
 *                   we only set Done bit to generate interrupt since no matter which bit of FLT, HLIM, +LIM
 *                   or -LIM is set, the Done bit is set. As such, the interrupt source is identified by the 
 *                   ISR. Note, there is no interrupt clear concept. To stop the interrupt, we need to clear 
 *                   the mask bit. This means we need to enable the mask bit to enable the interrupt every time 
 *                   after a GO command which will also knock down the Done bit.
 *
 * @param useencoder - the least 4 bits determine to the 4 axes to use encoder when set (=1) 
 * @param encoderRatio0 ~ encoderRatio3  - hardware encoder ratio. For example, a quardrature encoder would 
 * 					 have its ratio of 0.25 since every line generates 4 counts.
 * 
 */
extern "C" int Hytec8601Configure(const char *portName, 
				int numAxes,
				int movingPollPeriod, 
				int idlePollPeriod,
				int cardnum,
	          	int ip_carrier,
	          	int ipslot,
	          	int vector,
	          	int useencoder,
	          	double encoderRatio0,
	          	double encoderRatio1,
	          	double encoderRatio2,
	          	double encoderRatio3)
{
    HytecMotorController *pHytecMotorController
        = new HytecMotorController(portName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.,
				cardnum, ip_carrier, ipslot, vector, useencoder, encoderRatio0, 
				encoderRatio1, encoderRatio2, encoderRatio3);
    pHytecMotorController = NULL;
    return(asynSuccess);
}

static const iocshArg Hy8601ConfigArg0 = {"portName", iocshArgString};
static const iocshArg Hy8601ConfigArg1 = {"numAxes", iocshArgInt};
static const iocshArg Hy8601ConfigArg2 = {"movingPollPeriod", iocshArgInt};
static const iocshArg Hy8601ConfigArg3 = {"idlePollPeriod", iocshArgInt};
static const iocshArg Hy8601ConfigArg4 = {"cardnum", iocshArgInt};
static const iocshArg Hy8601ConfigArg5 = {"carrier", iocshArgInt};
static const iocshArg Hy8601ConfigArg6 = {"ipslot", iocshArgInt};
static const iocshArg Hy8601ConfigArg7 = {"vector", iocshArgInt};
static const iocshArg Hy8601ConfigArg8 = {"useencoder", iocshArgInt};
static const iocshArg Hy8601ConfigArg9 = {"encoderRatio0", iocshArgDouble};
static const iocshArg Hy8601ConfigArg10 = {"encoderRatio1", iocshArgDouble};
static const iocshArg Hy8601ConfigArg11 = {"encoderRatio2", iocshArgDouble};
static const iocshArg Hy8601ConfigArg12 = {"encoderRatio3", iocshArgDouble};

static const iocshArg * const Hy8601ConfigArgs[] =  {&Hy8601ConfigArg0,
                                                       &Hy8601ConfigArg1,
                                                       &Hy8601ConfigArg2,
                                                       &Hy8601ConfigArg3,
                                                       &Hy8601ConfigArg4,
                                                       &Hy8601ConfigArg5,
													   &Hy8601ConfigArg6,
													   &Hy8601ConfigArg7,
													   &Hy8601ConfigArg8,
													   &Hy8601ConfigArg9,
													   &Hy8601ConfigArg10,
													   &Hy8601ConfigArg11,
													   &Hy8601ConfigArg12};

static const iocshFuncDef configHy8601 = {"Hytec8601Configure", 13, Hy8601ConfigArgs};
static void configHy8601CallFunc(const iocshArgBuf *args)
{
    Hytec8601Configure(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, 
			args[5].ival, args[6].ival, args[7].ival, args[8].ival, args[9].dval, 
			args[10].dval, args[11].dval, args[12].dval);
}

static void Hytec8601Register(void)
{
    iocshRegister(&configHy8601, configHy8601CallFunc);
}

epicsExportRegistrar(Hytec8601Register);


