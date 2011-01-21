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
/*  Source file name      :-      drvHy8601Asyn.c                               */
/*                                                                              */
/*  Initial creation date :-      23-Apr-2007                                   */
/*                                                                              */
/*  Original Developers   :-      Peter Denison                                 */
/*  Subsequent Developers :-      Jim Chen. Started from 13-Oct-2009            */
/*                                                                              */
/********************************************************************************/
/*                                                                              */
/*  Description :- Asyn motor Driver interface for                              */
/*                 Hytec 8601 Stepper Motor IP module                           */
/*                                                                              */
/*                                                                              */
/*  (C)2009 Hytec Electronics Ltd.                                              */
/*                                                                              */
/********************************************************************************/
/*                                                                              */
/*  Revision history: (comment and initial revisions)                           */
/*                                                                              */
/*  vers.       revised              modified by         date                   */
/*  -----       -----------        ----------------   ---------------           */
/*  0.1         Initial version      Peter Denison       23/04/2007             */
/*  1.0         Continue development Jim Chen            13/10/2009             */
/*  1.1         Added changes:       Jim Chen            12/11/2009             */
/*              1. Interrupt support to callbacks                               */
/*              2. Added resolution to allow user engineering unit              */
/*              3. Added encoder ratio                                          */
/*              4. Added soft low/high limit but these are in engineering unit. */
/*  1.2         Fixed a bug          Jim Chen            21/07/2010             */
/*              for reading the absolute position                               */
/*  1.3         Fixed a bug          Jim Chen            01/11/2010             */
/*              for reading absolute position and encoder position              */
/*  1.4         Modified             Jim Chen            11/11/2010             */
/*              to osi version and capped the stop acceleration input           */
/*                                                                              */
/*                                                                              */
/*                                                                              */
/********************************************************************************/

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "paramLib.h"

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
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"
/*#include <drvMotorAsyn.h>*/
#include "asynDriver.h"

#include "drvHy8601Asyn.h"

motorAxisDrvSET_t drvHy8601Asyn =
  {
    14,
    motorAxisReport,            /**< Standard EPICS driver report function (optional) */
    motorAxisInit,              /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLog,            /**< Defines an external logging function (optional) */
    motorAxisOpen,              /**< Driver open function */
    motorAxisClose,             /**< Driver close function */
    motorAxisSetCallback,       /**< Provides a callback function the driver can call when the status updates */
    motorAxisSetDouble,         /**< Pointer to function to set a double value */
    motorAxisSetInteger,        /**< Pointer to function to set an integer value */
    motorAxisGetDouble,         /**< Pointer to function to get a double value */
    motorAxisGetInteger,        /**< Pointer to function to get an integer value */
    motorAxisHome,              /**< Pointer to function to execute a more to reference or home */
    motorAxisMove,              /**< Pointer to function to execute a position move */
    motorAxisVelocityMove,      /**< Pointer to function to execute a velocity mode move */
    motorAxisStop               /**< Pointer to function to stop motion */
  };

epicsExportAddress(drvet, drvHy8601Asyn);

/* typedef struct motorAxis * AXIS_ID; */

typedef struct drvHy8601 * HY8601DRV_ID;
typedef struct drvHy8601
{
    HY8601DRV_ID pNext;
    asynUser * pasynUser;
    int card;
    int ip_carrier;
    int ipslot;
    int vector;
    int useencoder;
    double encoderRatio;                    /**< (double) Number of encoder counts in one motor count (encoder counts/motor counts) */
    double resolution;                      /**< (double) Number of motor units per engineering unit */
    double softLowLimit;                    /**< (double) Low soft limit in motor units ???? Shouldn't these two in engineering unit? */
    double softHighLimit;                   /**< (double) High soft limit in motor units ???? */
    volatile void *regbase;
    int nAxes;
    AXIS_HDL axis;
    epicsThreadId motorThread;
    motorAxisLogFunc print;
    void *logParam;
    epicsTimeStamp now;

    epicsMessageQueueId intMsgQId;
    int messagesSent;                       /* for report */
    int messagesFailed;                     /* for report */

} drvHy8601_t;

typedef struct motorAxisHandle
{
    HY8601DRV_ID pDrv;
    int axis;
    volatile void *chanbase;
    asynUser * pasynUser;
    PARAMS params;
    motorAxisLogFunc print;
    void * logParam;
    epicsMutexId axisMutex;
} motorAxis;

static HY8601DRV_ID pFirstDrv = NULL;

static int drvHy8601LogMsg( void * param, const motorAxisLogMask_t logMask, const char *pFormat, ...);
static void drvHy8601GetAxisStatus( AXIS_HDL pAxis, asynUser * pasynUser, int csr_data );
int SetupCard(HY8601DRV_ID pDrv);
int InitialiseAxis(HY8601DRV_ID pDrv, int axis);

static motorAxisLogFunc drvPrint = drvHy8601LogMsg;
static void *drvLogParam = NULL;

#define PRINT   drvPrint
#define FLOW    motorAxisTraceFlow

#ifndef ERROR
#define ERROR   motorAxisTraceError
#endif

#define MAX_MESSAGES  100           /* maximum number of messages */
#define ENCODER_HARD_RATIO    0.25  /* encoder hardware ratio */

/* static drvHy8601_t drv={ NULL, NULL, drvHy8601LogMsg, { 0, 0 } }; */

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

static int statusCount=1000;

static void motorAxisReportAxis( AXIS_HDL pAxis, int level )
{
    printf( "Found driver for Hy8601 card %d:%c, axis %d\n",
	    pAxis->pDrv->ip_carrier, pAxis->pDrv->ipslot + 'A', pAxis->axis );

    if (level > 0)
    {
        motorParam->dump( pAxis->params );
    }
}

static void motorAxisReport( int level )
{
    HY8601DRV_ID pDrv;

    for (pDrv = pFirstDrv; pDrv != NULL; pDrv = pDrv->pNext)
    {
        int i;

        for ( i = 0; i < pDrv->nAxes; i++ )
            motorAxisReportAxis( &(pDrv->axis[i]), level );
    }
}


static int motorAxisInit( void )
{
    return MOTOR_AXIS_OK;
}

static int motorAxisSetLog( AXIS_HDL pAxis, motorAxisLogFunc logFunc, void *param)
{
    if (pAxis == NULL) {
        if (logFunc == NULL) {
	    drvPrint = drvHy8601LogMsg;
	    drvLogParam = NULL;
	} else {
	    drvPrint = logFunc;
	    drvLogParam = param;
	}
    } else {
	if (logFunc == NULL) {
	    pAxis->print = drvHy8601LogMsg;
	    pAxis->logParam = NULL;
	} else {
	    pAxis->print = logFunc;
	    pAxis->logParam = param;
	}
    }
    return MOTOR_AXIS_OK;
}

static AXIS_HDL motorAxisOpen( int card, int axis, char * param )
{
    HY8601DRV_ID pDrv;
    AXIS_HDL pAxis = NULL;

    for ( pDrv=pFirstDrv; pDrv != NULL && (card != pDrv->card); pDrv = pDrv->pNext){}

    if (pDrv != NULL)
    /* JSC. axis starts from 0 to 3 hence need to use asix >= 0 instead of axis > 0 */
        if (axis >= 0 && axis < pDrv->nAxes) pAxis = &(pDrv->axis[axis]);

    return pAxis;
}

static int motorAxisClose( AXIS_HDL pAxis )
{
    return MOTOR_AXIS_OK;
}

static int motorAxisGetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int * value )
{
    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        return motorParam->getInteger( pAxis->params, (paramIndex) function, value );
    }
}

static int motorAxisGetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double * value )
{
    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        return motorParam->getDouble( pAxis->params, (paramIndex) function, value );
    }
}

static int motorAxisSetCallback( AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param )
{
    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        return motorParam->setCallback( pAxis->params, callback, param );
    }
}

static int motorAxisPrimitive( AXIS_HDL pAxis, int length, char * string )
{
    return MOTOR_AXIS_OK;
}

static int motorAxisSetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double value )
{
    int status = MOTOR_AXIS_OK;
    double previous_resolution;

    if (pAxis == NULL) {
	return MOTOR_AXIS_ERROR;
    } else {
        switch (function) {
        case motorAxisPosition:
	    /* Write to position register */
	        SET_REG(pAxis->chanbase, REG_CURRPOSLO, ((int)floor(value * pAxis->pDrv->resolution)) & 0xFFFF);
	        SET_REG(pAxis->chanbase, REG_CURRPOSHI, ((int)floor(value * pAxis->pDrv->resolution)) >> 16);
            pAxis->print(pAxis->logParam, FLOW, "Set card %d, axis %d to position %f\n", pAxis->pDrv->card, pAxis->axis, value );
            break;

        case motorAxisResolution:
	    /* Calculate the resolution. it is initialised as 1.0 */
            previous_resolution = pAxis->pDrv->resolution;
            pAxis->pDrv->resolution = value;
            pAxis->pDrv->softLowLimit = pAxis->pDrv->resolution * pAxis->pDrv->softLowLimit / previous_resolution;      /* adjust soft low limit */
            pAxis->pDrv->softHighLimit = pAxis->pDrv->resolution * pAxis->pDrv->softHighLimit / previous_resolution;    /* adjust soft high limit */
            break;

        case motorAxisEncoderRatio:
	    /* Calculate the encoder ratio and store. Note, encoder ratio is initialised as 1/4 since the hardware design counts 4 of each encoder pulse */
            pAxis->pDrv->encoderRatio = ENCODER_HARD_RATIO * value;
            break;

        case motorAxisLowLimit:
	    /* Calculate the soft low limit to engineering unit */
            pAxis->pDrv->softLowLimit = pAxis->pDrv->resolution * value;
            break;

        case motorAxisHighLimit:
	    /* Calculate the soft high limit to engineering unit */
            pAxis->pDrv->softHighLimit = pAxis->pDrv->resolution * value;
            break;

        case motorAxisPGain:
            pAxis->print( pAxis->logParam, FLOW, "Cannot set Hy8601 card %d, axis %d pgain (%f)\n", pAxis->pDrv->card, pAxis->axis, value );
	    status = MOTOR_AXIS_ERROR;
            break;

        case motorAxisIGain:
            pAxis->print( pAxis->logParam, FLOW, "Cannot set Hy8601 card %d, axis %d igain (%f)\n", pAxis->pDrv->card, pAxis->axis, value );
	    status = MOTOR_AXIS_ERROR;
            break;

        case motorAxisDGain:
            pAxis->print( pAxis->logParam, FLOW, "Cannot set Hy8601 card %d, axis %d dgain (%f)\n", pAxis->pDrv->card, pAxis->axis, value );
	    status = MOTOR_AXIS_ERROR;
            break;

        case motorAxisClosedLoop:                       
            pAxis->print( pAxis->logParam, FLOW, "Cannot set Hy8601 card %d, axis %d closed loop (%s)\n", pAxis->pDrv->card, pAxis->axis, (value!=0?"ON":"OFF") );
	    status = MOTOR_AXIS_ERROR; 
            break;

        default:
            status = MOTOR_AXIS_ERROR;
            break;
	}

        if (status == MOTOR_AXIS_OK ) {
	    status = motorParam->setDouble( pAxis->params, function, value );
	}
    }
    return status;
}

static int motorAxisSetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int value )
{
    int status = MOTOR_AXIS_OK;

    if (pAxis == NULL) {
	return MOTOR_AXIS_ERROR;
    } else {
        status = motorAxisSetDouble( pAxis, function, (double) value );
    }
    return status;
}


static int motorAxisMove( AXIS_HDL pAxis, double position, int relative, double min_velocity, double max_velocity, double acceleration )
{
    int status = MOTOR_AXIS_ERROR;
    double result, current_position;
    int currpos;

    if (pAxis != NULL)
    {
	   if (min_velocity != 0) 
       {
	       /* Set start speed register, min_velocity is steps/s */
	       SET_REG(pAxis->chanbase, REG_STARTSTOPSPD, (int)floor(min_velocity));
	   }
	   if (max_velocity != 0) 
       {
	       /* Set high speed register, max_velocity is steps/s */
	       SET_REG(pAxis->chanbase, REG_HIGHSPD, (int)floor(max_velocity));
	   }
       if (acceleration >= 64) 
       {
	       /* Set ramp register, acceleration is steps/s/s */
	       SET_REG(pAxis->chanbase, REG_RAMPRATE, (int)floor(acceleration));
	   }
       if (relative) 
       {
	       /* Write to step counter register abs(position)*/
	       SET_REG(pAxis->chanbase, REG_STEPCNTLO, ((int)floor(abs(position * pAxis->pDrv->resolution))) & 0xFFFF);
	       SET_REG(pAxis->chanbase, REG_STEPCNTHI, ((int)floor(abs(position * pAxis->pDrv->resolution))) >> 16); 
	       /* Write dir flag based on relative position */
	       if (position >= 0) 
           {
		      CSR_SET(pAxis->chanbase, CSR_DIRECTION);
	       } else 
           {
		      CSR_CLR(pAxis->chanbase, CSR_DIRECTION);
	       }
       } else 
       {
	       /* Subtract absolute position register from position */
	       currpos = GET_REG(pAxis->chanbase, REG_CURRPOSHI) << 16;
	       currpos += GET_REG(pAxis->chanbase, REG_CURRPOSLO);
           if(pAxis->pDrv->useencoder) 
                current_position = (double)currpos * pAxis->pDrv->encoderRatio;        /* if use encoder, current position is adjusted by encoder ratio */
           else
                current_position = (double)currpos;                                    /* otherwise, current position is the motor units (steps) */
	       result = position * pAxis->pDrv->resolution - current_position;
/* printf("r=%f p=%f l=%f c=%f axis=%d\n", result, position, pAxis->pDrv->resolution, current_position, pAxis->axis); */ /* for debug  */
	       /* Write to step counter abs(result) */
	       SET_REG(pAxis->chanbase, REG_STEPCNTLO, ((int)floor(abs(result))) & 0xFFFF);
	       SET_REG(pAxis->chanbase, REG_STEPCNTHI, ((int)floor(abs(result))) >> 16); 
	       /* Write dir flag based on result */
	       if (result >= 0) 
           {
		      CSR_SET(pAxis->chanbase, CSR_DIRECTION);
	       } else 
           {
		      CSR_CLR(pAxis->chanbase, CSR_DIRECTION);
	       }
	   }

	   /* Write GO bit */
	   CSR_SET(pAxis->chanbase, CSR_GO);
       SET_REG(pAxis->chanbase,REG_INTMASK,DONE_INT);             /* to enable interrupt */

	   status = MOTOR_AXIS_OK;
    }
    return status;
}

static int motorAxisHome( AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards )
{
    int status = MOTOR_AXIS_ERROR;

    if (pAxis != NULL)
    {
	   if (min_velocity != 0)  
       {
	       /* Set start speed register, min_velocity is steps/s */
	       SET_REG(pAxis->chanbase, REG_STARTSTOPSPD, (int)floor(min_velocity));
	   }
	   if (max_velocity != 0) 
       {
	       /* Set high speed register, max_velocity is steps/s */
	       SET_REG(pAxis->chanbase, REG_HIGHSPD, (int)floor(max_velocity));
	   }
       if (acceleration != 0) 
       {
	       /* Set ramp register, acceleration is steps/s/s */
	       SET_REG(pAxis->chanbase, REG_RAMPRATE, (int)floor(acceleration));
	   }
	   /* Write direction bit */
	   if (max_velocity >= 0) 
       {
	       CSR_SET(pAxis->chanbase, CSR_DIRECTION);
	   } else 
       {
	       CSR_CLR(pAxis->chanbase, CSR_DIRECTION);
	   }
	   /* Write SH bit and start moving towards home */
	   CSR_SET(pAxis->chanbase, CSR_HOMESTOP | CSR_JOG);
       SET_REG(pAxis->chanbase,REG_INTMASK,DONE_INT);             /* to enable interrupt */

       status = MOTOR_AXIS_OK;
    }
    return status;
}


static int motorAxisVelocityMove(  AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration )
{
    int status = MOTOR_AXIS_ERROR;

    if (pAxis != NULL)
    {
	   if (min_velocity != 0) 
       {
	       /* Set start speed register, min_velocity is steps/s */
	       SET_REG(pAxis->chanbase, REG_STARTSTOPSPD, (int)floor(min_velocity));
	   }
	   if (velocity != 0) 
       {
	       /* Set high speed register, velocity is steps/s */
	       SET_REG(pAxis->chanbase, REG_HIGHSPD, (int)floor(velocity));
	   }
       if (acceleration != 0) 
       {
	       /* XXXX: set ramp register, acceleration is steps/s/s */
	       SET_REG(pAxis->chanbase, REG_RAMPRATE, (int)floor(acceleration));
	   }
	   /* Write direction bit */
	   if (velocity >= 0) 
       {
	       CSR_SET(pAxis->chanbase, CSR_DIRECTION);
	   } else 
       {
	       CSR_CLR(pAxis->chanbase, CSR_DIRECTION);
	   }
	   /* Set Jog flag */
	   CSR_SET(pAxis->chanbase, CSR_JOG);
       SET_REG(pAxis->chanbase,REG_INTMASK,DONE_INT);             /* to enable interrupt */


       status = MOTOR_AXIS_OK;
    }
    return status;
}

static int motorAxisProfileMove( AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger )
{
    return MOTOR_AXIS_ERROR;
}

static int motorAxisTriggerProfile( AXIS_HDL pAxis )
{
    return MOTOR_AXIS_ERROR;
}

static int motorAxisStop( AXIS_HDL pAxis, double acceleration )
{
    int status = MOTOR_AXIS_ERROR;
    int value;

    if (pAxis != NULL)
    {
        if (acceleration != 0) 
        {
           value = GET_REG(pAxis->chanbase, REG_RAMPRATE);
           if((int)floor(acceleration) < 64) 
               value = 64;
           else if((int)floor(acceleration) < value)
               value = (int)floor(acceleration);

	       /* Set ramp register, acceleration is steps/s/s */
	       SET_REG(pAxis->chanbase, REG_RAMPRATE, value);
	       /* Clear JOG and GO bits */
	       CSR_CLR(pAxis->chanbase, CSR_JOG | CSR_GO);
	    }else
    	    /* Write abort bit to 1 */
    	    CSR_SET(pAxis->chanbase, CSR_CRASHSTOP);

        status = MOTOR_AXIS_OK;
    }
    return status;
}

/** \defgroup ISR8601 Routines to handle interrupts
@{
*/
static void ISR_8601(int priv)
{
    HY8601DRV_ID pDrv = (HY8601DRV_ID) priv;
    int data[4];
    int chan;

    for (chan = 0; chan < HY8601_NUM_AXES; chan++) 
    {
        data[chan] = GET_REG(pDrv->axis[chan].chanbase, REG_CSR);
        SET_REG(pDrv->axis[chan].chanbase, REG_INTMASK, 0);             /* disable interrupt */
    }

    /* wake up INT queue */
    if (epicsMessageQueueTrySend(pDrv->intMsgQId, data, sizeof(data)) == 0)
        pDrv->messagesSent++;
    else
        pDrv->messagesFailed++;
}

/**
}
*/
static void intQueuedTask( HY8601DRV_ID pDrv )
{
    int data[4];
    int chan;

    while(1) 
    {
        /* Wait for event from interrupt routine */
        epicsMessageQueueReceive(pDrv->intMsgQId, data, sizeof(data));
                 
        for (chan = 0; chan < HY8601_NUM_AXES; chan++) 
        {
           drvHy8601GetAxisStatus( &(pDrv->axis[chan]), pDrv->pasynUser, data[chan] );
        }

 /*printf("\nasynDrover! Pointer=%d\n\n", data[0]); */    /* for debug */
        /* re-enable interrupt. for Linux IOCs */
        ipmIrqCmd(pDrv->ip_carrier, pDrv->ipslot, 0, ipac_irqEnable);
    }
}


/** \defgroup drvHy8601Task Routines to implement the motor axis task
@{

*/


static void drvHy8601GetAxisChanges( AXIS_HDL pAxis, asynUser * pasynUser )
{
    double position, error=0.0;
    epicsUInt16 csr;

    /* Set position, error = step count, velocity */
    position = GET_REG(pAxis->chanbase, REG_CURRPOSHI) << 16;
    position += GET_REG(pAxis->chanbase, REG_CURRPOSLO);
    if(pAxis->pDrv->useencoder) 
        position = position * pAxis->pDrv->encoderRatio;         /* if encoder is used, adjust it according to the ratio  */
    else
        position = position / pAxis->pDrv->resolution;           /* otherwise convert the position back to engineering unit */

    csr = GET_REG(pAxis->chanbase, REG_CSR);

    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
    {
        motorParam->setInteger(  pAxis->params, motorAxisPosition,      (int) floor(position-error+0.5) );
        motorParam->setInteger(  pAxis->params, motorAxisEncoderPosn,   (int) floor(position+0.5) );
/* printf("p=%f csr=%x axis=%d\n", position, csr, pAxis->axis); */ /* for debug  */
        motorParam->setInteger( pAxis->params, motorAxisDirection,     ((csr & CSR_DIRECTION) != 0 ) );
        motorParam->setInteger( pAxis->params, motorAxisHasEncoder,    ((csr & CSR_ENCODDET) == 0) );

        motorParam->setInteger( pAxis->params, motorAxisDone,          ((csr & CSR_DONE) != 0 ) );
        motorParam->setInteger( pAxis->params, motorAxisHighHardLimit, ((csr & CSR_MAXLMT) != 0 ) );
        motorParam->setInteger( pAxis->params, motorAxisHomeSignal,    ((csr & CSR_HOMELMT) != 0 ) );
        motorParam->setInteger( pAxis->params, motorAxisProblem,       ((csr & CSR_DRVSTAT) != 0) );
        motorParam->setInteger( pAxis->params, motorAxisLowHardLimit,  ((csr & CSR_MINLMT) != 0) );

        motorParam->callCallback( pAxis->params );
        epicsMutexUnlock( pAxis->axisMutex );
    }
}

static void drvHy8601GetAxisStatus( AXIS_HDL pAxis, asynUser * pasynUser, int csr_data )
{

    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
    {
        motorParam->setInteger( pAxis->params, motorAxisDone,          ((csr_data & CSR_DONE) != 0 ) );
        motorParam->setInteger( pAxis->params, motorAxisHighHardLimit, ((csr_data & CSR_MAXLMT) != 0 ) );
        motorParam->setInteger( pAxis->params, motorAxisHomeSignal,    ((csr_data & CSR_HOMELMT) != 0 ) );
        if (csr_data & CSR_HOMELMT) CSR_CLR(pAxis->chanbase, CSR_HOMESTOP);                     /* after home limit is reported, clear Stop at home */
        motorParam->setInteger( pAxis->params, motorAxisProblem,       ((csr_data & CSR_DRVSTAT) != 0) );
        motorParam->setInteger( pAxis->params, motorAxisLowHardLimit,  ((csr_data & CSR_MINLMT) != 0) );
        motorParam->callCallback( pAxis->params );
        epicsMutexUnlock( pAxis->axisMutex );
    }
}


#define DELTA 1.0
static void drvHy8601Task( HY8601DRV_ID pDrv )
{
    while ( 1 )
    {
        int i;

        if (statusCount)
        {
            statusCount--;
            for ( i = 0; i < pDrv->nAxes; i++ )
            {
                drvHy8601GetAxisChanges( &(pDrv->axis[i]), pDrv->pasynUser );
            }
        }
        epicsThreadSleep( DELTA );
    }
}

/**
 * Configure the Hy8601 card
 * 
 * Check if Interrupt Vector is Valid (i.e. 0 to 254).
 * Search through Structures of Card Info to ensure...
 * 		VME Slot and IP Slot Combination Spare.
 * Create New Card Info Structure and populate it, 
 *              including all IP slots data.
 * 
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
 * @param useencoder - determine to use encoder when set (=1) 
 * @param limitmode  not used.
 * 
 * @retval MOTOR_AXIS_OK    everything alright.
 * @retval MOTOR_AXIS_ERROR an error occured.
 * 
 * This function draws tiny fragments from the original CONFIGURE() routine
 * and the original DRIVER_INIT() in the Hytec driver
 */
int Hy8601Configure(int cardnum,
	          int ip_carrier,
	          int ipslot,
	          int vector,
	          int useencoder,
	          int limitmode)
{
    int nAxes = HY8601_NUM_AXES;
    int i;
    int status = MOTOR_AXIS_OK;
    HY8601DRV_ID pDrv;
    HY8601DRV_ID * ppLast = &(pFirstDrv);

    /* Check the validity of the arguments */
    /* Is Interrupt Vector Valid...*/
    if (vector <= 0 || vector > 255) 
    {
	   printf("ERROR! illegal interrupt vector number %d !\n", vector);
       printf("must be in range [1..255]\n");
	   return MOTOR_AXIS_ERROR;
    }

    /* Arguments OK, so now check for duplicate cards */
    /* Search through the link list of card information structures...*/
    for ( pDrv = pFirstDrv;
          pDrv != NULL &&
	      !((pDrv->ip_carrier == ip_carrier) && (pDrv->ipslot == ipslot)) &&
	      cardnum != pDrv->card;
          pDrv = pDrv->pNext )
    {
        ppLast = &(pDrv->pNext);
    }

    if (pDrv != NULL) 
    {
	    /* If the card number has been used */
	    if (pDrv->card == cardnum) 
        {
	       PRINT( drvLogParam, ERROR, "Card number %d has already been assigned to a Hy8601 card\n", cardnum);
	       return MOTOR_AXIS_ERROR;
	    } else 
        {
	       /* If the IP Carrier / IP Slot combination is used... */
	       /* Display Error Message and Exit with Error */
	       PRINT( drvLogParam, ERROR, "Motor for card %d already exists\n", cardnum );
	       printf("ERROR! card in ip_carrier %d and ipslot %d already configured\n",
		      ip_carrier,ipslot);
	       return MOTOR_AXIS_ERROR;
	    }
    }

    PRINT( drvLogParam, FLOW,
	   "Creating Hy8601 motor driver as card: %d, with %d axes\n",
	   cardnum, nAxes );
    
    pDrv = (HY8601DRV_ID) calloc( 1, sizeof(drvHy8601_t) );

    if (pDrv == NULL) 
    {
	   asynPrint(pDrv->pasynUser, ASYN_TRACE_ERROR,
		  "drvHy8601Configure: unable to create driver for card %d: "
		  "insufficient memory\n",
		  cardnum );
	   return MOTOR_AXIS_ERROR;
    }

    pDrv->card = cardnum;
    pDrv->nAxes = nAxes;
    pDrv->ip_carrier = ip_carrier;
    pDrv->ipslot = ipslot;
    pDrv->vector = vector;
    pDrv->useencoder = useencoder;
    pDrv->encoderRatio = ENCODER_HARD_RATIO;            /* encoder ratio is hardwared as 4 counts of each encoder pulse */  
    pDrv->resolution = 1.0;                             /* initial resolution is 1 */
    pDrv->softLowLimit = 0.0;                           /* initialise soft low limit to 0 */
    pDrv->softHighLimit = 0.0;                          /* initialise soft high limit to 0 */

    if ((status = SetupCard(pDrv)) != MOTOR_AXIS_OK) 
    {
	   return status;
    }

    pDrv->axis = (AXIS_HDL) calloc( nAxes, sizeof( motorAxis ) );
	
    if (pDrv->axis == NULL ) 
    {
	   /* Allocation of per-axis array failed */
	   free ( pDrv );
	   return MOTOR_AXIS_ERROR;
    }

    for (i=0; i<nAxes && status == MOTOR_AXIS_OK; i++ ) 
    {
	   if ((pDrv->axis[i].params = motorParam->create( 0, MOTOR_AXIS_NUM_PARAMS )) != NULL &&
	       (pDrv->axis[i].axisMutex = epicsMutexCreate( )) != NULL )
	   {
	       /* Enable interrupts ?? */
	       InitialiseAxis(pDrv, i);
		
	       asynPrint( pDrv->pasynUser, ASYN_TRACE_FLOW, "Created motor for VME %d slot %c, axis %d OK\n", ip_carrier, ipslot+'A', i );
	   } else 
       {
	       asynPrint(pDrv->pasynUser, ASYN_TRACE_ERROR,
		      "drvHy8601Create: unable to set create axis %d: insufficient memory\n", 
		      i );
	       status = MOTOR_AXIS_ERROR;
	   }
    }

    /* If any of them failed, wind out all allocated memory */
    if ( status == MOTOR_AXIS_ERROR ) 
    {
	   for (i=0; i<nAxes; i++ ) 
       {
	       if (pDrv->axis[i].params != NULL) 
           {
		      motorParam->destroy( pDrv->axis[i].params );
	       }
	       if (pDrv->axis[i].axisMutex != NULL) 
           {
		      epicsMutexDestroy( pDrv->axis[i].axisMutex );
	       }
	   }
	   free ( pDrv );
	   return MOTOR_AXIS_ERROR;
    }

    /* Add the driver onto the end of the list */
    *ppLast = pDrv;
    
    pDrv->motorThread = epicsThreadCreate( "drvHy8601Thread", 
					   epicsThreadPriorityLow,
					   epicsThreadGetStackSize(epicsThreadStackMedium),
					   (EPICSTHREADFUNC) drvHy8601Task,
					   (void *) pDrv );
    if (pDrv->motorThread == NULL) 
    {
	   PRINT( drvLogParam, ERROR, "Cannot start motor polling thread\n" );
	   return MOTOR_AXIS_ERROR;
    }


    pDrv->intMsgQId = epicsMessageQueueCreate(MAX_MESSAGES, 
                                              (HY8601_NUM_AXES + 2)*sizeof(int));
    if (epicsThreadCreate("drvHy8601intQueuedTask",
                           epicsThreadPriorityLow,
                           epicsThreadGetStackSize(epicsThreadStackMedium),
                           (EPICSTHREADFUNC)intQueuedTask,
                           pDrv) == NULL)
    {
	   PRINT( drvLogParam, ERROR, "drvHy8601intQueuedTask epicsThreadCreate failure\n" );
	   return MOTOR_AXIS_ERROR;
    }

    return status;

    /* Clear Initialise All IP Slots Motor Record */
    /* XXX Initialise params and hardware fullspeed, ramp, etc. */
    /*	for (chan=0; chan<NUMCHAN; chan++)
	{
		motptr->chanbase=motptr->regbase=0xffffffff;
		motptr->channum=0xffff;
		motptr->fullspeedcode=motptr->rampcode=0xff;
		motptr->cbprocptr=NULL;
		motptr++;
		}*/
}

static int drvHy8601LogMsg( void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{

    va_list	pvar;
    int		nchar=0;
    asynUser *  pasynUser =  (asynUser *) param;
    int         reason = (int) mask;

    if ( pasynUser == NULL )
    {
        va_start(pvar, pFormat);
        vprintf( pFormat, pvar );
        va_end (pvar);
    }
    else if ( pasynTrace->getTraceMask(pasynUser) & reason )
    {
        va_start(pvar, pFormat);
        nchar = pasynTrace->vprint( pasynUser, reason, pFormat, pvar );
        va_end (pvar);
    }

    return(nchar);
}

/* Following code mostly from the original Hytec driver */

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
02-NOV-2004     DAN     1.1.1.3     Intial Version.
    
**********************************************************/
static int checkprom(void *pr,int expmodel,int verbose)
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
	int		ishytec,ismodel,strok;
  
	/* Begin */

	/* Debug - Display the passed Address */ 
	if (verbose) printf("PROM CONTENTS AT: %p\n",pr);
	/* Read the first 6 Characters from the passed Address */ 
	strncpy(lstr, (char*)pr, 6);
	/* Convert it to a String by Null terminating it */
	lstr[6]=0;

	/* Compare to Expected String (i.e. "VITA4 ") */
	strok = (strcmp(lstr, IP_DETECT_STR) == 0);

	if (verbose) printf("PROM header: '%6s'\n",lstr);
	/* Set manid to PROM Address of the Hytec ID */
	manid    = *((int*)(pr+PR_ID));
	/* Set ishytec to TRUE if Hytec ID is as expected */
	ishytec  = (manid ==HYTECID);
	/* Set modelnum to PROM Address of the IP Model */
	modelnum = *((epicsUInt16*)(pr+PR_MODN));
	/* Set ismodel to TRUE if IP Model is as expected */
	ismodel  = (modelnum==expmodel);
	
	if (verbose)
	{
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
    
		printf("PROM flags: 0x%04hx\nPROM number of bytes used: 0x%04hx (%hd)\n\n",
		*((short int*)(pr+PR_FLAGS)),
		*((short int*)(pr+PR_BYTES)),
		*((short int*)(pr+PR_BYTES)));
}

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
	
	return (/* SCO strok && */ishytec && ismodel);
}

int SetupCard(HY8601DRV_ID pDrv)
{
    void *regbase;
    int status;
    epicsUInt16 rprobe;

    /* Register the card in A16 address space */
    regbase=ipmBaseAddr( pDrv->ip_carrier,
			 pDrv->ipslot,
			 ipac_addrIO);

    /* If failed to register card... */
    if (regbase == NULL) 
    {
	   /* Log Error Message */
	   PRINT(drvLogParam, ERROR,
	      DEVICE_NAME ": Cannot register Hy8601 A16 device.\n");
	   return MOTOR_AXIS_ERROR;
    }

    /* OK so far, now Verify that something exists at the specified address */
    /* Check memory by seeing if you can read it, if you can NOT... */
    if ((status=devReadProbe(sizeof(epicsUInt16),
			     regbase,
			     (char *)&rprobe) != S_dev_success)) 
    {
	   /* Display a bit or Error Debug Information */
	   printf(DEVICE_NAME ": NO DEVICE: A16 Memory Probe error (status=%d)\n",
	       status);
	   printf("(ip_carrier %d, ipslot %d) at address %p\n",
	       pDrv->ip_carrier,
	       pDrv->ipslot,
	       regbase);

	   return MOTOR_AXIS_ERROR;
    }

    /* Memory Check OK, so check PROM */
    /* JSC. checkprom returns 1 = ok, 0 = failed. */
    if (checkprom(regbase+PROM_OFFS, PROM_MODEL, 1)!=1) 
    {
	   return MOTOR_AXIS_ERROR;
    }

    /* Setup Interrupts. THIS SEEMS NOT RIGHT. 8601 HAS 4 AXIS AND EACH ONE HAS A VECTOR SO 4 ISRs ARE NEEDED?????????????? */
    status=ipmIntConnect(pDrv->ip_carrier,
			 pDrv->ipslot,
			 pDrv->vector,
			 (void *)ISR_8601,
			 (int)pDrv);
      
    /* If Interrupts NOT Setup OK */
    if (status!=OK) 
    {
	   /* Display Error Message */
	   printf(DEVICE_NAME ": intConnect ERROR, slot %d", pDrv->ip_carrier);
	   return MOTOR_AXIS_ERROR;
    }

    /* Setup Carrier Card */
    ipmIrqCmd(pDrv->ip_carrier,
	      pDrv->ipslot,
	      0,
	      ipac_irqEnable);

    /* If everything still OK, Initialise the Registers... */
    /* Initialise the motdat structures and set configured if everything went OK */
    /* XXX Clashes with REG_STEPCNTHI ???
    *(volatile epicsUInt16*)(regbase+GLOBALGO)=0;
    */

    pDrv->regbase = regbase;
    return MOTOR_AXIS_OK;
}

int InitialiseAxis(HY8601DRV_ID pDrv, int axis)
{
    AXIS_HDL pAxis = &pDrv->axis[axis];
    volatile void *chanbase;

    /* Required for the motor axis infrastructure */
    pAxis->axis = axis;
    pAxis->logParam = pDrv->pasynUser;
    pAxis->pasynUser = pDrv->pasynUser;
    pAxis->pDrv = pDrv;

    /* Initialisation specific to this model of controller */
    chanbase = pAxis->chanbase = pDrv->regbase + REG_BANK_OFFS + axis * REG_BANK_SZ;

    /* XXX add debug level to print statements ??
    if (debug) {
	printf("Motor Output - %d Base Address %p\n", axis, chanbase);
    }
    */
                        
    /* Reset Card */
    CSR_SET(chanbase, CSR_RESET);
    CSR_CLR(chanbase, CSR_RESET);

    /* check use encoder set by the configure */
    if(pDrv->useencoder)
        CSR_SET(chanbase, CSR_ENCODUSE);

    /* enable interrupt, set vector */
    /* Please note, all 4 axis of 8601 have been set the same vector. 
     * It is the ISR's responsibility to check which axis generates the interrupt at runtime.
     * If the mask of individual axis needs to be set differently, extra interface should 
     * be created, but they should still share the same ISR routine. */
    CSR_SET(chanbase, CSR_INTEN);
    SET_REG(chanbase,REG_INTVECTOR,pDrv->vector);

    return MOTOR_AXIS_OK;
}	

