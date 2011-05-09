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
/*				c).Changes asynMotorDriver.cpp to asynMotorController.cpp.		*/
/*  2.2         Bugs fix 				Jim Chen       	 14/04/2011             */
/*  	        Fixed setPosition bug											*/
/*				Added firmware version parameter								*/
/*                                                                              */
/********************************************************************************/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

static const char *driverName = "HytecMotorDriver";

/* CSR Register bit definitions */
#define CSR_HOMESTOP	0x8000
#define CSR_INTEN		0x4000
#define CSR_DONE		0x2000
#define CSR_CRASHSTOP	0x1000
#define CSR_DIRECTION	0x0800
#define CSR_AUX2		0x0400
#define CSR_AUX1		0x0200
#define CSR_ENCODUSE	0x0100
#define CSR_ENCODDET	0x0080
#define CSR_JOG			0x0040
#define CSR_GO			0x0020
#define CSR_DRVSTAT		0x0010
#define CSR_HOMELMT		0x0008
#define CSR_MAXLMT		0x0004
#define CSR_MINLMT		0x0002
#define CSR_RESET		0x0001

/* New Hardware Register Map */
#define REG_STEPCNTLO	0x00
#define REG_STEPCNTHI	0x02
#define REG_CURRPOSLO	0x04
#define REG_CURRPOSHI	0x06
#define REG_STARTSTOPSPD    0x08
#define REG_HIGHSPD	0x0A
#define REG_RAMPRATE	0x0C
#define REG_CSR		0x0E
#define REG_INTMASK	0x10
#define REG_INTVECTOR	0x12
#define REG_INTREQUEST	0x14
#define REG_CURRENTSPD	0x16
#define REG_SPARE1	0x18
#define REG_SPARE2	0x1A
#define REG_SPARE3	0x1C
#define REG_SPARE4	0x1E

#define PROM_MODEL	0x8601
#define PROM_OFFS	0x80

#define REG_BANK_OFFS	0x00
#define REG_BANK_SZ	0x20

#define INT_LMT         (CSR_MINLMT | CSR_MAXLMT | CSR_HOMELMT)
#define INT_SRCS        (CSR_RESET | INT_LMT | CSR_DRVSTAT | CSR_DONE)

/* define a mask to enable all sources of interrupts */
#define ALL_INTS     (INT_SRCS | CSR_INTEN)
#define DONE_INT     (CSR_DONE)                         /* the only interrupt suggested to use. JC 12-Nov-2009 */

#define HY8601_NUM_AXES 4

#define IP_DETECT_STR "VITA4 "

#define GET_REG(base,reg) (*(volatile epicsUInt16 *)((base)+(reg)))
#define SET_REG(base,reg,val) do { *(volatile epicsUInt16 *)((base)+(reg)) = (val);} while(0)
#define CSR_SET(base,bit) do { *(volatile epicsUInt16 *)((base)+REG_CSR) |= (bit);} while(0)
/* Please NOTE, The DONE bit is cleared by either ORing an "1" or ANDing an "1" (if it is already "1") to it. 
 * Yet neither ORing nor ANDing it with "0" would affect it. As such, clearing any other bit in the CSR with
 * the AND operation need to be very careful so that it doesn't knock down the DONE bit. This is why in the 
 * following macro after inverting the bit in the CSR, we clear the DONE bit to the inversion result in order 
 * not to affect the DONE bit. */
#define CSR_CLR(base,bit) do { *(volatile epicsUInt16 *)((base)+REG_CSR) &= ((~(bit)) & 0xDFFF);} while(0) 


#define HytecPowerControlString		"HYTEC_POWER"
#define HytecBrakeControlString		"HYTEC_BRAKE"
#define HytecFirmwareString			"HYTEC_FWVERSION"
#define HytecMoveAllString			"HYTEC_MOVEALL"


class HytecMotorAxis : public asynMotorAxis
{
public:
  	HytecMotorAxis(class HytecMotorController *pC, int axis, double ratio, int vector);
  	asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  	asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  	asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  	asynStatus stop(double acceleration);
  	asynStatus poll(bool *moving);
  	asynStatus setPosition(double position);
	int getVector();
	volatile char * getChanbase();
	//the following are called from non-member function so they are here
	
private:
	asynStatus InitialiseAxis();
	HytecMotorController *pC_;

    volatile char *chanbase;
    int vector;

    int useencoder;
    double encoderRatio;                    /**< (double) Number of encoder counts in one motor count (encoder counts/motor counts) */
    double resolution;                      /**< (double) Number of motor units per engineering unit */
    double softLowLimit;                    /**< (double) Low soft limit in motor units ???? Shouldn't these two in engineering unit? */
    double softHighLimit;                   /**< (double) High soft limit in motor units ???? */
	double absPosition;
	double desiredMove;
	int times;								/* to remember the the done bit set the first time */
	int absAskingPosition;


friend class HytecMotorController;
};


class HytecMotorController : asynMotorController 
{
public:
	HytecMotorController(const char *portName, int numAxes, double movingPollPeriod, 
				double idlePollPeriod, int cardnum, int ip_carrier, int ipslot, 
				int vector, int useencoder, double encoderRatio0, double encoderRatio1, 
				double encoderRatio2, double encoderRatio3);

    // These are the methods that we override from asynMotorDriver
    asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    void report(FILE *fp, int level);

  	HytecMotorAxis* getAxis(asynUser *pasynUser);
    HytecMotorAxis* getAxis(int axisNo);

	//the following are called from non-member function so they are here
	void drvHy8601GetAxisStatus( HytecMotorAxis *pAxis, int csr_data );
	int getNumAxes();
	int getIPCarrier();
	int getIPSlot();
	epicsMessageQueueId getINTMsgQID();
	void increaseMsgSent();
	void increaseMsgFail();
	

protected:
	// New function codes
    int HytecPowerControl_;
#define FIRST_HYTEC_PARAM HytecPowerControl_
    int HytecBrakeControl_;
    int HytecFWVersion_;
    int HytecMoveAll_;
#define LAST_HYTEC_PARAM HytecMoveAll_

#define NUM_HYTEC_PARAMS (&LAST_HYTEC_PARAM - &FIRST_HYTEC_PARAM + 1)

private:
	asynStatus SetupCard();
	int checkprom(char *pr,int expmodel);

    int numAxes;
    epicsMessageQueueId intMsgQId;
    int messagesSent;                       // for report
    int messagesFailed;                     // for report
    int ip_carrier;
    int ipslot;

    asynUser *pasynUser;

    int card;
    int vector;
    int useencoder;
    volatile char *regbase;

    epicsThreadId motorThread;

friend class HytecMotorAxis;
};
