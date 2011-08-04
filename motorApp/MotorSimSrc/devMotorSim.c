/* devXxxSoft.c */
/* Example device support module */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "epicsFindSymbol.h"
#include "dbAccess.h"
#include "recGbl.h"
#include "registryDriverSupport.h"
#include "drvSup.h"
/* #include "callback.h" */

#include "motorRecord.h"
#include "motor.h"
#include "epicsExport.h"
#include "motor_interface.h"

/*Create the dset for devMotor */
static long init_record(struct motorRecord *);
static CALLBACK_VALUE update_values(struct motorRecord *);
static long start_trans(struct motorRecord *);
static RTN_STATUS build_trans( motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS end_trans(struct motorRecord *);

struct motor_dset devMotorSim={ { 8,
				NULL,
				NULL,
				(DEVSUPFUN) init_record,
                                  NULL },
				update_values,
				start_trans,
				build_trans,
				end_trans };

epicsExportAddress(dset,devMotorSim);

#define SET_BIT(val,mask,set) ((set)? ((val) | (mask)): ((val) & ~(mask)))

/**\struct motorStatus_t

    This structure is returned by motorAxisGetStatus and contains all
    the current information required by the motor record to indicate
    current motor status. It should probably be extended to include
    all of status information which might be useful (i.e. current
    values of all parameter values that can be set).

    Note that the updateCount can be used to indicate whether any
    parameters have been changes since the last call to the motorAxisGetStatus routine.

*/
typedef struct motorStatus_t 
{
    epicsUInt32 status;  /**< bit mask of errors and other binary information. The bit positions are in motor.h */
    epicsInt32  position;         /**< Current motor position in motor steps (if not servoing) or demand position (if servoing) */
    epicsInt32  encoder_position; /**< Current motor position in encoder units (only available if a servo system). */
} motorStatus_t;

typedef struct
{
  motorAxisDrvSET_t * drvset;
  AXIS_HDL pAxis;
  struct motorRecord * pRec;
  motorStatus_t status;
  double min_vel;
  double max_vel;
  double acc;
  motor_cmnd move_cmd;
  double param;
  int needUpdate;
} motorPrvt_t;



void motor_callback( void * param, unsigned int nReasons, unsigned int * reasons )
{
  struct motorRecord * pRec = (struct motorRecord *) param;
  motorPrvt_t * pPrvt = (motorPrvt_t *) pRec->dpvt;
  AXIS_HDL pAxis = pPrvt->pAxis;
  motorStatus_t * status = &(pPrvt->status);
  int i;

  for ( i = 0; i < nReasons; i++ )
  {
      switch (reasons[i])
      {
      case motorAxisEncoderPosn:
          pPrvt->drvset->getInteger( pAxis, reasons[i], &(status->encoder_position) );
          break;
      case motorAxisPosition:
          pPrvt->drvset->getInteger( pAxis, reasons[i], &(status->position) );
          break;
      case motorAxisDirection:
      case motorAxisDone:
      case motorAxisHighHardLimit:
      case motorAxisHomeSignal:
      case motorAxisSlip:
      case motorAxisPowerOn:
      case motorAxisFollowingError:
      case motorAxisHomeEncoder:
      case motorAxisHasEncoder:
      case motorAxisProblem:
      case motorAxisMoving:
      case motorAxisHasClosedLoop:
      case motorAxisCommError:
      case motorAxisLowHardLimit:
      {
          int flag;
          int mask = (0x1<<(reasons[i]-motorAxisDirection));

          pPrvt->drvset->getInteger( pAxis, reasons[i], &flag );
          status->status = SET_BIT( status->status, mask, flag );
          break;
      }
      default:
          break;
      }
  }

  if (nReasons > 0)
  {
      pPrvt->needUpdate = 1;
      scanOnce( (struct dbCommon *)pRec );
  }
}
       

static long init_record(struct motorRecord * pRec )
{
    if (pRec->out.type == VME_IO)
      {
	/* I should extract the first word of the parameter as the driver support entry table name,
	   and pass the rest to the driver, but I am a bit lazy at the moment */
	motorAxisDrvSET_t * drvset = (motorAxisDrvSET_t *) registryDriverSupportFind( pRec->out.value.vmeio.parm );

	if (drvset != NULL &&
	    drvset->open != NULL )
	  {
	    AXIS_HDL axis = (*drvset->open)( pRec->out.value.vmeio.card,
					     pRec->out.value.vmeio.signal,
					     pRec->out.value.vmeio.parm );
	    if (axis != NULL)
	      {
		pRec->dpvt = calloc( 1, sizeof( motorPrvt_t ) );
		if (pRec->dpvt != NULL)
                {
                    int i;
		    motorPrvt_t * pPrvt = (motorPrvt_t *) pRec->dpvt;

		    pPrvt->drvset = drvset;
		    pPrvt->pAxis = axis;
		    pPrvt->pRec = pRec;
		    pPrvt->move_cmd = -1;
                    pPrvt->drvset->getInteger( axis, motorAxisEncoderPosn, &(pPrvt->status.encoder_position) );
                    pPrvt->drvset->getInteger( axis, motorAxisPosition, &(pPrvt->status.position) );

                    /* Set the status bits */
                    for ( i = motorAxisDirection; i <= motorAxisLowHardLimit; i++ )
                    {
		      /* Set the default to be zero for unsupported flags */
                        int flag=0;
                        int mask = (0x1<<(i-motorAxisDirection));

                        pPrvt->drvset->getInteger( axis, i, &flag );
                        pPrvt->status.status = SET_BIT( pPrvt->status.status, mask, flag );
                    }

		    (*drvset->setCallback)( axis, motor_callback, (void *) pRec );
                }
		else 
		  {
		    if (drvset->close) (*drvset->close)( axis );
		    recGblRecordError(S_drv_noDrvSup,(void *)pRec,
				      "devMotor (init_record) cannot open driver support");
		    return S_db_noMemory;
		  }
	      }
	    else
	      {
		recGblRecordError(S_drv_noDrvSup,(void *)pRec,
				  "devMotor (init_record) cannot open device support");
		return S_db_noSupport;
	      }
	  }
	else
	  {
	    recGblRecordError(S_drv_noDrvet,(void *)pRec,
			      "devMotor (init_record) cannot find device support entry table");
	    return S_db_noSupport;
	  }
      }
    else
      {
	recGblRecordError(S_db_badField,(void *)pRec,
			  "devMotor (init_record) Illegal INP field");
        return(S_db_badField);
      }

    return(0);
}

CALLBACK_VALUE update_values(struct motorRecord * pRec)
{
  motorPrvt_t * pPrvt = (motorPrvt_t *) pRec->dpvt;
  motorStatus_t stat = pPrvt->status;
  CALLBACK_VALUE rc;

  rc = NOTHING_DONE;

  if ( pPrvt->needUpdate )
  {
      pRec->rmp = stat.position;
      pRec->rep = stat.encoder_position;
      /* pRec->rvel = ptrans->vel; */
      pRec->msta = stat.status;
      rc = CALLBACK_DATA;
      pPrvt->needUpdate = 0;
  }
  return (rc);
}

static long start_trans(struct motorRecord * pRec )
{
    return(OK);
}

static RTN_STATUS build_trans( motor_cmnd command, 
			       double * param,
			       struct motorRecord * pRec )
{
  RTN_STATUS status = OK;
  motorPrvt_t * pPrvt = (motorPrvt_t *) pRec->dpvt;

  switch ( command )
    {
    case MOVE_ABS:
    case MOVE_REL:
    case HOME_FOR:
    case HOME_REV:
      pPrvt->move_cmd = command;
      pPrvt->param = *param;
      break;
    case LOAD_POS:
        status = (*pPrvt->drvset->setDouble)(pPrvt->pAxis, motorAxisPosition, *param );
      break;
    case SET_VEL_BASE:
      pPrvt->min_vel = *param;
      break;
    case SET_VELOCITY:
      pPrvt->max_vel = *param;
      break;
    case SET_ACCEL:
      pPrvt->acc = *param;
      break;
    case GO:
      switch (pPrvt->move_cmd)
	{
	case MOVE_ABS:
	  status = (*pPrvt->drvset->move)(pPrvt->pAxis, pPrvt->param, 0, 
					  pPrvt->min_vel, pPrvt->max_vel, pPrvt->acc);
	  break;
	case MOVE_REL:
	  status = (*pPrvt->drvset->move)(pPrvt->pAxis, pPrvt->param, 1, 
					  pPrvt->min_vel, pPrvt->max_vel, pPrvt->acc);
	  break;
	case HOME_FOR:
	  status = (*pPrvt->drvset->home)(pPrvt->pAxis, pPrvt->min_vel, pPrvt->max_vel, pPrvt->acc, 1);
	  break;
	case HOME_REV:
	  status = (*pPrvt->drvset->home)(pPrvt->pAxis, pPrvt->min_vel, pPrvt->max_vel, pPrvt->acc, 0);
	  break;
	default:
	  status = ERROR;
	}
      pPrvt->move_cmd = -1;
      break;
    case SET_ENC_RATIO:
      status = (*pPrvt->drvset->setDouble)(pPrvt->pAxis, motorAxisEncoderRatio, param[0]/param[1] );
      break;
    case GET_INFO:
      pPrvt->needUpdate = 1; 
      break;
    case STOP_AXIS:
      status = (*pPrvt->drvset->stop)(pPrvt->pAxis, pPrvt->acc );
      break;
    case JOG:
      status = (*pPrvt->drvset->velocityMove)(pPrvt->pAxis, pPrvt->min_vel, *param, pPrvt->acc);
      break;
    case JOG_VELOCITY:
      status = (*pPrvt->drvset->velocityMove)(pPrvt->pAxis, pPrvt->min_vel, *param, pPrvt->acc);
      break;
    case SET_PGAIN:
      status = (*pPrvt->drvset->setDouble)(pPrvt->pAxis, motorAxisPGain, *param );
      break;
    case SET_IGAIN:
      status = (*pPrvt->drvset->setDouble)(pPrvt->pAxis, motorAxisIGain, *param );
      break;
    case SET_DGAIN:
      status = (*pPrvt->drvset->setDouble)(pPrvt->pAxis, motorAxisIGain, *param );
      break;
    case ENABLE_TORQUE:
      status = (*pPrvt->drvset->setInteger)(pPrvt->pAxis, motorAxisClosedLoop, 1 );
      break;
    case DISABL_TORQUE:
      status = (*pPrvt->drvset->setInteger)(pPrvt->pAxis, motorAxisClosedLoop, 0 );
      break;
    case SET_HIGH_LIMIT:
      status = (*pPrvt->drvset->setDouble)(pPrvt->pAxis, motorAxisHighLimit, *param );
      break;
    case SET_LOW_LIMIT:
      status = (*pPrvt->drvset->setDouble)(pPrvt->pAxis, motorAxisLowLimit, *param );
      break;
    default:
      status = ERROR;
    }

  return(status);
}

static RTN_STATUS end_trans(struct motorRecord * pRec )
{
/*   motorPrvt_t * pPrvt = (motorPrvt_t *) pRec->dpvt; */

/*   callbackRequestProcessCallback( &(pPrvt->callback), priorityLow, pRec ); */
  return(OK);
}
