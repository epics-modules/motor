/*
FILENAME...     devSoft.cc
USAGE...        Motor record device level support for Soft channel.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 06/15/99
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 *
 * .00 02-06-02 rls - Don't process from events unless interruptAccept is TRUE.
 *                  - When done transitions from false to true it is not
 *                    communicated to the motor record until after the last
 *                    readback update.
 *                  - In soft_process(), call dbProcess() instead of directly
 *                    calling motor record's process().
 *                  - In soft_rdbl_func(), reset motor record's target to actual
 *                    position after last readback if motion was not initiated
 *                    by this record.
 * .01 10-29-02 rls - LOCK field added to prevent synchronization due to
 *                    changing readback.
 * .02 06-16-03 rls   Convert to R3.14.x.
 * .03 08-03-05 rls - Added debug messages.
 *                  - Fix compiler error with "gcc version 3.4.2 20041017 (Red
 *                    Hat 3.4.2-6.fc3)".
 */


/*
NOTES...
- Can't call CA functions until after dbLockInitRecords() has been called and initialized lock sets.   
*/


#include        <dbDefs.h>
#include        <dbFldTypes.h>
#include        <dbAccess.h>
#include        <dbEvent.h>
#include        <recSup.h>

#include        "motorRecord.h"
#include        "motor.h"
#include        "devSoft.h"

#include        "epicsExport.h"
#include        "errlog.h"

/*----------------debugging-----------------*/
volatile int devSoftdebug = 0;
extern "C"
{epicsExportAddress(int, devSoftdebug);}
static inline void Debug(int level, const char *format, ...)
{
#ifdef DEBUG
    if (level < devSoftdebug)
    {
        va_list pVar;
        va_start(pVar, format);
        vprintf(format, pVar);
        va_end(pVar);
    }
#endif
}

static CALLBACK_VALUE update(struct motorRecord *);
static long start(struct motorRecord *);
static RTN_STATUS build(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS end(struct motorRecord *);
static void soft_process(struct motorRecord *);


struct motor_dset devMotorSoft =
{
    {8, NULL, (DEVSUPFUN) soft_init, (DEVSUPFUN) soft_init_record, NULL},
    update,
    start,
    build,
    end
};

extern "C" {epicsExportAddress(dset, devMotorSoft);}

static CALLBACK_VALUE update(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;
    msta_field status;

#ifdef DMR_SOFTMOTOR_MODS
    if (ptr->load_position)
    {
        mr->rmp = ptr->new_position;
        mr->rep = ptr->new_position;
        ptr->load_position = FALSE;
    }
#endif

    status.All = mr->msta;

    Debug(5, "update(): dmov=%d for %s.\n", mr->dmov, mr->name);

    if (ptr->dinp_value == SOFTMOVE || ptr->dinp_value == HARDMOVE)
    {
        Debug(5, "update(): DMOV=0 for %s.\n", mr->name);
        status.Bits.RA_DONE = 0;
    }
    else
    {
        Debug(5, "update(): DMOV=1 for %s.\n", mr->name);
        status.Bits.RA_DONE = 1;
    }
    mr->msta = status.All;
    return(ptr->callback_flag);
}


static long start(struct motorRecord *mr)
{
    return((long) OK);
}


static RTN_STATUS end(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    if (ptr->default_done_behavior == YES)
    {
        msta_field status;

        status.All = 0;
        status.Bits.RA_DONE = 1;
        mr->msta = status.All;
    }
    return(OK);
}


static RTN_STATUS build(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    const short int stop = 1;
    long int status = 0;

    switch (command)
    {
        case MOVE_ABS:
        case MOVE_REL:
            status = dbPutLink(&mr->out, DBR_DOUBLE, &mr->dval, 1);
            break;

        case STOP_AXIS:
            status = dbPutLink(&mr->stoo, DBR_SHORT, &stop, 1);
            break;

        case SET_HIGH_LIMIT:
        case SET_LOW_LIMIT:
            status = OK;
            break;

        case LOAD_POS:
            {
                struct soft_private *ptr = (struct soft_private *) mr->dpvt;
                msta_field msta;

#ifdef DMR_SOFTMOTOR_MODS
                ptr->load_position = TRUE;
                ptr->new_position = *parms;
#endif
                msta.All = 0;
                msta.Bits.RA_DONE = 1;
                mr->msta = msta.All;
                callbackRequest(&ptr->callback);
            }
            break;

        default:
            status = ERROR;
    }
    return(status == 0 ? OK : ERROR);
}


/*
FUNCTION... void soft_dinp_func(struct motorRecord *, short)
USAGE...    Update soft channel device input links and
    process soft channel motor record when done moving.
LOGIC...
    IF DINP link value is FALSE.
    IF this soft motor's DMOV is FALSE.
        This is a soft motor initiated move.
        Set SOFTMOVE indicator.
    ELSE IF LOCK field set to NO.
        This is NOT a soft motor initiated move.
        Set HARDMOVE indicator.
        Set soft motor's DMOV FALSE.
        Set PostProcess (PP) TRUE.
    ENDIF
    ELSE
    IF DINP state is HARDMOVE.
        Set PostProcess (PP) True.
    Set DINP state to DONE.
    Process soft channel record.
    ENDIF    
*/
void soft_dinp_func(struct motorRecord *mr, short newdinp)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    if (interruptAccept != TRUE)
        return;

    /* Test for hard motor started moving or initialization. */
    if (newdinp == 0)
    {
        if (mr->dmov == FALSE)
        {
            Debug(5, "soft_dinp_func(): SOFTMOVE set for %s.\n", mr->name);
            ptr->dinp_value = SOFTMOVE;
        }
        else if (mr->lock == menuYesNoNO)
        {   /* Hard motor is moving independent of soft motor. */
            Debug(5, "soft_dinp_func(): HARDMOVE set for %s.\n", mr->name);
            ptr->dinp_value = HARDMOVE;
            mr->dmov = FALSE;
            db_post_events(mr, &mr->dmov, DBE_VAL_LOG);
            mr->pp = TRUE;
            db_post_events(mr, &mr->pp, DBE_VAL_LOG);
        }
    }
    else        /* Hard motor is done moving. */
    {
        if (ptr->dinp_value == HARDMOVE)
            mr->pp = TRUE;
        ptr->dinp_value = DONE;
        Debug(5, "soft_dinp_func(): Done moving set for %s.\n", mr->name);
        soft_process(mr);       /* Process in case there is no readback callback. */
    }
}


void soft_rinp_func(struct motorRecord *mr, long newrinp)
{
    if (interruptAccept != TRUE)
        return;

    mr->rmp = newrinp;
    soft_process(mr);
}


void soft_rdbl_func(struct motorRecord *mr, double newrdbl)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    if (interruptAccept != TRUE)
        return;

    newrdbl = newrdbl / mr->mres;
    mr->rmp = NINT(newrdbl);

    Debug(5, "soft_rdbl_func(): updated RMP = %d for %s.\n", mr->rmp, mr->name);

    if (ptr->initialized == false)
    {
        /* Reset Target to Actual position. */
        unsigned short mask = (DBE_VALUE | DBE_LOG);

        mr->dmov = FALSE;
        db_post_events(mr, &mr->dmov, mask);
        mr->pp = TRUE;
        db_post_events(mr, &mr->pp, mask);

        ptr->dinp_value = DONE;
        ptr->initialized = true;
    }
    soft_process(mr);
}


/*
FUNCTION... static void soft_process(struct motorRecord *)
USAGE...    Process the soft channel motor record.
LOGIC...
    Lock soft channel record - call dbScanLock().
    Set call back flag to CALLBACK_DATA so readback will get updated.
    Process soft channel record - call process().
    Unlock soft channel record - call dbScanUnlock().
*/

static void soft_process(struct motorRecord *mr)
{
    struct soft_private *ptr = (struct soft_private *) mr->dpvt;

    dbScanLock((struct dbCommon *) mr);
    ptr->callback_flag = CALLBACK_DATA;
    dbProcess((struct dbCommon *) mr);  /* Process the soft channel record. */
    ptr->callback_flag = NOTHING_DONE;
    dbScanUnlock((struct dbCommon *) mr);
}


/*
FUNCTION... void soft_motor_callback(CALLBACK *)
USAGE...    Process motor record after the following events:
            - LOAD_POS motor command.
LOGIC...
*/

void soft_motor_callback(CALLBACK *cbptr)
{
    void *mr;

    callbackGetUser(mr, cbptr);
    soft_process((motorRecord *) mr);
}

