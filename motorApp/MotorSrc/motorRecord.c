/*
FILENAME...	motorRecord.c
USAGE...	Record Support Routines for the Motor record.

Version:	$Revision: 1.6 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2000-08-18 19:44:37 $
*/

/*
 *		Original Author: Jim Kowalkowski
 *		Previous Author: Tim Mooney
 *		Current Author: Ron Sluiter
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *	Copyright 1995, 1996 the University of Chicago Board of Governors.
 *
 *	This software was produced under U.S. Government contract:
 *	(W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *	Developed by
 *		The Beamline Controls and Data Acquisition Group
 *		Experimental Facilities Division
 *		Advanced Photon Source
 *		Argonne National Laboratory
 *
 *	Co-developed with
 *		The Controls and Computing Group
 *		Accelerator Systems Division
 *		Advanced Photon Source
 *		Argonne National Laboratory
 *
 *
 * Modification Log:
 * -----------------
 * .01  01-18-93 jbk  initial development
 * .02  01-25-93 tmm  initial development continued
 * .03  09-13-93 tmm  disabled HOPR & LOPR with #define USE_HOPR_LOPR 0
 * .04  08-01-94 tmm  move to 3.11.6 (tsLocalTime -> recGblGetTimeStamp)
 * .05  09-07-94 tmm  make .diff zero when we set drive fields to readback
 *                    fields
 * .06  09-07-94 tmm  test for stop and spmg every time record processes
 * .07  11-15-94 jps  create .rvel to indicate motor actually moving
 * .08  06-12-95 tmm  added VERSION and .vers field, fixed stuck .dmov flag
 * .09  06-14-95 tmm  propagate dial-value calibration to user value;
 *                    fix precision and units assoc. with .vers field
 * .10  07-20-95 tmm  added record name to debug output
 * .11  07-20-95 tmm  check that we've actually moved before worrying whether
 *                    we're going in the wrong direction
 * .12  08-23-95 tmm  version 1.5  removed HOPR/LOPR code, added dial dir
 *                    (sign of .mres).  Fixed coding error in jog*.  No
 *                    longer propagate dial-value calib. to user value
 * .13  08-25-95 tmm  version 1.51  limit switches now reported w.r.t user
 *                    coordinates.  Added raw limit switches .rhls, .rlls.
 * .14  08-28-95 tmm  version 1.6  Added .card field, so users can know which
 *                    card in crate is associated with this motor.
 * .15  08-30-95 tmm  version 1.7  Added .nmap field to supplement .mmap, and
 *                    rewrote MARK() macros.  
 * .16  08-31-95 tmm  version 1.8  Freeze-offset switch (.foff) replaces earlier
 *                    "propagate dial-value calibration to user value"
 * .17  09-01-95 tmm  limit-violation changes.
 * .18  09-07-95 tmm  version 1.9  Added .fof / .vof fields to put .foff (freeze
 *                    offset) into frozen / variable mode.
 *                    Added .urev, .srev, .s, .sbas, .sbak fields to meet user
 *                    demands for more intuitive setting of speeds and
 *                    resolution.
 *                    Modified offset handling.  Changing .off should never
 *                    cause the motor to move.
 * .19  09-08-95 tmm  version 1.91  Work around 8-character limit in handling
 *                    by dbGet() of the string returned by get_units().
 * .20  09-13-95 tmm  version 1.92  Allow .rdbl (readback inlink) and .rlnk
 *                    (readout outlink) to be channel-access links.
 * .21  09-26-95 tmm  version 1.93  Post process (as if STOP had been hit)
 *                    when a limit switch is hit.  When post process sets,
 *                    e.g., VAL=RBV, then also set LVAL=RBV.
 * .22  11-21-95 tmm  Version 1.94  Fix post_MARKed_fields.  Fix pmr->pp bug
 *                    that caused post processing every time.
 * .23  12-27-95 tmm  Version 1.95  Implemented MISS field.  More sanity checks
 *                    on resolution, speed, and acceleration.
 * .24  02-09-96 tmm  Version 1.96  Don't db_post dmov==0 if special() already
 *                    posted it.
 * .25  03-19-96 tmm  v1.97  Much more precise encoder-ratio calculation.
 *                    Made actual home speed independent of UEIP flag.
 *                    (Thanks to Vicky Austin, Royal Greenwich Observatory.)
 * .26  06-07-96 tmm  v1.98 Added more debugging output.
 * .27  06-10-96 tmm  v2.00 Fixed the way changes to resolutions and speeds are
 *                    handled in init_record() and in special().  Now, when
 *                    motor resolution is changed, speeds in revs/sec are held
 *                    constant, which requires speeds in EGUs/sec to change.
 * .28  08-02-96 tmm  v2.1 Changing mres or eres while in SET mode no longer
 *                    changes the raw motor position, but instead changes user
 *                    and dial positions to agree with the old raw position.
 * .29  08-09-96 tmm  v2.2 Changed the order in which PV's are posted.  DMOV is
 *                    last; frequently changed PV's are first.
 * .30  08-14-96 tmm  v2.3 Fixed long-standing bug in special handling of DVAL,
 *                    RVAL, RLV.
 * .31  08-14-96 tmm  v3.0 Conversion to EPICS 3.13.0
 * .32  12-16-96 tmm  v3.1 Fixed moving-in-right-dir check, and jog, when using
 *                    relative moves.  Fixed JOG limits.
 * .33  05-29-97 tmm  v3.2 Watchdog callback for encoder-settling time delay.
 *                    Added DLY field to implement delay.
 *                    Split postProcess() and maybeRetry() out of process().
 *                    Fix problem of first call to process() looking like a
 *                    device-support callback (first move command fails).
 *                    Don't do mid-course checks if using encoder or readback
 *                    device.  (We don't know how badly they might behave while
 *                    the motor's moving.)  Set .res field in init_record().
 *                    Set retry deadband according to .mres, .eres, and .bdst.
 * .34  05-30-97 tmm  v3.3 Relative moves know about motor granularity.
 * .35  10-11-99 rls  v4.0 Enforce finite state machine on jog request, jogging,
 *			stopping a jog and backlash after jogging.  These
 *			modifications fix the "runaway jog" problem discovered
 *			with the MM4000 device support using serial communication
 *			when the user "hammers" on the jog request.
 */

#define VERSION 4.2

#include	<vxWorks.h>
#include	<stdlib.h>
#include	<string.h>
#include	<wdLib.h>
#include	<alarm.h>
#include	<dbDefs.h>

#ifdef __cplusplus
extern "C" {
#include	<callback.h>
#include	<dbAccess.h>
#include	<dbScan.h>
#include	<recSup.h>
#include	<dbEvent.h>
}
#else
#include	<callback.h>
#include	<dbAccess.h>
#include	<dbScan.h>
#include	<recSup.h>
#include	<dbEvent.h>
#endif

#include	<dbFldTypes.h>
#include	<devSup.h>
#include	<errMdef.h>
#include	<math.h>

#define GEN_SIZE_OFFSET
#ifdef __cplusplus
extern "C" {
#include	"motorRecord.h"
}
#else
#include	"motorRecord.h"
#endif
#undef GEN_SIZE_OFFSET

#include	"motor.h"


#define STATIC static
#define MAX(a,b) ((a)>(b)?(a):(b))	/* nearest integer */

/*** Forward references ***/

STATIC long do_work(motorRecord *);
STATIC void alarm(motorRecord *);
STATIC void monitor(motorRecord *);
STATIC void post_MARKed_fields(motorRecord *, unsigned short);
STATIC void process_motor_info(motorRecord *);
STATIC void load_pos(motorRecord *);
STATIC void check_speed_and_resolution(motorRecord *);
STATIC void set_dial_highlimit(motorRecord *, struct motor_dset *);
STATIC void set_dial_lowlimit(motorRecord *, struct motor_dset *);
STATIC void set_userlimits(motorRecord *);
STATIC void range_check(motorRecord *, float *, double, double);

/*** Record Support Entry Table (RSET) functions. ***/

STATIC long init_record(motorRecord *, int);
STATIC long process(motorRecord *);
STATIC long special(struct dbAddr *, int);
STATIC long get_units(struct dbAddr *, char *);
STATIC long get_precision(struct dbAddr *, long *);
STATIC long get_graphic_double(struct dbAddr *, struct dbr_grDouble *);
STATIC long get_control_double(struct dbAddr *, struct dbr_ctrlDouble *);
STATIC long get_alarm_double(struct dbAddr *, struct dbr_alDouble *);

/* record support entry table */
#ifdef __cplusplus
struct local_rset
{
    long number;			/*number of support routines*/
    long (*report) (FILE, int);		/*print report*/
    long (*init) (void);		/*init support*/
    long (*init_record) (void *, int);	/*init support for particular record*/
    long (*process) (void *);
    long (*special) (struct dbAddr *, int);
    long (*get_value) (void *, struct valueDes *);
    long (*cvt_dbaddr) (struct dbAddr *);
    long (*get_array_info) (struct dbAddr *, long *, long *);
    long (*put_array_info) (struct dbAddr *, long);
    long (*get_units) (struct dbAddr *, char *);
    long (*get_precision) (struct dbAddr *, long *);
    long (*get_enum_str) (struct dbAddr *, char *);
    long (*get_enum_strs) (struct dbAddr *, struct dbr_enumStrs *);
    long (*put_enum_str) (struct dbAddr *, char *);
    long (*get_graphic_double) (struct dbAddr *, struct dbr_grDouble *);
    long (*get_control_double) (struct dbAddr *, struct dbr_ctrlDouble *);
    long (*get_alarm_double) (struct dbAddr *, struct dbr_alDouble *);
};
#endif

#ifdef __cplusplus
struct local_rset motorRSET =
#else
struct rset motorRSET =
#endif
{
    RSETNUMBER,
    NULL,
    NULL,
    init_record,
    process,
    special,
    NULL,
    NULL,
    NULL,
    NULL,
    get_units,
    get_precision,
    NULL,
    NULL,
    NULL,
    get_graphic_double,
    get_control_double,
    get_alarm_double
};


/*******************************************************************************
Support for tracking the progress of motor from one invocation of 'process()'
to the next.  The field 'pmr->mip' stores the motion in progress using these
fields.  ('pmr' is a pointer to motorRecord.)
*******************************************************************************/
#define MIP_DONE	0x0000	/* No motion is in progress.                 */
#define MIP_JOGF	0x0001	/* A jog-forward command is in progress.     */
#define MIP_JOGR	0x0002	/* A jog-reverse command is in progress.     */
#define MIP_JOG_BL	0x0004	/* Done jogging; now take out backlash.      */
#define MIP_JOG		(MIP_JOGF | MIP_JOGR | MIP_JOG_BL)
#define MIP_HOMF	0x0008	/* A home-forward command is in progress.    */
#define MIP_HOMR	0x0010	/* A home-reverse command is in progress.    */
#define MIP_HOME	(MIP_HOMF | MIP_HOMR)
#define MIP_MOVE	0x0020	/* A move not resulting from Jog* or Hom*.   */
#define MIP_RETRY	0x0040	/* A retry is in progress.                   */
#define MIP_LOAD_P	0x0080	/* A load-position command is in progress.   */
#define MIP_LOAD_ER	0x0100	/* A load-encoder-ratio command in progress. */
#define MIP_STOP	0x0200	/* We're trying to stop.  When combined with */
/*                                 MIP_JOG* or MIP_HOM*, the jog or home     */
/*                                 command is performed after motor stops    */
#define MIP_DELAY_REQ	0x0400	/* We set the delay watchdog */
#define MIP_DELAY_ACK	0x0800	/* Delay watchdog is calling us back */
#define MIP_DELAY	(MIP_DELAY_REQ | MIP_DELAY_ACK)	/* Waiting for readback
							 * to settle */
#define MIP_JOG_REQ	0x1000	/* Jog Request. */
#define MIP_JOG_STOP	0x2000	/* Stop jogging. */

/*******************************************************************************
Support for keeping track of which record fields have been changed, so we can
eliminate redundant db_post_events() without having to think, and without having
to keep lots of "last value of field xxx" fields in the record.  The idea is
to say...

	MARK(M_XXXX);

when you mean...

	db_post_events(pmr, &pmr->xxxx, monitor_mask);

Before leaving, you have to call post_MARKed_fields() to actually post the
field to all listeners.  monitor() does this.

	--- NOTE WELL ---
	The macros below assume that the variable "pmr" exists and points to a
	motor record, like so:
		motorRecord *pmr;
	No check is made in this code to ensure that this really is true.
*******************************************************************************/
/* Bit field for "mmap". */
typedef union
{
    unsigned long All;
    struct
    {
	unsigned long M_VAL	:1;
	unsigned long M_DVAL	:1;
	unsigned long M_HLM	:1;
	unsigned long M_LLM	:1;
	unsigned long M_DMOV	:1;
	unsigned long M_SPMG	:1;
	unsigned long M_RCNT	:1;
	unsigned long M_MRES	:1;
	unsigned long M_ERES	:1;
	unsigned long M_UEIP	:1;
	unsigned long M_URIP	:1;
	unsigned long M_LVIO	:1;
	unsigned long M_RVAL	:1;
	unsigned long M_RLV	:1;
	unsigned long M_OFF	:1;
	unsigned long M_RBV	:1;
	unsigned long M_DHLM	:1;
	unsigned long M_DLLM	:1;
	unsigned long M_DRBV	:1;
	unsigned long M_RDBD	:1;
	unsigned long M_MOVN	:1;
	unsigned long M_HLS	:1;
	unsigned long M_LLS	:1;
	unsigned long M_RRBV	:1;
	unsigned long M_RMP	:1;
	unsigned long M_REP	:1;
	unsigned long M_MSTA	:1;
	unsigned long M_ATHM	:1;
	unsigned long M_TDIR	:1;
	unsigned long M_MIP	:1;
	unsigned long M_DIFF	:1;
	unsigned long M_RDIF	:1;
    } Bits;
} mmap_field;

/* Bit field for "nmap". */
typedef union
{
    unsigned long All;
    struct
    {
	unsigned long M_S	:1;
	unsigned long M_SBAS	:1;
	unsigned long M_SBAK	:1;
	unsigned long M_SREV	:1;
	unsigned long M_UREV	:1;
	unsigned long M_VELO	:1;
	unsigned long M_VBAS	:1;
	unsigned long M_BVEL	:1;
	unsigned long M_MISS	:1;
	unsigned long M_ACCL	:1;
	unsigned long M_BACC	:1;
    } Bits;
} nmap_field;


#define MARK(FIELD) {mmap_field temp; temp.All = pmr->mmap; \
		    temp.Bits.FIELD = 1; pmr->mmap = temp.All;}
#define MARK_AUX(FIELD) {nmap_field temp; temp.All = pmr->nmap; \
		    temp.Bits.FIELD = 1; pmr->nmap = temp.All;}

#define UNMARK(FIELD) {mmap_field temp; temp.All = pmr->mmap; \
		    temp.Bits.FIELD = 0; pmr->mmap = temp.All;}
#define UNMARK_AUX(FIELD) {nmap_field temp; temp.All = pmr->nmap; \
		    temp.Bits.FIELD = 0; pmr->nmap = temp.All;}

/*
WARNING!!! The following macros assume that a variable (i.e., mmap_bits
	and/or nmap_bits) has been declared within the scope its' occurence
	AND initialized.
*/
		
#define MARKED(FIELD) (mmap_bits.Bits.FIELD)
#define MARKED_AUX(FIELD) (nmap_bits.Bits.FIELD)

#define UNMARK_ALL	pmr->mmap = pmr->nmap = 0

/*******************************************************************************
Device support allows us to string several motor commands into a single
"transaction", using the calls prototyped below:

	int start_trans(dbCommon *mr)
	int build_trans(int command, double *parms, dbCommon *mr)
	int end_trans(struct dbCommon *mr, int go)

For clarity and to avoid typo's, the macros defined below provide simplified
calls.

		--- NOTE WELL ---
	The following macros assume that the variable "pmr" points to a motor
	record, and that the variable "pdset" points to that motor record's device
	support entry table:
		motorRecord *pmr;
		struct motor_dset *pdset = (struct motor_dset *)(pmr->dset);

	No checks are made in this code to ensure that these conditions are met.
*******************************************************************************/
/* To begin a transaction... */
#define INIT_MSG()				(*pdset->start_trans)(pmr)

/* To send a single command... */
#define WRITE_MSG(cmd,parms)	(*pdset->build_trans)((cmd), (parms), pmr)

/* To end a transaction and send accumulated commands to the motor... */
#define SEND_MSG()				(*pdset->end_trans)(pmr)


/*
The DLY feature uses the VxWorks watchdog facility to issue a callbackRequest()
on the structure below.  This structure is dynamically allocated by
init_record().  init_record() saves the pointer to this structure in the
motorRecord.  See process() for use of this structure when Done Moving field
(DMOV) is TRUE.
*/

struct callback		/* DLY feature callback structure. */
{
    CALLBACK callback;
    struct motorRecord *precord;
    WDOG_ID wd_id;
};

STATIC void callbackFunc(struct callback * pcb)
{
    motorRecord *pmr = pcb->precord;

    /*
     * It's possible user has requested stop, or in some other way rescinded
     * the delay request that resulted in this callback.  Check to make sure
     * this callback hasn't been orphaned by events occurring between the time
     * the watchdog was started and the time this function was invoked.
     */
    if (pmr->mip & MIP_DELAY_REQ)
    {
	pmr->mip |= MIP_DELAY_ACK;
	scanOnce(pmr);
    }
}


/******************************************************************************
	enforceMinRetryDeadband()

Calculate minumum retry deadband (.rdbd) achievable under current
circumstances, and enforce this minimum value.
******************************************************************************/
STATIC void enforceMinRetryDeadband(motorRecord * pmr)
{
    float min_rdbd;
    int encoder = (pmr->msta & EA_PRESENT) && pmr->ueip;
    int backlash = pmr->bdst > 0.;

    if (backlash && encoder)
	min_rdbd = 0.51 * (fabs(pmr->eres) + fabs(pmr->mres));
    else if (encoder)
	min_rdbd = 0.51 * MAX(fabs(pmr->eres), fabs(pmr->mres));
    else
	min_rdbd = 0.51 * fabs(pmr->mres);

    if (pmr->rdbd < min_rdbd)
    {
	pmr->rdbd = min_rdbd;
	db_post_events(pmr, &pmr->rdbd, DBE_VALUE);
    }
}


/******************************************************************************
	init_record()

Called twice after an EPICS database has been loaded, and then never called
again.
*******************************************************************************/
STATIC long init_record(motorRecord * pmr, int pass)
{
    struct motor_dset *pdset;
    long status;
    struct callback *pcallback;	/* v3.2 */
    const unsigned short monitor_mask = DBE_VALUE;
    const char errmsg[] = "motor:init_record()";

    if (pass == 0)
    {
	pmr->vers = VERSION;
	return(OK);
    }
    /* Check that we have a device-support entry table. */
    pdset = (struct motor_dset *) pmr->dset;
    if (pdset == NULL)
    {
	recGblRecordError(S_dev_noDSET, (void *) pmr, (char *) errmsg);
	return (S_dev_noDSET);
    }
    /* Check that DSET has pointers to functions we need. */
    if ((pdset->base.number < 8) ||
	(pdset->update_values == NULL) ||
	(pdset->start_trans == NULL) ||
	(pdset->build_trans == NULL) ||
	(pdset->end_trans == NULL))
    {
	recGblRecordError(S_dev_missingSup, (void *) pmr, (char *) errmsg);
	return (S_dev_missingSup);
    }

    /*** setup callback for readback settling time delay (v3.2) ***/
    pcallback = (struct callback *) (calloc(1, sizeof(struct callback)));
    pmr->cbak = (void *) pcallback;
    callbackSetCallback((void (*)(struct callbackPvt *)) callbackFunc,
			&pcallback->callback);
    callbackSetPriority(pmr->prio, &pcallback->callback);
    pcallback->precord = pmr;
    pcallback->wd_id = wdCreate();

    /*
     * Reconcile two different ways of specifying speed and resolution; make
     * sure things are sane.
     */
    check_speed_and_resolution(pmr);

    /* Call device support to initialize itself and the driver */
    if (pdset->base.init_record)
    {
	status = (*pdset->base.init_record) (pmr);
	if (status)
	{
	    pmr->card = -1;
	    return (status);
	}
	switch (pmr->out.type)
	{
	    case (VME_IO):
		pmr->card = pmr->out.value.vmeio.card;
		break;
	    case (CONSTANT):
	    case (PV_LINK):
	    case (DB_LINK):
	    case (CA_LINK):
		pmr->card = -1;
		break;
	    default:
		recGblRecordError(S_db_badField, (void *) pmr, (char *) errmsg);
		return(ERROR);
	}
    }
    /*
     * .dol (Desired Output Location) is a struct containing either a link to
     * some other field in this database, or a constant intended to initialize
     * the .val field.  If the latter, get that initial value and apply it.
     */
    if (pmr->dol.type == CONSTANT)
    {
	pmr->udf = FALSE;
	recGblInitConstantLink(&pmr->dol, DBF_DOUBLE, &pmr->val);
    }

    /*
     * Get motor position, encoder position, status, and readback-link value by
     * calling process_motor_info().
     * 
     * v3.2 Fix so that first call to process() doesn't appear to be a callback
     * from device support.  (Reset ptrans->callback_changed to NO in devSup).
     */
    (*pdset->update_values) (pmr);

    if (pmr->eres == 0.0)
    {
	pmr->eres = pmr->mres;
	MARK(M_ERES);
    }

    /* v3.2 Set .res according to whether an encoder is in use. */
    pmr->res = ((pmr->msta & EA_PRESENT) && pmr->ueip) ? pmr->eres : pmr->mres;

    process_motor_info(pmr);
    enforceMinRetryDeadband(pmr);

    /*
     * If we're in closed-loop mode, initializing the user- and dial-coordinate
     * motor positions (.val and .dval) is someone else's job. Otherwise,
     * initialize them to the readback values (.rbv and .drbv) set by our
     * recent call to process_motor_info().
     */
    if (pmr->omsl != CLOSED_LOOP)
    {
	pmr->val = pmr->rbv;
	MARK(M_VAL);
	pmr->dval = pmr->drbv;
	MARK(M_DVAL);
	pmr->rval = pmr->rrbv;
	MARK(M_RVAL);
    }

    /* Reset limits in case database values are invalid. */
    set_dial_highlimit(pmr, pdset);
    set_dial_lowlimit(pmr, pdset);

    /* Initialize miscellaneous control fields. */
    pmr->dmov = TRUE;
    MARK(M_DMOV);
    pmr->movn = FALSE;
    MARK(M_MOVN);
    pmr->lspg = pmr->spmg = motorSPMG_Go;
    MARK(M_SPMG);
    pmr->pp = TRUE;
    pmr->diff = pmr->dval - pmr->drbv;
    MARK(M_DIFF);
    pmr->rdif = NINT(pmr->diff / pmr->mres);
    MARK(M_RDIF);
    pmr->lval = pmr->val;
    pmr->ldvl = pmr->dval;
    pmr->lrvl = pmr->rval;
    pmr->lvio = 0;		/* init limit-violation field */
    if ((pmr->drbv > pmr->dhlm + pmr->res) ||
	(pmr->drbv < pmr->dllm - pmr->res))
    {
	pmr->lvio = 1;
	MARK(M_LVIO);
    }

    /* Make sure readback-delay field accurately conveys the time delay we */
    /* can actually implement (nearest number of ticks of watchdog timer.) */
    pmr->dly = (NINT(vxTicksPerSecond * pmr->dly)) / (float) vxTicksPerSecond;
    db_post_events(pmr, &pmr->dly, monitor_mask);

    monitor(pmr);
    return(OK);
}


/******************************************************************************
	postProcess()

Post process a command or motion after motor has stopped. We do this for
any of several reasons:
	1) This is the first call to process()
	2) User hit a "Stop" button, and motor has stopped.
	3) User released a "Jog*" button and motor has stopped.
	4) Hom* command has completed.
	5) User hit Hom* or Jog* while motor was moving, causing a
		'stop' to be sent to the motor, and the motor has stopped.
	6) User caused a new value to be written to the motor hardware's
		position register.
	7) We hit a limit switch.
LOGIC:
    Clear post process command field; PP.
    IF Output Mode Select field set to CLOSED_LOOP.
	Make drive values agree with readback value;
	    VAL  <- RBV
	    DVAL <- DRBV
	    RVAL <- RRBV
	    DIFF <- RDIF <- 0
    ENDIF
    IF done with either load-position or load-encoder-ratio commands.
	Clear MIP.
    ELSE IF done homing.
	...
	...
    ELSE IF done stopping after jog.
	Clear DMOV and JOG_STOP state in MIP.
	...
	...
    ELSE IF done with backlash after jog.
	Clear MIP.
	IF (JOGF field ON, AND, Hard High limit OFF), OR,
		(JOGR field ON, AND, Hard Low  limit OFF)
	    Set Jog request state ON.
	ENDIF
    ENDIF
    
    
******************************************************************************/
STATIC long postProcess(motorRecord * pmr)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    const unsigned short monitor_mask = DBE_VALUE;

    pmr->pp = 0;

    if (pmr->omsl != CLOSED_LOOP)
    {
	/* Make drive values agree with readback value. */
	pmr->val = pmr->rbv;
	MARK(M_VAL);
	pmr->dval = pmr->drbv;
	MARK(M_DVAL);
	pmr->rval = pmr->rrbv;
	MARK(M_RVAL);
	pmr->diff = 0.;
	MARK(M_DIFF);
	pmr->rdif = 0;
	MARK(M_RDIF);
    }

    if (pmr->mip & (MIP_LOAD_P | MIP_LOAD_ER))
    {
	/* We sent LOAD_POS or SET_ENC_RATIO, followed by GET_INFO. */
	pmr->mip = 0;
	MARK(M_MIP);

    }
    else if (pmr->mip & MIP_HOME)
    {
	/* Home command */
	if (pmr->mip & MIP_STOP)
	{
	    /* Stopped and Hom* button still down.  Now do Hom*. */
	    double vbase = pmr->vbas / fabs(pmr->res);
	    double hvel = 1000 * fabs(pmr->mres / pmr->eres);
	    double hpos = 0;

	    if (hvel <= vbase)
		hvel = vbase + 1;
	    pmr->mip &= ~MIP_STOP;
	    MARK(M_MIP);
	    pmr->dmov = FALSE;
	    MARK(M_DMOV);
	    pmr->rcnt = 0;
	    MARK(M_RCNT);
	    INIT_MSG();
	    WRITE_MSG(SET_VEL_BASE, &vbase);
	    WRITE_MSG(SET_VELOCITY, &hvel);
	    WRITE_MSG((pmr->mip & MIP_HOMF) ? HOME_FOR : HOME_REV, &hpos);
	    WRITE_MSG(GO, NULL);
	    SEND_MSG();
	    pmr->pp = TRUE;
	}
	else
	{
	    if (pmr->mip & MIP_HOMF)
	    {
		pmr->mip &= ~MIP_HOMF;
		MARK(M_MIP);
		pmr->homf = 0;
		db_post_events(pmr, &pmr->homf, monitor_mask);
	    }
	    else if (pmr->mip & MIP_HOMR)
	    {
    
		pmr->mip &= ~MIP_HOMR;
		MARK(M_MIP);
		pmr->homr = 0;
		db_post_events(pmr, &pmr->homr, monitor_mask);
	    }
	}
    }
    else if (pmr->mip & MIP_JOG_STOP)
    {
	pmr->mip &= ~MIP_JOG_STOP;
	if (fabs(pmr->bdst) >  fabs(pmr->res))
	{
	    /* First part of jog done. Do backlash correction. */
	    double bvel = pmr->bvel / fabs(pmr->res);
	    double bacc = bvel / pmr->bacc;
	    double vbase = pmr->vbas / fabs(pmr->res);
	    double vel = pmr->velo / fabs(pmr->res);
	    double acc = vel / pmr->accl;
	    double bpos = (pmr->dval - pmr->bdst) / pmr->res;
	    double currpos = pmr->dval / pmr->res;
	    double newpos;

	    /* Use if encoder or ReadbackLink is in use. */
	    int use_rel = ((pmr->msta & EA_PRESENT) && pmr->ueip) || pmr->urip;
	    double relpos = pmr->diff / pmr->res;
	    double relbpos = ((pmr->dval - pmr->bdst) - pmr->drbv) / pmr->res;

	    pmr->dmov = FALSE;
	    MARK(M_DMOV);
	    pmr->mip = MIP_JOG_BL;
	    MARK(M_MIP);
	    pmr->pp = TRUE;

	    INIT_MSG();

	    WRITE_MSG(SET_VEL_BASE, &vbase);
	    if (vel <= vbase)
		vel = vbase + 1;
	    WRITE_MSG(SET_VELOCITY, &vel);
	    WRITE_MSG(SET_ACCEL, &acc);
	    if (use_rel)
		WRITE_MSG(MOVE_REL, &relbpos);
	    else
		WRITE_MSG(MOVE_ABS, &bpos);
	    WRITE_MSG(GO, NULL);

	    if (bvel <= vbase)
		bvel = vbase + 1;
	    WRITE_MSG(SET_VELOCITY, &bvel);
	    WRITE_MSG(SET_ACCEL, &bacc);
	    if (use_rel)
	    {
		relpos = (relpos - relbpos) * pmr->frac;
		WRITE_MSG(MOVE_REL, &relpos);
	    }
	    else
	    {
		newpos = bpos + pmr->frac * (currpos - bpos);
		pmr->rval = NINT(newpos);
		WRITE_MSG(MOVE_ABS, &newpos);
	    }
	    WRITE_MSG(GO, NULL);
	    SEND_MSG();
	}
	else
	{
	    pmr->mip = 0;	/* Backup distance = 0; skip backlash. */
	    MARK(M_MIP);
	    if ((pmr->jogf && !pmr->hls) || (pmr->jogr && !pmr->lls))
		pmr->mip |= MIP_JOG_REQ;
	}
    }
    else if (pmr->mip & MIP_JOG_BL)
    {
	/* Completed backlash part of jog command. */
	pmr->mip = 0;
	MARK(M_MIP);
	if ((pmr->jogf && !pmr->hls) || (pmr->jogr && !pmr->lls))
	    pmr->mip |= MIP_JOG_REQ;
    }
    /* Save old values for next call. */
    pmr->lval = pmr->val;
    pmr->ldvl = pmr->dval;
    pmr->lrvl = pmr->rval;
    pmr->mip &= ~MIP_STOP;
    MARK(M_MIP);
    return(OK);
}


/******************************************************************************
	maybeRetry()

Compare target with actual position.  If retry is indicated, set variables so
that it will happen when we return.
******************************************************************************/
STATIC void maybeRetry(motorRecord * pmr)
{
    if ((fabs(pmr->diff) > pmr->rdbd) && !pmr->hls && !pmr->lls)
    {
	/* No, we're not close enough.  Try again. */
	/* If max retry count is zero, retry is disabled */
	if (pmr->rtry == 0)
	{
	    pmr->mip &= MIP_JOG_REQ;/* Clear everything, except jog request; for
					jog reactivation in postProcess(). */
	    MARK(M_MIP);
	}
	else
	{
	    if (++(pmr->rcnt) > pmr->rtry)
	    {
		/* Too many retries. */
		/* pmr->spmg = motorSPMG_Pause; MARK(M_SPMG); */
		pmr->mip = 0;
		MARK(M_MIP);
		pmr->lval = pmr->val;
		pmr->ldvl = pmr->dval;
		pmr->lrvl = pmr->rval;

		/* We should probably be triggering alarms here. */
		pmr->miss = 1;
		MARK_AUX(M_MISS);
	    }
	    else
	    {
		pmr->dmov = FALSE;
		MARK(M_DMOV);
		pmr->mip = MIP_RETRY;
		MARK(M_MIP);
	    }
	    MARK(M_RCNT);
	}
    }
    else
    {
	/* Yes, we're close enough to the desired value. */
	pmr->mip &= MIP_JOG_REQ;/* Clear everything, except jog request; for
				    jog reactivation in postProcess(). */
	MARK(M_MIP);
	if (pmr->miss)
	{
	    pmr->miss = 0;
	    MARK_AUX(M_MISS);
	}

	/* If motion was initiated by "Move" button, pause. */
	if (pmr->spmg == motorSPMG_Move)
	{
	    pmr->spmg = motorSPMG_Pause;
	    MARK(M_SPMG);
	}
    }
}


/******************************************************************************
	process()

Called under many different circumstances for many different reasons.

1) Someone poked our .proc field, or some other field that is marked
'process-passive' in the motorRecord.ascii file.  In this case, we
determine which fields have changed since the last time we were invoked
and attempt to act accordingly.

2) Device support will call us periodically while a motor is moving, and
once after it stops.  In these cases, we infer that device support has
called us by looking at the flag it set, report the motor's state, and
fire off readback links.  If the motor has stopped, we fire off forward links
as well.

Note that this routine handles all motor records, and that several 'copies'
of this routine may execute 'simultaneously' (in the multitasking sense), as
long as they operate on different records.  This much is normal for an EPICS
record, and the normal mechanism for ensuring that a record does not get
processed by more than one 'simultaneous' copy of this routine (the .pact field)
works here as well.

However, it is normal for an EPICS record to be either 'synchronous' (runs
to completion at every invocation of process()) or 'asynchronous' (begins
processing at one invocation and forbids all further invocations except the
callback invocation from device support that completes processing).  This
record is worse than asynchronous because we can't forbid invocations while
a motor is moving (else a motor could not be stopped), nor can we complete
processing until a motor stops.

Backlash correction would complicate this picture further, since a motor
must stop before backlash correction starts and stops it again, but device
support and the Oregon Microsystems controller allow us to string two move
commands together--even with different velocities and accelerations.

Backlash-corrected jogs (move while user holds 'jog' button down) do
complicate the picture:  we can't string the jog command together with a
backlash correction because we don't know when the user is going to release
the jog button.  Worst of all, it is possible for the user to give us a
'jog' command while the motor is moving.  Then we have to do the following
in separate invocations of process():
	tell the motor to stop
	handle motor-in-motion callbacks while the motor slows down
	recognize the stopped-motor callback and begin jogging
	handle motor-in-motion callbacks while the motor jogs
	recognize when the user releases the jog button and tell the motor to stop
	handle motor-in-motion callbacks while the motor slows down
	recognize the stopped-motor callback and begin a backlash correction
	handle motor-in-motion callbacks while the motor is moving
	recognize the stopped-motor callback and fire off forward links
For this reason, a fair amount of code is devoted to keeping track of
where the motor is in a sequence of movements that comprise a single motion.

LOGIC:
    Initialize.
    IF this record is being processed by another task (i.e., PACT != 0).
    	NORMAL RETURN.
    ENDIF
    Set Processing Active indicator field (PACT) ON.
    Call device support update_values().
    IF motor status field (MSTA) was modified.
    	Mark MSTA as changed.
    ENDIF
    IF function was invoked by a callback, OR, process delay acknowledged is ON?
	Set process reason indicator to CALLBACK_DATA.
	Call process_motor_info().
	IF motor-in-motion indicator (MOVN) is ON.
	    IF [The UEIP {"Use Encoder If Present"} is set to NO], AND,
	       [The URIP {"Use RDBL Link If Present"} is set to NO], AND,
	       [Dist. to target {DIFF} > 2 x (|Backlash Dist.| + Retry Deadband)], AND,
	       [Previous RDIF (pre_rdif) is nonzero], AND
	       [The polarity of the raw dist. to target {RDIF} has changed], AND,
	       [Raw Velocity is nonzero], AND,
	       [Raw Readback Value {RRBV} has been changed], AND,
	       [MIP indicates this move is either (a result of a retry),OR,
	    		(not from a Jog* or Hom*)]
		Send Stop Motor command.
		Set STOP indicator in MIP ON.
		Mark MIP as changed.
	    ENDIF
	ELSE
	    Set the Done Moving field (DMOV) TRUE and mark DMOV as changed.
	    IF the High or Low limit switch is TRUE.
		Set the Post Process field to TRUE.
	    ENDIF
	    IF the Post Process field is TRUE.
		Call postProcess().
	    ENDIF
	    IF the Done Moving field (DMOV) is TRUE.
		Initialize delay ticks.
		IF process delay acknowledged is ON, OR, ticks <= 0.
		    Clear process delay request and ack. indicators in MIP field.
		    Mark MIP as changed.
		    Call maybeRetry().
		ELSE
		    Set process delay request indicator ON in MIP field.
		    Mark MIP as changed.
		    Start WatchDog?
		    Set the Done Moving field (DMOV) to FALSE.
		    Set Processing Active indicator field (PACT) OFF.
		    NORMAL RETURN.
		ENDIF
	    ENDIF
	ENDIF
	Save previous RDIF (pre_rdif) for target direction reversal detection in
		above Stop logic.
    ENDIF
    IF Jog indicator is ON in MIP field.
	Update Limit violation (LVIO) based on Jog direction (JOGF/JOGR) and VELO.
    ELSE IF Homing indicator is ON in MIP field.
	Update Limit violation (LVIO) based on Home direction (HOMF/HOMR) and VELO.
    ELSE
	Update Limit violation (LVIO).
    ENDIF
    IF Limit violation (LVIO) has changed.
	Mark LVIO as changed.
	IF Limit violation (LVIO) is TRUE, AND, SET is OFF (i.e., Use/Set is Set).
	    Set STOP field ON.
	    Clear JOGF and JOGR fields.
	ENDIF
    ENDIF
    IF STOP field is ON, OR,
       SPMG field Stop indicator is ON, OR,
       SPMG field Pause indicator is ON, OR,
       function was NOT invoked by a callback, OR,
       Done Moving field (DMOV) is TRUE, OR,
       RETRY indicator is ON in MIP field.
	Call do_work().
    ENDIF
    Update Readback output link (RLNK), call dbPutLink().
    IF Done Moving field (DMOV) is TRUE.
	Process the forward-scan-link record, call recGblFwdLink().
    ENDIF
    Update record timestamp, call recGblGetTimeStamp().
    Process alarms, call alarm().
    Monitor changes to record fields, call monitor().
    Set Processing Active indicator field (PACT) OFF.
    Exit.

*******************************************************************************/
STATIC long process(motorRecord * pmr)
{
    long status = OK, process_reason;
    int old_lvio = pmr->lvio;
    unsigned int old_msta = pmr->msta;
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    struct callback *pcallback = (struct callback *) pmr->cbak;	/* v3.2 */

    if (pmr->pact)
	return(OK);

    pmr->pact = 1;

    /*** Who called us? ***/
    /*
     * Call device support to get raw motor position/status and to see whether
     * this is a callback.
     */
    process_reason = (*pdset->update_values) (pmr);
    if (pmr->msta != old_msta)
	MARK(M_MSTA);

    if ((process_reason == CALLBACK_DATA) || (pmr->mip & MIP_DELAY_ACK))
    {
	/*
	 * This is, effectively, a callback from device support: a
	 * motor-in-motion update, some asynchronous acknowledgement of a
	 * command we sent in a previous life, or a callback thay we requested
	 * to delay while readback device settled.
	 */

	/*
	 * If we were invoked by the readback-delay callback, then this is just
	 * a continuation of the device-support callback.
	 */
	process_reason = CALLBACK_DATA;

	/*
	 * Get position and status from motor controller. Get readback-link
	 * value if link exists.
	 */
	process_motor_info(pmr);

	/* For Soft Channel device support; skip most record processing if
	 * called due to readback updates.
	 */
	if (pmr->dmov)
	    goto process_exit;

	if (pmr->movn)
	{
            mmap_field mmap_bits;

	    mmap_bits.All = pmr->mmap; /* Initialize for MARKED. */

	    /* This is a motor-in-motion update from device support. */
	    /*
	     * Are we going in the wrong direction?  (Don't be fooled by a
	     * backlash correction into saying "yes". 'Jog*' and 'Hom*' motions
	     * don't get midcourse corrections since we don't know their
	     * destinations.) v3.2: Don't do this check with an encoder or
	     * readback device.
	     */

	    if (!pmr->ueip &&
		!pmr->urip &&
		(fabs(pmr->diff) > 2 * (fabs(pmr->bdst) + pmr->rdbd)) &&
		(pmr->pdif != 0) &&
		((pmr->rdif & 0x80000000) != (pmr->pdif & 0x80000000)) &&
		(pmr->rvel != 0) &&
		(MARKED(M_RRBV)) &&
		((pmr->mip == MIP_RETRY) || (pmr->mip == MIP_MOVE)))
	    {

		/* We're going in the wrong direction. Readback problem? */
		printf("%s:tdir = %d\n", pmr->name, pmr->tdir);
		INIT_MSG();
		WRITE_MSG(STOP_AXIS, NULL);
		SEND_MSG();
		pmr->mip |= MIP_STOP;
		MARK(M_MIP);
	    }
	    status = 0;
	}
	else
	{
	    /* Motor has stopped. */
	    /* Assume we're done moving until we find out otherwise. */
	    pmr->dmov = TRUE;
	    MARK(M_DMOV);

	    if (pmr->hls || pmr->lls)
		pmr->pp = TRUE;

	    if (pmr->pp)
	    {
		status = postProcess(pmr);
	    }			/* if (pmr->pp) */

	    /* Are we "close enough" to desired position? */
	    if (pmr->dmov)
	    {
		int ticks = (int) NINT(vxTicksPerSecond * pmr->dly);

		if (pmr->mip & MIP_DELAY_ACK || (ticks <= 0))
		{
		    pmr->mip &= ~MIP_DELAY;
		    MARK(M_MIP);/* done delaying */
		    maybeRetry(pmr);
		}
		else
		{
		    pmr->mip |= MIP_DELAY_REQ;
		    MARK(M_MIP);
		    status = wdStart(pcallback->wd_id, ticks,
				  (FUNCPTR) callbackRequest, (int) pcallback);
		    pmr->dmov = FALSE;
		    pmr->pact = 0;
		    return(OK);
		}
	    }
	}
	pmr->pdif = pmr->rdif;
    }				/* if (process_reason == CALLBACK_DATA) */

    /* check for soft-limit violation */
    if (pmr->mip & MIP_JOG)
	pmr->lvio = (pmr->jogf && (pmr->drbv > pmr->dhlm - pmr->velo)) ||
	    (pmr->jogr && (pmr->drbv < pmr->dllm + pmr->velo));
    else if (pmr->mip & MIP_HOME)
	pmr->lvio = (pmr->homf && (pmr->drbv > pmr->dhlm - pmr->velo)) ||
	    (pmr->homr && (pmr->drbv < pmr->dllm + pmr->velo));
    else
	pmr->lvio = (pmr->drbv > pmr->dhlm + fabs(pmr->res)) ||
	    (pmr->drbv < pmr->dllm - fabs(pmr->res));

    if (pmr->lvio != old_lvio)
    {
	MARK(M_LVIO);
	if (pmr->lvio && !pmr->set)
	{
	    pmr->stop = 1;
	    pmr->jogf = pmr->jogr = 0;
	}
    }
    /* Do we need to examine the record to figure out what work to perform? */
    if (pmr->stop || (pmr->spmg == motorSPMG_Stop) ||
	(pmr->spmg == motorSPMG_Pause) ||
	(process_reason != CALLBACK_DATA) || pmr->dmov || pmr->mip & MIP_RETRY)
    {
	status = do_work(pmr);
    }

    /* Fire off readback link */
    status = dbPutLink(&(pmr->rlnk), DBR_DOUBLE, &(pmr->rbv), 1);

    if (pmr->dmov)
	recGblFwdLink(pmr);	/* Process the forward-scan-link record. */
    
process_exit:    
    /*** We're done.  Report the current state of the motor. ***/
    recGblGetTimeStamp(pmr);
    alarm(pmr);			/* If we've violated alarm limits, yell. */
    monitor(pmr);		/* If values have changed, broadcast them. */
    pmr->pact = 0;
    return (status);
}


/******************************************************************************
	do_work()
Here, we do the real work of processing the motor record.

The equations that transform between user and dial coordinates follow.
Note: if user and dial coordinates differ in sign, we have to reverse the
sense of the limits in going between user and dial.

Dial to User:
userVAL	= DialVAL * DIR + OFFset
userHLM	= (DIR==+) ? DialHLM + OFFset : -DialLLM + OFFset
userLLM = (DIR==+) ? DialLLM + OFFset : -DialHLM + OFFset

User to Dial:
DialVAL	= (userVAL - OFFset) / DIR
DialHLM	= (DIR==+) ? userHLM - OFFset : -userLLM + OFFset
DialLLM = (DIR==+) ? userLLM - OFFset : -userHLM + OFFset

Offset:
OFFset	= userVAL - DialVAL * DIR

LOGIC:
    Initialize.
    
    IF Stop button activated, AND, NOT processing a STOP request.
	Set MIP field to indicate processing a STOP request.
	Mark MIP field as changed.  Set Post process command field TRUE.
	Clear Jog forward and reverse request.  Clear Stop request.
	Send STOP_AXIS message to controller.
    	NORMAL RETURN.
    ENDIF
    
    IF Stop/Pause/Move/Go field has changed.
        Update Last Stop/Pause/Move/Go field.
	IF SPMG field set to STOP, OR, PAUSE.
	    Set MIP field to indicate processing a STOP request.
	    Mark MIP field changed.
	    Send STOP_AXIS message to controller.
	    IF SPMG field set to STOP.
		IF Motor is moving (MOVN).
		    Set Post process command TRUE.
		ELSE
		    Set VAL <- RBV and mark as changed.
		    Set DVAL <- DRBV and mark as changed.
		    Set RVAL <- RRBV and mark as changed.
		ENDIF
	    ENDIF
	    NORMAL RETURN.
	ELSE
	    Clear MIP and RCNT. Mark both as changed.
	ENDIF
    ENDIF
    
    IF MRES, OR, ERES, OR, UEIP are marked as changed.
	IF UEIP set to YES, AND, MSTA indicates an encoder is present.
	    IF |MRES| and/or |ERES| is very near zero.
		Set MRES and/or ERES to one (1.0).
	    ENDIF
	    Set sign of ERES to same sign as MRES.
	    .....
	    .....
	ELSE
	    Set the [encoder (ticks) / motor (steps)] ratio to unity (1).
	    Set RES <- MRES.
	ENDIF
	- call enforceMinRetryDeadband().
	IF MSTA indicates an encoder is present.
	    Send the ticks/steps ratio motor command.
	ENDIF
	IF the SET position field is ON.
	    Set the PP field TRUE and send the update info. motor command.
	ELSE
	    - call load_pos().
	ENDIF
	NORMAL RETURN
    ENDIF
    
    IF OMSL set to CLOSED_LOOP, AND, DOL type set to DB_LINK.
	Use DOL field to get DB link - call dbGetLink().
	IF error return from dbGetLink().
	    Set Undefined Link indicator (UDF) TRUE.
	    ERROR RETURN.
	ENDIF
	Set Undefined Link indicator (UDF) FALSE.
    ELSE
	IF No Limit violation, AND, (Homing forward/OR/reverse request, AND,
		NOT processing Homing forward/OR/reverse, AND, NOT At
		High/OR/Low Limit Switch)
	    IF (STOPPED, OR, PAUSED)
		Set DMOV FALSE (Home command will be processed from
		    postProcess() when SPMG is set to GO).
	    ENDIF
	    ...
	    ...
	    NORMAL RETURN.
	ENDIF
	IF NOT currently jogging, AND, NOT (STOPPED, OR, PAUSED), AND,
		No Limit violation, AND, Jog Request is ON.
	    IF (Forward jog, AND, DVAL > [DHLM - VELO]), OR,
	       (Reverse jog, AND, DVAL > [DLLM + VELO])
		Set limit violation (LVIO) ON.
		NORMAL RETURN.
	    ENDIF
	    Set Jogging [forward/reverse] state ON.
	    ...
	    ...
	    NORMAL RETURN
	ENDIF
	IF Jog request is OFF, AND, jog is active.
	    Set post process TRUE.
	    Send STOP_AXIS message to controller.
	ELSE IF process jog stop or backlash.
	    NORMAL RETURN.  NOTE: Don't want "DVAL has changed..." logic to
			    get processed.
	ENDIF
    ENDIF
    
    IF VAL field has changed.
	Mark VAL changed.
	IF the SET position field is ON, AND, the FOFF field is "Variable".
	    ....
	ELSE
	    Calculate DVAL based on VAL, OFF and DIR.
	ENDIF
    ENDIF

    Update LVIO field.
    
    IF LVIO field has changed.
        Mark LVIO field.
    ENDIF
    
    IF Limit violation occurred.
    ENDIF
    
    IF Stop/Pause/Move/Go field set to STOP, OR, PAUSE.
    	NORMAL RETURN.
    ENDIF
    
    IF DVAL field has changed, OR, NOT done moving.
	Mark DVAL as changed.
	Calculate new DIFF and RDIF fields and mark as changed.
	IF the SET position field is ON.
	    Load new raw motor position w/out moving it - call load_pos().
	    NORMAL RETURN.
	ELSE
	    Calculate....
	    
	    IF new raw commanded position = current raw feedback position.
		IF not done moving, AND, [either no motion-in-progress, OR,
					    retry-in-progress].
		    Set done moving TRUE.
		    NORMAL RETURN.
		    NOTE: maybeRetry() can send control here even though the
			move is to the same raw position.
		ENDIF
	    ENDIF
	    ....
	    ....
	    IF motion in progress indicator is OFF.
		Set MIP MOVE indicator ON.
		.....
		.....
		.....
		Send message to controller.
	    ENDIF
	ENDIF
    ENDIF

    NORMAL RETURN.
    
    
*******************************************************************************/
STATIC long do_work(motorRecord * pmr)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    int dir_positive = (pmr->dir == motorDIR_Pos);
    int dir = dir_positive ? 1 : -1;
    int set = pmr->set;
    int stopped = (pmr->spmg == motorSPMG_Stop || pmr->spmg == motorSPMG_Pause);
    int old_lvio = pmr->lvio;
    const unsigned short monitor_mask = DBE_VALUE;
    mmap_field mmap_bits;

    /*** Process Stop button. ***/
    if (pmr->stop && (pmr->mip != MIP_STOP))
    {
	/* Stop motor. */
	pmr->pp = TRUE;
	pmr->jogf = pmr->jogr = 0;
	pmr->stop = 0;
	goto stop_all;
    }

    /*** Process Stop/Pause/Go_Pause/Go switch. ***
    *
    * STOP	means make the motor stop and, when it does, make the drive
    *       fields (e.g., .val) agree with the readback fields (e.g., .rbv)
    *       so the motor stays stopped until somebody gives it a new place
    *       to go and sets the switch to MOVE or GO.
    *
    * PAUSE	means stop the motor like the old steppermotorRecord stops
    *       a motor:  At the next call to process() the motor will continue
    *       moving to .val.
    *
    * MOVE	means Go to .val, but then wait for another explicit Go or
    *       Go_Pause before moving the motor, even if the .dval field
    *       changes.
    *
    * GO	means Go, and then respond to any field whose change causes
    *       .dval to change as if .dval had received a dbPut().
    *       (Implicit Go, as implemented in the old steppermotorRecord.)
    *       Note that a great many fields (.val, .rvl, .off, .twf, .homf,
    *       .jogf, etc.) can make .dval change.
    */
    if (pmr->spmg != pmr->lspg)
    {
	pmr->lspg = pmr->spmg;
	if (pmr->spmg == motorSPMG_Stop ||
	    pmr->spmg == motorSPMG_Pause)
	{
	    /*
	     * If STOP, make drive values agree with readback values (when the
	     * motor actually stops).
	     */
	    if (pmr->spmg == motorSPMG_Stop)
	    {
		if (pmr->movn)
		    pmr->pp = TRUE;	/* Do when motor stops. */
		else
		{
		    pmr->val = pmr->rbv;
		    MARK(M_VAL);
		    pmr->dval = pmr->drbv;
		    MARK(M_DVAL);
		    pmr->rval = pmr->rrbv;
		    MARK(M_RVAL);
		}
	    }
stop_all:   /* Cancel any operations. */
	    if (pmr->mip & MIP_HOMF)
	    {
		pmr->homf = 0;
		db_post_events(pmr, &pmr->homf, monitor_mask);
	    }
	    else if (pmr->mip & MIP_HOMR)
	    {
		pmr->homr = 0;
		db_post_events(pmr, &pmr->homr, monitor_mask);
	    }
	    pmr->mip = MIP_STOP;
	    MARK(M_MIP);
	    INIT_MSG();
	    WRITE_MSG(STOP_AXIS, NULL);
	    SEND_MSG();
	    return(OK);
	}
	/* Test for "queued" jog request. */
	else if (pmr->spmg == motorSPMG_Go && ((pmr->jogf && !pmr->hls) || (pmr->jogr && !pmr->lls)))
	    pmr->mip = MIP_JOG_REQ;
	else
	{
	    pmr->mip = 0;
	    MARK(M_MIP);
	    pmr->rcnt = 0;
	    MARK(M_RCNT);
	}
    }

    /*** Handle changes in motor/encoder resolution, and in .ueip. ***/
    mmap_bits.All = pmr->mmap; /* Initialize for MARKED. */
    if (MARKED(M_MRES) || MARKED(M_ERES) || MARKED(M_UEIP))
    {
	/* encoder pulses, motor pulses */
	double ep_mp[2];
	long m;

	/* Set the encoder ratio.  Note this is blatantly device dependent. */
	if ((pmr->msta & EA_PRESENT) && pmr->ueip)
	{
	    /* defend against divide by zero */
	    if (fabs(pmr->mres) < 1.e-9)
	    {
		pmr->mres = 1.;
		MARK(M_MRES);
	    }
	    if (pmr->eres == 0.0)
	    {
		pmr->eres = pmr->mres;
		MARK(M_ERES);
	    }
	    /*
	     * OMS hardware can't handle negative motor or encoder resolution
	     * in the SET_ENCODER_RATIO command. For now, we simply don't allow
	     * motor and encoder resolutions to differ in sign.
	     */
	    if ((pmr->mres < 0.) != (pmr->eres < 0.))
	    {
		pmr->eres *= -1.;
		MARK(M_ERES);
	    }
	    /* Calculate encoder ratio. */
	    for (m = 10000000; (m > 1) &&
		 (fabs(m / pmr->eres) > 1.e6 || fabs(m / pmr->mres) > 1.e6); m /= 10);
	    ep_mp[0] = fabs(m / pmr->eres);
	    ep_mp[1] = fabs(m / pmr->mres);
	    /* Select encoder resolution for use in later calculations. */
	    pmr->res = pmr->eres;
	}
	else
	{
	    ep_mp[0] = 1.;
	    ep_mp[1] = 1.;
	    pmr->res = pmr->mres;
	}

	/* Make sure retry deadband is achievable */
	enforceMinRetryDeadband(pmr);

	if (pmr->msta & EA_PRESENT)
	{
	    INIT_MSG();
	    WRITE_MSG(SET_ENC_RATIO, ep_mp);
	    SEND_MSG();
	}
	if (pmr->set)
	{
	    pmr->pp = TRUE;
	    INIT_MSG();
	    WRITE_MSG(GET_INFO, NULL);
	    SEND_MSG();
	}
	else
	    load_pos(pmr);

	return(OK);
    }
    /*** Collect .val (User value) changes from all sources. ***/
    if (pmr->omsl == CLOSED_LOOP && pmr->dol.type == DB_LINK)
    {
	/** If we're in CLOSED_LOOP mode, get value from input link. **/
	long status;

	status = dbGetLink(&(pmr->dol), DBR_DOUBLE, &(pmr->val), NULL, NULL);
	if (!RTN_SUCCESS(status))
	{
	    pmr->udf = TRUE;
	    return (1);
	}
	pmr->udf = FALSE;
	/* Later, we'll act on this new value of .val. */
    }
    else
    {
	/** Check out all the buttons and other sources of motion **/

	/* Send motor to home switch in forward direction. */
	if (!pmr->lvio &&
	    ((pmr->homf && !(pmr->mip & MIP_HOMF) && !pmr->hls) ||
	     (pmr->homr && !(pmr->mip & MIP_HOMR) && !pmr->lls)))
	{
	    if (stopped)
	    {
		pmr->dmov = FALSE;
		MARK(M_DMOV);
    		return(OK);
	    }
	    /* check for limit violation */
	    if ((pmr->homf && (pmr->dval > pmr->dhlm - pmr->velo)) ||
		(pmr->homr && (pmr->dval < pmr->dllm + pmr->velo)))
	    {
		pmr->lvio = 1;
		MARK(M_LVIO);
		return(OK);
	    }
	    pmr->mip = pmr->homf ? MIP_HOMF : MIP_HOMR;
	    MARK(M_MIP);
	    pmr->pp = TRUE;
	    if (pmr->movn)
	    {
		pmr->mip |= MIP_STOP;
		MARK(M_MIP);
		INIT_MSG();
		WRITE_MSG(STOP_AXIS, NULL);
		SEND_MSG();
	    }
	    else
	    {
		double vbase, hvel, hpos;

		/* defend against divide by zero */
		if (pmr->eres == 0.0)
		{
		    pmr->eres = pmr->mres;
		    MARK(M_ERES);
		}

		vbase = pmr->vbas / fabs(pmr->res);
		hvel = 1000 * fabs(pmr->mres / pmr->eres);
		hpos = 0;

		INIT_MSG();
		WRITE_MSG(SET_VEL_BASE, &vbase);
		if (hvel <= vbase)
		    hvel = vbase + 1;
		WRITE_MSG(SET_VELOCITY, &hvel);
		WRITE_MSG((pmr->mip & MIP_HOMF) ? HOME_FOR : HOME_REV, &hpos);
		/*
		 * WRITE_MSG(SET_VELOCITY, &hvel); WRITE_MSG(MOVE_ABS, &hpos);
		 */
		WRITE_MSG(GO, NULL);
		SEND_MSG();
		pmr->dmov = FALSE;
		MARK(M_DMOV);
		pmr->rcnt = 0;
		MARK(M_RCNT);
	    }
	    return(OK);
	}
	/*
	 * Jog motor.  Move continuously until we hit a software limit or a
	 * limit switch, or until user releases button.
	 */
	if (!(pmr->mip & MIP_JOG) && !stopped && !pmr->lvio && (pmr->mip & MIP_JOG_REQ))
	{
	    /* check for limit violation */
	    if ((pmr->jogf && (pmr->dval > pmr->dhlm - pmr->velo)) ||
		(pmr->jogr && (pmr->dval < pmr->dllm + pmr->velo)))
	    {
		pmr->lvio = 1;
		MARK(M_LVIO);
		return(OK);
	    }
	    pmr->mip = pmr->jogf ? MIP_JOGF : MIP_JOGR;
	    MARK(M_MIP);
	    if (pmr->movn)
	    {
		pmr->pp = TRUE;
		pmr->mip |= MIP_STOP;
		MARK(M_MIP);
		INIT_MSG();
		WRITE_MSG(STOP_AXIS, NULL);
		SEND_MSG();
	    }
	    else
	    {
		double jogv = (pmr->velo * dir) / pmr->res;
		double jacc = fabs(jogv) / pmr->accl;

		pmr->dmov = FALSE;
		MARK(M_DMOV);
		pmr->pp = TRUE;
		if (pmr->jogr)
		    jogv = -jogv;
		INIT_MSG();
		WRITE_MSG(SET_ACCEL, &jacc);
		WRITE_MSG(JOG, &jogv);
		SEND_MSG();
	    }
	    return(OK);
	}
	/* Stop jogging. */
	if (((pmr->mip & MIP_JOG_REQ) == 0) && 
	    ((pmr->mip & MIP_JOGF) || (pmr->mip & MIP_JOGR)))
	{
	    /* Stop motor.  When stopped, process() will correct backlash. */
	    pmr->pp = TRUE;
	    pmr->mip |= MIP_JOG_STOP;
	    pmr->mip &= ~(MIP_JOGF | MIP_JOGR);
	    INIT_MSG();
	    WRITE_MSG(STOP_AXIS, NULL);
	    SEND_MSG();
	    return(OK);
	}
	else if (pmr->mip & (MIP_JOG_STOP | MIP_JOG_BL))
	    return(OK);	/* Normal return if process jog stop or backlash. */

	/*
	 * Tweak motor forward (reverse).  Increment motor's position by a
	 * value stored in pmr->twv.
	 */
	if (pmr->twf || pmr->twr)
	{
	    pmr->val += pmr->twv * (pmr->twf ? 1 : -1);
	    /* Later, we'll act on this. */
	    if (pmr->twf)
		pmr->twf = 0;
	    if (pmr->twr)
		pmr->twr = 0;
	}
	/*
	 * New relative value.  Someone has poked a value into the "move
	 * relative" field (just like the .val field, but relative instead of
	 * absolute.)
	 */
	if (pmr->rlv != pmr->lrlv)
	{
	    pmr->val += pmr->rlv;
	    /* Later, we'll act on this. */
	    pmr->rlv = 0.;
	    MARK(M_RLV);
	    pmr->lrlv = pmr->rlv;
	}
	/* New raw value.  Propagate to .dval and act later. */
	if (pmr->rval != pmr->lrvl)
	    pmr->dval = pmr->rval * pmr->res;	/* Later, we'll act on this. */
    }

    /*** Collect .dval (Dial value) changes from all sources. ***
    * Now we either act directly on the .val change and return, or we
    * propagate it into a .dval change.
    */
    if (pmr->val != pmr->lval)
    {
	MARK(M_VAL);
	if (set && !pmr->foff)
	{
	    /*
	     * Act directly on .val. and return. User wants to redefine .val
	     * without moving the motor and without making a change to .dval.
	     * Adjust the offset and recalc user limits back into agreement
	     * with dial limits.
	     */
	    pmr->off = pmr->val - pmr->dval * dir;
	    pmr->rbv = pmr->drbv * dir + pmr->off;
	    MARK(M_OFF);
	    MARK(M_RBV);

	    set_userlimits(pmr);	/* Translate dial limits to user limits. */

	    pmr->lval = pmr->val;
	    pmr->mip = 0;
	    MARK(M_MIP);
	    pmr->dmov = TRUE;
	    MARK(M_DMOV);
	    return(OK);
	}
	else
	    /*
	     * User wants to move the motor, or to recalibrate both user and
	     * dial.  Propagate .val to .dval.
	     */
	    pmr->dval = (pmr->val - pmr->off) / dir;	/* Later we'll act on this. */	    
    }
    /* Record limit violation */
    pmr->lvio = (pmr->dval > pmr->dhlm) || (pmr->dval > pmr->dhlm + pmr->bdst)
	|| (pmr->dval < pmr->dllm) || (pmr->dval < pmr->dllm + pmr->bdst);
    if (pmr->lvio != old_lvio)
	MARK(M_LVIO);
    if (pmr->lvio)
    {
	pmr->val = pmr->lval;
	MARK(M_VAL);
	pmr->dval = pmr->ldvl;
	MARK(M_DVAL);
	pmr->rval = pmr->lrvl;
	MARK(M_RVAL);
	pmr->dmov = TRUE;
	MARK(M_DMOV);
	return(OK);
    }

    if (pmr->spmg == motorSPMG_Stop || pmr->spmg == motorSPMG_Pause)
	return(OK);
    
    /* IF DVAL field has changed, OR, NOT done moving. */
    if (pmr->dval != pmr->ldvl || !pmr->dmov)
    {
	if (pmr->dval != pmr->ldvl)
	    MARK(M_DVAL);
	pmr->diff = pmr->dval - pmr->drbv;
	MARK(M_DIFF);
	pmr->rdif = NINT(pmr->diff / pmr->mres);
	MARK(M_RDIF);
	if (set)
	{
	    load_pos(pmr);
	    /*
	     * device support will call us back when load is done.
	     */
	    return(OK);
	}
	else
	{
	    /** Calc new raw position, and do a (backlash-corrected?) move. **/
	    double rbvpos = pmr->drbv / pmr->res;	/* where motor is  */
	    double currpos = pmr->ldvl / pmr->res;	/* where we are    */
	    double newpos = pmr->dval / pmr->res;	/* where to go     */
	    double vbase = pmr->vbas / fabs(pmr->res);	/* base speed      */
	    double vel = pmr->velo / fabs(pmr->res);	/* normal speed    */
	    double acc = vel / pmr->accl;	/* normal accel.   */
	    /*
	     * 'bpos' is one backlash distance away from 'newpos'.
	     */
	    double bpos = (pmr->dval - pmr->bdst) / pmr->res;
	    double bvel = pmr->bvel / fabs(pmr->res);	/* backlash speed  */
	    double bacc = bvel / pmr->bacc;	/* backlash accel. */
	    double slop = 0.95 * pmr->rdbd;
	    /*** Use if encoder or ReadbackLink is in use. ***/
	    int use_rel = ((pmr->msta & EA_PRESENT) && pmr->ueip) || pmr->urip;
	    double dMdR = pmr->mres / pmr->res;
	    double relpos = pmr->diff / pmr->res;
	    double relbpos = ((pmr->dval - pmr->bdst) - pmr->drbv) / pmr->res;
	    long rpos, npos;
	    /*
	     * Relative-move target positions with motor-resolution
	     * granularity. The hardware is going to convert encoder steps to
	     * motor steps by truncating any fractional part, instead of
	     * converting to nearest integer, so we prepare for that.
	     */
	    double mRelPos = (NINT(relpos / dMdR) + ((relpos > 0) ? .5 : -.5)) * dMdR;
	    double mRelBPos = (NINT(relbpos / dMdR) + ((relbpos > 0) ? .5 : -.5)) * dMdR;

	    rpos = NINT(rbvpos);
	    npos = NINT(newpos);
	    if (npos == rpos)
	    {
		if (pmr->dmov == FALSE && (pmr->mip == 0 || pmr->mip == MIP_RETRY))
		{
		    pmr->dmov = TRUE;
		    MARK(M_DMOV);
		    if (pmr->mip != 0)
		    {
			pmr->mip = 0;
			MARK(M_MIP);
		    }
		}
		return(OK);
	    }

	    /*
	     * Post new values, recalc .val to reflect the change in .dval. (We
	     * no longer know the origin of the .dval change.  If user changed
	     * .val, we're ok as we are, but if .dval was changed directly, we
	     * must make .val agree.)
	     */
	    pmr->val = pmr->dval * dir + pmr->off;
	    pmr->rval = NINT(pmr->dval / pmr->res);
	    MARK(M_DVAL);
	    MARK(M_VAL);
	    MARK(M_RVAL);
	    /* reset retry counter if this is not a retry */
	    if ((pmr->mip & MIP_RETRY) == 0)
	    {
		pmr->rcnt = 0;
		MARK(M_RCNT);
	    }

	    /*
	     * If we're within retry deadband, move only in preferred dir.
	     */
	    if (use_rel)
	    {
		if ((fabs(pmr->diff) < slop) &&
		    ((mRelPos > 0) != (pmr->bdst > 0)))
		{
		    pmr->dmov = TRUE;
		    MARK(M_DMOV);
		    return(OK);
		}
	    }
	    else
	    {
		if ((fabs(pmr->diff) < slop) &&
		    ((newpos > currpos) != (pmr->bdst > 0)))
		{
		    pmr->dmov = TRUE;
		    MARK(M_DMOV);
		    return(OK);
		}
	    }

	    if (pmr->mip == 0 || pmr->mip == MIP_RETRY)
	    {
		pmr->mip = MIP_MOVE;
		MARK(M_MIP);
		/* v1.96 Don't post dmov if special already did. */
		if (pmr->dmov)
		{
		    pmr->dmov = FALSE;
		    MARK(M_DMOV);
		}
		pmr->ldvl = pmr->dval;
		pmr->lval = pmr->val;
		pmr->lrvl = pmr->rval;
    
		INIT_MSG();
    
		/* Is backlash correction disabled? */
		if (fabs(pmr->bdst) <  fabs(pmr->res))
		{
		    /* Yes, just move to newpos at (vel,acc) */
		    WRITE_MSG(SET_VEL_BASE, &vbase);
		    WRITE_MSG(SET_VELOCITY, &vel);
		    WRITE_MSG(SET_ACCEL, &acc);
		    if (use_rel)
		    {
			mRelPos *= pmr->frac;
			WRITE_MSG(MOVE_REL, &relpos);
		    }
		    else
		    {
			newpos = currpos + pmr->frac * (newpos - currpos);
			WRITE_MSG(MOVE_ABS, &newpos);
		    }
		    WRITE_MSG(GO, NULL);
		}
		/*
		 * If current position is already within backlash range, or if
		 * we already within retry deadband of desired position, then
		 * we don't have to take out backlash.
		 */
		else if ((fabs(pmr->diff) < slop) ||
			 (use_rel && ((relbpos < 0) == (relpos > 0))) ||
			 (!use_rel && (((currpos + slop) > bpos) == (newpos > currpos))))
		{
		    /**
		     * Yes, assume backlash has already been taken out.
		     * Move to newpos at (bvel, bacc).
		     */
		    WRITE_MSG(SET_VEL_BASE, &vbase);
		    WRITE_MSG(SET_VELOCITY, &bvel);
		    WRITE_MSG(SET_ACCEL, &bacc);
		    /********************************************************
		     * Backlash correction imposes a much larger penalty on
		     * overshoot than on undershoot. Here, we allow user to
		     * specify (by .frac) the fraction of the backlash distance
		     * to move as a first approximation. When the motor stops
		     * and we're not yet at 'newpos', the callback will give
		     * us another chance, and we'll go .frac of the remaining
		     * distance, and so on. This algorithm is essential when
		     * the drive creeps after a move (e.g., piezo inchworm),
		     * and helpful when the readback device has a latency
		     * problem (e.g., interpolated encoder), or is a little
		     * nonlinear. (Blatantly nonlinear readback is not handled
		     * by the motor record.)
		     */
		    if (use_rel)
		    {
			mRelPos *= pmr->frac;
			WRITE_MSG(MOVE_REL, &relpos);
		    }
		    else
		    {
			newpos = currpos + pmr->frac * (newpos - currpos);
			WRITE_MSG(MOVE_ABS, &newpos);
		    }
		    WRITE_MSG(GO, NULL);
		}
		else
		{
		    /* We need to take out backlash.  Go to bpos at (vel,acc). */
		    WRITE_MSG(SET_VEL_BASE, &vbase);
		    WRITE_MSG(SET_VELOCITY, &vel);
		    WRITE_MSG(SET_ACCEL, &acc);
		    if (use_rel)
			WRITE_MSG(MOVE_REL, &mRelBPos);
		    else
			WRITE_MSG(MOVE_ABS, &bpos);
		    WRITE_MSG(GO, NULL);
    
		    /* Move to newpos at (bvel, bacc). */
		    WRITE_MSG(SET_VELOCITY, &bvel);
		    WRITE_MSG(SET_ACCEL, &bacc);
		    /* See note regarding backlash and overshoot above. */
		    if (use_rel)
		    {
			mRelPos = (mRelPos - mRelBPos) * pmr->frac;
			WRITE_MSG(MOVE_REL, &mRelPos);
		    }
		    else
		    {
			newpos = bpos + pmr->frac * (newpos - bpos);
			WRITE_MSG(MOVE_ABS, &newpos);
		    }
		    WRITE_MSG(GO, NULL);
		}
		SEND_MSG();
	    }
	}
    }
    return(OK);
}


/******************************************************************************
	special()
*******************************************************************************/
STATIC long special(struct dbAddr * paddr, int after)
{
    motorRecord *pmr = (motorRecord *) paddr->precord;
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    const unsigned short monitor_mask = DBE_VALUE;
    int dir_positive = (pmr->dir == motorDIR_Pos);
    int dir = dir_positive ? 1 : -1;
    BOOLEAN changed = OFF;
    int fieldIndex = dbGetFieldIndex(paddr);
    double offset, tmp_raw, tmp_limit, fabs_urev;
    long rtnval;
    motor_cmnd command;
    double temp_dbl;
    float *temp_flt;


    /*
     * Someone wrote to drive field.  Blink .dmov unless record is disabled.
     */
    if (after == 0)
    {
	switch (fieldIndex)
	{
	    case motorRecordVAL:
	    case motorRecordDVAL:
	    case motorRecordRVAL:
	    case motorRecordRLV:
		if (pmr->disa == pmr->disv || pmr->disp)
		    return(OK);
		pmr->dmov = FALSE;
		db_post_events(pmr, &pmr->dmov, monitor_mask);
		return(OK);

	    case motorRecordHOMF:
	    case motorRecordHOMR:
		if (pmr->mip & MIP_HOME)
		    return(ERROR);	/* Prevent record processing. */
		break;
	}
	return(OK);
    }

    fabs_urev = fabs(pmr->urev);

    switch (fieldIndex)
    {
	/* new vbas: make sbas agree */
    case motorRecordVBAS:
	if (pmr->vbas < 0.0)
	{
	    pmr->vbas = 0.0;	    
	    db_post_events(pmr, &pmr->vbas, monitor_mask);
	}

	if ((pmr->urev != 0.0) && (pmr->sbas != (temp_dbl = pmr->vbas / fabs_urev)))
	{
	    pmr->sbas = temp_dbl;
	    db_post_events(pmr, &pmr->sbas, monitor_mask);
	}
	break;

	/* new sbas: make vbas agree */
    case motorRecordSBAS:
	if (pmr->sbas < 0.0)
	{
	    pmr->sbas = 0.0;
	    db_post_events(pmr, &pmr->sbas, monitor_mask);
	}

	if (pmr->vbas != (temp_dbl = fabs_urev * pmr->sbas))
	{
	    pmr->vbas = temp_dbl;
	    db_post_events(pmr, &pmr->vbas, monitor_mask);
	}
	break;

	/* new vmax: make smax agree */
    case motorRecordVMAX:
	if (pmr->vmax < 0.0)
	{
	    pmr->vmax = 0.0;
	    db_post_events(pmr, &pmr->vmax, monitor_mask);
	}

	if ((pmr->urev != 0.0) && (pmr->smax != (temp_dbl = pmr->vmax / fabs_urev)))
	{
	    pmr->smax = temp_dbl;
	    db_post_events(pmr, &pmr->smax, monitor_mask);
	}
	break;

	/* new smax: make vmax agree */
    case motorRecordSMAX:
	if (pmr->smax < 0.0)
	{
	    pmr->smax = 0.0;
	    db_post_events(pmr, &pmr->smax, monitor_mask);
	}

	if (pmr->vmax != (temp_dbl = fabs_urev * pmr->smax))
	{
	    pmr->vmax = temp_dbl;
	    db_post_events(pmr, &pmr->vmax, monitor_mask);
	}
	break;

	/* new velo: make s agree */
    case motorRecordVELO:
	range_check(pmr, &pmr->velo, pmr->vbas, pmr->vmax);

	if ((pmr->urev != 0.0) && (pmr->s != (temp_dbl = pmr->velo / fabs_urev)))
	{
	    pmr->s = temp_dbl;
	    db_post_events(pmr, &pmr->s, monitor_mask);
	}
	break;

	/* new s: make velo agree */
    case motorRecordS:
	range_check(pmr, &pmr->s, pmr->sbas, pmr->smax);

	if (pmr->velo != (temp_dbl = fabs_urev * pmr->s))
	{
	    pmr->velo = temp_dbl;
	    db_post_events(pmr, &pmr->velo, monitor_mask);
	}
	break;

	/* new bvel: make sbak agree */
    case motorRecordBVEL:
	range_check(pmr, &pmr->bvel, pmr->vbas, pmr->vmax);

	if ((pmr->urev != 0.0) && (pmr->sbak != (temp_dbl = pmr->bvel / fabs_urev)))
	{
	    pmr->sbak = temp_dbl;
	    db_post_events(pmr, &pmr->sbak, monitor_mask);
	}
	break;

	/* new sbak: make bvel agree */
    case motorRecordSBAK:
	range_check(pmr, &pmr->sbak, pmr->sbas, pmr->smax);

	if (pmr->bvel != (temp_dbl = fabs_urev * pmr->sbak))
	{
	    pmr->bvel = temp_dbl;
	    db_post_events(pmr, &pmr->bvel, monitor_mask);
	}
	break;

	/* new accl */
    case motorRecordACCL:
	if (pmr->accl <= 0.0)
	{
	    pmr->accl = 0.1;
	    db_post_events(pmr, &pmr->accl, monitor_mask);
	}
	break;

	/* new bacc */
    case motorRecordBACC:
	if (pmr->bacc <= 0.0)
	{
	    pmr->bacc = 0.1;
	    db_post_events(pmr, &pmr->bacc, monitor_mask);
	}
	break;

	/* new rdbd */
    case motorRecordRDBD:
	enforceMinRetryDeadband(pmr);
	break;

	/* new dir */
    case motorRecordDIR:
	if (pmr->foff)
	{
	    pmr->val = pmr->dval * dir + pmr->off;
	    MARK(M_VAL);
	}
	else
	{
	    pmr->off = pmr->val - pmr->dval * dir;
	    MARK(M_OFF);
	}
	pmr->rbv = pmr->drbv * dir + pmr->off;
	MARK(M_RBV);
	set_userlimits(pmr);	/* Translate dial limits to user limits. */
	break;

	/* new offset */
    case motorRecordOFF:
	pmr->val = pmr->dval * dir + pmr->off;
	pmr->lval = pmr->ldvl * dir + pmr->off;
	pmr->rbv = pmr->drbv * dir + pmr->off;
	MARK(M_VAL);
	MARK(M_RBV);
	set_userlimits(pmr);	/* Translate dial limits to user limits. */
	break;

	/* new user high limit */
    case motorRecordHLM:
	offset = pmr->off;
	if (dir_positive)
	{
	    command = SET_HIGH_LIMIT;
	    tmp_limit = pmr->hlm - offset;
	    MARK(M_DHLM);
	}
	else
	{
	    command = SET_LOW_LIMIT;
	    tmp_limit = -(pmr->hlm) + offset;
	    MARK(M_DLLM);
	}

	tmp_raw = tmp_limit / pmr->res;

	INIT_MSG();
	rtnval = (*pdset->build_trans)(command, &tmp_raw, pmr);
	if (rtnval != OK)
	{
	    /* If an error occured, build_trans() has reset
	     * dial high or low limit to controller's value. */

	    if (dir_positive)
		pmr->hlm = pmr->dhlm + offset;
	    else
		pmr->hlm = -(pmr->dllm) + offset;
	}
	else
	{
	    SEND_MSG();
	    if (dir_positive)
		pmr->dhlm = tmp_limit;
	    else
		pmr->dllm = tmp_limit;
	}
	MARK(M_HLM);
	break;

	/* new user low limit */
    case motorRecordLLM:
	offset = pmr->off;
	if (dir_positive)
	{
	    command = SET_LOW_LIMIT;
	    tmp_limit = pmr->llm - offset;
	    MARK(M_DLLM);
	}
	else
	{
	    command = SET_HIGH_LIMIT;
	    tmp_limit = -(pmr->llm) + offset;
	    MARK(M_DHLM);
	}

	tmp_raw = tmp_limit / pmr->res;

	INIT_MSG();
	rtnval = (*pdset->build_trans)(command, &tmp_raw, pmr);
	if (rtnval != OK)
	{
	    /* If an error occured, build_trans() has reset
	     * dial high or low limit to controller's value. */

	    if (dir_positive)
		pmr->llm = pmr->dllm + offset;
	    else
		pmr->llm = -(pmr->dhlm) + offset;
	}
	else
	{
	    SEND_MSG();
	    if (dir_positive)
		pmr->dllm = tmp_limit;
	    else
		pmr->dhlm = tmp_limit;
	}
	MARK(M_LLM);
	break;

	/* new dial high limit */
    case motorRecordDHLM:
	set_dial_highlimit(pmr, pdset);
	break;

	/* new dial low limit */
    case motorRecordDLLM:
	set_dial_lowlimit(pmr, pdset);
	break;

	/* new frac (move fraction) */
    case motorRecordFRAC:
	/* enforce limit */
	if (pmr->frac < 0.1)
	{
	    pmr->frac = 0.1;
	    changed = ON;
	}
	if (pmr->frac > 1.5)
	{
	    pmr->frac = 1.5;
	    changed = ON;
	}
	if (changed == ON)
	    db_post_events(pmr, &pmr->frac, monitor_mask);
	break;

	/* new mres: make urev agree, and change (velo,bvel,vbas) to leave */
	/* (s,sbak,sbas) constant */
    case motorRecordMRES:
	MARK(M_MRES);		/* MARK it so we'll remember to tell device
				 * support */
	if (pmr->urev != (temp_dbl = pmr->mres * pmr->srev))
	{
	    pmr->urev = temp_dbl;
	    fabs_urev = fabs(pmr->urev);	/* Update local |UREV|. */
	    MARK_AUX(M_UREV);
	}
	goto velcheckB;

	/* new urev: make mres agree, and change (velo,bvel,vbas) to leave */
	/* (s,sbak,sbas) constant */

    case motorRecordUREV:
	if (pmr->mres != (temp_dbl = pmr->urev / pmr->srev))
	{
	    pmr->mres = temp_dbl;
	    MARK(M_MRES);
	}

velcheckB:
	if (pmr->velo != (temp_dbl = fabs_urev * pmr->s))
	{
	    pmr->velo = temp_dbl;
	    MARK_AUX(M_VELO);
	}
	if (pmr->vbas != (temp_dbl = fabs_urev * pmr->sbas))
	{
	    pmr->vbas = temp_dbl;
	    MARK_AUX(M_VBAS);
	}
	if (pmr->bvel != (temp_dbl = fabs_urev * pmr->sbak))
	{
	    pmr->bvel = temp_dbl;
	    MARK_AUX(M_BVEL);
	}
	if (pmr->vmax != (temp_dbl = fabs_urev * pmr->smax))
	{
	    pmr->vmax = temp_dbl;
	    db_post_events(pmr, &pmr->vmax, monitor_mask);
	}
	break;

	/* new srev: make mres agree */
    case motorRecordSREV:
	if (pmr->srev <= 0)
	{
	    pmr->srev = 200;
	    MARK_AUX(M_SREV);
	}
	if (pmr->mres != pmr->urev / pmr->srev)
	{
	    pmr->mres = pmr->urev / pmr->srev;
	    MARK(M_MRES);
	}
	break;

	/* new eres (encoder resolution) */
    case motorRecordERES:
	if (pmr->eres == 0.0)	/* Don't allow ERES = 0. */
    	    pmr->eres = pmr->mres;
	MARK(M_ERES);
	break;

	/* new ueip flag */
    case motorRecordUEIP:
	MARK(M_UEIP);
	/* Ideally, we should be recalculating speeds, but at the moment */
	/* we don't know whether hardware even has an encoder. */
	break;

	/* new urip flag */
    case motorRecordURIP:
	break;

	/* Set to SET mode  */
    case motorRecordSSET:
	pmr->set = 1;
	db_post_events(pmr, &pmr->set, DBE_VALUE);
	break;

	/* Set to USE mode  */
    case motorRecordSUSE:
	pmr->set = 0;
	db_post_events(pmr, &pmr->set, DBE_VALUE);
	break;

	/* Set freeze-offset to freeze mode */
    case motorRecordFOF:
	pmr->foff = 1;
	db_post_events(pmr, &pmr->foff, DBE_VALUE);
	break;

	/* Set freeze-offset to variable mode */
    case motorRecordVOF:
	pmr->foff = 0;
	db_post_events(pmr, &pmr->foff, DBE_VALUE);
	break;

	/*
	 * New readback-delay time time.  Show effect of roundoff to 60-Hz
	 * clock.
	 */
    case motorRecordDLY:
	pmr->dly = (NINT(vxTicksPerSecond * pmr->dly)) / (float) vxTicksPerSecond;
	db_post_events(pmr, &pmr->dly, monitor_mask);
	break;

	/* New backlash distance.  Make sure retry deadband is achievable. */
    case motorRecordBDST:
	enforceMinRetryDeadband(pmr);
	break;

    case motorRecordPCOF:
	temp_flt = &pmr->pcof;
	command = SET_PGAIN;
	goto pidcof;
    case motorRecordICOF:
	temp_flt = &pmr->icof;
	command = SET_IGAIN;
	goto pidcof;
    case motorRecordDCOF:
	temp_flt = &pmr->dcof;
	command = SET_DGAIN;
pidcof:
	if ((pmr->msta & GAIN_SUPPORT) != 0)
	{
	    if (*temp_flt < 0.0)	/* Validity check;  0.0 <= gain <= 1.0 */
	    {
		*temp_flt = 0.0;
		changed = ON;
	    }
	    else if (*temp_flt > 1.0)
	    {
		*temp_flt = 1.0;
		changed = ON;
	    }

	    temp_dbl = *temp_flt;

	    INIT_MSG();
	    rtnval = (*pdset->build_trans)(command, &temp_dbl, pmr);
            /* If an error occured, build_trans() has reset the gain
	     * parameter to a valid value for this controller. */
	    if (rtnval != OK)
		changed = ON;

	    SEND_MSG();
	    if (changed == 1)
		db_post_events(pmr, temp_flt, monitor_mask);
	}
	break;

    case motorRecordCNEN:
	if ((pmr->msta & GAIN_SUPPORT) != 0)
	{
	    double tempdbl;
	    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);

	    INIT_MSG();
	    tempdbl = pmr->cnen;
	    if (pmr->cnen != 0)
		WRITE_MSG(ENABLE_TORQUE, &tempdbl);
	    else
		WRITE_MSG(DISABL_TORQUE, &tempdbl);
	    SEND_MSG();
	}

    case motorRecordJOGF:
	if (pmr->jogf == 0)
	    pmr->mip &= ~MIP_JOG_REQ;
	else if (pmr->mip == 0 && !pmr->hls)
	    pmr->mip |= MIP_JOG_REQ;
	break;

    case motorRecordJOGR:
	if (pmr->jogr == 0)
	    pmr->mip &= ~MIP_JOG_REQ;
	else if (pmr->mip == 0 && !pmr->lls)
	    pmr->mip |= MIP_JOG_REQ;
	break;

    default:
	break;
    }

    switch (fieldIndex)	/* Re-check slew (VBAS) and backlash (VBAS) velocities. */
    {
	case motorRecordVMAX:
	case motorRecordSMAX:
	    if (pmr->vmax != 0.0 && pmr->vmax < pmr->vbas)
	    {
		pmr->vbas = pmr->vmax;
		MARK_AUX(M_VBAS);
		pmr->sbas = pmr->smax;
		MARK_AUX(M_SBAS);
	    }
	    goto velcheckA;

	case motorRecordVBAS:
	case motorRecordSBAS:
	    if (pmr->vmax != 0.0 && pmr->vbas > pmr->vmax)
	    {
		pmr->vmax = pmr->vbas;
		db_post_events(pmr, &pmr->vmax, monitor_mask);
		pmr->smax = pmr->sbas;
		db_post_events(pmr, &pmr->smax, monitor_mask);
	    }
velcheckA:
	    range_check(pmr, &pmr->velo, pmr->vbas, pmr->vmax);
    
	    if ((pmr->urev != 0.0) && (pmr->s != (temp_dbl = pmr->velo / fabs_urev)))
	    {
		pmr->s = temp_dbl;
		db_post_events(pmr, &pmr->s, monitor_mask);
	    }
    
	    range_check(pmr, &pmr->bvel, pmr->vbas, pmr->vmax);
    
	    if ((pmr->urev != 0.0) && (pmr->sbak != (temp_dbl = pmr->bvel / fabs_urev)))
	    {
		pmr->sbak = temp_dbl;
		db_post_events(pmr, &pmr->sbak, monitor_mask);
	    }
    }
    /* Do not process (i.e., clear) marked fields here.  PP fields (e.g., MRES) must remain marked. */
    return(OK);
}


/******************************************************************************
	get_units()
*******************************************************************************/
STATIC long get_units(struct dbAddr * paddr, char *units)
{
    motorRecord *pmr = (motorRecord *) paddr->precord;
    int siz = dbr_units_size - 1;	/* "dbr_units_size" from dbAccess.h */
    char s[30];
    int fieldIndex = dbGetFieldIndex(paddr);

    switch (fieldIndex)
    {

    case motorRecordVELO:
    case motorRecordBVEL:
    case motorRecordVBAS:
	strncpy(s, pmr->egu, DB_UNITS_SIZE);
	strcat(s, "/sec");
	break;

    case motorRecordACCL:
    case motorRecordBACC:
	strcpy(s, "sec");
	break;

    case motorRecordS:
    case motorRecordSBAS:
    case motorRecordSBAK:
	strcpy(s, "rev/sec");
	break;

    case motorRecordSREV:
	strcpy(s, "steps/rev");
	break;

    case motorRecordUREV:
	strncpy(s, pmr->egu, DB_UNITS_SIZE);
	strcat(s, "/rev");
	break;

    default:
	strncpy(s, pmr->egu, DB_UNITS_SIZE);
	break;
    }
    s[siz] = '\0';
    strncpy(units, s, siz + 1);
    return (0);
}

/******************************************************************************
	get_graphic_double()
*******************************************************************************/
STATIC long get_graphic_double(struct dbAddr * paddr, struct dbr_grDouble * pgd)
{
    motorRecord *pmr = (motorRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

    switch (fieldIndex)
    {

    case motorRecordVAL:
    case motorRecordRBV:
	pgd->upper_disp_limit = pmr->hlm;
	pgd->lower_disp_limit = pmr->llm;
	break;

    case motorRecordDVAL:
    case motorRecordDRBV:
	pgd->upper_disp_limit = pmr->dhlm;
	pgd->lower_disp_limit = pmr->dllm;
	break;

    case motorRecordRVAL:
    case motorRecordRRBV:
	if (pmr->res >= 0)
	{
	    pgd->upper_disp_limit = pmr->dhlm / pmr->res;
	    pgd->lower_disp_limit = pmr->dllm / pmr->res;
	}
	else
	{
	    pgd->upper_disp_limit = pmr->dllm / pmr->res;
	    pgd->lower_disp_limit = pmr->dhlm / pmr->res;
	}
	break;

    default:
	recGblGetGraphicDouble(paddr, pgd);
	break;
    }

    return (0);
}

/******************************************************************************
	get_control_double()
*******************************************************************************/
STATIC long
 get_control_double(struct dbAddr * paddr, struct dbr_ctrlDouble * pcd)
{
    motorRecord *pmr = (motorRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

    switch (fieldIndex)
    {

    case motorRecordVAL:
    case motorRecordRBV:
	pcd->upper_ctrl_limit = pmr->hlm;
	pcd->lower_ctrl_limit = pmr->llm;
	break;

    case motorRecordDVAL:
    case motorRecordDRBV:
	pcd->upper_ctrl_limit = pmr->dhlm;
	pcd->lower_ctrl_limit = pmr->dllm;
	break;

    case motorRecordRVAL:
    case motorRecordRRBV:
	if (pmr->res >= 0)
	{
	    pcd->upper_ctrl_limit = pmr->dhlm / pmr->res;
	    pcd->lower_ctrl_limit = pmr->dllm / pmr->res;
	}
	else
	{
	    pcd->upper_ctrl_limit = pmr->dllm / pmr->res;
	    pcd->lower_ctrl_limit = pmr->dhlm / pmr->res;
	}
	break;

    default:
	recGblGetControlDouble(paddr, pcd);
	break;
    }
    return (0);
}

/******************************************************************************
	get_precision()
*******************************************************************************/
STATIC long get_precision(struct dbAddr * paddr, long *precision)
{
    motorRecord *pmr = (motorRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

    *precision = pmr->prec;
    switch (fieldIndex)
    {

    case motorRecordRRBV:
    case motorRecordRMP:
    case motorRecordREP:
	*precision = 0;
	break;

    case motorRecordVERS:
	*precision = 2;
	break;

    default:
	recGblGetPrec(paddr, precision);
	break;
    }
    return (0);
}



/******************************************************************************
	get_alarm_double()
*******************************************************************************/
STATIC long get_alarm_double(struct dbAddr * paddr, struct dbr_alDouble * pad)
{
    motorRecord *pmr = (motorRecord *) paddr->precord;
    int fieldIndex = dbGetFieldIndex(paddr);

    if (fieldIndex == motorRecordVAL || fieldIndex == motorRecordDVAL)
    {
	pad->upper_alarm_limit = pmr->hihi;
	pad->upper_warning_limit = pmr->high;
	pad->lower_warning_limit = pmr->low;
	pad->lower_alarm_limit = pmr->lolo;
    }
    else
    {
	recGblGetAlarmDouble(paddr, pad);
    }
    return (0);
}


/******************************************************************************
	alarm()
*******************************************************************************/
STATIC void alarm(motorRecord * pmr)
{
    if (pmr->udf == TRUE)
    {
	recGblSetSevr(pmr, UDF_ALARM, INVALID_ALARM);
	return;
    }
    /* limit-switch and soft-limit violations */
    if (pmr->hlsv && (pmr->hls || (pmr->dval > pmr->dhlm)))
    {
	recGblSetSevr(pmr, HIGH_ALARM, pmr->hlsv);
	return;
    }
    if (pmr->hlsv && (pmr->lls || (pmr->dval < pmr->dllm)))
    {
	recGblSetSevr(pmr, LOW_ALARM, pmr->hlsv);
	return;
    }
    if ((pmr->msta & CNTRL_COMM_ERR) != 0)
    {
	pmr->msta &= ~CNTRL_COMM_ERR;
	MARK(M_MSTA);
	recGblSetSevr(pmr, COMM_ALARM, INVALID_ALARM);
    }
    return;
}


/******************************************************************************
	monitor()
*******************************************************************************/
STATIC void monitor(motorRecord * pmr)
{
    unsigned short monitor_mask;

    monitor_mask = recGblResetAlarms(pmr);
    monitor_mask |= (DBE_VALUE | DBE_LOG);

    /*
     * Mark .val, .dval changes, and save old values for backlash correction.
     */
    if (pmr->val != pmr->lval)
	MARK(M_VAL);
    if (pmr->dval != pmr->ldvl)
	MARK(M_DVAL);
    if (pmr->rval != pmr->lrvl)
	MARK(M_RVAL);

    /* Catch all previous 'calls' to MARK(). */
    post_MARKed_fields(pmr, monitor_mask);
    return;
}


/******************************************************************************
	post_MARKed_fields()
*******************************************************************************/
STATIC void post_MARKed_fields(motorRecord * pmr, unsigned short mask)
{
    mmap_field mmap_bits;
    nmap_field nmap_bits;
    
    mmap_bits.All = pmr->mmap; /* Initialize for MARKED. */
    nmap_bits.All = pmr->nmap; /* Initialize for MARKED_AUX. */
    
    if (MARKED(M_RBV))
    {
	db_post_events(pmr, &pmr->rbv, mask);
	UNMARK(M_RBV);
    }
    if (MARKED(M_RRBV))
    {
	db_post_events(pmr, &pmr->rrbv, mask);
	UNMARK(M_RRBV);
    }
    if (MARKED(M_DRBV))
    {
	db_post_events(pmr, &pmr->drbv, mask);
	UNMARK(M_DRBV);
    }
    if (MARKED(M_RMP))
    {
	db_post_events(pmr, &pmr->rmp, mask);
	UNMARK(M_RMP);
    }
    if (MARKED(M_REP))
    {
	db_post_events(pmr, &pmr->rep, mask);
	UNMARK(M_REP);
    }
    if (MARKED(M_DIFF))
    {
	db_post_events(pmr, &pmr->diff, mask);
	UNMARK(M_DIFF);
    }
    if (MARKED(M_RDIF))
    {
	db_post_events(pmr, &pmr->rdif, mask);
	UNMARK(M_RDIF);
    }
    if (MARKED(M_MSTA))
    {
	db_post_events(pmr, &pmr->msta, mask);
	UNMARK(M_MSTA);
	if (pmr->msta & GAIN_SUPPORT)
	{
	    unsigned short pos_maint = (pmr->msta & EA_POSITION) ? 1 : 0;
	    if (pos_maint != pmr->cnen)
	    {
		pmr->cnen = pos_maint;
		db_post_events(pmr, &pmr->cnen, mask);
	    }
	}
    }

    if ((pmr->mmap == 0) && (pmr->nmap == 0))
	return;

    /* short circuit: less frequently posted PV's go below this line. */
    mmap_bits.All = pmr->mmap; /* Initialize for MARKED. */
    nmap_bits.All = pmr->nmap; /* Initialize for MARKED_AUX. */

    if (MARKED(M_VAL))
	db_post_events(pmr, &pmr->val, mask);
    if (MARKED(M_DVAL))
	db_post_events(pmr, &pmr->dval, mask);
    if (MARKED(M_RVAL))
	db_post_events(pmr, &pmr->rval, mask);
    if (MARKED(M_TDIR))
	db_post_events(pmr, &pmr->tdir, mask);
    if (MARKED(M_MIP))
	db_post_events(pmr, &pmr->mip, mask);
    if (MARKED(M_HLM))
	db_post_events(pmr, &pmr->hlm, mask);
    if (MARKED(M_LLM))
	db_post_events(pmr, &pmr->llm, mask);
    if (MARKED(M_SPMG))
	db_post_events(pmr, &pmr->spmg, mask);
    if (MARKED(M_RCNT))
	db_post_events(pmr, &pmr->rcnt, mask);
    if (MARKED(M_RLV))
	db_post_events(pmr, &pmr->rlv, mask);
    if (MARKED(M_OFF))
	db_post_events(pmr, &pmr->off, mask);
    if (MARKED(M_DHLM))
	db_post_events(pmr, &pmr->dhlm, mask);
    if (MARKED(M_DLLM))
	db_post_events(pmr, &pmr->dllm, mask);
    if (MARKED(M_HLS))
    {
	db_post_events(pmr, &pmr->hls, mask);
	if ((pmr->dir == motorDIR_Pos) == (pmr->res >= 0))
	    db_post_events(pmr, &pmr->rhls, mask);
	else
	    db_post_events(pmr, &pmr->rlls, mask);
    }
    if (MARKED(M_LLS))
    {
	db_post_events(pmr, &pmr->lls, mask);
	if ((pmr->dir == motorDIR_Pos) == (pmr->res >= 0))
	    db_post_events(pmr, &pmr->rlls, mask);
	else
	    db_post_events(pmr, &pmr->rhls, mask);
    }
    if (MARKED(M_ATHM))
	db_post_events(pmr, &pmr->athm, mask);
    if (MARKED(M_MRES))
	db_post_events(pmr, &pmr->mres, mask);
    if (MARKED(M_ERES))
	db_post_events(pmr, &pmr->eres, mask);
    if (MARKED(M_UEIP))
	db_post_events(pmr, &pmr->ueip, mask);
    if (MARKED(M_URIP))
	db_post_events(pmr, &pmr->urip, mask);
    if (MARKED(M_LVIO))
	db_post_events(pmr, &pmr->lvio, mask);
    if (MARKED(M_RDBD))
	db_post_events(pmr, &pmr->rdbd, mask);

    if (MARKED_AUX(M_S))
	db_post_events(pmr, &pmr->s, mask);
    if (MARKED_AUX(M_SBAS))
	db_post_events(pmr, &pmr->sbas, mask);
    if (MARKED_AUX(M_SBAK))
	db_post_events(pmr, &pmr->sbak, mask);
    if (MARKED_AUX(M_SREV))
	db_post_events(pmr, &pmr->srev, mask);
    if (MARKED_AUX(M_UREV))
	db_post_events(pmr, &pmr->urev, mask);
    if (MARKED_AUX(M_VELO))
	db_post_events(pmr, &pmr->velo, mask);
    if (MARKED_AUX(M_VBAS))
	db_post_events(pmr, &pmr->vbas, mask);
    if (MARKED_AUX(M_BVEL))
	db_post_events(pmr, &pmr->bvel, mask);
    if (MARKED_AUX(M_MISS))
	db_post_events(pmr, &pmr->miss, mask);
    if (MARKED_AUX(M_ACCL))
	db_post_events(pmr, &pmr->accl, mask);
    if (MARKED_AUX(M_BACC))
	db_post_events(pmr, &pmr->bacc, mask);
    if (MARKED(M_MOVN))
	db_post_events(pmr, &pmr->movn, mask);
    if (MARKED(M_DMOV))
	db_post_events(pmr, &pmr->dmov, mask);

    UNMARK_ALL;
}


/******************************************************************************
	process_motor_info()
*******************************************************************************/
STATIC void
 process_motor_info(motorRecord * pmr)
{
    unsigned long status = pmr->msta;
    double old_drbv = pmr->drbv;
    double old_rbv = pmr->rbv;
    long old_rrbv = pmr->rrbv;
    short old_tdir = pmr->tdir;
    short old_movn = pmr->movn;
    short old_hls = pmr->hls;
    short old_lls = pmr->lls;
    short old_athm = pmr->athm;
    int dir = (pmr->dir == motorDIR_Pos) ? 1 : -1;

    /*** Process record fields. ***/

    /* Calculate raw and dial readback values. */
    if ((status & EA_PRESENT) && pmr->ueip)
    {
	/* An encoder is present and the user wants us to use it. */
	pmr->rrbv = pmr->rep;
    }
    else
	pmr->rrbv = pmr->rmp;

    pmr->drbv = pmr->rrbv * pmr->res;
    MARK(M_RMP);
    MARK(M_REP);
    if (pmr->rrbv != old_rrbv)
	MARK(M_RRBV);
    if (pmr->drbv != old_drbv)
	MARK(M_DRBV);

    /* Calculate user readback value. */
    pmr->rbv = dir * pmr->drbv + pmr->off;
    if (pmr->rbv != old_rbv)
	MARK(M_RBV);

    /* Get current or most recent direction. */
    pmr->tdir = (status & RA_DIRECTION) ? 1 : 0;
    if (pmr->tdir != old_tdir)
	MARK(M_TDIR);

    /* Get motor-now-moving indicator. */
    pmr->movn = (status & (RA_DONE | RA_PROBLEM | RA_OVERTRAVEL)) ? 0 : 1;
    if (pmr->movn != old_movn)
	MARK(M_MOVN);

    /* Get states of high, low limit switches. */
    pmr->rhls = (status & RA_OVERTRAVEL) && pmr->tdir;
    pmr->rlls = (status & RA_OVERTRAVEL) && !pmr->tdir;
    pmr->hls = ((pmr->dir == motorDIR_Pos) == (pmr->res >= 0)) ? pmr->rhls : pmr->rlls;
    pmr->lls = ((pmr->dir == motorDIR_Pos) == (pmr->res >= 0)) ? pmr->rlls : pmr->rhls;
    if (pmr->hls != old_hls)
	MARK(M_HLS);
    if (pmr->lls != old_lls)
	MARK(M_LLS);

    /* Get state of motor's or encoder's home switch. */
    if ((status & EA_PRESENT) && pmr->ueip)
	pmr->athm = (status & EA_HOME) ? 1 : 0;
    else
	pmr->athm = (status & RA_HOME) ? 1 : 0;

    if (pmr->athm != old_athm)
	MARK(M_ATHM);


    /*
     * If we've got an external readback device, get Dial readback from it, and
     * propagate to User readback. We do this after motor and encoder readbacks
     * have been read and propagated to .rbv in case .rdbl is a link involving
     * that field.
     */
    if (pmr->urip)
    {
	long status;

	old_drbv = pmr->drbv;
	status = dbGetLink(&(pmr->rdbl), DBR_DOUBLE, &(pmr->drbv), NULL, NULL);
	if (!RTN_SUCCESS(status))
	    pmr->drbv = old_drbv;
	else
	{
	    pmr->drbv *= pmr->rres;
	    pmr->rbv = pmr->drbv * dir + pmr->off;
	    if (pmr->drbv != old_drbv)
	    {
		MARK(M_DRBV);
		MARK(M_RBV);
	    }
	}
    }
    pmr->diff = pmr->dval - pmr->drbv;
    MARK(M_DIFF);
    pmr->rdif = NINT(pmr->diff / pmr->res);
    MARK(M_RDIF);
}

/* Calc and load new raw position into motor w/out moving it. */
STATIC void load_pos(motorRecord * pmr)
{
    struct motor_dset *pdset = (struct motor_dset *) (pmr->dset);
    double newpos = pmr->dval / pmr->res;

    pmr->ldvl = pmr->dval;
    pmr->lval = pmr->val;
    pmr->lrvl = pmr->rval = (long) newpos;

    if (pmr->foff)
    {
	/* Translate dial value to user value. */
	if (pmr->dir == motorDIR_Pos)
	    pmr->val = pmr->off + pmr->dval;
	else
	    pmr->val = pmr->off - pmr->dval;
	MARK(M_VAL);
    }
    else
    {
	/* Translate dial limits to user limits. */
	if (pmr->dir == motorDIR_Pos)
	    pmr->off = pmr->val - pmr->dval;
	else
	    pmr->off = pmr->val + pmr->dval;
	MARK(M_OFF);
	set_userlimits(pmr);	/* Translate dial limits to user limits. */
    }
    pmr->mip = MIP_LOAD_P;
    MARK(M_MIP);
    pmr->pp = TRUE;
    pmr->dmov = FALSE;
    MARK(M_DMOV);

    /* Load pos. into motor controller.  Get new readback vals. */
    INIT_MSG();
    WRITE_MSG(LOAD_POS, &newpos);
    SEND_MSG();
    INIT_MSG();
    WRITE_MSG(GET_INFO, NULL);
    SEND_MSG();
}

/*
 * FUNCTION... static void check_speed_and_resolution(motorRecord *)
 *
 * INPUT ARGUMENTS...
 *	1 - motor record pointer
 *
 * RETRUN ARGUMENTS... None.
 *
 * LOGIC...
 *
 *  IF SREV negative.
 *	Set SREV <- 200.
 *  ENDIF
 *  IF UREV nonzero.
 *	Set MRES < - |UREV| / SREV.
 *  ENDIF
 *  IF MRES zero.
 *	Set MRES <- 1.0
 *  ENDIF
 *  IF UREV does not match MRES.
 *	Set UREV <- MRES * SREV.
 *  ENDIF
 *
 *  IF SMAX > 0.
 *	Set VMAX <- SMAX * |UREV|.
 *  ELSE IF VMAX > 0.
 *	Set SMAX <- VMAX / |UREV|.
 *  ELSE
 *	Set both SMAX and VMAX to zero.
 *  ENDIF
 *
 *  IF SBAS is nonzero.
 *	Range check; 0 < SBAS < SMAX.
 *	Set VBAS <- SBAS * |UREV|.
 *  ELSE
 *	Range check; 0 < VBAS < VMAX.
 *	Set SBAS <- VBAS / |UREV|.
 *  ENDIF
 *
 *  IF S is nonzero.
 *	Range check; SBAS < S < SMAX.
 *	VELO <- S * |UREV|.
 *  ELSE
 *	Range check; VBAS < VELO < VMAX.
 *	S < - VELO / |UREV|.
 *  ENDIF
 *
 *  IF SBAK is nonzero.
 *	Range check; SBAS < SBAK < SMAX.
 *	BVEL <- SBAK * |UREV|.
 *  ELSE
 *	Range check; VBAS < BVEL < VMAX.
 *	SBAK <- BVEL / |UREV|.
 *  ENDIF
 *
 *  IF ACCL or BACC is zero.
 *	Set ACCL/BACC to 0.1
 *  ENDIF
 *
 *  NORMAL RETURN.
 */

STATIC void check_speed_and_resolution(motorRecord * pmr)
{
    const unsigned short monitor_mask = DBE_VALUE;
    double fabs_urev = fabs(pmr->urev);

    /*
     * Reconcile two different ways of specifying speed, resolution, and make
     * sure things are sane.
     */

    /* SREV (steps/revolution) must be sane. */
    if (pmr->srev <= 0)
    {
	pmr->srev = 200;
	MARK_AUX(M_SREV);
    }

    /* UREV (EGU/revolution) <--> MRES (EGU/step) */
    if (pmr->urev != 0.0)
    {
	pmr->mres = pmr->urev / pmr->srev;
	MARK(M_MRES);
    }
    if (pmr->mres == 0.0)
    {
	pmr->mres = 1.0;
	MARK(M_MRES);
    }
    if (pmr->urev != pmr->mres * pmr->srev)
    {
	pmr->urev = pmr->mres * pmr->srev;
        fabs_urev = fabs(pmr->urev);	/* Update local |UREV|. */
	MARK_AUX(M_UREV);
    }

    /* SMAX (revolutions/sec) <--> VMAX (EGU/sec) */
    if (pmr->smax > 0.0)
	pmr->vmax = pmr->smax * fabs_urev;
    else if (pmr->vmax > 0.0)
	pmr->smax = pmr->vmax / fabs_urev;
    else
	pmr->smax = pmr->vmax = 0.0;
    db_post_events(pmr, &pmr->vmax, monitor_mask);
    db_post_events(pmr, &pmr->smax, monitor_mask);

    /* SBAS (revolutions/sec) <--> VBAS (EGU/sec) */
    if (pmr->sbas != 0.0)
    {
	range_check(pmr, &pmr->sbas, 0.0, pmr->smax);
	pmr->vbas = pmr->sbas * fabs_urev;
    }
    else
    {
	range_check(pmr, &pmr->vbas, 0.0, pmr->vmax);
	pmr->sbas = pmr->vbas / fabs_urev;
    }
    db_post_events(pmr, &pmr->vbas, monitor_mask);
    db_post_events(pmr, &pmr->sbas, monitor_mask);

    
    /* S (revolutions/sec) <--> VELO (EGU/sec) */
    if (pmr->s != 0.0)
    {
	range_check(pmr, &pmr->s, pmr->sbas, pmr->smax);
	pmr->velo = pmr->s * fabs_urev;
    }
    else
    {
	range_check(pmr, &pmr->velo, pmr->vbas, pmr->vmax);
	pmr->s = pmr->velo / fabs_urev;
    }
    db_post_events(pmr, &pmr->velo, monitor_mask);
    db_post_events(pmr, &pmr->s, monitor_mask);

    /* SBAK (revolutions/sec) <--> BVEL (EGU/sec) */
    if (pmr->sbak != 0.0)
    {
	range_check(pmr, &pmr->sbak, pmr->sbas, pmr->smax);
	pmr->bvel = pmr->sbak * fabs_urev;
    }
    else
    {
	range_check(pmr, &pmr->bvel, pmr->vbas, pmr->vmax);
	pmr->sbak = pmr->bvel / fabs_urev;
    }
    db_post_events(pmr, &pmr->sbak, monitor_mask);
    db_post_events(pmr, &pmr->bvel, monitor_mask);

    /* Sanity check on acceleration time. */
    if (pmr->accl == 0.0)
    {
	pmr->accl = 0.1;
	MARK_AUX(M_ACCL);
    }
    if (pmr->bacc == 0.0)
    {
	pmr->bacc = 0.1;
	MARK_AUX(M_BACC);
    }
}

/*
FUNCTION... void set_dial_highlimit(motorRecord *)
USAGE... Set dial-coordinate high limit.
NOTES... This function sends a command to the device to set the raw dial high
limit.  This is done so that a device level function may do an error check on
the validity of the limit.  This is to support those devices (e.g., MM4000)
that have their own, read-only, travel limits.
*/
STATIC void set_dial_highlimit(motorRecord *pmr, struct motor_dset *pdset)
{
    int dir_positive = (pmr->dir == motorDIR_Pos);
    double offset, tmp_raw;
    long rtnval;

    tmp_raw = pmr->dhlm / pmr->res;
    INIT_MSG();
    rtnval = (*pdset->build_trans)(SET_HIGH_LIMIT, &tmp_raw, pmr);
    offset = pmr->off;
    if (rtnval == OK)
	SEND_MSG();

    if (dir_positive)
    {
	pmr->hlm = pmr->dhlm + offset;
	MARK(M_HLM);
    }
    else
    {
	pmr->llm = -(pmr->dhlm) + offset;
	MARK(M_LLM);
    }
    MARK(M_DHLM);
}

/*
FUNCTION... void set_dial_lowlimit(motorRecord *)
USAGE... Set dial-coordinate low limit.
NOTES... This function sends a command to the device to set the raw dial low
limit.  This is done so that a device level function may do an error check on
the validity of the limit.  This is to support those devices (e.g., MM4000)
that have their own, read-only, travel limits.
*/
STATIC void set_dial_lowlimit(motorRecord *pmr, struct motor_dset *pdset)
{
    int dir_positive = (pmr->dir == motorDIR_Pos);
    double offset, tmp_raw;
    long rtnval;

    tmp_raw = pmr->dllm / pmr->res;

    INIT_MSG();
    rtnval = (*pdset->build_trans)(SET_LOW_LIMIT, &tmp_raw, pmr);
    offset = pmr->off;
    if (rtnval == OK)
	SEND_MSG();

    if (dir_positive)
    {
	pmr->llm = pmr->dllm + offset;
	MARK(M_LLM);
    }
    else
    {
	pmr->hlm = -(pmr->dllm) + offset;
	MARK(M_HLM);
    }
    MARK(M_DLLM);
}

/*
FUNCTION... void set_userlimits(motorRecord *)
USAGE... Translate dial-coordinate limits to user-coordinate limits.
*/
STATIC void set_userlimits(motorRecord *pmr)
{
    if (pmr->dir == motorDIR_Pos)
    {
	pmr->hlm = pmr->dhlm + pmr->off;
	pmr->llm = pmr->dllm + pmr->off;
    }
    else
    {
	pmr->hlm = -(pmr->dllm) + pmr->off;
	pmr->llm = -(pmr->dhlm) + pmr->off;
    }
    MARK(M_HLM);
    MARK(M_LLM);
}

/*
FUNCTION... void range_check(motorRecord *, float *, double, double)
USAGE... Limit parameter to valid range; i.e., min. <= parameter <= max.

INPUT...	parm - pointer to parameter to be range check.
		min  - minimum value.
		max  - 0 = max. range check disabled; !0 = maximum value.
*/
STATIC void range_check(motorRecord *pmr, float *parm_ptr, double min, double max)
{
    const unsigned short monitor_mask = DBE_VALUE;
    BOOLEAN changed = OFF;
    double parm_val = *parm_ptr;

    if (parm_val < min)
    {
	parm_val = min;
	changed = ON;
    }
    if (max != 0.0 && parm_val > max)
    {
	parm_val = max;
	changed = ON;
    }

    if (changed == ON)
    {
	*parm_ptr = parm_val;
	db_post_events(pmr, parm_ptr, monitor_mask);
    }
}

