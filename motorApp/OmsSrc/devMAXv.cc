/*
FILENAME...	devMAXV.cc
USAGE... Device level support for OMS MAXv model.

Version:	$Revision: 1.2 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-12-20 21:11:53 $
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 04/05/04
 *      Current Author: Ron Sluiter
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *	      The Controls and Automation Group (AT-8)
 *	      Ground Test Accelerator
 *	      Accelerator Technology Division
 *	      Los Alamos National Laboratory
 *
 *      Co-developed with
 *	      The Controls and Computing Group
 *	      Accelerator Systems Division
 *	      Advanced Photon Source
 *	      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  04-05-05 rls Copied from devOms58.cc
 */

#include "motorRecord.h"
#include "devOmsCom.h"
#include "epicsExport.h"

extern int MAXv_num_cards;
extern struct driver_table MAXv_access;

/* ----------------Create the dsets for devMAXv----------------- */
static long MAXv_init(void *);
static long MAXv_init_record(void *);
static long MAXv_start_trans(struct motorRecord *);
static RTN_STATUS MAXv_end_trans(struct motorRecord *);

struct motor_dset devMAXv =
{
    {8, NULL, MAXv_init, MAXv_init_record, NULL},
    motor_update_values,
    MAXv_start_trans,
    oms_build_trans,
    MAXv_end_trans
};

extern "C" {epicsExportAddress(dset,devMAXv);}

static struct board_stat **MAXv_cards;
static const char errmsg[] = {"\n\n!!!ERROR!!! - Oms MAXv driver uninitialized.\n"};

static long MAXv_init(void *arg)
{
    int after = (int) arg;

    if (*(MAXv_access.init_indicator) == NO)
    {
	errlogSevPrintf(errlogMinor, "%s", errmsg);
    	return(ERROR);
    }
    else
	return(motor_init_com(after, MAXv_num_cards, &MAXv_access, &MAXv_cards));
}

static long MAXv_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, MAXv_num_cards, &MAXv_access, MAXv_cards));
}

static long MAXv_start_trans(struct motorRecord *mr)
{
    struct motor_trans *trans;
    long rtnval;

    rtnval = motor_start_trans_com(mr, MAXv_cards);
    /* Initialize a STOP_AXIS command termination string pointer. */
    trans = (struct motor_trans *) mr->dpvt;
    trans->motor_call.termstring = " ID";
    return(rtnval);
}

static RTN_STATUS MAXv_end_trans(struct motorRecord *mr)
{
    if (*(MAXv_access.init_indicator) == NO)
    {
	errlogSevPrintf(errlogMinor, "%s", errmsg);
	return(ERROR);
    }
    else
	return(motor_end_trans_com(mr, &MAXv_access));
}

