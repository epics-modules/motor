/*
FILENAME...	drvOms.h
USAGE... This file contains OMS driver "include" information that is
		specific to OMS models VME8 and VME44.

Version:	$Revision: 1.5 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2008-03-14 20:36:10 $
*/

/*
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
 * .00 10-23-03 rls - VX2 spurious interrupt fix; enable all interrupts, including
 *		      transmit buffer empty.
 * .01 10-28-03 rls - moved OMS specific "irqdatastr" from motordrvCom.h to here.
 */

#ifndef	INCdrvOmsh
#define	INCdrvOmsh 1

#include "drvOmsCom.h"
#include "epicsRingBytes.h"

/*
 * VME8/44 default profile
 */

#define OMS_NUM_CARDS		8
#define OMS_NUM_CHANNELS	8
#define OMS_NUM_ADDRS		0xFC00
#define OMS_BRD_SIZE		0x10	/* card address boundary */
#define OMS_RESP_Q_SZ		0x100	/* maximum oms response message size */

/* status register */
#define STAT_IRQ		0x80
#define STAT_TRANS_BUF_EMPTY	0x40
#define STAT_INPUT_BUF_FULL	0x20
#define STAT_DONE		0x10
#define STAT_OVERTRAVEL		0x08
#define STAT_ENCODER_REQ	0x04
#define STAT_UNUSED		0x02
#define STAT_ERROR		0x01

/* done flag register */
#define DONE_X	0x01
#define DONE_Y	0x02
#define DONE_Z	0x04
#define DONE_T	0x08
#define DONE_U	0x10
#define DONE_V	0x20
#define DONE_R	0x40
#define DONE_S	0x80

/* interrupt control register */
#define IRQ_ENABLE	0x80
#define IRQ_TRANS_BUF	0x40
#define IRQ_INPUT_BUF	0x20
#define IRQ_DONE	0x10

#define IRQ_ENABLE_ALL	(IRQ_ENABLE|IRQ_TRANS_BUF|IRQ_INPUT_BUF|IRQ_DONE)

struct vmex_motor
{
    epicsUInt8 unused0;
    epicsUInt8 data;
    epicsUInt8 unused1;
    epicsUInt8 done;
    epicsUInt8 unused2;
    epicsUInt8 control;
    epicsUInt8 unused3;
    epicsUInt8 status;
    epicsUInt8 unused4;
    epicsUInt8 vector;
    epicsUInt8 unused5[6];
};

struct irqdatastr	/* Used only for VME44. */
{
    /* Interrupt Handling control elements */
    int irqErrno;	/* Error indicator from isr */
    epicsUInt8 irqEnable;
    epicsRingBytesId recv_rng;	/* message receiving control */
    epicsEvent *recv_sem;
    epicsRingBytesId send_rng;	/* message transmitting control */
};

#endif	/* INCdrvOmsh */
