/*
FILENAME...     drvOms58.h
USAGE...        OMS driver level "include" information that is specific to OMS
                model VME58.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Jim Kowalkowski
 *      Current Author: Joe Sullivan
 *      Date: 11/14/94
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
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  01-18-93 jbk initialized
 * .02  11-14-94 jps copy drvOms.c and modify to point to vme58 driver
 *      ...
 * .06  12-07-94 jps first released version w/interrupt supprt
 * .07  12-20-94 jps rearrange the device init routines
 * .08  05-03-96 jps convert 32bit card accesses to 16bit - vme58PCB
 *                   version D does not support 32bit accesses.
 * .09  05-09-97 jps increase maximum card count to 15
 * .10  08-22-01 rls "int" type specifications for all bit-fields.
 * .11  03-21-05 rls replace VxWorks specific "uint16_t" with "epicsUInt16".
 * .12  01-26-11 rls added reboot flag in DPRAM R/W reserved area.
 *  
 */

#ifndef INCdrvOms58h
#define INCdrvOms58h 1

#include "drvOmsCom.h"

/*
 * VME58 default profile
 */

#define OMS_NUM_CARDS           15
#define OMS_NUM_CHANNELS        8
#define OMS_NUM_ADDRS           0x4000
#define OMS_BRD_SIZE            0x1000	/* card address boundary */

#define BUFFER_SIZE             256

/* Board control register structures */


/* VME58 DUAL-PORT MEMORY MAP */
typedef struct
{
    epicsUInt16 encPos[2];
    epicsUInt16 cmndPos[2];
    epicsUInt16 cmndVel[2];
    epicsUInt16 accel[2];
    epicsUInt16 maxVel[2];
    epicsUInt16 baseVel[2];
    epicsUInt16 dFltrGain[2];
    epicsUInt16 dFltrPole[2];
    epicsUInt16 dFltrZero[2];
    epicsUInt16 reserved[46];
} MOTOR_DATA_REGS;

/* Definitions for VME58 I/O Registers */

/* Control Register - Offset = 0x0FE1 */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int intReqEna  :1; /* Master interrupt request enable */
        unsigned int ioIntEna   :1; /* I/O bits 0 and 1 interrupt enable */
        unsigned int directInEna:1; /* Interrupt request to the VME58 ? */
        unsigned int doneIntEna :1; /* Done detect interrupt enable */
        unsigned int otIntEna   :1; /* Overtravel detect interrupt enable */
        unsigned int slipIntEna :1; /* Encoder slip detect interrupt enable */
        unsigned int            :1; /* Unused  */
        unsigned int update     :1; /* Data area update request */
    } Bits;
} CNTRL_REG;

/* Status Register - Offset = 0x0FE3 */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int interrupt  :1; /* Interrupt dectect */
        unsigned int directIn   :1; /* Direct input interrupt detect */
        unsigned int directOut  :1; /* Direct ouput interrupt detect */
        unsigned int done       :1; /* Motion done detect */
        unsigned int overtravel :1; /* Overtravel detect */
        unsigned int encoderSlip:1; /* Encoder slip detect */
        unsigned int cardOK     :1; /* Powerup initilization complete */
        unsigned int cmndError  :1; /* Command error dectect */
    } Bits;
} STATUS_REG;

/* I/O Register(0-7) -  Offset = 0x0FE5 */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int io_7 :1; /* Bit 7 */
        unsigned int io_6 :1; /* Bit 6 */
        unsigned int io_5 :1; /* Bit 5 */
        unsigned int io_4 :1; /* Bit 4 */
        unsigned int io_3 :1; /* Bit 3 */
        unsigned int io_2 :1; /* Bit 2 */
        unsigned int io_1 :1; /* Bit 1 */
        unsigned int io_0 :1; /* Bit 0 */
    } Bits;
} IO_LOW_REG;

/* Slip Flag Register - Offset = 0x0FE7 */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int slip_s :1; /* status of S axis */
        unsigned int slip_r :1; /* status of R axis */
        unsigned int slip_v :1; /* status of V axis */
        unsigned int slip_u :1; /* status of U axis */
        unsigned int slip_t :1; /* status of T axis */
        unsigned int slip_z :1; /* status of Z axis */
        unsigned int slip_y :1; /* status of Y axis */
        unsigned int slip_x :1; /* status of X axis */
    } Bits;
} SLIP_REG;

/* Done Flag Register - Offset = 0x0FE9 */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int done_s :1; /* status of S axis */
        unsigned int done_r :1; /* status of R axis */
        unsigned int done_v :1; /* status of V axis */
        unsigned int done_u :1; /* status of U axis */
        unsigned int done_t :1; /* status of T axis */
        unsigned int done_z :1; /* status of Z axis */
        unsigned int done_y :1; /* status of Y axis */
        unsigned int done_x :1; /* status of X axis */
    } Bits;
} DONE_REG;

/* I/O High Register(8-13) - Offset = 0x0FEB */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int       :1; /* Unused */
        unsigned int       :1; /* Unused */
        unsigned int io_13 :1; /* Bit 13 */
        unsigned int io_12 :1; /* Bit 12 */
        unsigned int io_11 :1; /* Bit 11 */
        unsigned int io_10 :1; /* Bit 10 */
        unsigned int io_9  :1; /* Bit 9  */
        unsigned int io_8  :1; /* Bit 8  */
    } Bits;
} IO_HIGH_REG;


/* Limit Switch Status Register - Offset = 0x0FED */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int limit_s :1; /* status of S axis */
        unsigned int limit_r :1; /* status of R axis */
        unsigned int limit_v :1; /* status of V axis */
        unsigned int limit_u :1; /* status of U axis */
        unsigned int limit_t :1; /* status of T axis */
        unsigned int limit_z :1; /* status of Z axis */
        unsigned int limit_y :1; /* status of Y axis */
        unsigned int limit_x :1; /* status of X axis */
    } Bits;
} LIMIT_REG;

/* Home Switch Status Register - Offset = 0x0FEF */
typedef union
{
    epicsUInt8 All;
    struct
    {
        unsigned int home_s :1; /* status of S axis */
        unsigned int home_r :1; /* status of R axis */
        unsigned int home_v :1; /* status of V axis */
        unsigned int home_u :1; /* status of U axis */
        unsigned int home_t :1; /* status of T axis */
        unsigned int home_z :1; /* status of Z axis */
        unsigned int home_y :1; /* status of Y axis */
        unsigned int home_x :1; /* status of X axis */
    } Bits;
} HOME_REG;

typedef struct
{
    epicsUInt8 unused00;
    epicsUInt8 cntrlReg;    /* Control Register - Read/Write */
    epicsUInt8 unused02;
    epicsUInt8 statusReg;   /* Status Register  - Read */
    epicsUInt8 unused04;
    epicsUInt8 ioLowReg;    /* IO bits 0-7 status register   - Read */
    epicsUInt8 unused06;
    epicsUInt8 slipReg;     /* Encoder slip status register - Read */
    epicsUInt8 unused08;
    epicsUInt8 doneReg;     /* Axis done status register - Read */
    epicsUInt8 unused0A;
    epicsUInt8 ioHighReg;   /* IO bits 8-13 status register - Read */
    epicsUInt8 unused0C;
    epicsUInt8 limitReg;    /* Limit switch  status register - Read */
    epicsUInt8 unused0E;
    epicsUInt8 homeReg;     /* Home switch  status register - Read */
    epicsUInt8 unusedF0;
    epicsUInt8 intVector;   /* Interrupt vector */
} MOTOR_CNTRL_REGS;


/* OMS VME dual port memory map */
struct vmex_motor
{                                               /* Offset */
    epicsInt16 inPutIndex;                      /* 0x0000 */
    epicsInt16 outGetIndex;                     /* 0x0002 */
    epicsInt16 inBuffer[BUFFER_SIZE];           /* 0x0004 */
    epicsInt16 reserved0[254];                  /* 0x0204 */
    MOTOR_DATA_REGS data[OMS_NUM_CHANNELS];     /* 0x0400 */
    epicsInt16 outPutIndex;                     /* 0x0800 */
    epicsInt16 inGetIndex;                      /* 0x0802 */
    epicsInt16 outBuffer[BUFFER_SIZE];          /* 0x0804 */
    epicsInt16 rebootind;                       /* 0x0A04 - used by driver to
                                                 * test for board reboot. */
    epicsInt16 reserved1[749];                  /* 0x0A06 */
    MOTOR_CNTRL_REGS control;                   /* 0x0FE0 */
};

#endif	/* INCdrvOms58h */
