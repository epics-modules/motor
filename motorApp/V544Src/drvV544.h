/* File: drvV544.h 		   	*/
/* Version: 1.2		   	*/
/* Date Last Modified: 8/6/96	*/


/* Device Support Definitions for Highland motor controller */
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
 * .01  01-18-93	jbk     initialized
 * .02  11-14-94	jps     copy drvOms.h and modify to point to vme58 driver
 * .03  ??-??-95	jps     copy drvOms58.h and modify to point to Highland driver
 *      ...
 */




/*
 * valid command types for the driver, the order is importance, 0 is of
 * lowest importance.  Device support will choose the greatest one to use as
 * the driver transaction type.
 */

#define UNDEFINED (unsigned char)0	/* garbage type */
#define IMMEDIATE (unsigned char)1	/* 'i' an execute immediate, no reply */
#define MOVE_TERM (unsigned char)2	/* 't' terminate a previous active
					 * motion */
#define MOTION    (unsigned char)3	/* 'm' will produce motion updates */
#define VELOCITY  (unsigned char)4	/* 'v' make motion updates till
					 * MOVE_TERM */
#define INFO      (unsigned char)5	/* 'f' get curr motor/encoder pos and
					 * stat */
#define QUERY     (unsigned char)6	/* 'q' specialty type, not needed */

/* Set list size to accomidate the 2 move  commands 
 * when all parameters are being changed */
#define MAX_LIST_SIZE 10

typedef struct CmndDef
{
  int32_t pos;       
  uint16_t vel;
  uint16_t accel;       
  uint16_t standby;     /* Drive current standby percent */
  uint16_t cmnd;        /* Motion command (ABS,INC, HOME,...) */
} CMND_DEF;

/* message queue management - device and driver support only */
struct mess_node
{
    CALLBACK callback;
    int signal;
    int card;
    unsigned char type;
    int     cmndCnt;      /* Number of commands in list */
    int     cmndIndx;     /* Current command index */
    CMND_DEF cmndList[MAX_LIST_SIZE];
    long position;
    long encoder_position;
    long velocity;
    unsigned long status;
    struct dbCommon *precord;
    struct mess_node *next;
};
typedef struct mess_node MOTOR_CALL;
typedef struct mess_node MOTOR_RETURN;

typedef struct mess_card_query
{
    char *card_name;
    int total_axis;
} MOTOR_CARD_QUERY;

typedef struct mess_axis_query
{
    long position;
    long encoder_position;
    unsigned long status;
} MOTOR_AXIS_QUERY;

struct v544_support
{
    int (*send) (MOTOR_CALL *);
    int (*free) (struct mess_node *);
    int (*get_card_info) (int, MOTOR_CARD_QUERY *);
    int (*get_axis_info) (int, int signal, MOTOR_AXIS_QUERY *);
};

/* Board control register structures */


/*
 * V544 default profile 
 */

#define V544_NUM_CARDS           8
#define V544_NUM_CHANNELS        4
#define V544_INTERRUPT_TYPE      intVME
#define V544_ADDRS_TYPE          atVMEA16
#define V544_NUM_ADDRS           0xDD00
#define V544_INT_LEVEL           5      /* default interrupt level (1-6) */
#define V544_BRD_SIZE            0x80  /* card address boundary */

/* Board type info */
#define V544_LABEL            "V544"
#define V544_VXID             0xFEEE   /* Manufacturer registered VXI ID */

#define BRD_OFFSET            0x80
#define VXI_BASE              0xC000
#define VXI_VECTOR(v)         ((v-VXI_BASE)/0x40)

/* V544 Parameter Scaling Constants */
/* (assuming half step scaling by the record */
#define STEP_SCALE  256     /* Position scaling per step  */
#define VEL_SCALE   16      /* Velocity scaling per step/sec  */
#define ACC_SCALE   256     /* Acceleration scaling per step/sec/sec */

/* V544 Parameter Limits    */
#define MAX_VEL    4096 
#define MAX_ACCEL  65535
#define MAX_POS    (2**31)

/* V544 Control Register Definitions */

/* VCSR register status bits (board level status) */
#define VCSR_RES    0x1       /* Soft Reset */
#define VCSR_SFT    0x4       /* Self-test passed */
#define VCSR_RDY    0x8       /* Ready to operate */
#define VCSR_MOD    0x4000    /* Module ID 'off' */

/* VCSR register commands (board level commands) */
#define VCSR_FRZ     0x8000   /* Board level freeze on motor positions updates */

/* OPTS (options) register status masks */
#define OPTS_ENC     0x1      /* Encoder option enabled */
#define OPTS_ACT     0x2      /* Encoder FPGA installed */
#define OPTS_SER     0x4      /* Serial port enabled */

/* Per-axis Command/Status register definitions */
/* status bits */
#define CS_BUSY_TO  2     /* Maximum command ACK lantency in polling cycles */
#define CS_ABORT_TO  1900   /* Dwell loop count for send_cmnd() check of abort bug */
#define CS_ERROR   0x8000   /* Command error flag */
#define CS_DONE    0x80     /* Command completed flag */
#define CS_BUSY    0x40     /* Command accepted   */

/* CMD codes (bits 0-4)*/
#define CS_IDLE          0x0    /* Soft-reset - update status/position regs */
#define CS_RUN_CW        0x1    /* Constant velocity - CW  */
#define CS_RUN_CCW       0x2    /* Constant velocity - CCW */
#define CS_MOVE_ABS      0x3    /* Move to absolute motor position */
#define CS_MOVE_INC      0x4    /* Move to relative motor position */
#define CS_SEEK_INDX     0x5    /* Stop ABS move at index switch */
#define CS_BACKOFF_CW    0x6    /* Stop CW ABS move when off limit switch */
#define CS_BACKOFF_CCW   0x7    /* Stop CCW ABS move when off limit switch */
#define CS_SET_ZERO      0x8    /* Set absolute zero to motor position */
#define CS_SET_POSITION  0x9    /* Set logical position to motor position */
#define CS_READ_THETA    0xA    /* Read motor quadrant/microstep position */

/* Operations */
#define CS_GO            0x20   /* Initiate command execution */

/* Interrupt control */
#define CS_IEN_ON           0x4000 /* Bit 14 - Enable interrupt on "DONE" */ 
#define CS_IEN_OFF          0x0 

/*                Motor PARKing currents (bits 9,10) */
#define CS_PARK_100      0x000  /* Part motor at 100% current */
#define CS_PARK_75       0x200  /*                75%         */
#define CS_PARK_50       0x400  /*                50%         */
#define CS_PARK_0        0x600  /*                 0%         */


/* V544 STATus Register Bit Mapping */
#define STAT_BADC               0x8000     /* Illegal command detected */
#define STAT_SPRA               0x20       /* Spare limit switch status */
#define STAT_MOTR               0x8        /* Motor driver error */
#define STAT_CCWL               0x4        /* CCW limit switch status */
#define STAT_CWL                0x2        /* CW limit switch status */
#define STAT_INDX               0x1        /* INDEX switch status */




/* V544 DUAL-PORT MEMORY MAP */
/* Define byte offsets into axis control registers */
#define COARSE_POS_ID   0x0
#define FINE_POS_ID     0x2
#define VELOCITY_ID     0x4
#define ACCEL_ID        0x6
#define LS_DECEL_ID     0x8
#define AUX_REG_ID      0xa
#define STAT_REG_ID     0xc
#define CMND_REG_ID     0xe

/*
 * union posReg{       
 *      int32_t all;
 *      struct {
 *	int16_t high;
 *      int16_t low;
 *      } split;
 *    };
*/

typedef struct pos_reg_str {
      int16_t coarse;
      uint16_t fine;
   } POS_REG;

/* Define register structure */
typedef struct motor_reg_str {
      POS_REG pos;
      uint16_t velocity;
      uint16_t accel;
      uint16_t lsDecel;
      uint16_t aux;
      uint16_t status;
      uint16_t cmnd;
   } MOTOR_REG;


struct vmex_motor {
   uint16_t vxid;
   uint16_t vtype;
   uint16_t vcsr;
   uint16_t pgid;
   uint16_t prev;
   uint16_t vsn;  
   uint16_t opts;
   uint16_t scan;
  
   /* Programming/Status Registers */
   MOTOR_REG motorReg[V544_NUM_CHANNELS];

   /* Readback Registers  */
   POS_REG posReg[V544_NUM_CHANNELS];     /* Actual positions */
   uint16_t velReg[V544_NUM_CHANNELS];    /* Actual velocity  */
};

