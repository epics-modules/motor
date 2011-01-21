#ifndef DRV_HY8601_H
#define DRV_HY8601_H

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_NAME	"drvHy8601Asyn"

/* CSR Register bit definitions */
#define CSR_HOMESTOP	0x8000
#define CSR_INTEN	0x4000
#define CSR_DONE	0x2000
#define CSR_CRASHSTOP	0x1000
#define CSR_DIRECTION	0x0800
#define CSR_AUX2	0x0400
#define CSR_AUX1	0x0200
#define CSR_ENCODUSE	0x0100
#define CSR_ENCODDET	0x0080
#define CSR_JOG		0x0040
#define CSR_GO		0x0020
#define CSR_DRVSTAT	0x0010
#define CSR_HOMELMT	0x0008
#define CSR_MAXLMT	0x0004
#define CSR_MINLMT	0x0002
#define CSR_RESET	0x0001

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
#define CSR_CLR(base,bit) do { *(volatile epicsUInt16 *)((base)+REG_CSR) &= ~(bit);} while(0) 

int drvHy8601AsynCreate( char *port, int addr, int card, int nAxes );
int Hy8601Configure(int cardnum,
		    int ip_carrier,
		    int ipslot,
		    int vector,
		    int useencoder,
		    int limitmode);

#ifdef __cplusplus
}
#endif
#endif
