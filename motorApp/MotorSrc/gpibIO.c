/* gpibIO.c - Routines for asynchronous EPICS GPIB I/O 
 *
 *      Author:  Mark Rivers
 *      Date:    October 30, 1997
 *
 * Modification Log:
 * -----------------
 * .01  30-Oct-1997  mlr  Initial release
 * 
 * These routines provide a simple interface to GPIB I/O.  They hide the
 * details of the driver.  They are intended for use by applications which do
 * simple synchronous reads and writes, and no control operations like Serial
 * Poll.
 */

#include <stdlib.h>
#include <stdio.h>
#include <vxWorks.h>
#include <gpibIO.h>

#define GPIB_SEND 0
#define GPIB_RECEIVE 1

#ifdef NODEBUG
#define Debug(l,f,v) ;
#else
#define Debug(l,f,v) { if(l<=gpibIODebug) printf(f,v); }
#endif

volatile int gpibIODebug = 0;

extern struct drvGpibSet drvGpib;       /* entry points to driver functions */

struct gpibMessage {
    struct dpvtGpibHead head;
    int function;
    int address;
    int timeout;
    int status;
    int buff_len;
    int terminator;
    SEM_ID semID;
    char *buffer;
};

static int gpibIOCallback(struct gpibMessage *gpibMessage);

struct gpibInfo *gpibIOInit(int link, int address)
{
    struct gpibInfo *gpibInfo;
    gpibInfo = (struct gpibInfo *) malloc(sizeof(struct gpibInfo));
    gpibInfo->head.workStart = (int (*)()) gpibIOCallback;
    gpibInfo->head.link = link;
    gpibInfo->head.device = address;
    gpibInfo->address = address;
    gpibInfo->semID = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);

    (*(drvGpib.ioctl))(GPIB_IO, link, NULL, IBGENLINK, 0, NULL);
    (*(drvGpib.ioctl))(GPIB_IO, link, NULL, IBGETLINK, 0, &gpibInfo->head.pibLink);
    Debug(1, "gpibIOInit, address = %d\n", address);
    return(gpibInfo);
}


int gpibIOSend(struct gpibInfo *info, char const *buffer, int buff_len, int timeout)
{
    struct gpibMessage *gpibMessage;
    int status;

    gpibMessage = (struct gpibMessage *) malloc(sizeof(struct gpibMessage));
    gpibMessage->address = info->address;
    gpibMessage->buffer = (char *) buffer;
    gpibMessage->buff_len = buff_len;
    gpibMessage->timeout = timeout;
    gpibMessage->function = GPIB_SEND;
    gpibMessage->head = info->head;
    gpibMessage->semID = info->semID;
    Debug(2, "gpibIOSend, calling driver, buffer = %s\n", buffer);
    (*(drvGpib.qGpibReq))(gpibMessage, IB_Q_LOW);
    semTake(gpibMessage->semID, WAIT_FOREVER);
    status = gpibMessage->status;
    free(gpibMessage);
    return (status);
}


int gpibIORecv(struct gpibInfo *info, char *buffer, int buff_len, 
                               int terminator, int timeout)
{
    struct gpibMessage *gpibMessage;
    int status;

    gpibMessage = (struct gpibMessage *) malloc(sizeof(struct gpibMessage));
    gpibMessage->address = info->address;
    gpibMessage->buffer = buffer;
    gpibMessage->buff_len = buff_len;
    gpibMessage->terminator = terminator;
    gpibMessage->timeout = timeout;
    gpibMessage->function = GPIB_RECEIVE;
    gpibMessage->head = info->head;
    gpibMessage->semID = info->semID;
    Debug(2, "gpibIORecv, calling driver, address = %d\n", info->address);
    (*(drvGpib.qGpibReq))(gpibMessage, IB_Q_LOW);
    semTake(gpibMessage->semID, WAIT_FOREVER);
    status = gpibMessage->status;
    free(gpibMessage);
    return (status);
}


static int gpibIOCallback(struct gpibMessage *gpibMessage)
{
    int timeout;
    int nrequest, nactual;

    timeout = gpibMessage->timeout * sysClkRateGet() / 1000;
    if (timeout < 1) timeout = 1;

    switch(gpibMessage->function)
    {
    case GPIB_SEND:
        Debug(2, "gpibIOCallback, GPIB_SEND,  message=%s\n", 
                                gpibMessage->buffer);
        /* write the message to the GPIB listen address */
        nrequest = gpibMessage->buff_len;
        nactual =(*(drvGpib.writeIb))(gpibMessage->head.pibLink, 
                    gpibMessage->address, gpibMessage->buffer, 
                    nrequest, timeout);
        if (nactual != nrequest) {
            /* Something is wrong if we couldn't write everything */
            Debug(1, "Error writing GPIB, requested length=%d\n", nrequest);
            Debug(1, "                       actual length=%d\n", nactual);
        }
        gpibMessage->status = nactual;
        semGive(gpibMessage->semID);  /* gpibIOSend is waiting for this */
        break;
    case GPIB_RECEIVE:
        /* Read a message from the GPIB listen address */
        nrequest = gpibMessage->buff_len;
        nactual = (*(drvGpib.readIbEos))(gpibMessage->head.pibLink, 
                    gpibMessage->address, gpibMessage->buffer, 
                    nrequest, timeout, gpibMessage->terminator);
        /* Append a null terminator if there is room */
        if (nactual < nrequest) gpibMessage->buffer[nactual]='\0';
        Debug(2, "gpibIOCallback, GPIB_RECEIVE,  message=%s\n", 
                                gpibMessage->buffer);
        Debug(2, "gpibIOCallback, GPIB_RECEIVE,  nrequest=%d\n", nrequest);
        Debug(2, "gpibIOCallback, GPIB_RECEIVE,  nactual=%d\n", nactual);
        gpibMessage->status = nactual;
        semGive(gpibMessage->semID);  /* gpibIORecv is waiting for this */
        break;
    }
    return (0);
}
