/*
 * These routines provide a simple interface to Hideos serial I/O.  They hide 
 * the details of Hideos. They are intended for use by applications which do
 * simple synchronous reads and writes, and no control operations like setting
 * baud rates, etc.
 */

#include "serialIO.h"

#ifdef	DEBUG
#define Debug(l, f, args...) {if (l <= serialIODebug) printf(f, ## args);}
#else
#define Debug(l, f, args...)
#endif

int serialIODebug = 0;


struct serialInfo *cc_serialIOInit(int card, char *taskName)
{
    struct serialInfo *serialInfo;
    int status;

    serialInfo = (struct serialInfo *) malloc(sizeof(struct serialInfo));
    serialInfo->bpd = new BPD;
    status = serialInfo->bpd->Bind(serialInfo->td, card, taskName);
    if (status < 0) {
        Debug(1, 
            "serialIOInit: Cannot bind to remote Hideos task %s\n", taskName);
        return (NULL);
    }
    else
        Debug(1, 
            "serialIOInit: Bound to remote Hideos task %s\n", taskName);
        return (serialInfo);
}

int cc_serialIOSend(struct serialInfo *serialInfo, char const *buffer, 
                    int buffer_len, int timeout)
{
    int status;
    IndirectStringMsg* sm = (IndirectStringMsg*)
        hideos_resources->GetMessagePool()->GetMessage(IndirectStringMsgType);
    Message* rm;

      
    if (sm->IsQueued()) {
       printf("serialIOSend: 1st message %lx already queued!!\n", (long)sm);
       exit(1);
    }

    sm->SetWrite(timeout);
    sm->SetBuffers((unsigned char *)buffer,buffer_len,NULL,0);

    status = serialInfo->bpd->Send(serialInfo->td, sm);
    if (status != 0) {
        Debug(1, "serialIOSend: error sending message %s\n", buffer);
	hideos_resources->GetMessagePool()->FreeMessage(sm);
        goto done;
    }
    Debug(2, "serialIOSend: sent message %s\n", buffer);
    status = serialInfo->bpd->Receive((Message *&)rm);
    if (status != 0) {
        Debug(1, "serialIOSend: error receiving message, status=%d\n", status);
        goto done;
    }
    Debug(2, "serialIOSend: received message, status=%d\n", status);

    hideos_resources->GetMessagePool()->FreeMessage(rm);

done:
    return (status);
}


int cc_serialIORecv(struct serialInfo *serialInfo, char *buffer, 
                    int buffer_len, int terminator, int timeout)
{
    int status, nrec = 0;
    IndirectStringMsg* sm = (IndirectStringMsg*)
        hideos_resources->GetMessagePool()->GetMessage(IndirectStringMsgType);
    IndirectStringMsg* rm;

    if (sm->IsQueued()) {
       printf("serialIORecv: message %lx already queued!!\n", (long)sm);
       exit(1);
    }

    sm->SetRead(timeout);
    if (terminator == -1)
        sm->SetComplexDelimiter((char *)NULL, (char *)NULL, 0);
    else
        sm->SetSimpleDelimiter(terminator);

    sm->SetBuffers(NULL, 0, (unsigned char *)buffer, buffer_len);

    status = serialInfo->bpd->Send(serialInfo->td, sm);
    if (status != 0) {
        Debug(1, "serialIORecv: error sending message, status = %d\n", status);
	hideos_resources->GetMessagePool()->FreeMessage(sm);
        goto doneRecv;
    }
    Debug(2, "serialIORecv: sent message status = %d\n", status);
    status = serialInfo->bpd->Receive((Message *&)rm);
    if (status < 0) {
        Debug(1,"serialIORecv: error receiving message, status = %d\n",
                                status);
        goto doneRecv;
    }

    nrec = sm->GetDestSize();
    if (sm->return_code != 0) {
        Debug(1,"serialIORecv: error, return code = %ld\n", sm->return_code);
    }
    Debug(2,"serialIORecv: Received %d bytes\n", nrec);
    /* Append a NULL byte to the response if there is room */
    if (nrec < buffer_len) buffer[nrec] = '\0';

    Debug(2,"serialIORecv: Received message = %s\n", buffer);

    hideos_resources->GetMessagePool()->FreeMessage(rm);

doneRecv:
    return (nrec);
}


extern "C"
{
struct serialInfo *serialIOInit(int card, char *taskName)
{
    return (cc_serialIOInit(card, taskName));
}

int serialIOSend(struct serialInfo *serialInfo, char const *buffer,
		 int buffer_len, int timeout)
{
    return (cc_serialIOSend(serialInfo, buffer, buffer_len, timeout));
}

int serialIORecv(struct serialInfo *serialInfo, char *buffer, int buffer_len,
                    int terminator, int timeout)
{
    return (cc_serialIORecv(serialInfo, buffer, buffer_len, 
                    terminator, timeout));
}

}
