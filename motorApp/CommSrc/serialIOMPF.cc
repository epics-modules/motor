/*
 * These routines provide a simple interface to MPF serial I/O.  They hide 
 * the details of MPF They are intended for use by applications which do
 * simple synchronous reads and writes, and no control operations like setting
 * baud rates, etc.
 *
 * Author: Mark Rivers
 * Date:   4/24/98
 * Modifications:
 *         9/30/99   Converted to MPF
 */

#include <vxWorks.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <msgQLib.h>
#include <tickLib.h>
#include <taskLib.h>
#include <time.h>

#include "epicsPrint.h"

#include "Message.h"
#include "ConnectMessage.h"
#include "Char8ArrayMessage.h"
#include "serialServer.h"

// Minimum number of ticks to wait for server reply
#define MIN_MSGQ_WAIT (1 * CLOCKS_PER_SEC)

class serialIO
{
public:
        serialIO(int card, char *serverName, int *createdOK);
        int serialIOSend(char const *buffer, int buffer_len, int timeout);
        int serialIORecv(char *buffer, int buffer_len, int terminator, 
                                int timeout);
        static void serialIOCallback(Message *message, void *pointer);
private:
        MessageClient* pMessageClient;
        MSG_Q_ID msgQId;
};

#ifndef	NODEBUG
#define DEBUG(l, f, args...) {if (l <= serialIODebug) printf(f, ## args);}
#else
#define DEBUG(l, f, args...)
#endif

volatile int serialIODebug = 0;


serialIO::serialIO(int card, char *serverName, int *createdOK)
{
    int status;
    Message *pmess;

    *createdOK = 1;
    // Create a message queue for the callback
    msgQId = msgQCreate(4, sizeof(Message *), MSG_Q_FIFO);
    DEBUG(5, "serialIOInit: message queue created, ID=%p\n", msgQId);
    pMessageClient = new MessageClient(serialIOCallback,(void *)this);
    DEBUG(5, "serialIOInit: message client created=%p\n", pMessageClient);
    status = pMessageClient->bind(serverName, card);
    if (status) {
        epicsPrintf("serialIOInit: Cannot bind to MPF server %s\n", serverName);
        *createdOK = 0;
    }
    else
        DEBUG(1, "serialIOInit: Bound to MPF server %s\n", serverName);
    // Wait for connect message to be received, 2 second timeout
    status = msgQReceive(msgQId, (char *)&pmess, sizeof(pmess), 
                                          2*CLOCKS_PER_SEC);
    if (status == ERROR) {
        epicsPrintf("serialIO: error calling msgQReceive, status = %d\n",
                                status);
        *createdOK = 0;
    }
    if (pmess->getType() != messageTypeConnect) {
        epicsPrintf("serialIO: incorrect message type received = %d\n",
                                pmess->getType());
        *createdOK = 0;
    }
}

int serialIO::serialIOSend(char const *buffer, int buffer_len, int timeout)
{
    int status;
    Char8ArrayMessage *psm = new Char8ArrayMessage;
    Char8ArrayMessage *prm = NULL;
    Message *pmess;
    int wait;

    psm->allocValue(buffer_len);
    psm->setSize(buffer_len);
    memcpy(psm->value, buffer, buffer_len);
    psm->cmd = cmdWrite;
    psm->timeout = timeout/1000;
    status = pMessageClient->send(psm);
    if (status) {
        DEBUG(1, "serialIOSend: error sending message %s\n", buffer);
        goto finish;
    }
    DEBUG(2, "serialIOSend: sent message %s\n", buffer);

    // Wait for response back from server
    wait = 2*timeout/1000*CLOCKS_PER_SEC;
    if (wait < MIN_MSGQ_WAIT) wait = MIN_MSGQ_WAIT;
    status = msgQReceive(msgQId, (char *)&pmess, sizeof(pmess), wait);
    if (status == ERROR) {
        epicsPrintf("serialIOSend: error calling msgQReceive=%d\n", status);
        goto finish;
    }
    DEBUG(5, "serialIOSend:  got message, pmess=%p\n", pmess);
    if (pmess->getType() == messageTypeChar8Array) {
        prm = (Char8ArrayMessage *)pmess;
        status = prm->status;
        if (status) DEBUG(1, "serialIOSend: error receiving message, status=%d\n", 
                                                status);
        DEBUG(4, "serialIOSend: received message, status=%d\n", status);
    } else {
        epicsPrintf("serialIOInit: incorrect message type received = %d\n",
                                pmess->getType());
    }

    delete prm;

finish:
    return (status);
}


int serialIO::serialIORecv(char *buffer, int buffer_len, int terminator, 
                           int timeout)
{
    int status, nrec = 0;
    Char8ArrayMessage *psm = new Char8ArrayMessage;
    Char8ArrayMessage *prm = NULL;
    Message *pmess;
    int wait;

    psm->timeout = timeout/1000;
    // MPF uses seconds, not milliseconds for timeout.  If the desired timeout
    // is non-zero then use a minimum 1 second timeout
    if ((timeout > 0) && (psm->timeout < 1)) psm->timeout = 1;
    psm->cmd = cmdRead|cmdSetEom;
    psm->eomString[0] = terminator;
    if (terminator == -1)
        psm->eomLen = 0;
    else {
        psm->eomLen = 1;
        psm->eomString[0] = terminator;
    }

    
    status = pMessageClient->send(psm);
    if (status != 0) {
        epicsPrintf("serialIORecv: error sending message, status = %d\n", 
                        status);
        goto done;
    }
    DEBUG(2, "serialIORecv: sent message status = %d, timeout=%d\n", 
                                             status, timeout);
    wait = 2*timeout/1000*CLOCKS_PER_SEC;
    if (wait < MIN_MSGQ_WAIT) wait = MIN_MSGQ_WAIT;
    status = msgQReceive(msgQId, (char *)&pmess, sizeof(pmess), wait);
    if (status == ERROR) {
        epicsPrintf("serialIORecv: error calling msgQReceive, status = %d\n",
                                status);
        goto done;
    }

    if (pmess->getType() != messageTypeChar8Array) {
        epicsPrintf("serialIOInit: incorrect message type received = %d\n",
                                pmess->getType());
        goto cleanup;
    }
    prm = (Char8ArrayMessage *)pmess;
    if (prm->status != 0) {
        DEBUG(1,"serialIORecv: error, return status = %d\n", prm->status);
        goto cleanup;
    }
    nrec = prm->getSize();
    if (nrec > buffer_len) nrec=buffer_len;
    memcpy(buffer, prm->value, nrec);
    DEBUG(2,"serialIORecv: Received %d bytes\n", nrec);
    // Append a NULL byte to the response if there is room
    if (nrec < buffer_len) buffer[nrec] = '\0';

    DEBUG(2,"serialIORecv: Received %d bytes, message = \n%s\n", nrec, buffer);
cleanup:
    delete prm;

done:
    return (nrec);
}

void serialIO::serialIOCallback(Message *message, void *pointer)
{
    serialIO *psi = (serialIO *)pointer;
    int status;

    // If this is a Connect message or a Char8ArrayMessage then send it to
    // the message queue.
    DEBUG(5, "serialIOCallback: message type=%d\n", message->getType());
    if(message->getType()==messageTypeConnect) {
        ConnectMessage *pcm = (ConnectMessage *)message;
        if (pcm->status != connectYes) {
            DEBUG(1, "serialIOCallback: disconnect message?, status=%d\n", 
                                                pcm->status);
            delete message;
            return;
        }
    }
    else if(message->getType()!=messageTypeChar8Array) {
        epicsPrintf("serialIOCallback: not Connect message or Char8ArrayMessage\n");
        delete message;
        return;
    }
    status = msgQSend(psi->msgQId, (char *)&message, sizeof(message), 
                                       NO_WAIT, MSG_PRI_NORMAL);
    if (status == ERROR) {
        epicsPrintf("serialIOCallback: error from msgQSend, status = %d\n",
                                                    status);
    }
}

extern "C"
{
void* serialIOInit(int card, char *serverName)
{
    int createdOK=0;
    serialIO *psi = new serialIO(card, serverName, &createdOK);
    if (createdOK) {
        return( (void *)(psi) );
    } else {
        delete psi;
        return(NULL);
    }
}

int serialIOSend(serialIO *psi, char const *buffer,
		 int buffer_len, int timeout)
{
    return (psi->serialIOSend(buffer, buffer_len, timeout));
}

int serialIORecv(serialIO *psi, char *buffer, int buffer_len,
                    int terminator, int timeout)
{
    return (psi->serialIORecv(buffer, buffer_len, terminator, timeout));
}

}
