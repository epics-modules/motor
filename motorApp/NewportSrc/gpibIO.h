#include <rngLib.h>
#include <dbCommon.h>
#include <drvSup.h>
#include <drvGpibInterface.h>

struct gpibInfo
{
    struct dpvtGpibHead head;
    int address;
    SEM_ID semID;
};
struct gpibInfo *gpibIOInit(int link, int address);
int gpibIOSend(struct gpibInfo *info, char const *buffer, int buff_len, int timeout);
int gpibIORecv(struct gpibInfo *info, char *buffer, int buff_len, 
	       int terminator, int timeout);
