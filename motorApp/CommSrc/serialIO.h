#ifdef __cplusplus

#include "vxWorks.h"
#include "vme.h"
#include "iv.h"
#include "stdio.h"
#include "string.h"
#include "cacheLib.h"
#include "taskLib.h"

#include "gen/all_msg_ids.h"
#include "msg/serial_config_msg.h"
#include "msg/string_msg.h"
#include "hideos/globals.h"
#include "hideos/resources.h"
#include "hideos/msgpool.h"
#include "hideos/registry.h"
#include "hideos/drvBp.h"


struct serialInfo
{
    BPD *bpd;
    TD  td;
};

/* Function prototypes */
struct serialInfo *cc_serialIOInit(int card, char *task);
int cc_serialIOSend(struct serialInfo *serialInfo, char const *buffer,
                    int buffer_len, int timeout);

#else  /* For C just define serialInfo as a dummy structure since it can't
          understand the include files which define what it really is */
struct serialInfo
{
    int dummy;
};
#endif

#ifdef __cplusplus
extern "C"
{
#endif
struct serialInfo *serialIOInit(int card, char *task);
int serialIOSend(struct serialInfo *serialInfo, char const *buffer,
		 int buffer_len, int timeout);
int serialIORecv(struct serialInfo *serialInfo, char *buffer, int buffer_len,
                    int terminator, int timeout);
#ifdef __cplusplus
}
#endif
