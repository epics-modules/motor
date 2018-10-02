#ifndef INCmotorepicsinc
#define INCmotorepicsinc 1

#include "epicsVersion.h"

#ifndef VERSION_INT
#  define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#endif

#define VERSION_INT_3_16 VERSION_INT(3,16,0,0)
#if EPICS_VERSION_INT < VERSION_INT_3_16
#define RECSUPFUN_CAST (RECSUPFUN)
#else
#define RECSUPFUN_CAST
#define REC_TYPE motorRecord
#define USE_TYPED_RSET
#endif

#include <dbDefs.h>
#include <callback.h>
#include <dbScan.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <errlog.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsEvent.h>
#include <cantProceed.h> /* !! for callocMustSucceed() */
#include <dbEvent.h>
#include    <devSup.h>

#endif
