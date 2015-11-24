/*
FILENAME...     motorUtilAux.cc
USAGE...        Motor Record Utility Support.

*/

/*
*  Original Author: Kevin Peterson (kmpeters@anl.gov)
*  Date: December 11, 2002
*
*  UNICAT
*  Advanced Photon Source
*  Argonne National Laboratory
*
*  Current Author: Ron Sluiter
*
* Notes:
* - A Channel Access (CA) oriented file had to be created separate from the
* primary motorUtil.cc file because CA and record access code cannot both
* reside in the same file; each defines (redefines) the DBR's.
*
* Modification Log:
* -----------------
* .01 03-06-06 rls Fixed wrong call; replaced dbGetNRecordTypes() with
*                  dbGetNRecords().
* .02 03-20-07 tmm sprintf() does not include terminating null in num of chars
*                  converted, so getMotorList was not allocating space for it.
* .03 09-09-08 rls Visual C++ link errors on improper pdbbase declaration.
*/

#include <string.h>

#include <cantProceed.h>
#include <dbStaticLib.h>
#include <errlog.h>

#include "dbAccessDefs.h"


/* ----- Function Declarations ----- */
char **getMotorList();
/* ----- --------------------- ----- */

extern int numMotors;
/* ----- --------------------- ----- */

char ** getMotorList()
{
    DBENTRY dbentry, *pdbentry = &dbentry;
    long    status, a_status;
    char    **paprecords = 0, temp[PVNAME_STRINGSZ];
    int     num_entries = 0, length = 0, index = 0;

    dbInitEntry(pdbbase,pdbentry);
    status = dbFindRecordType(pdbentry,"motor");
    if (status)
        errlogPrintf("getMotorList(): No record description\n");

    while (!status)
    {
        num_entries = dbGetNRecords(pdbentry);
        paprecords = (char **) callocMustSucceed(num_entries, sizeof(char *),
                                                 "getMotorList(1st)");
        status = dbFirstRecord(pdbentry);
        while (!status)
        {
            a_status = dbIsAlias(pdbentry);
	    if (a_status == 0)
	    {
                length = sprintf(temp, "%s", dbGetRecordName(pdbentry));
                paprecords[index] = (char *) callocMustSucceed(length+1,
                                           sizeof(char), "getMotorList(2nd)");
                strcpy(paprecords[index], temp);
                index++;
	    }
            status = dbNextRecord(pdbentry);
        }
        numMotors = index;
    }

    dbFinishEntry(pdbentry);
    return(paprecords);
}

