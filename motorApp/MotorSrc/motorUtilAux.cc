/*
FILENAME...     motorUtilAux.cc
USAGE...        Motor Record Utility Support.

Version:        $Revision: 1.2 $
Modified By:    $Author: sluiter $
Last Modified:  $Date: 2006-03-06 19:20:55 $
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
*/

#include <string.h>

#include <cantProceed.h>
#include <dbStaticLib.h>
#include <errlog.h>

/* ----- Function Declarations ----- */
char **getMotorList();
/* ----- --------------------- ----- */

/* ----- External Declarations ----- */
extern struct dbBase *pdbbase;
extern int numMotors;
/* ----- --------------------- ----- */

char ** getMotorList()
{
    DBENTRY dbentry, *pdbentry = &dbentry;
    long    status;
    char    **paprecords = 0, temp[29];
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
            length = sprintf(temp, "%s", dbGetRecordName(pdbentry));
            paprecords[index] = (char *) callocMustSucceed(length,
                                           sizeof(char), "getMotorList(2nd)");
            strcpy(paprecords[index], temp); 
            status = dbNextRecord(pdbentry);
            index++;
        }
        numMotors = index;
    }

    dbFinishEntry(pdbentry);
    return(paprecords);
}

