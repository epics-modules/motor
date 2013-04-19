/* aSub support for concatenating strings into a result longer than a
 * DBF_STRING.  Used by EnsemblePSOFly.db
 */
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include <dbDefs.h>
#include <dbCommon.h>
#include <recSup.h>
#include <aSubRecord.h>

static long concatString(aSubRecord *pasub) {
	/* printf("concatString: entry\n"); */
	if ((strlen(pasub->a) <= 50) && strlen(pasub->b)) {
		strcpy(pasub->vala, pasub->a);
		strcat(pasub->vala, pasub->b);
		/* printf("concatString: vala='%s'\n", (char *)pasub->vala); */
	}
	return(0);
}

#include <registryFunction.h>
#include <epicsExport.h>

static registryFunctionRef concatStringRef[] = {
	{"concatString", (REGISTRYFUNCTION)concatString},
};

static void concatStringRegister(void) {
	registryFunctionRefAdd(concatStringRef, NELEMENTS(concatStringRef));
}

epicsExportRegistrar(concatStringRegister);
