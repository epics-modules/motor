/*
FILENAME...	serialIO.h
USAGE...	.

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2003-05-07 13:42:47 $
*/
 
/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

/*
 *      Original Author: Mark Rivers
 *
 * Modification Log:
 * -----------------
 */


#ifdef __cplusplus
extern "C"
{
#endif
struct serialInfo *serialIOInit(int card, char *task);
int serialIOSend(struct serialInfo *serialInfo, char const *buffer,
		 int buffer_len, int timeout);
int serialIORecv(struct serialInfo *serialInfo, char *buffer, int buffer_len,
                    char *terminator, int timeout);
#ifdef __cplusplus
}
#endif
