/*
FILENAME...	drvMVP2001.h
USAGE... This file contains driver "include" information that is specific to
	 the MicroMo MVP 2001 B02 (Linear, RS-485).

Version:	$Revision: 1.3 $
Modified By:	$Author: sluiter $
Last Modified:	$Date: 2004-03-16 15:14:55 $
*/

/*
 *      Original Author: Kevin Peterson
 *      Date: 08/27/2002
 *
 *  Illinois Open Source License
 *  University of Illinois
 *  Open Source License
 *
 *
 *  Copyright (c) 2004,  UNICAT.  All rights reserved.
 *
 *
 *  Developed by:
 *
 *  UNICAT, Advanced Photon Source, Argonne National Laboratory
 *
 *  Frederick Seitz Materials Research Laboratory,
 *  University of Illinois at Urbana-Champaign
 *
 *  http://www.uni.aps.anl.gov
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the
 *  "Software"), to deal with the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *
 *  Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimers.
 *
 *
 *  Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimers in the
 *  documentation and/or other materials provided with the distribution.
 *
 *
 *  Neither the names of UNICAT, Frederick Seitz Materials Research 
 *  Laboratory, University of Illinois at Urbana-Champaign,
 *  nor the names of its contributors may be used to endorse or promote
 *  products derived from this Software without specific prior written
 *  permission.
 *
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 *  ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.
 *
 *
 * Modification Log:
 * -----------------
 * .01	08/27/2002  kmp  copied from drvIM483.h (rev 1.1, mod .01) and
 *			 customized for the MVP2001.
 *
 */

#ifndef	INCdrvMVP2001h
#define	INCdrvMVP2001h 1

#include "motor.h"
#include "motordrvCom.h"
#include "serialIO.h"

#define SERIAL_TIMEOUT	2000 /* Command timeout in msec */


/* MVP2001 (chain) specific data is stored in this structure. */
struct MVPcontroller
{
    int port_type;			/* GPIB_PORT or RS232_PORT 		*/
    serialIO *serialInfo;  		/* For RS-232 				*/
    int serial_card;            	/* Card on which Hideos is running 	*/
    char serial_task[20];		/* Hideos task name for serial port 	*/
    int maxCurrent[MAX_AXIS];	   	/* Maximum current (in mA) 		*/
    int encoderCpr[MAX_AXIS];	   	/* Encoder counts per revolution 	*/
    CommStatus status;			/* Controller communication status. 	*/
};

/* Motor status response for MVP2001. */
typedef union
{
    epicsUInt16 All;
    struct
    {
      bool minusLS		:1;	/*  negative limit switch hit 		*/  
      bool extEvent2		:1;	/*  external event #2 			*/
      bool plusLS		:1;	/*  positive limit switch hit		*/
      bool extEvent1		:1;	/*  external event #1 			*/
      bool emergencyStop	:1;	/*  emergency stop flag is active 	*/
      bool localMode		:1;	/*  1:local mode - 0:remote mode 	*/
      bool softLimit		:1;	/*  soft-limit has been reached 	*/
      bool NOT_power		:1;	/*  motor power 0 - ON; 1 - OFF. 	*/
      bool offTraj		:1;	/*  off traj (more than FD com. value) 	*/
      bool devicenetErr	:1;	/*  error in devicenet message packets 	*/
      bool devicenet		:1;	/*  devicenet connection active 	*/
      bool trajComplete	:1;	/*  1=traj (set by T com.) is complete 	*/ 
      bool aiState		:1;	/*  ANM mode not used--always 1 	*/
      bool operatingMode	:1;	/*  1:Velocity - 0:Position 		*/
      bool inPosition	:1;	/*  in-position indicator 		*/
      bool inMotion		:1;	/*  in-motion indicator 		*/
    } Bits;
} MOTOR_STATUS;

/* Function prototypes. */
extern RTN_STATUS MVP2001Setup(int, int, int);
extern RTN_STATUS MVP2001Config(int, int, int, const char *);

#endif	/* INCdrvMVP2001h */

