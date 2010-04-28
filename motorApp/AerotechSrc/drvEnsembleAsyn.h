/*
FILENAME...     drvEnsembleAsyn.h
USAGE... This file contains Aerotech Ensemble Asyn driver "include" information.

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *      Original Author: Chad Weimer
 *      Date: 10/16/97
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *            The Controls and Automation Group (AT-8)
 *            Ground Test Accelerator
 *            Accelerator Technology Division
 *            Los Alamos National Laboratory
 *
 *      Co-developed with
 *            The Controls and Computing Group
 *            Accelerator Systems Division
 *            Advanced Photon Source
 *            Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .00  04-01-08 caw initialized from drvMM4000.h (Newport)
 */
 
#ifndef DRV_MOTOR_ENSEMBLE_ASYN_H
#define DRV_MOTOR_ENSEMBLE_ASYN_H

#include "motor.h"

/* Axis Status Register Bitmap */
typedef union
{
    epicsUInt32 All;
    struct
    {
#ifdef MSB_First
        unsigned int ESTOP                :1;
        unsigned int cos_encoder_err      :1;
        unsigned int sin_encoder_err      :1;
        unsigned int hall_C               :1;

        unsigned int hall_B               :1;
        unsigned int hall_A               :1;
        unsigned int marker               :1;
        unsigned int home_limit           :1;

        unsigned int CCW_limit            :1;
        unsigned int CW_limit             :1;
        unsigned int na18_21              :4; /* N/A bits 18-21 */
        unsigned int gantry_master_active :1;
        unsigned int gantry_mode_active   :1;
        
        unsigned int master_suppressed    :1;
        unsigned int homing_active        :1;
        unsigned int joystick_active      :1;
        unsigned int axis_calib_enabled   :1;

        unsigned int axis_calib_active    :1;
        unsigned int na10                 :1; /* N/A bit 10 */
        unsigned int motion_ccw           :1;
        unsigned int brake_on             :1;

        unsigned int current_clamp        :1;
        unsigned int pos_capture_active   :1;
        unsigned int dec_active           :1;
        unsigned int acc_active           :1;

        unsigned int move_active          :1;
        unsigned int in_position          :1;
        unsigned int home_cycle_complete  :1;
        unsigned int axis_enabled         :1;
#else
        unsigned int axis_enabled         :1;
        unsigned int home_cycle_complete  :1;
        unsigned int in_position          :1;
        unsigned int move_active          :1;

        unsigned int acc_active           :1;
        unsigned int dec_active           :1;
        unsigned int pos_capture_active   :1;
        unsigned int current_clamp        :1;

        unsigned int brake_on             :1;
        unsigned int motion_ccw           :1;
        unsigned int na10                 :1; /* N/A bit 10 */
        unsigned int axis_calib_active    :1;

        unsigned int axis_calib_enabled   :1;
        unsigned int joystick_active      :1;
        unsigned int homing_active        :1;
        unsigned int master_suppressed    :1;        
        
        unsigned int gantry_mode_active   :1;
        unsigned int gantry_master_active :1;
        unsigned int na18_21              :4; /* N/A bits 18-21 */
        unsigned int CW_limit             :1;
        unsigned int CCW_limit            :1;

        unsigned int home_limit           :1;
        unsigned int marker               :1;
        unsigned int hall_A               :1;
        unsigned int hall_B               :1;

        unsigned int hall_C               :1;
        unsigned int sin_encoder_err      :1;
        unsigned int cos_encoder_err      :1;
        unsigned int ESTOP                :1;
#endif
    } Bits;
} Axis_Status;

/* LimitLevelMask parameter bitmap */
typedef union
{
    epicsUInt32 All;
    struct
    {
#ifdef MSB_First
        unsigned int na5                  :27;
        unsigned int EOTswitch            :1;
        unsigned int LIF481mode           :1;
        unsigned int CWEOTSWstate         :1;
        unsigned int CCWEOTSWstate        :1;
        unsigned int HomeSWstate          :1;
#else
        unsigned int HomeSWstate          :1;
        unsigned int CCWEOTSWstate        :1;
        unsigned int CWEOTSWstate         :1;
        unsigned int LIF481mode           :1;
        unsigned int EOTswitch            :1;
        unsigned int na5                  :27;
#endif
    } Bits;
} Switch_Level;

#ifdef __cplusplus
extern "C" {
#endif

	int EnsembleAsynSetup(int numControllers); /* number of Ensemble controllers in system.  */

	int EnsembleAsynConfig(int card,             /* Controller number */
		const char *portName, /* asyn port name of serial or GPIB port */
		int asynAddress,      /* asyn subaddress for GPIB */
		int numAxes,          /* Number of axes this controller supports */
		int movingPollPeriod, /* Time to poll (msec) when an axis is in motion */
		int idlePollPeriod);  /* Time to poll (msec) when an axis is idle. 0 for no polling */

#ifdef __cplusplus
}
#endif
#endif
