/*
FILENAME...     drvXPSasyn.c
USAGE...        Newport XPS EPICS asyn motor device driver

*/

/*
Original Author: Mark Rivers
*/

/*
Copyright (c) 2005 University of Chicago and the Regents of the University of 
California. All rights reserved.

synApps is distributed subject to the following license conditions:
SOFTWARE LICENSE AGREEMENT
Software: synApps 
Versions: Release 4-5 and higher.

   1. The "Software", below, refers to synApps (in either source code, or 
      binary form and accompanying documentation). Each licensee is addressed 
          as "you" or "Licensee."

   2. The copyright holders shown above and their third-party licensors hereby 
      grant Licensee a royalty-free nonexclusive license, subject to the 
          limitations stated herein and U.S. Government license rights.

   3. You may modify and make a copy or copies of the Software for use within 
      your organization, if you meet the following conditions:
         1. Copies in source code must include the copyright notice and this 
                    Software License Agreement.
         2. Copies in binary form must include the copyright notice and this 
                    Software License Agreement in the documentation and/or other 
                        materials provided with the copy.

   4. You may modify a copy or copies of the Software or any portion of it, thus
      forming a work based on the Software, and distribute copies of such work
      outside your organization, if you meet all of the following conditions:
         1. Copies in source code must include the copyright notice and this 
                    Software License Agreement;
         2. Copies in binary form must include the copyright notice and this 
                    Software License Agreement in the documentation and/or other 
                        materials provided with the copy;
         3. Modified copies and works based on the Software must carry 
                    prominent notices stating that you changed specified portions of 
                        the Software.

   5. Portions of the Software resulted from work developed under a 
      U.S. Government contract and are subject to the following license: 
          the Government is granted for itself and others acting on its behalf a 
          paid-up, nonexclusive, irrevocable worldwide license in this computer 
          software to reproduce, prepare derivative works, and perform publicly and 
          display publicly.

   6. WARRANTY DISCLAIMER. THE SOFTWARE IS SUPPLIED "AS IS" WITHOUT WARRANTY OF 
      ANY KIND. THE COPYRIGHT HOLDERS, THEIR THIRD PARTY LICENSORS, THE UNITED 
          STATES, THE UNITED STATES DEPARTMENT OF ENERGY, AND THEIR EMPLOYEES: (1) 
          DISCLAIM ANY WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO 
          ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
          PURPOSE, TITLE OR NON-INFRINGEMENT, (2) DO NOT ASSUME ANY LEGAL LIABILITY 
          OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR USEFULNESS OF THE 
          SOFTWARE, (3) DO NOT REPRESENT THAT USE OF THE SOFTWARE WOULD NOT 
          INFRINGE PRIVATELY OWNED RIGHTS, (4) DO NOT WARRANT THAT THE SOFTWARE WILL 
          FUNCTION UNINTERRUPTED, THAT IT IS ERROR-FREE OR THAT ANY ERRORS WILL BE 
          CORRECTED.

   7. LIMITATION OF LIABILITY. IN NO EVENT WILL THE COPYRIGHT HOLDERS, THEIR 
      THIRD PARTY LICENSORS, THE UNITED STATES, THE UNITED STATES DEPARTMENT OF 
          ENERGY, OR THEIR EMPLOYEES: BE LIABLE FOR ANY INDIRECT, INCIDENTAL, 
          CONSEQUENTIAL, SPECIAL OR PUNITIVE DAMAGES OF ANY KIND OR NATURE, 
          INCLUDING BUT NOT LIMITED TO LOSS OF PROFITS OR LOSS OF DATA, FOR ANY 
          REASON WHATSOEVER, WHETHER SUCH LIABILITY IS ASSERTED ON THE BASIS OF 
          CONTRACT, TORT (INCLUDING NEGLIGENCE OR STRICT LIABILITY), OR OTHERWISE, 
          EVEN IF ANY OF SAID PARTIES HAS BEEN WARNED OF THE POSSIBILITY OF SUCH 
          LOSS OR DAMAGES.
*/

/*
Modification Log:
-----------------
01 11-17-2009 rls Added file header.
*/


#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "drvXPSAsyn.h"
#include "paramLib.h"

#include "epicsFindSymbol.h"
#include "epicsTime.h"
#include "epicsThread.h"
#include "epicsEvent.h"
#include "epicsString.h"
#include "epicsStdio.h"
#include "epicsMutex.h"
#include "ellLib.h"
#include "iocsh.h"

#include "drvSup.h"
#include "epicsExport.h"
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"
#include "XPS_C8_drivers.h"
#include "XPSAsynInterpose.h"
#include "tclCall.h"

extern int xps_gathering(int);

motorAxisDrvSET_t motorXPS = 
  {
    15,
    motorAxisReport,            /**< Standard EPICS driver report function (optional) */
    motorAxisInit,              /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLog,            /**< Defines an external logging function (optional) */
    motorAxisOpen,              /**< Driver open function */
    motorAxisClose,             /**< Driver close function */
    motorAxisSetCallback,       /**< Provides a callback function the driver can call when the status updates */
    motorAxisSetDouble,         /**< Pointer to function to set a double value */
    motorAxisSetInteger,        /**< Pointer to function to set an integer value */
    motorAxisGetDouble,         /**< Pointer to function to get a double value */
    motorAxisGetInteger,        /**< Pointer to function to get an integer value */
    motorAxisHome,              /**< Pointer to function to execute a more to reference or home */
    motorAxisMove,              /**< Pointer to function to execute a position move */
    motorAxisVelocityMove,      /**< Pointer to function to execute a velocity mode move */
    motorAxisStop,              /**< Pointer to function to stop motion */
    motorAxisforceCallback,     /**< Pointer to function to request a poller status update */
    motorAxisProfileMove,       /**< Pointer to function to execute a profile move */
    motorAxisTriggerProfile     /**< Pointer to function to trigger a profile move */
  };

epicsExportAddress(drvet, motorXPS);

typedef enum { none, positionMove, velocityMove, homeReverseMove, homeForwardsMove } moveType;

/* typedef struct motorAxis * AXIS_ID; */

typedef struct {
    epicsMutexId XPSC8Lock;
    int numAxes;
    char firmwareVersion[100];
    double movingPollPeriod;
    double idlePollPeriod;
    epicsEventId pollEventId;
    epicsEventId homeEventId;
    int moveToHomeAxis;
    AXIS_HDL pAxis;  /* array of axes */
    int movesDeferred;
} XPSController;

/** Struct that contains information about the XPS corrector loop.*/ 
typedef struct
{
  bool ClosedLoopStatus;
  double KP; /**< Main proportional term from PID loop.*/
  double KI; /**< Main integral term from PID loop.*/
  double KD; /**< Main differential term from PID loop.*/
  double KS;
  double IntegrationTime;
  double DerivativeFilterCutOffFrequency;
  double GKP;
  double GKI;
  double GKD;
  double KForm;
  double FeedForwardGainVelocity;
  double FeedForwardGainAcceleration;
  double Friction;
} xpsCorrectorInfo_t;

typedef struct motorAxisHandle
{
    XPSController *pController;
    int moveSocket;
    int pollSocket;
    int noDisabledError;
    PARAMS params;
    double currentPosition;
    double currentVelocity;
    double theoryPosition;
    double velocity;
    double accel;
    double minJerkTime; /* for the SGamma function */
    double maxJerkTime;
    double highLimit;
    double lowLimit;
    double stepSize;
    char *ip;
    char *positionerName;  /* read in using NameConfig*/
    char *groupName;
    int axisStatus;
    int positionerError;
    int card;
    int axis;
    motorAxisLogFunc print;
    void *logParam;
    epicsMutexId mutexId;
    xpsCorrectorInfo_t xpsCorrectorInfo;
    double deferred_position;
    int deferred_move;
    int deferred_relative;
    int referencing_mode;
    int referencing_mode_move;
} motorAxis;

typedef struct
{
  AXIS_HDL pFirst;
  epicsThreadId motorThread;
  motorAxisLogFunc print;
  void *logParam;
  epicsTimeStamp now;
} motorXPS_t;

/** Struct for a list of strings describing the different corrector types possible on the XPS.*/
typedef struct {
  char *PIPosition;
  char *PIDFFVelocity;
  char *PIDFFAcceleration;
  char *PIDDualFFVoltage;
  char *NoCorrector;
} CorrectorTypes_t;

const static CorrectorTypes_t CorrectorTypes={
  "PositionerCorrectorPIPosition",
  "PositionerCorrectorPIDFFVelocity",
  "PositionerCorrectorPIDFFAcceleration",
  "PositionerCorrectorPIDDualFFVoltage",
  "NoCorrector"
};

/** This is controlled via the XPSEnableSetPosition function (available via the IOC shell). */ 
static int doSetPosition = 1;

/**
 * Parameter to control the sleep time used when setting position. 
 * A function called XPSSetPosSleepTime(int) (millisec parameter) 
 * is available in the IOC shell to control this.
 */
static double setPosSleepTime = 0.5;
/**
 * Parameter to control the enable and disable of the poller
 * A function called XPSDisablePoll(int)
 * is available in the IOC shell to control this.
 */
static int disablePoll = 0;

/** Deadband to use for the velocity comparison with zero. */
#define XPS_VELOCITY_DEADBAND 0.0000001

static int motorXPSLogMsg(void * param, const motorAxisLogMask_t logMask, const char *pFormat, ...);
#define PRINT   (pAxis->print)
#define FLOW    motorAxisTraceFlow
#define MOTOR_ERROR   motorAxisTraceError
#define IODRIVER  motorAxisTraceIODriver

#define XPS_MAX_AXES 8
#define XPSC8_END_OF_RUN_MINUS  0x80000100
#define XPSC8_END_OF_RUN_PLUS   0x80000200
#define XPSC8_ZM_HIGH_LEVEL     0x00000004

#define TCP_TIMEOUT 2.0
static motorXPS_t drv={ NULL, NULL, motorXPSLogMsg, 0, { 0, 0 } };
static int numXPSControllers;
/* Pointer to array of controller strutures */
static XPSController *pXPSController=NULL;

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

static char* getXPSError(AXIS_HDL pAxis, int status, char *buffer);

/*Utility functions for dealing with XPS groups and setting corrector information.*/
static int isAxisInGroup(const AXIS_HDL pAxis);
static int setXPSAxisPID(AXIS_HDL pAxis, const double * value, int pidoption);
static int getXPSAxisPID(AXIS_HDL pAxis);
static void setXPSPIDValue(xpsCorrectorInfo_t *xpsCorrectorInfo, const double * value, int pidoption); 

/*Wrapper functions for the verbose PositionerCorrector functions.*/
static int PositionerCorrectorPIPositionGetWrapper(AXIS_HDL pAxis);
static int PositionerCorrectorPIDFFVelocityGetWrapper(AXIS_HDL pAxis);
static int PositionerCorrectorPIDFFAccelerationGetWrapper(AXIS_HDL pAxis);
static int PositionerCorrectorPIDDualFFVoltageGetWrapper(AXIS_HDL pAxis);

static int PositionerCorrectorPIPositionSetWrapper(AXIS_HDL pAxis);
static int PositionerCorrectorPIDFFVelocitySetWrapper(AXIS_HDL pAxis);
static int PositionerCorrectorPIDFFAccelerationSetWrapper(AXIS_HDL pAxis);
static int PositionerCorrectorPIDDualFFVoltageSetWrapper(AXIS_HDL pAxis);

/*Deferred moves functions.*/
static int processDeferredMoves(const XPSController * pController);
static int processDeferredMovesInGroup(const XPSController * pController, char * groupName);

/*Move to home functions*/
static int movePositionerToHome(AXIS_HDL pAxis);

static void motorAxisReportAxis(AXIS_HDL pAxis, int level)
{
    if (level > 0)
    {
        printf("Axis %d name:%s\n", pAxis->axis, pAxis->positionerName);
        printf("   pollSocket:%d, moveSocket:%d\n", pAxis->pollSocket, pAxis->moveSocket);
        printf("   axisStatus:%d\n", pAxis->axisStatus);
        printf("   stepSize:%g\n", pAxis->stepSize);
    }
}

static void motorAxisReport(int level)
{
    int i, j;

    for(i=0; i<numXPSControllers; i++) {
        printf("Controller %d firmware version: %s\n", i, pXPSController[i].firmwareVersion);
        for(j=0; j<pXPSController[i].numAxes; j++) {
           motorAxisReportAxis(&pXPSController[i].pAxis[j], level);
        }
    }   
}


static int motorAxisInit(void)
{
  int controller = 0;
  int axis = 0;
  AXIS_HDL pAxis;

  for(controller=0; controller<numXPSControllers; controller++) {
    for(axis=0; axis<pXPSController[controller].numAxes; axis++) {
      pAxis = &pXPSController[controller].pAxis[axis];

      if (!pAxis->mutexId) break;
      epicsMutexLock(pAxis->mutexId);

      /*Set GAIN_SUPPORT.*/
      motorParam->setInteger(pAxis->params, motorAxisHasClosedLoop, 1);
      /*Readback PID and set in motor record.*/
      /*NOTE: this will require PID to be allowed to be set greater than 1 in motor record.*/
      /*And we need to implement this in Asyn layer.*/
      getXPSAxisPID(pAxis);
      motorParam->setDouble(pAxis->params, motorAxisPGain, (pAxis->xpsCorrectorInfo).KP);
      motorParam->setDouble(pAxis->params, motorAxisIGain, (pAxis->xpsCorrectorInfo).KI);
      motorParam->setDouble(pAxis->params, motorAxisDGain, (pAxis->xpsCorrectorInfo).KD);

      /*Set motorAxisHasEncoder so that we can use UEIP field.*/
      motorParam->setDouble(pAxis->params, motorAxisHasEncoder, 1);

      /*Initialise deferred move flags.*/
      pAxis->deferred_relative = 0;
      pAxis->deferred_position = 0;
      /*Disable deferred move for the axis. Should not cause move of this axis
       if other axes in same group do deferred move.*/
      pAxis->deferred_move = 0;

      /*Initialise referencing mode flag. If this is set, ignore the home state reported by the controller.*/
      pAxis->referencing_mode = 0;

      motorParam->callCallback(pAxis->params);
      epicsMutexUnlock(pAxis->mutexId);

    }
  } 

  return MOTOR_AXIS_OK;
}

static int motorAxisSetLog( AXIS_HDL pAxis, motorAxisLogFunc logFunc, void * param )
{
    if (pAxis == NULL) 
    {
        if (logFunc == NULL)
        {
            drv.print=motorXPSLogMsg;
            drv.logParam = NULL;
        }
        else
        {
            drv.print=logFunc;
            drv.logParam = param;
        }
    }
    else
    {
        if (logFunc == NULL)
        {
            pAxis->print=motorXPSLogMsg;
            pAxis->logParam = NULL;
        }
        else
        {
            pAxis->print=logFunc;
            pAxis->logParam = param;
        }
    }
    return MOTOR_AXIS_OK;
}

static AXIS_HDL motorAxisOpen(int card, int axis, char * param)
{
  AXIS_HDL pAxis;

  if (card >= numXPSControllers) return(NULL);
  if (axis >= pXPSController[card].numAxes) return(NULL);
  pAxis = &pXPSController[card].pAxis[axis];
  return pAxis;
}

static int motorAxisClose(AXIS_HDL pAxis)
{
  return MOTOR_AXIS_OK;
}

static int motorAxisGetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int * value)
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      switch (function) {
      case motorAxisDeferMoves:
        *value = pAxis->pController->movesDeferred;
        return MOTOR_AXIS_OK;
        break;
      default:
        return motorParam->getInteger(pAxis->params, (paramIndex) function, value);
      }
    }
}

static int motorAxisGetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double * value)
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      switch (function) {
      case motorAxisDeferMoves:
        *value = pAxis->pController->movesDeferred;
        return MOTOR_AXIS_OK;
        break;
      default:
        return motorParam->getDouble(pAxis->params, (paramIndex) function, value);
      }
    }
}

static int motorAxisSetCallback(AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param)
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      return motorParam->setCallback(pAxis->params, callback, param);
    }
}

/**
 * Perform a deferred move (a coordinated group move) on all the axes in a group.
 * @param pController Pointer to XPSController structure.
 * @param groupName Pointer to string naming the group on which to perform the group move.
 * @return motor driver status code.
 */
static int processDeferredMovesInGroup(const XPSController * pController, char * groupName)
{
  double *positions = NULL;
  int positions_index = 0;
  int first_loop = 1;
  int axis = 0;
  int NbPositioners = 0;
  int relativeMove = 0;
  int status = 0;

  AXIS_HDL pAxis = NULL;

  /*Loop over all axes in this controller.*/
  for (axis=0; axis<pController->numAxes; axis++) {
    pAxis = &pController->pAxis[axis];
    
    PRINT(pAxis->logParam, FLOW, "Executing deferred move on XPS: %d, Group: %s\n", pAxis->card, groupName);
    
    /*Ignore axes in other groups.*/
    if (!strcmp(pAxis->groupName, groupName)) {
      if (first_loop) {
        /*Get the number of axes in this group, and allocate buffer for positions.*/
        NbPositioners = isAxisInGroup(pAxis);
        if ((positions = (double *)calloc(NbPositioners, sizeof(double))) == NULL) {
          PRINT(pAxis->logParam, MOTOR_ERROR, "Cannot allocate memory for positions array in processDeferredMovesInGroup.\n" );
          return MOTOR_AXIS_ERROR;
        }
        first_loop = 0;
      }
      
      /*Set relative flag for the actual move at the end of the funtion.*/
      if (pAxis->deferred_relative) {
        relativeMove = 1;
      }
      
      /*Build position buffer.*/
      if (pAxis->deferred_move) {
        positions[positions_index] = 
          pAxis->deferred_relative ? (pAxis->currentPosition + pAxis->deferred_position) : pAxis->deferred_position;
      } else {
        positions[positions_index] = 
          pAxis->deferred_relative ? 0 : pAxis->theoryPosition;
      }
      
      /*Next axis in this group.*/
      positions_index++;
    }
  }
  
  PRINT(pAxis->logParam, FLOW, "Executing deferred move on XPS: %d, Group: %s\n", pAxis->card, groupName);

  /*Send the group move command.*/
  if (relativeMove) {
    status = GroupMoveRelative(pAxis->moveSocket,
                               groupName,
                               NbPositioners,
                               positions);
  } else {
    status = GroupMoveAbsolute(pAxis->moveSocket,
                               groupName,
                               NbPositioners,
                               positions);
  }
  
  /*Clear the defer flag for all the axes in this group.*/
  /*We need to do this for the XPS, because we cannot do partial group moves. Every axis
    in the group will be included the next time we do a group move.*/
  for (axis=0; axis<pController->numAxes; axis++) {
    pAxis = &pController->pAxis[axis];
    /*Ignore axes in other groups.*/
    if (!strcmp(pAxis->groupName, groupName)) {
      pAxis->deferred_move = 0;
    }
  }
  
  if (status!=0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, "Error peforming GroupMoveAbsolute/Relative in processDeferredMovesInGroup. XPS Return code: %d\n", status);
    if (positions != NULL) {
      free(positions);
    }
    return MOTOR_AXIS_ERROR;
  }
    
  if (positions != NULL) {
    free(positions);
  }

  /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
  epicsEventSignal(pAxis->pController->pollEventId);
  
  return MOTOR_AXIS_OK;
  
}

/**
 * Process deferred moves for a controller and groups.
 * This function calculates which unique groups in the controller
 * and passes the controller pointer and group name to processDeferredMovesInGroup.
 * @return motor driver status code.
 */
static int processDeferredMoves(const XPSController * pController)
{
  int status = MOTOR_AXIS_ERROR;
  int axis = 0;
  int i = 0;
  int dealWith = 0;
  /*Array to cache up to XPS_MAX_AXES group names. Don't initialise to null*/
  char *groupNames[XPS_MAX_AXES];
  char *blankGroupName = " ";
  AXIS_HDL pAxis = NULL;

  /*Clear group name cache.*/
  for (i=0; i<XPS_MAX_AXES; i++) {
    groupNames[i] = blankGroupName;
  }

  /*Loop over axes, testing for unique groups.*/
  for(axis=0; axis<pController->numAxes; axis++) {
    pAxis = &pController->pAxis[axis];
    
    PRINT(pAxis->logParam, FLOW, "Processing deferred moves on XPS: %d\n", pAxis->card);
    
    /*Call processDeferredMovesInGroup only once for each group on this controller.
      Positioners in the same group may not be adjacent in list, so we have to test for this.*/
    for (i=0; i<XPS_MAX_AXES; i++) {
      if (strcmp(pAxis->groupName, groupNames[i])) {
        dealWith++;
        groupNames[i] = pAxis->groupName;
      }
    }
    if (dealWith == XPS_MAX_AXES) {
      dealWith = 0;
      /*Group name was not in cache, so deal with this group.*/
      status = processDeferredMovesInGroup(pController, pAxis->groupName);
    }
    /*Next axis, and potentially next group.*/
  }
  
  return status;
}

static int motorAxisSetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    int status = 0;
    int axisIndex = 0;
    int axisIndexInGrp = 0;
    int axesInGroup = 0;
    double deviceValue;
    double positions[XPS_MAX_AXES] = {0.0};
    
    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    if (!pAxis->mutexId) {
      PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetDouble[%d,%d]: invalid mutex ID. Call XPSConfigAxis first for initialization.\n", pAxis->card, pAxis->axis);
      return MOTOR_AXIS_ERROR;
    }
    else
    {
      if (epicsMutexLock( pAxis->mutexId ) == epicsMutexLockOK)
        {
        switch ((int)function)
        {
        case motorAxisPosition:
        {
          /*If the user has disabled setting the controller position, skip this.*/
          if (!doSetPosition) {
            PRINT(pAxis->logParam, MOTOR_ERROR, "XPS set position is disabled. Enable it using XPSEnableSetPosition(1).\n");
          } else {
            /*Test if this axis is in a XPS group.*/
            axesInGroup = isAxisInGroup(pAxis);

            if (axesInGroup>1) {
              /*We are in a group, so we need to read the positions of all the axes in the group,
                kill the group, and set all the positions in the group using referencing mode.
                We read the positions seperately, rather than in one command, because we can't assume
                that the ordering is the same in the XPS as in the driver.*/
              for (axisIndex=0; axisIndex<pAxis->pController->numAxes; axisIndex++) {
                status = GroupPositionCurrentGet(pAxis->pollSocket, 
                                                 pAxis->pController->pAxis[axisIndex].positionerName, 
                                                 1, 
                                                 &positions[axisIndex]);
              }
              if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupPositionCurrentGet(%d,%d). Aborting set position. XPS API Error: %d.\n", 
                      pAxis->card, pAxis->axis, status);
                ret_status = MOTOR_AXIS_ERROR;
              } else {
                status = GroupKill(pAxis->pollSocket, 
                                   pAxis->groupName);
                status = GroupInitialize(pAxis->pollSocket,
                                         pAxis->groupName);
                if (status != 0) {
                  PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupKill/GroupInitialize(%d,%d). Aborting set position. XPS API Error: %d.\n", 
                        pAxis->card, pAxis->axis, status);
                  ret_status = MOTOR_AXIS_ERROR;
                } else {

                  /*Wait after axis initialisation (we don't want to set position immediately after
                    initialisation because the stage can oscillate slightly).*/
                  epicsThreadSleep(setPosSleepTime);

                  status = GroupReferencingStart(pAxis->pollSocket, 
                                                 pAxis->groupName);
                  axisIndexInGrp = 0;
                  /*Set positions for all axes in the group using the cached values.*/
                  for (axisIndex=0; axisIndex<pAxis->pController->numAxes; axisIndex++) {
                    if (!strcmp(pAxis->groupName, pAxis->pController->pAxis[axisIndex].groupName)) {
                      /*But skip the current axis, because we do this just after the loop.*/
                      if (strcmp(pAxis->positionerName, pAxis->pController->pAxis[axisIndex].positionerName)) {
                        status = GroupReferencingActionExecute(pAxis->pollSocket, 
                                                               pAxis->pController->pAxis[axisIndex].positionerName, 
                                                               "SetPosition", 
                                                               "None", 
                                                               positions[axisIndexInGrp]);
                      }
                      ++axisIndexInGrp;
                    }
                  }
                  /*Now reset the position of the axis we are interested in, using the argument passed into this function.*/
                  status = GroupReferencingActionExecute(pAxis->pollSocket, 
                                                         pAxis->positionerName, 
                                                         "SetPosition", 
                                                         "None", 
                                                         value*(pAxis->stepSize));
                  /*Stop referencing, then we are homed on all axes in group.*/
                  /*Some types of XPS axes (eg. spindle) need a sleep here, otherwise 
                    the axis can be left in referencing mode.*/
                  epicsThreadSleep(0.05);
                  status = GroupReferencingStop(pAxis->pollSocket, 
                                                pAxis->groupName);
                  if (status != 0) {
                    PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing referencing set position (%d,%d). XPS API Error: %d.", 
                          pAxis->card, pAxis->axis, status);
                    ret_status = MOTOR_AXIS_ERROR;
                  } else {
                    ret_status = MOTOR_AXIS_OK;
                  }
                }
              }
            } else {
              /*We are not in a group, so we just need to use the XPS
                referencing mode to set the position.*/
              status = GroupKill(pAxis->pollSocket, 
                                 pAxis->groupName);
              status = GroupInitialize(pAxis->pollSocket,
                                       pAxis->groupName);
              if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupKill/GroupInitialize(%d,%d). XPS API Error: %d. Aborting set position.\n", 
                      pAxis->card, pAxis->axis, status);
                ret_status = MOTOR_AXIS_ERROR;
              } else {
                /*Wait after axis initialisation (we don't want to set position immediately after
                  initialisation because the stage can oscillate slightly).*/
                epicsThreadSleep(setPosSleepTime);

                status = GroupReferencingStart(pAxis->pollSocket, 
                                               pAxis->groupName);
                status = GroupReferencingActionExecute(pAxis->pollSocket, 
                                                       pAxis->positionerName, 
                                                       "SetPosition", 
                                                       "None", 
                                                       value*(pAxis->stepSize));
                /*Some types of XPS axes (eg. spindle) need a sleep here, otherwise 
                  the axis can be left in referencing mode.*/
                epicsThreadSleep(0.05);
                status = GroupReferencingStop(pAxis->pollSocket, 
                                              pAxis->groupName);
                if (status != 0) {
                  PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing referencing set position (%d,%d). XPS API Error: %d.", 
                        pAxis->card, pAxis->axis, status);
                  ret_status = MOTOR_AXIS_ERROR;
                } else {
                  ret_status = MOTOR_AXIS_OK;
                }
              }
            }
          }
          break;
        }
        case motorAxisEncoderRatio:
        {
            PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetDouble: XPS does not support setting encoder ratio\n");
            break;
        }
        case motorAxisResolution:
        {
            pAxis->stepSize = value;
            PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d stepSize to %f\n", pAxis->card, pAxis->axis, value);
            break;
        }
        case motorAxisLowLimit:
        {
            deviceValue = value*pAxis->stepSize;
            /* We need to read the current highLimit because otherwise we could be setting it to an invalid value */
            status = PositionerUserTravelLimitsGet(pAxis->pollSocket,
                                                   pAxis->positionerName,
                                                   &pAxis->lowLimit, &pAxis->highLimit);
            if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetDouble[%d,%d]: error performing PositionerUserTravelLimitsGet "
                      "for high limit=%f, status=%d\n", pAxis->card, pAxis->axis, deviceValue, status);
            }
            status = PositionerUserTravelLimitsSet(pAxis->pollSocket,
                                                   pAxis->positionerName,
                                                   deviceValue, pAxis->highLimit);
            if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetDouble[%d,%d]: error performing PositionerUserTravelLimitsSet "
                      "for low limit=%f, status=%d\n", pAxis->card, pAxis->axis, deviceValue, status);
            } else { 
                pAxis->lowLimit = deviceValue;
                PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d low limit to %f\n", pAxis->card, pAxis->axis, deviceValue);
                ret_status = MOTOR_AXIS_OK;
            }
            break;
        }
        case motorAxisHighLimit:
        {
            deviceValue = value*pAxis->stepSize;
            /* We need to read the current highLimit because otherwise we could be setting it to an invalid value */
            status = PositionerUserTravelLimitsGet(pAxis->pollSocket,
                                                   pAxis->positionerName,
                                                   &pAxis->lowLimit, &pAxis->highLimit);
            if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetDouble[%d,%d]: error performing PositionerUserTravelLimitsGet "
                      "for high limit=%f, status=%d\n", pAxis->card, pAxis->axis, deviceValue, status);
            }
            status = PositionerUserTravelLimitsSet(pAxis->pollSocket,
                                                   pAxis->positionerName,
                                                   pAxis->lowLimit, deviceValue);
            if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetDouble[%d,%d]: error performing PositionerUserTravelLimitsSet "
                      "for high limit=%f, status=%d\n", pAxis->card, pAxis->axis, deviceValue, status);
            } else { 
                pAxis->highLimit = deviceValue;
                PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d high limit to %f\n", pAxis->card, pAxis->axis, deviceValue);
                ret_status = MOTOR_AXIS_OK;
            }
            break;
        }
        case motorAxisPGain:
        {
          status = setXPSAxisPID(pAxis, &value, 0);
          break;
        }
        case motorAxisIGain:
        {
          status = setXPSAxisPID(pAxis, &value, 1);
          break;
        }
        case motorAxisDGain:
        {
          status = setXPSAxisPID(pAxis, &value, 2);
          break;
        }
        case motorAxisClosedLoop:
        {
            PRINT(pAxis->logParam, MOTOR_ERROR, "XPS does not support changing closed loop or torque\n");
            break;
        }
        case minJerkTime:
        {
            pAxis->minJerkTime = value;
            ret_status = MOTOR_AXIS_OK;
            break;
        }
        case maxJerkTime:
        {
            pAxis->maxJerkTime = value;
            ret_status = MOTOR_AXIS_OK;
            break;
        }
        case motorAxisDeferMoves:
        {
          PRINT(pAxis->logParam, FLOW, "Setting deferred move mode on XPS %d to %d\n", pAxis->card, value);
          if (value == 0.0 && pAxis->pController->movesDeferred != 0) {
            status = processDeferredMoves(pAxis->pController);
          }
          pAxis->pController->movesDeferred = (int)value;
          if (status) {
            PRINT(pAxis->logParam, MOTOR_ERROR, "Deferred moved failed on XPS %d, status=%d\n", pAxis->card, status);
            ret_status = MOTOR_AXIS_ERROR;
          } else {
            ret_status = MOTOR_AXIS_OK;
          }
          break;
        }
        default:
            PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetDouble[%d,%d]: unknown function %d\n", pAxis->card, pAxis->axis, function);
            break;
        }

        if (status == MOTOR_AXIS_OK )
          {
            motorParam->setDouble( pAxis->params, function, value );
            motorParam->callCallback( pAxis->params );
          }
        epicsMutexUnlock( pAxis->mutexId );
        }
    }
    
    return ret_status;
}


static int motorAxisSetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    int status = 0;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    switch (function) {
    case motorAxisClosedLoop:
        if (value) {
            status = GroupMotionEnable(pAxis->pollSocket, pAxis->groupName);
            if (status) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetInteger[%d,%d]: error calling GroupMotionEnable status=%d\n",
                      pAxis->card, pAxis->axis, status);
            } else {
                PRINT(pAxis->logParam, FLOW, "motorAxisSetInteger set card %d, axis %d closed loop enable\n",
                      pAxis->card, pAxis->axis);
            }
            ret_status = MOTOR_AXIS_OK;
        } else {
            status = GroupMotionDisable(pAxis->pollSocket, pAxis->groupName);
            if (status) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetInteger[%d,%d]: error calling GroupMotionDisable status=%d\n",
                      pAxis->card, pAxis->axis, status);
            } else {
                PRINT(pAxis->logParam, FLOW, "motorAxisSetInteger set card %d, axis %d closed loop disable\n",
                      pAxis->card, pAxis->axis);
            }
            ret_status = MOTOR_AXIS_OK;
        }
        break;
    case motorAxisDeferMoves:
    {
      PRINT(pAxis->logParam, FLOW, "Setting deferred move mode on XPS %d to %d\n", pAxis->card, value);
      if (value == 0 && pAxis->pController->movesDeferred != 0) {
        status = processDeferredMoves(pAxis->pController);
      }
      pAxis->pController->movesDeferred = value;
      if (status) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Deferred moved failed on XPS %d, status=%d\n", pAxis->card, status);
        ret_status = MOTOR_AXIS_ERROR;
      } else {
        ret_status = MOTOR_AXIS_OK;
      }
      break;
    }
    case motorAxisMoveToHome:
      {
        if (value == 1) {
          PRINT(pAxis->logParam, FLOW, "Starting move to home for axis %s %s\n", pAxis->groupName, pAxis->positionerName);
          pAxis->pController->moveToHomeAxis = pAxis->axis;
          epicsEventSignal(pAxis->pController->homeEventId);
          ret_status = MOTOR_AXIS_OK;
        } else {
          ret_status = MOTOR_AXIS_OK;
        }
        break;
      }
    default:
        PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisSetInteger[%d,%d]: unknown function %d\n", pAxis->card, pAxis->axis, function);
        break;
    }
    if (ret_status != MOTOR_AXIS_ERROR) {
      status = motorParam->setInteger(pAxis->params, function, value);
    }
    return ret_status;
}


static int motorAxisMove(AXIS_HDL pAxis, double position, int relative, 
                          double min_velocity, double max_velocity, double acceleration)
{
    int status = 0;
    char errorString[100];
    double deviceUnits;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    if (!pAxis->mutexId) {
      PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisMove[%d,%d]: invalid mutex ID. Call XPSConfigAxis first for initialization.\n", pAxis->card, pAxis->axis);
      return MOTOR_AXIS_ERROR;
    }

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f\n",
          pAxis->card, pAxis->axis, position, min_velocity, max_velocity, acceleration);

    /* Look at the last poll value of the positioner status.  If it is disabled, then enable it */
    if (pAxis->axisStatus >= 20 && pAxis->axisStatus <= 36) {
        status = GroupMotionEnable(pAxis->pollSocket, pAxis->groupName);
        if (status) {
            PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisMove[%d,%d]: error performing GroupMotionEnable %d\n",pAxis->card, pAxis->axis, status);
            /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
            return MOTOR_AXIS_ERROR;
        }
    }

    status = PositionerSGammaParametersSet(pAxis->pollSocket,
                                           pAxis->positionerName, 
                                           max_velocity*pAxis->stepSize,
                                           ((acceleration!=0) ? acceleration*pAxis->stepSize : pAxis->accel),
                                           pAxis->minJerkTime,
                                           pAxis->maxJerkTime);
    if (status != 0) {
        ErrorStringGet(pAxis->pollSocket, status, errorString);
        PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing PositionerSGammaParametersSet[%d,%d] %d: %s\n",
              pAxis->card, pAxis->axis, status, errorString);
        return MOTOR_AXIS_ERROR;
    }

    deviceUnits = position * pAxis->stepSize;
    if (relative) {
      if (pAxis->pController->movesDeferred == 0) {
        status = GroupMoveRelative(pAxis->moveSocket,
                                   pAxis->positionerName,
                                   1,
                                   &deviceUnits); 
      } else {
        pAxis->deferred_position = deviceUnits;
        pAxis->deferred_move = 1;
        pAxis->deferred_relative = relative;
      }
      if (status != 0 && status != -27) {
        PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupMoveRelative[%d,%d] %d\n", pAxis->card, pAxis->axis, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
        return MOTOR_AXIS_ERROR;
      }
    } else {
      if (pAxis->pController->movesDeferred == 0) {
        status = GroupMoveAbsolute(pAxis->moveSocket,
                                   pAxis->positionerName,
                                   1,
                                   &deviceUnits); 
      } else {
        pAxis->deferred_position = deviceUnits;
        pAxis->deferred_move = 1;
        pAxis->deferred_relative = relative;
      }
      if (status != 0 && status != -27) {
        PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupMoveAbsolute[%d,%d] %d\n",pAxis->card, pAxis->axis, status);
        /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
        return MOTOR_AXIS_ERROR;
      }
    }
    /* Tell paramLib that the motor is moving.  
     * This will force a callback on the next poll, even if the poll says the motor is already done. */
    if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK)
    {
        /* Insure that the motor record's next status update sees motorAxisDone = False. */
        motorParam->setInteger(pAxis->params, motorAxisDone, 0);
        motorParam->callCallback(pAxis->params);
        epicsMutexUnlock(pAxis->mutexId);
    }
    
    motorParam->callCallback( pAxis->params );
    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    return MOTOR_AXIS_OK;
}

static int motorAxisHome(AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards)
{
    int status;
    int groupStatus;
    char errorBuffer[100];
    int axis = 0;
    XPSController *pController = NULL;
    AXIS_HDL pTempAxis = NULL;
    
    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    pController = pAxis->pController;

    /* Find out if any axes are in the same group, and clear the referencing mode for them.*/
    for (axis=0; axis<pController->numAxes; axis++) {
      pTempAxis = &pController->pAxis[axis];
      if (strcmp(pAxis->groupName, pTempAxis->groupName) == 0) {
        pTempAxis->referencing_mode = 0;
      }  
    }

    status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
    /* The XPS won't allow a home command if the group is in the Ready state
     * If the group is Ready, then make it not Ready  */
    if (groupStatus >= 10 && groupStatus <= 18) {
        status = GroupKill(pAxis->pollSocket, pAxis->groupName);
        if (status) {
            PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisHome[%d,%d]: error calling GroupKill error=%s\n",
                  pAxis->card, pAxis->axis, getXPSError(pAxis, status, errorBuffer));
            return MOTOR_AXIS_ERROR;
        }
    }
    status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
    /* If axis not initialized, then initialize it */
    if ((groupStatus >= 0 && groupStatus <= 9) || (groupStatus == 50) || (groupStatus == 63)) {
        status = GroupInitialize(pAxis->pollSocket, pAxis->groupName);
        if (status) {
            PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisHome[%d,%d]: error calling GroupInitialize error=%s\n",
                  pAxis->card, pAxis->axis, getXPSError(pAxis, status, errorBuffer));
            return MOTOR_AXIS_ERROR;
        }
    }
    status = GroupHomeSearch(pAxis->moveSocket, pAxis->groupName);
    if (status) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisHome[%d,%d]: error calling GroupHomeSearch error=%s\n",
              pAxis->card, pAxis->axis, getXPSError(pAxis, status, errorBuffer));
        return MOTOR_AXIS_ERROR;
    }

    /* Tell paramLib that the motor is moving.  
     * This will force a callback on the next poll, even if the poll says the motor is already done. */
    
    if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK)
    {
        /* Insure that the motor record's next status update sees motorAxisDone = False. */
        motorParam->setInteger(pAxis->params, motorAxisDone, 0);
        motorParam->callCallback(pAxis->params);
        epicsMutexUnlock(pAxis->mutexId);
    }
    
    motorParam->callCallback( pAxis->params );
    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);
    PRINT(pAxis->logParam, FLOW, "motorAxisHome: set card %d, axis %d to home\n",
          pAxis->card, pAxis->axis);
    return MOTOR_AXIS_OK;
}


static int motorAxisVelocityMove(AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration)
{
    int status = MOTOR_AXIS_ERROR;
    double deviceVelocity, deviceAcceleration;

    if (pAxis == NULL) return(status);
    status = GroupJogModeEnable(pAxis->pollSocket, pAxis->groupName);
    if (status) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisVelocityMove[%d,%d]: error calling GroupJogModeEnable=%d\n",
              pAxis->card, pAxis->axis, status);
        return MOTOR_AXIS_ERROR;
    }
    deviceVelocity = velocity * pAxis->stepSize;
    deviceAcceleration = acceleration * pAxis->stepSize;
    status = GroupJogParametersSet(pAxis->moveSocket, pAxis->positionerName, 1, &deviceVelocity, &deviceAcceleration);
    if (status) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "motorAxisVelocityMove[%d,%d]: error calling GroupJogParametersSet=%d\n",
              pAxis->card, pAxis->axis, status);
        return MOTOR_AXIS_ERROR;
    }
    /* Tell paramLib that the motor is moving.  
     * This will force a callback on the next poll, even if the poll says the motor is already done. */

    if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK)
    {
        /* Insure that the motor record's next status update sees motorAxisDone = False. */
        motorParam->setInteger(pAxis->params, motorAxisDone, 0);
        motorParam->callCallback(pAxis->params);
        epicsMutexUnlock(pAxis->mutexId);
    }

    motorParam->callCallback( pAxis->params );
    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);
    PRINT(pAxis->logParam, FLOW, "motorAxisVelocityMove card %d, axis %d move velocity=%f, accel=%f\n",
          pAxis->card, pAxis->axis, velocity, acceleration);
    return status;
}

static int motorAxisProfileMove(AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger)
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisTriggerProfile(AXIS_HDL pAxis)
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisStop(AXIS_HDL pAxis, double acceleration)
{
    int status;
    double deviceVelocity=0.;
    double deviceAcceleration;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    
    /* We need to read the status, because a jog is stopped differently from a move */ 

    status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &pAxis->axisStatus);
    if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupStatusGet[%d,%d] status=%d%\n",\
              pAxis->card, pAxis->axis, status);
        return MOTOR_AXIS_ERROR;
    }
    if (pAxis->axisStatus == 47) {
        deviceAcceleration = acceleration * pAxis->stepSize;
        status = GroupJogParametersSet(pAxis->moveSocket, pAxis->positionerName, 1, &deviceVelocity, &deviceAcceleration);
        if (status != 0) {
            PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupJogParametersSet[%d,%d] status=%d\n",\
                  pAxis->card, pAxis->axis, status);
            return MOTOR_AXIS_ERROR;
        }
    }
    
    if ((pAxis->axisStatus == 44) || (pAxis->axisStatus == 45)) {
        status = GroupMoveAbort(pAxis->moveSocket, pAxis->groupName);
        if (status != 0) {
            PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupMoveAbort axis=%s status=%d. Trying again.\n",\
                  pAxis->positionerName, status);
            GroupMoveAbort(pAxis->moveSocket, pAxis->groupName);
            return MOTOR_AXIS_ERROR;
        }
    }

    if (pAxis->axisStatus == 43) {
        status = GroupKill(pAxis->moveSocket, pAxis->groupName);
        if (status != 0) {
            PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupKill axis=%s status=%d\n",\
                  pAxis->positionerName, status);
            return MOTOR_AXIS_ERROR;
        }
    }
    

    /*Clear defer move flag for this axis.*/
    pAxis->deferred_move = 0;


    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d to stop with accel=%f\n",
          pAxis->card, pAxis->axis, acceleration);
    return MOTOR_AXIS_OK;
}


/*Commented out for now, in case we don't need this.*/
static int motorAxisforceCallback(AXIS_HDL pAxis)
{
     if (pAxis == NULL)
         return (MOTOR_AXIS_ERROR);

     PRINT(pAxis->logParam, FLOW, "motorAxisforceCallback: request card %d, axis %d status update\n",
           pAxis->card, pAxis->axis);

     motorParam->forceCallback(pAxis->params);

     epicsEventSignal(pAxis->pController->pollEventId);
     return (MOTOR_AXIS_OK);
}



static void XPSPoller(XPSController *pController)
{
    /* This is the task that polls the XPS */
    double timeout;
    AXIS_HDL pAxis;
    int status;
    int i;
    int axisDone;
    int anyMoving;
    int forcedFastPolls=0;
    double actualVelocity, theoryVelocity, acceleration;
    double theoryPosition=0;

    timeout = pController->idlePollPeriod;
    epicsEventSignal(pController->pollEventId);  /* Force on poll at startup */

    while(1) {
            while(disablePoll==1)
            {
                    epicsThreadSleep(0.1);
            }
        if (timeout != 0.) status = epicsEventWaitWithTimeout(pController->pollEventId, timeout);
        else               status = epicsEventWait(pController->pollEventId);
        if (status == epicsEventWaitOK) {
            /* We got an event, rather than a timeout.  This is because other software
             * knows that an axis should have changed state (started moving, etc.).
             * Force a minimum number of fast polls, because the controller status
             * might not have changed the first few polls
             */
            forcedFastPolls = 10;
         }
        
        anyMoving = 0;
        for (i=0; i<pController->numAxes; i++) {
            pAxis = &pController->pAxis[i];
            if (!pAxis->mutexId) break;
            if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK) {
            status = GroupStatusGet(pAxis->pollSocket, 
                                    pAxis->groupName, 
                                    &pAxis->axisStatus);
            if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "XPSPoller: error calling GroupStatusGet[%d,%d], status=%d\n", pAxis->card, pAxis->axis, status);
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
            } else {
                PRINT(pAxis->logParam, IODRIVER, "XPSPoller: %s axisStatus=%d\n", pAxis->positionerName, pAxis->axisStatus);
                motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
                /* Set done flag by default */
                axisDone = 1;
                if (pAxis->axisStatus >= 10 && pAxis->axisStatus <= 18) {
                    /* These states mean ready from move/home/jog etc */
                }
                if (pAxis->axisStatus >= 43 && pAxis->axisStatus <= 48) {
                    /* These states mean it is moving/homeing/jogging etc*/
                    axisDone = 0;
                    anyMoving = 1;
                    if (pAxis->axisStatus == 47) {
                        /* We are jogging.  When the velocity gets back to 0 disable jogging */
                        status = GroupJogParametersGet(pAxis->pollSocket, pAxis->positionerName, 1, &theoryVelocity, &acceleration);
                        status = GroupJogCurrentGet(pAxis->pollSocket, pAxis->positionerName, 1, &actualVelocity, &acceleration);
                        if (status != 0) {
                            PRINT(pAxis->logParam, MOTOR_ERROR, "XPSPoller: error calling GroupJogCurrentGet[%d,%d], status=%d\n", pAxis->card, pAxis->axis, status);
                        } else {
                            if (actualVelocity == 0. && theoryVelocity == 0.) {
                                status = GroupJogModeDisable(pAxis->pollSocket, pAxis->groupName);
                                if (status != 0) {
                                    PRINT(pAxis->logParam, MOTOR_ERROR, "XPSPoller: error calling GroupJogModeDisable[%d,%d], status=%d\n", pAxis->card, pAxis->axis, status);
                                    /* In this mode must do a group kill? */
                                    status = GroupKill(pAxis->pollSocket, pAxis->groupName);
                                    PRINT(pAxis->logParam, MOTOR_ERROR, "XPSPoller: called GroupKill!\n");
                                }
                            } 
                        }
                    }
                }
                /* Set the status */
                motorParam->setInteger(pAxis->params, XPSStatus, pAxis->axisStatus);
                /* Set the axis done parameter */
                /* AND the done flag with the inverse of deferred_move.*/
                axisDone &= !pAxis->deferred_move;
                motorParam->setInteger(pAxis->params, motorAxisDone, axisDone);
                
                /*Read the controller software limits in case these have been changed by a TCL script.*/
                status = PositionerUserTravelLimitsGet(pAxis->pollSocket, pAxis->positionerName, &pAxis->lowLimit, &pAxis->highLimit);
                if (status == 0) {
                  motorParam->setDouble(pAxis->params, motorAxisLowLimit, (pAxis->lowLimit/pAxis->stepSize));
                  motorParam->setDouble(pAxis->params, motorAxisHighLimit, (pAxis->highLimit/pAxis->stepSize));
                }

                /*Set the ATHM signal.*/
                if (pAxis->axisStatus == 11) {
                  if (pAxis->referencing_mode == 0) {
                    motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 1);
                  } else {
                    motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 0);
                  }
                } else {
                    motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 0);
                }

                /*Set the HOMED signal.*/
                if ((pAxis->axisStatus >= 10 && pAxis->axisStatus <= 21) || 
                    (pAxis->axisStatus == 44)) {
                  if (pAxis->referencing_mode == 0) {
                    motorParam->setInteger(pAxis->params, motorAxisHomed, 1);
                  } else {
                    motorParam->setInteger(pAxis->params, motorAxisHomed, 0);
                  }
                } else {
                  motorParam->setInteger(pAxis->params, motorAxisHomed, 0);
                }

                /*Test for following error, and set appropriate param.*/
                if ((pAxis->axisStatus == 21 || pAxis->axisStatus == 22) ||
                    (pAxis->axisStatus >= 24 && pAxis->axisStatus <= 26) ||
                    (pAxis->axisStatus == 28 || pAxis->axisStatus == 35)) {
                  PRINT(pAxis->logParam, FLOW, "XPS Axis %d in following error. XPS State Code: %d\n",
                           pAxis->axis, pAxis->axisStatus);
                  motorParam->setInteger(pAxis->params, motorAxisFollowingError, 1);
                } else {
                  motorParam->setInteger(pAxis->params, motorAxisFollowingError, 0);
                }

                /*Test for states that mean we cannot move an axis (disabled, uninitialised, etc.) 
                  and set motorAxisPowerOn (CNEN).*/
                if ((pAxis->axisStatus < 10) || ((pAxis->axisStatus >= 20) && (pAxis->axisStatus <= 42)) ||
                    (pAxis->axisStatus == 50) || (pAxis->axisStatus == 63) || (pAxis->axisStatus == 64)) {
                  if ( (pAxis->noDisabledError > 0) && (pAxis->axisStatus==20) ) {
                    motorParam->setInteger(pAxis->params, motorAxisPowerOn, 1);                    
                  } else {
                    PRINT(pAxis->logParam, FLOW, "XPS Axis %d is uninitialised/disabled/not referenced. XPS State Code: %d\n",
                           pAxis->axis, pAxis->axisStatus);
                    motorParam->setInteger(pAxis->params, motorAxisPowerOn, 0);
                  }
                } else {
                  motorParam->setInteger(pAxis->params, motorAxisPowerOn, 1);
                }
                
                /*Test for uninitialized states.*/
                if ((pAxis->axisStatus < 10) || (pAxis->axisStatus == 42) || (pAxis->axisStatus == 50) || (pAxis->axisStatus == 63)) {
                  PRINT(pAxis->logParam, FLOW, "XPS Axis %d is uninitialised. XPS State Code: %d\n",
                         pAxis->axis, pAxis->axisStatus);
                  motorParam->setInteger(pAxis->params, motorAxisProblem, 1);
                } else {
                  motorParam->setInteger(pAxis->params, motorAxisProblem, 0);
                }
                
            }

            status = GroupPositionSetpointGet(pAxis->pollSocket,
                                              pAxis->positionerName,
                                              1,
                                              &theoryPosition);

            pAxis->theoryPosition = theoryPosition;

            status = GroupPositionCurrentGet(pAxis->pollSocket,
                                             pAxis->positionerName,
                                             1,
                                             &pAxis->currentPosition);

            if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "XPSPoller: error calling GroupPositionCurrentGet[%d,%d], status=%d\n", pAxis->card, pAxis->axis, status);
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
            } else {
                motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
                motorParam->setDouble(pAxis->params, motorAxisPosition,    (theoryPosition/pAxis->stepSize));
                motorParam->setDouble(pAxis->params, motorAxisEncoderPosn, (pAxis->currentPosition/pAxis->stepSize));
            }

            status = PositionerErrorGet(pAxis->pollSocket,
                                        pAxis->positionerName,
                                        &pAxis->positionerError);
            if (status != 0) {
                PRINT(pAxis->logParam, MOTOR_ERROR, "XPSPoller: error calling PositionerErrorGet[%d,%d], status=%d\n", pAxis->card, pAxis->axis, status);
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
            } else {
                motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
                /* These are hard limits */
                if (pAxis->positionerError & XPSC8_END_OF_RUN_PLUS) {
                    motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 1);
                } else {
                    motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 0);
                }
                if (pAxis->positionerError & XPSC8_END_OF_RUN_MINUS) {
                    motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 1);
                } else {
                    motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 0);
                }
            }

            /*Read the current velocity and use it set motor direction and moving flag.*/
            status = GroupVelocityCurrentGet(pAxis->pollSocket,
                                             pAxis->positionerName,
                                             1,
                                             &pAxis->currentVelocity);
            if (status != 0) {
              PRINT(pAxis->logParam, MOTOR_ERROR, "XPSPoller: error calling GroupPositionVelocityGet[%d,%d], status=%d\n", pAxis->card, pAxis->axis, status);
              motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
            } else {
              motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
              motorParam->setInteger(pAxis->params, motorAxisDirection, (pAxis->currentVelocity > XPS_VELOCITY_DEADBAND));
              motorParam->setInteger(pAxis->params, motorAxisMoving,    (fabs(pAxis->currentVelocity) > XPS_VELOCITY_DEADBAND));
            }
            
            motorParam->callCallback(pAxis->params);

            epicsMutexUnlock(pAxis->mutexId);
        }

        } /* Next axis */

        if (forcedFastPolls > 0) {
            timeout = pController->movingPollPeriod;
            forcedFastPolls--;
        } else if (anyMoving) {
            timeout = pController->movingPollPeriod;
        } else {
            timeout = pController->idlePollPeriod;
        }

    } /* End while */

}

/**
 * This is the task that deals with the special move to home switch function 
 */
static void XPSMoveToHome(XPSController *pController)
{
  
  AXIS_HDL pAxis;
  int status = 0;
  
  while(1) {
    
    status = epicsEventWait(pController->homeEventId);
    if (status == epicsEventWaitOK) { 
      pAxis = &(pController->pAxis[pController->moveToHomeAxis]);
      status = movePositionerToHome(pAxis);
      if (status) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Move to home failed on XPS %d, status=%d\n", pAxis->card, status);
        PRINT(pAxis->logParam, MOTOR_ERROR, "Axis %s %s\n\n", pAxis->groupName, pAxis->positionerName);
      }
    }
    
  }
  
}


static char *getXPSError(AXIS_HDL pAxis, int status, char *buffer)
{
    status = ErrorStringGet(pAxis->pollSocket, status, buffer);
    return buffer;
}

static int motorXPSLogMsg(void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{

    va_list     pvar;
    int         nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout,pFormat,pvar);
    va_end (pvar);
    printf("\n");
    return(nchar);
}


int XPSSetup(int num_controllers)   /* number of XPS controllers in system.  */
{

    if (num_controllers < 1) {
        printf("XPSSetup, num_controllers must be > 0\n");
        return MOTOR_AXIS_ERROR;
    }
    numXPSControllers = num_controllers;
    pXPSController = (XPSController *)calloc(numXPSControllers, sizeof(XPSController)); 
    return MOTOR_AXIS_OK;
}


int XPSConfig(int card,           /* Controller number */
              const char *ip,     /* XPS IP address or IP name */
              int port,           /* IP port number that XPS is listening on */
              int numAxes,        /* Number of axes this controller supports */
              int movingPollPeriod, /* Time to poll (msec) when an axis is in motion */
              int idlePollPeriod)   /* Time to poll (msec) when an axis is idle. 0 for no polling */

{
    AXIS_HDL pAxis;
    int axis;
    int pollSocket;
    XPSController *pController;
    char threadName[20];

    if (numXPSControllers < 1) {
        printf("XPSConfig: no XPS controllers allocated, call XPSSetup first\n");
        return MOTOR_AXIS_ERROR;
    }
    if ((card < 0) || (card >= numXPSControllers)) {
        printf("XPSConfig: card must in range 0 to %d\n", numXPSControllers-1);
        return MOTOR_AXIS_ERROR;
    }
    if ((numAxes < 1) || (numAxes > XPS_MAX_AXES)) {
        printf("XPSConfig: numAxes must in range 1 to %d\n", XPS_MAX_AXES);
        return MOTOR_AXIS_ERROR;
    }

    pController = &pXPSController[card];
    pController->pAxis = (AXIS_HDL) calloc(numAxes, sizeof(motorAxis));
    pController->numAxes = numAxes;
    pController->movingPollPeriod = movingPollPeriod/1000.;
    pController->idlePollPeriod = idlePollPeriod/1000.;

    pollSocket = TCP_ConnectToServer((char *)ip, port, TCP_TIMEOUT);

    if (pollSocket < 0) {
        printf("XPSConfig: error calling TCP_ConnectToServer for pollSocket\n");
        return MOTOR_AXIS_ERROR;
    }

    for (axis=0; axis<numAxes; axis++) {
        pAxis = &pController->pAxis[axis];
        pAxis->pController = pController;
        pAxis->card = card;
        pAxis->axis = axis;
        pAxis->pollSocket = pollSocket;
        pAxis->ip = epicsStrDup(ip);
        pAxis->moveSocket = TCP_ConnectToServer((char *)ip, port, TCP_TIMEOUT);
        if (pAxis->moveSocket < 0) {
            printf("XPSConfig: error calling TCP_ConnectToServer for move socket\n");
            return MOTOR_AXIS_ERROR;
        }
        /* Set the poll rate on the moveSocket to a negative number, which means that SendAndReceive should do only a write, no read */
        TCP_SetTimeout(pAxis->moveSocket, -0.1);
        /* printf("XPSConfig: pollSocket=%d, moveSocket=%d, ip=%s, port=%d,"
         *     " axis=%d controller=%d\n",
         *     pAxis->pollSocket, pAxis->moveSocket, ip, port, axis, card);
         */
        pAxis->params = motorParam->create(0, MOTOR_AXIS_NUM_PARAMS + XPS_NUM_PARAMS);
    }

    FirmwareVersionGet(pollSocket, pController->firmwareVersion);
    
    pController->pollEventId = epicsEventMustCreate(epicsEventEmpty);
    pController->homeEventId = epicsEventMustCreate(epicsEventEmpty);

    /* Create the poller thread for this controller */
    epicsSnprintf(threadName, sizeof(threadName), "XPS:%d", card);
    epicsThreadCreate(threadName,
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) XPSPoller, (void *) pController);

    /* Create the thread for this controller to deal with move to home functions*/
    epicsSnprintf(threadName, sizeof(threadName), "XPS home:%d", card);
    epicsThreadCreate(threadName,
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) XPSMoveToHome, (void *) pController);
    

    return MOTOR_AXIS_OK;

}


int XPSConfigAxis(int card,                   /* specify which controller 0-up*/
                  int axis,                   /* axis number 0-7 */
                  const char *positionerName, /* groupName.positionerName e.g. Diffractometer.Phi */
                  const char *stepsPerUnit,   /* steps per user unit */
                  int noDisabledError)        /* If 1 then don't report disabled state as error */
{
    XPSController *pController;
    AXIS_HDL pAxis;
    char *index;
    int status;
    double stepSize;

    if (numXPSControllers < 1) {
        printf("XPSConfigAxis: no XPS controllers allocated, call XPSSetup first\n");
        return MOTOR_AXIS_ERROR;
    }
    if ((card < 0) || (card >= numXPSControllers)) {
        printf("XPSConfigAxis: card must in range 0 to %d\n", numXPSControllers-1);
        return MOTOR_AXIS_ERROR;
    }
    pController = &pXPSController[card];
    if ((axis < 0) || (axis >= pController->numAxes)) {
        printf("XPSConfigAxis: axis must in range 0 to %d\n", pController->numAxes-1);
        return MOTOR_AXIS_ERROR;
    }
    pAxis = &pController->pAxis[axis];
    index = strchr(positionerName, '.');
    if (index == NULL) {
        printf("XPSConfigAxis: positionerName must be of form group.positioner\n");
        return MOTOR_AXIS_ERROR;
    }
    pAxis->positionerName = epicsStrDup(positionerName);
    pAxis->groupName = epicsStrDup(positionerName);
    pAxis->noDisabledError = noDisabledError;
    index = strchr(pAxis->groupName, '.');
    if (index != NULL) *index = '\0';  /* Terminate group name at place of '.' */

    stepSize = strtod(stepsPerUnit, NULL);
    pAxis->stepSize = 1./stepSize;
    /* Read some information from the controller for this axis */
    status = PositionerSGammaParametersGet(pAxis->pollSocket,
                                           pAxis->positionerName,
                                           &pAxis->velocity,
                                           &pAxis->accel,
                                           &pAxis->minJerkTime,
                                           &pAxis->maxJerkTime);
    pAxis->mutexId = epicsMutexMustCreate();

    /* Send a signal to the poller task which will make it do a poll, 
     * updating values for this axis to use the new resolution (stepSize) */
    epicsEventSignal(pAxis->pController->pollEventId);

    /*Initialise this to zero, to disable the movePositionerToHome function by default.*/
    pAxis->referencing_mode_move = 0;
    
    return MOTOR_AXIS_OK;
}

/**
 * Function to enable/disable the write down of position to the 
 * XPS controller. Call this function at IOC shell.
 * @param setPos 0=disable, 1=enable
 */
void XPSEnableSetPosition(int setPos) 
{
  doSetPosition = setPos;
}

/**
 * Function to set the threadSleep time used when setting the XPS position.
 * The sleep is performed after the axes are initialised, to take account of any
 * post initialisation wobble.
 * @param posSleep The time in miliseconds to sleep.
 */
void XPSSetPosSleepTime(int posSleep) 
{
  setPosSleepTime = (double)posSleep / 1000.0;
}

void XPSDisablePoll(int disablePollVal)
{
        disablePoll = disablePollVal;
}


/* Utility functions.*/

/**
 * Test if axis is configured as an XPS single axis or a group.
 * This is done by comparing cached group names.
 * @param pAxis Axis struct AXIS_HDL
 * @return 1 if in group single group, or return the number of axes in the group.
 */
static int isAxisInGroup(const AXIS_HDL pAxis)
{
  int axisIndex=0;
  int group=0;

  for(axisIndex=0; axisIndex<pAxis->pController->numAxes; ++axisIndex) {
    if (!strcmp(pAxis->groupName, pAxis->pController->pAxis[axisIndex].groupName)) {
      ++group;
    }
  } 
   
  return group;
}


/**
 * Function to set the XPS controller PID parameters.
 * @param pAxis Axis struct AXIS_HDL
 * @param value The desired value of the parameter.
 * @param pidoption Set to 0 for P, 1 for I and 2 for D.
 *
 * @return Zero if success, non-zero if error (and equal to XPS API error if error is from XPS).
 */
static int setXPSAxisPID(AXIS_HDL pAxis, const double * value, int pidoption)
{
  int status = 0;
  char correctorType[250] = {'\0'};

  /*The XPS function that we use to set the PID parameters is dependant on the 
    type of corrector in use for that axis.*/
  status = PositionerCorrectorTypeGet(pAxis->pollSocket,
                                      pAxis->positionerName,
                                      correctorType);
  if (status != 0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorTypeGet. Card: %d, Axis: %d, XPS API Error: %d\n",
          pAxis->card, pAxis->axis, status);
  } else {

    if (!strcmp(correctorType, CorrectorTypes.PIPosition)) {
      /*Read the PID parameters first.*/
      status = PositionerCorrectorPIPositionGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIPositionGet. Aborting setting PID. XPS API Error: %d\n", status);
        return status;
      }

      /*Set the P, I or D parameter in the xpsCorrectorInfo struct.*/
      setXPSPIDValue(&pAxis->xpsCorrectorInfo, value, pidoption); 

      /*Now set the parameters in the XPS.*/
      status = PositionerCorrectorPIPositionSetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIPositionSet. XPS API Error: %d\n", status);
        return status;
      }

    } else if (!strcmp(correctorType, CorrectorTypes.PIDFFVelocity)) {
      status = PositionerCorrectorPIDFFVelocityGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDFFVelocityGet. Aborting setting PID. XPS API Error: %d\n", status);
        return status;
      }

      setXPSPIDValue(&pAxis->xpsCorrectorInfo, value, pidoption); 

      status = PositionerCorrectorPIDFFVelocitySetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDFFVelocitySet. XPS API Error: %d\n", status);
        return status;
      }

    } else if (!strcmp(correctorType, CorrectorTypes.PIDFFAcceleration)) {
      status = PositionerCorrectorPIDFFAccelerationGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDFFAccelerationGet. Aborting setting PID. XPS API Error: %d\n", status);
        return status;
      }

      setXPSPIDValue(&pAxis->xpsCorrectorInfo, value, pidoption); 

      status = PositionerCorrectorPIDFFAccelerationSetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDFFAccelerationSet. XPS API Error: %d\n", status);
        return status;
      }

    } else if (!strcmp(correctorType, CorrectorTypes.PIDDualFFVoltage)) {
      status = PositionerCorrectorPIDDualFFVoltageGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDDualFFVoltageGet. Aborting setting PID. XPS API Error: %d\n", status);
        return status;
      }

      setXPSPIDValue(&pAxis->xpsCorrectorInfo, value, pidoption); 

      status = PositionerCorrectorPIDDualFFVoltageSetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDDualFFVoltageSet. XPS API Error: %d\n", status);
        return status;
      }

    } else if (!strcmp(correctorType, CorrectorTypes.NoCorrector)) {
      printf("drvXPSAsyn::setXPSAxisPID. XPS corrector type is %s. Cannot set PID.\n", correctorType); 

    } else {
      printf("ERROR: drvXPSAsyn::setXPSAxisPID. %s is not a valid corrector type. PID not set.\n", correctorType); 
    }
  }

  return status;
}

/**
 * Function to read the PID values from the XPS (and any other XPS corrector info that is valid for the axis). 
 * The read values are set in the AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Zero if success, non-zero if error (and equal to XPS API error if error is from XPS).
 */
static int getXPSAxisPID(AXIS_HDL pAxis) 
{
  int status = 0;
  char correctorType[250] = {'\0'};
  
  /*The XPS function that we use to set the PID parameters is dependant on the 
    type of corrector in use for that axis.*/
  status = PositionerCorrectorTypeGet(pAxis->pollSocket,
                                      pAxis->positionerName,
                                      correctorType);
  if (status != 0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorTypeGet. Card: %d, Axis: %d, XPS API Error: %d\n",
          pAxis->card, pAxis->axis, status);
  } else {

    if (!strcmp(correctorType, CorrectorTypes.PIPosition)) {
      /*Read the PID parameters and set in pAxis.*/
      status = PositionerCorrectorPIPositionGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIPositionGet. XPS API Error: %d\n", status);
        return status;
      }
      
    } else if (!strcmp(correctorType, CorrectorTypes.PIDFFVelocity)) {
      status = PositionerCorrectorPIDFFVelocityGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDFFVelocityGet. XPS API Error: %d\n", status);
        return status;
      }

    } else if (!strcmp(correctorType, CorrectorTypes.PIDFFAcceleration)) {
      status = PositionerCorrectorPIDFFAccelerationGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDFFAccelerationGet. XPS API Error: %d\n", status);
        return status;
      }

    } else if (!strcmp(correctorType, CorrectorTypes.PIDDualFFVoltage)) {
      status = PositionerCorrectorPIDDualFFVoltageGetWrapper(pAxis);
      if (status != 0) {
        PRINT(pAxis->logParam, MOTOR_ERROR, "Error with PositionerCorrectorPIDDualFFVoltageGet. XPS API Error: %d\n", status);
        return status;
      }

    } else if (!strcmp(correctorType, CorrectorTypes.NoCorrector)) {
        printf("drvXPSAsyn::setXPSAxisPID. XPS corrector type is %s.\n", correctorType); 
        
    } else {
      printf("ERROR: drvXPSAsyn::setXPSAxisPID. %s is not a valid corrector type.\n", correctorType); 
    }
  }

  return 0;
}


/**
 * Set the P, I or D parameter in a xpsCorrectorInfo_t struct.
 * @param xpsCorrectorInfo Pointer to a xpsCorrectorInfo_t struct.
 * @param value The value to set.
 * @param pidoption Set to 0 for P, 1 for I and 2 for D.
 */
static void setXPSPIDValue(xpsCorrectorInfo_t *xpsCorrectorInfo, const double * value, int pidoption) 
{
  if ((pidoption < 0) || (pidoption > 2)) {
    printf("ERROR: drvXPSAsyn::setXPSPIDValue. pidoption out of range\n");
  } else {
    switch (pidoption) {
    case 0:
      xpsCorrectorInfo->KP = *value;
      break;
    case 1:
      xpsCorrectorInfo->KI = *value;
      break;
    case 2:
      xpsCorrectorInfo->KD = *value;
      break;
    default:
      /*Do nothing.*/
      break;
    }
  }  
}

/**
 * Wrapper function for PositionerCorrectorPIPositionGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIPositionGetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIPositionGet(pAxis->pollSocket,
                                          pAxis->positionerName,
                                          &xpsCorrectorInfo->ClosedLoopStatus,
                                          &xpsCorrectorInfo->KP, 
                                          &xpsCorrectorInfo->KI, 
                                          &xpsCorrectorInfo->IntegrationTime);
}

/**
 * Wrapper function for PositionerCorrectorPIDFFVelocityGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIDFFVelocityGetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIDFFVelocityGet(pAxis->pollSocket,
                                             pAxis->positionerName,
                                             &xpsCorrectorInfo->ClosedLoopStatus,
                                             &xpsCorrectorInfo->KP, 
                                             &xpsCorrectorInfo->KI,
                                             &xpsCorrectorInfo->KD,
                                             &xpsCorrectorInfo->KS,
                                             &xpsCorrectorInfo->IntegrationTime,
                                             &xpsCorrectorInfo->DerivativeFilterCutOffFrequency,
                                             &xpsCorrectorInfo->GKP,
                                             &xpsCorrectorInfo->GKI,
                                             &xpsCorrectorInfo->GKD,
                                             &xpsCorrectorInfo->KForm,
                                             &xpsCorrectorInfo->FeedForwardGainVelocity);
}


/**
 * Wrapper function for PositionerCorrectorPIDFFAccelerationGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIDFFAccelerationGetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIDFFAccelerationGet(pAxis->pollSocket,
                                                 pAxis->positionerName,
                                                 &xpsCorrectorInfo->ClosedLoopStatus,
                                                 &xpsCorrectorInfo->KP, 
                                                 &xpsCorrectorInfo->KI,
                                                 &xpsCorrectorInfo->KD,
                                                 &xpsCorrectorInfo->KS,
                                                 &xpsCorrectorInfo->IntegrationTime,
                                                 &xpsCorrectorInfo->DerivativeFilterCutOffFrequency,
                                                 &xpsCorrectorInfo->GKP,
                                                 &xpsCorrectorInfo->GKI,
                                                 &xpsCorrectorInfo->GKD,
                                                 &xpsCorrectorInfo->KForm,
                                                 &xpsCorrectorInfo->FeedForwardGainAcceleration);
}


/**
 * Wrapper function for PositionerCorrectorPIDDualFFVoltageGet.
 * It will set parameters in a AXIS_HDL struct.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIDDualFFVoltageGetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIDDualFFVoltageGet(pAxis->pollSocket,
                                                pAxis->positionerName,
                                                &xpsCorrectorInfo->ClosedLoopStatus,
                                                &xpsCorrectorInfo->KP, 
                                                &xpsCorrectorInfo->KI,
                                                &xpsCorrectorInfo->KD,
                                                &xpsCorrectorInfo->KS,
                                                &xpsCorrectorInfo->IntegrationTime,
                                                &xpsCorrectorInfo->DerivativeFilterCutOffFrequency,
                                                &xpsCorrectorInfo->GKP,
                                                &xpsCorrectorInfo->GKI,
                                                &xpsCorrectorInfo->GKD,
                                                &xpsCorrectorInfo->KForm,
                                                &xpsCorrectorInfo->FeedForwardGainVelocity,
                                                &xpsCorrectorInfo->FeedForwardGainAcceleration,
                                                &xpsCorrectorInfo->Friction);
}


/**
 * Wrapper function for PositionerCorrectorPIPositionSet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIPositionSetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIPositionSet(pAxis->pollSocket,
                                          pAxis->positionerName,
                                          xpsCorrectorInfo->ClosedLoopStatus,
                                          xpsCorrectorInfo->KP, 
                                          xpsCorrectorInfo->KI, 
                                          xpsCorrectorInfo->IntegrationTime);
}


/**
 * Wrapper function for PositionerCorrectorPIDFFVelocitySet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIDFFVelocitySetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIDFFVelocitySet(pAxis->pollSocket,
                                             pAxis->positionerName,
                                             xpsCorrectorInfo->ClosedLoopStatus,
                                             xpsCorrectorInfo->KP, 
                                             xpsCorrectorInfo->KI,
                                             xpsCorrectorInfo->KD,
                                             xpsCorrectorInfo->KS,
                                             xpsCorrectorInfo->IntegrationTime,
                                             xpsCorrectorInfo->DerivativeFilterCutOffFrequency,
                                             xpsCorrectorInfo->GKP,
                                             xpsCorrectorInfo->GKI,
                                             xpsCorrectorInfo->GKD,
                                             xpsCorrectorInfo->KForm,
                                             xpsCorrectorInfo->FeedForwardGainVelocity);
}

/**
 * Wrapper function for PositionerCorrectorPIDFFAccelerationSet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIDFFAccelerationSetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIDFFAccelerationSet(pAxis->pollSocket,
                                                 pAxis->positionerName,
                                                 xpsCorrectorInfo->ClosedLoopStatus,
                                                 xpsCorrectorInfo->KP, 
                                                 xpsCorrectorInfo->KI,
                                                 xpsCorrectorInfo->KD,
                                                 xpsCorrectorInfo->KS,
                                                 xpsCorrectorInfo->IntegrationTime,
                                                 xpsCorrectorInfo->DerivativeFilterCutOffFrequency,
                                                 xpsCorrectorInfo->GKP,
                                                 xpsCorrectorInfo->GKI,
                                                 xpsCorrectorInfo->GKD,
                                                 xpsCorrectorInfo->KForm,
                                                 xpsCorrectorInfo->FeedForwardGainAcceleration);
}

/**
 * Wrapper function for PositionerCorrectorPIDDualFFVoltageSet.
 * @param pAxis Axis struct AXIS_HDL.
 * @return Return value from XPS function.
 */
static int PositionerCorrectorPIDDualFFVoltageSetWrapper(AXIS_HDL pAxis)
{
  xpsCorrectorInfo_t *xpsCorrectorInfo = &pAxis->xpsCorrectorInfo;
  return PositionerCorrectorPIDDualFFVoltageSet(pAxis->pollSocket,
                                                pAxis->positionerName,
                                                xpsCorrectorInfo->ClosedLoopStatus,
                                                xpsCorrectorInfo->KP, 
                                                xpsCorrectorInfo->KI,
                                                xpsCorrectorInfo->KD,
                                                xpsCorrectorInfo->KS,
                                                xpsCorrectorInfo->IntegrationTime,
                                                xpsCorrectorInfo->DerivativeFilterCutOffFrequency,
                                                xpsCorrectorInfo->GKP,
                                                xpsCorrectorInfo->GKI,
                                                xpsCorrectorInfo->GKD,
                                                xpsCorrectorInfo->KForm,
                                                xpsCorrectorInfo->FeedForwardGainVelocity,
                                                xpsCorrectorInfo->FeedForwardGainAcceleration,
                                                xpsCorrectorInfo->Friction);
}


/**
 * Function to move an axis to roughly the home position.
 * It first does a kill, followed by a referencing start/stop 
 * sequence on an group. Then uses the hardware status to
 * determine which direction to move.
 * The distance to move is set when enabling this functionality
 * (it is disabled by default, because it only applies to
 * stages with home switches in the middle of travel).
 *
 */
static int movePositionerToHome(AXIS_HDL pAxis)
{
  int status = 0;
  int axis = 0;
  int groupStatus = 0;
  int initialHardwareStatus = 0;
  int hardwareStatus = 0;
  double defaultDistance = 0;
  double vel=0;
  double accel=0;
  double minJerk=0;
  double maxJerk=0;

  XPSController *pController = NULL;
  AXIS_HDL pTempAxis = NULL;
  pController = pAxis->pController;

  if (pAxis->referencing_mode_move == 0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, "[%d,%d]: This function has not been enabled.\n", 
          pAxis->card, pAxis->axis);
    return MOTOR_AXIS_ERROR;
  }

  defaultDistance = (double) pAxis->referencing_mode_move;
  
  /*NOTE: the XPS has some race conditions in its firmware. That's why I placed some
    epicsThreadSleep calls between the XPS functions below.*/

  /* The XPS won't allow a home command if the group is in the Ready state
     * If the group is Ready, then make it not Ready  */
  status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
  if (groupStatus >= 10 && groupStatus <= 18) {
    status = GroupKill(pAxis->moveSocket, pAxis->groupName);
  }
  epicsThreadSleep(0.05);
  status = GroupInitialize(pAxis->pollSocket, pAxis->groupName);
  if (status) {
    PRINT(pAxis->logParam, MOTOR_ERROR, "movePositionerToHome[%d,%d]: error calling GroupInitialize\n",
          pAxis->card, pAxis->axis);
    return MOTOR_AXIS_ERROR;
  }
  epicsThreadSleep(0.05);
  status = GroupReferencingStart(pAxis->moveSocket, pAxis->groupName);
  epicsThreadSleep(0.05);
  status = GroupReferencingStop(pAxis->moveSocket, pAxis->groupName);
  epicsThreadSleep(0.05);

  status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
  if (groupStatus != 11) {
    PRINT(pAxis->logParam, MOTOR_ERROR, "movePositionerToHome[%d,%d]: error putting axis into referencing mode\n",
          pAxis->card, pAxis->axis);
    return MOTOR_AXIS_ERROR;
  }
  
  /* Find out if any axes are in the same group, and set referencing mode for them all.*/
  for (axis=0; axis<pController->numAxes; axis++) {
    pTempAxis = &pController->pAxis[axis];
    if (strcmp(pAxis->groupName, pTempAxis->groupName) == 0) {
      pTempAxis->referencing_mode = 1;
    }  
  }
  
  /*Set status bits correctly*/
  if (epicsMutexLock(pAxis->mutexId) == epicsMutexLockOK) {
    motorParam->setInteger(pAxis->params, motorAxisHomed, 0);
    motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 0);  
    motorParam->callCallback(pAxis->params);
    epicsMutexUnlock(pAxis->mutexId);
  }
  

  /*Read which side of the home switch we are on.*/
  status = PositionerHardwareStatusGet(pAxis->pollSocket, pAxis->positionerName, &initialHardwareStatus);
  if (status) {
    PRINT(pAxis->logParam, MOTOR_ERROR, "movePositionerToHome[%d,%d]: error calling PositionerHardwareStatusGet\n", 
          pAxis->card, pAxis->axis);
    return MOTOR_AXIS_ERROR;
  }
  
  if (!(XPSC8_ZM_HIGH_LEVEL & initialHardwareStatus)) {
    defaultDistance = defaultDistance * -1.0;
  }

  /*I want to set a slow speed here, so as not to move at default (max) speed. The user must have chance to
    stop things if it looks like it has past the home switch and is not stopping. First I need to read what is currently
    set for velocity, and then I divide it by 2.*/
  status = PositionerSGammaParametersGet(pAxis->pollSocket,
                                         pAxis->positionerName, 
                                         &vel, &accel, &minJerk, &maxJerk);
  if (status != 0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing PositionerSGammaParametersGet[%d,%d].\n",
          pAxis->card, pAxis->axis);
    GroupKill(pAxis->moveSocket, pAxis->groupName);
    return MOTOR_AXIS_ERROR;
  }
  status = PositionerSGammaParametersSet(pAxis->pollSocket,
                                         pAxis->positionerName, 
                                         (vel/2), accel, minJerk, maxJerk);
  if (status != 0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing PositionerSGammaParametersSet[%d,%d].\n",
          pAxis->card, pAxis->axis);
    GroupKill(pAxis->moveSocket, pAxis->groupName);
    return MOTOR_AXIS_ERROR;
  }
  epicsThreadSleep(0.05);
  
  /*Move in direction of home switch.*/
  status = GroupMoveRelative(pAxis->moveSocket, pAxis->positionerName, 1,
                             &defaultDistance); 
  if (status != 0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupMoveRelative axis=%s status=%d\n", \
          pAxis->positionerName, status);
    /*Issue a kill here if we have failed to move.*/
    status = GroupKill(pAxis->moveSocket, pAxis->groupName);
    return MOTOR_AXIS_ERROR;
  }
  
  epicsThreadSleep(0.1);

  status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
  
  if (groupStatus == 44) {
    while (1) {
      epicsThreadSleep(0.2);
      status = PositionerHardwareStatusGet(pAxis->pollSocket, pAxis->positionerName, &hardwareStatus);
      if (hardwareStatus != initialHardwareStatus) {
        break;
      }
      status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
      if (groupStatus != 44) {
        /* move finished for some other reason.*/
        PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupMoveRelative axis=%s status=%d\n", \
              pAxis->positionerName, status);
        /*Issue a kill here if we have failed to move.*/
        status = GroupKill(pAxis->moveSocket, pAxis->groupName);
        return MOTOR_AXIS_ERROR;
      }
    }
  } else {
    PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupMoveRelative axis=%s status=%d\n", \
          pAxis->positionerName, status);
    /*Issue a kill here if we have failed to move.*/
    status = GroupKill(pAxis->moveSocket, pAxis->groupName);
    return MOTOR_AXIS_ERROR;
  }

  status = GroupMoveAbort(pAxis->pollSocket, pAxis->groupName);
  if (status != 0) {
    PRINT(pAxis->logParam, MOTOR_ERROR, " Error performing GroupMoveAbort axis=%s status=%d\n",        \
          pAxis->positionerName, status);
    /*This should really have worked. Do a kill instead.*/
    status = GroupKill(pAxis->moveSocket, pAxis->groupName);
    return MOTOR_AXIS_ERROR;
  }

  return status;
}


/**
 * Function to enable the movePositionerToHome function on a per axis.
 * It also sets the distance to try to move by for an axis.
 */
void XPSEnableMoveToHome(int card, const char * positionerName, int distance)
{

  XPSController *pController = NULL;
  AXIS_HDL pAxis = NULL;
  int axisIndex = 0;

  if (distance<=0) {
    printf("Error in XPSEnableMoveToHome. distance must be positive.\n");
    
  } else {

    /*Get the axis referenence.*/
    pController = &pXPSController[card];
    
    for (axisIndex=0; axisIndex<pController->numAxes; ++axisIndex) {
      pAxis = &(pController->pAxis[axisIndex]);
      if (!strcmp(positionerName, pAxis->positionerName)) {
              pAxis->referencing_mode_move = distance;
        break;
      }
    } 
    
  }
  
}






/* Code for iocsh registration */

/* Newport XPS Gathering Test */
static const iocshArg XPSGatheringArg0 = {"Element Period*10^4", iocshArgInt};
static const iocshArg * const XPSGatheringArgs[1] = {&XPSGatheringArg0};
static const iocshFuncDef XPSC8GatheringTest = {"xps_gathering", 1, XPSGatheringArgs};
static void XPSC8GatheringTestCallFunc(const iocshArgBuf *args)
{
    xps_gathering(args[0].ival);
}

/* XPS tcl execute function */
static const iocshArg tclcallArg0 = {"tcl name", iocshArgString};
static const iocshArg tclcallArg1 = {"Task name", iocshArgString};
static const iocshArg tclcallArg2 = {"Function args", iocshArgString};
static const iocshArg * const tclcallArgs[3] = {&tclcallArg0,
                                                &tclcallArg1,
                                                &tclcallArg2};
static const iocshFuncDef TCLRun = {"tclcall", 3, tclcallArgs};
static void TCLRunCallFunc(const  iocshArgBuf *args)
{
    tclcall(args[0].sval, args[1].sval, args[2].sval);
}


/* XPSSetup */
static const iocshArg XPSSetupArg0 = {"Number of XPS controllers", iocshArgInt};
static const iocshArg * const XPSSetupArgs[1] =  {&XPSSetupArg0};
static const iocshFuncDef setupXPS = {"XPSSetup", 1, XPSSetupArgs};
static void setupXPSCallFunc(const iocshArgBuf *args)
{
    XPSSetup(args[0].ival);
}


/* XPSConfig */
static const iocshArg XPSConfigArg0 = {"Card being configured", iocshArgInt};
static const iocshArg XPSConfigArg1 = {"IP", iocshArgString};
static const iocshArg XPSConfigArg2 = {"Port", iocshArgInt};
static const iocshArg XPSConfigArg3 = {"Number of Axes", iocshArgInt};
static const iocshArg XPSConfigArg4 = {"Moving poll rate", iocshArgInt};
static const iocshArg XPSConfigArg5 = {"Idle poll rate", iocshArgInt};
static const iocshArg * const XPSConfigArgs[6] = {&XPSConfigArg0,
                                                  &XPSConfigArg1,
                                                  &XPSConfigArg2,
                                                  &XPSConfigArg3,
                                                  &XPSConfigArg4,
                                                  &XPSConfigArg5};
static const iocshFuncDef configXPS = {"XPSConfig", 6, XPSConfigArgs};
static void configXPSCallFunc(const iocshArgBuf *args)
{
    XPSConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival,
              args[4].ival, args[5].ival);
}


/* XPSConfigAxis */
static const iocshArg XPSConfigAxisArg0 = {"Card number", iocshArgInt};
static const iocshArg XPSConfigAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg XPSConfigAxisArg2 = {"Axis name", iocshArgString};
static const iocshArg XPSConfigAxisArg3 = {"Steps per unit", iocshArgString};
static const iocshArg XPSConfigAxisArg4 = {"No Disabled Error", iocshArgInt};
static const iocshArg * const XPSConfigAxisArgs[5] = {&XPSConfigAxisArg0,
                                                      &XPSConfigAxisArg1,
                                                      &XPSConfigAxisArg2,
                                                      &XPSConfigAxisArg3,
                                                      &XPSConfigAxisArg4};
                                                      
static const iocshFuncDef configXPSAxis = {"XPSConfigAxis", 5, XPSConfigAxisArgs};

static void configXPSAxisCallFunc(const iocshArgBuf *args)
{
    XPSConfigAxis(args[0].ival, args[1].ival, args[2].sval, args[3].sval, args[4].ival);
}


/* void XPSEnableSetPosition(int setPos) */
static const iocshArg XPSEnableSetPositionArg0 = {"Set Position Flag", iocshArgInt};
static const iocshArg * const XPSEnableSetPositionArgs[1] = {&XPSEnableSetPositionArg0};
static const iocshFuncDef xpsEnableSetPosition = {"XPSEnableSetPosition", 1, XPSEnableSetPositionArgs};
static void xpsEnableSetPositionCallFunc(const iocshArgBuf *args)
{
    XPSEnableSetPosition(args[0].ival);
}

/* void XPSSetPosSleepTime(int posSleep) */
static const iocshArg XPSSetPosSleepTimeArg0 = {"Set Position Sleep Time", iocshArgInt};
static const iocshArg * const XPSSetPosSleepTimeArgs[1] = {&XPSSetPosSleepTimeArg0};
static const iocshFuncDef xpsSetPosSleepTime = {"XPSSetPosSleepTime", 1, XPSSetPosSleepTimeArgs};
static void xpsSetPosSleepTimeCallFunc(const iocshArgBuf *args)
{
    XPSSetPosSleepTime(args[0].ival);
}

/* void XPSDisablePoll(int posSleep) */
static const iocshArg XPSDisablePollArg0 = {"Set disablePoll value", iocshArgInt};
static const iocshArg * const XPSDisablePollArgs[1] = {&XPSDisablePollArg0};
static const iocshFuncDef xpsDisablePoll = {"XPSDisablePoll", 1, XPSDisablePollArgs};
static void xpsDisablePollCallFunc(const iocshArgBuf *args)
{
    XPSDisablePoll(args[0].ival);
}

/* void XPSEnableMoveToHome(int card, const char * positionerName, int distance) */
static const iocshArg XPSEnableMoveToHomeArg0 = {"Card number", iocshArgInt};
static const iocshArg XPSEnableMoveToHomeArg1 = {"Axis name", iocshArgString};
static const iocshArg XPSEnableMoveToHomeArg2 = {"Distance", iocshArgInt};
static const iocshArg * const XPSEnableMoveToHomeArgs[3] = {&XPSEnableMoveToHomeArg0,
                                                            &XPSEnableMoveToHomeArg1,
                                                            &XPSEnableMoveToHomeArg2};
static const iocshFuncDef xpsEnableMoveToHome = {"XPSEnableMoveToHome", 3, XPSEnableMoveToHomeArgs};
static void xpsEnableMoveToHomeCallFunc(const iocshArgBuf *args)
{
  XPSEnableMoveToHome(args[0].ival, args[1].sval, args[2].ival);
}

static void XPSRegister(void)
{

    iocshRegister(&setupXPS,      setupXPSCallFunc);
    iocshRegister(&configXPS,     configXPSCallFunc);
    iocshRegister(&configXPSAxis, configXPSAxisCallFunc);
    iocshRegister(&xpsEnableSetPosition, xpsEnableSetPositionCallFunc);
    iocshRegister(&xpsSetPosSleepTime, xpsSetPosSleepTimeCallFunc);
    iocshRegister(&xpsDisablePoll, xpsDisablePollCallFunc);
    iocshRegister(&TCLRun,        TCLRunCallFunc);
    iocshRegister(&XPSC8GatheringTest, XPSC8GatheringTestCallFunc);
    iocshRegister(&xpsEnableMoveToHome, xpsEnableMoveToHomeCallFunc);
}

epicsExportRegistrar(XPSRegister);

