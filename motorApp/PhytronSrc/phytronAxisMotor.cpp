/*
FILENAME... phytronAxisMotor.cpp
USAGE...    Motor driver support for Phytron Axis controller.

Tom Slejko & Bor Marolt
Cosylab d.d. 2014
 
Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#include <drvAsynIPPort.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <cantProceed.h>

#include <asynOctetSyncIO.h>

#include "phytronAxisMotor.h"
#include <epicsExport.h>

using namespace std;

#ifndef ASYN_TRACE_WARNING
#define ASYN_TRACE_WARNING ASYN_TRACE_ERROR
#endif

//Used for casting position doubles to integers
#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/*
 * Contains phytronController instances, phytronCreateAxis uses it to find and
 * bind axis object to the correct controller object.
 */
static vector<phytronController*> controllers;

/** Creates a new phytronController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPort that was created previously to connect to the phytron controller
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
phytronController::phytronController(const char *phytronPortName, const char *asynPortName,
                                 double movingPollPeriod, double idlePollPeriod, double timeout)
  :  asynMotorController(phytronPortName,
                         0xFF,
                         NUM_PHYTRON_PARAMS,
                         0, //No additional interfaces beyond those in base class
                         0, //No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)// Default priority and stack size
{
  asynStatus status;
  size_t response_len;
  phytronStatus phyStatus;
  static const char *functionName = "phytronController::phytronController";

  //Timeout is defined in milliseconds, but sendPhytronCommand expects seconds
  timeout_ = timeout/1000;

  //pyhtronCreateAxis uses portName to identify the controller
  this->controllerName_ = (char *) mallocMustSucceed(sizeof(char)*(strlen(portName)+1),
      "phytronController::phytronController: Controller name memory allocation failed.\n");

  strcpy(this->controllerName_, portName);

  //Create Controller parameters
  createParam(controllerStatusString,     asynParamInt32, &this->controllerStatus_);
  createParam(controllerStatusResetString,asynParamInt32, &this->controllerStatusReset_);
  createParam(resetControllerString,      asynParamInt32, &this->resetController_);

  //Create Axis parameters
  createParam(axisStatusResetString,      asynParamInt32, &this->axisStatusReset_);
  createParam(axisResetString,            asynParamInt32, &this->axisReset_);
  createParam(axisStatusString,           asynParamInt32, &this->axisStatus_);
  createParam(homingProcedureString,      asynParamInt32, &this->homingProcedure_);
  createParam(axisModeString,             asynParamInt32, &this->axisMode_);
  createParam(mopOffsetPosString,         asynParamInt32, &this->mopOffsetPos_);
  createParam(mopOffsetNegString,         asynParamInt32, &this->mopOffsetNeg_);
  createParam(stepResolutionString,       asynParamInt32, &this->stepResolution_);
  createParam(stopCurrentString,          asynParamInt32, &this->stopCurrent_);
  createParam(runCurrentString,           asynParamInt32, &this->runCurrent_);
  createParam(boostCurrentString,         asynParamInt32, &this->boostCurrent_);
  createParam(encoderTypeString,          asynParamInt32, &this->encoderType_);
  createParam(initRecoveryTimeString,     asynParamInt32, &this->initRecoveryTime_);
  createParam(positionRecoveryTimeString, asynParamInt32, &this->positionRecoveryTime_);
  createParam(boostConditionString,       asynParamInt32, &this->boost_);
  createParam(encoderRateString,          asynParamInt32, &this->encoderRate_);
  createParam(switchTypString,            asynParamInt32, &this->switchTyp_);
  createParam(pwrStageModeString,         asynParamInt32, &this->pwrStageMode_);
  createParam(encoderResolutionString,    asynParamInt32, &this->encoderRes_);
  createParam(encoderFunctionString,      asynParamInt32, &this->encoderFunc_);
  createParam(encoderSFIWidthString,      asynParamInt32, &this->encoderSFIWidth_);
  createParam(encoderDirectionString,     asynParamInt32, &this->encoderDirection_);
  createParam(powerStagetMonitorString,   asynParamInt32, &this->powerStageMonitor_);
  createParam(currentDelayTimeString,     asynParamInt32, &this->currentDelayTime_);
  createParam(powerStageTempString,       asynParamFloat64, &this->powerStageTemp_);
  createParam(motorTempString,            asynParamFloat64, &this->motorTemp_);


  /* Connect to phytron controller */
  status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
      "%s: cannot connect to phytron controller\n",
      functionName);
  } else {
    //phytronCreateAxis will search for the controller for axis registration
    controllers.push_back(this);

    //RESET THE CONTROLLER
    sprintf(this->outString_, "CR");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
            "phytronController::phytronController: Could not reset controller %s\n", this->controllerName_);
    }

    //Wait for reset to finish
    epicsThreadSleep(10.0);

    startPoller(movingPollPeriod, idlePollPeriod, 5);
  }

}

/** Creates a new phytronController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPPort that was created previously to connect to the phytron controller
  * \param[in] numController     number of axes that this controller supports is numController*AXES_PER_CONTROLLER
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int phytronCreateController(const char *phytronPortName, const char *asynPortName,
                                   int movingPollPeriod, int idlePollPeriod, double timeout)
{
  phytronController *pphytronController = new phytronController(phytronPortName, asynPortName, movingPollPeriod/1000., idlePollPeriod/1000., timeout);
  pphytronController = NULL;
  return asynSuccess;
}

/** asynUsers use this to read integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus phytronController::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus;

  //Call base implementation first
  asynPortDriver::readInt32(pasynUser, value);

  //Check if this is a call to read a controller parameter
  if(pasynUser->reason == resetController_ || pasynUser->reason == controllerStatusReset_){
    //Called only on initialization of bo records RESET and RESET-STATUS
    return asynSuccess;
  } else if (pasynUser->reason == controllerStatus_){
    size_t response_len;
    sprintf(this->outString_, "ST");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "phytronAxis::readInt32: Reading controller %s status failed with error "
          "code: %d\n", this->controllerName_, phyStatus);
      return phyToAsyn(phyStatus);
    }

    *value = atoi(this->inString_);
    return asynSuccess;
  }

  //This is an axis request, find the axis
  pAxis = getAxis(pasynUser);
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::readInt32: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  if(pasynUser->reason == homingProcedure_){
    getIntegerParam(pAxis->axisNo_, homingProcedure_, value);
    return asynSuccess;
  } else if (pasynUser->reason == axisReset_ || pasynUser->reason == axisStatusReset_){
    //Called only on initialization of AXIS-RESET and AXIS-STATUS-RESET bo records
    return asynSuccess;
  } else if (pasynUser->reason == axisMode_){
    sprintf(this->outString_, "M%.1fP01R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == mopOffsetPos_){
    sprintf(this->outString_, "M%.1fP11R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == mopOffsetNeg_){
    sprintf(this->outString_, "M%.1fP12R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == stepResolution_){
    sprintf(this->outString_, "M%.1fP45R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == stopCurrent_){
    sprintf(this->outString_, "M%.1fP40R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == runCurrent_){
    sprintf(this->outString_, "M%.1fP41R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == boostCurrent_){
    sprintf(this->outString_, "M%.1fP42R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == encoderType_){
    sprintf(this->outString_, "M%.1fP34R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == initRecoveryTime_){
    sprintf(this->outString_, "M%.1fP13R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == positionRecoveryTime_){
    sprintf(this->outString_, "M%.1fP16R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == boost_){
    sprintf(this->outString_, "M%.1fP17R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderRate_){
    sprintf(this->outString_, "M%.1fP26R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == switchTyp_){
    sprintf(this->outString_, "M%.1fP27R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == pwrStageMode_){
    sprintf(this->outString_, "M%.1fP28R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderRes_){
    sprintf(this->outString_, "M%.1fP35R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderFunc_){
    sprintf(this->outString_, "M%.1fP36R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%.1fP37R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderDirection_){
    sprintf(this->outString_, "M%.1fP38R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == currentDelayTime_){
    sprintf(this->outString_, "M%.1fP43R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == powerStageMonitor_){
    sprintf(this->outString_, "M%.1fP53R", pAxis->axisModuleNo_);
  }


  phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &pAxis->response_len);
  if(phyStatus){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::readInt32: Failed with status %d for reason %d\n", phyStatus, pasynUser->reason);
    return phyToAsyn(phyStatus);
  }

  *value = atoi(this->inString_);

  //{STOP,RUN,BOOST} current records have EGU set to mA, but device returns 10mA
  if(pasynUser->reason == stopCurrent_ || pasynUser->reason == runCurrent_ ||
      pasynUser->reason == boostCurrent_)
  {
    *value *= 10;
  } // else if


  return asynSuccess;
}

/** asynUsers use this to write integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[in] value       Parameter value to be written
 */
asynStatus phytronController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus;

  //Call base implementation first
  asynMotorController::writeInt32(pasynUser, value);

  /*
   * Check if this is a call to reset the controller, else it is an axis request
   */
  if(pasynUser->reason == resetController_){
    size_t response_len;
    sprintf(this->outString_, "CR");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "phytronAxis::writeInt32: Reseting controller %s failed with error code: %d\n", this->controllerName_, phyStatus);
    }
    resetAxisEncoderRatio();
    return phyToAsyn(phyStatus);
  } else if(pasynUser->reason == controllerStatusReset_){
    size_t response_len;
    sprintf(this->outString_, "STC");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
     asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
         "phytronAxis::writeInt32: Reseting controller %s failed with error code: %d\n", this->controllerName_, phyStatus);
    }
    return phyToAsyn(phyStatus);
  }
  /*
   * This is an axis request, find the axis
   */
  pAxis = getAxis(pasynUser);
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::writeInt32: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  if(pasynUser->reason == homingProcedure_){
    setIntegerParam(pAxis->axisNo_, pasynUser->reason, value);
    callParamCallbacks();
    return asynSuccess;
  } else if(pasynUser->reason == axisReset_){
    sprintf(this->outString_, "M%.1fC", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisStatusReset_){
    sprintf(this->outString_, "SEC%.1f", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisMode_){
    sprintf(this->outString_, "M%.1fP01=%d", pAxis->axisModuleNo_,value);
  } else if(pasynUser->reason == mopOffsetPos_){
    sprintf(this->outString_, "M%.1fP11=%d", pAxis->axisModuleNo_,value);
  } else if(pasynUser->reason == mopOffsetNeg_){
    sprintf(this->outString_, "M%.1fP12=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == stepResolution_){
    sprintf(this->outString_, "M%.1fP45=%d", pAxis->axisModuleNo_,value);
  }  else if (pasynUser->reason == stopCurrent_){
    value /= 10; //STOP_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%.1fP40=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == runCurrent_){
    value /= 10; //RUN_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%.1fP41=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == boostCurrent_){
    value /= 10; //BOOST_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%.1fP42=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == encoderType_){
    sprintf(this->outString_, "M%.1fP34=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == initRecoveryTime_){
    sprintf(this->outString_, "M%.1fP13=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == positionRecoveryTime_){
    sprintf(this->outString_, "M%.1fP16=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == boost_){
    sprintf(this->outString_, "M%.1fP17=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderRate_){
    sprintf(this->outString_, "M%.1fP26=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == switchTyp_){
    sprintf(this->outString_, "M%.1fP27=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == pwrStageMode_){
    sprintf(this->outString_, "M%.1fP28=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderRes_){
    sprintf(this->outString_, "M%.1fP35=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderFunc_){
    //Value is VAL field of parameter P37 record. If P37 is positive P36 is set to 1, else 0
    sprintf(this->outString_, "M%.1fP36=%d", pAxis->axisModuleNo_, value > 0 ? 1 : 0);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%.1fP37=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%.1fP38=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == powerStageMonitor_){
    sprintf(this->outString_, "M%.1fP53=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == currentDelayTime_){
    sprintf(this->outString_, "M%.1fP43=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == encoderDirection_){
    sprintf(this->outString_, "M%.1fP38=%d", pAxis->axisModuleNo_, value);
  }

  phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &pAxis->response_len);
  if(phyStatus){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::writeInt32: Failed with status %d for reason %d\n", phyStatus, pasynUser->reason);
    return phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** asynUsers use this to read float parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus phytronController::readFloat64(asynUser *pasynUser, epicsFloat64 *value){
  phytronAxis   *pAxis;
  phytronStatus phyStatus;

  pAxis = getAxis(pasynUser);
  if(!pAxis){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::readFloat64: Axis not found on the controller %s\n", this->controllerName_);
    return asynError;
  }

  //Call base implementation first
  asynPortDriver::readFloat64(pasynUser, value);

  if(pasynUser->reason == powerStageTemp_){
    sprintf(this->outString_, "M%.1fP49R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == motorTemp_){
    sprintf(this->outString_, "M%.1fP54R", pAxis->axisModuleNo_);
  }

  phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &pAxis->response_len);
  if(phyStatus){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::readFloat64: Failed with status %d for reason %d\n", phyStatus, pasynUser->reason);
    return phyToAsyn(phyStatus);
  }

  *value = atof(this->inString_);

  //Power stage and motor temperature records have EGU °C, but device returns 0.1 °C
  *value /= 10;

  return phyToAsyn(phyStatus);

}
/*
 * Reset the motorEncoderRatio to 1 after the reset of MCM unit
 */
void phytronController::resetAxisEncoderRatio(){

  for(uint32_t i = 0; i < axes.size(); i++){
    setDoubleParam(axes[i]->axisNo_, motorEncoderRatio_, 1);
  }
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void phytronController::report(FILE *fp, int level)
{
  fprintf(fp, "MCB-4B motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an phytronAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number.
  */
phytronAxis* phytronController::getAxis(asynUser *pasynUser)
{
  return static_cast<phytronAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an phytronAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number.
  */
phytronAxis* phytronController::getAxis(int axisNo)
{
  return static_cast<phytronAxis*>(asynMotorController::getAxis(axisNo));
}

/**
 * @brief implements phytron specific data fromat
 * @param output
 * @param input
 * @param maxChars
 * @param nread
 * @param timeout
 * @return
 */
phytronStatus phytronController::sendPhytronCommand(const char *command, char *response_buffer, size_t response_max_len, size_t *nread)
{
    char buffer[255];
    char* buffer_end=buffer;
    static const char *functionName = "phytronController::sendPhytronCommand";

    *(buffer_end++)=0x02;                               //STX
    *(buffer_end++)='0';                                //Module address TODO: add class member
    buffer_end += sprintf(buffer_end,"%s",command);     //Append command
    *(buffer_end++)=0x3a;                               //Append separator

    buffer_end += sprintf(buffer_end,"%c%c",'X','X');   //XX disables checksum
    *(buffer_end++)=0x03;                               //Append ETX
    *(buffer_end)=0x0;                                  //Null terminate message for saftey

    phytronStatus status = (phytronStatus) writeReadController(buffer,buffer,255,nread, timeout_);
    if(status){
        return status;
    }

    char* nack_ack = strchr(buffer,0x02); //Find STX
    if(!nack_ack){
        nread=0;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "%s: Communication failed\n",
          functionName);
        return phytronInvalidReturn;
    }
    nack_ack++; //NACK/ACK is one
    //ACK, extract response
    if(*nack_ack==0x06){
        char* separator = strchr(nack_ack,0x3a);          //find separator

        /* Copy data from nack_ack to
         * separator into buffer */
        uint32_t len = separator-nack_ack-1;              //calculate length of message
        if(len > response_max_len) len=response_max_len;

        memcpy(response_buffer,nack_ack+1,len);           //copy payload to destination
        response_buffer[separator-nack_ack-1]=0;          //Add NULL terminator

        *nread=strlen(response_buffer);
    }
    //NAK return error
    else if(*nack_ack==0x15){
        nread=0;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "%s: Nack sent by the controller\n",
          functionName);
        return phytronInvalidCommand;
    }

    return status;

}

/** Castst phytronStatus to asynStatus enumeration
 * \param[in] phyStatus
 */
asynStatus phytronController::phyToAsyn(phytronStatus phyStatus){
  if(phyStatus == phytronInvalidReturn || phyStatus == phytronInvalidCommand) return asynError;
  return (asynStatus) phyStatus;
}


//******************************************************************************
//                   PHYTRON AXIS IMPLEMENTATION
//******************************************************************************

/** Creates a new phytronAxis object.
  * Configuration command, called directly or from iocsh
  * \param[in] controllerName    Name of the asyn port created by calling phytronCreateController from st.cmd
  * \param[in] module            Index of the I1AM01 module controlling this axis
  * \param[in] axis              Axis index
  */
extern "C" int phytronCreateAxis(const char* controllerName, int module, int axis){

  phytronAxis *pAxis;

  //Find the controller
  uint32_t i;
  for(i = 0; i < controllers.size(); i++){
    if(!strcmp(controllers[i]->controllerName_, controllerName)) {
      pAxis = new phytronAxis(controllers[i], module*10 + axis);
      controllers[i]->axes.push_back(pAxis);
      break;
    }
  }

  //If controller is not found, report error
  if(i == controllers.size()){
    printf("ERROR: phytronCreateAxis: Controller %s is not registered\n", controllerName);
    return asynError;
  }

  return asynSuccess;
}

/** Creates a new phytronAxis object.
  * \param[in] pC Pointer to the phytronController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
phytronAxis::phytronAxis(phytronController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    axisModuleNo_((float)axisNo/10),
    pC_(pC),
    response_len(0)
{

  //Controller always supports encoder. Encoder enable/disable is set through UEIP
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);

  setDoubleParam(pC_->motorEncoderRatio_, 1);

}


/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void phytronAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

/** Sets velocity parameters before the move is executed. Controller produces a
 * trapezoidal speed profile defined by these parmeters.
 * \param[in] minVelocity   Start velocity
 * \param[in] maxVelocity   Maximum velocity
 * \param[in] moveType      Type of movement determines which controller speed parameters are set
 */
phytronStatus phytronAxis::setVelocity(double minVelocity, double maxVelocity, int moveType)
{

  phytronStatus maxStatus = phytronSuccess;
  phytronStatus minStatus = phytronSuccess;
  maxVelocity = fabs(maxVelocity);
  minVelocity = fabs(minVelocity);

  if(maxVelocity > MAX_VELOCITY){
    maxVelocity = MAX_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to high, setting to"
              "maximum velocity: %d!\n", axisNo_, maxVelocity, MAX_VELOCITY);
  } else if (maxVelocity < MIN_VELOCITY){
    maxVelocity = MIN_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to low, setting to"
              "minimum velocity: %d!\n", axisNo_, maxVelocity, MIN_VELOCITY);
  }

  if(minVelocity > MAX_VELOCITY){
    minVelocity = MAX_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to high, setting to"
              "maximum velocity: %d!\n", axisNo_, maxVelocity, MAX_VELOCITY);
  } else if (minVelocity < MIN_VELOCITY){
    minVelocity = MIN_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to low, setting to"
              "minimum velocity: %d!\n", axisNo_, minVelocity, MIN_VELOCITY);
  }


  if(moveType == stdMove){
    //Set maximum velocity (P14)
    sprintf(pC_->outString_, "M%.1fP14=%f", axisModuleNo_, maxVelocity);
    maxStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);

    //Set minimum velocity (P04)
    sprintf(pC_->outString_, "M%.1fP04=%f", axisModuleNo_, minVelocity);
    minStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  } else if (moveType == homeMove){
    //Set maximum velocity (P08)
    sprintf(pC_->outString_, "M%.1fP08=%f", axisModuleNo_, maxVelocity);
    maxStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);

    //Set minimum velocity (P10)
    sprintf(pC_->outString_, "M%.1fP10=%f", axisModuleNo_, minVelocity);
    minStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  }

  return (maxStatus > minStatus) ? maxStatus : minStatus;
}

/** Sets acceleration parameters before the move is executed.
 * \param[in] acceleration  Acceleration to be used in the move
 * \param[in] moveType      Type of movement determines which controller acceleration parameters is set
 */
phytronStatus phytronAxis::setAcceleration(double acceleration, int moveType)
{
  if(acceleration > MAX_ACCELERATION){
    acceleration = MAX_ACCELERATION;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setAcceleration: Failed for axis %d - Acceleration %f is to high, "
              "setting to maximum acceleration: %d!\n", axisNo_, acceleration, MAX_ACCELERATION);
  } else if(acceleration < MIN_ACCELERATION){
    acceleration = MIN_ACCELERATION;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setAcceleration: Failed for axis %d - Acceleration %f is to low, "
              "setting to minimum acceleration: %d!\n", axisNo_, acceleration, MIN_ACCELERATION);
  }

  if (moveType == stdMove){
    sprintf(pC_->outString_, "M%.1fP15=%f", axisModuleNo_, acceleration);
  } else if(moveType == homeMove){
    sprintf(pC_->outString_, "M%.1fP09=%f", axisModuleNo_, acceleration);
  } else if (moveType == stopMove){
    sprintf(pC_->outString_, "M%.1fP07=%f", axisModuleNo_, acceleration);
  }

  return pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
}

/** Execute the move.
 * \param[in] position      Target position (relative or absolute).
 * \param[in] relative      Is the move absolute or relative
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus phytronAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  phytronStatus phyStatus;

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setVelocity(minVelocity, maxVelocity, stdMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::move: Setting the velocity for axis %d to %f failed with error "
              "code: %d!\n", axisNo_, maxVelocity, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setAcceleration(acceleration, stdMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::move: Setting the acceleration for axis %d to %f failed with "
              "error code: %d!\n", axisNo_, acceleration, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  if (relative) {
    sprintf(pC_->outString_, "M%.1f%c%d", axisModuleNo_, position>0 ? '+':'-', abs(NINT(position)));
  } else {
    sprintf(pC_->outString_, "M%.1fA%d", axisModuleNo_, NINT(position));
  }

  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::move: Moving axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** Execute the homing procedure
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 * \param[in] forwards      Direction of homing move
 */
asynStatus phytronAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  phytronStatus phyStatus;
  int           homingType;

  pC_->getIntegerParam(axisNo_, pC_->homingProcedure_, &homingType);

  phyStatus =  setVelocity(minVelocity, maxVelocity, homeMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
        "phytronAxis::home: Setting the velocity for axis %d to %f failed with error "
        "code: %d!\n", axisNo_, maxVelocity, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  phyStatus =  setAcceleration(acceleration, homeMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
        "phytronAxis::home: Setting the acceleration for axis %d to %f failed with "
        "error code: %d!\n", axisNo_, acceleration, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  if(forwards){
    if(homingType == limit) sprintf(pC_->outString_, "M%.1fR+", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "M%.1fR+C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "M%.1fR+I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "M%.1fR+^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "M%.1fR+C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "M%.1fRC+", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "M%.1fRC+^I", axisModuleNo_);
  } else {
    if(homingType == limit) sprintf(pC_->outString_, "M%.1fR-", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "M%.1fR-C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "M%.1fR-I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "M%.1fR-^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "M%.1fR-C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "M%.1fRC-", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "M%.1fRC-^I", axisModuleNo_);
  }

  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::home: Moving axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** Jog the motor. Direction is determined by sign of the maxVelocity profile
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus phytronAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  phytronStatus phyStatus;

  phyStatus = setVelocity(minVelocity, maxVelocity, stdMove);
  if(phyStatus){

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::moveVelocity: Setting the velocity for axis %d to %f failed with error "
             "code: %d!\n", axisNo_, maxVelocity, phyStatus);
  }

  phyStatus = setAcceleration(acceleration, stdMove);
  if(phyStatus){
   asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::moveVelocity: Setting the acceleration for axis %d to %f failed with "
             "error code: %d!\n", axisNo_, acceleration, phyStatus);
  }

  if(maxVelocity < 0) {
    sprintf(pC_->outString_, "M%.1fL-", axisModuleNo_);
  } else {
    sprintf(pC_->outString_, "M%.1fL+", axisModuleNo_);
  }

  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::moveVelocity: Moving axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** Stop the motor
 * \param[in] acceleration  Deceleration to be used
 */
asynStatus phytronAxis::stop(double acceleration)
{
  phytronStatus phyStatus;

  phyStatus = setAcceleration(acceleration, stopMove);

  if(phyStatus){
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
            "phytronAxis::stop: Setting the acceleration for axis %d to %f failed with "
            "error code: %d!\n", axisNo_, acceleration, phyStatus);
  }

  sprintf(pC_->outString_, "M%.1fS", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::stop: Stopping axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

//NOTE: Use this for step-slip check?
asynStatus phytronAxis::setEncoderRatio(double ratio){

  phytronStatus phyStatus;

  sprintf(pC_->outString_, "M%.1fP39=%f", axisModuleNo_, 1/ratio);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::setEncoderRatio: Failed for axis %d with status %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }


  return asynSuccess;
}

//NOTE: Keep this for step-slip check?
asynStatus phytronAxis::setEncoderPosition(double position){


  return asynError;
}

/** Set the new position of the motor on the controller
 * \param[in] position  New absolute motor position
 */
asynStatus phytronAxis::setPosition(double position)
{
  phytronStatus phyStatus = phytronSuccess;

  sprintf(pC_->outString_, "M%.1fP20=%f", axisModuleNo_, position);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::setPosition: Setting position %f on axis %d failed with error code: %d!\n", position, axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status,
  * and the drive power-on status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false).
  */
asynStatus phytronAxis::poll(bool *moving)
{
  int axisStatus;
  double position;
  double encoderPosition;
  double encoderRatio;
  phytronStatus phyStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "M%.1fP20R", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::poll: Reading axis position failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  position = atof(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, position);

  // Read the current encoder value
  sprintf(pC_->outString_, "M%.1fP22R", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::poll: Reading encoder value failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  encoderPosition = atof(pC_->inString_);

  /*
   * The encoder position returned by the controller is weighted by the controller
   * resolutio. To get absolute encoder position, the received position must be
   * multiplied by the encoder resolution.
   */
  pC_->getDoubleParam(axisNo_, pC_->motorEncoderRatio_, &encoderRatio);
  setDoubleParam(pC_->motorEncoderPosition_, encoderPosition*encoderRatio);

  // Read the moving status of this motor
  sprintf(pC_->outString_, "M%.1f==H", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::poll: Reading axis moving status failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  *moving = (pC_->inString_[0] == 'E') ? 0:1;
  setIntegerParam(pC_->motorStatusDone_, !*moving);

  sprintf(pC_->outString_, "M%.1fSE", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::poll: Reading axis status failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  axisStatus = atoi(pC_->inString_);
  setIntegerParam(pC_->motorStatusHighLimit_, (axisStatus & 0x10)/0x10);
  setIntegerParam(pC_->motorStatusLowLimit_, (axisStatus & 0x20)/0x20);
  setIntegerParam(pC_->motorStatusAtHome_, (axisStatus & 0x40)/0x40);

  setIntegerParam(pC_->motorStatusHomed_, (axisStatus & 0x08)/0x08);
  setIntegerParam(pC_->motorStatusHome_, (axisStatus & 0x08)/0x08);

  setIntegerParam(pC_->motorStatusSlip_, (axisStatus & 0x4000)/0x4000);

  //Update the axis status record ($(P)$(M)_STATUS)
  setIntegerParam(pC_->axisStatus_, axisStatus);

  //No problem occurred
  setIntegerParam(pC_->motorStatusProblem_, 0);

  callParamCallbacks();
  return asynSuccess;
}

/** Parameters for iocsh phytron axis registration*/
static const iocshArg phytronCreateAxisArg0 = {"Controller Name", iocshArgString};
static const iocshArg phytronCreateAxisArg1 = {"Module index", iocshArgInt};
static const iocshArg phytronCreateAxisArg2 = {"Axis index", iocshArgInt};
static const iocshArg* const phytronCreateAxisArgs[] = {&phytronCreateAxisArg0,
                                                      &phytronCreateAxisArg1,
                                                      &phytronCreateAxisArg2};

/** Parameters for iocsh phytron controller registration */
static const iocshArg phytronCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg phytronCreateControllerArg1 = {"PhytronAxis port name", iocshArgString};
static const iocshArg phytronCreateControllerArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateControllerArg3 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateControllerArg4 = {"Idle poll period (ms)", iocshArgDouble};
static const iocshArg * const phytronCreateControllerArgs[] = {&phytronCreateControllerArg0,
                                                             &phytronCreateControllerArg1,
                                                             &phytronCreateControllerArg2,
                                                             &phytronCreateControllerArg3,
                                                             &phytronCreateControllerArg4};

static const iocshFuncDef phytronCreateAxisDef = {"phytronCreateAxis", 3, phytronCreateAxisArgs};
static const iocshFuncDef phytronCreateControllerDef = {"phytronCreateController", 5, phytronCreateControllerArgs};

static void phytronCreateControllerCallFunc(const iocshArgBuf *args)
{
  phytronCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].dval);
}

static void phytronCreateAxisCallFunc(const iocshArgBuf *args)
{
  phytronCreateAxis(args[0].sval, args[1].ival, args[2].ival);
}

static void phytronRegister(void)
{
  iocshRegister(&phytronCreateControllerDef, phytronCreateControllerCallFunc);
  iocshRegister(&phytronCreateAxisDef, phytronCreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(phytronRegister);
}
