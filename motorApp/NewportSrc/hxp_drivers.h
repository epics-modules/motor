/*************************************************
 *   XPS_API.h                                   *
 *                                               *
 *   Description:                                *
 *       XPS functions                           *
 *************************************************/

#ifdef _WIN32
    #ifndef DLL
        #ifdef _DLL  /* _DLL is defined by EPICS if we are being compiled to call DLLs */
            #define DLL _declspec(dllimport)
        #else
            #define DLL
        #endif
    #endif
#else
    #define DLL 
    #define __stdcall
#endif
#ifdef __cplusplus
extern "C"
{
#else
#typedef int bool;  /* C does not know bool, only C++ */
#endif


DLL int __stdcall HXPTCP_ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut); 
DLL void __stdcall HXPTCP_SetTimeout(int SocketIndex, double Timeout); 
DLL void __stdcall HXPTCP_CloseSocket(int SocketIndex); 
DLL char * __stdcall HXPTCP_GetError(int SocketIndex); 
DLL char * __stdcall HXPGetLibraryVersion(void); 
DLL int __stdcall HXPControllerMotionKernelTimeLoadGet (int SocketIndex, double * CPUTotalLoadRatio, double * CPUCorrectorLoadRatio, double * CPUProfilerLoadRatio, double * CPUServitudesLoadRatio);  /* Get controller motion kernel time load */
DLL int __stdcall HXPElapsedTimeGet (int SocketIndex, double * ElapsedTime);  /* Return elapsed time from controller power on */
DLL int __stdcall HXPErrorStringGet (int SocketIndex, int ErrorCode, char * ErrorString);  /* Return the error string corresponding to the error code */
DLL int __stdcall HXPFirmwareVersionGet (int SocketIndex, char * Version);  /* Return firmware version */
DLL int __stdcall HXPTCLScriptExecute (int SocketIndex, char * TCLFileName, char * TaskName, char * ParametersList);  /* Execute a TCL script from a TCL file */
DLL int __stdcall HXPTCLScriptExecuteAndWait (int SocketIndex, char * TCLFileName, char * TaskName, char * InputParametersList, char * OutputParametersList);  /* Execute a TCL script from a TCL file and wait the end of execution to return */
DLL int __stdcall HXPTCLScriptKill (int SocketIndex, char * TaskName);  /* Kill TCL Task */
DLL int __stdcall HXPTimerGet (int SocketIndex, char * TimerName, int * FrequencyTicks);  /* Get a timer */
DLL int __stdcall HXPTimerSet (int SocketIndex, char * TimerName, int FrequencyTicks);  /* Set a timer */
DLL int __stdcall HXPReboot (int SocketIndex);  /* Reboot the controller */
DLL int __stdcall HXPLogin (int SocketIndex, char * Name, char * Password);  /* Log in */
DLL int __stdcall HXPCloseAllOtherSockets (int SocketIndex);  /* Close all socket beside the one used to send this command */
DLL int __stdcall HXPEventAdd (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter, char * ActionName, char * ActionParameter1, char * ActionParameter2, char * ActionParameter3);  /* ** OBSOLETE ** Add an event */
DLL int __stdcall HXPEventGet (int SocketIndex, char * PositionerName, char * EventsAndActionsList);  /* ** OBSOLETE ** Read events and actions list */
DLL int __stdcall HXPEventRemove (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);  /* ** OBSOLETE ** Delete an event */
DLL int __stdcall HXPEventWait (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);  /* ** OBSOLETE ** Wait an event */
DLL int __stdcall HXPEventExtendedConfigurationTriggerSet (int SocketIndex, int NbElements, char * ExtendedEventNameList, char * EventParameter1List, char * EventParameter2List, char * EventParameter3List, char * EventParameter4List);  /* Configure one or several events */
DLL int __stdcall HXPEventExtendedConfigurationTriggerGet (int SocketIndex, char * EventTriggerConfiguration);  /* Read the event configuration */
DLL int __stdcall HXPEventExtendedConfigurationActionSet (int SocketIndex, int NbElements, char * ExtendedActionNameList, char * ActionParameter1List, char * ActionParameter2List, char * ActionParameter3List, char * ActionParameter4List);  /* Configure one or several actions */
DLL int __stdcall HXPEventExtendedConfigurationActionGet (int SocketIndex, char * ActionConfiguration);  /* Read the action configuration */
DLL int __stdcall HXPEventExtendedStart (int SocketIndex, int * ID);  /* Launch the last event and action configuration and return an ID */
DLL int __stdcall HXPEventExtendedAllGet (int SocketIndex, char * EventActionConfigurations);  /* Read all event and action configurations */
DLL int __stdcall HXPEventExtendedGet (int SocketIndex, int ID, char * EventTriggerConfiguration, char * ActionConfiguration);  /* Read the event and action configuration defined by ID */
DLL int __stdcall HXPEventExtendedRemove (int SocketIndex, int ID);  /* Remove the event and action configuration defined by ID */
DLL int __stdcall HXPEventExtendedWait (int SocketIndex);  /* Wait events from the last event configuration */
DLL int __stdcall HXPGatheringConfigurationGet (int SocketIndex, char * Type);  /* Read different mnemonique type */
DLL int __stdcall HXPGatheringConfigurationSet (int SocketIndex, int NbElements, char * TypeList);  /* Configuration acquisition */
DLL int __stdcall HXPGatheringCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);  /* Maximum number of samples and current number during acquisition */
DLL int __stdcall HXPGatheringStopAndSave (int SocketIndex);  /* Stop acquisition and save data */
DLL int __stdcall HXPGatheringDataAcquire (int SocketIndex);  /* Acquire a configured data */
DLL int __stdcall HXPGatheringDataGet (int SocketIndex, int IndexPoint, char * DataBufferLine);  /* Get a data line from gathering buffer */
DLL int __stdcall HXPGatheringReset (int SocketIndex);  /* Empty the gathered data in memory to start new gathering from scratch */
DLL int __stdcall HXPGatheringRun (int SocketIndex, int DataNumber, int Divisor);  /* Start a new gathering */
DLL int __stdcall HXPGatheringStop (int SocketIndex);  /* Stop the data gathering (without saving to file) */
DLL int __stdcall HXPGatheringExternalConfigurationSet (int SocketIndex, int NbElements, char * TypeList);  /* Configuration acquisition */
DLL int __stdcall HXPGatheringExternalConfigurationGet (int SocketIndex, char * Type);  /* Read different mnemonique type */
DLL int __stdcall HXPGatheringExternalCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);  /* Maximum number of samples and current number during acquisition */
DLL int __stdcall HXPGatheringExternalStopAndSave (int SocketIndex);  /* Stop acquisition and save data */
DLL int __stdcall HXPGlobalArrayGet (int SocketIndex, int Number, char * ValueString);  /* Get global array value */
DLL int __stdcall HXPGlobalArraySet (int SocketIndex, int Number, char * ValueString);  /* Set global array value */
DLL int __stdcall HXPDoubleGlobalArrayGet (int SocketIndex, int Number, double * DoubleValue);  /* Get double global array value */
DLL int __stdcall HXPDoubleGlobalArraySet (int SocketIndex, int Number, double DoubleValue);  /* Set double global array value */
DLL int __stdcall HXPGPIOAnalogGet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogValue[]);  /* Read analog input or analog output for one or few input */
DLL int __stdcall HXPGPIOAnalogSet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogOutputValue[]);  /* Set analog output for one or few output */
DLL int __stdcall HXPGPIOAnalogGainGet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);  /* Read analog input gain (1, 2, 4 or 8) for one or few input */
DLL int __stdcall HXPGPIOAnalogGainSet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);  /* Set analog input gain (1, 2, 4 or 8) for one or few input */
DLL int __stdcall HXPGPIODigitalGet (int SocketIndex, char * GPIOName, unsigned short * DigitalValue);  /* Read digital output or digital input  */
DLL int __stdcall HXPGPIODigitalSet (int SocketIndex, char * GPIOName, unsigned short Mask, unsigned short DigitalOutputValue);  /* Set Digital Output for one or few output TTL */
DLL int __stdcall HXPGroupCorrectorOutputGet (int SocketIndex, char * GroupName, int NbElements, double CorrectorOutput[]);  /* Return corrector outputs */
DLL int __stdcall HXPGroupHomeSearch (int SocketIndex, char * GroupName);  /* Start home search sequence */
DLL int __stdcall HXPGroupHomeSearchAndRelativeMove (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);  /* Start home search sequence and execute a displacement */
DLL int __stdcall HXPGroupInitialize (int SocketIndex, char * GroupName);  /* Start the initialization */
DLL int __stdcall HXPGroupInitializeWithEncoderCalibration (int SocketIndex, char * GroupName);  /* Start the initialization with encoder calibration */
DLL int __stdcall HXPGroupKill (int SocketIndex, char * GroupName);  /* Kill the group */
DLL int __stdcall HXPGroupMoveAbort (int SocketIndex, char * GroupName);  /* Abort a move */
DLL int __stdcall HXPGroupMoveAbsolute (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);  /* Do an absolute move */
DLL int __stdcall HXPGroupMoveRelative (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);  /* Do a relative move */
DLL int __stdcall HXPGroupMotionDisable (int SocketIndex, char * GroupName);  /* Set Motion disable on selected group */
DLL int __stdcall HXPGroupMotionEnable (int SocketIndex, char * GroupName);  /* Set Motion enable on selected group */
DLL int __stdcall HXPGroupPositionCorrectedProfilerGet (int SocketIndex, char * GroupName, double PositionX, double PositionY, double * CorrectedProfilerPositionX, double * CorrectedProfilerPositionY);  /* Return corrected profiler positions */
DLL int __stdcall HXPGroupPositionCurrentGet (int SocketIndex, char * GroupName, int NbElements, double CurrentEncoderPosition[]);  /* Return current positions */
DLL int __stdcall HXPGroupPositionSetpointGet (int SocketIndex, char * GroupName, int NbElements, double SetPointPosition[]);  /* Return setpoint positions */
DLL int __stdcall HXPGroupPositionTargetGet (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);  /* Return target positions */
DLL int __stdcall HXPGroupStatusGet (int SocketIndex, char * GroupName, int * Status);  /* Return group status */
DLL int __stdcall HXPGroupStatusStringGet (int SocketIndex, int GroupStatusCode, char * GroupStatusString);  /* Return the group status string corresponding to the group status code */
DLL int __stdcall HXPKillAll (int SocketIndex);  /* Put all groups in 'Not initialized' state */
DLL int __stdcall HXPRestartApplication (int SocketIndex);  /* Restart the Controller */
DLL int __stdcall HXPPositionerBacklashGet (int SocketIndex, char * PositionerName, double * BacklashValue, char * BacklaskStatus);  /* Read backlash value and status */
DLL int __stdcall HXPPositionerBacklashSet (int SocketIndex, char * PositionerName, double BacklashValue);  /* Set backlash value */
DLL int __stdcall HXPPositionerBacklashEnable (int SocketIndex, char * PositionerName);  /* Enable the backlash */
DLL int __stdcall HXPPositionerBacklashDisable (int SocketIndex, char * PositionerName);  /* Disable the backlash */
DLL int __stdcall HXPPositionerCorrectorNotchFiltersSet (int SocketIndex, char * PositionerName, double NotchFrequency1, double NotchBandwith1, double NotchGain1, double NotchFrequency2, double NotchBandwith2, double NotchGain2);  /* Update filters parameters  */
DLL int __stdcall HXPPositionerCorrectorNotchFiltersGet (int SocketIndex, char * PositionerName, double * NotchFrequency1, double * NotchBandwith1, double * NotchGain1, double * NotchFrequency2, double * NotchBandwith2, double * NotchGain2);  /* Read filters parameters  */
DLL int __stdcall HXPPositionerCorrectorPIDFFAccelerationSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainAcceleration);  /* Update corrector parameters */
DLL int __stdcall HXPPositionerCorrectorPIDFFAccelerationGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainAcceleration);  /* Read corrector parameters */
DLL int __stdcall HXPPositionerCorrectorPIDFFVelocitySet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity);  /* Update corrector parameters */
DLL int __stdcall HXPPositionerCorrectorPIDFFVelocityGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity);  /* Read corrector parameters */
DLL int __stdcall HXPPositionerCorrectorPIDDualFFVoltageSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity, double FeedForwardGainAcceleration, double Friction);  /* Update corrector parameters */
DLL int __stdcall HXPPositionerCorrectorPIDDualFFVoltageGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity, double * FeedForwardGainAcceleration, double * Friction);  /* Read corrector parameters */
DLL int __stdcall HXPPositionerCorrectorPIPositionSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double IntegrationTime);  /* Update corrector parameters */
DLL int __stdcall HXPPositionerCorrectorPIPositionGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * IntegrationTime);  /* Read corrector parameters */
DLL int __stdcall HXPPositionerCorrectorTypeGet (int SocketIndex, char * PositionerName, char * CorrectorType);  /* Read corrector type */
DLL int __stdcall HXPPositionerCurrentVelocityAccelerationFiltersSet (int SocketIndex, char * PositionerName, double CurrentVelocityCutOffFrequency, double CurrentAccelerationCutOffFrequency);  /* Set current velocity and acceleration cut off frequencies */
DLL int __stdcall HXPPositionerCurrentVelocityAccelerationFiltersGet (int SocketIndex, char * PositionerName, double * CurrentVelocityCutOffFrequency, double * CurrentAccelerationCutOffFrequency);  /* Get current velocity and acceleration cut off frequencies */
DLL int __stdcall HXPPositionerDriverStatusGet (int SocketIndex, char * PositionerName, int * DriverStatus);  /* Read positioner driver status */
DLL int __stdcall HXPPositionerDriverStatusStringGet (int SocketIndex, int PositionerDriverStatus, char * PositionerDriverStatusString);  /* Return the positioner driver status string corresponding to the positioner error code */
DLL int __stdcall HXPPositionerEncoderAmplitudeValuesGet (int SocketIndex, char * PositionerName, double * CalibrationSinusAmplitude, double * CurrentSinusAmplitude, double * CalibrationCosinusAmplitude, double * CurrentCosinusAmplitude);  /* Read analog interpolated encoder amplitude values */
DLL int __stdcall HXPPositionerEncoderCalibrationParametersGet (int SocketIndex, char * PositionerName, double * SinusOffset, double * CosinusOffset, double * DifferentialGain, double * PhaseCompensation);  /* Read analog interpolated encoder calibration parameters */
DLL int __stdcall HXPPositionerErrorGet (int SocketIndex, char * PositionerName, int * ErrorCode);  /* Read and clear positioner error code */
DLL int __stdcall HXPPositionerErrorRead (int SocketIndex, char * PositionerName, int * ErrorCode);  /* Read only positioner error code without clear it */
DLL int __stdcall HXPPositionerErrorStringGet (int SocketIndex, int PositionerErrorCode, char * PositionerErrorString);  /* Return the positioner status string corresponding to the positioner error code */
DLL int __stdcall HXPPositionerHardwareStatusGet (int SocketIndex, char * PositionerName, int * HardwareStatus);  /* Read positioner hardware status */
DLL int __stdcall HXPPositionerHardwareStatusStringGet (int SocketIndex, int PositionerHardwareStatus, char * PositionerHardwareStatusString);  /* Return the positioner hardware status string corresponding to the positioner error code */
DLL int __stdcall HXPPositionerHardInterpolatorFactorGet (int SocketIndex, char * PositionerName, int * InterpolationFactor);  /* Get hard interpolator parameters */
DLL int __stdcall HXPPositionerHardInterpolatorFactorSet (int SocketIndex, char * PositionerName, int InterpolationFactor);  /* Set hard interpolator parameters */
DLL int __stdcall HXPPositionerMaximumVelocityAndAccelerationGet (int SocketIndex, char * PositionerName, double * MaximumVelocity, double * MaximumAcceleration);  /* Return maximum velocity and acceleration of the positioner */
DLL int __stdcall HXPPositionerMotionDoneGet (int SocketIndex, char * PositionerName, double * PositionWindow, double * VelocityWindow, double * CheckingTime, double * MeanPeriod, double * TimeOut);  /* Read motion done parameters */
DLL int __stdcall HXPPositionerMotionDoneSet (int SocketIndex, char * PositionerName, double PositionWindow, double VelocityWindow, double CheckingTime, double MeanPeriod, double TimeOut);  /* Update motion done parameters */
DLL int __stdcall HXPPositionerSGammaExactVelocityAjustedDisplacementGet (int SocketIndex, char * PositionerName, double DesiredDisplacement, double * AdjustedDisplacement);  /* Return adjusted displacement to get exact velocity */
DLL int __stdcall HXPPositionerSGammaParametersGet (int SocketIndex, char * PositionerName, double * Velocity, double * Acceleration, double * MinimumTjerkTime, double * MaximumTjerkTime);  /* Read dynamic parameters for one axe of a group for a future displacement  */
DLL int __stdcall HXPPositionerSGammaParametersSet (int SocketIndex, char * PositionerName, double Velocity, double Acceleration, double MinimumTjerkTime, double MaximumTjerkTime);  /* Update dynamic parameters for one axe of a group for a future displacement */
DLL int __stdcall HXPPositionerSGammaPreviousMotionTimesGet (int SocketIndex, char * PositionerName, double * SettingTime, double * SettlingTime);  /* Read SettingTime and SettlingTime */
DLL int __stdcall HXPPositionerStageParameterGet (int SocketIndex, char * PositionerName, char * ParameterName, char * ParameterValue);  /* Return the stage parameter */
DLL int __stdcall HXPPositionerStageParameterSet (int SocketIndex, char * PositionerName, char * ParameterName, char * ParameterValue);  /* Save the stage parameter */
DLL int __stdcall HXPPositionerUserTravelLimitsGet (int SocketIndex, char * PositionerName, double * UserMinimumTarget, double * UserMaximumTarget);  /* Read UserMinimumTarget and UserMaximumTarget */
DLL int __stdcall HXPPositionerUserTravelLimitsSet (int SocketIndex, char * PositionerName, double UserMinimumTarget, double UserMaximumTarget);  /* Update UserMinimumTarget and UserMaximumTarget */
DLL int __stdcall HXPHexapodMoveAbsolute (int SocketIndex, char * GroupName, char * CoordinateSystem, double X, double Y, double Z, double U, double V, double W);  /* Hexapod absolute move in a specific coordinate system */
DLL int __stdcall HXPHexapodMoveIncremental (int SocketIndex, char * GroupName, char * CoordinateSystem, double dX, double dY, double dZ, double dU, double dV, double dW);  /* Hexapod incremental move in a specific coordinate system */
DLL int __stdcall HXPHexapodCoordinatesGet (int SocketIndex, char * GroupName, char * CoordinateSystemIn, char * CoordinateSystemOut, double Xin, double Yin, double Zin, double Uin, double Vin, double Win, double * Xout, double * Yout, double * Zout, double * Uout, double * Vout, double * Wout);  /* Get coordinates in a specific coordinate system of a point specified in another coordinate system */
DLL int __stdcall HXPHexapodCoordinateSystemSet (int SocketIndex, char * GroupName, char * CoordinateSystem, double X, double Y, double Z, double U, double V, double W);  /* Modify the position of a coordinate system */
DLL int __stdcall HXPHexapodCoordinateSystemGet (int SocketIndex, char * GroupName, char * CoordinateSystem, double * X, double * Y, double * Z, double * U, double * V, double * W);  /* Get the position of a coordinate system */
DLL int __stdcall HXPOptionalModuleExecute (int SocketIndex, char * ModuleFileName, char * TaskName);  /* Execute an optional module */
DLL int __stdcall HXPOptionalModuleKill (int SocketIndex, char * TaskName);  /* Kill an optional module */
DLL int __stdcall HXPControllerStatusGet (int SocketIndex, int * ControllerStatus);  /* Read controller current status */
DLL int __stdcall HXPControllerStatusStringGet (int SocketIndex, int ControllerStatusCode, char * ControllerStatusString);  /* Return the controller status string corresponding to the controller status code */
DLL int __stdcall HXPEEPROMCIESet (int SocketIndex, int CardNumber, char * ReferenceString);  /* Set CIE EEPROM reference string */
DLL int __stdcall HXPEEPROMDACOffsetCIESet (int SocketIndex, int PlugNumber, double DAC1Offset, double DAC2Offset);  /* Set CIE DAC offsets */
DLL int __stdcall HXPEEPROMDriverSet (int SocketIndex, int PlugNumber, char * ReferenceString);  /* Set Driver EEPROM reference string */
DLL int __stdcall HXPEEPROMINTSet (int SocketIndex, int CardNumber, char * ReferenceString);  /* Set INT EEPROM reference string */
DLL int __stdcall HXPCPUCoreAndBoardSupplyVoltagesGet (int SocketIndex, double * VoltageCPUCore, double * SupplyVoltage1P5V, double * SupplyVoltage3P3V, double * SupplyVoltage5V, double * SupplyVoltage12V, double * SupplyVoltageM12V, double * SupplyVoltageM5V, double * SupplyVoltage5VSB);  /* Get power informations */
DLL int __stdcall HXPCPUTemperatureAndFanSpeedGet (int SocketIndex, double * CPUTemperature, double * CPUFanSpeed);  /* Get CPU temperature and fan speed */
DLL int __stdcall HXPActionListGet (int SocketIndex, char * ActionList);  /* Action list */
DLL int __stdcall HXPActionExtendedListGet (int SocketIndex, char * ActionList);  /* Action extended list */
DLL int __stdcall HXPAPIExtendedListGet (int SocketIndex, char * Method);  /* API method list */
DLL int __stdcall HXPAPIListGet (int SocketIndex, char * Method);  /* API method list without extended API */
DLL int __stdcall HXPErrorListGet (int SocketIndex, char * ErrorsList);  /* Error list */
DLL int __stdcall HXPEventListGet (int SocketIndex, char * EventList);  /* General event list */
DLL int __stdcall HXPGatheringListGet (int SocketIndex, char * list);  /* Gathering type list */
DLL int __stdcall HXPGatheringExtendedListGet (int SocketIndex, char * list);  /* Gathering type extended list */
DLL int __stdcall HXPGatheringExternalListGet (int SocketIndex, char * list);  /* External Gathering type list */
DLL int __stdcall HXPGroupStatusListGet (int SocketIndex, char * GroupStatusList);  /* Group status list */
DLL int __stdcall HXPHardwareInternalListGet (int SocketIndex, char * InternalHardwareList);  /* Internal hardware list */
DLL int __stdcall HXPHardwareDriverAndStageGet (int SocketIndex, int PlugNumber, char * DriverName, char * StageName);  /* Smart hardware */
DLL int __stdcall HXPObjectsListGet (int SocketIndex, char * ObjectsList);  /* Group name and positioner name */
DLL int __stdcall HXPPositionerErrorListGet (int SocketIndex, char * PositionerErrorList);  /* Positioner error list */
DLL int __stdcall HXPPositionerHardwareStatusListGet (int SocketIndex, char * PositionerHardwareStatusList);  /* Positioner hardware status list */
DLL int __stdcall HXPPositionerDriverStatusListGet (int SocketIndex, char * PositionerDriverStatusList);  /* Positioner driver status list */
DLL int __stdcall HXPReferencingActionListGet (int SocketIndex, char * list);  /* Get referencing action list */
DLL int __stdcall HXPReferencingSensorListGet (int SocketIndex, char * list);  /* Get referencing sensor list */
DLL int __stdcall HXPGatheringUserDatasGet (int SocketIndex, double * UserData1, double * UserData2, double * UserData3, double * UserData4, double * UserData5, double * UserData6, double * UserData7, double * UserData8);  /* Return UserDatas values */
DLL int __stdcall HXPControllerMotionKernelPeriodMinMaxGet (int SocketIndex, double * MinimumCorrectorPeriod, double * MaximumCorrectorPeriod, double * MinimumProfilerPeriod, double * MaximumProfilerPeriod, double * MinimumServitudesPeriod, double * MaximumServitudesPeriod);  /* Get controller motion kernel min/max periods */
DLL int __stdcall HXPControllerMotionKernelPeriodMinMaxReset (int SocketIndex);  /* Reset controller motion kernel min/max periods */
DLL int __stdcall HXPTestTCP (int SocketIndex, char * InputString, char * ReturnString);  /* Test TCP/IP transfert */
DLL int __stdcall HXPPrepareForUpdate (int SocketIndex);  /* Kill QNX processes for firmware update */


#ifdef __cplusplus
}
#endif
