/*************************************************
 *   XPS_API.h                                   *
 *                                               *
 *   Description:                                *
 *       XPS functions                           *
 *************************************************/

#define DLL epicsShareFunc

#if !defined(_WIN32) && !defined(CYGWIN32)
#define __stdcall
#endif

#ifdef __rtems__
    #include "strtok_r.h"
#endif

#ifdef __cplusplus
extern "C"
{
#else
typedef int bool;  /* C does not know bool, only C++ */
#endif


DLL int __stdcall TCP_ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut); 
DLL void __stdcall TCP_SetTimeout(int SocketIndex, double Timeout); 
DLL void __stdcall TCP_CloseSocket(int SocketIndex); 
DLL char * __stdcall TCP_GetError(int SocketIndex); 
DLL char * __stdcall GetLibraryVersion(void); 
DLL int __stdcall ControllerMotionKernelTimeLoadGet (int SocketIndex, double * CPUTotalLoadRatio, double * CPUCorrectorLoadRatio, double * CPUProfilerLoadRatio, double * CPUServitudesLoadRatio);  /* Get controller motion kernel time load */
DLL int __stdcall ControllerStatusGet (int SocketIndex, int * ControllerStatus);  /* Read controller current status */
DLL int __stdcall ControllerStatusStringGet (int SocketIndex, int ControllerStatusCode, char * ControllerStatusString);  /* Return the controller status string corresponding to the controller status code */
DLL int __stdcall ElapsedTimeGet (int SocketIndex, double * ElapsedTime);  /* Return elapsed time from controller power on */
DLL int __stdcall ErrorStringGet (int SocketIndex, int ErrorCode, char * ErrorString);  /* Return the error string corresponding to the error code */
DLL int __stdcall FirmwareVersionGet (int SocketIndex, char * Version);  /* Return firmware version */
DLL int __stdcall TCLScriptExecute (int SocketIndex, char * TCLFileName, char * TaskName, char * ParametersList);  /* Execute a TCL script from a TCL file */
DLL int __stdcall TCLScriptExecuteAndWait (int SocketIndex, char * TCLFileName, char * TaskName, char * InputParametersList, char * OutputParametersList);  /* Execute a TCL script from a TCL file and wait the end of execution to return */
DLL int __stdcall TCLScriptExecuteWithPriority (int SocketIndex, char * TCLFileName, char * TaskName, char * TaskPriorityLevel, char * ParametersList);  /* Execute a TCL script with defined priority */
DLL int __stdcall TCLScriptKill (int SocketIndex, char * TaskName);  /* Kill TCL Task */
DLL int __stdcall TimerGet (int SocketIndex, char * TimerName, int * FrequencyTicks);  /* Get a timer */
DLL int __stdcall TimerSet (int SocketIndex, char * TimerName, int FrequencyTicks);  /* Set a timer */
DLL int __stdcall Reboot (int SocketIndex);  /* Reboot the controller */
DLL int __stdcall Login (int SocketIndex, char * Name, char * Password);  /* Log in */
DLL int __stdcall CloseAllOtherSockets (int SocketIndex);  /* Close all socket beside the one used to send this command */
DLL int __stdcall HardwareDateAndTimeGet (int SocketIndex, char * DateAndTime);  /* Return hardware date and time */
DLL int __stdcall HardwareDateAndTimeSet (int SocketIndex, char * DateAndTime);  /* Set hardware date and time */
DLL int __stdcall EventAdd (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter, char * ActionName, char * ActionParameter1, char * ActionParameter2, char * ActionParameter3);  /* ** OBSOLETE ** Add an event */
DLL int __stdcall EventGet (int SocketIndex, char * PositionerName, char * EventsAndActionsList);  /* ** OBSOLETE ** Read events and actions list */
DLL int __stdcall EventRemove (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);  /* ** OBSOLETE ** Delete an event */
DLL int __stdcall EventWait (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);  /* ** OBSOLETE ** Wait an event */
DLL int __stdcall EventExtendedConfigurationTriggerSet (int SocketIndex, int NbElements, char * ExtendedEventNameList, char * EventParameter1List, char * EventParameter2List, char * EventParameter3List, char * EventParameter4List);  /* Configure one or several events */
DLL int __stdcall EventExtendedConfigurationTriggerGet (int SocketIndex, char * EventTriggerConfiguration);  /* Read the event configuration */
DLL int __stdcall EventExtendedConfigurationActionSet (int SocketIndex, int NbElements, char * ExtendedActionNameList, char * ActionParameter1List, char * ActionParameter2List, char * ActionParameter3List, char * ActionParameter4List);  /* Configure one or several actions */
DLL int __stdcall EventExtendedConfigurationActionGet (int SocketIndex, char * ActionConfiguration);  /* Read the action configuration */
DLL int __stdcall EventExtendedStart (int SocketIndex, int * ID);  /* Launch the last event and action configuration and return an ID */
DLL int __stdcall EventExtendedAllGet (int SocketIndex, char * EventActionConfigurations);  /* Read all event and action configurations */
DLL int __stdcall EventExtendedGet (int SocketIndex, int ID, char * EventTriggerConfiguration, char * ActionConfiguration);  /* Read the event and action configuration defined by ID */
DLL int __stdcall EventExtendedRemove (int SocketIndex, int ID);  /* Remove the event and action configuration defined by ID */
DLL int __stdcall EventExtendedWait (int SocketIndex);  /* Wait events from the last event configuration */
DLL int __stdcall GatheringConfigurationGet (int SocketIndex, char * Type);  /*Read different mnemonique type */
DLL int __stdcall GatheringConfigurationSet (int SocketIndex, int NbElements, char * TypeList);  /* Configuration acquisition */
DLL int __stdcall GatheringCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);  /* Maximum number of samples and current number during acquisition */
DLL int __stdcall GatheringStopAndSave (int SocketIndex);  /* Stop acquisition and save data */
DLL int __stdcall GatheringDataAcquire (int SocketIndex);  /* Acquire a configured data */
DLL int __stdcall GatheringDataGet (int SocketIndex, int IndexPoint, char * DataBufferLine);  /* Get a data line from gathering buffer */
DLL int __stdcall GatheringDataMultipleLinesGet (int SocketIndex, int IndexPoint, int NumberOfLines, char * DataBufferLine);  /* Get multiple data lines from gathering buffer */
DLL int __stdcall GatheringReset (int SocketIndex);  /* Empty the gathered data in memory to start new gathering from scratch */
DLL int __stdcall GatheringRun (int SocketIndex, int DataNumber, int Divisor);  /* Start a new gathering */
DLL int __stdcall GatheringRunAppend (int SocketIndex);  /* Re-start the stopped gathering to add new data */
DLL int __stdcall GatheringStop (int SocketIndex);  /* Stop the data gathering (without saving to file) */
DLL int __stdcall GatheringExternalConfigurationSet (int SocketIndex, int NbElements, char * TypeList);  /* Configuration acquisition */
DLL int __stdcall GatheringExternalConfigurationGet (int SocketIndex, char * Type);  /* Read different mnemonique type */
DLL int __stdcall GatheringExternalCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);  /* Maximum number of samples and current number during acquisition */
DLL int __stdcall GatheringExternalDataGet (int SocketIndex, int IndexPoint, char * DataBufferLine);  /* Get a data line from external gathering buffer */
DLL int __stdcall GatheringExternalStopAndSave (int SocketIndex);  /* Stop acquisition and save data */
DLL int __stdcall GlobalArrayGet (int SocketIndex, int Number, char * ValueString);  /* Get global array value */
DLL int __stdcall GlobalArraySet (int SocketIndex, int Number, char * ValueString);  /* Set global array value */
DLL int __stdcall DoubleGlobalArrayGet (int SocketIndex, int Number, double * DoubleValue);  /* Get double global array value */
DLL int __stdcall DoubleGlobalArraySet (int SocketIndex, int Number, double DoubleValue);  /* Set double global array value */
DLL int __stdcall GPIOAnalogGet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogValue[]);  /* Read analog input or analog output for one or few input */
DLL int __stdcall GPIOAnalogSet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogOutputValue[]);  /* Set analog output for one or few output */
DLL int __stdcall GPIOAnalogGainGet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);  /* Read analog input gain (1, 2, 4 or 8) for one or few input */
DLL int __stdcall GPIOAnalogGainSet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);  /* Set analog input gain (1, 2, 4 or 8) for one or few input */
DLL int __stdcall GPIODigitalGet (int SocketIndex, char * GPIOName, unsigned short * DigitalValue);  /* Read digital output or digital input  */
DLL int __stdcall GPIODigitalSet (int SocketIndex, char * GPIOName, unsigned short Mask, unsigned short DigitalOutputValue);  /* Set Digital Output for one or few output TTL */
DLL int __stdcall GroupAccelerationSetpointGet (int SocketIndex, char * GroupName, int NbElements, double SetpointAcceleration[]);  /* Return setpoint accelerations */
DLL int __stdcall GroupAnalogTrackingModeEnable (int SocketIndex, char * GroupName, char * Type);  /* Enable Analog Tracking mode on selected group */
DLL int __stdcall GroupAnalogTrackingModeDisable (int SocketIndex, char * GroupName);  /* Disable Analog Tracking mode on selected group */
DLL int __stdcall GroupCorrectorOutputGet (int SocketIndex, char * GroupName, int NbElements, double CorrectorOutput[]);  /* Return corrector outputs */
DLL int __stdcall GroupCurrentFollowingErrorGet (int SocketIndex, char * GroupName, int NbElements, double CurrentFollowingError[]);  /* Return current following errors */
DLL int __stdcall GroupHomeSearch (int SocketIndex, char * GroupName);  /* Start home search sequence */
DLL int __stdcall GroupHomeSearchAndRelativeMove (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);  /* Start home search sequence and execute a displacement */
DLL int __stdcall GroupInitialize (int SocketIndex, char * GroupName);  /* Start the initialization */
DLL int __stdcall GroupInitializeWithEncoderCalibration (int SocketIndex, char * GroupName);  /* Start the initialization with encoder calibration */
DLL int __stdcall GroupJogParametersSet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);  /* Modify Jog parameters on selected group and activate the continuous move */
DLL int __stdcall GroupJogParametersGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);  /* Get Jog parameters on selected group */
DLL int __stdcall GroupJogCurrentGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);  /* Get Jog current on selected group */
DLL int __stdcall GroupJogModeEnable (int SocketIndex, char * GroupName);  /* Enable Jog mode on selected group */
DLL int __stdcall GroupJogModeDisable (int SocketIndex, char * GroupName);  /* Disable Jog mode on selected group */
DLL int __stdcall GroupKill (int SocketIndex, char * GroupName);  /* Kill the group */
DLL int __stdcall GroupMoveAbort (int SocketIndex, char * GroupName);  /* Abort a move */
DLL int __stdcall GroupMoveAbsolute (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);  /* Do an absolute move */
DLL int __stdcall GroupMoveRelative (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);  /* Do a relative move */
DLL int __stdcall GroupMotionDisable (int SocketIndex, char * GroupName);  /* Set Motion disable on selected group */
DLL int __stdcall GroupMotionEnable (int SocketIndex, char * GroupName);  /* Set Motion enable on selected group */
DLL int __stdcall GroupPositionCorrectedProfilerGet (int SocketIndex, char * GroupName, double PositionX, double PositionY, double * CorrectedProfilerPositionX, double * CorrectedProfilerPositionY);  /* Return corrected profiler positions */
DLL int __stdcall GroupPositionCurrentGet (int SocketIndex, char * GroupName, int NbElements, double CurrentEncoderPosition[]);  /* Return current positions */
DLL int __stdcall GroupPositionPCORawEncoderGet (int SocketIndex, char * GroupName, double PositionX, double PositionY, double * PCORawPositionX, double * PCORawPositionY);  /* Return PCO raw encoder positions */
DLL int __stdcall GroupPositionSetpointGet (int SocketIndex, char * GroupName, int NbElements, double SetPointPosition[]);  /* Return setpoint positions */
DLL int __stdcall GroupPositionTargetGet (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);  /* Return target positions */
DLL int __stdcall GroupReferencingActionExecute (int SocketIndex, char * PositionerName, char * ReferencingAction, char * ReferencingSensor, double ReferencingParameter);  /* Execute an action in referencing mode */
DLL int __stdcall GroupReferencingStart (int SocketIndex, char * GroupName);  /* Enter referencing mode */
DLL int __stdcall GroupReferencingStop (int SocketIndex, char * GroupName);  /* Exit referencing mode */
DLL int __stdcall GroupStatusGet (int SocketIndex, char * GroupName, int * Status);  /* Return group status */
DLL int __stdcall GroupStatusStringGet (int SocketIndex, int GroupStatusCode, char * GroupStatusString);  /* Return the group status string corresponding to the group status code */
DLL int __stdcall GroupVelocityCurrentGet (int SocketIndex, char * GroupName, int NbElements, double CurrentVelocity[]);  /* Return current velocities */
DLL int __stdcall KillAll (int SocketIndex);  /* Put all groups in 'Not initialized' state */
DLL int __stdcall PositionerAnalogTrackingPositionParametersGet (int SocketIndex, char * PositionerName, char * GPIOName, double * Offset, double * Scale, double * Velocity, double * Acceleration);  /* Read dynamic parameters for one axe of a group for a future analog tracking position */
DLL int __stdcall PositionerAnalogTrackingPositionParametersSet (int SocketIndex, char * PositionerName, char * GPIOName, double Offset, double Scale, double Velocity, double Acceleration);  /* Update dynamic parameters for one axe of a group for a future analog tracking position */
DLL int __stdcall PositionerAnalogTrackingVelocityParametersGet (int SocketIndex, char * PositionerName, char * GPIOName, double * Offset, double * Scale, double * DeadBandThreshold, int * Order, double * Velocity, double * Acceleration);  /* Read dynamic parameters for one axe of a group for a future analog tracking velocity */
DLL int __stdcall PositionerAnalogTrackingVelocityParametersSet (int SocketIndex, char * PositionerName, char * GPIOName, double Offset, double Scale, double DeadBandThreshold, int Order, double Velocity, double Acceleration);  /* Update dynamic parameters for one axe of a group for a future analog tracking velocity */
DLL int __stdcall PositionerBacklashGet (int SocketIndex, char * PositionerName, double * BacklashValue, char * BacklaskStatus);  /* Read backlash value and status */
DLL int __stdcall PositionerBacklashSet (int SocketIndex, char * PositionerName, double BacklashValue);  /* Set backlash value */
DLL int __stdcall PositionerBacklashEnable (int SocketIndex, char * PositionerName);  /* Enable the backlash */
DLL int __stdcall PositionerBacklashDisable (int SocketIndex, char * PositionerName);  /* Disable the backlash */
DLL int __stdcall PositionerCorrectorNotchFiltersSet (int SocketIndex, char * PositionerName, double NotchFrequency1, double NotchBandwith1, double NotchGain1, double NotchFrequency2, double NotchBandwith2, double NotchGain2);  /* Update filters parameters  */
DLL int __stdcall PositionerCorrectorNotchFiltersGet (int SocketIndex, char * PositionerName, double * NotchFrequency1, double * NotchBandwith1, double * NotchGain1, double * NotchFrequency2, double * NotchBandwith2, double * NotchGain2);  /* Read filters parameters  */
DLL int __stdcall PositionerCorrectorPIDFFAccelerationSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainAcceleration);  /* Update corrector parameters */
DLL int __stdcall PositionerCorrectorPIDFFAccelerationGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainAcceleration);  /* Read corrector parameters */
DLL int __stdcall PositionerCorrectorPIDFFVelocitySet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity);  /* Update corrector parameters */
DLL int __stdcall PositionerCorrectorPIDFFVelocityGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity);  /* Read corrector parameters */
DLL int __stdcall PositionerCorrectorPIDDualFFVoltageSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity, double FeedForwardGainAcceleration, double Friction);  /* Update corrector parameters */
DLL int __stdcall PositionerCorrectorPIDDualFFVoltageGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity, double * FeedForwardGainAcceleration, double * Friction);  /* Read corrector parameters */
DLL int __stdcall PositionerCorrectorPIPositionSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double IntegrationTime);  /* Update corrector parameters */
DLL int __stdcall PositionerCorrectorPIPositionGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * IntegrationTime);  /* Read corrector parameters */
DLL int __stdcall PositionerCorrectorTypeGet (int SocketIndex, char * PositionerName, char * CorrectorType);  /* Read corrector type */
DLL int __stdcall PositionerCurrentVelocityAccelerationFiltersGet (int SocketIndex, char * PositionerName, double * CurrentVelocityCutOffFrequency, double * CurrentAccelerationCutOffFrequency);  /* Get current velocity and acceleration cutoff frequencies */
DLL int __stdcall PositionerCurrentVelocityAccelerationFiltersSet (int SocketIndex, char * PositionerName, double CurrentVelocityCutOffFrequency, double CurrentAccelerationCutOffFrequency);  /* Set current velocity and acceleration cutoff frequencies */
DLL int __stdcall PositionerDriverFiltersGet (int SocketIndex, char * PositionerName, double * KI, double * NotchFrequency, double * NotchBandwidth, double * NotchGain, double * LowpassFrequency);  /* Get driver filters parameters */
DLL int __stdcall PositionerDriverFiltersSet (int SocketIndex, char * PositionerName, double KI, double NotchFrequency, double NotchBandwidth, double NotchGain, double LowpassFrequency);  /* Set driver filters parameters */
DLL int __stdcall PositionerDriverPositionOffsetsGet (int SocketIndex, char * PositionerName, double * StagePositionOffset, double * GagePositionOffset);  /* Get driver stage and gage position offset */
DLL int __stdcall PositionerDriverStatusGet (int SocketIndex, char * PositionerName, int * DriverStatus);  /* Read positioner driver status */
DLL int __stdcall PositionerDriverStatusStringGet (int SocketIndex, int PositionerDriverStatus, char * PositionerDriverStatusString);  /* Return the positioner driver status string corresponding to the positioner error code */
DLL int __stdcall PositionerEncoderAmplitudeValuesGet (int SocketIndex, char * PositionerName, double * CalibrationSinusAmplitude, double * CurrentSinusAmplitude, double * CalibrationCosinusAmplitude, double * CurrentCosinusAmplitude);  /* Read analog interpolated encoder amplitude values */
DLL int __stdcall PositionerEncoderCalibrationParametersGet (int SocketIndex, char * PositionerName, double * SinusOffset, double * CosinusOffset, double * DifferentialGain, double * PhaseCompensation);  /* Read analog interpolated encoder calibration parameters */
DLL int __stdcall PositionerErrorGet (int SocketIndex, char * PositionerName, int * ErrorCode);  /* Read and clear positioner error code */
DLL int __stdcall PositionerErrorRead (int SocketIndex, char * PositionerName, int * ErrorCode);  /* Read only positioner error code without clear it */
DLL int __stdcall PositionerErrorStringGet (int SocketIndex, int PositionerErrorCode, char * PositionerErrorString);  /* Return the positioner status string corresponding to the positioner error code */
DLL int __stdcall PositionerExcitationSignalGet (int SocketIndex, char * PositionerName, int * Mode, double * Frequency, double * Amplitude, double * Time);  /* Read disturbing signal parameters */
DLL int __stdcall PositionerExcitationSignalSet (int SocketIndex, char * PositionerName, int Mode, double Frequency, double Amplitude, double Time);  /* Update disturbing signal parameters */
DLL int __stdcall PositionerExternalLatchPositionGet (int SocketIndex, char * PositionerName, double * Position);  /* Read external latch position */
DLL int __stdcall PositionerHardwareStatusGet (int SocketIndex, char * PositionerName, int * HardwareStatus);  /* Read positioner hardware status */
DLL int __stdcall PositionerHardwareStatusStringGet (int SocketIndex, int PositionerHardwareStatus, char * PositionerHardwareStatusString);  /* Return the positioner hardware status string corresponding to the positioner error code */
DLL int __stdcall PositionerHardInterpolatorFactorGet (int SocketIndex, char * PositionerName, int * InterpolationFactor);  /* Get hard interpolator parameters */
DLL int __stdcall PositionerHardInterpolatorFactorSet (int SocketIndex, char * PositionerName, int InterpolationFactor);  /* Set hard interpolator parameters */
DLL int __stdcall PositionerMaximumVelocityAndAccelerationGet (int SocketIndex, char * PositionerName, double * MaximumVelocity, double * MaximumAcceleration);  /* Return maximum velocity and acceleration of the positioner */
DLL int __stdcall PositionerMotionDoneGet (int SocketIndex, char * PositionerName, double * PositionWindow, double * VelocityWindow, double * CheckingTime, double * MeanPeriod, double * TimeOut);  /* Read motion done parameters */
DLL int __stdcall PositionerMotionDoneSet (int SocketIndex, char * PositionerName, double PositionWindow, double VelocityWindow, double CheckingTime, double MeanPeriod, double TimeOut);  /* Update motion done parameters */
DLL int __stdcall PositionerPositionCompareAquadBAlwaysEnable (int SocketIndex, char * PositionerName);  /* Enable AquadB signal in always mode */
DLL int __stdcall PositionerPositionCompareAquadBWindowedGet (int SocketIndex, char * PositionerName, double * MinimumPosition, double * MaximumPosition, bool * EnableState);  /* Read position compare AquadB windowed parameters */
DLL int __stdcall PositionerPositionCompareAquadBWindowedSet (int SocketIndex, char * PositionerName, double MinimumPosition, double MaximumPosition);  /* Set position compare AquadB windowed parameters */
DLL int __stdcall PositionerPositionCompareGet (int SocketIndex, char * PositionerName, double * MinimumPosition, double * MaximumPosition, double * PositionStep, bool * EnableState);  /* Read position compare parameters */
DLL int __stdcall PositionerPositionCompareSet (int SocketIndex, char * PositionerName, double MinimumPosition, double MaximumPosition, double PositionStep);  /* Set position compare parameters */
DLL int __stdcall PositionerPositionCompareEnable (int SocketIndex, char * PositionerName);  /* Enable position compare */
DLL int __stdcall PositionerPositionCompareDisable (int SocketIndex, char * PositionerName);  /* Disable position compare */
DLL int __stdcall PositionerPositionComparePulseParametersGet (int SocketIndex, char * PositionerName, double * PCOPulseWidth, double * EncoderSettlingTime);  /* Get position compare PCO pulse parameters */
DLL int __stdcall PositionerPositionComparePulseParametersSet (int SocketIndex, char * PositionerName, double PCOPulseWidth, double EncoderSettlingTime);  /* Set position compare PCO pulse parameters */
DLL int __stdcall PositionerRawEncoderPositionGet (int SocketIndex, char * PositionerName, double UserEncoderPosition, double * RawEncoderPosition);  /* Get the raw encoder position */
DLL int __stdcall PositionersEncoderIndexDifferenceGet (int SocketIndex, char * PositionerName, double * distance);  /* Return the difference between index of primary axis and secondary axis (only after homesearch) */
DLL int __stdcall PositionerSGammaExactVelocityAjustedDisplacementGet (int SocketIndex, char * PositionerName, double DesiredDisplacement, double * AdjustedDisplacement);  /* Return adjusted displacement to get exact velocity */
DLL int __stdcall PositionerSGammaParametersGet (int SocketIndex, char * PositionerName, double * Velocity, double * Acceleration, double * MinimumTjerkTime, double * MaximumTjerkTime);  /* Read dynamic parameters for one axe of a group for a future displacement  */
DLL int __stdcall PositionerSGammaParametersSet (int SocketIndex, char * PositionerName, double Velocity, double Acceleration, double MinimumTjerkTime, double MaximumTjerkTime);  /* Update dynamic parameters for one axe of a group for a future displacement */
DLL int __stdcall PositionerSGammaPreviousMotionTimesGet (int SocketIndex, char * PositionerName, double * SettingTime, double * SettlingTime);  /* Read SettingTime and SettlingTime */
DLL int __stdcall PositionerStageParameterGet (int SocketIndex, char * PositionerName, char * ParameterName, char * ParameterValue);  /* Return the stage parameter */
DLL int __stdcall PositionerStageParameterSet (int SocketIndex, char * PositionerName, char * ParameterName, char * ParameterValue);  /* Save the stage parameter */
DLL int __stdcall PositionerTimeFlasherGet (int SocketIndex, char * PositionerName, double * MinimumPosition, double * MaximumPosition, double * PositionStep, bool * EnableState);  /* Read time flasher parameters */
DLL int __stdcall PositionerTimeFlasherSet (int SocketIndex, char * PositionerName, double MinimumPosition, double MaximumPosition, double TimeInterval);  /* Set time flasher parameters */
DLL int __stdcall PositionerTimeFlasherEnable (int SocketIndex, char * PositionerName);  /* Enable time flasher */
DLL int __stdcall PositionerTimeFlasherDisable (int SocketIndex, char * PositionerName);  /* Disable time flasher */
DLL int __stdcall PositionerUserTravelLimitsGet (int SocketIndex, char * PositionerName, double * UserMinimumTarget, double * UserMaximumTarget);  /* Read UserMinimumTarget and UserMaximumTarget */
DLL int __stdcall PositionerUserTravelLimitsSet (int SocketIndex, char * PositionerName, double UserMinimumTarget, double UserMaximumTarget);  /* Update UserMinimumTarget and UserMaximumTarget */
DLL int __stdcall PositionerDACOffsetGet (int SocketIndex, char * PositionerName, short * DACOffset1, short * DACOffset2);  /* Get DAC offsets */
DLL int __stdcall PositionerDACOffsetSet (int SocketIndex, char * PositionerName, short DACOffset1, short DACOffset2);  /* Set DAC offsets */
DLL int __stdcall PositionerDACOffsetDualGet (int SocketIndex, char * PositionerName, short * PrimaryDACOffset1, short * PrimaryDACOffset2, short * SecondaryDACOffset1, short * SecondaryDACOffset2);  /* Get dual DAC offsets */
DLL int __stdcall PositionerDACOffsetDualSet (int SocketIndex, char * PositionerName, short PrimaryDACOffset1, short PrimaryDACOffset2, short SecondaryDACOffset1, short SecondaryDACOffset2);  /* Set dual DAC offsets */
DLL int __stdcall PositionerCorrectorAutoTuning (int SocketIndex, char * PositionerName, int TuningMode, double * KP, double * KI, double * KD);  /* Astrom&Hagglund based auto-tuning */
DLL int __stdcall PositionerAccelerationAutoScaling (int SocketIndex, char * PositionerName, double * Scaling);  /* Astrom&Hagglund based auto-scaling */
DLL int __stdcall MultipleAxesPVTVerification (int SocketIndex, char * GroupName, char * TrajectoryFileName);  /* Multiple axes PVT trajectory verification */
DLL int __stdcall MultipleAxesPVTVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);  /* Multiple axes PVT trajectory verification result get */
DLL int __stdcall MultipleAxesPVTExecution (int SocketIndex, char * GroupName, char * TrajectoryFileName, int ExecutionNumber);  /* Multiple axes PVT trajectory execution */
DLL int __stdcall MultipleAxesPVTParametersGet (int SocketIndex, char * GroupName, char * FileName, int * CurrentElementNumber);  /* Multiple axes PVT trajectory get parameters */
DLL int __stdcall MultipleAxesPVTPulseOutputSet (int SocketIndex, char * GroupName, int StartElement, int EndElement, double TimeInterval);  /* Configure pulse output on trajectory */
DLL int __stdcall MultipleAxesPVTPulseOutputGet (int SocketIndex, char * GroupName, int * StartElement, int * EndElement, double * TimeInterval);  /* Get pulse output on trajectory configuration */
DLL int __stdcall SingleAxisSlaveModeEnable (int SocketIndex, char * GroupName);  /* Enable the slave mode */
DLL int __stdcall SingleAxisSlaveModeDisable (int SocketIndex, char * GroupName);  /* Disable the slave mode */
DLL int __stdcall SingleAxisSlaveParametersSet (int SocketIndex, char * GroupName, char * PositionerName, double Ratio);  /* Set slave parameters */
DLL int __stdcall SingleAxisSlaveParametersGet (int SocketIndex, char * GroupName, char * PositionerName, double * Ratio);  /* Get slave parameters */
DLL int __stdcall SpindleSlaveModeEnable (int SocketIndex, char * GroupName);  /* Enable the slave mode */
DLL int __stdcall SpindleSlaveModeDisable (int SocketIndex, char * GroupName);  /* Disable the slave mode */
DLL int __stdcall SpindleSlaveParametersSet (int SocketIndex, char * GroupName, char * PositionerName, double Ratio);  /* Set slave parameters */
DLL int __stdcall SpindleSlaveParametersGet (int SocketIndex, char * GroupName, char * PositionerName, double * Ratio);  /* Get slave parameters */
DLL int __stdcall GroupSpinParametersSet (int SocketIndex, char * GroupName, double Velocity, double Acceleration);  /* Modify Spin parameters on selected group and activate the continuous move */
DLL int __stdcall GroupSpinParametersGet (int SocketIndex, char * GroupName, double * Velocity, double * Acceleration);  /* Get Spin parameters on selected group */
DLL int __stdcall GroupSpinCurrentGet (int SocketIndex, char * GroupName, double * Velocity, double * Acceleration);  /* Get Spin current on selected group */
DLL int __stdcall GroupSpinModeStop (int SocketIndex, char * GroupName, double Acceleration);  /* Stop Spin mode on selected group with specified acceleration */
DLL int __stdcall XYLineArcVerification (int SocketIndex, char * GroupName, char * TrajectoryFileName);  /* XY trajectory verification */
DLL int __stdcall XYLineArcVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);  /* XY trajectory verification result get */
DLL int __stdcall XYLineArcExecution (int SocketIndex, char * GroupName, char * TrajectoryFileName, double Velocity, double Acceleration, int ExecutionNumber);  /* XY trajectory execution */
DLL int __stdcall XYLineArcParametersGet (int SocketIndex, char * GroupName, char * FileName, double * Velocity, double * Acceleration, int * CurrentElementNumber);  /* XY trajectory get parameters */
DLL int __stdcall XYLineArcPulseOutputSet (int SocketIndex, char * GroupName, double StartLength, double EndLength, double PathLengthInterval);  /* Configure pulse output on trajectory */
DLL int __stdcall XYLineArcPulseOutputGet (int SocketIndex, char * GroupName, double * StartLength, double * EndLength, double * PathLengthInterval);  /* Get pulse output on trajectory configuration */
DLL int __stdcall XYZGroupPositionCorrectedProfilerGet (int SocketIndex, char * GroupName, double PositionX, double PositionY, double PositionZ, double * CorrectedProfilerPositionX, double * CorrectedProfilerPositionY, double * CorrectedProfilerPositionZ);  /* Return corrected profiler positions */
DLL int __stdcall XYZSplineVerification (int SocketIndex, char * GroupName, char * TrajectoryFileName);  /* XYZ trajectory verifivation */
DLL int __stdcall XYZSplineVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);  /* XYZ trajectory verification result get */
DLL int __stdcall XYZSplineExecution (int SocketIndex, char * GroupName, char * TrajectoryFileName, double Velocity, double Acceleration);  /* XYZ trajectory execution */
DLL int __stdcall XYZSplineParametersGet (int SocketIndex, char * GroupName, char * FileName, double * Velocity, double * Acceleration, int * CurrentElementNumber);  /* XYZ trajectory get parameters */
DLL int __stdcall OptionalModuleExecute (int SocketIndex, char * ModuleFileName, char * TaskName);  /* Execute an optional module */
DLL int __stdcall OptionalModuleKill (int SocketIndex, char * TaskName);  /* Kill an optional module */
DLL int __stdcall EEPROMCIESet (int SocketIndex, int CardNumber, char * ReferenceString);  /* Set CIE EEPROM reference string */
DLL int __stdcall EEPROMDACOffsetCIESet (int SocketIndex, int PlugNumber, double DAC1Offset, double DAC2Offset);  /* Set CIE DAC offsets */
DLL int __stdcall EEPROMDriverSet (int SocketIndex, int PlugNumber, char * ReferenceString);  /* Set Driver EEPROM reference string */
DLL int __stdcall EEPROMINTSet (int SocketIndex, int CardNumber, char * ReferenceString);  /* Set INT EEPROM reference string */
DLL int __stdcall CPUCoreAndBoardSupplyVoltagesGet (int SocketIndex, double * VoltageCPUCore, double * SupplyVoltage1P5V, double * SupplyVoltage3P3V, double * SupplyVoltage5V, double * SupplyVoltage12V, double * SupplyVoltageM12V, double * SupplyVoltageM5V, double * SupplyVoltage5VSB);  /* Get power informations */
DLL int __stdcall CPUTemperatureAndFanSpeedGet (int SocketIndex, double * CPUTemperature, double * CPUFanSpeed);  /* Get CPU temperature and fan speed */
DLL int __stdcall ActionListGet (int SocketIndex, char * ActionList);  /* Action list */
DLL int __stdcall ActionExtendedListGet (int SocketIndex, char * ActionList);  /* Action extended list */
DLL int __stdcall APIExtendedListGet (int SocketIndex, char * Method);  /* API method list */
DLL int __stdcall APIListGet (int SocketIndex, char * Method);  /* API method list without extended API */
DLL int __stdcall ControllerStatusListGet (int SocketIndex, char * ControllerStatusList);  /* Controller status list */
DLL int __stdcall ErrorListGet (int SocketIndex, char * ErrorsList);  /* Error list */
DLL int __stdcall EventListGet (int SocketIndex, char * EventList);  /* General event list */
DLL int __stdcall GatheringListGet (int SocketIndex, char * list);  /* Gathering type list */
DLL int __stdcall GatheringExtendedListGet (int SocketIndex, char * list);  /* Gathering type extended list */
DLL int __stdcall GatheringExternalListGet (int SocketIndex, char * list);  /* External Gathering type list */
DLL int __stdcall GroupStatusListGet (int SocketIndex, char * GroupStatusList);  /* Group status list */
DLL int __stdcall HardwareInternalListGet (int SocketIndex, char * InternalHardwareList);  /* Internal hardware list */
DLL int __stdcall HardwareDriverAndStageGet (int SocketIndex, int PlugNumber, char * DriverName, char * StageName);  /* Smart hardware */
DLL int __stdcall ObjectsListGet (int SocketIndex, char * ObjectsList);  /* Group name and positioner name */
DLL int __stdcall PositionerErrorListGet (int SocketIndex, char * PositionerErrorList);  /* Positioner error list */
DLL int __stdcall PositionerHardwareStatusListGet (int SocketIndex, char * PositionerHardwareStatusList);  /* Positioner hardware status list */
DLL int __stdcall PositionerDriverStatusListGet (int SocketIndex, char * PositionerDriverStatusList);  /* Positioner driver status list */
DLL int __stdcall ReferencingActionListGet (int SocketIndex, char * list);  /* Get referencing action list */
DLL int __stdcall ReferencingSensorListGet (int SocketIndex, char * list);  /* Get referencing sensor list */
DLL int __stdcall GatheringUserDatasGet (int SocketIndex, double * UserData1, double * UserData2, double * UserData3, double * UserData4, double * UserData5, double * UserData6, double * UserData7, double * UserData8);  /* Return user data values */
DLL int __stdcall ControllerMotionKernelPeriodMinMaxGet (int SocketIndex, double * MinimumCorrectorPeriod, double * MaximumCorrectorPeriod, double * MinimumProfilerPeriod, double * MaximumProfilerPeriod, double * MinimumServitudesPeriod, double * MaximumServitudesPeriod);  /* Get controller motion kernel min/max periods */
DLL int __stdcall ControllerMotionKernelPeriodMinMaxReset (int SocketIndex);  /* Reset controller motion kernel min/max periods */
DLL int __stdcall SocketsStatusGet (int SocketIndex, char * SocketsStatus);  /* Get sockets current status */
DLL int __stdcall TestTCP (int SocketIndex, char * InputString, char * ReturnString);  /* Test TCP/IP transfert */


#ifdef __cplusplus
}
#endif
