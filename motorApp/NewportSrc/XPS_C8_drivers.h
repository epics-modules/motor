/*////////////////////////////////////////////////////////////////////
* Created header file xps_c8_drivers.h for API headings 
*/ 


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
typedef int bool;
#endif


DLL int __stdcall TCP_ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut); 
DLL void __stdcall TCP_SetTimeout(int SocketIndex, double Timeout); 
DLL void __stdcall TCP_CloseSocket(int SocketIndex); 
DLL char * __stdcall TCP_GetError(int SocketIndex); 
DLL char * __stdcall GetLibraryVersion(void); 
DLL int __stdcall ElapsedTimeGet (int SocketIndex, double * ElapsedTime);
DLL int __stdcall ErrorStringGet (int SocketIndex, int ErrorCode, char * ErrorString);
DLL int __stdcall FirmwareVersionGet (int SocketIndex, char * Version);
DLL int __stdcall TCLScriptExecute (int SocketIndex, char * TCLFileName, char * TaskName, char * ParametersList);
DLL int __stdcall TCLScriptExecuteAndWait (int SocketIndex, char * TCLFileName, char * TaskName, char * InputParametersList, char * OutputParametersList);
DLL int __stdcall TCLScriptKill (int SocketIndex, char * TaskName);
DLL int __stdcall TimerGet (int SocketIndex, char * TimerName, int * FrequencyTicks);
DLL int __stdcall TimerSet (int SocketIndex, char * TimerName, int FrequencyTicks);
DLL int __stdcall Reboot (int SocketIndex);
DLL int __stdcall EventAdd (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter, char * ActionName, char * ActionParameter1, char * ActionParameter2, char * ActionParameter3);
DLL int __stdcall EventGet (int SocketIndex, char * PositionerName, char * EventsAndActionsList);
DLL int __stdcall EventRemove (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);
DLL int __stdcall EventWait (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);
DLL int __stdcall EventExtendedConfigurationTriggerSet (int SocketIndex, int NbElements, char * ExtendedEventNameList, char * EventParameter1List, char * EventParameter2List, char * EventParameter3List, char * EventParameter4List);
DLL int __stdcall EventExtendedConfigurationTriggerGet (int SocketIndex, char * EventTriggerConfiguration);
DLL int __stdcall EventExtendedConfigurationActionSet (int SocketIndex, int NbElements, char * ExtendedActionNameList, char * ActionParameter1List, char * ActionParameter2List, char * ActionParameter3List, char * ActionParameter4List);
DLL int __stdcall EventExtendedConfigurationActionGet (int SocketIndex, char * ActionConfiguration);
DLL int __stdcall EventExtendedStart (int SocketIndex, int * ID);
DLL int __stdcall EventExtendedAllGet (int SocketIndex, char * EventActionConfigurations);
DLL int __stdcall EventExtendedGet (int SocketIndex, int ID, char * EventTriggerConfiguration, char * ActionConfiguration);
DLL int __stdcall EventExtendedRemove (int SocketIndex, int ID);
DLL int __stdcall EventExtendedWait (int SocketIndex);
DLL int __stdcall GatheringConfigurationGet (int SocketIndex, char * Type);
DLL int __stdcall GatheringConfigurationSet (int SocketIndex, int NbElements, char * TypeList);
DLL int __stdcall GatheringCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);
DLL int __stdcall GatheringStopAndSave (int SocketIndex);
DLL int __stdcall GatheringDataAcquire (int SocketIndex);
DLL int __stdcall GatheringReset (int SocketIndex);
DLL int __stdcall GatheringExternalConfigurationSet (int SocketIndex, int NbElements, char * TypeList);
DLL int __stdcall GatheringExternalConfigurationGet (int SocketIndex, char * Type);
DLL int __stdcall GatheringExternalCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);
DLL int __stdcall GatheringExternalStopAndSave (int SocketIndex);
DLL int __stdcall GlobalArrayGet (int SocketIndex, int Number, char * ValueString);
DLL int __stdcall GlobalArraySet (int SocketIndex, int Number, char * ValueString);
DLL int __stdcall GPIOAnalogGet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogValue[]);
DLL int __stdcall GPIOAnalogSet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogOutputValue[]);
DLL int __stdcall GPIOAnalogGainGet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);
DLL int __stdcall GPIOAnalogGainSet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);
DLL int __stdcall GPIODigitalGet (int SocketIndex, char * GPIOName, unsigned short * DigitalValue);
DLL int __stdcall GPIODigitalSet (int SocketIndex, char * GPIOName, unsigned short Mask, unsigned short DigitalOutputValue);
DLL int __stdcall GroupAnalogTrackingModeEnable (int SocketIndex, char * GroupName, char * Type);
DLL int __stdcall GroupAnalogTrackingModeDisable (int SocketIndex, char * GroupName);
DLL int __stdcall GroupCorrectorOutputGet (int SocketIndex, char * GroupName, int NbElements, double CorrectorOutput[]);
DLL int __stdcall GroupHomeSearch (int SocketIndex, char * GroupName);
DLL int __stdcall GroupHomeSearchAndRelativeMove (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);
DLL int __stdcall GroupInitialize (int SocketIndex, char * GroupName);
DLL int __stdcall GroupInitializeWithEncoderCalibration (int SocketIndex, char * GroupName);
DLL int __stdcall GroupJogParametersSet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL int __stdcall GroupJogParametersGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL int __stdcall GroupJogCurrentGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL int __stdcall GroupJogModeEnable (int SocketIndex, char * GroupName);
DLL int __stdcall GroupJogModeDisable (int SocketIndex, char * GroupName);
DLL int __stdcall GroupKill (int SocketIndex, char * GroupName);
DLL int __stdcall GroupMoveAbort (int SocketIndex, char * GroupName);
DLL int __stdcall GroupMoveAbsolute (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);
DLL int __stdcall GroupMoveRelative (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);
DLL int __stdcall GroupMotionDisable (int SocketIndex, char * GroupName);
DLL int __stdcall GroupMotionEnable (int SocketIndex, char * GroupName);
DLL int __stdcall GroupPositionCurrentGet (int SocketIndex, char * GroupName, int NbElements, double CurrentEncoderPosition[]);
DLL int __stdcall GroupPositionSetpointGet (int SocketIndex, char * GroupName, int NbElements, double SetPointPosition[]);
DLL int __stdcall GroupPositionTargetGet (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);
DLL int __stdcall GroupReferencingActionExecute (int SocketIndex, char * PositionerName, char * ReferencingAction, char * ReferencingSensor, double ReferencingParameter);
DLL int __stdcall GroupReferencingStart (int SocketIndex, char * GroupName);
DLL int __stdcall GroupReferencingStop (int SocketIndex, char * GroupName);
DLL int __stdcall GroupStatusGet (int SocketIndex, char * GroupName, int * Status);
DLL int __stdcall GroupStatusStringGet (int SocketIndex, int GroupStatusCode, char * GroupStatusString);
DLL int __stdcall GroupVelocityCurrentGet (int SocketIndex, char * GroupName, int NbElements, double CurrentVelocity[]);
DLL int __stdcall KillAll (int SocketIndex);
DLL int __stdcall PositionerAnalogTrackingPositionParametersGet (int SocketIndex, char * PositionerName, char * GPIOName, double * Offset, double * Scale, double * Velocity, double * Acceleration);
DLL int __stdcall PositionerAnalogTrackingPositionParametersSet (int SocketIndex, char * PositionerName, char * GPIOName, double Offset, double Scale, double Velocity, double Acceleration);
DLL int __stdcall PositionerAnalogTrackingVelocityParametersGet (int SocketIndex, char * PositionerName, char * GPIOName, double * Offset, double * Scale, double * DeadBandThreshold, int * Order, double * Velocity, double * Acceleration);
DLL int __stdcall PositionerAnalogTrackingVelocityParametersSet (int SocketIndex, char * PositionerName, char * GPIOName, double Offset, double Scale, double DeadBandThreshold, int Order, double Velocity, double Acceleration);
DLL int __stdcall PositionerBacklashGet (int SocketIndex, char * PositionerName, double * BacklashValue, char * BacklaskStatus);
DLL int __stdcall PositionerBacklashSet (int SocketIndex, char * PositionerName, double BacklashValue);
DLL int __stdcall PositionerBacklashEnable (int SocketIndex, char * PositionerName);
DLL int __stdcall PositionerBacklashDisable (int SocketIndex, char * PositionerName);
DLL int __stdcall PositionerCorrectorNotchFiltersSet (int SocketIndex, char * PositionerName, double NotchFrequency1, double NotchBandwith1, double NotchGain1, double NotchFrequency2, double NotchBandwith2, double NotchGain2);
DLL int __stdcall PositionerCorrectorNotchFiltersGet (int SocketIndex, char * PositionerName, double * NotchFrequency1, double * NotchBandwith1, double * NotchGain1, double * NotchFrequency2, double * NotchBandwith2, double * NotchGain2);
DLL int __stdcall PositionerCorrectorPIDFFAccelerationSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainAcceleration);
DLL int __stdcall PositionerCorrectorPIDFFAccelerationGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainAcceleration);
DLL int __stdcall PositionerCorrectorPIDFFVelocitySet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity);
DLL int __stdcall PositionerCorrectorPIDFFVelocityGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity);
DLL int __stdcall PositionerCorrectorPIDDualFFVoltageSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity, double FeedForwardGainAcceleration, double Friction);
DLL int __stdcall PositionerCorrectorPIDDualFFVoltageGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity, double * FeedForwardGainAcceleration, double * Friction);
DLL int __stdcall PositionerCorrectorPIPositionSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double IntegrationTime);
DLL int __stdcall PositionerCorrectorPIPositionGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * IntegrationTime);
DLL int __stdcall PositionerCorrectorTypeGet (int SocketIndex, char * PositionerName, char * CorrectorType);
DLL int __stdcall PositionerCurrentVelocityAccelerationFiltersSet (int SocketIndex, char * PositionerName, double CurrentVelocityCutOffFrequency, double CurrentAccelerationCutOffFrequency);
DLL int __stdcall PositionerCurrentVelocityAccelerationFiltersGet (int SocketIndex, char * PositionerName, double * CurrentVelocityCutOffFrequency, double * CurrentAccelerationCutOffFrequency);
DLL int __stdcall PositionerDriverStatusGet (int SocketIndex, char * PositionerName, int * DriverStatus);
DLL int __stdcall PositionerDriverStatusStringGet (int SocketIndex, int PositionerDriverStatus, char * PositionerDriverStatusString);
DLL int __stdcall PositionerEncoderAmplitudeValuesGet (int SocketIndex, char * PositionerName, double * MaxSinusAmplitude, double * CurrentSinusAmplitude, double * MaxCosinusAmplitude, double * CurrentCosinusAmplitude);
DLL int __stdcall PositionerEncoderCalibrationParametersGet (int SocketIndex, char * PositionerName, double * SinusOffset, double * CosinusOffset, double * DifferentialGain, double * PhaseCompensation);
DLL int __stdcall PositionerErrorGet (int SocketIndex, char * PositionerName, int * ErrorCode);
DLL int __stdcall PositionerErrorStringGet (int SocketIndex, int PositionerErrorCode, char * PositionerErrorString);
DLL int __stdcall PositionerHardwareStatusGet (int SocketIndex, char * PositionerName, int * HardwareStatus);
DLL int __stdcall PositionerHardwareStatusStringGet (int SocketIndex, int PositionerHardwareStatus, char * PositionerHardwareStatusString);
DLL int __stdcall PositionerHardInterpolatorFactorGet (int SocketIndex, char * PositionerName, int * InterpolationFactor);
DLL int __stdcall PositionerHardInterpolatorFactorSet (int SocketIndex, char * PositionerName, int InterpolationFactor);
DLL int __stdcall PositionerMaximumVelocityAndAccelerationGet (int SocketIndex, char * PositionerName, double * MaximumVelocity, double * MaximumAcceleration);
DLL int __stdcall PositionerMotionDoneGet (int SocketIndex, char * PositionerName, double * PositionWindow, double * VelocityWindow, double * CheckingTime, double * MeanPeriod, double * TimeOut);
DLL int __stdcall PositionerMotionDoneSet (int SocketIndex, char * PositionerName, double PositionWindow, double VelocityWindow, double CheckingTime, double MeanPeriod, double TimeOut);
DLL int __stdcall PositionerPositionCompareGet (int SocketIndex, char * PositionerName, double * MinimumPosition, double * MaximumPosition, double * PositionStep, bool * EnableState);
DLL int __stdcall PositionerPositionCompareSet (int SocketIndex, char * PositionerName, double MinimumPosition, double MaximumPosition, double PositionStep);
DLL int __stdcall PositionerPositionCompareEnable (int SocketIndex, char * PositionerName);
DLL int __stdcall PositionerPositionCompareDisable (int SocketIndex, char * PositionerName);
DLL int __stdcall PositionerSGammaExactVelocityAjustedDisplacementGet (int SocketIndex, char * PositionerName, double DesiredDisplacement, double * AdjustedDisplacement);
DLL int __stdcall PositionerSGammaParametersGet (int SocketIndex, char * PositionerName, double * Velocity, double * Acceleration, double * MinimumTjerkTime, double * MaximumTjerkTime);
DLL int __stdcall PositionerSGammaParametersSet (int SocketIndex, char * PositionerName, double Velocity, double Acceleration, double MinimumTjerkTime, double MaximumTjerkTime);
DLL int __stdcall PositionerSGammaPreviousMotionTimesGet (int SocketIndex, char * PositionerName, double * SettingTime, double * SettlingTime);
DLL int __stdcall PositionerStageParameterGet (int SocketIndex, char * PositionerName, char * ParameterName, char * ParameterValue);
DLL int __stdcall PositionerUserTravelLimitsGet (int SocketIndex, char * PositionerName, double * UserMinimumTarget, double * UserMaximumTarget);
DLL int __stdcall PositionerUserTravelLimitsSet (int SocketIndex, char * PositionerName, double UserMinimumTarget, double UserMaximumTarget);
DLL int __stdcall MultipleAxesPVTVerification (int SocketIndex, char * GroupName, char * TrajectoryFileName);
DLL int __stdcall MultipleAxesPVTVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);
DLL int __stdcall MultipleAxesPVTExecution (int SocketIndex, char * GroupName, char * TrajectoryFileName, int ExecutionNumber);
DLL int __stdcall MultipleAxesPVTParametersGet (int SocketIndex, char * GroupName, char * FileName, int * CurrentElementNumber);
DLL int __stdcall MultipleAxesPVTPulseOutputSet (int SocketIndex, char * GroupName, int StartElement, int EndElement, double TimeInterval);
DLL int __stdcall MultipleAxesPVTPulseOutputGet (int SocketIndex, char * GroupName, int * StartElement, int * EndElement, double * TimeInterval);
DLL int __stdcall SingleAxisSlaveModeEnable (int SocketIndex, char * GroupName);
DLL int __stdcall SingleAxisSlaveModeDisable (int SocketIndex, char * GroupName);
DLL int __stdcall SingleAxisSlaveParametersSet (int SocketIndex, char * GroupName, char * PositionerName, double Ratio);
DLL int __stdcall SingleAxisSlaveParametersGet (int SocketIndex, char * GroupName, char * PositionerName, double * Ratio);
DLL int __stdcall SpindleSlaveModeEnable (int SocketIndex, char * GroupName);
DLL int __stdcall SpindleSlaveModeDisable (int SocketIndex, char * GroupName);
DLL int __stdcall SpindleSlaveParametersSet (int SocketIndex, char * GroupName, char * PositionerName, double Ratio);
DLL int __stdcall SpindleSlaveParametersGet (int SocketIndex, char * GroupName, char * PositionerName, double * Ratio);
DLL int __stdcall GroupSpinParametersSet (int SocketIndex, char * GroupName, double Velocity, double Acceleration);
DLL int __stdcall GroupSpinParametersGet (int SocketIndex, char * GroupName, double * Velocity, double * Acceleration);
DLL int __stdcall GroupSpinCurrentGet (int SocketIndex, char * GroupName, double * Velocity, double * Acceleration);
DLL int __stdcall GroupSpinModeStop (int SocketIndex, char * GroupName, double Acceleration);
DLL int __stdcall XYLineArcVerification (int SocketIndex, char * GroupName, char * TrajectoryFileName);
DLL int __stdcall XYLineArcVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);
DLL int __stdcall XYLineArcExecution (int SocketIndex, char * GroupName, char * TrajectoryFileName, double Velocity, double Acceleration, int ExecutionNumber);
DLL int __stdcall XYLineArcParametersGet (int SocketIndex, char * GroupName, char * FileName, double * Velocity, double * Acceleration, int * CurrentElementNumber);
DLL int __stdcall XYLineArcPulseOutputSet (int SocketIndex, char * GroupName, double StartLength, double EndLength, double PathLengthInterval);
DLL int __stdcall XYLineArcPulseOutputGet (int SocketIndex, char * GroupName, double * StartLength, double * EndLength, double * PathLengthInterval);
DLL int __stdcall XYZSplineVerification (int SocketIndex, char * GroupName, char * TrajectoryFileName);
DLL int __stdcall XYZSplineVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);
DLL int __stdcall XYZSplineExecution (int SocketIndex, char * GroupName, char * TrajectoryFileName, double Velocity, double Acceleration);
DLL int __stdcall XYZSplineParametersGet (int SocketIndex, char * GroupName, char * FileName, double * Velocity, double * Acceleration, int * CurrentElementNumber);
DLL int __stdcall EEPROMCIESet (int SocketIndex, int CardNumber, char * ReferenceString);
DLL int __stdcall EEPROMDACOffsetCIESet (int SocketIndex, int PlugNumber, double DAC1Offset, double DAC2Offset);
DLL int __stdcall EEPROMDriverSet (int SocketIndex, int PlugNumber, char * ReferenceString);
DLL int __stdcall EEPROMINTSet (int SocketIndex, int CardNumber, char * ReferenceString);
DLL int __stdcall CPUCoreAndBoardSupplyVoltagesGet (int SocketIndex, double * VoltageCPUCore, double * SupplyVoltage1P5V, double * SupplyVoltage3P3V, double * SupplyVoltage5V, double * SupplyVoltage12V, double * SupplyVoltageM12V, double * SupplyVoltageM5V, double * SupplyVoltage5VSB);
DLL int __stdcall CPUTemperatureAndFanSpeedGet (int SocketIndex, double * CPUTemperature, double * CPUFanSpeed);
DLL int __stdcall ActionListGet (int SocketIndex, char * ActionList);
DLL int __stdcall ActionExtendedListGet (int SocketIndex, char * ActionList);
DLL int __stdcall APIExtendedListGet (int SocketIndex, char * Method);
DLL int __stdcall APIListGet (int SocketIndex, char * Method);
DLL int __stdcall ErrorListGet (int SocketIndex, char * ErrorsList);
DLL int __stdcall EventListGet (int SocketIndex, char * EventList);
DLL int __stdcall GatheringListGet (int SocketIndex, char * list);
DLL int __stdcall GatheringExtendedListGet (int SocketIndex, char * list);
DLL int __stdcall GatheringExternalListGet (int SocketIndex, char * list);
DLL int __stdcall GroupStatusListGet (int SocketIndex, char * GroupStatusList);
DLL int __stdcall HardwareInternalListGet (int SocketIndex, char * InternalHardwareList);
DLL int __stdcall HardwareDriverAndStageGet (int SocketIndex, int PlugNumber, char * DriverName, char * StageName);
DLL int __stdcall ObjectsListGet (int SocketIndex, char * ObjectsList);
DLL int __stdcall PositionerErrorListGet (int SocketIndex, char * PositionerErrorList);
DLL int __stdcall PositionerHardwareStatusListGet (int SocketIndex, char * PositionerHardwareStatusList);
DLL int __stdcall PositionerDriverStatusListGet (int SocketIndex, char * PositionerDriverStatusList);
DLL int __stdcall ReferencingActionListGet (int SocketIndex, char * list);
DLL int __stdcall ReferencingSensorListGet (int SocketIndex, char * list);
DLL int __stdcall GatheringUserDatasGet (int SocketIndex, double * UserData1, double * UserData2, double * UserData3, double * UserData4, double * UserData5, double * UserData6, double * UserData7, double * UserData8);
DLL int __stdcall TestTCP (int SocketIndex, char * InputString, char * ReturnString);


#ifdef __cplusplus
}
#endif
