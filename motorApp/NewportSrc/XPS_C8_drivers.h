////////////////////////////////////////////////////////////////////
// Created header file XPS_C8_drivers.h for API headings 
// 


#ifdef _WIN32
/* #ifndef DLL */
/* #define DLL _declspec(dllimport) */
#define DLL
#define __stdcall
#else
#define DLL
#define __stdcall
#endif


#ifdef __cplusplus
extern "C"
{
#endif


DLL int __stdcall TCP_ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut); 
DLL void __stdcall TCP_SetTimeout(int SocketIndex, double Timeout); 
DLL void __stdcall TCP_CloseSocket(int SocketIndex); 
DLL char * __stdcall TCP_GetError(int SocketIndex); 
DLL char * __stdcall GetLibraryVersion(void); 
DLL long __stdcall ElapsedTimeGet (int SocketIndex, double * ElapsedTime);
DLL long __stdcall ErrorStringGet (int SocketIndex, int ErrorCode, char * ErrorString);
DLL long __stdcall FirmwareVersionGet (int SocketIndex, char * Version);
DLL long __stdcall TCLScriptExecute (int SocketIndex, char * TCLFileName, char * TaskName, char * ParametersList);
DLL long __stdcall TCLScriptExecuteAndWait (int SocketIndex, char * TCLFileName, char * TaskName, char * InputParametersList, char * OutputParametersList);
DLL long __stdcall TCLScriptKill (int SocketIndex, char * TaskName);
DLL long __stdcall TimerGet (int SocketIndex, char * TimerName, int * FrequencyTicks);
DLL long __stdcall TimerSet (int SocketIndex, char * TimerName, int FrequencyTicks);
DLL long __stdcall Reboot (int SocketIndex);
DLL long __stdcall EventAdd (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter, char * ActionName, char * ActionParameter1, char * ActionParameter2, char * ActionParameter3);
DLL long __stdcall EventGet (int SocketIndex, char * PositionerName, char * EventsAndActionsList);
DLL long __stdcall EventRemove (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);
DLL long __stdcall EventWait (int SocketIndex, char * PositionerName, char * EventName, char * EventParameter);
DLL long __stdcall GatheringConfigurationGet (int SocketIndex, char * Type);
DLL long __stdcall GatheringConfigurationSet (int SocketIndex, int NbElements, char * TypeList);
DLL long __stdcall GatheringCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);
DLL long __stdcall GatheringStopAndSave (int SocketIndex);
DLL long __stdcall GatheringExternalConfigurationSet (int SocketIndex, int NbElements, char * TypeList);
DLL long __stdcall GatheringExternalConfigurationGet (int SocketIndex, char * Type);
DLL long __stdcall GatheringExternalCurrentNumberGet (int SocketIndex, int * CurrentNumber, int * MaximumSamplesNumber);
DLL long __stdcall GatheringExternalStopAndSave (int SocketIndex);
DLL long __stdcall GlobalArrayGet (int SocketIndex, int Number, char * ValueString);
DLL long __stdcall GlobalArraySet (int SocketIndex, int Number, char * ValueString);
DLL long __stdcall GPIOAnalogGet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogValue[]);
DLL long __stdcall GPIOAnalogSet (int SocketIndex, int NbElements, char * GPIONameList, double AnalogOutputValue[]);
DLL long __stdcall GPIOAnalogGainGet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);
DLL long __stdcall GPIOAnalogGainSet (int SocketIndex, int NbElements, char * GPIONameList, int AnalogInputGainValue[]);
DLL long __stdcall GPIODigitalGet (int SocketIndex, char * GPIOName, unsigned short * DigitalValue);
DLL long __stdcall GPIODigitalSet (int SocketIndex, char * GPIOName, unsigned short Mask, unsigned short DigitalOutputValue);
DLL long __stdcall GroupAnalogTrackingModeEnable (int SocketIndex, char * GroupName, char * Type);
DLL long __stdcall GroupAnalogTrackingModeDisable (int SocketIndex, char * GroupName);
DLL long __stdcall GroupCorrectorOutputGet (int SocketIndex, char * GroupName, int NbElements, double CorrectorOutput[]);
DLL long __stdcall GroupHomeSearch (int SocketIndex, char * GroupName);
DLL long __stdcall GroupHomeSearchAndRelativeMove (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);
DLL long __stdcall GroupInitialize (int SocketIndex, char * GroupName);
DLL long __stdcall GroupInitializeWithEncoderCalibration (int SocketIndex, char * GroupName);
DLL long __stdcall GroupJogParametersSet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL long __stdcall GroupJogParametersGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL long __stdcall GroupJogCurrentGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL long __stdcall GroupJogModeEnable (int SocketIndex, char * GroupName);
DLL long __stdcall GroupJogModeDisable (int SocketIndex, char * GroupName);
DLL long __stdcall GroupKill (int SocketIndex, char * GroupName);
DLL long __stdcall GroupMoveAbort (int SocketIndex, char * GroupName);
DLL long __stdcall GroupMoveAbsolute (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);
DLL long __stdcall GroupMoveRelative (int SocketIndex, char * GroupName, int NbElements, double TargetDisplacement[]);
DLL long __stdcall GroupMotionDisable (int SocketIndex, char * GroupName);
DLL long __stdcall GroupMotionEnable (int SocketIndex, char * GroupName);
DLL long __stdcall GroupPositionCurrentGet (int SocketIndex, char * GroupName, int NbElements, double CurrentEncoderPosition[]);
DLL long __stdcall GroupPositionSetpointGet (int SocketIndex, char * GroupName, int NbElements, double SetPointPosition[]);
DLL long __stdcall GroupPositionTargetGet (int SocketIndex, char * GroupName, int NbElements, double TargetPosition[]);
DLL long __stdcall GroupReferencingActionExecute (int SocketIndex, char * PositionerName, char * ReferencingAction, char * ReferencingSensor, double ReferencingParameter);
DLL long __stdcall GroupReferencingStart (int SocketIndex, char * GroupName);
DLL long __stdcall GroupReferencingStop (int SocketIndex, char * GroupName);
DLL long __stdcall GroupStatusGet (int SocketIndex, char * GroupName, int * Status);
DLL long __stdcall GroupStatusStringGet (int SocketIndex, int GroupStatusCode, char * GroupStatusString);
DLL long __stdcall KillAll (int SocketIndex);
DLL long __stdcall PositionerAnalogTrackingPositionParametersGet (int SocketIndex, char * PositionerName, char * GPIOName, double * Offset, double * Scale, double * Velocity, double * Acceleration);
DLL long __stdcall PositionerAnalogTrackingPositionParametersSet (int SocketIndex, char * PositionerName, char * GPIOName, double Offset, double Scale, double Velocity, double Acceleration);
DLL long __stdcall PositionerAnalogTrackingVelocityParametersGet (int SocketIndex, char * PositionerName, char * GPIOName, double * Offset, double * Scale, double * DeadBandThreshold, int * Order, double * Velocity, double * Acceleration);
DLL long __stdcall PositionerAnalogTrackingVelocityParametersSet (int SocketIndex, char * PositionerName, char * GPIOName, double Offset, double Scale, double DeadBandThreshold, int Order, double Velocity, double Acceleration);
DLL long __stdcall PositionerBacklashGet (int SocketIndex, char * PositionerName, double * BacklashValue, char * BacklaskStatus);
DLL long __stdcall PositionerBacklashSet (int SocketIndex, char * PositionerName, double BacklashValue);
DLL long __stdcall PositionerBacklashEnable (int SocketIndex, char * PositionerName);
DLL long __stdcall PositionerBacklashDisable (int SocketIndex, char * PositionerName);
DLL long __stdcall PositionerCorrectorNotchFiltersSet (int SocketIndex, char * PositionerName, double NotchFrequency1, double NotchBandwith1, double NotchGain1, double NotchFrequency2, double NotchBandwith2, double NotchGain2);
DLL long __stdcall PositionerCorrectorNotchFiltersGet (int SocketIndex, char * PositionerName, double * NotchFrequency1, double * NotchBandwith1, double * NotchGain1, double * NotchFrequency2, double * NotchBandwith2, double * NotchGain2);
DLL long __stdcall PositionerCorrectorPIDFFAccelerationSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainAcceleration);
DLL long __stdcall PositionerCorrectorPIDFFAccelerationGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainAcceleration);
DLL long __stdcall PositionerCorrectorPIDFFVelocitySet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity);
DLL long __stdcall PositionerCorrectorPIDFFVelocityGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity);
DLL long __stdcall PositionerCorrectorPIDDualFFVoltageSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double KD, double KS, double IntegrationTime, double DerivativeFilterCutOffFrequency, double GKP, double GKI, double GKD, double KForm, double FeedForwardGainVelocity, double FeedForwardGainAcceleration, double Friction);
DLL long __stdcall PositionerCorrectorPIDDualFFVoltageGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * KD, double * KS, double * IntegrationTime, double * DerivativeFilterCutOffFrequency, double * GKP, double * GKI, double * GKD, double * KForm, double * FeedForwardGainVelocity, double * FeedForwardGainAcceleration, double * Friction);
DLL long __stdcall PositionerCorrectorPIPositionSet (int SocketIndex, char * PositionerName, bool ClosedLoopStatus, double KP, double KI, double IntegrationTime);
DLL long __stdcall PositionerCorrectorPIPositionGet (int SocketIndex, char * PositionerName, bool * ClosedLoopStatus, double * KP, double * KI, double * IntegrationTime);
DLL long __stdcall PositionerCorrectorTypeGet (int SocketIndex, char * PositionerName, char * CorrectorType);
DLL long __stdcall PositionerCurrentVelocityAccelerationFiltersSet (int SocketIndex, char * PositionerName, double CurrentVelocityCutOffFrequency, double CurrentAccelerationCutOffFrequency);
DLL long __stdcall PositionerCurrentVelocityAccelerationFiltersGet (int SocketIndex, char * PositionerName, double * CurrentVelocityCutOffFrequency, double * CurrentAccelerationCutOffFrequency);
DLL long __stdcall PositionerEncoderAmplitudeValuesGet (int SocketIndex, char * PositionerName, double * MaxSinusAmplitude, double * CurrentSinusAmplitude, double * MaxCosinusAmplitude, double * CurrentCosinusAmplitude);
DLL long __stdcall PositionerEncoderCalibrationParametersGet (int SocketIndex, char * PositionerName, double * SinusOffset, double * CosinusOffset, double * DifferentialGain, double * PhaseCompensation);
DLL long __stdcall PositionerErrorGet (int SocketIndex, char * PositionerName, int * ErrorCode);
DLL long __stdcall PositionerErrorStringGet (int SocketIndex, int PositionerErrorCode, char * PositionerErrorString);
DLL long __stdcall PositionerHardwareStatusGet (int SocketIndex, char * PositionerName, int * HardwareStatus);
DLL long __stdcall PositionerHardwareStatusStringGet (int SocketIndex, int PositionerHardwareStatus, char * PositonerHardwareStatusString);
DLL long __stdcall PositionerHardInterpolatorFactorGet (int SocketIndex, char * PositionerName, int * InterpolationFactor);
DLL long __stdcall PositionerHardInterpolatorFactorSet (int SocketIndex, char * PositionerName, int InterpolationFactor);
DLL long __stdcall PositionerMaximumVelocityAndAccelerationGet (int SocketIndex, char * PositionerName, double * MaximumVelocity, double * MaximumAcceleration);
DLL long __stdcall PositionerMotionDoneGet (int SocketIndex, char * PositionerName, double * PositionWindow, double * VelocityWindow, double * CheckingTime, double * MeanPeriod, double * TimeOut);
DLL long __stdcall PositionerMotionDoneSet (int SocketIndex, char * PositionerName, double PositionWindow, double VelocityWindow, double CheckingTime, double MeanPeriod, double TimeOut);
DLL long __stdcall PositionerPositionCompareGet (int SocketIndex, char * PositionerName, double * MinimumPosition, double * MaximumPosition, double * PositionStep, bool * EnableState);
DLL long __stdcall PositionerPositionCompareSet (int SocketIndex, char * PositionerName, double MinimumPosition, double MaximumPosition, double PositionStep);
DLL long __stdcall PositionerPositionCompareEnable (int SocketIndex, char * PositionerName);
DLL long __stdcall PositionerPositionCompareDisable (int SocketIndex, char * PositionerName);
DLL long __stdcall PositionerSGammaExactVelocityAjustedDisplacementGet (int SocketIndex, char * PositionerName, double DesiredDisplacement, double * AdjustedDisplacement);
DLL long __stdcall PositionerSGammaParametersGet (int SocketIndex, char * PositionerName, double * Velocity, double * Acceleration, double * MinimumTjerkTime, double * MaximumTjerkTime);
DLL long __stdcall PositionerSGammaParametersSet (int SocketIndex, char * PositionerName, double Velocity, double Acceleration, double MinimumTjerkTime, double MaximumTjerkTime);
DLL long __stdcall PositionerSGammaPreviousMotionTimesGet (int SocketIndex, char * PositionerName, double * SettingTime, double * SettlingTime);
DLL long __stdcall PositionerUserTravelLimitsGet (int SocketIndex, char * PositionerName, double * UserMinimumTarget, double * UserMaximumTarget);
DLL long __stdcall PositionerUserTravelLimitsSet (int SocketIndex, char * PositionerName, double UserMinimumTarget, double UserMaximumTarget);
DLL long __stdcall MultipleAxesPVTVerification (int SocketIndex, char * GroupName, char * FileName);
DLL long __stdcall MultipleAxesPVTVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);
DLL long __stdcall MultipleAxesPVTExecution (int SocketIndex, char * GroupName, char * FileName, int ExecutionNumber);
DLL long __stdcall MultipleAxesPVTParametersGet (int SocketIndex, char * GroupName, char * FileName, int * CurrentElementNumber);
DLL long __stdcall SingleAxisSlaveModeEnable (int SocketIndex, char * GroupName);
DLL long __stdcall SingleAxisSlaveModeDisable (int SocketIndex, char * GroupName);
DLL long __stdcall SingleAxisSlaveParametersSet (int SocketIndex, char * GroupName, char * PositionerName, double Ratio);
DLL long __stdcall SingleAxisSlaveParametersGet (int SocketIndex, char * GroupName, char * PositionerName, double * Ratio);
DLL long __stdcall SpindleSlaveModeEnable (int SocketIndex, char * GroupName);
DLL long __stdcall SpindleSlaveModeDisable (int SocketIndex, char * GroupName);
DLL long __stdcall SpindleSlaveParametersSet (int SocketIndex, char * GroupName, char * PositionerName, double Ratio);
DLL long __stdcall SpindleSlaveParametersGet (int SocketIndex, char * GroupName, char * PositionerName, double * Ratio);
DLL long __stdcall GroupSpinParametersSet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL long __stdcall GroupSpinParametersGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL long __stdcall GroupSpinCurrentGet (int SocketIndex, char * GroupName, int NbElements, double Velocity[], double Acceleration[]);
DLL long __stdcall GroupSpinModeStop (int SocketIndex, char * GroupName, double Acceleration);
DLL long __stdcall XYLineArcVerification (int SocketIndex, char * GroupName, char * FileName);
DLL long __stdcall XYLineArcVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);
DLL long __stdcall XYLineArcExecution (int SocketIndex, char * GroupName, char * FileName, double Velocity, double Acceleration, int ExecutionNumber);
DLL long __stdcall XYLineArcParametersGet (int SocketIndex, char * GroupName, char * FileName, double * Velocity, double * Acceleration, int * CurrentElementNumber);
DLL long __stdcall XYZSplineVerification (int SocketIndex, char * GroupName, char * FileName);
DLL long __stdcall XYZSplineVerificationResultGet (int SocketIndex, char * PositionerName, char * FileName, double * MinimumPosition, double * MaximumPosition, double * MaximumVelocity, double * MaximumAcceleration);
DLL long __stdcall XYZSplineExecution (int SocketIndex, char * GroupName, char * FileName, double Velocity, double Acceleration);
DLL long __stdcall XYZSplineParametersGet (int SocketIndex, char * GroupName, char * FileName, double * Velocity, double * Acceleration, int * CurrentElementNumber);
DLL long __stdcall EEPROMCIESet (int SocketIndex, int CardNumber, char * ReferenceString);
DLL long __stdcall EEPROMDACOffsetCIESet (int SocketIndex, int PlugNumber, double DAC1Offset, double DAC2Offset);
DLL long __stdcall EEPROMDriverSet (int SocketIndex, int PlugNumber, char * ReferenceString);
DLL long __stdcall EEPROMINTSet (int SocketIndex, int CardNumber, char * ReferenceString);
DLL long __stdcall CPUCoreAndBoardSupplyVoltagesGet (int SocketIndex, double * VoltageCPUCore, double * SupplyVoltage1P5V, double * SupplyVoltage3P3V, double * SupplyVoltage5V, double * SupplyVoltage12V, double * SupplyVoltageM12V, double * SupplyVoltageM5V, double * SupplyVoltage5VSB);
DLL long __stdcall CPUTemperatureAndFanSpeedGet (int SocketIndex, double * CPUTemperature, double * CPUFanSpeed);
DLL long __stdcall ActionListGet (int SocketIndex, char * ActionList);
DLL long __stdcall ActionExtendedListGet (int SocketIndex, char * ActionList);
DLL long __stdcall APIExtendedListGet (int SocketIndex, char * Method);
DLL long __stdcall APIListGet (int SocketIndex, char * Method);
DLL long __stdcall ErrorListGet (int SocketIndex, char * ErrorsList);
DLL long __stdcall EventListGet (int SocketIndex, char * EventList);
DLL long __stdcall EventExtendedListGet (int SocketIndex, char * EventList);
DLL long __stdcall GatheringListGet (int SocketIndex, char * list);
DLL long __stdcall GatheringExtendedListGet (int SocketIndex, char * list);
DLL long __stdcall GatheringExternalListGet (int SocketIndex, char * list);
DLL long __stdcall GroupStatusListGet (int SocketIndex, char * GroupStatusList);
DLL long __stdcall HardwareInternalListGet (int SocketIndex, char * InternalHardwareList);
DLL long __stdcall HardwareDriverAndStageGet (int SocketIndex, int PlugNumber, char * DriverName, char * StageName);
DLL long __stdcall ObjectsListGet (int SocketIndex, char * ObjectsList);
DLL long __stdcall PositionerErrorListGet (int SocketIndex, char * PositionerErrorList);
DLL long __stdcall PositionerHardwareStatusListGet (int SocketIndex, char * PositionerHardwareStatusList);
DLL long __stdcall GatheringUserDatasGet (int SocketIndex, double * UserData1, double * UserData2, double * UserData3, double * UserData4, double * UserData5, double * UserData6, double * UserData7, double * UserData8);
DLL long __stdcall TestTCP (int SocketIndex, char * InputString, char * ReturnString);


#ifdef __cplusplus
}
#endif
