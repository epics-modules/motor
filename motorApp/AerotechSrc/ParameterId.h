/// \file ParameterId.h
/// \brief File contains parameter identifiers
///
/// This file is version dependant and needs to match the rest of the software

#ifndef __PARAMETER_ID_H__
#define __PARAMETER_ID_H__


/// \brief Represents a parameter identifier
typedef enum {
	/// \brief The AxisType parameter
	PARAMETERID_AxisType = ( (0 << 24) | 0 ),
	/// \brief The ReverseMotionDirection parameter
	PARAMETERID_ReverseMotionDirection = ( (0 << 24) | 1 ),
	/// \brief The CountsPerUnit parameter
	PARAMETERID_CountsPerUnit = ( (0 << 24) | 2 ),
	/// \brief The ServoRate parameter
	PARAMETERID_ServoRate = ( (0 << 24) | 3 ),
	/// \brief The ServoSetup parameter
	PARAMETERID_ServoSetup = ( (0 << 24) | 4 ),
	/// \brief The GainKpos parameter
	PARAMETERID_GainKpos = ( (0 << 24) | 5 ),
	/// \brief The GainKi parameter
	PARAMETERID_GainKi = ( (0 << 24) | 6 ),
	/// \brief The GainKp parameter
	PARAMETERID_GainKp = ( (0 << 24) | 7 ),
	/// \brief The GainVff parameter
	PARAMETERID_GainVff = ( (0 << 24) | 8 ),
	/// \brief The GainAff parameter
	PARAMETERID_GainAff = ( (0 << 24) | 9 ),
	/// \brief The GainKv parameter
	PARAMETERID_GainKv = ( (0 << 24) | 10 ),
	/// \brief The GainKpi parameter
	PARAMETERID_GainKpi = ( (0 << 24) | 11 ),
	/// \brief The ServoFilter0CoeffN0 parameter
	PARAMETERID_ServoFilter0CoeffN0 = ( (0 << 24) | 12 ),
	/// \brief The ServoFilter0CoeffN1 parameter
	PARAMETERID_ServoFilter0CoeffN1 = ( (0 << 24) | 13 ),
	/// \brief The ServoFilter0CoeffN2 parameter
	PARAMETERID_ServoFilter0CoeffN2 = ( (0 << 24) | 14 ),
	/// \brief The ServoFilter0CoeffD1 parameter
	PARAMETERID_ServoFilter0CoeffD1 = ( (0 << 24) | 15 ),
	/// \brief The ServoFilter0CoeffD2 parameter
	PARAMETERID_ServoFilter0CoeffD2 = ( (0 << 24) | 16 ),
	/// \brief The ServoFilter1CoeffN0 parameter
	PARAMETERID_ServoFilter1CoeffN0 = ( (0 << 24) | 17 ),
	/// \brief The ServoFilter1CoeffN1 parameter
	PARAMETERID_ServoFilter1CoeffN1 = ( (0 << 24) | 18 ),
	/// \brief The ServoFilter1CoeffN2 parameter
	PARAMETERID_ServoFilter1CoeffN2 = ( (0 << 24) | 19 ),
	/// \brief The ServoFilter1CoeffD1 parameter
	PARAMETERID_ServoFilter1CoeffD1 = ( (0 << 24) | 20 ),
	/// \brief The ServoFilter1CoeffD2 parameter
	PARAMETERID_ServoFilter1CoeffD2 = ( (0 << 24) | 21 ),
	/// \brief The AmplifierDeadtime parameter
	PARAMETERID_AmplifierDeadtime = ( (0 << 24) | 22 ),
	/// \brief The RolloverCounts parameter
	PARAMETERID_RolloverCounts = ( (0 << 24) | 23 ),
	/// \brief The CurrentGainKi parameter
	PARAMETERID_CurrentGainKi = ( (0 << 24) | 24 ),
	/// \brief The CurrentGainKp parameter
	PARAMETERID_CurrentGainKp = ( (0 << 24) | 25 ),
	/// \brief The FaultMask parameter
	PARAMETERID_FaultMask = ( (0 << 24) | 26 ),
	/// \brief The FaultMaskDisable parameter
	PARAMETERID_FaultMaskDisable = ( (0 << 24) | 27 ),
	/// \brief The FaultMaskDecel parameter
	PARAMETERID_FaultMaskDecel = ( (0 << 24) | 28 ),
	/// \brief The EnableBrakeControl parameter
	PARAMETERID_EnableBrakeControl = ( (0 << 24) | 29 ),
	/// \brief The FaultMaskOutput parameter
	PARAMETERID_FaultMaskOutput = ( (0 << 24) | 30 ),
	/// \brief The ESTOPFaultInput parameter
	PARAMETERID_ESTOPFaultInput = ( (0 << 24) | 31 ),
	/// \brief The PositionErrorThreshold parameter
	PARAMETERID_PositionErrorThreshold = ( (0 << 24) | 32 ),
	/// \brief The AverageCurrentThreshold parameter
	PARAMETERID_AverageCurrentThreshold = ( (0 << 24) | 33 ),
	/// \brief The AverageCurrentTime parameter
	PARAMETERID_AverageCurrentTime = ( (0 << 24) | 34 ),
	/// \brief The VelocityCommandThreshold parameter
	PARAMETERID_VelocityCommandThreshold = ( (0 << 24) | 35 ),
	/// \brief The VelocityErrorThreshold parameter
	PARAMETERID_VelocityErrorThreshold = ( (0 << 24) | 36 ),
	/// \brief The SoftwareLimitLow parameter
	PARAMETERID_SoftwareLimitLow = ( (0 << 24) | 37 ),
	/// \brief The SoftwareLimitHigh parameter
	PARAMETERID_SoftwareLimitHigh = ( (0 << 24) | 38 ),
	/// \brief The MaxCurrentClamp parameter
	PARAMETERID_MaxCurrentClamp = ( (0 << 24) | 39 ),
	/// \brief The InPositionDistance parameter
	PARAMETERID_InPositionDistance = ( (0 << 24) | 40 ),
	/// \brief The MotorType parameter
	PARAMETERID_MotorType = ( (0 << 24) | 41 ),
	/// \brief The CyclesPerRev parameter
	PARAMETERID_CyclesPerRev = ( (0 << 24) | 42 ),
	/// \brief The CountsPerRev parameter
	PARAMETERID_CountsPerRev = ( (0 << 24) | 43 ),
	/// \brief The CommutationOffset parameter
	PARAMETERID_CommutationOffset = ( (0 << 24) | 44 ),
	/// \brief The AutoMsetTime parameter
	PARAMETERID_AutoMsetTime = ( (0 << 24) | 45 ),
	/// \brief The AutoMsetCurrent parameter
	PARAMETERID_AutoMsetCurrent = ( (0 << 24) | 46 ),
	/// \brief The PositionFeedbackType parameter
	PARAMETERID_PositionFeedbackType = ( (0 << 24) | 47 ),
	/// \brief The PositionFeedbackChannel parameter
	PARAMETERID_PositionFeedbackChannel = ( (0 << 24) | 48 ),
	/// \brief The VelocityFeedbackType parameter
	PARAMETERID_VelocityFeedbackType = ( (0 << 24) | 49 ),
	/// \brief The VelocityFeedbackChannel parameter
	PARAMETERID_VelocityFeedbackChannel = ( (0 << 24) | 50 ),
	/// \brief The EncoderMultiplicationFactor parameter
	PARAMETERID_EncoderMultiplicationFactor = ( (0 << 24) | 51 ),
	/// \brief The EncoderSineGain parameter
	PARAMETERID_EncoderSineGain = ( (0 << 24) | 52 ),
	/// \brief The EncoderSineOffset parameter
	PARAMETERID_EncoderSineOffset = ( (0 << 24) | 53 ),
	/// \brief The EncoderCosineGain parameter
	PARAMETERID_EncoderCosineGain = ( (0 << 24) | 54 ),
	/// \brief The EncoderCosineOffset parameter
	PARAMETERID_EncoderCosineOffset = ( (0 << 24) | 55 ),
	/// \brief The EncoderPhase parameter
	PARAMETERID_EncoderPhase = ( (0 << 24) | 56 ),
	/// \brief The GantryMasterAxis parameter
	PARAMETERID_GantryMasterAxis = ( (0 << 24) | 57 ),
	/// \brief The LimitDecelDistance parameter
	PARAMETERID_LimitDecelDistance = ( (0 << 24) | 59 ),
	/// \brief The LimitDebounceTime parameter
	PARAMETERID_LimitDebounceTime = ( (0 << 24) | 60 ),
	/// \brief The EndOfTravelLimitSetup parameter
	PARAMETERID_EndOfTravelLimitSetup = ( (0 << 24) | 61 ),
	/// \brief The BacklashDistance parameter
	PARAMETERID_BacklashDistance = ( (0 << 24) | 62 ),
	/// \brief The FaultOutputSetup parameter
	PARAMETERID_FaultOutputSetup = ( (0 << 24) | 63 ),
	/// \brief The FaultOutputState parameter
	PARAMETERID_FaultOutputState = ( (0 << 24) | 64 ),
	/// \brief The IOSetup parameter
	PARAMETERID_IOSetup = ( (0 << 24) | 65 ),
	/// \brief The BrakeOutput parameter
	PARAMETERID_BrakeOutput = ( (0 << 24) | 66 ),
	/// \brief The EncoderDivider parameter
	PARAMETERID_EncoderDivider = ( (0 << 24) | 67 ),
	/// \brief The ExternalFaultDigitalInput parameter
	PARAMETERID_ExternalFaultDigitalInput = ( (0 << 24) | 68 ),
	/// \brief The BrakeDisableDelay parameter
	PARAMETERID_BrakeDisableDelay = ( (0 << 24) | 69 ),
	/// \brief The MaxJogDistance parameter
	PARAMETERID_MaxJogDistance = ( (0 << 24) | 70 ),
	/// \brief The DefaultSpeed parameter
	PARAMETERID_DefaultSpeed = ( (0 << 24) | 71 ),
	/// \brief The DefaultRampRate parameter
	PARAMETERID_DefaultRampRate = ( (0 << 24) | 72 ),
	/// \brief The AbortDecelRate parameter
	PARAMETERID_AbortDecelRate = ( (0 << 24) | 73 ),
	/// \brief The HomeType parameter
	PARAMETERID_HomeType = ( (0 << 24) | 74 ),
	/// \brief The HomeSetup parameter
	PARAMETERID_HomeSetup = ( (0 << 24) | 75 ),
	/// \brief The HomeSpeed parameter
	PARAMETERID_HomeSpeed = ( (0 << 24) | 76 ),
	/// \brief The HomeOffset parameter
	PARAMETERID_HomeOffset = ( (0 << 24) | 77 ),
	/// \brief The HomeRampRate parameter
	PARAMETERID_HomeRampRate = ( (0 << 24) | 78 ),
	/// \brief The DefaultWaitMode parameter
	PARAMETERID_DefaultWaitMode = ( (0 << 24) | 79 ),
	/// \brief The DefaultSCurve parameter
	PARAMETERID_DefaultSCurve = ( (0 << 24) | 80 ),
	/// \brief The DataCollectionPoints parameter
	PARAMETERID_DataCollectionPoints = ( (0 << 24) | 81 ),
	/// \brief The StepperResolution parameter
	PARAMETERID_StepperResolution = ( (0 << 24) | 83 ),
	/// \brief The StepperRunningCurrent parameter
	PARAMETERID_StepperRunningCurrent = ( (0 << 24) | 84 ),
	/// \brief The StepperHoldingCurrent parameter
	PARAMETERID_StepperHoldingCurrent = ( (0 << 24) | 85 ),
	/// \brief The StepperVerificationSpeed parameter
	PARAMETERID_StepperVerificationSpeed = ( (0 << 24) | 86 ),
	/// \brief The LimitDebounceDistance parameter
	PARAMETERID_LimitDebounceDistance = ( (0 << 24) | 87 ),
	/// \brief The ServoFilter2CoeffN0 parameter
	PARAMETERID_ServoFilter2CoeffN0 = ( (0 << 24) | 88 ),
	/// \brief The ServoFilter2CoeffN1 parameter
	PARAMETERID_ServoFilter2CoeffN1 = ( (0 << 24) | 89 ),
	/// \brief The ServoFilter2CoeffN2 parameter
	PARAMETERID_ServoFilter2CoeffN2 = ( (0 << 24) | 90 ),
	/// \brief The ServoFilter2CoeffD1 parameter
	PARAMETERID_ServoFilter2CoeffD1 = ( (0 << 24) | 91 ),
	/// \brief The ServoFilter2CoeffD2 parameter
	PARAMETERID_ServoFilter2CoeffD2 = ( (0 << 24) | 92 ),
	/// \brief The ServoFilter3CoeffN0 parameter
	PARAMETERID_ServoFilter3CoeffN0 = ( (0 << 24) | 93 ),
	/// \brief The ServoFilter3CoeffN1 parameter
	PARAMETERID_ServoFilter3CoeffN1 = ( (0 << 24) | 94 ),
	/// \brief The ServoFilter3CoeffN2 parameter
	PARAMETERID_ServoFilter3CoeffN2 = ( (0 << 24) | 95 ),
	/// \brief The ServoFilter3CoeffD1 parameter
	PARAMETERID_ServoFilter3CoeffD1 = ( (0 << 24) | 96 ),
	/// \brief The ServoFilter3CoeffD2 parameter
	PARAMETERID_ServoFilter3CoeffD2 = ( (0 << 24) | 97 ),
	/// \brief The GearCamSource parameter
	PARAMETERID_GearCamSource = ( (0 << 24) | 98 ),
	/// \brief The GearCamIndex parameter
	PARAMETERID_GearCamIndex = ( (0 << 24) | 99 ),
	/// \brief The GearCamScaleFactor parameter
	PARAMETERID_GearCamScaleFactor = ( (0 << 24) | 100 ),
	/// \brief The GearCamAnalogDeadband parameter
	PARAMETERID_GearCamAnalogDeadband = ( (0 << 24) | 105 ),
	/// \brief The PrintBufferSize parameter
	PARAMETERID_PrintBufferSize = ( (0 << 24) | 106 ),
	/// \brief The SerialPort0XonCharacter parameter
	PARAMETERID_SerialPort0XonCharacter = ( (0 << 24) | 109 ),
	/// \brief The SerialPort0XoffCharacter parameter
	PARAMETERID_SerialPort0XoffCharacter = ( (0 << 24) | 110 ),
	/// \brief The SerialPort0BaudRate parameter
	PARAMETERID_SerialPort0BaudRate = ( (0 << 24) | 111 ),
	/// \brief The SerialPort0Setup parameter
	PARAMETERID_SerialPort0Setup = ( (0 << 24) | 112 ),
	/// \brief The TaskExecutionSetup parameter
	PARAMETERID_TaskExecutionSetup = ( (0 << 24) | 113 ),
	/// \brief The CodeSize parameter
	PARAMETERID_CodeSize = ( (0 << 24) | 114 ),
	/// \brief The DataSize parameter
	PARAMETERID_DataSize = ( (0 << 24) | 115 ),
	/// \brief The StackSize parameter
	PARAMETERID_StackSize = ( (0 << 24) | 116 ),
	/// \brief The AutoRunProgram parameter
	PARAMETERID_AutoRunProgram = ( (0 << 24) | 118 ),
	/// \brief The MaxJogSpeed parameter
	PARAMETERID_MaxJogSpeed = ( (0 << 24) | 123 ),
	/// \brief The GlobalIntegers parameter
	PARAMETERID_GlobalIntegers = ( (0 << 24) | 124 ),
	/// \brief The GlobalDoubles parameter
	PARAMETERID_GlobalDoubles = ( (0 << 24) | 125 ),
	/// \brief The DecimalPlaces parameter
	PARAMETERID_DecimalPlaces = ( (0 << 24) | 126 ),
	/// \brief The TaskErrorAbortAxes parameter
	PARAMETERID_TaskErrorAbortAxes = ( (0 << 24) | 127 ),
	/// \brief The CalibrationFile1D parameter
	PARAMETERID_CalibrationFile1D = ( (0 << 24) | 128 ),
	/// \brief The UnitsName parameter
	PARAMETERID_UnitsName = ( (0 << 24) | 129 ),
	/// \brief The Socket2RemoteIPAddress parameter
	PARAMETERID_Socket2RemoteIPAddress = ( (0 << 24) | 130 ),
	/// \brief The Socket2Port parameter
	PARAMETERID_Socket2Port = ( (0 << 24) | 131 ),
	/// \brief The Socket2Setup parameter
	PARAMETERID_Socket2Setup = ( (0 << 24) | 132 ),
	/// \brief The Socket2TransmissionSize parameter
	PARAMETERID_Socket2TransmissionSize = ( (0 << 24) | 133 ),
	/// \brief The Socket3RemoteIPAddress parameter
	PARAMETERID_Socket3RemoteIPAddress = ( (0 << 24) | 134 ),
	/// \brief The Socket3Port parameter
	PARAMETERID_Socket3Port = ( (0 << 24) | 135 ),
	/// \brief The Socket3Setup parameter
	PARAMETERID_Socket3Setup = ( (0 << 24) | 136 ),
	/// \brief The Socket3TransmissionSize parameter
	PARAMETERID_Socket3TransmissionSize = ( (0 << 24) | 137 ),
	/// \brief The Socket2Timeout parameter
	PARAMETERID_Socket2Timeout = ( (0 << 24) | 138 ),
	/// \brief The Socket3Timeout parameter
	PARAMETERID_Socket3Timeout = ( (0 << 24) | 139 ),
	/// \brief The AxisName parameter
	PARAMETERID_AxisName = ( (0 << 24) | 140 ),
	/// \brief The UserInteger0 parameter
	PARAMETERID_UserInteger0 = ( (0 << 24) | 141 ),
	/// \brief The UserInteger1 parameter
	PARAMETERID_UserInteger1 = ( (0 << 24) | 142 ),
	/// \brief The UserDouble0 parameter
	PARAMETERID_UserDouble0 = ( (0 << 24) | 143 ),
	/// \brief The UserDouble1 parameter
	PARAMETERID_UserDouble1 = ( (0 << 24) | 144 ),
	/// \brief The UserString0 parameter
	PARAMETERID_UserString0 = ( (0 << 24) | 145 ),
	/// \brief The UserString1 parameter
	PARAMETERID_UserString1 = ( (0 << 24) | 146 ),
	/// \brief The EnDatEncoderSetup parameter
	PARAMETERID_EnDatEncoderSetup = ( (0 << 24) | 147 ),
	/// \brief The EnDatEncoderResolution parameter
	PARAMETERID_EnDatEncoderResolution = ( (0 << 24) | 148 ),
	/// \brief The EnDatEncoderTurns parameter
	PARAMETERID_EnDatEncoderTurns = ( (0 << 24) | 149 ),
	/// \brief The CommandSetup parameter
	PARAMETERID_CommandSetup = ( (0 << 24) | 150 ),
	/// \brief The SerialPort1XonCharacter parameter
	PARAMETERID_SerialPort1XonCharacter = ( (0 << 24) | 152 ),
	/// \brief The SerialPort1XoffCharacter parameter
	PARAMETERID_SerialPort1XoffCharacter = ( (0 << 24) | 153 ),
	/// \brief The SerialPort1BaudRate parameter
	PARAMETERID_SerialPort1BaudRate = ( (0 << 24) | 154 ),
	/// \brief The SerialPort1Setup parameter
	PARAMETERID_SerialPort1Setup = ( (0 << 24) | 155 ),
	/// \brief The RequiredAxes parameter
	PARAMETERID_RequiredAxes = ( (0 << 24) | 156 ),
	/// \brief The JoystickInput1MinVoltage parameter
	PARAMETERID_JoystickInput1MinVoltage = ( (0 << 24) | 157 ),
	/// \brief The JoystickInput1MaxVoltage parameter
	PARAMETERID_JoystickInput1MaxVoltage = ( (0 << 24) | 158 ),
	/// \brief The JoystickInput1Deadband parameter
	PARAMETERID_JoystickInput1Deadband = ( (0 << 24) | 159 ),
	/// \brief The JoystickInput0MinVoltage parameter
	PARAMETERID_JoystickInput0MinVoltage = ( (0 << 24) | 160 ),
	/// \brief The JoystickInput0MaxVoltage parameter
	PARAMETERID_JoystickInput0MaxVoltage = ( (0 << 24) | 161 ),
	/// \brief The JoystickInput0Deadband parameter
	PARAMETERID_JoystickInput0Deadband = ( (0 << 24) | 162 ),
	/// \brief The JoystickLowSpeed parameter
	PARAMETERID_JoystickLowSpeed = ( (0 << 24) | 163 ),
	/// \brief The JoystickHighSpeed parameter
	PARAMETERID_JoystickHighSpeed = ( (0 << 24) | 164 ),
	/// \brief The JoystickSetup parameter
	PARAMETERID_JoystickSetup = ( (0 << 24) | 165 ),
	/// \brief The HomePositionSet parameter
	PARAMETERID_HomePositionSet = ( (0 << 24) | 166 ),
	/// \brief The TaskTerminationAxes parameter
	PARAMETERID_TaskTerminationAxes = ( (0 << 24) | 167 ),
	/// \brief The TaskStopAbortAxes parameter
	PARAMETERID_TaskStopAbortAxes = ( (0 << 24) | 168 ),
	/// \brief The CalibrationFile2D parameter
	PARAMETERID_CalibrationFile2D = ( (0 << 24) | 169 ),
	/// \brief The FaultMaskDisableDelay parameter
	PARAMETERID_FaultMaskDisableDelay = ( (0 << 24) | 170 ),
	/// \brief The DefaultCoordinatedSpeed parameter
	PARAMETERID_DefaultCoordinatedSpeed = ( (0 << 24) | 171 ),
	/// \brief The DefaultCoordinatedRampRate parameter
	PARAMETERID_DefaultCoordinatedRampRate = ( (0 << 24) | 172 ),
	/// \brief The DefaultDependentCoordinatedRampRate parameter
	PARAMETERID_DefaultDependentCoordinatedRampRate = ( (0 << 24) | 173 ),
	/// \brief The GpibTerminatingCharacter parameter
	PARAMETERID_GpibTerminatingCharacter = ( (0 << 24) | 174 ),
	/// \brief The GpibPrimaryAddress parameter
	PARAMETERID_GpibPrimaryAddress = ( (0 << 24) | 175 ),
	/// \brief The GpibParallelResponse parameter
	PARAMETERID_GpibParallelResponse = ( (0 << 24) | 176 ),
	/// \brief The CommandTerminatingCharacter parameter
	PARAMETERID_CommandTerminatingCharacter = ( (0 << 24) | 177 ),
	/// \brief The CommandSuccessCharacter parameter
	PARAMETERID_CommandSuccessCharacter = ( (0 << 24) | 178 ),
	/// \brief The CommandInvalidCharacter parameter
	PARAMETERID_CommandInvalidCharacter = ( (0 << 24) | 179 ),
	/// \brief The CommandFaultCharacter parameter
	PARAMETERID_CommandFaultCharacter = ( (0 << 24) | 180 ),
	/// \brief The FaultAbortAxes parameter
	PARAMETERID_FaultAbortAxes = ( (0 << 24) | 182 ),
	/// \brief The HarmonicCancellation0Type parameter
	PARAMETERID_HarmonicCancellation0Type = ( (0 << 24) | 185 ),
	/// \brief The HarmonicCancellation0Period parameter
	PARAMETERID_HarmonicCancellation0Period = ( (0 << 24) | 186 ),
	/// \brief The HarmonicCancellation0Gain parameter
	PARAMETERID_HarmonicCancellation0Gain = ( (0 << 24) | 188 ),
	/// \brief The HarmonicCancellation0Phase parameter
	PARAMETERID_HarmonicCancellation0Phase = ( (0 << 24) | 189 ),
	/// \brief The HarmonicCancellation1Type parameter
	PARAMETERID_HarmonicCancellation1Type = ( (0 << 24) | 190 ),
	/// \brief The HarmonicCancellation1Period parameter
	PARAMETERID_HarmonicCancellation1Period = ( (0 << 24) | 191 ),
	/// \brief The HarmonicCancellation1Gain parameter
	PARAMETERID_HarmonicCancellation1Gain = ( (0 << 24) | 193 ),
	/// \brief The HarmonicCancellation1Phase parameter
	PARAMETERID_HarmonicCancellation1Phase = ( (0 << 24) | 194 ),
	/// \brief The HarmonicCancellation2Type parameter
	PARAMETERID_HarmonicCancellation2Type = ( (0 << 24) | 195 ),
	/// \brief The HarmonicCancellation2Period parameter
	PARAMETERID_HarmonicCancellation2Period = ( (0 << 24) | 196 ),
	/// \brief The HarmonicCancellation2Gain parameter
	PARAMETERID_HarmonicCancellation2Gain = ( (0 << 24) | 198 ),
	/// \brief The HarmonicCancellation2Phase parameter
	PARAMETERID_HarmonicCancellation2Phase = ( (0 << 24) | 199 ),
	/// \brief The CommandTimeout parameter
	PARAMETERID_CommandTimeout = ( (0 << 24) | 202 ),
	/// \brief The CommandTimeoutCharacter parameter
	PARAMETERID_CommandTimeoutCharacter = ( (0 << 24) | 203 ),
	/// \brief The ResolverReferenceGain parameter
	PARAMETERID_ResolverReferenceGain = ( (0 << 24) | 204 ),
	/// \brief The ResolverSetup parameter
	PARAMETERID_ResolverSetup = ( (0 << 24) | 205 ),
	/// \brief The ResolverReferencePhase parameter
	PARAMETERID_ResolverReferencePhase = ( (0 << 24) | 206 ),
	/// \brief The SoftwareLimitSetup parameter
	PARAMETERID_SoftwareLimitSetup = ( (0 << 24) | 210 ),
	/// \brief The SSINet1Setup parameter
	PARAMETERID_SSINet1Setup = ( (0 << 24) | 211 ),
	/// \brief The SSINet2Setup parameter
	PARAMETERID_SSINet2Setup = ( (0 << 24) | 212 ),
	/// \brief The EmulatedQuadratureDivider parameter
	PARAMETERID_EmulatedQuadratureDivider = ( (0 << 24) | 213 ),
	/// \brief The HarmonicCancellation3Type parameter
	PARAMETERID_HarmonicCancellation3Type = ( (0 << 24) | 214 ),
	/// \brief The HarmonicCancellation3Period parameter
	PARAMETERID_HarmonicCancellation3Period = ( (0 << 24) | 215 ),
	/// \brief The HarmonicCancellation3Gain parameter
	PARAMETERID_HarmonicCancellation3Gain = ( (0 << 24) | 217 ),
	/// \brief The HarmonicCancellation3Phase parameter
	PARAMETERID_HarmonicCancellation3Phase = ( (0 << 24) | 218 ),
	/// \brief The HarmonicCancellation4Type parameter
	PARAMETERID_HarmonicCancellation4Type = ( (0 << 24) | 219 ),
	/// \brief The HarmonicCancellation4Period parameter
	PARAMETERID_HarmonicCancellation4Period = ( (0 << 24) | 220 ),
	/// \brief The HarmonicCancellation4Gain parameter
	PARAMETERID_HarmonicCancellation4Gain = ( (0 << 24) | 222 ),
	/// \brief The HarmonicCancellation4Phase parameter
	PARAMETERID_HarmonicCancellation4Phase = ( (0 << 24) | 223 ),
	/// \brief The EnhancedThroughputChannel parameter
	PARAMETERID_EnhancedThroughputChannel = ( (0 << 24) | 224 ),
	/// \brief The EnhancedThroughputGain parameter
	PARAMETERID_EnhancedThroughputGain = ( (0 << 24) | 225 ),
	/// \brief The HarmonicCancellationSetup parameter
	PARAMETERID_HarmonicCancellationSetup = ( (0 << 24) | 226 ),
	/// \brief The EnhancedThroughputCurrentClamp parameter
	PARAMETERID_EnhancedThroughputCurrentClamp = ( (0 << 24) | 227 ),
	/// \brief The Analog0Filter0CoeffN0 parameter
	PARAMETERID_Analog0Filter0CoeffN0 = ( (0 << 24) | 228 ),
	/// \brief The Analog0Filter0CoeffN1 parameter
	PARAMETERID_Analog0Filter0CoeffN1 = ( (0 << 24) | 229 ),
	/// \brief The Analog0Filter0CoeffN2 parameter
	PARAMETERID_Analog0Filter0CoeffN2 = ( (0 << 24) | 230 ),
	/// \brief The Analog0Filter0CoeffD1 parameter
	PARAMETERID_Analog0Filter0CoeffD1 = ( (0 << 24) | 231 ),
	/// \brief The Analog0Filter0CoeffD2 parameter
	PARAMETERID_Analog0Filter0CoeffD2 = ( (0 << 24) | 232 ),
	/// \brief The Analog0Filter1CoeffN0 parameter
	PARAMETERID_Analog0Filter1CoeffN0 = ( (0 << 24) | 233 ),
	/// \brief The Analog0Filter1CoeffN1 parameter
	PARAMETERID_Analog0Filter1CoeffN1 = ( (0 << 24) | 234 ),
	/// \brief The Analog0Filter1CoeffN2 parameter
	PARAMETERID_Analog0Filter1CoeffN2 = ( (0 << 24) | 235 ),
	/// \brief The Analog0Filter1CoeffD1 parameter
	PARAMETERID_Analog0Filter1CoeffD1 = ( (0 << 24) | 236 ),
	/// \brief The Analog0Filter1CoeffD2 parameter
	PARAMETERID_Analog0Filter1CoeffD2 = ( (0 << 24) | 237 ),
	/// \brief The Analog1Filter0CoeffN0 parameter
	PARAMETERID_Analog1Filter0CoeffN0 = ( (0 << 24) | 238 ),
	/// \brief The Analog1Filter0CoeffN1 parameter
	PARAMETERID_Analog1Filter0CoeffN1 = ( (0 << 24) | 239 ),
	/// \brief The Analog1Filter0CoeffN2 parameter
	PARAMETERID_Analog1Filter0CoeffN2 = ( (0 << 24) | 240 ),
	/// \brief The Analog1Filter0CoeffD1 parameter
	PARAMETERID_Analog1Filter0CoeffD1 = ( (0 << 24) | 241 ),
	/// \brief The Analog1Filter0CoeffD2 parameter
	PARAMETERID_Analog1Filter0CoeffD2 = ( (0 << 24) | 242 ),
	/// \brief The Analog1Filter1CoeffN0 parameter
	PARAMETERID_Analog1Filter1CoeffN0 = ( (0 << 24) | 243 ),
	/// \brief The Analog1Filter1CoeffN1 parameter
	PARAMETERID_Analog1Filter1CoeffN1 = ( (0 << 24) | 244 ),
	/// \brief The Analog1Filter1CoeffN2 parameter
	PARAMETERID_Analog1Filter1CoeffN2 = ( (0 << 24) | 245 ),
	/// \brief The Analog1Filter1CoeffD1 parameter
	PARAMETERID_Analog1Filter1CoeffD1 = ( (0 << 24) | 246 ),
	/// \brief The Analog1Filter1CoeffD2 parameter
	PARAMETERID_Analog1Filter1CoeffD2 = ( (0 << 24) | 247 ),
	/// \brief The GlobalStrings parameter
	PARAMETERID_GlobalStrings = ( (0 << 24) | 248 ),
	/// \brief The DefaultCoordinatedRampMode parameter
	PARAMETERID_DefaultCoordinatedRampMode = ( (0 << 24) | 249 ),
	/// \brief The DefaultCoordinatedRampTime parameter
	PARAMETERID_DefaultCoordinatedRampTime = ( (0 << 24) | 250 ),
	/// \brief The DefaultCoordinatedRampDistance parameter
	PARAMETERID_DefaultCoordinatedRampDistance = ( (0 << 24) | 251 ),
	/// \brief The DefaultRampMode parameter
	PARAMETERID_DefaultRampMode = ( (0 << 24) | 252 ),
	/// \brief The DefaultRampTime parameter
	PARAMETERID_DefaultRampTime = ( (0 << 24) | 253 ),
	/// \brief The DefaultRampDistance parameter
	PARAMETERID_DefaultRampDistance = ( (0 << 24) | 254 ),
	/// \brief The ServoFilterSetup parameter
	PARAMETERID_ServoFilterSetup = ( (0 << 24) | 255 ),
	/// \brief The FeedbackSetup parameter
	PARAMETERID_FeedbackSetup = ( (0 << 24) | 256 ),
	/// \brief The EncoderMultiplierSetup parameter
	PARAMETERID_EncoderMultiplierSetup = ( (0 << 24) | 257 ),
	/// \brief The FaultSetup parameter
	PARAMETERID_FaultSetup = ( (0 << 24) | 258 ),
	/// \brief The ThresholdScheduleSetup parameter
	PARAMETERID_ThresholdScheduleSetup = ( (0 << 24) | 259 ),
	/// \brief The ThresholdRegion2High parameter
	PARAMETERID_ThresholdRegion2High = ( (0 << 24) | 260 ),
	/// \brief The ThresholdRegion2Low parameter
	PARAMETERID_ThresholdRegion2Low = ( (0 << 24) | 261 ),
	/// \brief The ThresholdRegion3GainKpos parameter
	PARAMETERID_ThresholdRegion3GainKpos = ( (0 << 24) | 262 ),
	/// \brief The ThresholdRegion3GainKp parameter
	PARAMETERID_ThresholdRegion3GainKp = ( (0 << 24) | 263 ),
	/// \brief The ThresholdRegion3GainKi parameter
	PARAMETERID_ThresholdRegion3GainKi = ( (0 << 24) | 264 ),
	/// \brief The ThresholdRegion3GainKpi parameter
	PARAMETERID_ThresholdRegion3GainKpi = ( (0 << 24) | 265 ),
	/// \brief The ThresholdRegion4High parameter
	PARAMETERID_ThresholdRegion4High = ( (0 << 24) | 266 ),
	/// \brief The ThresholdRegion4Low parameter
	PARAMETERID_ThresholdRegion4Low = ( (0 << 24) | 267 ),
	/// \brief The ThresholdRegion5GainKpos parameter
	PARAMETERID_ThresholdRegion5GainKpos = ( (0 << 24) | 268 ),
	/// \brief The ThresholdRegion5GainKp parameter
	PARAMETERID_ThresholdRegion5GainKp = ( (0 << 24) | 269 ),
	/// \brief The ThresholdRegion5GainKi parameter
	PARAMETERID_ThresholdRegion5GainKi = ( (0 << 24) | 270 ),
	/// \brief The ThresholdRegion5GainKpi parameter
	PARAMETERID_ThresholdRegion5GainKpi = ( (0 << 24) | 271 ),
	/// \brief The DynamicScheduleSetup parameter
	PARAMETERID_DynamicScheduleSetup = ( (0 << 24) | 272 ),
	/// \brief The DynamicGainKposScale parameter
	PARAMETERID_DynamicGainKposScale = ( (0 << 24) | 273 ),
	/// \brief The DynamicGainKpScale parameter
	PARAMETERID_DynamicGainKpScale = ( (0 << 24) | 274 ),
	/// \brief The DynamicGainKiScale parameter
	PARAMETERID_DynamicGainKiScale = ( (0 << 24) | 275 ),
	/// \brief The ServoFilter4CoeffN0 parameter
	PARAMETERID_ServoFilter4CoeffN0 = ( (0 << 24) | 276 ),
	/// \brief The ServoFilter4CoeffN1 parameter
	PARAMETERID_ServoFilter4CoeffN1 = ( (0 << 24) | 277 ),
	/// \brief The ServoFilter4CoeffN2 parameter
	PARAMETERID_ServoFilter4CoeffN2 = ( (0 << 24) | 278 ),
	/// \brief The ServoFilter4CoeffD1 parameter
	PARAMETERID_ServoFilter4CoeffD1 = ( (0 << 24) | 279 ),
	/// \brief The ServoFilter4CoeffD2 parameter
	PARAMETERID_ServoFilter4CoeffD2 = ( (0 << 24) | 280 ),
	/// \brief The ServoFilter5CoeffN0 parameter
	PARAMETERID_ServoFilter5CoeffN0 = ( (0 << 24) | 281 ),
	/// \brief The ServoFilter5CoeffN1 parameter
	PARAMETERID_ServoFilter5CoeffN1 = ( (0 << 24) | 282 ),
	/// \brief The ServoFilter5CoeffN2 parameter
	PARAMETERID_ServoFilter5CoeffN2 = ( (0 << 24) | 283 ),
	/// \brief The ServoFilter5CoeffD1 parameter
	PARAMETERID_ServoFilter5CoeffD1 = ( (0 << 24) | 284 ),
	/// \brief The ServoFilter5CoeffD2 parameter
	PARAMETERID_ServoFilter5CoeffD2 = ( (0 << 24) | 285 ),
	/// \brief The ServoFilter6CoeffN0 parameter
	PARAMETERID_ServoFilter6CoeffN0 = ( (0 << 24) | 286 ),
	/// \brief The ServoFilter6CoeffN1 parameter
	PARAMETERID_ServoFilter6CoeffN1 = ( (0 << 24) | 287 ),
	/// \brief The ServoFilter6CoeffN2 parameter
	PARAMETERID_ServoFilter6CoeffN2 = ( (0 << 24) | 288 ),
	/// \brief The ServoFilter6CoeffD1 parameter
	PARAMETERID_ServoFilter6CoeffD1 = ( (0 << 24) | 289 ),
	/// \brief The ServoFilter6CoeffD2 parameter
	PARAMETERID_ServoFilter6CoeffD2 = ( (0 << 24) | 290 ),
	/// \brief The ServoFilter7CoeffN0 parameter
	PARAMETERID_ServoFilter7CoeffN0 = ( (0 << 24) | 291 ),
	/// \brief The ServoFilter7CoeffN1 parameter
	PARAMETERID_ServoFilter7CoeffN1 = ( (0 << 24) | 292 ),
	/// \brief The ServoFilter7CoeffN2 parameter
	PARAMETERID_ServoFilter7CoeffN2 = ( (0 << 24) | 293 ),
	/// \brief The ServoFilter7CoeffD1 parameter
	PARAMETERID_ServoFilter7CoeffD1 = ( (0 << 24) | 294 ),
	/// \brief The ServoFilter7CoeffD2 parameter
	PARAMETERID_ServoFilter7CoeffD2 = ( (0 << 24) | 295 ),
	/// \brief The LinearAmpMaxPower parameter
	PARAMETERID_LinearAmpMaxPower = ( (0 << 24) | 296 ),
	/// \brief The LinearAmpDeratingFactor parameter
	PARAMETERID_LinearAmpDeratingFactor = ( (0 << 24) | 297 ),
	/// \brief The LinearAmpBusVoltage parameter
	PARAMETERID_LinearAmpBusVoltage = ( (0 << 24) | 298 ),
	/// \brief The MotorResistance parameter
	PARAMETERID_MotorResistance = ( (0 << 24) | 299 ),
	/// \brief The MotorBackEMFConstant parameter
	PARAMETERID_MotorBackEMFConstant = ( (0 << 24) | 300 ),
	/// \brief The GantrySetup parameter
	PARAMETERID_GantrySetup = ( (0 << 24) | 302 ),
	/// \brief The RolloverMode parameter
	PARAMETERID_RolloverMode = ( (0 << 24) | 303 ),
	/// \brief The ResolverCoarseChannel parameter
	PARAMETERID_ResolverCoarseChannel = ( (0 << 24) | 306 ),
	/// \brief The ResolverFeedbackRatio parameter
	PARAMETERID_ResolverFeedbackRatio = ( (0 << 24) | 307 ),
	/// \brief The ResolverFeedbackOffset parameter
	PARAMETERID_ResolverFeedbackOffset = ( (0 << 24) | 308 ),
	/// \brief The InPositionTime parameter
	PARAMETERID_InPositionTime = ( (0 << 24) | 319 ),
	/// \brief The ExternalFaultAnalogInput parameter
	PARAMETERID_ExternalFaultAnalogInput = ( (0 << 24) | 424 ),
	/// \brief The ExternalFaultThreshold parameter
	PARAMETERID_ExternalFaultThreshold = ( (0 << 24) | 425 ),
	/// \brief The DisplayAxes parameter
	PARAMETERID_DisplayAxes = ( (0 << 24) | 426 ),
	/// \brief The DefaultDependentCoordinatedSpeed parameter
	PARAMETERID_DefaultDependentCoordinatedSpeed = ( (0 << 24) | 427 ),
	/// \brief The AnalogFilterSetup parameter
	PARAMETERID_AnalogFilterSetup = ( (0 << 24) | 482 ),
	/// \brief The ModbusMasterSlaveIPAddress parameter
	PARAMETERID_ModbusMasterSlaveIPAddress = ( (0 << 24) | 489 ),
	/// \brief The ModbusMasterSlavePort parameter
	PARAMETERID_ModbusMasterSlavePort = ( (0 << 24) | 490 ),
	/// \brief The ModbusMasterSlaveID parameter
	PARAMETERID_ModbusMasterSlaveID = ( (0 << 24) | 491 ),
	/// \brief The ModbusMasterInputWords parameter
	PARAMETERID_ModbusMasterInputWords = ( (0 << 24) | 492 ),
	/// \brief The ModbusMasterOutputWords parameter
	PARAMETERID_ModbusMasterOutputWords = ( (0 << 24) | 493 ),
	/// \brief The ModbusMasterInputBits parameter
	PARAMETERID_ModbusMasterInputBits = ( (0 << 24) | 494 ),
	/// \brief The ModbusMasterOutputBits parameter
	PARAMETERID_ModbusMasterOutputBits = ( (0 << 24) | 495 ),
	/// \brief The ModbusMasterSetup parameter
	PARAMETERID_ModbusMasterSetup = ( (0 << 24) | 496 ),
	/// \brief The ModbusMasterVirtualInputs parameter
	PARAMETERID_ModbusMasterVirtualInputs = ( (0 << 24) | 499 ),
	/// \brief The ModbusMasterVirtualOutputs parameter
	PARAMETERID_ModbusMasterVirtualOutputs = ( (0 << 24) | 500 ),
	/// \brief The ModbusMasterOutputWordsSections parameter
	PARAMETERID_ModbusMasterOutputWordsSections = ( (0 << 24) | 501 ),
	/// \brief The ModbusMasterOutputBitsSections parameter
	PARAMETERID_ModbusMasterOutputBitsSections = ( (0 << 24) | 502 ),
	/// \brief The ModbusMasterRWReadOffset parameter
	PARAMETERID_ModbusMasterRWReadOffset = ( (0 << 24) | 503 ),
	/// \brief The ModbusMasterInputWordsOffset parameter
	PARAMETERID_ModbusMasterInputWordsOffset = ( (0 << 24) | 504 ),
	/// \brief The ModbusMasterOutputWordsOffset parameter
	PARAMETERID_ModbusMasterOutputWordsOffset = ( (0 << 24) | 505 ),
	/// \brief The ModbusMasterInputBitsOffset parameter
	PARAMETERID_ModbusMasterInputBitsOffset = ( (0 << 24) | 506 ),
	/// \brief The ModbusMasterOutputBitsOffset parameter
	PARAMETERID_ModbusMasterOutputBitsOffset = ( (0 << 24) | 507 ),
	/// \brief The ModbusMasterStatusWordsOffset parameter
	PARAMETERID_ModbusMasterStatusWordsOffset = ( (0 << 24) | 508 ),
	/// \brief The ModbusMasterStatusBitsOffset parameter
	PARAMETERID_ModbusMasterStatusBitsOffset = ( (0 << 24) | 509 ),
	/// \brief The ModbusMasterVirtualInputsOffset parameter
	PARAMETERID_ModbusMasterVirtualInputsOffset = ( (0 << 24) | 510 ),
	/// \brief The ModbusMasterVirtualOutputsOffset parameter
	PARAMETERID_ModbusMasterVirtualOutputsOffset = ( (0 << 24) | 511 ),
	/// \brief The ModbusMasterRWWriteOffset parameter
	PARAMETERID_ModbusMasterRWWriteOffset = ( (0 << 24) | 512 ),
	/// \brief The ModbusMasterFunctions parameter
	PARAMETERID_ModbusMasterFunctions = ( (0 << 24) | 513 ),
	/// \brief The ModbusMasterSlaveType parameter
	PARAMETERID_ModbusMasterSlaveType = ( (0 << 24) | 514 ),
	/// \brief The ModbusSlaveUnitID parameter
	PARAMETERID_ModbusSlaveUnitID = ( (0 << 24) | 516 ),
	/// \brief The ModbusSlaveInputWords parameter
	PARAMETERID_ModbusSlaveInputWords = ( (0 << 24) | 517 ),
	/// \brief The ModbusSlaveOutputWords parameter
	PARAMETERID_ModbusSlaveOutputWords = ( (0 << 24) | 518 ),
	/// \brief The ModbusSlaveInputBits parameter
	PARAMETERID_ModbusSlaveInputBits = ( (0 << 24) | 519 ),
	/// \brief The ModbusSlaveOutputBits parameter
	PARAMETERID_ModbusSlaveOutputBits = ( (0 << 24) | 520 ),
	/// \brief The ModbusSlaveInputWordsOffset parameter
	PARAMETERID_ModbusSlaveInputWordsOffset = ( (0 << 24) | 521 ),
	/// \brief The ModbusSlaveOutputWordsOffset parameter
	PARAMETERID_ModbusSlaveOutputWordsOffset = ( (0 << 24) | 522 ),
	/// \brief The ModbusSlaveInputBitsOffset parameter
	PARAMETERID_ModbusSlaveInputBitsOffset = ( (0 << 24) | 523 ),
	/// \brief The ModbusSlaveOutputBitsOffset parameter
	PARAMETERID_ModbusSlaveOutputBitsOffset = ( (0 << 24) | 524 ),
	/// \brief The ModbusSlaveRWReadOffset parameter
	PARAMETERID_ModbusSlaveRWReadOffset = ( (0 << 24) | 525 ),
	/// \brief The ModbusSlaveRWWriteOffset parameter
	PARAMETERID_ModbusSlaveRWWriteOffset = ( (0 << 24) | 526 ),
	/// \brief The CurrentOffsetA parameter
	PARAMETERID_CurrentOffsetA = ( (0 << 24) | 662 ),
	/// \brief The CurrentOffsetB parameter
	PARAMETERID_CurrentOffsetB = ( (0 << 24) | 663 ),
	/// \brief The CommandShaperSetup parameter
	PARAMETERID_CommandShaperSetup = ( (0 << 24) | 666 ),
	/// \brief The CommandShaperTime00 parameter
	PARAMETERID_CommandShaperTime00 = ( (0 << 24) | 667 ),
	/// \brief The CommandShaperTime01 parameter
	PARAMETERID_CommandShaperTime01 = ( (0 << 24) | 668 ),
	/// \brief The CommandShaperTime02 parameter
	PARAMETERID_CommandShaperTime02 = ( (0 << 24) | 669 ),
	/// \brief The CommandShaperTime03 parameter
	PARAMETERID_CommandShaperTime03 = ( (0 << 24) | 670 ),
	/// \brief The CommandShaperTime04 parameter
	PARAMETERID_CommandShaperTime04 = ( (0 << 24) | 671 ),
	/// \brief The CommandShaperTime05 parameter
	PARAMETERID_CommandShaperTime05 = ( (0 << 24) | 672 ),
	/// \brief The CommandShaperTime06 parameter
	PARAMETERID_CommandShaperTime06 = ( (0 << 24) | 673 ),
	/// \brief The CommandShaperTime07 parameter
	PARAMETERID_CommandShaperTime07 = ( (0 << 24) | 674 ),
	/// \brief The CommandShaperTime08 parameter
	PARAMETERID_CommandShaperTime08 = ( (0 << 24) | 675 ),
	/// \brief The CommandShaperTime09 parameter
	PARAMETERID_CommandShaperTime09 = ( (0 << 24) | 676 ),
	/// \brief The CommandShaperTime10 parameter
	PARAMETERID_CommandShaperTime10 = ( (0 << 24) | 677 ),
	/// \brief The CommandShaperTime11 parameter
	PARAMETERID_CommandShaperTime11 = ( (0 << 24) | 678 ),
	/// \brief The CommandShaperTime12 parameter
	PARAMETERID_CommandShaperTime12 = ( (0 << 24) | 679 ),
	/// \brief The CommandShaperTime13 parameter
	PARAMETERID_CommandShaperTime13 = ( (0 << 24) | 680 ),
	/// \brief The CommandShaperTime14 parameter
	PARAMETERID_CommandShaperTime14 = ( (0 << 24) | 681 ),
	/// \brief The CommandShaperTime15 parameter
	PARAMETERID_CommandShaperTime15 = ( (0 << 24) | 682 ),
	/// \brief The CommandShaperCoeff00 parameter
	PARAMETERID_CommandShaperCoeff00 = ( (0 << 24) | 683 ),
	/// \brief The CommandShaperCoeff01 parameter
	PARAMETERID_CommandShaperCoeff01 = ( (0 << 24) | 684 ),
	/// \brief The CommandShaperCoeff02 parameter
	PARAMETERID_CommandShaperCoeff02 = ( (0 << 24) | 685 ),
	/// \brief The CommandShaperCoeff03 parameter
	PARAMETERID_CommandShaperCoeff03 = ( (0 << 24) | 686 ),
	/// \brief The CommandShaperCoeff04 parameter
	PARAMETERID_CommandShaperCoeff04 = ( (0 << 24) | 687 ),
	/// \brief The CommandShaperCoeff05 parameter
	PARAMETERID_CommandShaperCoeff05 = ( (0 << 24) | 688 ),
	/// \brief The CommandShaperCoeff06 parameter
	PARAMETERID_CommandShaperCoeff06 = ( (0 << 24) | 689 ),
	/// \brief The CommandShaperCoeff07 parameter
	PARAMETERID_CommandShaperCoeff07 = ( (0 << 24) | 690 ),
	/// \brief The CommandShaperCoeff08 parameter
	PARAMETERID_CommandShaperCoeff08 = ( (0 << 24) | 691 ),
	/// \brief The CommandShaperCoeff09 parameter
	PARAMETERID_CommandShaperCoeff09 = ( (0 << 24) | 692 ),
	/// \brief The CommandShaperCoeff10 parameter
	PARAMETERID_CommandShaperCoeff10 = ( (0 << 24) | 693 ),
	/// \brief The CommandShaperCoeff11 parameter
	PARAMETERID_CommandShaperCoeff11 = ( (0 << 24) | 694 ),
	/// \brief The CommandShaperCoeff12 parameter
	PARAMETERID_CommandShaperCoeff12 = ( (0 << 24) | 695 ),
	/// \brief The CommandShaperCoeff13 parameter
	PARAMETERID_CommandShaperCoeff13 = ( (0 << 24) | 696 ),
	/// \brief The CommandShaperCoeff14 parameter
	PARAMETERID_CommandShaperCoeff14 = ( (0 << 24) | 697 ),
	/// \brief The CommandShaperCoeff15 parameter
	PARAMETERID_CommandShaperCoeff15 = ( (0 << 24) | 698 ),
	/// \brief The CommandShaper0Type parameter
	PARAMETERID_CommandShaper0Type = ( (0 << 24) | 703 ),
	/// \brief The CommandShaper0Frequency parameter
	PARAMETERID_CommandShaper0Frequency = ( (0 << 24) | 704 ),
	/// \brief The CommandShaper0Damping parameter
	PARAMETERID_CommandShaper0Damping = ( (0 << 24) | 705 ),
	/// \brief The CommandShaper1Type parameter
	PARAMETERID_CommandShaper1Type = ( (0 << 24) | 706 ),
	/// \brief The CommandShaper1Frequency parameter
	PARAMETERID_CommandShaper1Frequency = ( (0 << 24) | 707 ),
	/// \brief The CommandShaper1Damping parameter
	PARAMETERID_CommandShaper1Damping = ( (0 << 24) | 708 ),
	/// \brief The ResoluteEncoderSetup parameter
	PARAMETERID_ResoluteEncoderSetup = ( (0 << 24) | 715 ),
	/// \brief The ResoluteEncoderResolution parameter
	PARAMETERID_ResoluteEncoderResolution = ( (0 << 24) | 716 ),
	/// \brief The ResoluteEncoderUserResolution parameter
	PARAMETERID_ResoluteEncoderUserResolution = ( (0 << 24) | 717 ),
	/// \brief The AutofocusInput parameter
	PARAMETERID_AutofocusInput = ( (0 << 24) | 721 ),
	/// \brief The AutofocusTarget parameter
	PARAMETERID_AutofocusTarget = ( (0 << 24) | 722 ),
	/// \brief The AutofocusDeadband parameter
	PARAMETERID_AutofocusDeadband = ( (0 << 24) | 723 ),
	/// \brief The AutofocusGainKi parameter
	PARAMETERID_AutofocusGainKi = ( (0 << 24) | 724 ),
	/// \brief The AutofocusGainKp parameter
	PARAMETERID_AutofocusGainKp = ( (0 << 24) | 725 ),
	/// \brief The AutofocusLimitLow parameter
	PARAMETERID_AutofocusLimitLow = ( (0 << 24) | 726 ),
	/// \brief The AutofocusLimitHigh parameter
	PARAMETERID_AutofocusLimitHigh = ( (0 << 24) | 727 ),
	/// \brief The AutofocusSpeedClamp parameter
	PARAMETERID_AutofocusSpeedClamp = ( (0 << 24) | 728 ),
	/// \brief The AutofocusHoldInput parameter
	PARAMETERID_AutofocusHoldInput = ( (0 << 24) | 729 ),
	/// \brief The AutofocusSetup parameter
	PARAMETERID_AutofocusSetup = ( (0 << 24) | 730 ),
	/// \brief The ExternalSyncFrequency parameter
	PARAMETERID_ExternalSyncFrequency = ( (0 << 24) | 731 ),
	/// \brief The GainPff parameter
	PARAMETERID_GainPff = ( (0 << 24) | 762 ),
	/// \brief The AutofocusInitialRampTime parameter
	PARAMETERID_AutofocusInitialRampTime = ( (0 << 24) | 763 ),
} PARAMETERID;

#endif // __PARAMETER_ID_H__
