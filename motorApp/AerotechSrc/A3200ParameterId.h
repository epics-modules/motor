/// \file A3200ParameterId.h
/// \brief Contains parameter identifiers
///
/// This file is version dependent and needs to match the rest of the software
/// 
/// Copyright (c) Aerotech, Inc. 2010-2013.
/// 

#ifndef __A3200_PARAMETER_ID_H__
#define __A3200_PARAMETER_ID_H__


/// \brief Represents a parameter identifier
typedef enum {
	/// \brief The DataCollectionPoints parameter
	PARAMETERID_DataCollectionPoints = ( (2 << 24) | 0 ),
	/// \brief The DataCollectionItems parameter
	PARAMETERID_DataCollectionItems = ( (2 << 24) | 1 ),
	/// \brief The GlobalDoubles parameter
	PARAMETERID_GlobalDoubles = ( (2 << 24) | 21 ),
	/// \brief The CommandPort parameter
	PARAMETERID_CommandPort = ( (2 << 24) | 24 ),
	/// \brief The UserDouble0 parameter
	PARAMETERID_UserDouble0 = ( (2 << 24) | 35 ),
	/// \brief The UserDouble1 parameter
	PARAMETERID_UserDouble1 = ( (2 << 24) | 36 ),
	/// \brief The UserString0 parameter
	PARAMETERID_UserString0 = ( (2 << 24) | 37 ),
	/// \brief The UserString1 parameter
	PARAMETERID_UserString1 = ( (2 << 24) | 38 ),
	/// \brief The CommandSetup parameter
	PARAMETERID_CommandSetup = ( (2 << 24) | 39 ),
	/// \brief The RequiredAxes parameter
	PARAMETERID_RequiredAxes = ( (2 << 24) | 45 ),
	/// \brief The CommandTerminatingCharacter parameter
	PARAMETERID_CommandTerminatingCharacter = ( (2 << 24) | 51 ),
	/// \brief The CommandSuccessCharacter parameter
	PARAMETERID_CommandSuccessCharacter = ( (2 << 24) | 52 ),
	/// \brief The CommandInvalidCharacter parameter
	PARAMETERID_CommandInvalidCharacter = ( (2 << 24) | 53 ),
	/// \brief The CommandFaultCharacter parameter
	PARAMETERID_CommandFaultCharacter = ( (2 << 24) | 54 ),
	/// \brief The GlobalStrings parameter
	PARAMETERID_GlobalStrings = ( (2 << 24) | 65 ),
	/// \brief The SystemCompatibility parameter
	PARAMETERID_SystemCompatibility = ( (2 << 24) | 66 ),
	/// \brief The DependentSpeedScaleFactor parameter
	PARAMETERID_DependentSpeedScaleFactor = ( (2 << 24) | 67 ),
	/// \brief The GlobalAxisPoints parameter
	PARAMETERID_GlobalAxisPoints = ( (2 << 24) | 68 ),
	/// \brief The CallbackTimeout parameter
	PARAMETERID_CallbackTimeout = ( (2 << 24) | 69 ),
	/// \brief The CannedFunctions parameter
	PARAMETERID_CannedFunctions = ( (2 << 24) | 70 ),
	/// \brief The DisplayAxes parameter
	PARAMETERID_DisplayAxes = ( (2 << 24) | 71 ),
	/// \brief The SecondaryUnitsScaleFactor parameter
	PARAMETERID_SecondaryUnitsScaleFactor = ( (2 << 24) | 75 ),
	/// \brief The PrimaryUnitsMapping parameter
	PARAMETERID_PrimaryUnitsMapping = ( (2 << 24) | 76 ),
	/// \brief The SecondaryUnitsName parameter
	PARAMETERID_SecondaryUnitsName = ( (2 << 24) | 77 ),
	/// \brief The EnabledTasks parameter
	PARAMETERID_EnabledTasks = ( (2 << 24) | 236 ),
	/// \brief The FaultAckMoveOutOfLimit parameter
	PARAMETERID_FaultAckMoveOutOfLimit = ( (2 << 24) | 237 ),
	/// \brief The UserDouble2 parameter
	PARAMETERID_UserDouble2 = ( (2 << 24) | 238 ),
	/// \brief The UserDouble3 parameter
	PARAMETERID_UserDouble3 = ( (2 << 24) | 239 ),
	/// \brief The UserString2 parameter
	PARAMETERID_UserString2 = ( (2 << 24) | 240 ),
	/// \brief The UserString3 parameter
	PARAMETERID_UserString3 = ( (2 << 24) | 241 ),
	/// \brief The PLCReservedTasks parameter
	PARAMETERID_PLCReservedTasks = ( (2 << 24) | 243 ),
	/// \brief The SoftwareExternalFaultInput parameter
	PARAMETERID_SoftwareExternalFaultInput = ( (2 << 24) | 244 ),
	/// \brief The SignalLogSetup parameter
	PARAMETERID_SignalLogSetup = ( (2 << 24) | 245 ),
	/// \brief The SignalLogAxes parameter
	PARAMETERID_SignalLogAxes = ( (2 << 24) | 246 ),
	/// \brief The SignalLogTasks parameter
	PARAMETERID_SignalLogTasks = ( (2 << 24) | 247 ),
	/// \brief The SignalLogFaultMaskTrigger parameter
	PARAMETERID_SignalLogFaultMaskTrigger = ( (2 << 24) | 248 ),
	/// \brief The SignalLogSamplePeriod parameter
	PARAMETERID_SignalLogSamplePeriod = ( (2 << 24) | 249 ),
	/// \brief The SignalLogPointsBeforeTrigger parameter
	PARAMETERID_SignalLogPointsBeforeTrigger = ( (2 << 24) | 250 ),
	/// \brief The SignalLogPointsAfterTrigger parameter
	PARAMETERID_SignalLogPointsAfterTrigger = ( (2 << 24) | 251 ),
	/// \brief The MasterClockCorrectionFactor parameter
	PARAMETERID_MasterClockCorrectionFactor = ( (2 << 24) | 252 ),
	/// \brief The WebServerSetup parameter
	PARAMETERID_WebServerSetup = ( (2 << 24) | 261 ),
	/// \brief The WebServerPort parameter
	PARAMETERID_WebServerPort = ( (2 << 24) | 262 ),
	/// \brief The DefaultWaitMode parameter
	PARAMETERID_DefaultWaitMode = ( (1 << 24) | 0 ),
	/// \brief The DefaultSCurve parameter
	PARAMETERID_DefaultSCurve = ( (1 << 24) | 1 ),
	/// \brief The TaskErrorAbortAxes parameter
	PARAMETERID_TaskErrorAbortAxes = ( (1 << 24) | 7 ),
	/// \brief The JoystickInput1MinVoltage parameter
	PARAMETERID_JoystickInput1MinVoltage = ( (1 << 24) | 8 ),
	/// \brief The JoystickInput1MaxVoltage parameter
	PARAMETERID_JoystickInput1MaxVoltage = ( (1 << 24) | 9 ),
	/// \brief The JoystickInput1Deadband parameter
	PARAMETERID_JoystickInput1Deadband = ( (1 << 24) | 10 ),
	/// \brief The JoystickInput0MinVoltage parameter
	PARAMETERID_JoystickInput0MinVoltage = ( (1 << 24) | 11 ),
	/// \brief The JoystickInput0MaxVoltage parameter
	PARAMETERID_JoystickInput0MaxVoltage = ( (1 << 24) | 12 ),
	/// \brief The JoystickInput0Deadband parameter
	PARAMETERID_JoystickInput0Deadband = ( (1 << 24) | 13 ),
	/// \brief The TaskTerminationAxes parameter
	PARAMETERID_TaskTerminationAxes = ( (1 << 24) | 14 ),
	/// \brief The TaskStopAbortAxes parameter
	PARAMETERID_TaskStopAbortAxes = ( (1 << 24) | 15 ),
	/// \brief The DefaultCoordinatedSpeed parameter
	PARAMETERID_DefaultCoordinatedSpeed = ( (1 << 24) | 16 ),
	/// \brief The DefaultCoordinatedRampRate parameter
	PARAMETERID_DefaultCoordinatedRampRate = ( (1 << 24) | 17 ),
	/// \brief The DefaultDependentCoordinatedRampRate parameter
	PARAMETERID_DefaultDependentCoordinatedRampRate = ( (1 << 24) | 18 ),
	/// \brief The DefaultCoordinatedRampMode parameter
	PARAMETERID_DefaultCoordinatedRampMode = ( (1 << 24) | 19 ),
	/// \brief The DefaultCoordinatedRampTime parameter
	PARAMETERID_DefaultCoordinatedRampTime = ( (1 << 24) | 20 ),
	/// \brief The HCScanIncrement parameter
	PARAMETERID_HCScanIncrement = ( (1 << 24) | 22 ),
	/// \brief The HCMaxDisplacement parameter
	PARAMETERID_HCMaxDisplacement = ( (1 << 24) | 23 ),
	/// \brief The HCThreshold parameter
	PARAMETERID_HCThreshold = ( (1 << 24) | 24 ),
	/// \brief The HCAxis parameter
	PARAMETERID_HCAxis = ( (1 << 24) | 25 ),
	/// \brief The HCInputMode parameter
	PARAMETERID_HCInputMode = ( (1 << 24) | 26 ),
	/// \brief The HCInputChannelNum parameter
	PARAMETERID_HCInputChannelNum = ( (1 << 24) | 27 ),
	/// \brief The HCInvertSearch parameter
	PARAMETERID_HCInvertSearch = ( (1 << 24) | 28 ),
	/// \brief The HCWholeWindow parameter
	PARAMETERID_HCWholeWindow = ( (1 << 24) | 29 ),
	/// \brief The HCDelayTime parameter
	PARAMETERID_HCDelayTime = ( (1 << 24) | 31 ),
	/// \brief The SRMaxRadius parameter
	PARAMETERID_SRMaxRadius = ( (1 << 24) | 32 ),
	/// \brief The SRNumSpirals parameter
	PARAMETERID_SRNumSpirals = ( (1 << 24) | 33 ),
	/// \brief The SRSegmentLength parameter
	PARAMETERID_SRSegmentLength = ( (1 << 24) | 34 ),
	/// \brief The SRThreshold parameter
	PARAMETERID_SRThreshold = ( (1 << 24) | 35 ),
	/// \brief The SRAxis1 parameter
	PARAMETERID_SRAxis1 = ( (1 << 24) | 36 ),
	/// \brief The SRAxis2 parameter
	PARAMETERID_SRAxis2 = ( (1 << 24) | 37 ),
	/// \brief The SRInputMode parameter
	PARAMETERID_SRInputMode = ( (1 << 24) | 38 ),
	/// \brief The SRInputChannelNum parameter
	PARAMETERID_SRInputChannelNum = ( (1 << 24) | 39 ),
	/// \brief The SRInvertSearch parameter
	PARAMETERID_SRInvertSearch = ( (1 << 24) | 40 ),
	/// \brief The SRMotionType parameter
	PARAMETERID_SRMotionType = ( (1 << 24) | 41 ),
	/// \brief The SRDelayTime parameter
	PARAMETERID_SRDelayTime = ( (1 << 24) | 43 ),
	/// \brief The SFEndRadius parameter
	PARAMETERID_SFEndRadius = ( (1 << 24) | 44 ),
	/// \brief The SFNumSpirals parameter
	PARAMETERID_SFNumSpirals = ( (1 << 24) | 45 ),
	/// \brief The SFSegmentLength parameter
	PARAMETERID_SFSegmentLength = ( (1 << 24) | 46 ),
	/// \brief The SFAxis1 parameter
	PARAMETERID_SFAxis1 = ( (1 << 24) | 47 ),
	/// \brief The SFAxis2 parameter
	PARAMETERID_SFAxis2 = ( (1 << 24) | 48 ),
	/// \brief The SFInputMode parameter
	PARAMETERID_SFInputMode = ( (1 << 24) | 49 ),
	/// \brief The SFInputChannelNum parameter
	PARAMETERID_SFInputChannelNum = ( (1 << 24) | 50 ),
	/// \brief The SFInvertSearch parameter
	PARAMETERID_SFInvertSearch = ( (1 << 24) | 51 ),
	/// \brief The SFMotionType parameter
	PARAMETERID_SFMotionType = ( (1 << 24) | 52 ),
	/// \brief The SFDelayTime parameter
	PARAMETERID_SFDelayTime = ( (1 << 24) | 54 ),
	/// \brief The FASelectAxis1 parameter
	PARAMETERID_FASelectAxis1 = ( (1 << 24) | 55 ),
	/// \brief The FASelectAxis2 parameter
	PARAMETERID_FASelectAxis2 = ( (1 << 24) | 56 ),
	/// \brief The FASelectAxis3 parameter
	PARAMETERID_FASelectAxis3 = ( (1 << 24) | 57 ),
	/// \brief The FASelectAxis4 parameter
	PARAMETERID_FASelectAxis4 = ( (1 << 24) | 58 ),
	/// \brief The FASelectAxis5 parameter
	PARAMETERID_FASelectAxis5 = ( (1 << 24) | 59 ),
	/// \brief The FASelectAxis6 parameter
	PARAMETERID_FASelectAxis6 = ( (1 << 24) | 60 ),
	/// \brief The FAOffsetAxis1 parameter
	PARAMETERID_FAOffsetAxis1 = ( (1 << 24) | 61 ),
	/// \brief The FAOffsetAxis2 parameter
	PARAMETERID_FAOffsetAxis2 = ( (1 << 24) | 62 ),
	/// \brief The FAOffsetAxis3 parameter
	PARAMETERID_FAOffsetAxis3 = ( (1 << 24) | 63 ),
	/// \brief The FAOffsetAxis4 parameter
	PARAMETERID_FAOffsetAxis4 = ( (1 << 24) | 64 ),
	/// \brief The FAOffsetAxis5 parameter
	PARAMETERID_FAOffsetAxis5 = ( (1 << 24) | 65 ),
	/// \brief The FAOffsetAxis6 parameter
	PARAMETERID_FAOffsetAxis6 = ( (1 << 24) | 66 ),
	/// \brief The FAPosLimitAxis1 parameter
	PARAMETERID_FAPosLimitAxis1 = ( (1 << 24) | 67 ),
	/// \brief The FAPosLimitAxis2 parameter
	PARAMETERID_FAPosLimitAxis2 = ( (1 << 24) | 68 ),
	/// \brief The FAPosLimitAxis3 parameter
	PARAMETERID_FAPosLimitAxis3 = ( (1 << 24) | 69 ),
	/// \brief The FAPosLimitAxis4 parameter
	PARAMETERID_FAPosLimitAxis4 = ( (1 << 24) | 70 ),
	/// \brief The FAPosLimitAxis5 parameter
	PARAMETERID_FAPosLimitAxis5 = ( (1 << 24) | 71 ),
	/// \brief The FAPosLimitAxis6 parameter
	PARAMETERID_FAPosLimitAxis6 = ( (1 << 24) | 72 ),
	/// \brief The FANegLimitAxis1 parameter
	PARAMETERID_FANegLimitAxis1 = ( (1 << 24) | 73 ),
	/// \brief The FANegLimitAxis2 parameter
	PARAMETERID_FANegLimitAxis2 = ( (1 << 24) | 74 ),
	/// \brief The FANegLimitAxis3 parameter
	PARAMETERID_FANegLimitAxis3 = ( (1 << 24) | 75 ),
	/// \brief The FANegLimitAxis4 parameter
	PARAMETERID_FANegLimitAxis4 = ( (1 << 24) | 76 ),
	/// \brief The FANegLimitAxis5 parameter
	PARAMETERID_FANegLimitAxis5 = ( (1 << 24) | 77 ),
	/// \brief The FANegLimitAxis6 parameter
	PARAMETERID_FANegLimitAxis6 = ( (1 << 24) | 78 ),
	/// \brief The FATermTolerance parameter
	PARAMETERID_FATermTolerance = ( (1 << 24) | 79 ),
	/// \brief The FAMaxNumIterations parameter
	PARAMETERID_FAMaxNumIterations = ( (1 << 24) | 80 ),
	/// \brief The FASaturationValue parameter
	PARAMETERID_FASaturationValue = ( (1 << 24) | 81 ),
	/// \brief The FAReturnToStart parameter
	PARAMETERID_FAReturnToStart = ( (1 << 24) | 82 ),
	/// \brief The FAInputMode parameter
	PARAMETERID_FAInputMode = ( (1 << 24) | 83 ),
	/// \brief The FAInputChannelNum parameter
	PARAMETERID_FAInputChannelNum = ( (1 << 24) | 84 ),
	/// \brief The FAInvertSearch parameter
	PARAMETERID_FAInvertSearch = ( (1 << 24) | 85 ),
	/// \brief The FADelayTime parameter
	PARAMETERID_FADelayTime = ( (1 << 24) | 86 ),
	/// \brief The GCScanSize parameter
	PARAMETERID_GCScanSize = ( (1 << 24) | 87 ),
	/// \brief The GCScanIncrement parameter
	PARAMETERID_GCScanIncrement = ( (1 << 24) | 88 ),
	/// \brief The GCScanLines parameter
	PARAMETERID_GCScanLines = ( (1 << 24) | 89 ),
	/// \brief The GCEdgeValue parameter
	PARAMETERID_GCEdgeValue = ( (1 << 24) | 90 ),
	/// \brief The GCAxis1 parameter
	PARAMETERID_GCAxis1 = ( (1 << 24) | 91 ),
	/// \brief The GCAxis2 parameter
	PARAMETERID_GCAxis2 = ( (1 << 24) | 92 ),
	/// \brief The GCInputMode parameter
	PARAMETERID_GCInputMode = ( (1 << 24) | 93 ),
	/// \brief The GCInputChannelNum parameter
	PARAMETERID_GCInputChannelNum = ( (1 << 24) | 94 ),
	/// \brief The GCInvertSearch parameter
	PARAMETERID_GCInvertSearch = ( (1 << 24) | 95 ),
	/// \brief The GCSingleRasterMode parameter
	PARAMETERID_GCSingleRasterMode = ( (1 << 24) | 96 ),
	/// \brief The GCMotionType parameter
	PARAMETERID_GCMotionType = ( (1 << 24) | 97 ),
	/// \brief The GCDelayTime parameter
	PARAMETERID_GCDelayTime = ( (1 << 24) | 99 ),
	/// \brief The CMaxDisplacement1 parameter
	PARAMETERID_CMaxDisplacement1 = ( (1 << 24) | 100 ),
	/// \brief The CMaxDisplacement2 parameter
	PARAMETERID_CMaxDisplacement2 = ( (1 << 24) | 101 ),
	/// \brief The CMaxDisplacement3 parameter
	PARAMETERID_CMaxDisplacement3 = ( (1 << 24) | 102 ),
	/// \brief The CScanIncrement parameter
	PARAMETERID_CScanIncrement = ( (1 << 24) | 103 ),
	/// \brief The CEdgeValue parameter
	PARAMETERID_CEdgeValue = ( (1 << 24) | 104 ),
	/// \brief The CAxis1 parameter
	PARAMETERID_CAxis1 = ( (1 << 24) | 105 ),
	/// \brief The CAxis2 parameter
	PARAMETERID_CAxis2 = ( (1 << 24) | 106 ),
	/// \brief The CAxis3 parameter
	PARAMETERID_CAxis3 = ( (1 << 24) | 107 ),
	/// \brief The CInputMode parameter
	PARAMETERID_CInputMode = ( (1 << 24) | 108 ),
	/// \brief The CInputChannelNum parameter
	PARAMETERID_CInputChannelNum = ( (1 << 24) | 109 ),
	/// \brief The CInvertSearch parameter
	PARAMETERID_CInvertSearch = ( (1 << 24) | 110 ),
	/// \brief The CReturnToCenter parameter
	PARAMETERID_CReturnToCenter = ( (1 << 24) | 111 ),
	/// \brief The CDelayTime parameter
	PARAMETERID_CDelayTime = ( (1 << 24) | 113 ),
	/// \brief The HCPercentDrop parameter
	PARAMETERID_HCPercentDrop = ( (1 << 24) | 114 ),
	/// \brief The DefaultDependentCoordinatedSpeed parameter
	PARAMETERID_DefaultDependentCoordinatedSpeed = ( (1 << 24) | 115 ),
	/// \brief The CoordinatedAccelLimit parameter
	PARAMETERID_CoordinatedAccelLimit = ( (1 << 24) | 116 ),
	/// \brief The DependentCoordinatedAccelLimit parameter
	PARAMETERID_DependentCoordinatedAccelLimit = ( (1 << 24) | 117 ),
	/// \brief The CoordinatedCircularAccelLimit parameter
	PARAMETERID_CoordinatedCircularAccelLimit = ( (1 << 24) | 118 ),
	/// \brief The CoordinatedAccelLimitSetup parameter
	PARAMETERID_CoordinatedAccelLimitSetup = ( (1 << 24) | 119 ),
	/// \brief The MaxLookaheadMoves parameter
	PARAMETERID_MaxLookaheadMoves = ( (1 << 24) | 120 ),
	/// \brief The RadiusCorrectionThreshold parameter
	PARAMETERID_RadiusCorrectionThreshold = ( (1 << 24) | 121 ),
	/// \brief The RadiusErrorThreshold parameter
	PARAMETERID_RadiusErrorThreshold = ( (1 << 24) | 122 ),
	/// \brief The CutterTolerance parameter
	PARAMETERID_CutterTolerance = ( (1 << 24) | 123 ),
	/// \brief The SoftwareESTOPInput parameter
	PARAMETERID_SoftwareESTOPInput = ( (1 << 24) | 124 ),
	/// \brief The FeedholdInput parameter
	PARAMETERID_FeedholdInput = ( (1 << 24) | 125 ),
	/// \brief The FeedholdSetup parameter
	PARAMETERID_FeedholdSetup = ( (1 << 24) | 126 ),
	/// \brief The AnalogMFOInput parameter
	PARAMETERID_AnalogMFOInput = ( (1 << 24) | 127 ),
	/// \brief The Spindle0MSOInput parameter
	PARAMETERID_Spindle0MSOInput = ( (1 << 24) | 128 ),
	/// \brief The Spindle1MSOInput parameter
	PARAMETERID_Spindle1MSOInput = ( (1 << 24) | 129 ),
	/// \brief The Spindle2MSOInput parameter
	PARAMETERID_Spindle2MSOInput = ( (1 << 24) | 130 ),
	/// \brief The Spindle3MSOInput parameter
	PARAMETERID_Spindle3MSOInput = ( (1 << 24) | 131 ),
	/// \brief The Spindle0Axis parameter
	PARAMETERID_Spindle0Axis = ( (1 << 24) | 132 ),
	/// \brief The Spindle1Axis parameter
	PARAMETERID_Spindle1Axis = ( (1 << 24) | 133 ),
	/// \brief The Spindle2Axis parameter
	PARAMETERID_Spindle2Axis = ( (1 << 24) | 134 ),
	/// \brief The Spindle3Axis parameter
	PARAMETERID_Spindle3Axis = ( (1 << 24) | 135 ),
	/// \brief The CallStackSize parameter
	PARAMETERID_CallStackSize = ( (1 << 24) | 136 ),
	/// \brief The ModeStackSize parameter
	PARAMETERID_ModeStackSize = ( (1 << 24) | 137 ),
	/// \brief The TaskDoubles parameter
	PARAMETERID_TaskDoubles = ( (1 << 24) | 138 ),
	/// \brief The TaskStrings parameter
	PARAMETERID_TaskStrings = ( (1 << 24) | 139 ),
	/// \brief The TaskAxisPoints parameter
	PARAMETERID_TaskAxisPoints = ( (1 << 24) | 140 ),
	/// \brief The MonitorStatements parameter
	PARAMETERID_MonitorStatements = ( (1 << 24) | 141 ),
	/// \brief The MotionUpdateRate parameter
	PARAMETERID_MotionUpdateRate = ( (1 << 24) | 142 ),
	/// \brief The MotionBufferSize parameter
	PARAMETERID_MotionBufferSize = ( (1 << 24) | 143 ),
	/// \brief The ExecuteNumLines parameter
	PARAMETERID_ExecuteNumLines = ( (1 << 24) | 144 ),
	/// \brief The CoordinatedMoveDurationMinimum parameter
	PARAMETERID_CoordinatedMoveDurationMinimum = ( (1 << 24) | 145 ),
	/// \brief The DefaultMotionMode parameter
	PARAMETERID_DefaultMotionMode = ( (1 << 24) | 146 ),
	/// \brief The JoystickInput0 parameter
	PARAMETERID_JoystickInput0 = ( (1 << 24) | 147 ),
	/// \brief The JoystickInput1 parameter
	PARAMETERID_JoystickInput1 = ( (1 << 24) | 148 ),
	/// \brief The JoystickAxesSelect parameter
	PARAMETERID_JoystickAxesSelect = ( (1 << 24) | 149 ),
	/// \brief The JoystickSpeedSelect parameter
	PARAMETERID_JoystickSpeedSelect = ( (1 << 24) | 150 ),
	/// \brief The JoystickInterlock parameter
	PARAMETERID_JoystickInterlock = ( (1 << 24) | 151 ),
	/// \brief The JoystickInput2 parameter
	PARAMETERID_JoystickInput2 = ( (1 << 24) | 152 ),
	/// \brief The AnalogMFOMinVoltage parameter
	PARAMETERID_AnalogMFOMinVoltage = ( (1 << 24) | 153 ),
	/// \brief The AnalogMFOMaxVoltage parameter
	PARAMETERID_AnalogMFOMaxVoltage = ( (1 << 24) | 154 ),
	/// \brief The MaximumMFO parameter
	PARAMETERID_MaximumMFO = ( (1 << 24) | 155 ),
	/// \brief The JoystickInput2MinVoltage parameter
	PARAMETERID_JoystickInput2MinVoltage = ( (1 << 24) | 156 ),
	/// \brief The JoystickInput2MaxVoltage parameter
	PARAMETERID_JoystickInput2MaxVoltage = ( (1 << 24) | 157 ),
	/// \brief The JoystickInput2Deadband parameter
	PARAMETERID_JoystickInput2Deadband = ( (1 << 24) | 158 ),
	/// \brief The MinimumMFO parameter
	PARAMETERID_MinimumMFO = ( (1 << 24) | 159 ),
	/// \brief The AnalogMFOStep parameter
	PARAMETERID_AnalogMFOStep = ( (1 << 24) | 160 ),
	/// \brief The DefaultProgrammingMode parameter
	PARAMETERID_DefaultProgrammingMode = ( (1 << 24) | 161 ),
	/// \brief The DefaultCoordinatedRampType parameter
	PARAMETERID_DefaultCoordinatedRampType = ( (1 << 24) | 162 ),
	/// \brief The DefaultSpindle0Speed parameter
	PARAMETERID_DefaultSpindle0Speed = ( (1 << 24) | 163 ),
	/// \brief The DefaultSpindle1Speed parameter
	PARAMETERID_DefaultSpindle1Speed = ( (1 << 24) | 164 ),
	/// \brief The DefaultSpindle2Speed parameter
	PARAMETERID_DefaultSpindle2Speed = ( (1 << 24) | 165 ),
	/// \brief The DefaultSpindle3Speed parameter
	PARAMETERID_DefaultSpindle3Speed = ( (1 << 24) | 166 ),
	/// \brief The MotionInterpolationMode parameter
	PARAMETERID_MotionInterpolationMode = ( (1 << 24) | 167 ),
	/// \brief The DefaultTimeMode parameter
	PARAMETERID_DefaultTimeMode = ( (1 << 24) | 168 ),
	/// \brief The QueueBufferSize parameter
	PARAMETERID_QueueBufferSize = ( (1 << 24) | 169 ),
	/// \brief The AxisType parameter
	PARAMETERID_AxisType = ( (0 << 24) | 0 ),
	/// \brief The ReverseMotionDirection parameter
	PARAMETERID_ReverseMotionDirection = ( (0 << 24) | 1 ),
	/// \brief The CountsPerUnit parameter
	PARAMETERID_CountsPerUnit = ( (0 << 24) | 2 ),
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
	/// \brief The StepperResolution parameter
	PARAMETERID_StepperResolution = ( (0 << 24) | 79 ),
	/// \brief The StepperRunningCurrent parameter
	PARAMETERID_StepperRunningCurrent = ( (0 << 24) | 80 ),
	/// \brief The StepperHoldingCurrent parameter
	PARAMETERID_StepperHoldingCurrent = ( (0 << 24) | 81 ),
	/// \brief The StepperVerificationSpeed parameter
	PARAMETERID_StepperVerificationSpeed = ( (0 << 24) | 82 ),
	/// \brief The LimitDebounceDistance parameter
	PARAMETERID_LimitDebounceDistance = ( (0 << 24) | 83 ),
	/// \brief The ServoFilter2CoeffN0 parameter
	PARAMETERID_ServoFilter2CoeffN0 = ( (0 << 24) | 84 ),
	/// \brief The ServoFilter2CoeffN1 parameter
	PARAMETERID_ServoFilter2CoeffN1 = ( (0 << 24) | 85 ),
	/// \brief The ServoFilter2CoeffN2 parameter
	PARAMETERID_ServoFilter2CoeffN2 = ( (0 << 24) | 86 ),
	/// \brief The ServoFilter2CoeffD1 parameter
	PARAMETERID_ServoFilter2CoeffD1 = ( (0 << 24) | 87 ),
	/// \brief The ServoFilter2CoeffD2 parameter
	PARAMETERID_ServoFilter2CoeffD2 = ( (0 << 24) | 88 ),
	/// \brief The ServoFilter3CoeffN0 parameter
	PARAMETERID_ServoFilter3CoeffN0 = ( (0 << 24) | 89 ),
	/// \brief The ServoFilter3CoeffN1 parameter
	PARAMETERID_ServoFilter3CoeffN1 = ( (0 << 24) | 90 ),
	/// \brief The ServoFilter3CoeffN2 parameter
	PARAMETERID_ServoFilter3CoeffN2 = ( (0 << 24) | 91 ),
	/// \brief The ServoFilter3CoeffD1 parameter
	PARAMETERID_ServoFilter3CoeffD1 = ( (0 << 24) | 92 ),
	/// \brief The ServoFilter3CoeffD2 parameter
	PARAMETERID_ServoFilter3CoeffD2 = ( (0 << 24) | 93 ),
	/// \brief The MaxJogSpeed parameter
	PARAMETERID_MaxJogSpeed = ( (0 << 24) | 96 ),
	/// \brief The DecimalPlaces parameter
	PARAMETERID_DecimalPlaces = ( (0 << 24) | 97 ),
	/// \brief The UnitsName parameter
	PARAMETERID_UnitsName = ( (0 << 24) | 98 ),
	/// \brief The AxisName parameter
	PARAMETERID_AxisName = ( (0 << 24) | 99 ),
	/// \brief The EnDatEncoderSetup parameter
	PARAMETERID_EnDatEncoderSetup = ( (0 << 24) | 100 ),
	/// \brief The EnDatEncoderResolution parameter
	PARAMETERID_EnDatEncoderResolution = ( (0 << 24) | 101 ),
	/// \brief The EnDatEncoderTurns parameter
	PARAMETERID_EnDatEncoderTurns = ( (0 << 24) | 102 ),
	/// \brief The JoystickLowSpeed parameter
	PARAMETERID_JoystickLowSpeed = ( (0 << 24) | 103 ),
	/// \brief The JoystickHighSpeed parameter
	PARAMETERID_JoystickHighSpeed = ( (0 << 24) | 104 ),
	/// \brief The HomePositionSet parameter
	PARAMETERID_HomePositionSet = ( (0 << 24) | 105 ),
	/// \brief The FaultMaskDisableDelay parameter
	PARAMETERID_FaultMaskDisableDelay = ( (0 << 24) | 106 ),
	/// \brief The FaultAbortAxes parameter
	PARAMETERID_FaultAbortAxes = ( (0 << 24) | 107 ),
	/// \brief The HarmonicCancellation0Type parameter
	PARAMETERID_HarmonicCancellation0Type = ( (0 << 24) | 108 ),
	/// \brief The HarmonicCancellation0Period parameter
	PARAMETERID_HarmonicCancellation0Period = ( (0 << 24) | 109 ),
	/// \brief The HarmonicCancellation0Channel parameter
	PARAMETERID_HarmonicCancellation0Channel = ( (0 << 24) | 110 ),
	/// \brief The HarmonicCancellation0Gain parameter
	PARAMETERID_HarmonicCancellation0Gain = ( (0 << 24) | 111 ),
	/// \brief The HarmonicCancellation0Phase parameter
	PARAMETERID_HarmonicCancellation0Phase = ( (0 << 24) | 112 ),
	/// \brief The HarmonicCancellation1Type parameter
	PARAMETERID_HarmonicCancellation1Type = ( (0 << 24) | 113 ),
	/// \brief The HarmonicCancellation1Period parameter
	PARAMETERID_HarmonicCancellation1Period = ( (0 << 24) | 114 ),
	/// \brief The HarmonicCancellation1Channel parameter
	PARAMETERID_HarmonicCancellation1Channel = ( (0 << 24) | 115 ),
	/// \brief The HarmonicCancellation1Gain parameter
	PARAMETERID_HarmonicCancellation1Gain = ( (0 << 24) | 116 ),
	/// \brief The HarmonicCancellation1Phase parameter
	PARAMETERID_HarmonicCancellation1Phase = ( (0 << 24) | 117 ),
	/// \brief The HarmonicCancellation2Type parameter
	PARAMETERID_HarmonicCancellation2Type = ( (0 << 24) | 118 ),
	/// \brief The HarmonicCancellation2Period parameter
	PARAMETERID_HarmonicCancellation2Period = ( (0 << 24) | 119 ),
	/// \brief The HarmonicCancellation2Channel parameter
	PARAMETERID_HarmonicCancellation2Channel = ( (0 << 24) | 120 ),
	/// \brief The HarmonicCancellation2Gain parameter
	PARAMETERID_HarmonicCancellation2Gain = ( (0 << 24) | 121 ),
	/// \brief The HarmonicCancellation2Phase parameter
	PARAMETERID_HarmonicCancellation2Phase = ( (0 << 24) | 122 ),
	/// \brief The ResolverReferenceGain parameter
	PARAMETERID_ResolverReferenceGain = ( (0 << 24) | 123 ),
	/// \brief The ResolverSetup parameter
	PARAMETERID_ResolverSetup = ( (0 << 24) | 124 ),
	/// \brief The ResolverReferencePhase parameter
	PARAMETERID_ResolverReferencePhase = ( (0 << 24) | 125 ),
	/// \brief The SoftwareLimitSetup parameter
	PARAMETERID_SoftwareLimitSetup = ( (0 << 24) | 126 ),
	/// \brief The SSINet1Setup parameter
	PARAMETERID_SSINet1Setup = ( (0 << 24) | 127 ),
	/// \brief The SSINet2Setup parameter
	PARAMETERID_SSINet2Setup = ( (0 << 24) | 128 ),
	/// \brief The EmulatedQuadratureDivider parameter
	PARAMETERID_EmulatedQuadratureDivider = ( (0 << 24) | 129 ),
	/// \brief The HarmonicCancellation3Type parameter
	PARAMETERID_HarmonicCancellation3Type = ( (0 << 24) | 130 ),
	/// \brief The HarmonicCancellation3Period parameter
	PARAMETERID_HarmonicCancellation3Period = ( (0 << 24) | 131 ),
	/// \brief The HarmonicCancellation3Channel parameter
	PARAMETERID_HarmonicCancellation3Channel = ( (0 << 24) | 132 ),
	/// \brief The HarmonicCancellation3Gain parameter
	PARAMETERID_HarmonicCancellation3Gain = ( (0 << 24) | 133 ),
	/// \brief The HarmonicCancellation3Phase parameter
	PARAMETERID_HarmonicCancellation3Phase = ( (0 << 24) | 134 ),
	/// \brief The HarmonicCancellation4Type parameter
	PARAMETERID_HarmonicCancellation4Type = ( (0 << 24) | 135 ),
	/// \brief The HarmonicCancellation4Period parameter
	PARAMETERID_HarmonicCancellation4Period = ( (0 << 24) | 136 ),
	/// \brief The HarmonicCancellation4Channel parameter
	PARAMETERID_HarmonicCancellation4Channel = ( (0 << 24) | 137 ),
	/// \brief The HarmonicCancellation4Gain parameter
	PARAMETERID_HarmonicCancellation4Gain = ( (0 << 24) | 138 ),
	/// \brief The HarmonicCancellation4Phase parameter
	PARAMETERID_HarmonicCancellation4Phase = ( (0 << 24) | 139 ),
	/// \brief The EnhancedThroughputChannel parameter
	PARAMETERID_EnhancedThroughputChannel = ( (0 << 24) | 140 ),
	/// \brief The EnhancedThroughputGain parameter
	PARAMETERID_EnhancedThroughputGain = ( (0 << 24) | 141 ),
	/// \brief The HarmonicCancellationSetup parameter
	PARAMETERID_HarmonicCancellationSetup = ( (0 << 24) | 142 ),
	/// \brief The EnhancedThroughputCurrentClamp parameter
	PARAMETERID_EnhancedThroughputCurrentClamp = ( (0 << 24) | 143 ),
	/// \brief The Analog0Filter0CoeffN0 parameter
	PARAMETERID_Analog0Filter0CoeffN0 = ( (0 << 24) | 144 ),
	/// \brief The Analog0Filter0CoeffN1 parameter
	PARAMETERID_Analog0Filter0CoeffN1 = ( (0 << 24) | 145 ),
	/// \brief The Analog0Filter0CoeffN2 parameter
	PARAMETERID_Analog0Filter0CoeffN2 = ( (0 << 24) | 146 ),
	/// \brief The Analog0Filter0CoeffD1 parameter
	PARAMETERID_Analog0Filter0CoeffD1 = ( (0 << 24) | 147 ),
	/// \brief The Analog0Filter0CoeffD2 parameter
	PARAMETERID_Analog0Filter0CoeffD2 = ( (0 << 24) | 148 ),
	/// \brief The Analog0Filter1CoeffN0 parameter
	PARAMETERID_Analog0Filter1CoeffN0 = ( (0 << 24) | 149 ),
	/// \brief The Analog0Filter1CoeffN1 parameter
	PARAMETERID_Analog0Filter1CoeffN1 = ( (0 << 24) | 150 ),
	/// \brief The Analog0Filter1CoeffN2 parameter
	PARAMETERID_Analog0Filter1CoeffN2 = ( (0 << 24) | 151 ),
	/// \brief The Analog0Filter1CoeffD1 parameter
	PARAMETERID_Analog0Filter1CoeffD1 = ( (0 << 24) | 152 ),
	/// \brief The Analog0Filter1CoeffD2 parameter
	PARAMETERID_Analog0Filter1CoeffD2 = ( (0 << 24) | 153 ),
	/// \brief The Analog1Filter0CoeffN0 parameter
	PARAMETERID_Analog1Filter0CoeffN0 = ( (0 << 24) | 154 ),
	/// \brief The Analog1Filter0CoeffN1 parameter
	PARAMETERID_Analog1Filter0CoeffN1 = ( (0 << 24) | 155 ),
	/// \brief The Analog1Filter0CoeffN2 parameter
	PARAMETERID_Analog1Filter0CoeffN2 = ( (0 << 24) | 156 ),
	/// \brief The Analog1Filter0CoeffD1 parameter
	PARAMETERID_Analog1Filter0CoeffD1 = ( (0 << 24) | 157 ),
	/// \brief The Analog1Filter0CoeffD2 parameter
	PARAMETERID_Analog1Filter0CoeffD2 = ( (0 << 24) | 158 ),
	/// \brief The Analog1Filter1CoeffN0 parameter
	PARAMETERID_Analog1Filter1CoeffN0 = ( (0 << 24) | 159 ),
	/// \brief The Analog1Filter1CoeffN1 parameter
	PARAMETERID_Analog1Filter1CoeffN1 = ( (0 << 24) | 160 ),
	/// \brief The Analog1Filter1CoeffN2 parameter
	PARAMETERID_Analog1Filter1CoeffN2 = ( (0 << 24) | 161 ),
	/// \brief The Analog1Filter1CoeffD1 parameter
	PARAMETERID_Analog1Filter1CoeffD1 = ( (0 << 24) | 162 ),
	/// \brief The Analog1Filter1CoeffD2 parameter
	PARAMETERID_Analog1Filter1CoeffD2 = ( (0 << 24) | 163 ),
	/// \brief The DefaultRampMode parameter
	PARAMETERID_DefaultRampMode = ( (0 << 24) | 164 ),
	/// \brief The DefaultRampTime parameter
	PARAMETERID_DefaultRampTime = ( (0 << 24) | 165 ),
	/// \brief The ServoFilterSetup parameter
	PARAMETERID_ServoFilterSetup = ( (0 << 24) | 167 ),
	/// \brief The FeedbackSetup parameter
	PARAMETERID_FeedbackSetup = ( (0 << 24) | 168 ),
	/// \brief The EncoderMultiplierSetup parameter
	PARAMETERID_EncoderMultiplierSetup = ( (0 << 24) | 169 ),
	/// \brief The FaultSetup parameter
	PARAMETERID_FaultSetup = ( (0 << 24) | 170 ),
	/// \brief The ThresholdScheduleSetup parameter
	PARAMETERID_ThresholdScheduleSetup = ( (0 << 24) | 171 ),
	/// \brief The ThresholdRegion2High parameter
	PARAMETERID_ThresholdRegion2High = ( (0 << 24) | 172 ),
	/// \brief The ThresholdRegion2Low parameter
	PARAMETERID_ThresholdRegion2Low = ( (0 << 24) | 173 ),
	/// \brief The ThresholdRegion3GainKpos parameter
	PARAMETERID_ThresholdRegion3GainKpos = ( (0 << 24) | 174 ),
	/// \brief The ThresholdRegion3GainKp parameter
	PARAMETERID_ThresholdRegion3GainKp = ( (0 << 24) | 175 ),
	/// \brief The ThresholdRegion3GainKi parameter
	PARAMETERID_ThresholdRegion3GainKi = ( (0 << 24) | 176 ),
	/// \brief The ThresholdRegion3GainKpi parameter
	PARAMETERID_ThresholdRegion3GainKpi = ( (0 << 24) | 177 ),
	/// \brief The ThresholdRegion4High parameter
	PARAMETERID_ThresholdRegion4High = ( (0 << 24) | 178 ),
	/// \brief The ThresholdRegion4Low parameter
	PARAMETERID_ThresholdRegion4Low = ( (0 << 24) | 179 ),
	/// \brief The ThresholdRegion5GainKpos parameter
	PARAMETERID_ThresholdRegion5GainKpos = ( (0 << 24) | 180 ),
	/// \brief The ThresholdRegion5GainKp parameter
	PARAMETERID_ThresholdRegion5GainKp = ( (0 << 24) | 181 ),
	/// \brief The ThresholdRegion5GainKi parameter
	PARAMETERID_ThresholdRegion5GainKi = ( (0 << 24) | 182 ),
	/// \brief The ThresholdRegion5GainKpi parameter
	PARAMETERID_ThresholdRegion5GainKpi = ( (0 << 24) | 183 ),
	/// \brief The DynamicScheduleSetup parameter
	PARAMETERID_DynamicScheduleSetup = ( (0 << 24) | 184 ),
	/// \brief The DynamicGainKposScale parameter
	PARAMETERID_DynamicGainKposScale = ( (0 << 24) | 185 ),
	/// \brief The DynamicGainKpScale parameter
	PARAMETERID_DynamicGainKpScale = ( (0 << 24) | 186 ),
	/// \brief The DynamicGainKiScale parameter
	PARAMETERID_DynamicGainKiScale = ( (0 << 24) | 187 ),
	/// \brief The ServoFilter4CoeffN0 parameter
	PARAMETERID_ServoFilter4CoeffN0 = ( (0 << 24) | 188 ),
	/// \brief The ServoFilter4CoeffN1 parameter
	PARAMETERID_ServoFilter4CoeffN1 = ( (0 << 24) | 189 ),
	/// \brief The ServoFilter4CoeffN2 parameter
	PARAMETERID_ServoFilter4CoeffN2 = ( (0 << 24) | 190 ),
	/// \brief The ServoFilter4CoeffD1 parameter
	PARAMETERID_ServoFilter4CoeffD1 = ( (0 << 24) | 191 ),
	/// \brief The ServoFilter4CoeffD2 parameter
	PARAMETERID_ServoFilter4CoeffD2 = ( (0 << 24) | 192 ),
	/// \brief The ServoFilter5CoeffN0 parameter
	PARAMETERID_ServoFilter5CoeffN0 = ( (0 << 24) | 193 ),
	/// \brief The ServoFilter5CoeffN1 parameter
	PARAMETERID_ServoFilter5CoeffN1 = ( (0 << 24) | 194 ),
	/// \brief The ServoFilter5CoeffN2 parameter
	PARAMETERID_ServoFilter5CoeffN2 = ( (0 << 24) | 195 ),
	/// \brief The ServoFilter5CoeffD1 parameter
	PARAMETERID_ServoFilter5CoeffD1 = ( (0 << 24) | 196 ),
	/// \brief The ServoFilter5CoeffD2 parameter
	PARAMETERID_ServoFilter5CoeffD2 = ( (0 << 24) | 197 ),
	/// \brief The ServoFilter6CoeffN0 parameter
	PARAMETERID_ServoFilter6CoeffN0 = ( (0 << 24) | 198 ),
	/// \brief The ServoFilter6CoeffN1 parameter
	PARAMETERID_ServoFilter6CoeffN1 = ( (0 << 24) | 199 ),
	/// \brief The ServoFilter6CoeffN2 parameter
	PARAMETERID_ServoFilter6CoeffN2 = ( (0 << 24) | 200 ),
	/// \brief The ServoFilter6CoeffD1 parameter
	PARAMETERID_ServoFilter6CoeffD1 = ( (0 << 24) | 201 ),
	/// \brief The ServoFilter6CoeffD2 parameter
	PARAMETERID_ServoFilter6CoeffD2 = ( (0 << 24) | 202 ),
	/// \brief The ServoFilter7CoeffN0 parameter
	PARAMETERID_ServoFilter7CoeffN0 = ( (0 << 24) | 203 ),
	/// \brief The ServoFilter7CoeffN1 parameter
	PARAMETERID_ServoFilter7CoeffN1 = ( (0 << 24) | 204 ),
	/// \brief The ServoFilter7CoeffN2 parameter
	PARAMETERID_ServoFilter7CoeffN2 = ( (0 << 24) | 205 ),
	/// \brief The ServoFilter7CoeffD1 parameter
	PARAMETERID_ServoFilter7CoeffD1 = ( (0 << 24) | 206 ),
	/// \brief The ServoFilter7CoeffD2 parameter
	PARAMETERID_ServoFilter7CoeffD2 = ( (0 << 24) | 207 ),
	/// \brief The LinearAmpMaxPower parameter
	PARAMETERID_LinearAmpMaxPower = ( (0 << 24) | 208 ),
	/// \brief The LinearAmpDeratingFactor parameter
	PARAMETERID_LinearAmpDeratingFactor = ( (0 << 24) | 209 ),
	/// \brief The LinearAmpBusVoltage parameter
	PARAMETERID_LinearAmpBusVoltage = ( (0 << 24) | 210 ),
	/// \brief The MotorResistance parameter
	PARAMETERID_MotorResistance = ( (0 << 24) | 211 ),
	/// \brief The MotorBackEMFConstant parameter
	PARAMETERID_MotorBackEMFConstant = ( (0 << 24) | 212 ),
	/// \brief The GantrySetup parameter
	PARAMETERID_GantrySetup = ( (0 << 24) | 213 ),
	/// \brief The RolloverMode parameter
	PARAMETERID_RolloverMode = ( (0 << 24) | 214 ),
	/// \brief The GantrySeparationThreshold parameter
	PARAMETERID_GantrySeparationThreshold = ( (0 << 24) | 215 ),
	/// \brief The EmulatedQuadratureChannel parameter
	PARAMETERID_EmulatedQuadratureChannel = ( (0 << 24) | 216 ),
	/// \brief The ResolverCoarseChannel parameter
	PARAMETERID_ResolverCoarseChannel = ( (0 << 24) | 217 ),
	/// \brief The ResolverFeedbackRatio parameter
	PARAMETERID_ResolverFeedbackRatio = ( (0 << 24) | 218 ),
	/// \brief The ResolverFeedbackOffset parameter
	PARAMETERID_ResolverFeedbackOffset = ( (0 << 24) | 219 ),
	/// \brief The BrakeEnableDelay parameter
	PARAMETERID_BrakeEnableDelay = ( (0 << 24) | 220 ),
	/// \brief The MaxSpeedClamp parameter
	PARAMETERID_MaxSpeedClamp = ( (0 << 24) | 221 ),
	/// \brief The AutotuneAmplitude parameter
	PARAMETERID_AutotuneAmplitude = ( (0 << 24) | 222 ),
	/// \brief The AutotuneFrequency parameter
	PARAMETERID_AutotuneFrequency = ( (0 << 24) | 223 ),
	/// \brief The AutotuneBandwidth parameter
	PARAMETERID_AutotuneBandwidth = ( (0 << 24) | 224 ),
	/// \brief The AutotunePhaseMargin parameter
	PARAMETERID_AutotunePhaseMargin = ( (0 << 24) | 225 ),
	/// \brief The TrajectoryFIRFilter parameter
	PARAMETERID_TrajectoryFIRFilter = ( (0 << 24) | 226 ),
	/// \brief The TrajectoryIIRFilter parameter
	PARAMETERID_TrajectoryIIRFilter = ( (0 << 24) | 227 ),
	/// \brief The CalibrationIIRFilter parameter
	PARAMETERID_CalibrationIIRFilter = ( (0 << 24) | 228 ),
	/// \brief The BacklashIIRFilter parameter
	PARAMETERID_BacklashIIRFilter = ( (0 << 24) | 229 ),
	/// \brief The InPositionTime parameter
	PARAMETERID_InPositionTime = ( (0 << 24) | 230 ),
	/// \brief The InPositionDisableTimeout parameter
	PARAMETERID_InPositionDisableTimeout = ( (0 << 24) | 231 ),
	/// \brief The Stability0Threshold parameter
	PARAMETERID_Stability0Threshold = ( (0 << 24) | 232 ),
	/// \brief The Stability0Time parameter
	PARAMETERID_Stability0Time = ( (0 << 24) | 233 ),
	/// \brief The GainDff parameter
	PARAMETERID_GainDff = ( (0 << 24) | 234 ),
	/// \brief The StaticFrictionCompensation parameter
	PARAMETERID_StaticFrictionCompensation = ( (0 << 24) | 235 ),
	/// \brief The ServoOutputScaling parameter
	PARAMETERID_ServoOutputScaling = ( (0 << 24) | 236 ),
	/// \brief The ExternalFaultAnalogInput parameter
	PARAMETERID_ExternalFaultAnalogInput = ( (0 << 24) | 238 ),
	/// \brief The ExternalFaultThreshold parameter
	PARAMETERID_ExternalFaultThreshold = ( (0 << 24) | 239 ),
	/// \brief The Stability1Threshold parameter
	PARAMETERID_Stability1Threshold = ( (0 << 24) | 241 ),
	/// \brief The Stability1Time parameter
	PARAMETERID_Stability1Time = ( (0 << 24) | 242 ),
	/// \brief The AnalogFilterSetup parameter
	PARAMETERID_AnalogFilterSetup = ( (0 << 24) | 243 ),
	/// \brief The DefaultRampType parameter
	PARAMETERID_DefaultRampType = ( (0 << 24) | 244 ),
	/// \brief The ServoOutputOffsetA parameter
	PARAMETERID_ServoOutputOffsetA = ( (0 << 24) | 245 ),
	/// \brief The ServoOutputOffsetB parameter
	PARAMETERID_ServoOutputOffsetB = ( (0 << 24) | 246 ),
	/// \brief The AutoDisableTimeout parameter
	PARAMETERID_AutoDisableTimeout = ( (0 << 24) | 247 ),
	/// \brief The DriveIPAddress parameter
	PARAMETERID_DriveIPAddress = ( (0 << 24) | 256 ),
	/// \brief The DriveSubnetMask parameter
	PARAMETERID_DriveSubnetMask = ( (0 << 24) | 257 ),
	/// \brief The DriveDefaultGateway parameter
	PARAMETERID_DriveDefaultGateway = ( (0 << 24) | 258 ),
	/// \brief The CurrentOffsetA parameter
	PARAMETERID_CurrentOffsetA = ( (0 << 24) | 259 ),
	/// \brief The CurrentOffsetB parameter
	PARAMETERID_CurrentOffsetB = ( (0 << 24) | 260 ),
	/// \brief The CommandShaperSetup parameter
	PARAMETERID_CommandShaperSetup = ( (0 << 24) | 261 ),
	/// \brief The CommandShaperTime00 parameter
	PARAMETERID_CommandShaperTime00 = ( (0 << 24) | 262 ),
	/// \brief The CommandShaperTime01 parameter
	PARAMETERID_CommandShaperTime01 = ( (0 << 24) | 263 ),
	/// \brief The CommandShaperTime02 parameter
	PARAMETERID_CommandShaperTime02 = ( (0 << 24) | 264 ),
	/// \brief The CommandShaperTime03 parameter
	PARAMETERID_CommandShaperTime03 = ( (0 << 24) | 265 ),
	/// \brief The CommandShaperTime04 parameter
	PARAMETERID_CommandShaperTime04 = ( (0 << 24) | 266 ),
	/// \brief The CommandShaperTime05 parameter
	PARAMETERID_CommandShaperTime05 = ( (0 << 24) | 267 ),
	/// \brief The CommandShaperTime06 parameter
	PARAMETERID_CommandShaperTime06 = ( (0 << 24) | 268 ),
	/// \brief The CommandShaperTime07 parameter
	PARAMETERID_CommandShaperTime07 = ( (0 << 24) | 269 ),
	/// \brief The CommandShaperTime08 parameter
	PARAMETERID_CommandShaperTime08 = ( (0 << 24) | 270 ),
	/// \brief The CommandShaperTime09 parameter
	PARAMETERID_CommandShaperTime09 = ( (0 << 24) | 271 ),
	/// \brief The CommandShaperTime10 parameter
	PARAMETERID_CommandShaperTime10 = ( (0 << 24) | 272 ),
	/// \brief The CommandShaperTime11 parameter
	PARAMETERID_CommandShaperTime11 = ( (0 << 24) | 273 ),
	/// \brief The CommandShaperTime12 parameter
	PARAMETERID_CommandShaperTime12 = ( (0 << 24) | 274 ),
	/// \brief The CommandShaperTime13 parameter
	PARAMETERID_CommandShaperTime13 = ( (0 << 24) | 275 ),
	/// \brief The CommandShaperTime14 parameter
	PARAMETERID_CommandShaperTime14 = ( (0 << 24) | 276 ),
	/// \brief The CommandShaperTime15 parameter
	PARAMETERID_CommandShaperTime15 = ( (0 << 24) | 277 ),
	/// \brief The CommandShaperCoeff00 parameter
	PARAMETERID_CommandShaperCoeff00 = ( (0 << 24) | 278 ),
	/// \brief The CommandShaperCoeff01 parameter
	PARAMETERID_CommandShaperCoeff01 = ( (0 << 24) | 279 ),
	/// \brief The CommandShaperCoeff02 parameter
	PARAMETERID_CommandShaperCoeff02 = ( (0 << 24) | 280 ),
	/// \brief The CommandShaperCoeff03 parameter
	PARAMETERID_CommandShaperCoeff03 = ( (0 << 24) | 281 ),
	/// \brief The CommandShaperCoeff04 parameter
	PARAMETERID_CommandShaperCoeff04 = ( (0 << 24) | 282 ),
	/// \brief The CommandShaperCoeff05 parameter
	PARAMETERID_CommandShaperCoeff05 = ( (0 << 24) | 283 ),
	/// \brief The CommandShaperCoeff06 parameter
	PARAMETERID_CommandShaperCoeff06 = ( (0 << 24) | 284 ),
	/// \brief The CommandShaperCoeff07 parameter
	PARAMETERID_CommandShaperCoeff07 = ( (0 << 24) | 285 ),
	/// \brief The CommandShaperCoeff08 parameter
	PARAMETERID_CommandShaperCoeff08 = ( (0 << 24) | 286 ),
	/// \brief The CommandShaperCoeff09 parameter
	PARAMETERID_CommandShaperCoeff09 = ( (0 << 24) | 287 ),
	/// \brief The CommandShaperCoeff10 parameter
	PARAMETERID_CommandShaperCoeff10 = ( (0 << 24) | 288 ),
	/// \brief The CommandShaperCoeff11 parameter
	PARAMETERID_CommandShaperCoeff11 = ( (0 << 24) | 289 ),
	/// \brief The CommandShaperCoeff12 parameter
	PARAMETERID_CommandShaperCoeff12 = ( (0 << 24) | 290 ),
	/// \brief The CommandShaperCoeff13 parameter
	PARAMETERID_CommandShaperCoeff13 = ( (0 << 24) | 291 ),
	/// \brief The CommandShaperCoeff14 parameter
	PARAMETERID_CommandShaperCoeff14 = ( (0 << 24) | 292 ),
	/// \brief The CommandShaperCoeff15 parameter
	PARAMETERID_CommandShaperCoeff15 = ( (0 << 24) | 293 ),
	/// \brief The CommandShaper0Type parameter
	PARAMETERID_CommandShaper0Type = ( (0 << 24) | 294 ),
	/// \brief The CommandShaper0Frequency parameter
	PARAMETERID_CommandShaper0Frequency = ( (0 << 24) | 295 ),
	/// \brief The CommandShaper0Damping parameter
	PARAMETERID_CommandShaper0Damping = ( (0 << 24) | 296 ),
	/// \brief The CommandShaper1Type parameter
	PARAMETERID_CommandShaper1Type = ( (0 << 24) | 297 ),
	/// \brief The CommandShaper1Frequency parameter
	PARAMETERID_CommandShaper1Frequency = ( (0 << 24) | 298 ),
	/// \brief The CommandShaper1Damping parameter
	PARAMETERID_CommandShaper1Damping = ( (0 << 24) | 299 ),
	/// \brief The ResoluteEncoderSetup parameter
	PARAMETERID_ResoluteEncoderSetup = ( (0 << 24) | 306 ),
	/// \brief The ResoluteEncoderResolution parameter
	PARAMETERID_ResoluteEncoderResolution = ( (0 << 24) | 307 ),
	/// \brief The ResoluteEncoderUserResolution parameter
	PARAMETERID_ResoluteEncoderUserResolution = ( (0 << 24) | 308 ),
	/// \brief The AutofocusInput parameter
	PARAMETERID_AutofocusInput = ( (0 << 24) | 312 ),
	/// \brief The AutofocusTarget parameter
	PARAMETERID_AutofocusTarget = ( (0 << 24) | 313 ),
	/// \brief The AutofocusDeadband parameter
	PARAMETERID_AutofocusDeadband = ( (0 << 24) | 314 ),
	/// \brief The AutofocusGainKi parameter
	PARAMETERID_AutofocusGainKi = ( (0 << 24) | 315 ),
	/// \brief The AutofocusGainKp parameter
	PARAMETERID_AutofocusGainKp = ( (0 << 24) | 316 ),
	/// \brief The AutofocusLimitLow parameter
	PARAMETERID_AutofocusLimitLow = ( (0 << 24) | 317 ),
	/// \brief The AutofocusLimitHigh parameter
	PARAMETERID_AutofocusLimitHigh = ( (0 << 24) | 318 ),
	/// \brief The AutofocusSpeedClamp parameter
	PARAMETERID_AutofocusSpeedClamp = ( (0 << 24) | 319 ),
	/// \brief The AutofocusHoldInput parameter
	PARAMETERID_AutofocusHoldInput = ( (0 << 24) | 320 ),
	/// \brief The AutofocusSetup parameter
	PARAMETERID_AutofocusSetup = ( (0 << 24) | 321 ),
	/// \brief The TrajectoryDelayFilter parameter
	PARAMETERID_TrajectoryDelayFilter = ( (0 << 24) | 322 ),
	/// \brief The EncoderTrackingSensitivity parameter
	PARAMETERID_EncoderTrackingSensitivity = ( (0 << 24) | 323 ),
	/// \brief The MotorOutputConstant parameter
	PARAMETERID_MotorOutputConstant = ( (0 << 24) | 334 ),
	/// \brief The MotorOutputUnitsName parameter
	PARAMETERID_MotorOutputUnitsName = ( (0 << 24) | 335 ),
	/// \brief The TrajectoryDecimationTime parameter
	PARAMETERID_TrajectoryDecimationTime = ( (0 << 24) | 336 ),
	/// \brief The CrossAxisFeedforward00Axis parameter
	PARAMETERID_CrossAxisFeedforward00Axis = ( (0 << 24) | 337 ),
	/// \brief The CrossAxisFeedforward01Axis parameter
	PARAMETERID_CrossAxisFeedforward01Axis = ( (0 << 24) | 338 ),
	/// \brief The CrossAxisFeedforward02Axis parameter
	PARAMETERID_CrossAxisFeedforward02Axis = ( (0 << 24) | 339 ),
	/// \brief The CrossAxisFeedforward03Axis parameter
	PARAMETERID_CrossAxisFeedforward03Axis = ( (0 << 24) | 340 ),
	/// \brief The CrossAxisFeedforward04Axis parameter
	PARAMETERID_CrossAxisFeedforward04Axis = ( (0 << 24) | 341 ),
	/// \brief The CrossAxisFeedforward05Axis parameter
	PARAMETERID_CrossAxisFeedforward05Axis = ( (0 << 24) | 342 ),
	/// \brief The CrossAxisFeedforward00Gain parameter
	PARAMETERID_CrossAxisFeedforward00Gain = ( (0 << 24) | 343 ),
	/// \brief The CrossAxisFeedforward01Gain parameter
	PARAMETERID_CrossAxisFeedforward01Gain = ( (0 << 24) | 344 ),
	/// \brief The CrossAxisFeedforward02Gain parameter
	PARAMETERID_CrossAxisFeedforward02Gain = ( (0 << 24) | 345 ),
	/// \brief The CrossAxisFeedforward03Gain parameter
	PARAMETERID_CrossAxisFeedforward03Gain = ( (0 << 24) | 346 ),
	/// \brief The CrossAxisFeedforward04Gain parameter
	PARAMETERID_CrossAxisFeedforward04Gain = ( (0 << 24) | 347 ),
	/// \brief The CrossAxisFeedforward05Gain parameter
	PARAMETERID_CrossAxisFeedforward05Gain = ( (0 << 24) | 348 ),
	/// \brief The GainPff parameter
	PARAMETERID_GainPff = ( (0 << 24) | 350 ),
	/// \brief The AutofocusInitialRampTime parameter
	PARAMETERID_AutofocusInitialRampTime = ( (0 << 24) | 351 ),
	/// \brief The PositionAveragingChannel parameter
	PARAMETERID_PositionAveragingChannel = ( (0 << 24) | 352 ),
	/// \brief The EndOfTravelCurrentThresholdLow parameter
	PARAMETERID_EndOfTravelCurrentThresholdLow = ( (0 << 24) | 353 ),
	/// \brief The EndOfTravelCurrentThresholdHigh parameter
	PARAMETERID_EndOfTravelCurrentThresholdHigh = ( (0 << 24) | 354 ),
	/// \brief The AutofocusGainKi2 parameter
	PARAMETERID_AutofocusGainKi2 = ( (0 << 24) | 355 ),
	/// \brief The EnDatEncoderIncrementalResolution parameter
	PARAMETERID_EnDatEncoderIncrementalResolution = ( (0 << 24) | 356 ),
	/// \brief The MarkerSearchThreshold parameter
	PARAMETERID_MarkerSearchThreshold = ( (0 << 24) | 357 ),
	/// \brief The GainKd1 parameter
	PARAMETERID_GainKd1 = ( (0 << 24) | 358 ),
	/// \brief The GainKp1 parameter
	PARAMETERID_GainKp1 = ( (0 << 24) | 359 ),
	/// \brief The VelocityCommandThresholdBeforeHome parameter
	PARAMETERID_VelocityCommandThresholdBeforeHome = ( (0 << 24) | 360 ),
	/// \brief The SecondaryEncoderSineGain parameter
	PARAMETERID_SecondaryEncoderSineGain = ( (0 << 24) | 361 ),
	/// \brief The SecondaryEncoderSineOffset parameter
	PARAMETERID_SecondaryEncoderSineOffset = ( (0 << 24) | 362 ),
	/// \brief The SecondaryEncoderCosineGain parameter
	PARAMETERID_SecondaryEncoderCosineGain = ( (0 << 24) | 363 ),
	/// \brief The SecondaryEncoderCosineOffset parameter
	PARAMETERID_SecondaryEncoderCosineOffset = ( (0 << 24) | 364 ),
	/// \brief The SecondaryEncoderPhase parameter
	PARAMETERID_SecondaryEncoderPhase = ( (0 << 24) | 365 ),
	/// \brief The DriveOutputCommandDelay parameter
	PARAMETERID_DriveOutputCommandDelay = ( (0 << 24) | 366 ),
	/// \brief The InPosition2Distance parameter
	PARAMETERID_InPosition2Distance = ( (0 << 24) | 367 ),
	/// \brief The InPosition2Time parameter
	PARAMETERID_InPosition2Time = ( (0 << 24) | 368 ),
	/// \brief The StepperRunningCurrentDelay parameter
	PARAMETERID_StepperRunningCurrentDelay = ( (0 << 24) | 369 ),
	/// \brief The AbsoluteFeedbackOffset parameter
	PARAMETERID_AbsoluteFeedbackOffset = ( (0 << 24) | 371 ),
	/// \brief The CapSensorFilterLength parameter
	PARAMETERID_CapSensorFilterLength = ( (0 << 24) | 373 ),
	/// \brief The EnhancedTrackingScale parameter
	PARAMETERID_EnhancedTrackingScale = ( (0 << 24) | 374 ),
	/// \brief The EnhancedTrackingBandwidth parameter
	PARAMETERID_EnhancedTrackingBandwidth = ( (0 << 24) | 375 ),
	/// \brief The Analog0InputOffset parameter
	PARAMETERID_Analog0InputOffset = ( (0 << 24) | 376 ),
	/// \brief The Analog1InputOffset parameter
	PARAMETERID_Analog1InputOffset = ( (0 << 24) | 377 ),
	/// \brief The Analog2InputOffset parameter
	PARAMETERID_Analog2InputOffset = ( (0 << 24) | 378 ),
	/// \brief The Analog3InputOffset parameter
	PARAMETERID_Analog3InputOffset = ( (0 << 24) | 379 ),
	/// \brief The EnhancedTrackingSetup parameter
	PARAMETERID_EnhancedTrackingSetup = ( (0 << 24) | 380 ),
	/// \brief The EncoderMarkerAlignment parameter
	PARAMETERID_EncoderMarkerAlignment = ( (0 << 24) | 381 ),
	/// \brief The EncoderRadiusThresholdLow parameter
	PARAMETERID_EncoderRadiusThresholdLow = ( (0 << 24) | 382 ),
	/// \brief The EncoderRadiusThresholdHigh parameter
	PARAMETERID_EncoderRadiusThresholdHigh = ( (0 << 24) | 383 ),
	/// \brief The GainKsi1 parameter
	PARAMETERID_GainKsi1 = ( (0 << 24) | 384 ),
	/// \brief The GainKsi2 parameter
	PARAMETERID_GainKsi2 = ( (0 << 24) | 385 ),
} PARAMETERID;

#endif // __A3200_PARAMETER_ID_H__
