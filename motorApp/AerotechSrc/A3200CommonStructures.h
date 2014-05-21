/// \file A3200CommonStructures.h
/// \brief Contains some common structures and enumerations.
/// 
/// Copyright (c) Aerotech, Inc. 2010-2013.
/// 

#ifndef __A3200_COMMON_STRUCTURES_H__
#define __A3200_COMMON_STRUCTURES_H__

/// \brief This value represents information about the state of this task.
typedef enum
{
	/// \brief The Professional Option is not installed. You cannot use this task.
	TASKSTATE_Unavailable = 0,
	/// \brief This task was disabled by the EnabledTasks parameter.
	TASKSTATE_Inactive = 1,
	/// \brief No program is associated or running.
	TASKSTATE_Idle = 2,
	/// \brief A program is associated but not running.
	TASKSTATE_ProgramReady = 3,
	/// \brief A program is associated and running.
	TASKSTATE_ProgramRunning = 4,
	/// \brief A program is associated, was run, and was feedheld. This state is active from the moment the motion begins to decelerate due to a feedhold until the moment the motion begins to accelerate back to speed due to a feedhold release.
	TASKSTATE_ProgramFeedheld = 5,
	/// \brief A program is associated, was run, and was paused.
	TASKSTATE_ProgramPaused = 6,
	/// \brief A program is associated, was run, and completed.
	TASKSTATE_ProgramComplete = 7,
	/// \brief A task error occurred on this task.
	TASKSTATE_Error = 8,
	/// \brief This task is running in the queue mode.
	TASKSTATE_Queue = 9,
	/// \brief This task is reserved for use by the PLC.
	TASKSTATE_PLCReserved = 10,
}	TASKSTATE;

/// \brief This value provides status information about this axis. The following table describes each bit of this value. More status information for the axis is reported in Drive Status.
typedef enum
{
	/// \brief The axis is homed.
	AXISSTATUS_Homed = (1u << 0),
	/// \brief The axis is performing coordinated (LINEAR, CW, CCW, BEZIER), RAPID, or PVT motion.
	AXISSTATUS_Profiling = (1u << 1),
	/// \brief The controller finished waiting for motion on this axis to complete. The behavior of this bit depends on the selected wait mode. When in WAIT MODE MOVEDONE, this bit will mimic the Move Done bit, but when in WAIT MODE INPOS, this bit will not be active until both the Move Done bit and the In Position bit are both active.
	AXISSTATUS_WaitDone = (1u << 2),
	/// \brief Motion on the axis is controlled from the SMC.
	AXISSTATUS_CommandValid = (1u << 3),
	/// \brief The axis is currently homing.
	AXISSTATUS_Homing = (1u << 4),
	/// \brief The axis is currently enabling.
	AXISSTATUS_Enabling = (1u << 5),
	/// \brief This bit represents internal status.
	AXISSTATUS_JogGenerating = (1u << 7),
	/// \brief The axis is performing asynchronous motion (MOVEINC, MOVEABS, FREERUN).
	AXISSTATUS_Jogging = (1u << 8),
	/// \brief The SMC sent a command to the drive that will cause the drive to take control of the motion, but the drive has not yet done so.
	AXISSTATUS_DrivePending = (1u << 9),
	/// \brief The SMC sent an abort command to the drive, but the drive has not yet started the abort.
	AXISSTATUS_DriveAbortPending = (1u << 10),
	/// \brief Trajectory filtering is enabled for this axis using either the TrajectoryIIRFilter or TrajectoryFIRFilter parameters.
	AXISSTATUS_TrajectoryFiltering = (1u << 11),
	/// \brief Infinite Field of View (IFV) is enabled for this axis. Enable IFV by issuing the GALVO IFV ON command. Disable IFV by issuing the GALVO IFV OFF command.
	AXISSTATUS_IFVEnabled = (1u << 12),
	/// \brief A physical drive is associated with this axis. Axes with no drive attached will clear this bit and operate as virtual axes.
	AXISSTATUS_NotVirtual = (1u << 13),
	/// \brief The specified 1D calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
	AXISSTATUS_CalibrationEnabled1D = (1u << 14),
	/// \brief The specified 2D calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
	AXISSTATUS_CalibrationEnabled2D = (1u << 15),
	/// \brief The axis is currently performing motion under master/slave control (gearing, camming, or handwheel).
	AXISSTATUS_MasterSlaveControl = (1u << 16),
	/// \brief The axis is currently performing motion under control of the JOYSTICK command.
	AXISSTATUS_JoystickControl = (1u << 17),
	/// \brief Backlash compensation is enabled for this axis using the BacklashDistance parameter or the BACKLASH ON command.
	AXISSTATUS_BacklashActive = (1u << 18),
	/// \brief A Gain Mapping table was specified for this axis.
	AXISSTATUS_GainMappingEnabled = (1u << 19),
	/// \brief The axis is considered to be stable as configured by the Stability0Threshold and Stability0Time parameters.
	AXISSTATUS_Stability0 = (1u << 20),
	/// \brief Motion on this axis is being prevented by the BLOCKMOTION command.
	AXISSTATUS_MotionBlocked = (1u << 21),
	/// \brief Motion on this axis is done, meaning that the velocity command reached zero.
	AXISSTATUS_MoveDone = (1u << 22),
	/// \brief Motion on this axis is being clamped due to a software limit clamp or safe zone. Refer to the SoftwareLimitSetup parameter, and the Safe zone overview.
	AXISSTATUS_MotionClamped = (1u << 23),
	/// \brief This axis is part of a gantry pair and the gantry is correctly aligned. This bit will not be active until the gantry axes have been homed.
	AXISSTATUS_GantryAligned = (1u << 24),
	/// \brief The axis is currently performing gantry realignment motion.
	AXISSTATUS_GantryRealigning = (1u << 25),
	/// \brief The axis is considered to be stable as configured by the Stability1Threshold and Stability1Time parameters.
	AXISSTATUS_Stability1 = (1u << 26),
}	AXISSTATUS;
typedef enum
{
	/// \brief The axis is homed.
	AXISSTATUSBITS_HomedBit = 0,
	/// \brief The axis is performing coordinated (LINEAR, CW, CCW, BEZIER), RAPID, or PVT motion.
	AXISSTATUSBITS_ProfilingBit = 1,
	/// \brief The controller finished waiting for motion on this axis to complete. The behavior of this bit depends on the selected wait mode. When in WAIT MODE MOVEDONE, this bit will mimic the Move Done bit, but when in WAIT MODE INPOS, this bit will not be active until both the Move Done bit and the In Position bit are both active.
	AXISSTATUSBITS_WaitDoneBit = 2,
	/// \brief Motion on the axis is controlled from the SMC.
	AXISSTATUSBITS_CommandValidBit = 3,
	/// \brief The axis is currently homing.
	AXISSTATUSBITS_HomingBit = 4,
	/// \brief The axis is currently enabling.
	AXISSTATUSBITS_EnablingBit = 5,
	/// \brief This bit represents internal status.
	AXISSTATUSBITS_JogGeneratingBit = 7,
	/// \brief The axis is performing asynchronous motion (MOVEINC, MOVEABS, FREERUN).
	AXISSTATUSBITS_JoggingBit = 8,
	/// \brief The SMC sent a command to the drive that will cause the drive to take control of the motion, but the drive has not yet done so.
	AXISSTATUSBITS_DrivePendingBit = 9,
	/// \brief The SMC sent an abort command to the drive, but the drive has not yet started the abort.
	AXISSTATUSBITS_DriveAbortPendingBit = 10,
	/// \brief Trajectory filtering is enabled for this axis using either the TrajectoryIIRFilter or TrajectoryFIRFilter parameters.
	AXISSTATUSBITS_TrajectoryFilteringBit = 11,
	/// \brief Infinite Field of View (IFV) is enabled for this axis. Enable IFV by issuing the GALVO IFV ON command. Disable IFV by issuing the GALVO IFV OFF command.
	AXISSTATUSBITS_IFVEnabledBit = 12,
	/// \brief A physical drive is associated with this axis. Axes with no drive attached will clear this bit and operate as virtual axes.
	AXISSTATUSBITS_NotVirtualBit = 13,
	/// \brief The specified 1D calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
	AXISSTATUSBITS_CalibrationEnabled1DBit = 14,
	/// \brief The specified 2D calibration file contains a calibration table that corrects this axis. The state of this bit is not affected by the CALENABLE or CALDISABLE commands.
	AXISSTATUSBITS_CalibrationEnabled2DBit = 15,
	/// \brief The axis is currently performing motion under master/slave control (gearing, camming, or handwheel).
	AXISSTATUSBITS_MasterSlaveControlBit = 16,
	/// \brief The axis is currently performing motion under control of the JOYSTICK command.
	AXISSTATUSBITS_JoystickControlBit = 17,
	/// \brief Backlash compensation is enabled for this axis using the BacklashDistance parameter or the BACKLASH ON command.
	AXISSTATUSBITS_BacklashActiveBit = 18,
	/// \brief A Gain Mapping table was specified for this axis.
	AXISSTATUSBITS_GainMappingEnabledBit = 19,
	/// \brief The axis is considered to be stable as configured by the Stability0Threshold and Stability0Time parameters.
	AXISSTATUSBITS_Stability0Bit = 20,
	/// \brief Motion on this axis is being prevented by the BLOCKMOTION command.
	AXISSTATUSBITS_MotionBlockedBit = 21,
	/// \brief Motion on this axis is done, meaning that the velocity command reached zero.
	AXISSTATUSBITS_MoveDoneBit = 22,
	/// \brief Motion on this axis is being clamped due to a software limit clamp or safe zone. Refer to the SoftwareLimitSetup parameter, and the Safe zone overview.
	AXISSTATUSBITS_MotionClampedBit = 23,
	/// \brief This axis is part of a gantry pair and the gantry is correctly aligned. This bit will not be active until the gantry axes have been homed.
	AXISSTATUSBITS_GantryAlignedBit = 24,
	/// \brief The axis is currently performing gantry realignment motion.
	AXISSTATUSBITS_GantryRealigningBit = 25,
	/// \brief The axis is considered to be stable as configured by the Stability1Threshold and Stability1Time parameters.
	AXISSTATUSBITS_Stability1Bit = 26,
}	AXISSTATUSBITS;

/// \brief Represents the faults of an axis
typedef enum
{
	/// \brief The absolute value of the difference between the position command and the position feedback exceeded the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULT_PositionErrorFault = (1u << 0),
	/// \brief The average motor current exceeded the threshold specified by the AverageCurrentThreshold and AverageCurrentTime parameters.
	AXISFAULT_OverCurrentFault = (1u << 1),
	/// \brief The axis encountered the clockwise (positive) end-of-travel limit switch.
	AXISFAULT_CwEndOfTravelLimitFault = (1u << 2),
	/// \brief The axis encountered the counter-clockwise (negative) end-of-travel limit switch.
	AXISFAULT_CcwEndOfTravelLimitFault = (1u << 3),
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitHigh parameter.
	AXISFAULT_CwSoftwareLimitFault = (1u << 4),
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitLow parameter.
	AXISFAULT_CcwSoftwareLimitFault = (1u << 5),
	/// \brief The amplifier for this axis exceeded its maximum current rating or experienced an internal error.
	AXISFAULT_AmplifierFault = (1u << 6),
	/// \brief The drive detected a problem with the feedback device specified by the PositionFeedbackType and PositionFeedbackChannel parameters.
	AXISFAULT_PositionFeedbackFault = (1u << 7),
	/// \brief The drive detected a problem with the feedback device specified by the VelocityFeedbackType and VelocityFeedbackChannel parameters.
	AXISFAULT_VelocityFeedbackFault = (1u << 8),
	/// \brief The drive detected an invalid state (all high or all low) for the Hall-effect sensor inputs on this axis.
	AXISFAULT_HallSensorFault = (1u << 9),
	/// \brief The commanded velocity is more than the velocity command threshold. Before the axis is homed, this threshold is specified by the VelocityCommandThresholdBeforeHome parameter. After the axis is homed, this threshold is specified by the VelocityCommandThreshold parameter.
	AXISFAULT_MaxVelocityCommandFault = (1u << 10),
	/// \brief The emergency stop sense input, specified by the ESTOPFaultInput parameter, was triggered.
	AXISFAULT_EmergencyStopFault = (1u << 11),
	/// \brief The absolute value of the difference between the velocity command and the velocity feedback exceeded the threshold specified by the VelocityErrorThreshold parameter.
	AXISFAULT_VelocityErrorFault = (1u << 12),
	/// \brief The user-defined touch probe input was triggered.
	AXISFAULT_ProbeInputFault = (1u << 14),
	/// \brief The external fault input, specified by the ExternalFaultAnalogInput or ExternalFaultDigitalInput parameters, was triggered.
	AXISFAULT_ExternalFault = (1u << 15),
	/// \brief The motor thermistor input was triggered, which indicates that the motor exceeded its maximum recommended operating temperature.
	AXISFAULT_MotorTemperatureFault = (1u << 17),
	/// \brief The amplifier exceeded its maximum recommended operating temperature.
	AXISFAULT_AmplifierTemperatureFault = (1u << 18),
	/// \brief The encoder fault input on the motor feedback connector was triggered.
	AXISFAULT_EncoderFault = (1u << 19),
	/// \brief One or more of the drives on the network lost communications with the controller.
	AXISFAULT_CommunicationLostFault = (1u << 20),
	/// \brief The difference between the position command/feedback of the master axis and the position command/feedback of the slave axis of a gantry exceeded the value specified by the GantrySeparationThreshold parameter.
	AXISFAULT_GantryMisalignFault = (1u << 22),
	/// \brief The difference between the position feedback and the scaled (adjusted by GainKv) velocity feedback exceeds the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULT_FeedbackScalingFault = (1u << 23),
	/// \brief The distance that the axis moved while searching for the marker exceeded the threshold specified by the MarkerSearchThreshold parameter.
	AXISFAULT_MarkerSearchFault = (1u << 24),
	/// \brief The axis decelerated to a stop because the motion violated a safe zone.
	AXISFAULT_SafeZoneFault = (1u << 25),
	/// \brief The axis did not achieve in position status in the period specified by the InPositionDisableTimeout parameter.
	AXISFAULT_InPositionTimeoutFault = (1u << 26),
}	AXISFAULT;
typedef enum
{
	/// \brief The absolute value of the difference between the position command and the position feedback exceeded the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULTBITS_PositionErrorFaultBit = 0,
	/// \brief The average motor current exceeded the threshold specified by the AverageCurrentThreshold and AverageCurrentTime parameters.
	AXISFAULTBITS_OverCurrentFaultBit = 1,
	/// \brief The axis encountered the clockwise (positive) end-of-travel limit switch.
	AXISFAULTBITS_CwEndOfTravelLimitFaultBit = 2,
	/// \brief The axis encountered the counter-clockwise (negative) end-of-travel limit switch.
	AXISFAULTBITS_CcwEndOfTravelLimitFaultBit = 3,
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitHigh parameter.
	AXISFAULTBITS_CwSoftwareLimitFaultBit = 4,
	/// \brief The axis was commanded to move beyond the position specified by the SoftwareLimitLow parameter.
	AXISFAULTBITS_CcwSoftwareLimitFaultBit = 5,
	/// \brief The amplifier for this axis exceeded its maximum current rating or experienced an internal error.
	AXISFAULTBITS_AmplifierFaultBit = 6,
	/// \brief The drive detected a problem with the feedback device specified by the PositionFeedbackType and PositionFeedbackChannel parameters.
	AXISFAULTBITS_PositionFeedbackFaultBit = 7,
	/// \brief The drive detected a problem with the feedback device specified by the VelocityFeedbackType and VelocityFeedbackChannel parameters.
	AXISFAULTBITS_VelocityFeedbackFaultBit = 8,
	/// \brief The drive detected an invalid state (all high or all low) for the Hall-effect sensor inputs on this axis.
	AXISFAULTBITS_HallSensorFaultBit = 9,
	/// \brief The commanded velocity is more than the velocity command threshold. Before the axis is homed, this threshold is specified by the VelocityCommandThresholdBeforeHome parameter. After the axis is homed, this threshold is specified by the VelocityCommandThreshold parameter.
	AXISFAULTBITS_MaxVelocityCommandFaultBit = 10,
	/// \brief The emergency stop sense input, specified by the ESTOPFaultInput parameter, was triggered.
	AXISFAULTBITS_EmergencyStopFaultBit = 11,
	/// \brief The absolute value of the difference between the velocity command and the velocity feedback exceeded the threshold specified by the VelocityErrorThreshold parameter.
	AXISFAULTBITS_VelocityErrorFaultBit = 12,
	/// \brief The user-defined touch probe input was triggered.
	AXISFAULTBITS_ProbeInputFaultBit = 14,
	/// \brief The external fault input, specified by the ExternalFaultAnalogInput or ExternalFaultDigitalInput parameters, was triggered.
	AXISFAULTBITS_ExternalFaultBit = 15,
	/// \brief The motor thermistor input was triggered, which indicates that the motor exceeded its maximum recommended operating temperature.
	AXISFAULTBITS_MotorTemperatureFaultBit = 17,
	/// \brief The amplifier exceeded its maximum recommended operating temperature.
	AXISFAULTBITS_AmplifierTemperatureFaultBit = 18,
	/// \brief The encoder fault input on the motor feedback connector was triggered.
	AXISFAULTBITS_EncoderFaultBit = 19,
	/// \brief One or more of the drives on the network lost communications with the controller.
	AXISFAULTBITS_CommunicationLostFaultBit = 20,
	/// \brief The difference between the position command/feedback of the master axis and the position command/feedback of the slave axis of a gantry exceeded the value specified by the GantrySeparationThreshold parameter.
	AXISFAULTBITS_GantryMisalignFaultBit = 22,
	/// \brief The difference between the position feedback and the scaled (adjusted by GainKv) velocity feedback exceeds the threshold specified by the PositionErrorThreshold parameter.
	AXISFAULTBITS_FeedbackScalingFaultBit = 23,
	/// \brief The distance that the axis moved while searching for the marker exceeded the threshold specified by the MarkerSearchThreshold parameter.
	AXISFAULTBITS_MarkerSearchFaultBit = 24,
	/// \brief The axis decelerated to a stop because the motion violated a safe zone.
	AXISFAULTBITS_SafeZoneFaultBit = 25,
	/// \brief The axis did not achieve in position status in the period specified by the InPositionDisableTimeout parameter.
	AXISFAULTBITS_InPositionTimeoutFaultBit = 26,
}	AXISFAULTBITS;

/// \brief Represents the motor type
typedef enum
{
	/// \brief AC Brushless (Hall-Effect Switches)
	MOTORTYPE_ACBrushlessHallEffect = 0,
	/// \brief AC Brushless (Auto-MSET)
	MOTORTYPE_ACBrushlessAutoMSET = 1,
	/// \brief DC Brush
	MOTORTYPE_DCBrush = 2,
	/// \brief Stepper Motor
	MOTORTYPE_StepperMotor = 3,
	/// \brief Ceramic
	MOTORTYPE_Ceramic = 4,
	/// \brief AC Brushless Actuator
	MOTORTYPE_ACBrushlessActuator = 5,
	/// \brief 2-Phase AC Brushless
	MOTORTYPE_TwoPhaseACBrushless = 6,
	/// \brief AC Brushless (Commutation Search)
	MOTORTYPE_ACBrushlessCommutationSearch = 7,
	/// \brief Piezo Actuator
	MOTORTYPE_PiezoActuator = 8,
}	MOTORTYPE;

/// \brief This value provides status information from the drive connected to this axis. The following table describes each bit of this value. More status information for the axis is reported in Axis Status.
typedef enum
{
	/// \brief The axis is enabled.
	DRIVESTATUS_Enabled = (1u << 0),
	/// \brief This represents the state of the CW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	DRIVESTATUS_CwEndOfTravelLimitInput = (1u << 1),
	/// \brief This represents the state of the CCW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	DRIVESTATUS_CcwEndOfTravelLimitInput = (1u << 2),
	/// \brief This represents the state of the home limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	DRIVESTATUS_HomeLimitInput = (1u << 3),
	/// \brief This represents the state of the marker input.
	DRIVESTATUS_MarkerInput = (1u << 4),
	/// \brief This represents the state of the Hall-effect sensor A input.
	DRIVESTATUS_HallAInput = (1u << 5),
	/// \brief This represents the state of the Hall-effect sensor B input.
	DRIVESTATUS_HallBInput = (1u << 6),
	/// \brief This represents the state of the Hall-effect sensor C input.
	DRIVESTATUS_HallCInput = (1u << 7),
	/// \brief An error condition is present on the Sine encoder input of the position feedback device.
	DRIVESTATUS_SineEncoderError = (1u << 8),
	/// \brief An error condition is present on the Cosine encoder input of the position feedback device.
	DRIVESTATUS_CosineEncoderError = (1u << 9),
	/// \brief This represents the state of the emergency stop sense input.
	DRIVESTATUS_EmergencyStopInput = (1u << 10),
	/// \brief This represents the state of the dedicated brake output.
	DRIVESTATUS_BrakeOutput = (1u << 11),
	/// \brief The galvo command FIFO for this axis is full.
	DRIVESTATUS_GalvoFifoFull = (1u << 12),
	/// \brief The galvo axis is operating in submillisecond trajectory mode.
	DRIVESTATUS_GalvoSubMsecMode = (1u << 13),
	/// \brief The drive detected that no motor supply voltage is present.
	DRIVESTATUS_NoMotorSupply = (1u << 14),
	/// \brief The motor output is being clamped to the value specified by the MaxCurrentClamp parameter.
	DRIVESTATUS_CurrentClamp = (1u << 15),
	/// \brief The position of the marker is latched.
	DRIVESTATUS_MarkerLatch = (1u << 16),
	/// \brief The motor output is being limited to prevent damage to the amplifier.
	DRIVESTATUS_PowerLimiting = (1u << 17),
	/// \brief Internal drive state was latched by the PSOHALT logic.
	DRIVESTATUS_PSOHaltLatch = (1u << 18),
	/// \brief The amplifier is operating in high resolution current feedback mode.
	DRIVESTATUS_HighResolutionMode = (1u << 19),
	/// \brief The specified 2D calibration file contains a galvo calibration table that corrects this axis.
	DRIVESTATUS_GalvoCalibrationEnabled = (1u << 20),
	/// \brief The axis is operating under control of the AUTOFOCUS loop.
	DRIVESTATUS_AutofocusActive = (1u << 21),
	/// \brief The drive is programming its internal flash memory.
	DRIVESTATUS_ProgrammingFlash = (1u << 22),
	/// \brief The on-board encoder multiplier is performing a programming operation.
	DRIVESTATUS_ProgrammingMXH = (1u << 23),
	/// \brief The axis is operating under servo control.
	DRIVESTATUS_ServoControl = (1u << 24),
	/// \brief The axis is considered to be in position as configured by the InPositionDistance and InPositionTime parameters.
	DRIVESTATUS_InPosition = (1u << 25),
	/// \brief The axis is performing drive generated motion.
	DRIVESTATUS_MoveActive = (1u << 26),
	/// \brief The axis is accelerating.
	DRIVESTATUS_AccelerationPhase = (1u << 27),
	/// \brief The axis is decelerating.
	DRIVESTATUS_DecelerationPhase = (1u << 28),
	/// \brief The on-board encoder multiplier detected that the input signals may be exceeding the maximum input range, which results in clipping of the encoder signals.
	DRIVESTATUS_EncoderClipping = (1u << 29),
	/// \brief The axis is operating in dual-loop mode using two different feedback devices.
	DRIVESTATUS_DualLoopActive = (1u << 30),
	/// \brief The axis is considered to be in position as configured by the InPosition2Distance and InPosition2Time parameters.
	DRIVESTATUS_InPosition2 = (1u << 31),
}	DRIVESTATUS;
typedef enum
{
	/// \brief The axis is enabled.
	DRIVESTATUSBITS_EnabledBit = 0,
	/// \brief This represents the state of the CW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	DRIVESTATUSBITS_CwEndOfTravelLimitInputBit = 1,
	/// \brief This represents the state of the CCW end of travel limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	DRIVESTATUSBITS_CcwEndOfTravelLimitInputBit = 2,
	/// \brief This represents the state of the home limit input. It is not affected by the active polarity, which is configured by the EndOfTravelLimitSetup parameter.
	DRIVESTATUSBITS_HomeLimitInputBit = 3,
	/// \brief This represents the state of the marker input.
	DRIVESTATUSBITS_MarkerInputBit = 4,
	/// \brief This represents the state of the Hall-effect sensor A input.
	DRIVESTATUSBITS_HallAInputBit = 5,
	/// \brief This represents the state of the Hall-effect sensor B input.
	DRIVESTATUSBITS_HallBInputBit = 6,
	/// \brief This represents the state of the Hall-effect sensor C input.
	DRIVESTATUSBITS_HallCInputBit = 7,
	/// \brief An error condition is present on the Sine encoder input of the position feedback device.
	DRIVESTATUSBITS_SineEncoderErrorBit = 8,
	/// \brief An error condition is present on the Cosine encoder input of the position feedback device.
	DRIVESTATUSBITS_CosineEncoderErrorBit = 9,
	/// \brief This represents the state of the emergency stop sense input.
	DRIVESTATUSBITS_EmergencyStopInputBit = 10,
	/// \brief This represents the state of the dedicated brake output.
	DRIVESTATUSBITS_BrakeOutputBit = 11,
	/// \brief The galvo command FIFO for this axis is full.
	DRIVESTATUSBITS_GalvoFifoFullBit = 12,
	/// \brief The galvo axis is operating in submillisecond trajectory mode.
	DRIVESTATUSBITS_GalvoSubMsecModeBit = 13,
	/// \brief The drive detected that no motor supply voltage is present.
	DRIVESTATUSBITS_NoMotorSupplyBit = 14,
	/// \brief The motor output is being clamped to the value specified by the MaxCurrentClamp parameter.
	DRIVESTATUSBITS_CurrentClampBit = 15,
	/// \brief The position of the marker is latched.
	DRIVESTATUSBITS_MarkerLatchBit = 16,
	/// \brief The motor output is being limited to prevent damage to the amplifier.
	DRIVESTATUSBITS_PowerLimitingBit = 17,
	/// \brief Internal drive state was latched by the PSOHALT logic.
	DRIVESTATUSBITS_PSOHaltLatchBit = 18,
	/// \brief The amplifier is operating in high resolution current feedback mode.
	DRIVESTATUSBITS_HighResolutionModeBit = 19,
	/// \brief The specified 2D calibration file contains a galvo calibration table that corrects this axis.
	DRIVESTATUSBITS_GalvoCalibrationEnabledBit = 20,
	/// \brief The axis is operating under control of the AUTOFOCUS loop.
	DRIVESTATUSBITS_AutofocusActiveBit = 21,
	/// \brief The drive is programming its internal flash memory.
	DRIVESTATUSBITS_ProgrammingFlashBit = 22,
	/// \brief The on-board encoder multiplier is performing a programming operation.
	DRIVESTATUSBITS_ProgrammingMXHBit = 23,
	/// \brief The axis is operating under servo control.
	DRIVESTATUSBITS_ServoControlBit = 24,
	/// \brief The axis is considered to be in position as configured by the InPositionDistance and InPositionTime parameters.
	DRIVESTATUSBITS_InPositionBit = 25,
	/// \brief The axis is performing drive generated motion.
	DRIVESTATUSBITS_MoveActiveBit = 26,
	/// \brief The axis is accelerating.
	DRIVESTATUSBITS_AccelerationPhaseBit = 27,
	/// \brief The axis is decelerating.
	DRIVESTATUSBITS_DecelerationPhaseBit = 28,
	/// \brief The on-board encoder multiplier detected that the input signals may be exceeding the maximum input range, which results in clipping of the encoder signals.
	DRIVESTATUSBITS_EncoderClippingBit = 29,
	/// \brief The axis is operating in dual-loop mode using two different feedback devices.
	DRIVESTATUSBITS_DualLoopActiveBit = 30,
	/// \brief The axis is considered to be in position as configured by the InPosition2Distance and InPosition2Time parameters.
	DRIVESTATUSBITS_InPosition2Bit = 31,
}	DRIVESTATUSBITS;

/// \brief Represents the task execution modes
typedef enum
{
	/// \brief Run into subroutines
	TASKEXECUTIONMODE_RunInto = 0,
	/// \brief Step into subroutines
	TASKEXECUTIONMODE_StepInto = 1,
	/// \brief Step over subroutines
	TASKEXECUTIONMODE_StepOver = 2,
	/// \brief Run over subroutines
	TASKEXECUTIONMODE_RunOver = 3,
}	TASKEXECUTIONMODE;

/// \brief Represents the program automation modes
typedef enum
{
	/// \brief Automatically include the given file into any program that is compiled
	PROGRAMAUTOMATIONMODE_Include = 0,
	/// \brief Download file so it can be called into later on
	PROGRAMAUTOMATIONMODE_Download = 1,
	/// \brief Run file silently
	PROGRAMAUTOMATIONMODE_RunSilent = 2,
	/// \brief Run file
	PROGRAMAUTOMATIONMODE_Run = 3,
	/// \brief Download file and associate to a task but do not start execution
	PROGRAMAUTOMATIONMODE_DownloadAndAssociate = 5,
}	PROGRAMAUTOMATIONMODE;

/// \brief Represents the units types
typedef enum
{
	/// \brief Secondary units
	UNITSTYPE_Secondary = 0,
	/// \brief Primary units
	UNITSTYPE_Primary = 1,
}	UNITSTYPE;

/// \brief This value provides status information about this task. The following table describes each bit of this value. More status information for the task is reported in Task Status 1, Task Status 2, and Task Mode.
typedef enum
{
	/// \brief A program is associated with this task.
	TASKSTATUS0_ProgramAssociated = (1u << 0),
	/// \brief An immediate command is executing concurrently with a program.
	TASKSTATUS0_ImmediateConcurrent = (1u << 2),
	/// \brief An immediate command is executing.
	TASKSTATUS0_ImmediateExecuting = (1u << 3),
	/// \brief A return motion is executing due to an INTERRUPTMOTION command.
	TASKSTATUS0_ReturnMotionExecuting = (1u << 4),
	/// \brief The program is stopped.
	TASKSTATUS0_ProgramStopped = (1u << 5),
	/// \brief The task is using step into mode.
	TASKSTATUS0_SingleStepInto = (1u << 6),
	/// \brief The task is using step over mode.
	TASKSTATUS0_SingleStepOver = (1u << 7),
	/// \brief The program is reset.
	TASKSTATUS0_ProgramReset = (1u << 8),
	/// \brief One or more axes are decelerating due to an abort, task stop, task error, feedhold, ongosub, or retrace direction reversal that occurs during a RAPID, LINEAR, CW, CCW, or BEZIER motion.
	TASKSTATUS0_PendingAxesStop = (1u << 9),
	/// \brief A Software Emergency Stop is active as configured by the SoftwareESTOPInput parameter.
	TASKSTATUS0_SoftwareESTOPActive = (1u << 10),
	/// \brief Bit turns on as soon as motion begins to decelerate due to a feedhold. Bit turns off when motion begins to accelerate back to speed due to a feedhold release. The FeedHeldAxesStopped bit of Task Status 1 indicates when deceleration due to a feedhold stops.
	TASKSTATUS0_FeedHoldActive = (1u << 11),
	/// \brief A callback command was issued and is waiting for a front-end application to acknowledge the command.
	TASKSTATUS0_CallbackHoldActive = (1u << 12),
	/// \brief A callback command was issued and is waiting for a front-end application to handle the command.
	TASKSTATUS0_CallbackResponding = (1u << 13),
	/// \brief Spindle 0 is active.
	TASKSTATUS0_SpindleActive0 = (1u << 14),
	/// \brief Spindle 1 is active.
	TASKSTATUS0_SpindleActive1 = (1u << 15),
	/// \brief Spindle 2 is active.
	TASKSTATUS0_SpindleActive2 = (1u << 16),
	/// \brief Spindle 3 is active.
	TASKSTATUS0_SpindleActive3 = (1u << 17),
	/// \brief Represents the state of the probe.
	TASKSTATUS0_ProbeCycle = (1u << 18),
	/// \brief Retrace is active.
	TASKSTATUS0_Retrace = (1u << 19),
	/// \brief The POSOFFSET Command was used to set position offsets.
	TASKSTATUS0_SoftHomeActive = (1u << 20),
	/// \brief Interrupt motion is active.
	TASKSTATUS0_InterruptMotionActive = (1u << 21),
	/// \brief The task is executing a JOYSTICK Command.
	TASKSTATUS0_JoystickActive = (1u << 22),
	/// \brief Corner rounding is enabled.
	TASKSTATUS0_CornerRounding = (1u << 23),
	/// \brief The task is reserved for use by the PLC
	TASKSTATUS0_PLCReserved = (1u << 24),
	/// \brief The joystick is using the low speed mode.
	TASKSTATUS0_JoystickLowSpeedActive = (1u << 25),
	/// \brief A canned function is active.
	TASKSTATUS0_CannedFunctionActive = (1u << 26),
	/// \brief A canned function is executing.
	TASKSTATUS0_CannedFunctionExecuting = (1u << 27),
	/// \brief The task is executing Galvo Commands.
	TASKSTATUS0_GalvoMode = (1u << 28),
	/// \brief The task has control restrictions enabled.
	TASKSTATUS0_ProgramControlRestricted = (1u << 29),
}	TASKSTATUS0;
typedef enum
{
	/// \brief A program is associated with this task.
	TASKSTATUS0BITS_ProgramAssociatedBit = 0,
	/// \brief An immediate command is executing concurrently with a program.
	TASKSTATUS0BITS_ImmediateConcurrentBit = 2,
	/// \brief An immediate command is executing.
	TASKSTATUS0BITS_ImmediateExecutingBit = 3,
	/// \brief A return motion is executing due to an INTERRUPTMOTION command.
	TASKSTATUS0BITS_ReturnMotionExecutingBit = 4,
	/// \brief The program is stopped.
	TASKSTATUS0BITS_ProgramStoppedBit = 5,
	/// \brief The task is using step into mode.
	TASKSTATUS0BITS_SingleStepIntoBit = 6,
	/// \brief The task is using step over mode.
	TASKSTATUS0BITS_SingleStepOverBit = 7,
	/// \brief The program is reset.
	TASKSTATUS0BITS_ProgramResetBit = 8,
	/// \brief One or more axes are decelerating due to an abort, task stop, task error, feedhold, ongosub, or retrace direction reversal that occurs during a RAPID, LINEAR, CW, CCW, or BEZIER motion.
	TASKSTATUS0BITS_PendingAxesStopBit = 9,
	/// \brief A Software Emergency Stop is active as configured by the SoftwareESTOPInput parameter.
	TASKSTATUS0BITS_SoftwareESTOPActiveBit = 10,
	/// \brief Bit turns on as soon as motion begins to decelerate due to a feedhold. Bit turns off when motion begins to accelerate back to speed due to a feedhold release. The FeedHeldAxesStopped bit of Task Status 1 indicates when deceleration due to a feedhold stops.
	TASKSTATUS0BITS_FeedHoldActiveBit = 11,
	/// \brief A callback command was issued and is waiting for a front-end application to acknowledge the command.
	TASKSTATUS0BITS_CallbackHoldActiveBit = 12,
	/// \brief A callback command was issued and is waiting for a front-end application to handle the command.
	TASKSTATUS0BITS_CallbackRespondingBit = 13,
	/// \brief Spindle 0 is active.
	TASKSTATUS0BITS_SpindleActive0Bit = 14,
	/// \brief Spindle 1 is active.
	TASKSTATUS0BITS_SpindleActive1Bit = 15,
	/// \brief Spindle 2 is active.
	TASKSTATUS0BITS_SpindleActive2Bit = 16,
	/// \brief Spindle 3 is active.
	TASKSTATUS0BITS_SpindleActive3Bit = 17,
	/// \brief Represents the state of the probe.
	TASKSTATUS0BITS_ProbeCycleBit = 18,
	/// \brief Retrace is active.
	TASKSTATUS0BITS_RetraceBit = 19,
	/// \brief The POSOFFSET Command was used to set position offsets.
	TASKSTATUS0BITS_SoftHomeActiveBit = 20,
	/// \brief Interrupt motion is active.
	TASKSTATUS0BITS_InterruptMotionActiveBit = 21,
	/// \brief The task is executing a JOYSTICK Command.
	TASKSTATUS0BITS_JoystickActiveBit = 22,
	/// \brief Corner rounding is enabled.
	TASKSTATUS0BITS_CornerRoundingBit = 23,
	/// \brief The task is reserved for use by the PLC
	TASKSTATUS0BITS_PLCReservedBit = 24,
	/// \brief The joystick is using the low speed mode.
	TASKSTATUS0BITS_JoystickLowSpeedActiveBit = 25,
	/// \brief A canned function is active.
	TASKSTATUS0BITS_CannedFunctionActiveBit = 26,
	/// \brief A canned function is executing.
	TASKSTATUS0BITS_CannedFunctionExecutingBit = 27,
	/// \brief The task is executing Galvo Commands.
	TASKSTATUS0BITS_GalvoModeBit = 28,
	/// \brief The task has control restrictions enabled.
	TASKSTATUS0BITS_ProgramControlRestrictedBit = 29,
}	TASKSTATUS0BITS;

/// \brief This value provides status information about this task. The following table describes each bit of this value. More status information for the task is reported in Task Status 0, Task Status 2, and Task Mode.
typedef enum
{
	/// \brief Circular center points are specified in absolute coordinates.
	TASKSTATUS1_MotionModeAbsOffsets = (1u << 0),
	/// \brief An asynchronous motion is aborting.
	TASKSTATUS1_AsyncSMCMotionAbortPending = (1u << 1),
	/// \brief The controller interrupted execution of the program and decelerated all motion due to a change in the direction of MFO. The controller is pending execution of the program.
	TASKSTATUS1_RetraceReversalPending = (1u << 2),
	/// \brief A retrace operation was requested.
	TASKSTATUS1_RetraceRequested = (1u << 3),
	/// \brief An MSO change was issued.
	TASKSTATUS1_MSOChange = (1u << 4),
	/// \brief A spindle is feedheld.
	TASKSTATUS1_SpindleFeedHeld = (1u << 5),
	/// \brief Bit turns on as soon as deceleration due to a feedhold ends. Bit turns off when motion begins to accelerate back to speed due to a feedhold release. The FeedHoldActive bit of Task Status 0 indicates when a feedhold is issued.
	TASKSTATUS1_FeedHeldAxesStopped = (1u << 6),
	/// \brief Cutter radius compensation is performing a lead-on move.
	TASKSTATUS1_CutterRadiusEnabling = (1u << 7),
	/// \brief Cutter radius compensation is performing a lead-off move.
	TASKSTATUS1_CutterRadiusDisabling = (1u << 8),
	/// \brief Cutter offset compensation is performing a lead-on positive move.
	TASKSTATUS1_CutterOffsetsEnablingPositive = (1u << 9),
	/// \brief Cutter offset compensation is performing a lead-on negative move.
	TASKSTATUS1_CutterOffsetsEnablingNegative = (1u << 10),
	/// \brief Cutter offset compensation is performing a lead-off move.
	TASKSTATUS1_CutterOffsetsDisabling = (1u << 11),
	/// \brief An MFO change was issued.
	TASKSTATUS1_MFOChange = (1u << 12),
	/// \brief A axis fault or task error event was triggered.
	TASKSTATUS1_InterruptFaultPending = (1u << 13),
	/// \brief An ongosub is pending.
	TASKSTATUS1_OnGosubPending = (1u << 15),
	/// \brief A program stop is pending.
	TASKSTATUS1_ProgramStopPending = (1u << 16),
	/// \brief A canned function is pending.
	TASKSTATUS1_CannedFunctionPending = (1u << 17),
	/// \brief The MinimumMFO parameter is negative.
	TASKSTATUS1_NoMFOFloor = (1u << 18),
	/// \brief This bit represents internal status.
	TASKSTATUS1_Interrupted = (1u << 19),
	/// \brief The task is switching into Galvo mode.
	TASKSTATUS1_GalvoSwitchingIn = (1u << 21),
	/// \brief The task is switching out of Galvo mode.
	TASKSTATUS1_GalvoSwitchingOut = (1u << 22),
	/// \brief This bit represents internal status.
	TASKSTATUS1_ProgramSuppressed = (1u << 23),
	/// \brief This bit represents internal status.
	TASKSTATUS1_GalvoIFVDeactivationPending = (1u << 24),
}	TASKSTATUS1;
typedef enum
{
	/// \brief Circular center points are specified in absolute coordinates.
	TASKSTATUS1BITS_MotionModeAbsOffsetsBit = 0,
	/// \brief An asynchronous motion is aborting.
	TASKSTATUS1BITS_AsyncSMCMotionAbortPendingBit = 1,
	/// \brief The controller interrupted execution of the program and decelerated all motion due to a change in the direction of MFO. The controller is pending execution of the program.
	TASKSTATUS1BITS_RetraceReversalPendingBit = 2,
	/// \brief A retrace operation was requested.
	TASKSTATUS1BITS_RetraceRequestedBit = 3,
	/// \brief An MSO change was issued.
	TASKSTATUS1BITS_MSOChangeBit = 4,
	/// \brief A spindle is feedheld.
	TASKSTATUS1BITS_SpindleFeedHeldBit = 5,
	/// \brief Bit turns on as soon as deceleration due to a feedhold ends. Bit turns off when motion begins to accelerate back to speed due to a feedhold release. The FeedHoldActive bit of Task Status 0 indicates when a feedhold is issued.
	TASKSTATUS1BITS_FeedHeldAxesStoppedBit = 6,
	/// \brief Cutter radius compensation is performing a lead-on move.
	TASKSTATUS1BITS_CutterRadiusEnablingBit = 7,
	/// \brief Cutter radius compensation is performing a lead-off move.
	TASKSTATUS1BITS_CutterRadiusDisablingBit = 8,
	/// \brief Cutter offset compensation is performing a lead-on positive move.
	TASKSTATUS1BITS_CutterOffsetsEnablingPositiveBit = 9,
	/// \brief Cutter offset compensation is performing a lead-on negative move.
	TASKSTATUS1BITS_CutterOffsetsEnablingNegativeBit = 10,
	/// \brief Cutter offset compensation is performing a lead-off move.
	TASKSTATUS1BITS_CutterOffsetsDisablingBit = 11,
	/// \brief An MFO change was issued.
	TASKSTATUS1BITS_MFOChangeBit = 12,
	/// \brief A axis fault or task error event was triggered.
	TASKSTATUS1BITS_InterruptFaultPendingBit = 13,
	/// \brief An ongosub is pending.
	TASKSTATUS1BITS_OnGosubPendingBit = 15,
	/// \brief A program stop is pending.
	TASKSTATUS1BITS_ProgramStopPendingBit = 16,
	/// \brief A canned function is pending.
	TASKSTATUS1BITS_CannedFunctionPendingBit = 17,
	/// \brief The MinimumMFO parameter is negative.
	TASKSTATUS1BITS_NoMFOFloorBit = 18,
	/// \brief This bit represents internal status.
	TASKSTATUS1BITS_InterruptedBit = 19,
	/// \brief The task is switching into Galvo mode.
	TASKSTATUS1BITS_GalvoSwitchingInBit = 21,
	/// \brief The task is switching out of Galvo mode.
	TASKSTATUS1BITS_GalvoSwitchingOutBit = 22,
	/// \brief This bit represents internal status.
	TASKSTATUS1BITS_ProgramSuppressedBit = 23,
	/// \brief This bit represents internal status.
	TASKSTATUS1BITS_GalvoIFVDeactivationPendingBit = 24,
}	TASKSTATUS1BITS;

/// \brief This value provides status information about this task. The following table describes each bit of this value. More status information for the task is reported in Task Status 0, Task Status 1, and Task Mode.
typedef enum
{
	/// \brief The Parts Rotation Transformation is active.
	TASKSTATUS2_RotationActive = (1u << 0),
	/// \brief The Polar Coordinate Transformation is active.
	TASKSTATUS2_RThetaPolarActive = (1u << 1),
	/// \brief The Cylindrical Coordinate Transformation is active.
	TASKSTATUS2_RThetaCylindricalActive = (1u << 2),
	/// \brief Parts scaling is active.
	TASKSTATUS2_ScalingActive = (1u << 3),
	/// \brief Fixture offsets are active.
	TASKSTATUS2_OffsetFixtureActive = (1u << 4),
	/// \brief Profile motion is active.
	TASKSTATUS2_ProfileActive = (1u << 5),
	/// \brief Rapid motion is active.
	TASKSTATUS2_MotionModeRapid = (1u << 6),
	/// \brief Coordinated motion is active.
	TASKSTATUS2_MotionModeCoordinated = (1u << 7),
	/// \brief PVT motion is being executed.
	TASKSTATUS2_MotionPVT = (1u << 8),
	/// \brief The task is actively velocity profiling.
	TASKSTATUS2_MotionContinuousActive = (1u << 9),
	/// \brief Instantaneous acceleration will occur at the beginning of the move.
	TASKSTATUS2_MotionNoAccel = (1u << 10),
	/// \brief FIBER motion is active.
	TASKSTATUS2_MotionFiber = (1u << 11),
	/// \brief Positive cutter offset compensation is active.
	TASKSTATUS2_CutterOffsetsActivePos = (1u << 12),
	/// \brief Cutter radius compensation left is active.
	TASKSTATUS2_CutterRadiusActiveLeft = (1u << 13),
	/// \brief Cutter radius compensation right is active.
	TASKSTATUS2_CutterRadiusActiveRight = (1u << 14),
	/// \brief Negative cutter offset compensation is active.
	TASKSTATUS2_CutterOffsetsActiveNeg = (1u << 15),
	/// \brief Normalcy left is active.
	TASKSTATUS2_NormalcyActiveLeft = (1u << 16),
	/// \brief Normalcy right is active.
	TASKSTATUS2_NormalcyActiveRight = (1u << 17),
	/// \brief A normalcy alignment move is being performed.
	TASKSTATUS2_NormalcyAlignment = (1u << 18),
	/// \brief The motion mode is CW.
	TASKSTATUS2_MotionModeCW = (1u << 19),
	/// \brief The motion mode is CCW.
	TASKSTATUS2_MotionModeCCW = (1u << 20),
	/// \brief Feedrate limiting is active.
	TASKSTATUS2_LimitFeedRateActive = (1u << 21),
	/// \brief MFO limiting is active.
	TASKSTATUS2_LimitMFOActive = (1u << 22),
	/// \brief Coordinate System 1 Plane 1 is active.
	TASKSTATUS2_Coord1Plane1 = (1u << 23),
	/// \brief Coordinate System 1 Plane 2 is active.
	TASKSTATUS2_Coord1Plane2 = (1u << 24),
	/// \brief Coordinate System 1 Plane 3 is active.
	TASKSTATUS2_Coord1Plane3 = (1u << 25),
	/// \brief Coordinate System 2 Plane 1 is active.
	TASKSTATUS2_Coord2Plane1 = (1u << 26),
	/// \brief Coordinate System 2 Plane 2 is active.
	TASKSTATUS2_Coord2Plane2 = (1u << 27),
	/// \brief Coordinate System 2 Plane 3 is active.
	TASKSTATUS2_Coord2Plane3 = (1u << 28),
	/// \brief Mirroring is active.
	TASKSTATUS2_MirrorActive = (1u << 30),
}	TASKSTATUS2;
typedef enum
{
	/// \brief The Parts Rotation Transformation is active.
	TASKSTATUS2BITS_RotationActiveBit = 0,
	/// \brief The Polar Coordinate Transformation is active.
	TASKSTATUS2BITS_RThetaPolarActiveBit = 1,
	/// \brief The Cylindrical Coordinate Transformation is active.
	TASKSTATUS2BITS_RThetaCylindricalActiveBit = 2,
	/// \brief Parts scaling is active.
	TASKSTATUS2BITS_ScalingActiveBit = 3,
	/// \brief Fixture offsets are active.
	TASKSTATUS2BITS_OffsetFixtureActiveBit = 4,
	/// \brief Profile motion is active.
	TASKSTATUS2BITS_ProfileActiveBit = 5,
	/// \brief Rapid motion is active.
	TASKSTATUS2BITS_MotionModeRapidBit = 6,
	/// \brief Coordinated motion is active.
	TASKSTATUS2BITS_MotionModeCoordinatedBit = 7,
	/// \brief PVT motion is being executed.
	TASKSTATUS2BITS_MotionPVTBit = 8,
	/// \brief The task is actively velocity profiling.
	TASKSTATUS2BITS_MotionContinuousActiveBit = 9,
	/// \brief Instantaneous acceleration will occur at the beginning of the move.
	TASKSTATUS2BITS_MotionNoAccelBit = 10,
	/// \brief FIBER motion is active.
	TASKSTATUS2BITS_MotionFiberBit = 11,
	/// \brief Positive cutter offset compensation is active.
	TASKSTATUS2BITS_CutterOffsetsActivePosBit = 12,
	/// \brief Cutter radius compensation left is active.
	TASKSTATUS2BITS_CutterRadiusActiveLeftBit = 13,
	/// \brief Cutter radius compensation right is active.
	TASKSTATUS2BITS_CutterRadiusActiveRightBit = 14,
	/// \brief Negative cutter offset compensation is active.
	TASKSTATUS2BITS_CutterOffsetsActiveNegBit = 15,
	/// \brief Normalcy left is active.
	TASKSTATUS2BITS_NormalcyActiveLeftBit = 16,
	/// \brief Normalcy right is active.
	TASKSTATUS2BITS_NormalcyActiveRightBit = 17,
	/// \brief A normalcy alignment move is being performed.
	TASKSTATUS2BITS_NormalcyAlignmentBit = 18,
	/// \brief The motion mode is CW.
	TASKSTATUS2BITS_MotionModeCWBit = 19,
	/// \brief The motion mode is CCW.
	TASKSTATUS2BITS_MotionModeCCWBit = 20,
	/// \brief Feedrate limiting is active.
	TASKSTATUS2BITS_LimitFeedRateActiveBit = 21,
	/// \brief MFO limiting is active.
	TASKSTATUS2BITS_LimitMFOActiveBit = 22,
	/// \brief Coordinate System 1 Plane 1 is active.
	TASKSTATUS2BITS_Coord1Plane1Bit = 23,
	/// \brief Coordinate System 1 Plane 2 is active.
	TASKSTATUS2BITS_Coord1Plane2Bit = 24,
	/// \brief Coordinate System 1 Plane 3 is active.
	TASKSTATUS2BITS_Coord1Plane3Bit = 25,
	/// \brief Coordinate System 2 Plane 1 is active.
	TASKSTATUS2BITS_Coord2Plane1Bit = 26,
	/// \brief Coordinate System 2 Plane 2 is active.
	TASKSTATUS2BITS_Coord2Plane2Bit = 27,
	/// \brief Coordinate System 2 Plane 3 is active.
	TASKSTATUS2BITS_Coord2Plane3Bit = 28,
	/// \brief Mirroring is active.
	TASKSTATUS2BITS_MirrorActiveBit = 30,
}	TASKSTATUS2BITS;

/// \brief This value provides status information about this task. The following table describes each bit of this value. More status information for the task is reported in Task Status 0, Task Status 1, and Task Status 2.
typedef enum
{
	/// \brief Secondary units mode is in use.
	TASKMODE_Secondary = (1u << 0),
	/// \brief Absolute programming mode is in use.
	TASKMODE_Absolute = (1u << 1),
	/// \brief The acceleration type is configured for linear ramping.
	TASKMODE_AccelTypeLinear = (1u << 2),
	/// \brief The acceleration mode is rate-based.
	TASKMODE_AccelModeRate = (1u << 3),
	/// \brief Inverse dominance (G98) mode is active.
	TASKMODE_InverseDominance = (1u << 4),
	/// \brief Motion continuous (VELOCITY ON) mode is active.
	TASKMODE_MotionContinuous = (1u << 5),
	/// \brief Inverse circular (G111) mode is active.
	TASKMODE_InverseCircular = (1u << 6),
	/// \brief Spindles will abort on a program stop (G101).
	TASKMODE_SpindleStopOnProgramHalt = (1u << 7),
	/// \brief Block Delete mode is active (G112).
	TASKMODE_BlockDelete = (1u << 8),
	/// \brief Optional Pause mode is active (G114).
	TASKMODE_OptionalPause = (1u << 9),
	/// \brief The acceleration type is configured for scurve ramping.
	TASKMODE_AccelTypeScurve = (1u << 10),
	/// \brief MFO Lock mode is active (M48).
	TASKMODE_MFOLock = (1u << 11),
	/// \brief MSO Lock mode is active (M50).
	TASKMODE_MSOLock = (1u << 12),
	/// \brief The deceleration type is configured for linear ramping.
	TASKMODE_DecelTypeLinear = (1u << 13),
	/// \brief The deceleration type is configured for scurve ramping.
	TASKMODE_DecelTypeScurve = (1u << 14),
	/// \brief When this bit is true the task is executing in Auto mode. When this bit is false the task is executing in Step mode.
	TASKMODE_AutoMode = (1u << 15),
	/// \brief Programmed feed rates are specified in MPU (G93).
	TASKMODE_ProgramFeedRateMPU = (1u << 16),
	/// \brief Programmed feed rates are specified in units per revolution (G95).
	TASKMODE_ProgramFeedRateUPR = (1u << 17),
	/// \brief Block Delete 2 mode is active (G212).
	TASKMODE_BlockDelete2 = (1u << 22),
	/// \brief When this bit is true the task is executing in Over mode. When this bit is false the task is executing in Into mode.
	TASKMODE_OverMode = (1u << 23),
	/// \brief The deceleration mode is rate-based.
	TASKMODE_DecelModeRate = (1u << 24),
	/// \brief The task is using High-Speed Lookahead (LOOKAHEAD FAST).
	TASKMODE_HighSpeedLookAhead = (1u << 25),
	/// \brief MFO will affect asynchronous motion (OVERRIDEASYNC ON).
	TASKMODE_MFOActiveOnJog = (1u << 26),
	/// \brief The WAIT MODE INPOS wait mode is active.
	TASKMODE_WaitForInPos = (1u << 27),
	/// \brief When this bit is true the time mode is MINUTES. When this bit is false the time mode is SECONDS.
	TASKMODE_Minutes = (1u << 28),
	/// \brief The WAIT MODE AUTO wait mode is active.
	TASKMODE_WaitAuto = (1u << 30),
}	TASKMODE;
typedef enum
{
	/// \brief Secondary units mode is in use.
	TASKMODEBITS_SecondaryBit = 0,
	/// \brief Absolute programming mode is in use.
	TASKMODEBITS_AbsoluteBit = 1,
	/// \brief The acceleration type is configured for linear ramping.
	TASKMODEBITS_AccelTypeLinearBit = 2,
	/// \brief The acceleration mode is rate-based.
	TASKMODEBITS_AccelModeRateBit = 3,
	/// \brief Inverse dominance (G98) mode is active.
	TASKMODEBITS_InverseDominanceBit = 4,
	/// \brief Motion continuous (VELOCITY ON) mode is active.
	TASKMODEBITS_MotionContinuousBit = 5,
	/// \brief Inverse circular (G111) mode is active.
	TASKMODEBITS_InverseCircularBit = 6,
	/// \brief Spindles will abort on a program stop (G101).
	TASKMODEBITS_SpindleStopOnProgramHaltBit = 7,
	/// \brief Block Delete mode is active (G112).
	TASKMODEBITS_BlockDeleteBit = 8,
	/// \brief Optional Pause mode is active (G114).
	TASKMODEBITS_OptionalPauseBit = 9,
	/// \brief The acceleration type is configured for scurve ramping.
	TASKMODEBITS_AccelTypeScurveBit = 10,
	/// \brief MFO Lock mode is active (M48).
	TASKMODEBITS_MFOLockBit = 11,
	/// \brief MSO Lock mode is active (M50).
	TASKMODEBITS_MSOLockBit = 12,
	/// \brief The deceleration type is configured for linear ramping.
	TASKMODEBITS_DecelTypeLinearBit = 13,
	/// \brief The deceleration type is configured for scurve ramping.
	TASKMODEBITS_DecelTypeScurveBit = 14,
	/// \brief When this bit is true the task is executing in Auto mode. When this bit is false the task is executing in Step mode.
	TASKMODEBITS_AutoModeBit = 15,
	/// \brief Programmed feed rates are specified in MPU (G93).
	TASKMODEBITS_ProgramFeedRateMPUBit = 16,
	/// \brief Programmed feed rates are specified in units per revolution (G95).
	TASKMODEBITS_ProgramFeedRateUPRBit = 17,
	/// \brief Block Delete 2 mode is active (G212).
	TASKMODEBITS_BlockDelete2Bit = 22,
	/// \brief When this bit is true the task is executing in Over mode. When this bit is false the task is executing in Into mode.
	TASKMODEBITS_OverModeBit = 23,
	/// \brief The deceleration mode is rate-based.
	TASKMODEBITS_DecelModeRateBit = 24,
	/// \brief The task is using High-Speed Lookahead (LOOKAHEAD FAST).
	TASKMODEBITS_HighSpeedLookAheadBit = 25,
	/// \brief MFO will affect asynchronous motion (OVERRIDEASYNC ON).
	TASKMODEBITS_MFOActiveOnJogBit = 26,
	/// \brief The WAIT MODE INPOS wait mode is active.
	TASKMODEBITS_WaitForInPosBit = 27,
	/// \brief When this bit is true the time mode is MINUTES. When this bit is false the time mode is SECONDS.
	TASKMODEBITS_MinutesBit = 28,
	/// \brief The WAIT MODE AUTO wait mode is active.
	TASKMODEBITS_WaitAutoBit = 30,
}	TASKMODEBITS;

/// \brief This value provides status information about the queue of a given task. The following table describes each bit of this value.
typedef enum
{
	/// \brief This task is running in the Queue mode.
	QUEUESTATUS_QueueModeActive = (1u << 0),
	/// \brief The Queue buffer for this task is empty.
	QUEUESTATUS_QueueBufferEmpty = (1u << 1),
	/// \brief The Queue buffer for this task is full.
	QUEUESTATUS_QueueBufferFull = (1u << 2),
	/// \brief The Queue buffer for this task started.
	QUEUESTATUS_QueueBufferStarted = (1u << 3),
	/// \brief The Queue buffer for this task was paused.
	QUEUESTATUS_QueueBufferPaused = (1u << 4),
	/// \brief The given task is executing a large program line-by-line in queue mode.
	QUEUESTATUS_QueueLargeProgramExecuting = (1u << 5),
}	QUEUESTATUS;
typedef enum
{
	/// \brief This task is running in the Queue mode.
	QUEUESTATUSBITS_QueueModeActiveBit = 0,
	/// \brief The Queue buffer for this task is empty.
	QUEUESTATUSBITS_QueueBufferEmptyBit = 1,
	/// \brief The Queue buffer for this task is full.
	QUEUESTATUSBITS_QueueBufferFullBit = 2,
	/// \brief The Queue buffer for this task started.
	QUEUESTATUSBITS_QueueBufferStartedBit = 3,
	/// \brief The Queue buffer for this task was paused.
	QUEUESTATUSBITS_QueueBufferPausedBit = 4,
	/// \brief The given task is executing a large program line-by-line in queue mode.
	QUEUESTATUSBITS_QueueLargeProgramExecutingBit = 5,
}	QUEUESTATUSBITS;

/// \brief Represents the modes to wait for motion completion
typedef enum
{
	/// \brief Wait for MoveDone bit to be set
	WAITOPTION_MoveDone = 0,
	/// \brief Wait for InPosition bit to be set
	WAITOPTION_InPosition = 1,
}	WAITOPTION;

/// \brief Specifies the status flags of data collection
typedef enum
{
	/// \brief Data collection was triggered
	DATACOLLECTIONFLAGS_Triggered = (1u << 2),
	/// \brief Data collection is done
	DATACOLLECTIONFLAGS_Done = (1u << 3),
	/// \brief Data collection buffer overflowed
	DATACOLLECTIONFLAGS_Overflow = (1u << 4),
	/// \brief Continuous data collection mode is active
	DATACOLLECTIONFLAGS_ContinuousMode = (1u << 9),
	/// \brief Data collection was started by a SCOPETRIG
	DATACOLLECTIONFLAGS_IsScopeTrigInitiated = (1u << 11),
	/// \brief Data is being uploaded to the SMC from the drive buffers
	DATACOLLECTIONFLAGS_UploadingDriveBuffers = (1u << 16),
}	DATACOLLECTIONFLAGS;
typedef enum
{
	/// \brief Data collection was triggered
	DATACOLLECTIONFLAGSBITS_TriggeredBit = 2,
	/// \brief Data collection is done
	DATACOLLECTIONFLAGSBITS_DoneBit = 3,
	/// \brief Data collection buffer overflowed
	DATACOLLECTIONFLAGSBITS_OverflowBit = 4,
	/// \brief Continuous data collection mode is active
	DATACOLLECTIONFLAGSBITS_ContinuousModeBit = 9,
	/// \brief Data collection was started by a SCOPETRIG
	DATACOLLECTIONFLAGSBITS_IsScopeTrigInitiatedBit = 11,
	/// \brief Data is being uploaded to the SMC from the drive buffers
	DATACOLLECTIONFLAGSBITS_UploadingDriveBuffersBit = 16,
}	DATACOLLECTIONFLAGSBITS;

/// \brief The type of loop transmission disturbance to use
typedef enum
{
	/// \brief Turn off loop transmission
	LOOPTRANSMISSIONMODE_Off = 0,
	/// \brief Uses a sinusoid disturbance
	LOOPTRANSMISSIONMODE_Sinusoid = 1,
	/// \brief Uses a sinusoid disturbance and excites both axes of a gantry
	LOOPTRANSMISSIONMODE_SinusoidGantry = 3,
	/// \brief Uses a white noise disturbance
	LOOPTRANSMISSIONMODE_WhiteNoise = 2,
	/// \brief Use a white noise disturbance and excites both axes of a gantry
	LOOPTRANSMISSIONMODE_WhiteNoiseGantry = 4,
}	LOOPTRANSMISSIONMODE;

/// \brief The loop transmission type to use
typedef enum
{
	/// \brief Open Loop
	LOOPTRANSMISSIONTYPE_OpenLoop = 0,
	/// \brief Closed Loop
	LOOPTRANSMISSIONTYPE_ClosedLoop = 1,
	/// \brief Current Loop
	LOOPTRANSMISSIONTYPE_CurrentLoop = 2,
	/// \brief AF Open Loop
	LOOPTRANSMISSIONTYPE_AFOpenLoop = 3,
	/// \brief AF Closed Loop
	LOOPTRANSMISSIONTYPE_AFClosedLoop = 4,
}	LOOPTRANSMISSIONTYPE;

/// \brief Represents the OnOff mode in AeroBasic
typedef enum
{
	/// \brief Off or 0 is issued
	ONOFF_Off = 0,
	/// \brief On or 1 is issued
	ONOFF_On = 1,
}	ONOFF;

/// \brief Represents the PSO Encoder in AeroBasic
typedef enum
{
	/// \brief Primary PSO encoder
	PSOENCODER_Primary = 0,
	/// \brief Auxiliary PSO encoder
	PSOENCODER_Auxiliary = 1,
}	PSOENCODER;

/// \brief Represents the PSO mode in AeroBasic
typedef enum
{
	/// \brief Reset PSO
	PSOMODE_Reset = 0,
	/// \brief Turn off PSO
	PSOMODE_Off = 1,
	/// \brief Arm PSO
	PSOMODE_Arm = 2,
	/// \brief Fire PSO
	PSOMODE_Fire = 3,
	/// \brief Turn on PSO
	PSOMODE_On = 4,
	/// \brief Fire Continuous
	PSOMODE_FireContinuous = 5,
	/// \brief Arm PSO PWM
	PSOMODE_ArmPwm = 6,
}	PSOMODE;

/// \brief Specifies the window control mode to use
typedef enum
{
	/// \brief The windows are codependent
	PSOWINDOWDEPENDANCE_CoDependent = 0,
	/// \brief The windows are independent
	PSOWINDOWDEPENDANCE_Independent = 1,
}	PSOWINDOWDEPENDANCE;

/// \brief Modbus registers
typedef enum
{
	/// \brief Master input words (read only)
	REGISTERTYPE_MasterInputWords = 0,
	/// \brief Slave input words (read only)
	REGISTERTYPE_SlaveInputWords = 1,
	/// \brief Master Output words
	REGISTERTYPE_MasterOutputWords = 2,
	/// \brief Slave Output words
	REGISTERTYPE_SlaveOutputWords = 3,
	/// \brief Master input Bits (read only)
	REGISTERTYPE_MasterInputBits = 4,
	/// \brief Slave input Bits (read only)
	REGISTERTYPE_SlaveInputBits = 5,
	/// \brief Master Output Bits
	REGISTERTYPE_MasterOutputBits = 6,
	/// \brief Slave Output Bits
	REGISTERTYPE_SlaveOutputBits = 7,
}	REGISTERTYPE;

/// \brief Specifies the button that was clicked as a result of callback
typedef enum
{
	/// \brief Ok button was clicked
	INPUTBOXCLICKEDBUTTON_Ok = 1,
	/// \brief Cancel button was clicked
	INPUTBOXCLICKEDBUTTON_Cancel = 2,
}	INPUTBOXCLICKEDBUTTON;

/// \brief The kind of data to return in callbacks
typedef enum
{
	/// \brief Return a double
	INPUTBOXKIND_Double = 0,
	/// \brief Return an integer
	INPUTBOXKIND_Integer = 8388608,
	/// \brief Return a string
	INPUTBOXKIND_String = 4194304,
}	INPUTBOXKIND;

/// \brief Specifies the build result kind
typedef enum
{
	/// \brief The build result describes some informational (non-critical) detail about the compilation
	BUILDRESULTKIND_Information = 0,
	/// \brief The build result describes a warning that occurred during the compilation
	BUILDRESULTKIND_Warning = 1,
	/// \brief The build result describes an error that occurred during the compilation
	BUILDRESULTKIND_Error = 2,
}	BUILDRESULTKIND;

/// \brief 
typedef enum
{
	/// \brief The dimensionality of the calibration file is unknown
	AXISCALIBRATION_FILETYPE_UNKNOWN = 0,
	/// \brief The axis calibration file is 1 dimensional
	AXISCALIBRATION_FILETYPE_1D = 1,
	/// \brief The axis calibration file is 2 dimensional
	AXISCALIBRATION_FILETYPE_2D = 2,
	/// \brief The galvo calibration file is 2 dimensional
	AXISCALIBRATION_FILETYPE_GALVO_2D = 3,
}	AXISCALIBRATION;

/// \brief Represents the ramp type in AeroBasic
typedef enum
{
	/// \brief Linear-based ramp type
	RAMPTYPE_Linear = 0,
	/// \brief Scurve-based ramp type
	RAMPTYPE_Scurve = 1,
	/// \brief Sine-based ramp type
	RAMPTYPE_Sine = 2,
}	RAMPTYPE;

/// \brief Represents the ramp mode in AeroBasic
typedef enum
{
	/// \brief Rate-based acceleration and deceleration
	RAMPMODE_Rate = 0,
	/// \brief Time-based acceleration and deceleration
	RAMPMODE_Time = 1,
}	RAMPMODE;

/// \brief The signal from the master axis from which the slave axis is geared
typedef enum
{
	/// \brief Track the position feedback of the master
	GEARINGTRACKINGMODE_PositionFeedback = 0,
	/// \brief Track the position command of the master
	GEARINGTRACKINGMODE_PositionCommand = 1,
	/// \brief Track the auxiliary encoder channel of the master
	GEARINGTRACKINGMODE_AuxiliaryEncoder = 2,
}	GEARINGTRACKINGMODE;

/// \brief Signal the cam table will track when real axis is used as the master
typedef enum
{
	/// \brief Track the position feedback of the master
	CAMMINGTRACKINGMODE_PositionFeedback = 0,
	/// \brief Track the position command of the master
	CAMMINGTRACKINGMODE_PositionCommand = 1,
	/// \brief Track the auxiliary encoder channel of the master
	CAMMINGTRACKINGMODE_AuxiliaryEncoder = 2,
}	CAMMINGTRACKINGMODE;

/// \brief Represents the Interpolation Type to be used for the cam table being loaded
typedef enum
{
	/// \brief Use linear interpolation
	CAMMINGINTERPOLATIONTYPE_Linear = 0,
	/// \brief Use a cubic spline to interpolate between points
	CAMMINGINTERPOLATIONTYPE_CubicSpline = 1,
}	CAMMINGINTERPOLATIONTYPE;

/// \brief The synchronization mode to use for camming
typedef enum
{
	/// \brief Stop the slave axis from synchronizing with the master
	CAMMINGSYNCMODE_Stop = 0,
	/// \brief Begin camming with relative synchronization
	CAMMINGSYNCMODE_Relative = 1,
	/// \brief Begin camming with absolute synchronization
	CAMMINGSYNCMODE_Absolute = 2,
	/// \brief Begin camming where slave values are interpreted as velocities and not positions
	CAMMINGSYNCMODE_Velocity = 3,
}	CAMMINGSYNCMODE;

/// \brief The edge kinds that are available.
typedef enum
{
	/// \brief Represents a rising edge.
	EDGE_Rising = 0,
	/// \brief Represents a falling edge.
	EDGE_Falling = 1,
	/// \brief Represents a rising or falling edge.
	EDGE_RisingOrFalling = 2,
}	EDGE;

/// \brief Specifies the time base units.
typedef enum
{
	/// \brief The time units are seconds.
	TIMEUNIT_Seconds = 0,
	/// \brief The time units are minutes.
	TIMEUNIT_Minutes = 1,
	/// \brief The time units are milliseconds.
	TIMEUNIT_Milliseconds = 2,
	/// \brief The time units are microseconds.
	TIMEUNIT_Microseconds = 3,
}	TIMEUNIT;

/// \brief The window range activities that are available.
typedef enum
{
	/// \brief Represents entering a window range.
	WINDOWEVENT_Entering = 0,
	/// \brief Represents exiting a window range.
	WINDOWEVENT_Exiting = 1,
	/// \brief Represents entering or exiting a window range.
	WINDOWEVENT_EnteringOrExiting = 2,
}	WINDOWEVENT;

/// \brief This value provides status information about Sensor Fusion Data Recording. The following table describes each bit of this value.
typedef enum
{
	/// \brief The Sensor Fusion is currently executing a data recording session.
	DATARECORDINGFLAGS_Recording = (1u << 0),
	/// \brief The Sensor Fusion data recording buffer overflowed.
	DATARECORDINGFLAGS_Overflowed = (1u << 1),
}	DATARECORDINGFLAGS;
typedef enum
{
	/// \brief The Sensor Fusion is currently executing a data recording session.
	DATARECORDINGFLAGSBITS_RecordingBit = 0,
	/// \brief The Sensor Fusion data recording buffer overflowed.
	DATARECORDINGFLAGSBITS_OverflowedBit = 1,
}	DATARECORDINGFLAGSBITS;

/// \brief This value provides status information about Sensor Fusion Data Playback. The following table describes each bit of this value.
typedef enum
{
	/// \brief The Sensor Fusion is currently executing a data playback session.
	DATAPLAYBACKFLAGS_Playing = (1u << 0),
	/// \brief The Sensor Fusion data playback buffer underflowed.
	DATAPLAYBACKFLAGS_Underflowed = (1u << 1),
}	DATAPLAYBACKFLAGS;
typedef enum
{
	/// \brief The Sensor Fusion is currently executing a data playback session.
	DATAPLAYBACKFLAGSBITS_PlayingBit = 0,
	/// \brief The Sensor Fusion data playback buffer underflowed.
	DATAPLAYBACKFLAGSBITS_UnderflowedBit = 1,
}	DATAPLAYBACKFLAGSBITS;

/// \brief Represents the callback argument type
typedef enum
{
	/// \brief Callback double type argument
	CALLBACKARGUMENTTYPE_Double = 1,
	/// \brief Callback string type argument containing up to 187 characters
	CALLBACKARGUMENTTYPE_String = 3,
}	CALLBACKARGUMENTTYPE;

/// \brief This value specifies the type of a safe zone.
typedef enum
{
	/// \brief The zone cannot be entered.
	SAFEZONETYPE_NoEnter = 0,
	/// \brief The zone cannot be exited.
	SAFEZONETYPE_NoExit = 1,
	/// \brief The zone cannot be entered. A safe zone axis fault is generated after the decelerations complete.
	SAFEZONETYPE_NoEnterAxisFault = 2,
	/// \brief The zone cannot be exited. A safe zone axis fault is generated after the decelerations complete.
	SAFEZONETYPE_NoExitAxisFault = 3,
}	SAFEZONETYPE;

#endif // __A3200_COMMON_STRUCTURES_H__

