// PIControllerErrors.h
//
// This file defines symbols for each error code used in C/C++ programs.
//
// This file is automagically generated from the central error code list.
// DO NOT ADD ERROR CODES IN THIS FILE! Use the error list and the generation tool instead!
//
// after generation open file in VS and create the constants with numbers.
// This can be done by performing 2 search/replace actions with regulyr expression activetad:
// search for "\#define:b{:i}:b*{:z}L" replace with "#define \1__\2 \2L\n#define \1 \2L"
// search for "\#define:b{:i}:b*-{:z}L" replace with "#define \1__MINUS\2 -\2L\n#define \1 -\2L"

#ifndef __PI_CONTROLLER_ERROS_H__
#define __PI_CONTROLLER_ERROS_H__

//////////////////////////////////////////////////
//
// Dll Errors - DLL errors occured in GCS DLL
//
#define PI_UNKNOWN_AXIS_IDENTIFIER__MINUS1001 -1001L
#define PI_UNKNOWN_AXIS_IDENTIFIER -1001L  /**< \ingroup err
		Unknown axis identifier */
#define PI_NR_NAV_OUT_OF_RANGE__MINUS1002 -1002L
#define PI_NR_NAV_OUT_OF_RANGE -1002L  /**< \ingroup err
		Number for NAV out of range--must be in [1,10000] */
#define PI_INVALID_SGA__MINUS1003 -1003L
#define PI_INVALID_SGA -1003L  /**< \ingroup err
		Invalid value for SGA--must be one of {1, 10, 100, 1000} */
#define PI_UNEXPECTED_RESPONSE__MINUS1004 -1004L
#define PI_UNEXPECTED_RESPONSE -1004L  /**< \ingroup err
		Controller sent unexpected response */
#define PI_NO_MANUAL_PAD__MINUS1005 -1005L
#define PI_NO_MANUAL_PAD -1005L  /**< \ingroup err
		No manual control pad installed, calls to SMA and related commands are not allowed */
#define PI_INVALID_MANUAL_PAD_KNOB__MINUS1006 -1006L
#define PI_INVALID_MANUAL_PAD_KNOB -1006L  /**< \ingroup err
		Invalid number for manual control pad knob */
#define PI_INVALID_MANUAL_PAD_AXIS__MINUS1007 -1007L
#define PI_INVALID_MANUAL_PAD_AXIS -1007L  /**< \ingroup err
		Axis not currently controlled by a manual control pad */
#define PI_CONTROLLER_BUSY__MINUS1008 -1008L
#define PI_CONTROLLER_BUSY -1008L  /**< \ingroup err
		Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm) */
#define PI_THREAD_ERROR__MINUS1009 -1009L
#define PI_THREAD_ERROR -1009L  /**< \ingroup err
		Internal error--could not start thread */
#define PI_IN_MACRO_MODE__MINUS1010 -1010L
#define PI_IN_MACRO_MODE -1010L  /**< \ingroup err
		Controller is (already) in macro mode--command not valid in macro mode */
#define PI_NOT_IN_MACRO_MODE__MINUS1011 -1011L
#define PI_NOT_IN_MACRO_MODE -1011L  /**< \ingroup err
		Controller not in macro mode--command not valid unless macro mode active */
#define PI_MACRO_FILE_ERROR__MINUS1012 -1012L
#define PI_MACRO_FILE_ERROR -1012L  /**< \ingroup err
		Could not open file to write or read macro */
#define PI_NO_MACRO_OR_EMPTY__MINUS1013 -1013L
#define PI_NO_MACRO_OR_EMPTY -1013L  /**< \ingroup err
		No macro with given name on controller, or macro is empty */
#define PI_MACRO_EDITOR_ERROR__MINUS1014 -1014L
#define PI_MACRO_EDITOR_ERROR -1014L  /**< \ingroup err
		Internal error in macro editor */
#define PI_INVALID_ARGUMENT__MINUS1015 -1015L
#define PI_INVALID_ARGUMENT -1015L  /**< \ingroup err
		One or more arguments given to function is invalid (empty string, index out of range, ...) */
#define PI_AXIS_ALREADY_EXISTS__MINUS1016 -1016L
#define PI_AXIS_ALREADY_EXISTS -1016L  /**< \ingroup err
		Axis identifier is already in use by a connected stage */
#define PI_INVALID_AXIS_IDENTIFIER__MINUS1017 -1017L
#define PI_INVALID_AXIS_IDENTIFIER -1017L  /**< \ingroup err
		Invalid axis identifier */
#define PI_COM_ARRAY_ERROR__MINUS1018 -1018L
#define PI_COM_ARRAY_ERROR -1018L  /**< \ingroup err
		Could not access array data in COM server */
#define PI_COM_ARRAY_RANGE_ERROR__MINUS1019 -1019L
#define PI_COM_ARRAY_RANGE_ERROR -1019L  /**< \ingroup err
		Range of array does not fit the number of parameters */
#define PI_INVALID_SPA_CMD_ID__MINUS1020 -1020L
#define PI_INVALID_SPA_CMD_ID -1020L  /**< \ingroup err
		Invalid parameter ID given to SPA or SPA? */
#define PI_NR_AVG_OUT_OF_RANGE__MINUS1021 -1021L
#define PI_NR_AVG_OUT_OF_RANGE -1021L  /**< \ingroup err
		Number for AVG out of range--must be >0 */
#define PI_WAV_SAMPLES_OUT_OF_RANGE__MINUS1022 -1022L
#define PI_WAV_SAMPLES_OUT_OF_RANGE -1022L  /**< \ingroup err
		Incorrect number of samples given to WAV */
#define PI_WAV_FAILED__MINUS1023 -1023L
#define PI_WAV_FAILED -1023L  /**< \ingroup err
		Generation of wave failed */
#define PI_MOTION_ERROR__MINUS1024 -1024L
#define PI_MOTION_ERROR -1024L  /**< \ingroup err
		Motion error: position error too large, servo is switched off automatically */
#define PI_RUNNING_MACRO__MINUS1025 -1025L
#define PI_RUNNING_MACRO -1025L  /**< \ingroup err
		Controller is (already) running a macro */
#define PI_PZT_CONFIG_FAILED__MINUS1026 -1026L
#define PI_PZT_CONFIG_FAILED -1026L  /**< \ingroup err
		Configuration of PZT stage or amplifier failed */
#define PI_PZT_CONFIG_INVALID_PARAMS__MINUS1027 -1027L
#define PI_PZT_CONFIG_INVALID_PARAMS -1027L  /**< \ingroup err
		Current settings are not valid for desired configuration */
#define PI_UNKNOWN_CHANNEL_IDENTIFIER__MINUS1028 -1028L
#define PI_UNKNOWN_CHANNEL_IDENTIFIER -1028L  /**< \ingroup err
		Unknown channel identifier */
#define PI_WAVE_PARAM_FILE_ERROR__MINUS1029 -1029L
#define PI_WAVE_PARAM_FILE_ERROR -1029L  /**< \ingroup err
		Error while reading/writing wave generator parameter file */
#define PI_UNKNOWN_WAVE_SET__MINUS1030 -1030L
#define PI_UNKNOWN_WAVE_SET -1030L  /**< \ingroup err
		Could not find description of wave form. Maybe WG.INI is missing? */
#define PI_WAVE_EDITOR_FUNC_NOT_LOADED__MINUS1031 -1031L
#define PI_WAVE_EDITOR_FUNC_NOT_LOADED -1031L  /**< \ingroup err
		The WGWaveEditor DLL function was not found at startup */
#define PI_USER_CANCELLED__MINUS1032 -1032L
#define PI_USER_CANCELLED -1032L  /**< \ingroup err
		The user cancelled a dialog */
#define PI_C844_ERROR__MINUS1033 -1033L
#define PI_C844_ERROR -1033L  /**< \ingroup err
		Error from C-844 Controller */
#define PI_DLL_NOT_LOADED__MINUS1034 -1034L
#define PI_DLL_NOT_LOADED -1034L  /**< \ingroup err
		DLL necessary to call function not loaded, or function not found in DLL */
#define PI_PARAMETER_FILE_PROTECTED__MINUS1035 -1035L
#define PI_PARAMETER_FILE_PROTECTED -1035L  /**< \ingroup err
		The open parameter file is protected and cannot be edited */
#define PI_NO_PARAMETER_FILE_OPENED__MINUS1036 -1036L
#define PI_NO_PARAMETER_FILE_OPENED -1036L  /**< \ingroup err
		There is no parameter file open */
#define PI_STAGE_DOES_NOT_EXIST__MINUS1037 -1037L
#define PI_STAGE_DOES_NOT_EXIST -1037L  /**< \ingroup err
		Selected stage does not exist */
#define PI_PARAMETER_FILE_ALREADY_OPENED__MINUS1038 -1038L
#define PI_PARAMETER_FILE_ALREADY_OPENED -1038L  /**< \ingroup err
		There is already a parameter file open. Close it before opening a new file */
#define PI_PARAMETER_FILE_OPEN_ERROR__MINUS1039 -1039L
#define PI_PARAMETER_FILE_OPEN_ERROR -1039L  /**< \ingroup err
		Could not open parameter file */
#define PI_INVALID_CONTROLLER_VERSION__MINUS1040 -1040L
#define PI_INVALID_CONTROLLER_VERSION -1040L  /**< \ingroup err
		The version of the connected controller is invalid */
#define PI_PARAM_SET_ERROR__MINUS1041 -1041L
#define PI_PARAM_SET_ERROR -1041L  /**< \ingroup err
		Parameter could not be set with SPA--parameter not defined for this controller! */
#define PI_NUMBER_OF_POSSIBLE_WAVES_EXCEEDED__MINUS1042 -1042L
#define PI_NUMBER_OF_POSSIBLE_WAVES_EXCEEDED -1042L  /**< \ingroup err
		The maximum number of wave definitions has been exceeded */
#define PI_NUMBER_OF_POSSIBLE_GENERATORS_EXCEEDED__MINUS1043 -1043L
#define PI_NUMBER_OF_POSSIBLE_GENERATORS_EXCEEDED -1043L  /**< \ingroup err
		The maximum number of wave generators has been exceeded */
#define PI_NO_WAVE_FOR_AXIS_DEFINED__MINUS1044 -1044L
#define PI_NO_WAVE_FOR_AXIS_DEFINED -1044L  /**< \ingroup err
		No wave defined for specified axis */
#define PI_CANT_STOP_OR_START_WAV__MINUS1045 -1045L
#define PI_CANT_STOP_OR_START_WAV -1045L  /**< \ingroup err
		Wave output to axis already stopped/started */
#define PI_REFERENCE_ERROR__MINUS1046 -1046L
#define PI_REFERENCE_ERROR -1046L  /**< \ingroup err
		Not all axes could be referenced */
#define PI_REQUIRED_WAVE_NOT_FOUND__MINUS1047 -1047L
#define PI_REQUIRED_WAVE_NOT_FOUND -1047L  /**< \ingroup err
		Could not find parameter set required by frequency relation */
#define PI_INVALID_SPP_CMD_ID__MINUS1048 -1048L
#define PI_INVALID_SPP_CMD_ID -1048L  /**< \ingroup err
		Command ID given to SPP or SPP? is not valid */
#define PI_STAGE_NAME_ISNT_UNIQUE__MINUS1049 -1049L
#define PI_STAGE_NAME_ISNT_UNIQUE -1049L  /**< \ingroup err
		A stage name given to CST is not unique */
#define PI_FILE_TRANSFER_BEGIN_MISSING__MINUS1050 -1050L
#define PI_FILE_TRANSFER_BEGIN_MISSING -1050L  /**< \ingroup err
		A uuencoded file transfered did not start with \"begin\" followed by the proper filename */
#define PI_FILE_TRANSFER_ERROR_TEMP_FILE__MINUS1051 -1051L
#define PI_FILE_TRANSFER_ERROR_TEMP_FILE -1051L  /**< \ingroup err
		Could not create/read file on host PC */
#define PI_FILE_TRANSFER_CRC_ERROR__MINUS1052 -1052L
#define PI_FILE_TRANSFER_CRC_ERROR -1052L  /**< \ingroup err
		Checksum error when transfering a file to/from the controller */
#define PI_COULDNT_FIND_PISTAGES_DAT__MINUS1053 -1053L
#define PI_COULDNT_FIND_PISTAGES_DAT -1053L  /**< \ingroup err
		The PiStages.dat database could not be found. This file is required to connect a stage with the CST command */
#define PI_NO_WAVE_RUNNING__MINUS1054 -1054L
#define PI_NO_WAVE_RUNNING -1054L  /**< \ingroup err
		No wave being output to specified axis */
#define PI_INVALID_PASSWORD__MINUS1055 -1055L
#define PI_INVALID_PASSWORD -1055L  /**< \ingroup err
		Invalid password */
#define PI_OPM_COM_ERROR__MINUS1056 -1056L
#define PI_OPM_COM_ERROR -1056L  /**< \ingroup err
		Error during communication with OPM (Optical Power Meter), maybe no OPM connected */
#define PI_WAVE_EDITOR_WRONG_PARAMNUM__MINUS1057 -1057L
#define PI_WAVE_EDITOR_WRONG_PARAMNUM -1057L  /**< \ingroup err
		WaveEditor: Error during wave creation, incorrect number of parameters */
#define PI_WAVE_EDITOR_FREQUENCY_OUT_OF_RANGE__MINUS1058 -1058L
#define PI_WAVE_EDITOR_FREQUENCY_OUT_OF_RANGE -1058L  /**< \ingroup err
		WaveEditor: Frequency out of range */
#define PI_WAVE_EDITOR_WRONG_IP_VALUE__MINUS1059 -1059L
#define PI_WAVE_EDITOR_WRONG_IP_VALUE -1059L  /**< \ingroup err
		WaveEditor: Error during wave creation, incorrect index for integer parameter */
#define PI_WAVE_EDITOR_WRONG_DP_VALUE__MINUS1060 -1060L
#define PI_WAVE_EDITOR_WRONG_DP_VALUE -1060L  /**< \ingroup err
		WaveEditor: Error during wave creation, incorrect index for floating point parameter */
#define PI_WAVE_EDITOR_WRONG_ITEM_VALUE__MINUS1061 -1061L
#define PI_WAVE_EDITOR_WRONG_ITEM_VALUE -1061L  /**< \ingroup err
		WaveEditor: Error during wave creation, could not calculate value */
#define PI_WAVE_EDITOR_MISSING_GRAPH_COMPONENT__MINUS1062 -1062L
#define PI_WAVE_EDITOR_MISSING_GRAPH_COMPONENT -1062L  /**< \ingroup err
		WaveEditor: Graph display component not installed */
#define PI_EXT_PROFILE_UNALLOWED_CMD__MINUS1063 -1063L
#define PI_EXT_PROFILE_UNALLOWED_CMD -1063L  /**< \ingroup err
		User Profile Mode: Command is not allowed, check for required preparatory commands */
#define PI_EXT_PROFILE_EXPECTING_MOTION_ERROR__MINUS1064 -1064L
#define PI_EXT_PROFILE_EXPECTING_MOTION_ERROR -1064L  /**< \ingroup err
		User Profile Mode: First target position in User Profile is too far from current position */
#define PI_EXT_PROFILE_ACTIVE__MINUS1065 -1065L
#define PI_EXT_PROFILE_ACTIVE -1065L  /**< \ingroup err
		Controller is (already) in User Profile Mode */
#define PI_EXT_PROFILE_INDEX_OUT_OF_RANGE__MINUS1066 -1066L
#define PI_EXT_PROFILE_INDEX_OUT_OF_RANGE -1066L  /**< \ingroup err
		User Profile Mode: Block or Data Set index out of allowed range */
#define PI_PROFILE_GENERATOR_NO_PROFILE__MINUS1067 -1067L
#define PI_PROFILE_GENERATOR_NO_PROFILE -1067L  /**< \ingroup err
		ProfileGenerator: No profile has been created yet */
#define PI_PROFILE_GENERATOR_OUT_OF_LIMITS__MINUS1068 -1068L
#define PI_PROFILE_GENERATOR_OUT_OF_LIMITS -1068L  /**< \ingroup err
		ProfileGenerator: Generated profile exceeds limits of one or both axes */
#define PI_PROFILE_GENERATOR_UNKNOWN_PARAMETER__MINUS1069 -1069L
#define PI_PROFILE_GENERATOR_UNKNOWN_PARAMETER -1069L  /**< \ingroup err
		ProfileGenerator: Unknown parameter ID in Set/Get Parameter command */
#define PI_PROFILE_GENERATOR_PAR_OUT_OF_RANGE__MINUS1070 -1070L
#define PI_PROFILE_GENERATOR_PAR_OUT_OF_RANGE -1070L  /**< \ingroup err
		ProfileGenerator: Parameter out of allowed range */
#define PI_EXT_PROFILE_OUT_OF_MEMORY__MINUS1071 -1071L
#define PI_EXT_PROFILE_OUT_OF_MEMORY -1071L  /**< \ingroup err
		User Profile Mode: Out of memory */
#define PI_EXT_PROFILE_WRONG_CLUSTER__MINUS1072 -1072L
#define PI_EXT_PROFILE_WRONG_CLUSTER -1072L  /**< \ingroup err
		User Profile Mode: Cluster is not assigned to this axis */
#define PI_EXT_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER__MINUS1073 -1073L
#define PI_EXT_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER -1073L  /**< \ingroup err
		Unknown cluster identifier */
#define PI_INVALID_DEVICE_DRIVER_VERSION__MINUS1074 -1074L
#define PI_INVALID_DEVICE_DRIVER_VERSION -1074L  /**< \ingroup err
		The installed device driver doesn't match the required version. Please see the documentation to determine the required device driver version. */
#define PI_INVALID_LIBRARY_VERSION__MINUS1075 -1075L
#define PI_INVALID_LIBRARY_VERSION -1075L  /**< \ingroup err
		The library used doesn't match the required version. Please see the documentation to determine the required library version. */
#define PI_INTERFACE_LOCKED__MINUS1076 -1076L
#define PI_INTERFACE_LOCKED -1076L  /**< \ingroup err
		The interface is currently locked by another function. Please try again later. */
#define PI_PARAM_DAT_FILE_INVALID_VERSION__MINUS1077 -1077L
#define PI_PARAM_DAT_FILE_INVALID_VERSION -1077L  /**< \ingroup err
		Version of parameter DAT file does not match the required version. Current files are available at www.pi.ws. */
#define PI_CANNOT_WRITE_TO_PARAM_DAT_FILE__MINUS1078 -1078L
#define PI_CANNOT_WRITE_TO_PARAM_DAT_FILE -1078L  /**< \ingroup err
		Cannot write to parameter DAT file to store user defined stage type. */
#define PI_CANNOT_CREATE_PARAM_DAT_FILE__MINUS1079 -1079L
#define PI_CANNOT_CREATE_PARAM_DAT_FILE -1079L  /**< \ingroup err
		Cannot create parameter DAT file to store user defined stage type. */
#define PI_PARAM_DAT_FILE_INVALID_REVISION__MINUS1080 -1080L
#define PI_PARAM_DAT_FILE_INVALID_REVISION -1080L  /**< \ingroup err
		Parameter DAT file does not have correct revision. */
#define PI_USERSTAGES_DAT_FILE_INVALID_REVISION__MINUS1081 -1081L
#define PI_USERSTAGES_DAT_FILE_INVALID_REVISION -1081L  /**< \ingroup err
		User stages DAT file does not have correct revision. */
#define PI_SOFTWARE_TIMEOUT__MINUS1082 -1082L
#define PI_SOFTWARE_TIMEOUT -1082L  /**< \ingroup err
		Timeout Error. Some lengthy operation did not finish within expected time. */
//
//  End of Dll Errors
//////////////////////////////////////////////////

//////////////////////////////////////////////////
//
// Controller Errors - Errors set by the controller or the GCS DLL
//
#define PI_CNTR_NO_ERROR__0 0L
#define PI_CNTR_NO_ERROR 0L  /**< \ingroup err
		No error */
#define PI_CNTR_PARAM_SYNTAX__1 1L
#define PI_CNTR_PARAM_SYNTAX 1L  /**< \ingroup err
		Parameter syntax error */
#define PI_CNTR_UNKNOWN_COMMAND__2 2L
#define PI_CNTR_UNKNOWN_COMMAND 2L  /**< \ingroup err
		Unknown command */
#define PI_CNTR_COMMAND_TOO_LONG__3 3L
#define PI_CNTR_COMMAND_TOO_LONG 3L  /**< \ingroup err
		Command length out of limits or command buffer overrun */
#define PI_CNTR_SCAN_ERROR__4 4L
#define PI_CNTR_SCAN_ERROR 4L  /**< \ingroup err
		Error while scanning */
#define PI_CNTR_MOVE_WITHOUT_REF_OR_NO_SERVO__5 5L
#define PI_CNTR_MOVE_WITHOUT_REF_OR_NO_SERVO 5L  /**< \ingroup err
		Unallowable move attempted on unreferenced axis, or move attempted with servo off */
#define PI_CNTR_INVALID_SGA_PARAM__6 6L
#define PI_CNTR_INVALID_SGA_PARAM 6L  /**< \ingroup err
		Parameter for SGA not valid */
#define PI_CNTR_POS_OUT_OF_LIMITS__7 7L
#define PI_CNTR_POS_OUT_OF_LIMITS 7L  /**< \ingroup err
		Position out of limits */
#define PI_CNTR_VEL_OUT_OF_LIMITS__8 8L
#define PI_CNTR_VEL_OUT_OF_LIMITS 8L  /**< \ingroup err
		Velocity out of limits */
#define PI_CNTR_SET_PIVOT_NOT_POSSIBLE__9 9L
#define PI_CNTR_SET_PIVOT_NOT_POSSIBLE 9L  /**< \ingroup err
		Attempt to set pivot point while U,V and W not all 0 */
#define PI_CNTR_STOP__10 10L
#define PI_CNTR_STOP 10L  /**< \ingroup err
		Controller was stopped by command */
#define PI_CNTR_SST_OR_SCAN_RANGE__11 11L
#define PI_CNTR_SST_OR_SCAN_RANGE 11L  /**< \ingroup err
		Parameter for SST or for one of the embedded scan algorithms out of range */
#define PI_CNTR_INVALID_SCAN_AXES__12 12L
#define PI_CNTR_INVALID_SCAN_AXES 12L  /**< \ingroup err
		Invalid axis combination for fast scan */
#define PI_CNTR_INVALID_NAV_PARAM__13 13L
#define PI_CNTR_INVALID_NAV_PARAM 13L  /**< \ingroup err
		Parameter for NAV out of range */
#define PI_CNTR_INVALID_ANALOG_INPUT__14 14L
#define PI_CNTR_INVALID_ANALOG_INPUT 14L  /**< \ingroup err
		Invalid analog channel */
#define PI_CNTR_INVALID_AXIS_IDENTIFIER__15 15L
#define PI_CNTR_INVALID_AXIS_IDENTIFIER 15L  /**< \ingroup err
		Invalid axis identifier */
#define PI_CNTR_INVALID_STAGE_NAME__16 16L
#define PI_CNTR_INVALID_STAGE_NAME 16L  /**< \ingroup err
		Invalid stage name */
#define PI_CNTR_PARAM_OUT_OF_RANGE__17 17L
#define PI_CNTR_PARAM_OUT_OF_RANGE 17L  /**< \ingroup err
		Parameter out of range */
#define PI_CNTR_INVALID_MACRO_NAME__18 18L
#define PI_CNTR_INVALID_MACRO_NAME 18L  /**< \ingroup err
		Invalid macro name */
#define PI_CNTR_MACRO_RECORD__19 19L
#define PI_CNTR_MACRO_RECORD 19L  /**< \ingroup err
		Error while recording macro */
#define PI_CNTR_MACRO_NOT_FOUND__20 20L
#define PI_CNTR_MACRO_NOT_FOUND 20L  /**< \ingroup err
		Macro not found */
#define PI_CNTR_AXIS_HAS_NO_BRAKE__21 21L
#define PI_CNTR_AXIS_HAS_NO_BRAKE 21L  /**< \ingroup err
		Axis has no brake */
#define PI_CNTR_DOUBLE_AXIS__22 22L
#define PI_CNTR_DOUBLE_AXIS 22L  /**< \ingroup err
		Axis identifier specified more than once */
#define PI_CNTR_ILLEGAL_AXIS__23 23L
#define PI_CNTR_ILLEGAL_AXIS 23L  /**< \ingroup err
		Illegal axis */
#define PI_CNTR_PARAM_NR__24 24L
#define PI_CNTR_PARAM_NR 24L  /**< \ingroup err
		Incorrect number of parameters */
#define PI_CNTR_INVALID_REAL_NR__25 25L
#define PI_CNTR_INVALID_REAL_NR 25L  /**< \ingroup err
		Invalid floating point number */
#define PI_CNTR_MISSING_PARAM__26 26L
#define PI_CNTR_MISSING_PARAM 26L  /**< \ingroup err
		Parameter missing */
#define PI_CNTR_SOFT_LIMIT_OUT_OF_RANGE__27 27L
#define PI_CNTR_SOFT_LIMIT_OUT_OF_RANGE 27L  /**< \ingroup err
		Soft limit out of range */
#define PI_CNTR_NO_MANUAL_PAD__28 28L
#define PI_CNTR_NO_MANUAL_PAD 28L  /**< \ingroup err
		No manual pad found */
#define PI_CNTR_NO_JUMP__29 29L
#define PI_CNTR_NO_JUMP 29L  /**< \ingroup err
		No more step-response values */
#define PI_CNTR_INVALID_JUMP__30 30L
#define PI_CNTR_INVALID_JUMP 30L  /**< \ingroup err
		No step-response values recorded */
#define PI_CNTR_AXIS_HAS_NO_REFERENCE__31 31L
#define PI_CNTR_AXIS_HAS_NO_REFERENCE 31L  /**< \ingroup err
		Axis has no reference sensor */
#define PI_CNTR_STAGE_HAS_NO_LIM_SWITCH__32 32L
#define PI_CNTR_STAGE_HAS_NO_LIM_SWITCH 32L  /**< \ingroup err
		Axis has no limit switch */
#define PI_CNTR_NO_RELAY_CARD__33 33L
#define PI_CNTR_NO_RELAY_CARD 33L  /**< \ingroup err
		No relay card installed */
#define PI_CNTR_CMD_NOT_ALLOWED_FOR_STAGE__34 34L
#define PI_CNTR_CMD_NOT_ALLOWED_FOR_STAGE 34L  /**< \ingroup err
		Command not allowed for selected stage(s) */
#define PI_CNTR_NO_DIGITAL_INPUT__35 35L
#define PI_CNTR_NO_DIGITAL_INPUT 35L  /**< \ingroup err
		No digital input installed */
#define PI_CNTR_NO_DIGITAL_OUTPUT__36 36L
#define PI_CNTR_NO_DIGITAL_OUTPUT 36L  /**< \ingroup err
		No digital output configured */
#define PI_CNTR_NO_MCM__37 37L
#define PI_CNTR_NO_MCM 37L  /**< \ingroup err
		No more MCM responses */
#define PI_CNTR_INVALID_MCM__38 38L
#define PI_CNTR_INVALID_MCM 38L  /**< \ingroup err
		No MCM values recorded */
#define PI_CNTR_INVALID_CNTR_NUMBER__39 39L
#define PI_CNTR_INVALID_CNTR_NUMBER 39L  /**< \ingroup err
		Controller number invalid */
#define PI_CNTR_NO_JOYSTICK_CONNECTED__40 40L
#define PI_CNTR_NO_JOYSTICK_CONNECTED 40L  /**< \ingroup err
		No joystick configured */
#define PI_CNTR_INVALID_EGE_AXIS__41 41L
#define PI_CNTR_INVALID_EGE_AXIS 41L  /**< \ingroup err
		Invalid axis for electronic gearing, axis can not be slave */
#define PI_CNTR_SLAVE_POSITION_OUT_OF_RANGE__42 42L
#define PI_CNTR_SLAVE_POSITION_OUT_OF_RANGE 42L  /**< \ingroup err
		Position of slave axis is out of range */
#define PI_CNTR_COMMAND_EGE_SLAVE__43 43L
#define PI_CNTR_COMMAND_EGE_SLAVE 43L  /**< \ingroup err
		Slave axis cannot be commanded directly when electronic gearing is enabled */
#define PI_CNTR_JOYSTICK_CALIBRATION_FAILED__44 44L
#define PI_CNTR_JOYSTICK_CALIBRATION_FAILED 44L  /**< \ingroup err
		Calibration of joystick failed */
#define PI_CNTR_REFERENCING_FAILED__45 45L
#define PI_CNTR_REFERENCING_FAILED 45L  /**< \ingroup err
		Referencing failed */
#define PI_CNTR_OPM_MISSING__46 46L
#define PI_CNTR_OPM_MISSING 46L  /**< \ingroup err
		OPM (Optical Power Meter) missing */
#define PI_CNTR_OPM_NOT_INITIALIZED__47 47L
#define PI_CNTR_OPM_NOT_INITIALIZED 47L  /**< \ingroup err
		OPM (Optical Power Meter) not initialized or cannot be initialized */
#define PI_CNTR_OPM_COM_ERROR__48 48L
#define PI_CNTR_OPM_COM_ERROR 48L  /**< \ingroup err
		OPM (Optical Power Meter) Communication Error */
#define PI_CNTR_MOVE_TO_LIMIT_SWITCH_FAILED__49 49L
#define PI_CNTR_MOVE_TO_LIMIT_SWITCH_FAILED 49L  /**< \ingroup err
		Move to limit switch failed */
#define PI_CNTR_REF_WITH_REF_DISABLED__50 50L
#define PI_CNTR_REF_WITH_REF_DISABLED 50L  /**< \ingroup err
		Attempt to reference axis with referencing disabled */
#define PI_CNTR_AXIS_UNDER_JOYSTICK_CONTROL__51 51L
#define PI_CNTR_AXIS_UNDER_JOYSTICK_CONTROL 51L  /**< \ingroup err
		Selected axis is controlled by joystick */
#define PI_CNTR_COMMUNICATION_ERROR__52 52L
#define PI_CNTR_COMMUNICATION_ERROR 52L  /**< \ingroup err
		Controller detected communication error */
#define PI_CNTR_DYNAMIC_MOVE_IN_PROCESS__53 53L
#define PI_CNTR_DYNAMIC_MOVE_IN_PROCESS 53L  /**< \ingroup err
		MOV! motion still in progress */
#define PI_CNTR_UNKNOWN_PARAMETER__54 54L
#define PI_CNTR_UNKNOWN_PARAMETER 54L  /**< \ingroup err
		Unknown parameter */
#define PI_CNTR_NO_REP_RECORDED__55 55L
#define PI_CNTR_NO_REP_RECORDED 55L  /**< \ingroup err
		No commands were recorded with REP */
#define PI_CNTR_INVALID_PASSWORD__56 56L
#define PI_CNTR_INVALID_PASSWORD 56L  /**< \ingroup err
		Password invalid */
#define PI_CNTR_INVALID_RECORDER_CHAN__57 57L
#define PI_CNTR_INVALID_RECORDER_CHAN 57L  /**< \ingroup err
		Data Record Table does not exist */
#define PI_CNTR_INVALID_RECORDER_SRC_OPT__58 58L
#define PI_CNTR_INVALID_RECORDER_SRC_OPT 58L  /**< \ingroup err
		Source does not exist; number too low or too high */
#define PI_CNTR_INVALID_RECORDER_SRC_CHAN__59 59L
#define PI_CNTR_INVALID_RECORDER_SRC_CHAN 59L  /**< \ingroup err
		Source Record Table number too low or too high */
#define PI_CNTR_PARAM_PROTECTION__60 60L
#define PI_CNTR_PARAM_PROTECTION 60L  /**< \ingroup err
		Protected Param: current Command Level (CCL) too low */
#define PI_CNTR_AUTOZERO_RUNNING__61 61L
#define PI_CNTR_AUTOZERO_RUNNING 61L  /**< \ingroup err
		Command execution not possible while Autozero is running */
#define PI_CNTR_NO_LINEAR_AXIS__62 62L
#define PI_CNTR_NO_LINEAR_AXIS 62L  /**< \ingroup err
		Autozero requires at least one linear axis */
#define PI_CNTR_INIT_RUNNING__63 63L
#define PI_CNTR_INIT_RUNNING 63L  /**< \ingroup err
		Initialization still in progress */
#define PI_CNTR_READ_ONLY_PARAMETER__64 64L
#define PI_CNTR_READ_ONLY_PARAMETER 64L  /**< \ingroup err
		Parameter is read-only */
#define PI_CNTR_PAM_NOT_FOUND__65 65L
#define PI_CNTR_PAM_NOT_FOUND 65L  /**< \ingroup err
		Parameter not found in non-volatile memory */
#define PI_CNTR_VOL_OUT_OF_LIMITS__66 66L
#define PI_CNTR_VOL_OUT_OF_LIMITS 66L  /**< \ingroup err
		Voltage out of limits */
#define PI_CNTR_WAVE_TOO_LARGE__67 67L
#define PI_CNTR_WAVE_TOO_LARGE 67L  /**< \ingroup err
		Not enough memory available for requested wave curve */
#define PI_CNTR_NOT_ENOUGH_DDL_MEMORY__68 68L
#define PI_CNTR_NOT_ENOUGH_DDL_MEMORY 68L  /**< \ingroup err
		Not enough memory available for DDL table; DDL can not be started */
#define PI_CNTR_DDL_TIME_DELAY_TOO_LARGE__69 69L
#define PI_CNTR_DDL_TIME_DELAY_TOO_LARGE 69L  /**< \ingroup err
		Time delay larger than DDL table; DDL can not be started */
#define PI_CNTR_DIFFERENT_ARRAY_LENGTH__70 70L
#define PI_CNTR_DIFFERENT_ARRAY_LENGTH 70L  /**< \ingroup err
		The requested arrays have different lengths; query them separately */
#define PI_CNTR_GEN_SINGLE_MODE_RESTART__71 71L
#define PI_CNTR_GEN_SINGLE_MODE_RESTART 71L  /**< \ingroup err
		Attempt to restart the generator while it is running in single step mode */
#define PI_CNTR_ANALOG_TARGET_ACTIVE__72 72L
#define PI_CNTR_ANALOG_TARGET_ACTIVE 72L  /**< \ingroup err
		Motion commands and wave generator activation are not allowed when analog target is active */
#define PI_CNTR_WAVE_GENERATOR_ACTIVE__73 73L
#define PI_CNTR_WAVE_GENERATOR_ACTIVE 73L  /**< \ingroup err
		Motion commands are not allowed when wave generator output is active; use WGO to disable generator output */
#define PI_CNTR_AUTOZERO_DISABLED__74 74L
#define PI_CNTR_AUTOZERO_DISABLED 74L  /**< \ingroup err
		No sensor channel or no piezo channel connected to selected axis (sensor and piezo matrix) */
#define PI_CNTR_NO_WAVE_SELECTED__75 75L
#define PI_CNTR_NO_WAVE_SELECTED 75L  /**< \ingroup err
		Generator started (WGO) without having selected a wave table (WSL). */
#define PI_CNTR_IF_BUFFER_OVERRUN__76 76L
#define PI_CNTR_IF_BUFFER_OVERRUN 76L  /**< \ingroup err
		Interface buffer did overrun and command couldn't be received correctly */
#define PI_CNTR_NOT_ENOUGH_RECORDED_DATA__77 77L
#define PI_CNTR_NOT_ENOUGH_RECORDED_DATA 77L  /**< \ingroup err
		Data Record Table does not hold enough recorded data */
#define PI_CNTR_TABLE_DEACTIVATED__78 78L
#define PI_CNTR_TABLE_DEACTIVATED 78L  /**< \ingroup err
		Data Record Table is not configured for recording */
#define PI_CNTR_OPENLOOP_VALUE_SET_WHEN_SERVO_ON__79 79L
#define PI_CNTR_OPENLOOP_VALUE_SET_WHEN_SERVO_ON 79L  /**< \ingroup err
		Open-loop commands (SVA, SVR) are not allowed when servo is on */
#define PI_CNTR_RAM_ERROR__80 80L
#define PI_CNTR_RAM_ERROR 80L  /**< \ingroup err
		Hardware error affecting RAM */
#define PI_CNTR_MACRO_UNKNOWN_COMMAND__81 81L
#define PI_CNTR_MACRO_UNKNOWN_COMMAND 81L  /**< \ingroup err
		Not macro command */
#define PI_CNTR_MACRO_PC_ERROR__82 82L
#define PI_CNTR_MACRO_PC_ERROR 82L  /**< \ingroup err
		Macro counter out of range */
#define PI_CNTR_JOYSTICK_ACTIVE__83 83L
#define PI_CNTR_JOYSTICK_ACTIVE 83L  /**< \ingroup err
		Joystick is active */
#define PI_CNTR_MOTOR_IS_OFF__84 84L
#define PI_CNTR_MOTOR_IS_OFF 84L  /**< \ingroup err
		Motor is off */
#define PI_CNTR_ONLY_IN_MACRO__85 85L
#define PI_CNTR_ONLY_IN_MACRO 85L  /**< \ingroup err
		Macro-only command */
#define PI_CNTR_JOYSTICK_UNKNOWN_AXIS__86 86L
#define PI_CNTR_JOYSTICK_UNKNOWN_AXIS 86L  /**< \ingroup err
		Invalid joystick axis */
#define PI_CNTR_JOYSTICK_UNKNOWN_ID__87 87L
#define PI_CNTR_JOYSTICK_UNKNOWN_ID 87L  /**< \ingroup err
		Joystick unknown */
#define PI_CNTR_REF_MODE_IS_ON__88 88L
#define PI_CNTR_REF_MODE_IS_ON 88L  /**< \ingroup err
		Move without referenced stage */
#define PI_CNTR_NOT_ALLOWED_IN_CURRENT_MOTION_MODE__89 89L
#define PI_CNTR_NOT_ALLOWED_IN_CURRENT_MOTION_MODE 89L  /**< \ingroup err
		Command not allowed in current motion mode */
#define PI_CNTR_DIO_AND_TRACING_NOT_POSSIBLE__90 90L
#define PI_CNTR_DIO_AND_TRACING_NOT_POSSIBLE 90L  /**< \ingroup err
		No tracing possible while digital IOs are used on this HW revision. Reconnect to switch operation mode. */
#define PI_CNTR_COLLISION__91 91L
#define PI_CNTR_COLLISION 91L  /**< \ingroup err
		Move not possible, would cause collision */
#define PI_CNTR_SLAVE_NOT_FAST_ENOUGH__92 92L
#define PI_CNTR_SLAVE_NOT_FAST_ENOUGH 92L  /**< \ingroup err
		Stage is not capable of following the master. Check the gear ratio(SRA). */
#define PI_CNTR_CMD_NOT_ALLOWED_WHILE_AXIS_IN_MOTION__93 93L
#define PI_CNTR_CMD_NOT_ALLOWED_WHILE_AXIS_IN_MOTION 93L  /**< \ingroup err
		This command is not allowed while the affected axis or its master is in motion. */
#define PI_CNTR_OPEN_LOOP_JOYSTICK_ENABLED__94 94L
#define PI_CNTR_OPEN_LOOP_JOYSTICK_ENABLED 94L  /**< \ingroup err
		Servo cannot be switched on when open-loop joystick control is enabled. */
#define PI_CNTR_INVALID_SERVO_STATE_FOR_PARAMETER__95 95L
#define PI_CNTR_INVALID_SERVO_STATE_FOR_PARAMETER 95L  /**< \ingroup err
		This parameter cannot be changed in current servo mode. */
#define PI_CNTR_UNKNOWN_STAGE_NAME__96 96L
#define PI_CNTR_UNKNOWN_STAGE_NAME 96L  /**< \ingroup err
		Unknown stage name */
#define PI_LABVIEW_ERROR__100 100L
#define PI_LABVIEW_ERROR 100L  /**< \ingroup err
		PI LabVIEW driver reports error. See source control for details. */
#define PI_CNTR_NO_AXIS__200 200L
#define PI_CNTR_NO_AXIS 200L  /**< \ingroup err
		No stage connected to axis */
#define PI_CNTR_NO_AXIS_PARAM_FILE__201 201L
#define PI_CNTR_NO_AXIS_PARAM_FILE 201L  /**< \ingroup err
		File with axis parameters not found */
#define PI_CNTR_INVALID_AXIS_PARAM_FILE__202 202L
#define PI_CNTR_INVALID_AXIS_PARAM_FILE 202L  /**< \ingroup err
		Invalid axis parameter file */
#define PI_CNTR_NO_AXIS_PARAM_BACKUP__203 203L
#define PI_CNTR_NO_AXIS_PARAM_BACKUP 203L  /**< \ingroup err
		Backup file with axis parameters not found */
#define PI_CNTR_RESERVED_204__204 204L
#define PI_CNTR_RESERVED_204 204L  /**< \ingroup err
		PI internal error code 204 */
#define PI_CNTR_SMO_WITH_SERVO_ON__205 205L
#define PI_CNTR_SMO_WITH_SERVO_ON 205L  /**< \ingroup err
		SMO with servo on */
#define PI_CNTR_UUDECODE_INCOMPLETE_HEADER__206 206L
#define PI_CNTR_UUDECODE_INCOMPLETE_HEADER 206L  /**< \ingroup err
		uudecode: incomplete header */
#define PI_CNTR_UUDECODE_NOTHING_TO_DECODE__207 207L
#define PI_CNTR_UUDECODE_NOTHING_TO_DECODE 207L  /**< \ingroup err
		uudecode: nothing to decode */
#define PI_CNTR_UUDECODE_ILLEGAL_FORMAT__208 208L
#define PI_CNTR_UUDECODE_ILLEGAL_FORMAT 208L  /**< \ingroup err
		uudecode: illegal UUE format */
#define PI_CNTR_CRC32_ERROR__209 209L
#define PI_CNTR_CRC32_ERROR 209L  /**< \ingroup err
		CRC32 error */
#define PI_CNTR_ILLEGAL_FILENAME__210 210L
#define PI_CNTR_ILLEGAL_FILENAME 210L  /**< \ingroup err
		Illegal file name (must be 8-0 format) */
#define PI_CNTR_FILE_NOT_FOUND__211 211L
#define PI_CNTR_FILE_NOT_FOUND 211L  /**< \ingroup err
		File not found on controller */
#define PI_CNTR_FILE_WRITE_ERROR__212 212L
#define PI_CNTR_FILE_WRITE_ERROR 212L  /**< \ingroup err
		Error writing file on controller */
#define PI_CNTR_DTR_HINDERS_VELOCITY_CHANGE__213 213L
#define PI_CNTR_DTR_HINDERS_VELOCITY_CHANGE 213L  /**< \ingroup err
		VEL command not allowed in DTR Command Mode */
#define PI_CNTR_POSITION_UNKNOWN__214 214L
#define PI_CNTR_POSITION_UNKNOWN 214L  /**< \ingroup err
		Position calculations failed */
#define PI_CNTR_CONN_POSSIBLY_BROKEN__215 215L
#define PI_CNTR_CONN_POSSIBLY_BROKEN 215L  /**< \ingroup err
		The connection between controller and stage may be broken */
#define PI_CNTR_ON_LIMIT_SWITCH__216 216L
#define PI_CNTR_ON_LIMIT_SWITCH 216L  /**< \ingroup err
		The connected stage has driven into a limit switch, some controllers need CLR to resume operation */
#define PI_CNTR_UNEXPECTED_STRUT_STOP__217 217L
#define PI_CNTR_UNEXPECTED_STRUT_STOP 217L  /**< \ingroup err
		Strut test command failed because of an unexpected strut stop */
#define PI_CNTR_POSITION_BASED_ON_ESTIMATION__218 218L
#define PI_CNTR_POSITION_BASED_ON_ESTIMATION 218L  /**< \ingroup err
		While MOV! is running position can only be estimated! */
#define PI_CNTR_POSITION_BASED_ON_INTERPOLATION__219 219L
#define PI_CNTR_POSITION_BASED_ON_INTERPOLATION 219L  /**< \ingroup err
		Position was calculated during MOV motion */
#define PI_CNTR_INVALID_HANDLE__230 230L
#define PI_CNTR_INVALID_HANDLE 230L  /**< \ingroup err
		Invalid handle */
#define PI_CNTR_NO_BIOS_FOUND__231 231L
#define PI_CNTR_NO_BIOS_FOUND 231L  /**< \ingroup err
		No bios found */
#define PI_CNTR_SAVE_SYS_CFG_FAILED__232 232L
#define PI_CNTR_SAVE_SYS_CFG_FAILED 232L  /**< \ingroup err
		Save system configuration failed */
#define PI_CNTR_LOAD_SYS_CFG_FAILED__233 233L
#define PI_CNTR_LOAD_SYS_CFG_FAILED 233L  /**< \ingroup err
		Load system configuration failed */
#define PI_CNTR_SEND_BUFFER_OVERFLOW__301 301L
#define PI_CNTR_SEND_BUFFER_OVERFLOW 301L  /**< \ingroup err
		Send buffer overflow */
#define PI_CNTR_VOLTAGE_OUT_OF_LIMITS__302 302L
#define PI_CNTR_VOLTAGE_OUT_OF_LIMITS 302L  /**< \ingroup err
		Voltage out of limits */
#define PI_CNTR_OPEN_LOOP_MOTION_SET_WHEN_SERVO_ON__303 303L
#define PI_CNTR_OPEN_LOOP_MOTION_SET_WHEN_SERVO_ON 303L  /**< \ingroup err
		Open-loop motion attempted when servo ON */
#define PI_CNTR_RECEIVING_BUFFER_OVERFLOW__304 304L
#define PI_CNTR_RECEIVING_BUFFER_OVERFLOW 304L  /**< \ingroup err
		Received command is too long */
#define PI_CNTR_EEPROM_ERROR__305 305L
#define PI_CNTR_EEPROM_ERROR 305L  /**< \ingroup err
		Error while reading/writing EEPROM */
#define PI_CNTR_I2C_ERROR__306 306L
#define PI_CNTR_I2C_ERROR 306L  /**< \ingroup err
		Error on I2C bus */
#define PI_CNTR_RECEIVING_TIMEOUT__307 307L
#define PI_CNTR_RECEIVING_TIMEOUT 307L  /**< \ingroup err
		Timeout while receiving command */
#define PI_CNTR_TIMEOUT__308 308L
#define PI_CNTR_TIMEOUT 308L  /**< \ingroup err
		A lengthy operation has not finished in the expected time */
#define PI_CNTR_MACRO_OUT_OF_SPACE__309 309L
#define PI_CNTR_MACRO_OUT_OF_SPACE 309L  /**< \ingroup err
		Insufficient space to store macro */
#define PI_CNTR_EUI_OLDVERSION_CFGDATA__310 310L
#define PI_CNTR_EUI_OLDVERSION_CFGDATA 310L  /**< \ingroup err
		Configuration data has old version number */
#define PI_CNTR_EUI_INVALID_CFGDATA__311 311L
#define PI_CNTR_EUI_INVALID_CFGDATA 311L  /**< \ingroup err
		Invalid configuration data */
#define PI_CNTR_HARDWARE_ERROR__333 333L
#define PI_CNTR_HARDWARE_ERROR 333L  /**< \ingroup err
		Internal hardware error */
#define PI_CNTR_WAV_INDEX_ERROR__400 400L
#define PI_CNTR_WAV_INDEX_ERROR 400L  /**< \ingroup err
		Wave generator index error */
#define PI_CNTR_WAV_NOT_DEFINED__401 401L
#define PI_CNTR_WAV_NOT_DEFINED 401L  /**< \ingroup err
		Wave table not defined */
#define PI_CNTR_WAV_TYPE_NOT_SUPPORTED__402 402L
#define PI_CNTR_WAV_TYPE_NOT_SUPPORTED 402L  /**< \ingroup err
		Wave type not supported */
#define PI_CNTR_WAV_LENGTH_EXCEEDS_LIMIT__403 403L
#define PI_CNTR_WAV_LENGTH_EXCEEDS_LIMIT 403L  /**< \ingroup err
		Wave length exceeds limit */
#define PI_CNTR_WAV_PARAMETER_NR__404 404L
#define PI_CNTR_WAV_PARAMETER_NR 404L  /**< \ingroup err
		Wave parameter number error */
#define PI_CNTR_WAV_PARAMETER_OUT_OF_LIMIT__405 405L
#define PI_CNTR_WAV_PARAMETER_OUT_OF_LIMIT 405L  /**< \ingroup err
		Wave parameter out of range */
#define PI_CNTR_WGO_BIT_NOT_SUPPORTED__406 406L
#define PI_CNTR_WGO_BIT_NOT_SUPPORTED 406L  /**< \ingroup err
		WGO command bit not supported */
#define PI_CNTR_EMERGENCY_STOP_BUTTON_ACTIVATED__500 500L
#define PI_CNTR_EMERGENCY_STOP_BUTTON_ACTIVATED 500L  /**< \ingroup err
		The \"red knob\" is still set and disables system */
#define PI_CNTR_EMERGENCY_STOP_BUTTON_WAS_ACTIVATED__501 501L
#define PI_CNTR_EMERGENCY_STOP_BUTTON_WAS_ACTIVATED 501L  /**< \ingroup err
		The \"red knob\" was activated and still disables system - reanimation required */
#define PI_CNTR_REDUNDANCY_LIMIT_EXCEEDED__502 502L
#define PI_CNTR_REDUNDANCY_LIMIT_EXCEEDED 502L  /**< \ingroup err
		Position consistency check failed */
#define PI_CNTR_COLLISION_SWITCH_ACTIVATED__503 503L
#define PI_CNTR_COLLISION_SWITCH_ACTIVATED 503L  /**< \ingroup err
		Hardware collision sensor(s) are activated */
#define PI_CNTR_FOLLOWING_ERROR__504 504L
#define PI_CNTR_FOLLOWING_ERROR 504L  /**< \ingroup err
		Strut following error occurred, e.g. caused by overload or encoder failure */
#define PI_CNTR_SENSOR_SIGNAL_INVALID__505 505L
#define PI_CNTR_SENSOR_SIGNAL_INVALID 505L  /**< \ingroup err
		One sensor signal is not valid */
#define PI_CNTR_SERVO_LOOP_UNSTABLE__506 506L
#define PI_CNTR_SERVO_LOOP_UNSTABLE 506L  /**< \ingroup err
		Servo loop was unstable due to wrong parameter setting and switched off to avoid damage. */
#define PI_CNTR_UNKNOWN_ERROR__555 555L
#define PI_CNTR_UNKNOWN_ERROR 555L  /**< \ingroup err
		BasMac: unknown controller error */
#define PI_CNTR_NOT_ENOUGH_MEMORY__601 601L
#define PI_CNTR_NOT_ENOUGH_MEMORY 601L  /**< \ingroup err
		Not enough memory */
#define PI_CNTR_HW_VOLTAGE_ERROR__602 602L
#define PI_CNTR_HW_VOLTAGE_ERROR 602L  /**< \ingroup err
		Hardware voltage error */
#define PI_CNTR_HW_TEMPERATURE_ERROR__603 603L
#define PI_CNTR_HW_TEMPERATURE_ERROR 603L  /**< \ingroup err
		Hardware temperature out of range */
#define PI_CNTR_POSITION_ERROR_TOO_HIGH__604 604L
#define PI_CNTR_POSITION_ERROR_TOO_HIGH 604L  /**< \ingroup err
		Position error of any axis in the system is too high */
#define PI_CNTR_INPUT_OUT_OF_RANGE__606 606L
#define PI_CNTR_INPUT_OUT_OF_RANGE 606L  /**< \ingroup err
		Maximum value of input signal has been exceeded */
#define PI_CNTR_TOO_MANY_NESTED_MACROS__1000 1000L
#define PI_CNTR_TOO_MANY_NESTED_MACROS 1000L  /**< \ingroup err
		Too many nested macros */
#define PI_CNTR_MACRO_ALREADY_DEFINED__1001 1001L
#define PI_CNTR_MACRO_ALREADY_DEFINED 1001L  /**< \ingroup err
		Macro already defined */
#define PI_CNTR_NO_MACRO_RECORDING__1002 1002L
#define PI_CNTR_NO_MACRO_RECORDING 1002L  /**< \ingroup err
		Macro recording not activated */
#define PI_CNTR_INVALID_MAC_PARAM__1003 1003L
#define PI_CNTR_INVALID_MAC_PARAM 1003L  /**< \ingroup err
		Invalid parameter for MAC */
#define PI_CNTR_MACRO_DELETE_ERROR__1004 1004L
#define PI_CNTR_MACRO_DELETE_ERROR 1004L  /**< \ingroup err
		Deleting macro failed */
#define PI_CNTR_CONTROLLER_BUSY__1005 1005L
#define PI_CNTR_CONTROLLER_BUSY 1005L  /**< \ingroup err
		Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm) */
#define PI_CNTR_INVALID_IDENTIFIER__1006 1006L
#define PI_CNTR_INVALID_IDENTIFIER 1006L  /**< \ingroup err
		Invalid identifier (invalid special characters, ...) */
#define PI_CNTR_UNKNOWN_VARIABLE_OR_ARGUMENT__1007 1007L
#define PI_CNTR_UNKNOWN_VARIABLE_OR_ARGUMENT 1007L  /**< \ingroup err
		Variable or argument not defined */
#define PI_CNTR_RUNNING_MACRO__1008 1008L
#define PI_CNTR_RUNNING_MACRO 1008L  /**< \ingroup err
		Controller is (already) running a macro */
#define PI_CNTR_MACRO_INVALID_OPERATOR__1009 1009L
#define PI_CNTR_MACRO_INVALID_OPERATOR 1009L  /**< \ingroup err
		Invalid or missing operator for condition. Check necessary spaces around operator. */
#define PI_CNTR_MACRO_NO_ANSWER__1010 1010L
#define PI_CNTR_MACRO_NO_ANSWER 1010L  /**< \ingroup err
		No answer was received while executing WAC/MEX/JRC/... */
#define PI_CMD_NOT_VALID_IN_MACRO_MODE__1011 1011L
#define PI_CMD_NOT_VALID_IN_MACRO_MODE 1011L  /**< \ingroup err
		Command not valid during macro execution */
#define PI_CNTR_MOTION_ERROR__1024 1024L
#define PI_CNTR_MOTION_ERROR 1024L  /**< \ingroup err
		Motion error: position error too large, servo is switched off automatically */
#define PI_CNTR_EXT_PROFILE_UNALLOWED_CMD__1063 1063L
#define PI_CNTR_EXT_PROFILE_UNALLOWED_CMD 1063L  /**< \ingroup err
		User Profile Mode: Command is not allowed, check for required preparatory commands */
#define PI_CNTR_EXT_PROFILE_EXPECTING_MOTION_ERROR__1064 1064L
#define PI_CNTR_EXT_PROFILE_EXPECTING_MOTION_ERROR 1064L  /**< \ingroup err
		User Profile Mode: First target position in User Profile is too far from current position */
#define PI_CNTR_PROFILE_ACTIVE__1065 1065L
#define PI_CNTR_PROFILE_ACTIVE 1065L  /**< \ingroup err
		Controller is (already) in User Profile Mode */
#define PI_CNTR_PROFILE_INDEX_OUT_OF_RANGE__1066 1066L
#define PI_CNTR_PROFILE_INDEX_OUT_OF_RANGE 1066L  /**< \ingroup err
		User Profile Mode: Block or Data Set index out of allowed range */
#define PI_CNTR_PROFILE_OUT_OF_MEMORY__1071 1071L
#define PI_CNTR_PROFILE_OUT_OF_MEMORY 1071L  /**< \ingroup err
		User Profile Mode: Out of memory */
#define PI_CNTR_PROFILE_WRONG_CLUSTER__1072 1072L
#define PI_CNTR_PROFILE_WRONG_CLUSTER 1072L  /**< \ingroup err
		User Profile Mode: Cluster is not assigned to this axis */
#define PI_CNTR_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER__1073 1073L
#define PI_CNTR_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER 1073L  /**< \ingroup err
		Unknown cluster identifier */
#define PI_CNTR_ALREADY_HAS_SERIAL_NUMBER__2000 2000L
#define PI_CNTR_ALREADY_HAS_SERIAL_NUMBER 2000L  /**< \ingroup err
		Controller already has a serial number */
#define PI_CNTR_SECTOR_ERASE_FAILED__4000 4000L
#define PI_CNTR_SECTOR_ERASE_FAILED 4000L  /**< \ingroup err
		Sector erase failed */
#define PI_CNTR_FLASH_PROGRAM_FAILED__4001 4001L
#define PI_CNTR_FLASH_PROGRAM_FAILED 4001L  /**< \ingroup err
		Flash program failed */
#define PI_CNTR_FLASH_READ_FAILED__4002 4002L
#define PI_CNTR_FLASH_READ_FAILED 4002L  /**< \ingroup err
		Flash read failed */
#define PI_CNTR_HW_MATCHCODE_ERROR__4003 4003L
#define PI_CNTR_HW_MATCHCODE_ERROR 4003L  /**< \ingroup err
		HW match code missing/invalid */
#define PI_CNTR_FW_MATCHCODE_ERROR__4004 4004L
#define PI_CNTR_FW_MATCHCODE_ERROR 4004L  /**< \ingroup err
		FW match code missing/invalid */
#define PI_CNTR_HW_VERSION_ERROR__4005 4005L
#define PI_CNTR_HW_VERSION_ERROR 4005L  /**< \ingroup err
		HW version missing/invalid */
#define PI_CNTR_FW_VERSION_ERROR__4006 4006L
#define PI_CNTR_FW_VERSION_ERROR 4006L  /**< \ingroup err
		FW version missing/invalid */
#define PI_CNTR_FW_UPDATE_ERROR__4007 4007L
#define PI_CNTR_FW_UPDATE_ERROR 4007L  /**< \ingroup err
		FW update failed */
#define PI_CNTR_FW_CRC_PAR_ERROR__4008 4008L
#define PI_CNTR_FW_CRC_PAR_ERROR 4008L  /**< \ingroup err
		FW Parameter CRC wrong */
#define PI_CNTR_FW_CRC_FW_ERROR__4009 4009L
#define PI_CNTR_FW_CRC_FW_ERROR 4009L  /**< \ingroup err
		FW CRC wrong */
#define PI_CNTR_INVALID_PCC_SCAN_DATA__5000 5000L
#define PI_CNTR_INVALID_PCC_SCAN_DATA 5000L  /**< \ingroup err
		PicoCompensation scan data is not valid */
#define PI_CNTR_PCC_SCAN_RUNNING__5001 5001L
#define PI_CNTR_PCC_SCAN_RUNNING 5001L  /**< \ingroup err
		PicoCompensation is running, some actions can not be executed during scanning/recording */
#define PI_CNTR_INVALID_PCC_AXIS__5002 5002L
#define PI_CNTR_INVALID_PCC_AXIS 5002L  /**< \ingroup err
		Given axis can not be defined as PPC axis */
#define PI_CNTR_PCC_SCAN_OUT_OF_RANGE__5003 5003L
#define PI_CNTR_PCC_SCAN_OUT_OF_RANGE 5003L  /**< \ingroup err
		Defined scan area is larger than the travel range */
#define PI_CNTR_PCC_TYPE_NOT_EXISTING__5004 5004L
#define PI_CNTR_PCC_TYPE_NOT_EXISTING 5004L  /**< \ingroup err
		Given PicoCompensation type is not defined */
#define PI_CNTR_PCC_PAM_ERROR__5005 5005L
#define PI_CNTR_PCC_PAM_ERROR 5005L  /**< \ingroup err
		PicoCompensation parameter error */
#define PI_CNTR_PCC_TABLE_ARRAY_TOO_LARGE__5006 5006L
#define PI_CNTR_PCC_TABLE_ARRAY_TOO_LARGE 5006L  /**< \ingroup err
		PicoCompensation table is larger than maximum table length */
#define PI_CNTR_NEXLINE_ERROR__5100 5100L
#define PI_CNTR_NEXLINE_ERROR 5100L  /**< \ingroup err
		Common error in Nexline firmware module */
#define PI_CNTR_CHANNEL_ALREADY_USED__5101 5101L
#define PI_CNTR_CHANNEL_ALREADY_USED 5101L  /**< \ingroup err
		Output channel for Nexline can not be redefined for other usage */
#define PI_CNTR_NEXLINE_TABLE_TOO_SMALL__5102 5102L
#define PI_CNTR_NEXLINE_TABLE_TOO_SMALL 5102L  /**< \ingroup err
		Memory for Nexline signals is too small */
#define PI_CNTR_RNP_WITH_SERVO_ON__5103 5103L
#define PI_CNTR_RNP_WITH_SERVO_ON 5103L  /**< \ingroup err
		RNP can not be executed if axis is in closed loop */
#define PI_CNTR_RNP_NEEDED__5104 5104L
#define PI_CNTR_RNP_NEEDED 5104L  /**< \ingroup err
		relax procedure (RNP) needed */
#define PI_CNTR_AXIS_NOT_CONFIGURED__5200 5200L
#define PI_CNTR_AXIS_NOT_CONFIGURED 5200L  /**< \ingroup err
		Axis must be configured for this action */
//
//  End of Controller Errors
//////////////////////////////////////////////////

//////////////////////////////////////////////////
//
// Interface Errors - Interface errors occuring while communicating with the controller
//
#define COM_NO_ERROR__0 0L
#define COM_NO_ERROR 0L  /**< \ingroup err
		No error occurred during function call */
#define COM_ERROR__MINUS1 -1L
#define COM_ERROR -1L  /**< \ingroup err
		Error during com operation (could not be specified) */
#define SEND_ERROR__MINUS2 -2L
#define SEND_ERROR -2L  /**< \ingroup err
		Error while sending data */
#define REC_ERROR__MINUS3 -3L
#define REC_ERROR -3L  /**< \ingroup err
		Error while receiving data */
#define NOT_CONNECTED_ERROR__MINUS4 -4L
#define NOT_CONNECTED_ERROR -4L  /**< \ingroup err
		Not connected (no port with given ID open) */
#define COM_BUFFER_OVERFLOW__MINUS5 -5L
#define COM_BUFFER_OVERFLOW -5L  /**< \ingroup err
		Buffer overflow */
#define CONNECTION_FAILED__MINUS6 -6L
#define CONNECTION_FAILED -6L  /**< \ingroup err
		Error while opening port */
#define COM_TIMEOUT__MINUS7 -7L
#define COM_TIMEOUT -7L  /**< \ingroup err
		Timeout error */
#define COM_MULTILINE_RESPONSE__MINUS8 -8L
#define COM_MULTILINE_RESPONSE -8L  /**< \ingroup err
		There are more lines waiting in buffer */
#define COM_INVALID_ID__MINUS9 -9L
#define COM_INVALID_ID -9L  /**< \ingroup err
		There is no interface or DLL handle with the given ID */
#define COM_NOTIFY_EVENT_ERROR__MINUS10 -10L
#define COM_NOTIFY_EVENT_ERROR -10L  /**< \ingroup err
		Event/message for notification could not be opened */
#define COM_NOT_IMPLEMENTED__MINUS11 -11L
#define COM_NOT_IMPLEMENTED -11L  /**< \ingroup err
		Function not supported by this interface type */
#define COM_ECHO_ERROR__MINUS12 -12L
#define COM_ECHO_ERROR -12L  /**< \ingroup err
		Error while sending \"echoed\" data */
#define COM_GPIB_EDVR__MINUS13 -13L
#define COM_GPIB_EDVR -13L  /**< \ingroup err
		IEEE488: System error */
#define COM_GPIB_ECIC__MINUS14 -14L
#define COM_GPIB_ECIC -14L  /**< \ingroup err
		IEEE488: Function requires GPIB board to be CIC */
#define COM_GPIB_ENOL__MINUS15 -15L
#define COM_GPIB_ENOL -15L  /**< \ingroup err
		IEEE488: Write function detected no listeners */
#define COM_GPIB_EADR__MINUS16 -16L
#define COM_GPIB_EADR -16L  /**< \ingroup err
		IEEE488: Interface board not addressed correctly */
#define COM_GPIB_EARG__MINUS17 -17L
#define COM_GPIB_EARG -17L  /**< \ingroup err
		IEEE488: Invalid argument to function call */
#define COM_GPIB_ESAC__MINUS18 -18L
#define COM_GPIB_ESAC -18L  /**< \ingroup err
		IEEE488: Function requires GPIB board to be SAC */
#define COM_GPIB_EABO__MINUS19 -19L
#define COM_GPIB_EABO -19L  /**< \ingroup err
		IEEE488: I/O operation aborted */
#define COM_GPIB_ENEB__MINUS20 -20L
#define COM_GPIB_ENEB -20L  /**< \ingroup err
		IEEE488: Interface board not found */
#define COM_GPIB_EDMA__MINUS21 -21L
#define COM_GPIB_EDMA -21L  /**< \ingroup err
		IEEE488: Error performing DMA */
#define COM_GPIB_EOIP__MINUS22 -22L
#define COM_GPIB_EOIP -22L  /**< \ingroup err
		IEEE488: I/O operation started before previous operation completed */
#define COM_GPIB_ECAP__MINUS23 -23L
#define COM_GPIB_ECAP -23L  /**< \ingroup err
		IEEE488: No capability for intended operation */
#define COM_GPIB_EFSO__MINUS24 -24L
#define COM_GPIB_EFSO -24L  /**< \ingroup err
		IEEE488: File system operation error */
#define COM_GPIB_EBUS__MINUS25 -25L
#define COM_GPIB_EBUS -25L  /**< \ingroup err
		IEEE488: Command error during device call */
#define COM_GPIB_ESTB__MINUS26 -26L
#define COM_GPIB_ESTB -26L  /**< \ingroup err
		IEEE488: Serial poll-status byte lost */
#define COM_GPIB_ESRQ__MINUS27 -27L
#define COM_GPIB_ESRQ -27L  /**< \ingroup err
		IEEE488: SRQ remains asserted */
#define COM_GPIB_ETAB__MINUS28 -28L
#define COM_GPIB_ETAB -28L  /**< \ingroup err
		IEEE488: Return buffer full */
#define COM_GPIB_ELCK__MINUS29 -29L
#define COM_GPIB_ELCK -29L  /**< \ingroup err
		IEEE488: Address or board locked */
#define COM_RS_INVALID_DATA_BITS__MINUS30 -30L
#define COM_RS_INVALID_DATA_BITS -30L  /**< \ingroup err
		RS-232: 5 data bits with 2 stop bits is an invalid combination, as is 6, 7, or 8 data bits with 1.5 stop bits */
#define COM_ERROR_RS_SETTINGS__MINUS31 -31L
#define COM_ERROR_RS_SETTINGS -31L  /**< \ingroup err
		RS-232: Error configuring the COM port */
#define COM_INTERNAL_RESOURCES_ERROR__MINUS32 -32L
#define COM_INTERNAL_RESOURCES_ERROR -32L  /**< \ingroup err
		Error dealing with internal system resources (events, threads, ...) */
#define COM_DLL_FUNC_ERROR__MINUS33 -33L
#define COM_DLL_FUNC_ERROR -33L  /**< \ingroup err
		A DLL or one of the required functions could not be loaded */
#define COM_FTDIUSB_INVALID_HANDLE__MINUS34 -34L
#define COM_FTDIUSB_INVALID_HANDLE -34L  /**< \ingroup err
		FTDIUSB: invalid handle */
#define COM_FTDIUSB_DEVICE_NOT_FOUND__MINUS35 -35L
#define COM_FTDIUSB_DEVICE_NOT_FOUND -35L  /**< \ingroup err
		FTDIUSB: device not found */
#define COM_FTDIUSB_DEVICE_NOT_OPENED__MINUS36 -36L
#define COM_FTDIUSB_DEVICE_NOT_OPENED -36L  /**< \ingroup err
		FTDIUSB: device not opened */
#define COM_FTDIUSB_IO_ERROR__MINUS37 -37L
#define COM_FTDIUSB_IO_ERROR -37L  /**< \ingroup err
		FTDIUSB: IO error */
#define COM_FTDIUSB_INSUFFICIENT_RESOURCES__MINUS38 -38L
#define COM_FTDIUSB_INSUFFICIENT_RESOURCES -38L  /**< \ingroup err
		FTDIUSB: insufficient resources */
#define COM_FTDIUSB_INVALID_PARAMETER__MINUS39 -39L
#define COM_FTDIUSB_INVALID_PARAMETER -39L  /**< \ingroup err
		FTDIUSB: invalid parameter */
#define COM_FTDIUSB_INVALID_BAUD_RATE__MINUS40 -40L
#define COM_FTDIUSB_INVALID_BAUD_RATE -40L  /**< \ingroup err
		FTDIUSB: invalid baud rate */
#define COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_ERASE__MINUS41 -41L
#define COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_ERASE -41L  /**< \ingroup err
		FTDIUSB: device not opened for erase */
#define COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_WRITE__MINUS42 -42L
#define COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_WRITE -42L  /**< \ingroup err
		FTDIUSB: device not opened for write */
#define COM_FTDIUSB_FAILED_TO_WRITE_DEVICE__MINUS43 -43L
#define COM_FTDIUSB_FAILED_TO_WRITE_DEVICE -43L  /**< \ingroup err
		FTDIUSB: failed to write device */
#define COM_FTDIUSB_EEPROM_READ_FAILED__MINUS44 -44L
#define COM_FTDIUSB_EEPROM_READ_FAILED -44L  /**< \ingroup err
		FTDIUSB: EEPROM read failed */
#define COM_FTDIUSB_EEPROM_WRITE_FAILED__MINUS45 -45L
#define COM_FTDIUSB_EEPROM_WRITE_FAILED -45L  /**< \ingroup err
		FTDIUSB: EEPROM write failed */
#define COM_FTDIUSB_EEPROM_ERASE_FAILED__MINUS46 -46L
#define COM_FTDIUSB_EEPROM_ERASE_FAILED -46L  /**< \ingroup err
		FTDIUSB: EEPROM erase failed */
#define COM_FTDIUSB_EEPROM_NOT_PRESENT__MINUS47 -47L
#define COM_FTDIUSB_EEPROM_NOT_PRESENT -47L  /**< \ingroup err
		FTDIUSB: EEPROM not present */
#define COM_FTDIUSB_EEPROM_NOT_PROGRAMMED__MINUS48 -48L
#define COM_FTDIUSB_EEPROM_NOT_PROGRAMMED -48L  /**< \ingroup err
		FTDIUSB: EEPROM not programmed */
#define COM_FTDIUSB_INVALID_ARGS__MINUS49 -49L
#define COM_FTDIUSB_INVALID_ARGS -49L  /**< \ingroup err
		FTDIUSB: invalid arguments */
#define COM_FTDIUSB_NOT_SUPPORTED__MINUS50 -50L
#define COM_FTDIUSB_NOT_SUPPORTED -50L  /**< \ingroup err
		FTDIUSB: not supported */
#define COM_FTDIUSB_OTHER_ERROR__MINUS51 -51L
#define COM_FTDIUSB_OTHER_ERROR -51L  /**< \ingroup err
		FTDIUSB: other error */
#define COM_PORT_ALREADY_OPEN__MINUS52 -52L
#define COM_PORT_ALREADY_OPEN -52L  /**< \ingroup err
		Error while opening the COM port: was already open */
#define COM_PORT_CHECKSUM_ERROR__MINUS53 -53L
#define COM_PORT_CHECKSUM_ERROR -53L  /**< \ingroup err
		Checksum error in received data from COM port */
#define COM_SOCKET_NOT_READY__MINUS54 -54L
#define COM_SOCKET_NOT_READY -54L  /**< \ingroup err
		Socket not ready, you should call the function again */
#define COM_SOCKET_PORT_IN_USE__MINUS55 -55L
#define COM_SOCKET_PORT_IN_USE -55L  /**< \ingroup err
		Port is used by another socket */
#define COM_SOCKET_NOT_CONNECTED__MINUS56 -56L
#define COM_SOCKET_NOT_CONNECTED -56L  /**< \ingroup err
		Socket not connected (or not valid) */
#define COM_SOCKET_TERMINATED__MINUS57 -57L
#define COM_SOCKET_TERMINATED -57L  /**< \ingroup err
		Connection terminated (by peer) */
#define COM_SOCKET_NO_RESPONSE__MINUS58 -58L
#define COM_SOCKET_NO_RESPONSE -58L  /**< \ingroup err
		Can't connect to peer */
#define COM_SOCKET_INTERRUPTED__MINUS59 -59L
#define COM_SOCKET_INTERRUPTED -59L  /**< \ingroup err
		Operation was interrupted by a nonblocked signal */
#define COM_PCI_INVALID_ID__MINUS60 -60L
#define COM_PCI_INVALID_ID -60L  /**< \ingroup err
		No Device with this ID is present */
#define COM_PCI_ACCESS_DENIED__MINUS61 -61L
#define COM_PCI_ACCESS_DENIED -61L  /**< \ingroup err
		Driver could not be opened (on Vista: run as administrator!) */
//
//  End of Interface Errors
//////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//  obsolet, provided for backward compatibility
#define PI_CNTR_MOVE_WITHOUT_INI				5			// name "INI" is misleading - we mean "reference"
#define PI_CNTR_INVALID_AXIS					23			// misleading name: INVALID means something different than "illegal"
//
/////////////////////////////////////////////////////////////////////////////

#endif // __PI_CONTROLLER_ERROS_H__

