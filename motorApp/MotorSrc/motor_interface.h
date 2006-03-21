#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/**\mainpage Proposed EPICS Motor Axis API

For a long time EPICS motor support has either been via the complex,
and somewhat ill-defined motor record or via primitive record
support. The two approaches have been mutually incompatible and led to
much anguish. People who adopt the motor record get a standard
interface across all controllers, but it is inextensable. Those who
write their own driver using primitive records cannot port their
databases to other motor controllers. Neither situation is very satisfactory.

After looking at this for some time, the reason becames clear - the
the interface between the generic software for the motor record and
the underlying specific software for a given motor controller is very
complex, poorly defined and very specific to the motor record. Hence,
this is an attempt to define a clean, extensible API for motor
controller drivers to support, and we will be providing an interface
between this API and both motor records and other primitive
records. We intend to provide drivers for OMS, Delta Tau and some
Newport systems and hope others will adopt this approach.

To do this we have had to analyse the requirements needed to support
the motor record, and the following is the results.

\section control Analysis of commands that control hardware behaviour

All of these commands go through an interface with three routines
implementing a transaction style interface: start_trans, build_trans
and end_trans. The implication is that any build_trans commands issued
between a start_trans and an end_trans happen atomically. Build_trans
takes three parameters, the transaction type (an enumerated value), a
pointer to an array of double parameters, and a pointer to the motor record
itself.

\subsection sec1 build_trans commands that are only executed alone.

All of the commands in this section are only ever called alone,
bracketed by a start_trans and an end_trans call. Hence, they could be
re-implemented in a non-transaction based interface.

\subsubsection ss1 Commands that could be removed
\li \b PRIMITIVE Called line 316 motordevCom.cc. This command is used
just to pass initialisation strings to the controller from the motor
OUT link. It could be done as part of controller
initialisation. However, it has been retained in this interface to
implement the PREM and the POST fields of the motor record.

\subsubsection ss2 Commands for which the return status is consulted.

These return an error if there is a problem with the demand value. On return, I think the demand value should be
replaced by the permitted value (but I don't see this being used).

\li \b SET_DGAIN Called line 341 motordevCom.cc. Called line 2571 motorRecord.cc
\li \b SET_HIGH_LIMIT Not called in motordevCom.cc. Called line 2346, 2387, 3466 motorRecord.cc
\li \b SET_IGAIN Called line 334 motordevCom.cc. Called line 2571 motorRecord.cc
\li \b SET_LOW_LIMIT Not called in motordevCom.cc. Called line 2346, 2387, 3501 motorRecord.cc
\li \b SET_PGAIN Called line 326 motordevCom.cc. Called line 2571 motorRecord.cc

\subsubsection ss3 Other build_trans commands called singly

The return status is never checked when these transactions are built.

\li \b DISABL_TORQUE Not called in motordevCom.cc. Called line 2593 motorRecord.cc
\li \b ENABLE_TORQUE Not called in motordevCom.cc. Called line 2591 motorRecord.cc. These two \b _TORQUE commands are crying out to be implemented as a single command because the parameter is currently ignored. Note that they don't really enable or disable torque, they actually open or close the PID loop.

\li \b GET_INFO Called line 348 motordevCom.cc. Called line 1117, 1149, 1645, 1908, 2111, 3270 motorRecord.cc. Note that this ultimately leads to the motor status information being updated, and then this is accessed by a call to motor_update_values in motordevCom.
\li \b LOAD_POS Called line 309 motordevCom.cc. Called line 3267 motorRecord.cc. Note that the position loaded is not a demand position - this is just a command to set the current position to a given absolute value.
\li \b SET_ENC_RATIO Called line 300 motordevCom.cc. Called line 1638 motorRecord.cc.
\li \b STOP_AXIS Not called in motordevCom.cc. Called line 1094, 1533, 1566, 1701, 1760, 1798 motorRecord.cc. It is not clear exactly what acceleration should be used in stopping - but I assume that the one that initiated the last move is probably OK.

\subsection sec2 build_trans commands that are executed in groups

These commands are always executed in as part of a group of
transactions intended to initiate motion of some kind. The consist of:
\b SET_VEL_BASE, \b SET_VELOCITY, \b SET_ACCEL, \b JOG_VELOCITY, \b
HOME_FOR, \b HOME_REV, \b MOVE_REL, \b MOVE_ABS, \b GO and \b JOG. They implement three different kinds of move:

\li \b Home - Starts a home search.
\li \b Move - Starts a relative or absolute move of a fixed distance.
\li \b Jog - Starts moving at a given velocity and keeps moving until stopped.

The latter is an unfortunate name since I think the word Jog has
multiple meanings in motion control context, but here we are dictated
by the OMS definition where a jog is a constant velocity move. There
is also two JOG commands, JOG and JOG_VELOCITY, the only difference in
the OMS driver being that JOG clears the controller DONE flag for this
axis before starting.

\subsubsection ss4 Homing sequence.

\li \b SET_VEL_BASE
\li \b SET_VELOCITY 
\li \b HOME_FOR or \b HOME_REV
\li \b GO.

These five transactions are called as a set in lines 659-662 and also
in lines 1719-1726 in motorRecord.cc. Only one of HOME_FOR or HOME_REV
is called, dependent on the setting of the record homf or homr
fields. (What happens if you don't find the home switch - can't the
controller work out which way to attempt to home?). Note that no 
acceleration is specified.

\subsubsection ss5 Move sequence.
\li ( \b SET_VEL_BASE )
\li \b SET_VELOCITY
\li \b SET_ACCEL
\li \b MOVE_REL or \b MOVE_ABS
\li \b GO

These six transactions are called as a set three times in
motorRecord.cc (between lines 704-746, lines 778-798 and lines
2042-2102). Only one of MOVE_REL and MOVE_ABS is called depending on
whether it is a relative move or an absolute move, and the parameter
passed in this transaction is the actual move distance. SET_VEL_BASE
is in parentheses because it is not called every time - but the reason
for this is pretty unclear to me (I suspect it could be).

\li In the first invocation (lines  704-746), SET_VEL_BASE is only included if it is a MIP_JOG_STOP.
\li It is \b not called in the second invocation (lines 778-798).
\li It is  included in the third invocation (lines 2042-2102).

\subsubsection ss6 Jog sequences

There are two jog sequences:

\li \b SET_ACCEL followed by \b JOG. Called between lines 1782-1785 motorRecord.cc.
\li \b SET_ACCEL followed by \b JOG_VELOCITY. Called between lines 2622-2625 motorRecord.cc.

I don't really understand the real difference between these two set -
as far as I can see the only difference in the OMS driver is that the
JOG transaction executes a CA to clear the axis done bit before
running, whilst JOG_VELOCITY doesn't. I suspect that the two could be
replaced by a single command in a new interface. I suspect one allows
you to change the velocity whilst the motor is moving and the other
doesn't, but I'm not sure.

\subsubsection ss7 Notes on the implementation of motion in the API.

In the API I have implemented these three collections of commands as
three separate routines, not as multiple single parameter routines
with a start/end buffering command. I have done this for simplicity to
highlight the minimum functionality that needs to be provided to
support the motor record, but this may be the wrong
decision. Buffering could get complicated if multiple people wrote it
in different ways. Note that if we did go down the buffering route, I
see no need for either a GO command or a JOG and JOG_VELOCITY
command. The former is implicit in the end_trans, and the latter
should be amalgamated into a single command.

A third, compromise, route, which might be the best solution, is to
implement primitive commands for the move parameters (acceleration,
velocity etc), but define them such that they have an additional
parameter which says (for example):

\li If zero, they apply to the next motion command for this AXIS_HDL handle.
\li If non-zero they apply as default parameters to be used for any motion on this axis.

The latter could be used for the primitive record approach, and the
former for the motor record.

Finally, in the PREM and the POST fields aren't seen in this
implementation - I assume they are handled via device support calling
motorAxisPrimitive.

\section Status Analysis of commands that provide status information

As mentioned above, motor status is handled by issuing GET_INFO commands on a regular basis, and then calling pdset->update_values (which is actually motor_update_values in motordevCom.cc). This routine updates the:

\li Motor step position (motor record rmp field)
\li Encoder position (motor record rep field), if available.
\li Motor velocity (motor record vel field) and
\li Status bits (motor record msta field). The msta field is a bit field of motor status bits, some reflecting the status of the current move, and others representing motor capabilities.

There is also a certain amount of other updates done in device support, but I suspect these could possibly all be incorporated into a generic device support module if the driver API is sufficiently complete.

\section Other Other information flow.

There may be other paths of hidden information flow between the records and the controllers, but I haven't uncovered them.

\section Analysis How could this be documented as an acceptable interface.

There are a number of problems with the motor record as it stands.

\li There is no defined (i.e. written down) interface anywhere between
the motor record and driver code that implements the specific control
of a particular motor controller.
\li This is further complicated by there being multiple levels of
shared code - the motor record itself, motordevCom and
motordrvCom. These each have multiple interfaces between themselves
and shared code.
\li The arrangement of shared code makes it difficult to extend the
motor controller interface in a generic way to support records other
than the motor record.

As a result of this, I feel the first thing to do is to define an API
that can be implemented by a person developing a motor controller, with
the knowledge that if they provide this API, then they will have motor
record support. We can then extend the API to provide support for
other records, or the supporter of a particular motor controller can
do it on his own. We will then write device support that links the API
and the motor record, using the API as, effectively, a driver support
entry table.

*/

/** Forward declaration of AXIS_HDL, which is a pointer to an internal driver-dependent handle */
typedef struct motorAxisHandle * AXIS_HDL;

#define MOTOR_AXIS_OK (0)
#define MOTOR_AXIS_ERROR (-1)

#ifndef vxWorks
typedef unsigned int uint32_t;
typedef int int32_t;
#endif

/**\struct motorAxisParam_t

    This is an enumeration of parameters that affect the behaviour of the
    controller for this axis. The are used by the functions motorAxisSetDouble,
    motorAxisSetInt and other associated functions.
*/

typedef enum
{
    motorAxisPosition,         /**< (double) Sets the current motor actual position to a given value in motor units. 
                                 Returns the current motor actual position in motor units */
    motorAxisEncoderRatio,     /**< (double) Number of encoder counts in one motor count (encoder counts/motor counts) */
    motorAxisLowLimit,         /**< (double) Low soft limit in motor units */
    motorAxisHighLimit,        /**< (double) High soft limit in motor units */
    motorAxisPGain,            /**< (double) The proportional gain term for PID control in controller dependent units */
    motorAxisIGain,            /**< (double) The integral gain term for PID control in controller dependent units */
    motorAxisDGain,            /**< (double) The differential gain term for PID control in controller dependent units */
    motorAxisClosedLoop,       /**< (int)    Enables closed loop in the controller if non-zero */
    motorAxisEncoderPosn,      /**< (double, r/o) Current encoder position */
    motorAxisDirection,        /**< (int, r/o) last motion direction 0=Negative, 1=Positive */
    motorAxisDone,             /**< (int, r/o) a motion is complete */
    motorAxisHighHardLimit,    /**< (int, r/o) plus limit switch has been hit */
    motorAxisHomeSignal,       /**< (int, r/o) The home signal is on */
    motorAxisSlip,             /**< (int, r/o) encoder slip enabled (optional - not currently used in software?) */
    motorAxisPowerOn,          /**< (int, r/o) position maintenence enabled */
    motorAxisFollowingError,   /**< (int, r/o) slip/stall detected (optional - not currently used in software?) */
    motorAxisHomeEncoder,      /**< (int, r/o) encoder home signal on */
    motorAxisHasEncoder,       /**< encoder is present */
    motorAxisProblem,          /**< driver stopped polling */
    motorAxisMoving,           /**< non-zero velocity present (optional - not currently used in software?) */
    motorAxisHasClosedLoop,    /**< Motor supports closed-loop position control. */
    motorAxisCommError,        /**< Controller communication error. */
    motorAxisLowHardLimit      /**< minus limit switch has been hit */
} motorAxisParam_t;

#define MOTOR_AXIS_NUM_PARAMS ((int) motorAxisLowHardLimit + 1)

/**\defgroup EPICS EPICS driver support interface routines
@{
*/

typedef void (*motorAxisReportFunc)( int level );
/** Print the status of the motor controller to stdout

    This optional routine is intended to provide a debugging log about the motor controller
    and will typically be called by the EPICS dbior function. The level indicates
    the level of detail - typically level 0 prints a one line summary and higher
    levels provide increasing order of detail.

    \param level   [in] Report level
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static void motorAxisReport( int level );
#endif

typedef int (*motorAxisInitFunc)( void );

/** EPICS driver support entry function initialisation routine.

    This optional routine is provided purely so the motor driver support entry table 
    (motorAxisDrvSET_t) compatible with an EPICS driver support entry table.
    Typically it won't be provided and the driver support entry table
    initialised with a null pointer. However, if provided, it may be called
    at iocInit.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisInit( void );
#endif

typedef enum {motorAxisErrInfo,motorAxisErrMinor,motorAxisErrMajor,motorAxisErrFatal} motorAxisSev_t;

typedef int (*motorAxisLogFunc)( const motorAxisSev_t severity,const char *pFormat, ...);
typedef int (*motorAxisSetLogFunc)( motorAxisLogFunc logFunc );

/** Provide an external logging routine.

    This is an optional function which allows external software to hook
    the driver log routine into an external logging system. The
    external log function is a standard printf style routine with the
    exception that it has a first parameter which is a message
    severity indicator. This can have one of four values -
    infomational, minor, major or fatal.

    \param logFunc [in] Pointer to function of motorAxisLogFunc type.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisSetLog( motorAxisLogFunc logFunc );
#endif

/**@}*/

/**\defgroup Access Routines to open and close a connection to a motion axis.
@{
*/

typedef AXIS_HDL (*motorAxisOpenFunc)( int card, int axis, char * param );

/** Initialise connection to a motor axis.

    This routine should open a connection to an motor controller axis
    and return a driver dependent handle that can be used for subsequent calls to
    other axis routines supported by the motor controller. The driver should support
    multiple opens on a single axis and process all calls from separate threads on a
    fifo basis.

    \param card  [in] Number representing the motor controller.
    \param axis  [in] Axis number - some devices may have conventions on virtual axes.
    \param param [in] Arbitrary, driver defined, parameter string.

    \return AXIS_HDL - pointer to handle for using to future calls to motorAxis routines
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static AXIS_HDL motorAxisOpen( int card, int axis, char * param );
#endif

typedef int (*motorAxisCloseFunc)( AXIS_HDL pAxis );

/** Close a connection to a motor axis.

    This routine should close the connection to a motor controller axis previously
    opened with motorOpen, and clean up all space specifically allocated to this 
    axis handle.

    \param pAxis  [in]   Pointer to axis handle returned by motorAxisOpen.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisClose( AXIS_HDL pAxis );
#endif

/**@} - end of Access group*/

/**\defgroup Callback Routines to handle callbacks of status information
@{
*/

typedef void (*motorAxisCallbackFunc)( void *param, unsigned int numReasons, unsigned int * reasons );
typedef int (*motorAxisSetCallbackFunc)( AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param );

/** Set a callback function to be called when motor axis information changes

    This routine sets a function to be called by the driver if the motor status
    changes. The interest parameter indicates a minimum update period, ranging 
    from 0 (for all updates that the driver detects) to some large number (possibly
    in clock ticks) indicating a lack of interest. Normally 0 should not flood
    a system.

    Only one callback function is allowed per AXIS_HDL, and so subsequent calls to
    this function using the original axis identifier will replace the original
    callback. Setting the callback function to a NULL pointer will delete the
    callback hook.

    \param pAxis    [in]   Pointer to axis handle returned by motorAxisOpen.
    \param callback [in]   Pointer to a callback function taking a void parameter.
    \param param    [in]   Void pointer to parameter that should be used when calling the callback

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisSetCallback( AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param );
#endif

/**@} - end of Status group*/

/**\defgroup parameters Routines that set axis control parameters
 @{
*/

typedef int (*motorAxisStringFunc)( AXIS_HDL pAxis, int, char * );

/** Pass a controller specific string to the controller

    This optional routine passes a controller specific string down to the controller.
    The exact string format depends on the controller. The length parameter allows
    the string to contain embedded nulls.

    \param pAxis    [in]   Pointer to axis handle returned by motorAxisOpen.
    \param length   [in]   Length of the string to be passed to the controller.
    \param string   [in]   Character string to be passed to the controller.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisPrimitive( AXIS_HDL pAxis, int length, char * string );
#endif

typedef int (*motorAxisSetDoubleFunc)( AXIS_HDL pAxis, motorAxisParam_t, double );

/** Sets a double parameter in the controller.

    The parameters are described in the description of the motorAxisParam_t enumeration.

    \param pAxis     [in]   Pointer to axis handle returned by motorAxisOpen.
    \param function  [in]   One of the motorAxisParam_t values indicating which parameter to set.
    \param value     [in]   Value to be assigned to the parameter.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure or not supported. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisSetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double value );
#endif

typedef int (*motorAxisSetIntegerFunc)( AXIS_HDL pAxis,  motorAxisParam_t, int );

/** Sets an integer parameter in the controller.

    The parameters are described in the description of the motorAxisParam_t enumeration.

    \param pAxis     [in]   Pointer to axis handle returned by motorAxisOpen.
    \param function  [in]   One of the motorAxisParam_t values indicating which parameter to set.
    \param value     [in]   Value to be assigned to the parameter.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure or not supported. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisSetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int value );
#endif

typedef int (*motorAxisGetDoubleFunc)( AXIS_HDL pAxis, motorAxisParam_t, double * );

/** Gets a double parameter in the controller.

    The parameters are described in the description of the motorAxisParam_t enumeration.

    \param pAxis     [in]   Pointer to axis handle returned by motorAxisOpen.
    \param function  [in]   One of the motorAxisParam_t values indicating which parameter to get.
    \param value     [out]  Value of the parameter.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure or not supported. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisGetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double * value );
#endif

typedef int (*motorAxisGetIntegerFunc)( AXIS_HDL pAxis,  motorAxisParam_t, int * );

/** Gets an integer parameter in the controller.

    The parameters are described in the description of the motorAxisParam_t enumeration.

    \param pAxis     [in]   Pointer to axis handle returned by motorAxisOpen.
    \param function  [in]   One of the motorAxisParam_t values indicating which parameter to set.
    \param value     [in]   Value of the parameter.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure or not supported. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisGetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int * value );
#endif

/**@} - end parameters group*/

/**\defgroup Motion Routines that initiate and stop motion
 @{
*/

typedef int (*motorAxisMoveFunc)( AXIS_HDL pAxis, double position, int relative, double min_velocity, double max_velocity, double acceleraton );
/** Moves the axis to a given demand position and then stops.

    This is a normal move command. Moves consist of an acceleration phase, a coast phase and a deceleration phase.
    If min_velocity is greater than zero and the controller supports this function, the system will not demand 
    a non-zero velocity between + and - min_velocity. Move-on-move (i.e. a move command being issue before another
    is completed should not generate an error - the motor should immediately start moving to the latest demanded position.
    The function should return immediately the move has been started successfully.

    NOTE: I haven't done anything about the pre- and post- move strings yet.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param position      [in]   Position to move to in motor units.
    \param relative      [in]   If zero position is an absolute position, otherwise it is relative
                                to the current position.
    \param min_velocity  [in]   Minimum startup velocity in motor units/second. If negative, it will be ignored.
    \param max_velocity  [in]   Maximum velocity during move in motor units/second.
    \param acceleration  [in]   Maximum acceleration (or decelleration) during velocity ramp in 
                                motor units/second squared. Should be positive.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisMove( AXIS_HDL pAxis, double position, int relative, double min_velocity, double max_velocity, double acceleration );
#endif

typedef int (*motorAxisHomeFunc)( AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards );
/** Homes the axis, starting in a particular direction.

    This initiates a homing operation. If the controller supports it the home procedure can be initially in either
    a forward or a reverse direction. min_velocity and max_velocity are the same as for the motorAxisMove function.
    The routine returns as soon as the home has started successfully.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param min_velocity  [in]   Minimum startup velocity in motor units/second. If negative, it will be ignored.
    \param max_velocity  [in]   Maximum velocity during move in motor units/second.
    \param acceleration  [in]   Maximum acceleration (or decelleration) during velocity ramp in 
                                motor units/second squared. Should be positive.
    \param forwards      [in]   If zero, initial move is in negative direction, otherwise it is positive (possibly ignored).

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisHome( AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards );
#endif

typedef int (*motorAxisVelocityMoveFunc)( AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration );
/** Starts the axis moving at a constant velocity

    This initiates a constant velocity move (JOG in OMS parlance). The axis is moved at the velocity
    supplied after ramping up at the rate specified by the acceleration parameter. The motion will only stop if a limit is
    hit or a motorAxisStop command is issued.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param min_velocity  [in]   Minimum startup velocity in motor units/second. If negative, it will be ignored.
    \param max_velocity  [in]   Maximum velocity during move in motor units/second.
    \param acceleration  [in]   Maximum acceleration (or decelleration) during velocity ramp in 
                                motor units/second squared. Should be positive.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/


#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisVelocityMove(  AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration );
#endif

typedef int (*motorAxisProfileMoveFunc)( AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger );
/** Starts the axis moving along a tabulated position profile

    This optional command initiates a profiled motion following a
    table of position, time pairs. The times are expressed as
    differences in seconds from the first time. Hence, the times
    should form a monotonically increasing sequence with the first
    element being zero. If the first element is non-zero, then it is
    assumed that the velocity array is only one element long and it
    contains a fixed time increment between all the steps in the
    position array.

    The exact behaviour of the motion between the points is not
    precisely defined, but it is assumed that it will be a reasonably
    smooth motion within the performance of the underlying
    system. Either linear or spline interpolation would be acceptable,
    but stopping at every point with accelerations and decelerations
    would probably not be - unless that was all that the underlying
    system was capable of. In this case, the velocity acceleration
    parameters should be adjusted so that the acceleration and
    stationary phases are minimised. It can be assumed that the motion
    table is reasonable (i.e. the underlying system can follow it)
    and, if not, the controller should just make a best efforts
    attempt or return an on return from this routine.

    The motion along the profile will not start until triggered. If
    the relative parameter is zero (i.e. the positions indicated are
    absolute), then the controller should immediately move the the
    first position and stop awaiting the trigger.

    The trigger parameter defines when to initiate the motion along
    the profile. A number of special values are defined.

       \li 0 - Start immediately (i.e. no trigger).
       \li 1 - Start immediately a motorAxisTriggerProfile is called using any AXIS_HDL for this axis.
       \li 2 - Start immediately a motorAxisTriggerProfile is called using any AXIS_HDL for this controller.

    Any other value is controller dependent, and will probably be used
    for using hardware signals special to the controller.

    The motion continues to the end of the profile where it should
    stop. It can be interrupted by any other motion command, including
    motorAxisStop.

    Controllers may have limitations about how many profiled motions
    can be available, or different trigger types. They should indicate
    this by returning an error if the demands are unacceptable.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param npoints       [in]   Number of points in the position array and time array if the first time element is zero.
    \param positions     [in]   Double precision array of positions on the profile (motor units).
    \param times         [in]   Double precision array of times (seconds)
    \param relative      [in]   Integer which is non-zero for relative positions and zero for absolute
    \param trigger       [in]   Integer indication which trigger to use to initiate motion.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/


#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisProfileMove( AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger );
#endif

typedef int (*motorAxisTriggerProfileFunc)( AXIS_HDL pAxis );
/** Triggers a previously loaded profile motion.

    This starts a motion previously initialised with motorAxisProfileMove.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisTriggerProfile( AXIS_HDL pAxis );
#endif

typedef int (*motorAxisStopFunc)( AXIS_HDL pAxis, double acceleration );
/** Stops the axis from moving.

    This aborts any current motion and brings the axis to a halt at
    the current position. The command completes as soon as the stop is initiated.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param acceleration  [in]   Maximum acceleration (or decelleration) during velocity ramp in 
                                motor units/second squared. Should be positive.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
static int motorAxisStop( AXIS_HDL pAxis, double acceleration );
#endif

/**@} end motion group*/

/** The driver support entry table */

typedef struct
{
    int number;
    motorAxisReportFunc          report;            /**< Standard EPICS driver report function (optional) */
    motorAxisInitFunc            init;              /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLogFunc          setLog;            /**< Defines an external logging function (optional) */
    motorAxisOpenFunc            open;              /**< Driver open function */
    motorAxisCloseFunc           close;             /**< Driver close function */
    motorAxisSetCallbackFunc     setCallback;       /**< Provides a callback function the driver can call when the status updates */
    motorAxisStringFunc          primitive;         /**< Passes a controller dependedent string */
    motorAxisSetDoubleFunc       setDouble;         /**< Pointer to function to set a double value */
    motorAxisSetIntegerFunc      setInteger;        /**< Pointer to function to set an integer value */
    motorAxisGetDoubleFunc       getDouble;         /**< Pointer to function to get a double value */
    motorAxisGetIntegerFunc      getInteger;        /**< Pointer to function to get an integer value */
    motorAxisHomeFunc            home;              /**< Pointer to function to execute a more to reference or home */
    motorAxisMoveFunc            move;              /**< Pointer to function to execute a position move */
    motorAxisVelocityMoveFunc    velocityMove;      /**< Pointer to function to execute a velocity mode move */
    motorAxisStopFunc            stop;              /**< Pointer to function to stop motion */
} motorAxisDrvSET_t;

#ifdef __cplusplus
}
#endif
#endif
