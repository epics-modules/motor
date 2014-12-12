Readme for PIasyn - EPICS support for PI GCS2 stages
========================================================

The PIasyn driver is written to support PI motion controllers which
support GCS2 (General Command Set) as commanding language.
Currently this is implemented with different C++ classes where the
differences and specialties are handled. This can be implemented
by querying the controller what features are supported. So in future
developments these classes may disappear.

 Homing
========

a) stages with absolute sensors
Some PI stages, mostly piezo stages, use an absolute position sensor.
The correct absolute position is known to the controller immediately after power up.
For these stages homing is not necessary and thus not supported.
These stages will ignore HOMF and HOMR.
The HOME flag of the MSTA field is meaningless for these stages
and HOMED is always true,

b) stages with incremental sensors
Some PIStages needs to be referenced after controller power up. The state
is reflected by the HOMED flag of MSTA. There are different ways to home
a stage. The default is by using the reference switch. If a stage has
a reference switch HOMF and HOMR will move to this switch. The direction is
determined by the controller. The reference signal is edge based, so the controller
can find out on which side of the reference the stage is and can move using
the shortest path. HOMF and HOMR will trigger the same move. If the stage has
no reference switch the built in limit switches can be used to determine the absolute
position. In this case HOMF will send the stage to its positive limit switch and
HOMR to its negative limit switch. As the reference is edge based or not present the
HOME bit of the MSTA field is meaningless for these stages.

c) setting the homing velocity
Reducing the velocity used for homing is usually done to improve the accuracy.
PI controllers reach the reference every time from the same "side". So only this
last approach is performed with the homing velocity set with HVEL. The coarse move
to the proximity of the reference is done with the a velocity limited by the current
velocity (set with VELO) and the maximum possible velocity to decelerate in case of
hitting a limit switch.

d) stages without reference or limit switch sensors
The position can be set with setting the SET fiels to "Set"(1) and then setting DVAL
to the new position to be used. This will cause no motion on the controller.
The controller must support the commands "RON" and "POS"
(please see the user manual of the controller for details).
Especially digital piezo contorllers do not support this. 

 "Motion Errors"
=================

If the controller detects a "motion error", i.e. the difference between target and
current position is greater than some given limit (also known as "following error"),
the driver will set the PROBLEM bit in MSTA. Servo control is also switched off.
The EA_POSITION bit of MSTA will reflect the "servo control" state. To re-enable
servo control use CNEN.  


 C-702
=======

The C-702 controller uses an older "dialect" of GCS1.
That's why at some points different parameter IDs are used and why there is no communication
during homing. So the position cannot be updated during homing and will "jump" to the final
position after homing is finished.


