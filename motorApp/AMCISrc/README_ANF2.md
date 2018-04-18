# AMCI ANF2 

Asyn model 3 driver support for the AMCI ANF2 stepper motor controller

Modbus/TCP communication, using Mark Rivers' modbus module

## Supported Controller Models

The following ANF controller versions are supported:

ANF1E: 1-axis stepper controller, modbus tcp/ip
ANF1:  1-axis stepper controller, no network interface
ANF2E: 2-axis stepper controller, modbus tcp/ip
ANF2:  2-axis stepper controller, no network interface

A stack of controller can contain up to 6 modules, one of
which needs to have the ethernet option.  A single-channel
implementation would need the module with ethernet.

## Vendor Software

ANF2 configuration software is available from AMCI's website:

www.amci.com/product-software.asp

Note: The AMCI Net Configurator only works (allows a motor to be moved) if
the controller is configured for  EtherNet/IP, however, the EPICS support 
requires the device to be configured for Modbus-TCP.

## Controller Quirks

* The controller doesn't allow absolute moves when a limit is active; 
The only way to move a motor off of a limit is by jogging.

* The base speed is set when an axis is configured.  This driver corrects 
the acceleration sent by the motor record (VBAS isn't guaranteed to match
the base speed) and sends the acceleration necessary to achieve the desired
acceleration time.

* The controller doesn't remember its configuration after it is power-cycled.

* Sending the configuration to the controller invalidates the position
requiring either a home search or the redefinition of the current position.

* The configuration can't be read after a configuration is accepted by the
controller, after which it automatically switches into command mode.

* The command to stop an abosolute move generates an error if a jog is in 
progress.  The command to stop a jog doesn't stop an absolute move.

## Controller Configuration

The AMCI Net Configurator can be used to:

* Change the ip address from the default (192.168.0.50)

* Change the protocol to Modbus-TCP

* Determine hex config strings for each axis

### Example hex configurations

The AMCI Net Configurator can be used to generate hex configurations.  Here are some example:
```
0x86000000 - Step & Direction pulses, Diagnostic Feedback, No home switch, No limits
0x86280000 - Step & Direction pulses, Diagnostic Feedback, No home switch, Active-low CW/CCW limits
0x84000000 - Step & Direction pulses, No Feedback, No home switch, No Limits
0x84280000 - Step & Direction pulses, No Feedback, No home switch, Active-low CW/CCW limits
0x842C0004 - Step & Direction pulses, No Feedback, Active-high home switch, Active-low CW/CCW limits
0x85280000 - Step & Direction pulses, Quadrature Feedback, No home switch, Active-low CW/CCW limits
```
