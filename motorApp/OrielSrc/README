
                     Spectra-Physics Oriel Encoder Mike Motor Controllers
==============================================================================

Model: 18011 (Three motor controller/driver)


RS232 Configuration
----------------------------------

Default: 4800, 8, 1, N (DB25)	- Internal Jumper Setting to 19200

Software Setup: REMOTE/LOCAL switching ("R", "L")


Controller Specifications
------------------------------------------
No Software configuration available 

Position Units: Microns (format %.1f)
	Precision: 0.1 microns
	Limits:	   0.1 to 999999  (maximum string length = 7char)

Velocity Units: um/s (format %.1f)
	From 		To 		Precision
	0.5		4.99 	   	  %.2f
	5.0		49.9 	   	  %.1f
	50		200 	   	  %.0f


Single Character Commands 
------------------------------------------
       Command 		Successful Response		Failure Response
Local
	"L"		"\r\n\rOFF LINE\r\n"		"" (Already in local)
Remote
	"R"		"\rON LINE\r\n"			"" (Already in remote)
Jog + 
	">"		"\r\n"				"" (Already moving)
Jog - 
	"<"	 	"\r\n"				"" (Already moving)
Stop   
	"S"		"STOP\r\n"			"" (Already stopped)

Multi Character Commands 
------------------------------------------
       		Command 		Successful Response		Failure Response
Zero Abs 
Position	"CA\r"			"CA\r\n"			""  or      (Motor in motion)
									"xxxx.x\r\n"  (position info???)
Zero Rel 
Position	"CR\r"			"CR\r\n"			"" (when motor in motion)

Goto Abs 
Position	"Gxxx.x\r"		"Gxxx.x\r\r\n"			""  (when motor in motion)

Rel Position	"Txxx.x\r"		"Txxx.x\r\r\n"			""  (when motor in motion)

Set Velocity
		"Vx.x\r"		"Vx.x\r\r\n"			"" (when motor in motion)
									"Vx.x\r\r\n" (Echos command even if invalid)

Select Axis	
		"Mx\r"			"Mx\r\r\n"			"" (when motor in motion)


Inquiries Commands 
------------------------------------------
       		Command 		Successful Response		Failure Response
Abs Pos		"A\r"			    "xxxx.x\r\n"		  none
Abs Pos		"Z"			    "<char>"			  none
					    a,b,c,d,e



     
============== Build Info ======================

xxxApp/src
    Makefile
    --------
	xxx_Common_LIBS += Parker

    xxCommonInclude.dbd
    -------------------
	include "devEMC18011.dbd"


============= IOC Boot info ======================

iocBoot/iocLinux
    serial.cmd
    ----------

	# serial 1 is a RS232 link to a Oriel Encoder Mike Controller
	drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)
	asynSetOption(serial1,0,baud,4800)
	asynOctetSetOutputEos("serial1",0,"\r")
	asynOctetSetInputEos("serial1",0,"\n")


	.	
	.

	# Oriel Encoder Mike - driver setup parameters:
	#     (1) maximum number of controllers in system
	#     (2) motor task polling rate (min=1Hz, max=60Hz)
	EMC18011Setup(2, 60)

	# Oriel Encoder Mike driver configuration parameters:
	#     (1) controller being configured
	#     (2) asyn port name (string)
	EMC18011Config(0, "serial1")
	EMC18011Config(1, "serial3")
	

   motor.substitutions
   -------------------
	Set the DTYPE column to "EMC18011"


=================== Motor Record Setup ================================

EGU = microns
VELO = [0.5 to 200] Maximum and Base velocity not used

BDST = 0 	Controller does its own backlash 

MRES = .010 (set to velocity resolution, position resolution is .10 microns)
RDBD = .05  (position resolution is only 0.1 - will be rounded up)






