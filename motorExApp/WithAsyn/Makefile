#FILENAME...     Makefile
#USAGE...        Makefile for motor application example with Asyn


# "#!" marks lines that can be uncommented.

TOP = ../..
include $(TOP)/configure/CONFIG

PROD_IOC_DEFAULT = WithAsyn
PROD_IOC_vxWorks = WithAsynVx

COMMONDBDS  = base.dbd
COMMONDBDS += motorSupport.dbd
ifndef NO_MOTOR_ACSSRC
COMMONDBDS += devAcsMotor.dbd
endif
ifndef NO_MOTOR_IMSSRC
COMMONDBDS += devImsMotor.dbd
endif
ifndef NO_MOTOR_MCLENNANSRC
COMMONDBDS += devMclennanMotor.dbd
endif
ifndef NO_MOTOR_MICOSSRC
COMMONDBDS += devMicos.dbd
endif
ifndef NO_MOTOR_MICOSSRC
COMMONDBDS += devMicroMo.dbd
endif
ifndef NO_MOTOR_NEWPORTSRC
COMMONDBDS += devNewport.dbd
endif
ifdef SNCSEQ
COMMONDBDS += LdevNewportTS.dbd
endif
ifndef NO_MOTOR_PISRC
COMMONDBDS += devPIMotor.dbd
endif
ifndef NO_MOTOR_OMSSRC
COMMONDBDS += devOms.dbd
endif
COMMONDBDS += devSoftMotor.dbd
COMMONDBDS += motorSimSupport.dbd
ifndef NO_MOTOR_SMARTMOTORSRC
COMMONDBDS += devSmartMotorMotor.dbd
endif
ifndef NO_MOTOR_KOHZUSRC
COMMONDBDS += devKohzuMotor.dbd
endif
ifndef NO_MOTOR_ATTOCUBESRC
COMMONDBDS += devAttocube.dbd
endif
ifndef NO_MOTOR_AEROTECHSRC
COMMONDBDS += devAerotech.dbd
endif
ifndef NO_MOTOR_ACRSRC
COMMONDBDS += ACRMotorSupport.dbd
endif
COMMONDBDS += asyn.dbd
COMMONDBDS += drvAsynSerialPort.dbd
COMMONDBDS += drvAsynIPPort.dbd
ifdef BUSY
	COMMONDBDS += busySupport.dbd
	COMMONLIBS := busy $(COMMONLIBS)
endif
ifndef NO_MOTOR_PISRC
COMMONDBDS += PI_GCS2Support.dbd
endif
ifndef NO_MOTOR_PHYTRONSRC
COMMONDBDS += phytron.dbd
endif

DBD += WithAsyn.dbd
WithAsyn_DBD += $(COMMONDBDS)
ifdef SNCSEQ
	# seq-2-1 and earlier has devSequencer
	ifeq (seq-2-1, $(findstring seq-2-1, $(SNCSEQ)))
		WithAsyn_DBD += devSequencer.dbd
	else ifeq (seq-2.1, $(findstring seq-2.1, $(SNCSEQ)))
		WithAsyn_DBD += devSequencer.dbd
	endif

endif

DBD += WithAsynVx.dbd
WithAsynVx_DBD += $(COMMONDBDS)
#!WithAsynVx_DBD += drvGsIP488.dbd
ifdef IPAC
WithAsynVx_DBD += MiscVx.dbd
endif

WithAsyn_SRCS   += WithAsyn_registerRecordDeviceDriver.cpp
WithAsyn_SRCS_DEFAULT += WithAsynMain.cpp
WithAsynVx_SRCS += WithAsynVx_registerRecordDeviceDriver.cpp

WithAsyn_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

ifndef NO_MOTOR_AEROTECHSRC
COMMONLIBS += Aerotech
endif
ifndef NO_MOTOR_ATTOCUBESRC
COMMONLIBS += Attocube
endif
ifndef NO_MOTOR_OMSSRC
COMMONLIBS += oms
endif
ifndef NO_MOTOR_MCLENNANSRC
COMMONLIBS += Mclennan
endif

ifndef NO_MOTOR_MICOSSRC
COMMONLIBS += Micos
COMMONLIBS += MicroMo
endif
ifndef NO_MOTOR_PISRC
COMMONLIBS += PI
endif
ifndef NO_MOTOR_ACSSRC
COMMONLIBS += Acs
endif
ifndef NO_MOTOR_IMSSRC
COMMONLIBS += Ims
endif
ifndef NO_MOTOR_NEWPORTSRC
COMMONLIBS += Newport
endif
COMMONLIBS += softMotor
COMMONLIBS += motorSimSupport
ifndef NO_MOTOR_SMARTMOTORSRC
COMMONLIBS += SmartMotor
endif
ifndef NO_MOTOR_KOHZUSRC
COMMONLIBS += KohzuMotor
endif
ifndef NO_MOTOR_ACRSRC
COMMONLIBS += ACRMotor
endif
ifndef NO_MOTOR_PISRC
COMMONLIBS += PI_GCS2Support
endif
ifndef NO_MOTOR_PHYTRONSRC
COMMONLIBS += phytronAxisMotor
endif

COMMONLIBS += motor

# Needed for Newport SNL programs
WithAsyn_LIBS += $(COMMONLIBS)
WithAsyn_LIBS += asyn
ifdef SNCSEQ
	# seq-2-1 and earlier has devSequencer
	ifeq (seq-2-1, $(findstring seq-2-1, $(SNCSEQ)))
		WithAsyn_LIBS += seqDev seq pv
	else ifeq (seq-2.1, $(findstring seq-2.1, $(SNCSEQ)))
		WithAsyn_LIBS += seqDev seq pv
	else
		# seq 2.2.
		WithAsyn_LIBS += seq pv
	endif
endif
WithAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

WithAsynVx_LIBS += $(COMMONLIBS)
ifdef IPAC
WithAsynVx_LIBS += Ipac TyGSOctal
endif
WithAsynVx_LIBS += asyn
# Needed for Newport SNL programs
ifdef SNCSEQ
	ifeq (seq-2-1, $(findstring seq-2-1, $(SNCSEQ)))
		WithAsynVx_LIBS += seqDev seq pv
	else ifeq (seq-2.1, $(findstring seq-2.1, $(SNCSEQ)))
		WithAsynVx_LIBS += seqDev seq pv
	else
		# seq 2.2.
		WithAsynVx_LIBS += seq pv
	endif
endif
WithAsynVx_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

