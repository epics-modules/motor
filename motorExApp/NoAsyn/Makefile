#FILENAME...     Makefile
#USAGE...        Makefile for motor application example without Asyn


# "#!" marks lines that can be uncommented.

# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

PROD_IOC_DEFAULT = NoAsyn
PROD_IOC_vxWorks = NoAsynVx

COMMONDBDS  = base.dbd
COMMONDBDS += motorRecord.dbd
COMMONDBDS += devSoftMotor.dbd
#!COMMONDBDS += devMXmotor.dbd

DBD += NoAsyn.dbd
NoAsyn_DBD += $(COMMONDBDS)
NoAsyn_DBD += Misc.dbd

DBD += NoAsynVx.dbd
NoAsynVx_DBD += $(COMMONDBDS)
ifndef NO_MOTOR_DELTATAUSRC
NoAsynVx_DBD += devDeltaTau.dbd
endif
NoAsynVx_DBD += MiscVx.dbd

NoAsyn_SRCS += NoAsyn_registerRecordDeviceDriver.cpp
NoAsyn_SRCS_DEFAULT += NoAsynMain.cpp
NoAsynVx_SRCS += NoAsynVx_registerRecordDeviceDriver.cpp

#!COMMONLIBS += MXmotor
COMMONLIBS += softMotor
ifeq ($(OS_CLASS), vxWorks)
COMMONLIBS += oms
COMMONLIBS += DeltaTau
endif
COMMONLIBS += motor

NoAsyn_LIBS += $(COMMONLIBS)
NoAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

NoAsynVx_LIBS += $(COMMONLIBS)
NoAsynVx_OBJS += $(EPICS_BASE_BIN)/vxComLibrary
NoAsynVx_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

