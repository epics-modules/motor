# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += NoMPF.dbd
DBD += NoMPFVx.dbd

PROD_IOC_DEFAULT = NoMPF
PROD_IOC_vxWorks = NoMPFVx

NoMPF_SRCS += NoMPF_registerRecordDeviceDriver.cpp
NoMPF_SRCS_DEFAULT += NoMPFMain.cpp
#!NoMPF_LIBS += MXmotor
#!NoMPF_LIBS += softMotor
NoMPF_LIBS += motor
NoMPF_LIBS += $(EPICS_BASE_IOC_LIBS)

NoMPFVx_SRCS += NoMPFVx_registerRecordDeviceDriver.cpp
#!NoMPFVx_LIBS += softMotor
#!NoMPFVx_LIBS += oms
#!NoMPFVx_LIBS += DeltaTau
NoMPFVx_LIBS += motor

NoMPFVx_OBJS += $(EPICS_BASE_BIN)/vxComLibrary
NoMPFVx_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

