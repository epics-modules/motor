# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += NoMPF.dbd

PROD_IOC_vxWorks = NoMPF

NoMPF_SRCS += NoMPF_registerRecordDeviceDriver.cpp
NoMPF_SRCS_DEFAULT += NoMPFMain.cpp
NoMPF_SRCS_vxWorks += -nil-

#The following adds support from base/src/vxWorks
NoMPF_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

NoMPF_LIBS         += motor
NoMPF_LIBS_vxWorks += oms
NoMPF_LIBS         += softMotor
NoMPF_LIBS         += MXmotor

NoMPF_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

