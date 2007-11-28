# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += NoAsyn.dbd
DBD += NoAsynVx.dbd

PROD_IOC_DEFAULT = NoAsyn
PROD_IOC_vxWorks = NoAsynVx

NoAsyn_SRCS += NoAsyn_registerRecordDeviceDriver.cpp
NoAsyn_SRCS_DEFAULT += NoAsynMain.cpp
#!NoAsyn_LIBS += MXmotor
#!NoAsyn_LIBS += softMotor
#!NoAsyn_LIBS += oms
NoAsyn_LIBS += motor
NoAsyn_LIBS += asyn
NoAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

NoAsynVx_SRCS += NoAsynVx_registerRecordDeviceDriver.cpp
#!NoAsynVx_LIBS += softMotor
#!NoAsynVx_LIBS += oms
#!NoAsynVx_LIBS += DeltaTau
NoAsynVx_LIBS += motor
NoAsynVx_LIBS += asyn


NoAsynVx_OBJS += $(EPICS_BASE_BIN)/vxComLibrary
NoAsynVx_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

