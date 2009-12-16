# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBDINC += motorRecord.h
DBD += motorSupport.dbd

INC += motor.h motordevCom.h motordrvCom.h
INC += motordrvComCode.h

ifdef ASYN
INC += motor_interface.h
INC += paramLib.h
INC += asynMotorDriver.h
endif

LIBRARY_IOC += motor

# The following are required for all motor record configurations.
motor_SRCS += motorRecord.cc motordevCom.cc motordrvCom.cc
motor_SRCS += motorUtil.cc motorUtilAux.cc

ifdef ASYN
motor_SRCS += drvMotorAsyn.c
motor_SRCS += devMotorAsyn.c
motor_SRCS += paramLib.c
motor_SRCS += asynMotorDriver.cpp
motor_LIBS += asyn
endif

motor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
