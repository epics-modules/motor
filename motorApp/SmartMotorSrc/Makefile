# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devSmartMotorMotor.dbd

LIBRARY_IOC = SmartMotor

# Intelligent Motion Systems driver support.
SRCS += devSmartMotor.cc drvSmartMotor.cc

SmartMotor_LIBS += motor asyn
SmartMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

