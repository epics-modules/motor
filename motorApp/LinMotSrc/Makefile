# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devLinMotMotor.dbd

LIBRARY_IOC = LinMotMotor

# Khozu SC-800 motor controller.
SRCS += devLinMot.cc drvLinMot.cc

LinMotMotor_LIBS += motor asyn
LinMotMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

