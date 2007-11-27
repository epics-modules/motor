# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devKohzuMotor.dbd

LIBRARY_IOC = KohzuMotor

# Khozu SC-800 motor controller.
SRCS += devSC800.cc drvSC800.cc

KohzuMotor_LIBS += motor asyn
KohzuMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

