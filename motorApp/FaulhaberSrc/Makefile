# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devFaulhaberMotor.dbd

LIBRARY_IOC = Faulhaber

# Intelligent Motion Systems driver support.
SRCS += devMCDC2805.cc drvMCDC2805.cc

Faulhaber_LIBS += motor asyn
Faulhaber_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

