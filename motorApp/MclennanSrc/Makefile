# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devMclennanMotor.dbd

LIBRARY_IOC = Mclennan

# Intelligent Motion Systems driver support.
SRCS += MclennanRegister.cc
SRCS += devPM304.cc drvPM304.cc

Mclennan_LIBS += motor
Mclennan_LIBS += asyn
Mclennan_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

