# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devPIMotor.dbd

LIBRARY_IOC = PI

# Intelligent Motion Systems driver support.
SRCS += PiRegister.cc
SRCS += devPIC844.cc drvPIC844.cc

PI_LIBS += motor asyn
PI_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

