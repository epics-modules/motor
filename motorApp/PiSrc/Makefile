# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devPIMotor.dbd

LIBRARY_IOC = PI

# Intelligent Motion Systems driver support.
SRCS += devPIC844.cc drvPIC844.cc PiRegister.cc
SRCS += devPIC630.cc drvPIC630.cc PIC630Register.cc
SRCS += devPIC848.cc drvPIC848.cc PIC848Register.cc

PI_LIBS += motor asyn
PI_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

