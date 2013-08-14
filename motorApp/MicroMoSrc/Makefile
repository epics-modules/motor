# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devMicroMo.dbd

LIBRARY_IOC = MicroMo

# MVP2001 driver
SRCS += MicroMoRegister.cc
SRCS += devMVP2001.cc drvMVP2001.cc
# MVP2001 asyn model 3 driver
SRCS += MVP2001Driver.cpp

MicroMo_LIBS += motor asyn
MicroMo_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

