# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devMicroMo.dbd

LIBRARY_IOC = MicroMo

# Intelligent Motion Systems driver support.
SRCS += MicroMoRegister.cc
SRCS += devMVP2001.cc drvMVP2001.cc

MicroMo_LIBS += motor asyn
MicroMo_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

