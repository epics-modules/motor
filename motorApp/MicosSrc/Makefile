# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devMicos.dbd

LIBRARY_IOC = Micos

# Intelligent Motion Systems driver support.
SRCS += MicosRegister.cc
SRCS += devMicos.cc drvMicos.cc

Micos_LIBS += motor asyn
Micos_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

