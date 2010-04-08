# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devAerotech.dbd

LIBRARY_IOC = Aerotech

# Aerotech driver support.
SRCS += AerotechRegister.cc
SRCS += devEnsemble.cc drvEnsemble.cc
SRCS += devSoloist.cc  drvSoloist.cc
SRCS += drvEnsembleAsyn.cc

Aerotech_LIBS += motor asyn
Aerotech_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

