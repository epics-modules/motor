# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devAerotech.dbd

LIBRARY_IOC = Aerotech

# Aerotech driver support.
SRCS += AerotechRegister.cc
SRCS += devSoloist.cc  drvSoloist.cc
SRCS += drvEnsembleAsyn.cc
SRCS += drvA3200Asyn.cc

# EnsemblePSOFly.db support
SRCS += concatString.c

ifdef SNCSEQ
# Ensemble trajectory support
SRCS += EnsembleTrajectoryScan.st
endif

Aerotech_LIBS += motor asyn
ifdef SNCSEQ
Aerotech_LIBS += seq pv
endif
Aerotech_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

