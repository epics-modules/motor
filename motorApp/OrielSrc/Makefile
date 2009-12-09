# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

OPT_CXXFLAGS = -g -O0

DBD += devOriel.dbd

LIBRARY_IOC = Oriel

# Intelligent Motion Systems driver support.
SRCS += OrielRegister.cc
SRCS += devEMC18011.cc drvEMC18011.cc

Oriel_LIBS += motor asyn
Oriel_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

