# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

OPT_CXXFLAGS = -g -O0

DBD += devThorLabs.dbd

LIBRARY_IOC = ThorLabs

# Intelligent Motion Systems driver support.
SRCS += ThorLabsRegister.cc
SRCS += devMDT695.cc drvMDT695.cc

ThorLabs_LIBS += motor asyn
ThorLabs_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

