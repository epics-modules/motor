# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devImsMotor.dbd

LIBRARY_IOC = Ims

# Intelligent Motion Systems driver support.
SRCS += ImsRegister.cc
SRCS += devIM483SM.cc drvIM483SM.cc
SRCS += devIM483PL.cc drvIM483PL.cc
SRCS += devMDrive.cc  drvMDrive.cc

Ims_LIBS += motor asyn
Ims_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

