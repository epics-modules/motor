# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += devAcsMotor.dbd

LIBRARY_IOC = Acs

SRCS += AcsRegister.cc

# Advanced Control Systems driver support.
SRCS += devMCB4B.c drvMCB4B.c

Acs_LIBS += motor motorCOM_mpf
Acs_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

