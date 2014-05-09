# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devOms.dbd

LIBRARY_IOC_DEFAULT += oms

# The following is required for the OMS PC68/78 (i.e., devOmsPC68) device driver.
ifdef ASYN
SRCS += devOmsPC68.cc drvOmsPC68.cc
endif

# The following is required for the OMS VME8/44 (i.e., devOMS) device driver.
SRCS += devOms.cc drvOms.cc

# The following is required for the OMS VME58 (i.e., devOms58) device driver.
SRCS += devOms58.cc drvOms58.cc

# OMS MAXv device driver support.
SRCS += devMAXv.cc drvMAXv.cc

# Trajectory support for MAXV controller
ifdef SNCSEQ
ifdef ASYN
SRCS += MAX_trajectoryScan.st
endif
endif

# The following is required for all OMS device drivers.
SRCS += devOmsCom.cc

oms_LIBS += motor
ifdef ASYN
oms_LIBS += asyn
endif

# MAXV_trajectoryScan.st needs the following:
ifdef SNCSEQ
oms_LIBS += seq pv
endif


oms_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
