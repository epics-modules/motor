# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devOms.dbd

LIBRARY_IOC_DEFAULT += oms
LIBRARY_IOC_WIN32 += -nil-

# The following is required for the OMS VME8/44 (i.e., devOMS) device driver.
SRCS += devOms.cc drvOms.cc

# The following is required for the OMS VME58 (i.e., devOms58) device driver.
SRCS += devOms58.cc drvOms58.cc

# The following is required for the OMS MAXv device driver.
SRCS += devMAXv.cc drvMAXv.cc

# The following is required for all OMS device drivers.
SRCS += devOmsCom.cc

oms_LIBS += motor
oms_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
