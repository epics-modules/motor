# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devOms.dbd

LIBRARY_IOC_vxWorks = oms

# The following is required for all OMS device drivers.
SRCS += devOmsCom.cc

# The following is required for the OMS VME8/44 (i.e., devOMS) device driver.
SRCS += devOms.cc drvOms.cc

# The following is required for the OMS VME58 (i.e., devOms58) device driver.
SRCS += devOms58.cc drvOms58.cc

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
