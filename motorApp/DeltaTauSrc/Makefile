# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += devDeltaTau.dbd

LIBRARY_IOC_vxWorks = DeltaTau

SRCS += devPmac.cc drvPmac.cc


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
