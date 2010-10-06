# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devSPiiPlus.dbd

LIBRARY_IOC = acsTech80

# Intelligent Motion Systems driver support.
SRCS += ACSTech80Register.cc
SRCS += devSPiiPlus.cc drvSPiiPlus.cc

acsTech80_LIBS += motor asyn
acsTech80_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

