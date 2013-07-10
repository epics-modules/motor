# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += MicronixSupport.dbd

LIBRARY_IOC = Micronix

# Modular Motion Controller MMC-200 
SRCS += MMC200Driver.cpp

Micronix_LIBS += motor asyn
Micronix_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

