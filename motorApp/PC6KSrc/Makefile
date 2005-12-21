# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

OPT_CXXFLAGS = 

DBD += devPC6K.dbd

LIBRARY_IOC = Parker

# Intelligent Motion Systems driver support.
SRCS += ParkerRegister.cc
SRCS += devPC6K.cc drvPC6K.cc

Parker_LIBS += motor asyn
Parker_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

