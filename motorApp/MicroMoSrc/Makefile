# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devMicroMo.dbd

LIBRARY_IOC = MicroMo

# Intelligent Motion Systems driver support.
SRCS += MicroMoRegister.cc
SRCS += devMVP2001.cc drvMVP2001.cc

MicroMo_LIBS += motor motorCOM_mpf
MicroMo_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

