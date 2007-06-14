# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += devPIJena.dbd

LIBRARY_IOC = PIJena

# piezosystem jena motor controller support
SRCS += devPIJEDS.cc drvPIJEDS.cc PIJEDS_Register.cc

PIJena_LIBS += motor asyn
PIJena_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

