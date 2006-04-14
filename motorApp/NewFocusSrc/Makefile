# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
USR_CXXFLAGS += -DDEBUG

OPT_CXXFLAGS = 

DBD += devNewFocus.dbd

LIBRARY_IOC = NewFocus

SRCS += NewFocusRegister.cc

# PMNC8750 (i.e., PMNC8750) device driver.
SRCS += devPMNC8750.cc drvPMNC8750.cc

NewFocus_LIBS += motor asyn
NewFocus_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

