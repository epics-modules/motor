# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
# USR_CXXFLAGS += -DDEBUG
# OPT_CXXFLAGS = -g -O0

DBD += devNewFocus.dbd

LIBRARY_IOC = NewFocus

SRCS += NewFocusRegister.cc

# PMNC87xx (i.e., PMNC87xx) device driver.
SRCS += devPMNC87xx.cc drvPMNC87xx.cc

NewFocus_LIBS += motor asyn
NewFocus_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

