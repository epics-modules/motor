# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#
# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devAttocube.dbd

LIBRARY_IOC = Attocube

# ANC 150 asyn motor driver.
Attocube_SRCS += drvANC150Asyn.cc

Attocube_LIBS += motor asyn
Attocube_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

