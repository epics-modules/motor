# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += devLinMotMotor.dbd

LIBRARY_IOC = LinMot

# Intelligent Motion Systems driver support.
SRCS += LinMotRegister.cc
SRCS += devLinMot.cc drvLinMot.cc

LinMot_LIBS += motor
LinMot_LIBS += asyn
LinMot_LIBS += seq
LinMot_LIBS += pv
LinMot_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

