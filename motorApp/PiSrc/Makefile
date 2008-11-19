# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devPIMotor.dbd

LIBRARY_IOC = PI

# Intelligent Motion Systems driver support.
SRCS += devPIC844.cc drvPIC844.cc PiRegister.cc
SRCS += devPIC630.cc drvPIC630.cc PIC630Register.cc
SRCS += devPIC848.cc drvPIC848.cc PIC848Register.cc
SRCS += devPIC662.cc drvPIC662.cc PIC662Register.cc
SRCS += devPIC862.cc drvPIC862.cc PIC862Register.cc
SRCS += devPIE710.cc drvPIE710.cc PIE710Register.cc
SRCS += devPIE516.cc drvPIE516.cc PIE516Register.cc
SRCS += devPIE816.cc drvPIE816.cc PIE816Register.cc


PI_LIBS += motor asyn
PI_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

