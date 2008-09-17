# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devSoftMotor.dbd

LIBRARY_IOC = softMotor

# The following is required for the Soft Channel (i.e., MotorSoft) device driver.
SRCS += devSoft.cc devSoftAux.cc

softMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

