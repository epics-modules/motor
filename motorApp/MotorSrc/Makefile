# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
#USR_CFLAGS += -DDEBUG
#USR_CXXFLAGS += -DDEBUG

DBDINC += motorRecord.h

INC += motor.h motordevCom.h motordrvCom.h
INC += motordrvComCode.h

LIBRARY_IOC += motor

# The following are required for all motor record configurations.
motor_SRCS += motorRecord.cc motordevCom.cc motordrvCom.cc

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
