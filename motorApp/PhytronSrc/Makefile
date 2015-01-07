TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
# Build the IOC application

LIBRARY_IOC += phytronAxisMotor

DBD += phytron.dbd

# The following are compiled and added to the support library
phytronAxisMotor_SRCS += phytronAxisMotor.cpp

INC += phytronAxisMotor.h

phytronAxisMotor_LIBS += motor
phytronAxisMotor_LIBS += asyn
phytronAxisMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES

