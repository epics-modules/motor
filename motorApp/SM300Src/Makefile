# Makefile
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_IOC  = SM300Motor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += SM300Motor.dbd

# The following are compiled and added to the Support library
SRCS += SM300Register.cc

SRCS += SM300Driver.cpp


SM300Motor_LIBS += motor
SM300Motor_LIBS += asyn
SM300Motor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

