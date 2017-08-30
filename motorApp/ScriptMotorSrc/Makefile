# Makefile
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_IOC  = ScriptMotor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += ScriptMotorDriver.dbd

INC += ScriptMotorDriver.h

# The following are compiled and added to the Support library
ScriptMotor_SRCS += ScriptMotorDriver.cpp

ScriptMotor_LIBS += motor
ScriptMotor_LIBS += asyn
ScriptMotor_LIBS += lua
ScriptMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

