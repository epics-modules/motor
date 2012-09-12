# Makefile
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_IOC  = smarActMCSMotor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += devSmarActMCSMotor.dbd

INC += smarActMCSMotorDriver.h

# The following are compiled and added to the Support library
smarActMCSMotor_SRCS += smarActMCSMotorDriver.cpp

smarActMCSMotor_LIBS += motor
smarActMCSMotor_LIBS += asyn
smarActMCSMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

