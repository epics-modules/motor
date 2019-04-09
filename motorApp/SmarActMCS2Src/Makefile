TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += SmarActMCS2Motor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += devSmarActMCS2.dbd

# The following are compiled and added to the Support library
SmarActMCS2Motor_SRCS += SmarActMCS2MotorDriver.cpp

SmarActMCS2Motor_LIBS += motor
SmarActMCS2Motor_LIBS += asyn
SmarActMCS2Motor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

