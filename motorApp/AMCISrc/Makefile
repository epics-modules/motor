TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += AMCI

SRCS += AMCIRegister.cc

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += ANG1Support.dbd

# The following are compiled and added to the Support library
AMCI_SRCS += ANG1Driver.cpp

AMCI_LIBS += motor
AMCI_LIBS += asyn
AMCI_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

