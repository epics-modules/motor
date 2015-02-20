TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += motorSimSupport

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += motorSimSupport.dbd

# The following are compiled and added to the Support library
motorSimSupport_SRCS += route.c
motorSimSupport_SRCS += devMotorSim.c
motorSimSupport_SRCS += drvMotorSim.c
motorSimSupport_SRCS += motorSimDriver.cpp
motorSimSupport_SRCS += motorSimRegister.cc

motorSimSupport_LIBS += motor
motorSimSupport_LIBS += asyn
motorSimSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# build an ioc application

PROD_IOC = motorSim
# motorSim.dbd will be created and installed
DBD += motorSim.dbd

# motorSim.dbd will be made up from these files:
motorSim_DBD += base.dbd
motorSim_DBD += asyn.dbd
motorSim_DBD += motorSupport.dbd
motorSim_DBD += motorSimSupport.dbd
motorSim_DBD += asSupport.dbd
motorSim_DBD += calcSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
motorSim_SRCS += motorSim_registerRecordDeviceDriver.cpp
motorSim_SRCS_DEFAULT += motorSimMain.cpp
motorSim_SRCS_vxWorks += -nil-

# The following adds support from base/src/vxWorks
motorSim_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

motorSim_LIBS += motorSimSupport
motorSim_LIBS += motor
motorSim_LIBS += asyn
motorSim_LIBS += autosave
motorSim_LIBS += calc sscan

motorSim_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

# SCRIPTS += motorSimTest.boot
DB += motorSimTest.db

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

