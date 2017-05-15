TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = TCPsimMotor
# TCPsimMotor.dbd will be created and installed
DBD += TCPsimMotor.dbd

# TCPsimMotor.dbd will be made up from these files:
TCPsimMotor_DBD += base.dbd

# Include dbd files from all support applications:
#TCPsimMotor_DBD += xxx.dbd
TCPsimMotor_DBD += asyn.dbd
TCPsimMotor_DBD += drvAsynIPPort.dbd
TCPsimMotor_DBD += motorSupport.dbd
TCPsimMotor_DBD += TCPsimMotorController.dbd

# Add source files
TCPsimMotor_SRCS += TCPsimMotorController.cpp
TCPsimMotor_SRCS += TCPsimMotorAxis.cpp

# Add all the support libraries needed by this IOC
TCPsimMotor_LIBS += asyn
TCPsimMotor_LIBS += motor

# TCPsimMotor_registerRecordDeviceDriver.cpp derives from TCPsimMotor.dbd
TCPsimMotor_SRCS += TCPsimMotor_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
TCPsimMotor_SRCS_DEFAULT += TCPsimMotorMain.cpp
TCPsimMotor_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#TCPsimMotor_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
TCPsimMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

