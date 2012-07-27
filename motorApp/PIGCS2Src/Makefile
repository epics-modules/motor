TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += PI_GCS2Support

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += PI_GCS2Support.dbd

# The following are compiled and added to the Support library
PI_GCS2Support_SRCS += PIasynController.cpp
PI_GCS2Support_SRCS += PIasynAxis.cpp
PI_GCS2Support_SRCS += PIGCSController.cpp
PI_GCS2Support_SRCS += PIGCSMotorController.cpp
PI_GCS2Support_SRCS += PIGCSPiezoController.cpp
PI_GCS2Support_SRCS += PIE517Controller.cpp
PI_GCS2Support_SRCS += PIE755Controller.cpp
PI_GCS2Support_SRCS += PIHexapodController.cpp
PI_GCS2Support_SRCS += PIC702Controller.cpp
PI_GCS2Support_SRCS += translateerror.c

PI_GCS2Support_LIBS += motor
PI_GCS2Support_LIBS += asyn
PI_GCS2Support_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

