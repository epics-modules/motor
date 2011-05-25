TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build HytecMotor Module Library
# -------------------------------

LIBRARY_IOC += HytecMotor

DBD += HytecMotorDriver.dbd

# The following are compiled and added to the support library
HytecMotor_SRCS += HytecMotorDriver.cpp

INC += HytecMotorDriver.h

# We need to link this IOC Application against the EPICS Base libraries
HytecMotor_LIBS += motor
HytecMotor_LIBS += asyn Ipac
HytecMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

include $(TOP)/configure/RULES
