# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#
# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

DBD += devNewport.dbd

LIBRARY_IOC = Newport

SRCS += NewportRegister.cc

# MM3000 (i.e., MM3000) device driver.
SRCS += devMM3000.cc drvMM3000.cc

# MM4000/5 (i.e., MM4000) device driver.
SRCS += devMM4000.cc drvMM4000.cc

# PM500 device driver.
SRCS += devPM500.cc  drvPM500.cc

# ESP300 device driver.
SRCS += devESP300.cc drvESP300.cc

# XPS C8 device driver
#SRCS += Socket.cpp 
SRCS += asynOctetSocket.cpp xps_c8_driver.cpp devXPSC8.cc  drvXPSC8.cc

# XPS Gathering Test C script function -> int xpsgathering(void)
SRCS_vxWorks += XPSGathering.cc

# Function which will run a tcl script on an XPS
SRCS += tclCall.cc

Newport_LIBS += motor asyn
Newport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

