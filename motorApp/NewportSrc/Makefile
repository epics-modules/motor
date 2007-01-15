# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#
# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG

# Install the XPS header file in case applications in another module need to use it
INC += XPS_C8_drivers.h

DBD += devNewport.dbd

LIBRARY_IOC = Newport

SRCS += NewportRegister.cc

# MM3000 (i.e., MM3000) device driver.
SRCS += devMM3000.cc drvMM3000.cc

# MM4000/5 (i.e., MM4000) device driver.
SRCS += devMM4000.cc drvMM4000.cc
SRCS += drvMM4000Asyn.c
SRCS += MM4005_trajectoryScan.st

# PM500 device driver.
SRCS += devPM500.cc  drvPM500.cc

# ESP300 device driver.
SRCS += devESP300.cc drvESP300.cc

# XPS C8 device driver
SRCS += asynOctetSocket.cpp 
SRCS += XPS_C8_drivers.cpp drvXPSAsyn.c drvXPSAsynAux.c XPSAsynInterpose.c xps_ftp.c
SRCS += XPS_trajectoryScan.st

# XPS Gathering test function
SRCS += XPSGathering.c
SRCS += XPSGatheringRegister.c

# Function which will run a tcl script on an XPS
SRCS += tclCall.cc

Newport_LIBS += motor asyn seq pv
Newport_LIBS += $(EPICS_BASE_IOC_LIBS)

PROD_IOC += XPSGatheringMain
XPSGatheringMain_SRCS += XPSGatheringMain.c
XPSGatheringMain_LIBS += Newport motor asyn seq pv
XPSGatheringMain_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

