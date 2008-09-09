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

Newport_SRCS += NewportRegister.cc

# MM3000 (i.e., MM3000) device driver.
Newport_SRCS += devMM3000.cc drvMM3000.cc

# MM4000/5 (i.e., MM4000) device driver.
Newport_SRCS += devMM4000.cc drvMM4000.cc
Newport_SRCS += drvMM4000Asyn.c
Newport_SRCS += MM4005_trajectoryScan.st

# PM500 device driver.
Newport_SRCS += devPM500.cc  drvPM500.cc

# ESP300 device driver.
Newport_SRCS += devESP300.cc drvESP300.cc

# XPS C8 device driver
Newport_SRCS_DEFAULT += asynOctetSocket.cpp 
Newport_SRCS_DEFAULT += XPS_C8_drivers.cpp drvXPSAsyn.c drvXPSAsynAux.c XPSAsynInterpose.c xps_ftp.c
Newport_SRCS_DEFAULT += XPS_trajectoryScan.st

# strtok_r needed on WIN32
Newport_SRCS_WIN32 += strtok_r.cpp

# XPS Gathering test function
Newport_SRCS_DEFAULT += XPSGathering.c
Newport_SRCS_DEFAULT += XPSGatheringRegister.c

# Function which will run a tcl script on an XPS
Newport_SRCS += tclCall.cc

Newport_LIBS += motor asyn seq pv
Newport_LIBS += $(EPICS_BASE_IOC_LIBS)

PROD_IOC_DEFAULT += XPSGatheringMain
XPSGatheringMain_SRCS_DEFAULT += XPSGatheringMain.c
XPSGatheringMain_LIBS_DEFAULT += Newport motor asyn seq pv
XPSGatheringMain_LIBS_DEFAULT += $(EPICS_BASE_IOC_LIBS)
XPSGatheringMain_SYS_LIBS_solaris += socket nsl

# Build in all non-DEFAULT files for RTEMS
Newport_SRCS_RTEMS += -nil-
PROD_IOC_RTEMS += -nil-

include $(TOP)/configure/RULES

