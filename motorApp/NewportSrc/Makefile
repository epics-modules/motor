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
Newport_SRCS += MM4KMotorDriver.cpp
ifdef SNCSEQ
Newport_SRCS += MM4005_trajectoryScan.st
endif

# PM500 device driver.
Newport_SRCS += devPM500.cc  drvPM500.cc

# ESP300 device driver.
Newport_SRCS += devESP300.cc drvESP300.cc

# XPS C8 device driver
Newport_SRCS += asynOctetSocket.cpp 
Newport_SRCS += XPS_C8_drivers.cpp 
Newport_SRCS += drvXPSAsynAux.c
Newport_SRCS += xps_ftp.c
# This is the model 2 asyn driver
#Newport_SRCS += drvXPSAsyn.c XPSAsynInterpose.c
# This is the model 3 asyn driver
#!Newport_SRCS += XPSController.cpp
#!Newport_SRCS += XPSAxis.cpp
ifdef SNCSEQ
Newport_SRCS += XPS_trajectoryScan.st
endif

# strtok_r needed on WIN32
Newport_SRCS_WIN32 += strtok_r.c

# XPS Gathering test function
Newport_SRCS += XPSGathering.c
Newport_SRCS += XPSGatheringRegister.c

# Function which will run a tcl script on an XPS
Newport_SRCS += tclCall.cc

Newport_LIBS += motor asyn
ifdef SNCSEQ
Newport_LIBS += seq pv
endif
Newport_LIBS += $(EPICS_BASE_IOC_LIBS)
Newport_SYS_LIBS_WIN32 += ws2_32

PROD_IOC += XPSGatheringMain
XPSGatheringMain_SRCS += XPSGatheringMain.c
XPSGatheringMain_LIBS += Newport motor asyn
ifdef SNCSEQ
XPSGatheringMain_LIBS += seq pv
endif
XPSGatheringMain_LIBS += $(EPICS_BASE_IOC_LIBS)
XPSGatheringMain_SYS_LIBS_solaris += socket nsl

PROD_IOC += XPSGathering2
XPSGathering2_SRCS += XPSGathering2.c
XPSGathering2_LIBS += Newport motor asyn
XPSGathering2_LIBS += $(EPICS_BASE_IOC_LIBS)
XPSGathering2_SYS_LIBS_solaris += socket nsl

include $(TOP)/configure/RULES

