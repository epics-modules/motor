# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# 
#USR_CXXFLAGS += -DDEBUG

DBD += omsAsynSupport.dbd

LIBRARY_IOC_DEFAULT += omsAsyn

SRCS += omsBaseAxis.cpp
SRCS += omsBaseController.cpp
SRCS += omsMAXnet.cpp
SRCS += omsMAXv.cpp
#SRCS += omsMAXvEncFunc.cpp

omsAsyn_LIBS += motor
omsAsyn_LIBS += asyn
omsAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
