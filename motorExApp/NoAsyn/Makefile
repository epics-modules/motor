# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += NoMPF.dbd

# !!WARNING!! If you copy this file to your
# <ioctop> application, the following line
# MUST BE REMOVED and a MOTOR entry must be
# in your <ioctop>/config/RELEASE file.
MOTOR_LIB = $(TOP)/lib/$(T_A)

PROD_IOC_vxWorks = NoMPF

NoMPF_SRCS += NoMPF_registerRecordDeviceDriver.cpp
NoMPF_SRCS_DEFAULT += NoMPFMain.cpp
NoMPF_SRCS_vxWorks += -nil-

#The following adds support from base/src/vxWorks
NoMPF_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

NoMPF_LIBS += recIoc
NoMPF_LIBS += softDevIoc
NoMPF_LIBS += testDevIoc
NoMPF_LIBS += iocsh
NoMPF_LIBS += miscIoc
NoMPF_LIBS += rsrvIoc
NoMPF_LIBS += dbtoolsIoc
NoMPF_LIBS += asIoc
NoMPF_LIBS += dbIoc
NoMPF_LIBS += registryIoc
NoMPF_LIBS += dbStaticIoc
NoMPF_LIBS += ca
NoMPF_LIBS += Com

NoMPF_LIBS         += motor
NoMPF_LIBS_vxWorks += oms
NoMPF_LIBS         += softMotor
motor_DIR          = $(MOTOR_LIB)
oms_DIR            = $(MOTOR_LIB)
softMotor_DIR      = $(MOTOR_LIB)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

