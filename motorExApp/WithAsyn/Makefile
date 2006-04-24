# "#!" marks lines that can be uncommented.
# Choice either IPAC R2.5 and below, or, R2.6 
# and above. The default is IPAC R2.6 and above.

# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

ifeq ($(OS_CLASS), vxWorks)
DBD += WithMPFVx.dbd
endif
ifeq ($(OS_CLASS), WIN32)
DBD += WithMPFWin32.dbd
else
DBD += WithMPF.dbd
endif

PROD_IOC_DEFAULT = WithMPF
PROD_IOC_vxWorks = WithMPFVx

ifeq ($(OS_CLASS), WIN32) 
WithMPF_SRCS   += WithMPFWin32_registerRecordDeviceDriver.cpp
else
WithMPF_SRCS   += WithMPF_registerRecordDeviceDriver.cpp
endif
WithMPFVx_SRCS += WithMPFVx_registerRecordDeviceDriver.cpp
WithMPF_SRCS_DEFAULT += WithMPFMain.cpp

# Support from EPICS base
WithMPF_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Support from IPAC R2.5 and below.
#!WithMPFVx_LDOBJS_vxWorks += $(IPAC_BIN)/ipacLib
#!WithMPFVx_OBJS_vxWorks   += $(IPAC_BIN)/tyGSOctal

#!WithMPF_LIBS   += oms
#!WithMPF_LIBS   += Mclennan
#!WithMPF_LIBS   += Micos
#!WithMPF_LIBS   += MicroMo
#!WithMPF_LIBS   += PI
#!WithMPF_LIBS   += Acs
#!WithMPF_LIBS   += Ims
#!WithMPF_LIBS   += Newport
#!WithMPF_LIBS   += softMotor
#!WithMPF_LIBS   += motorSimSupport
WithMPF_LIBS   += motor
WithMPF_LIBS   += asyn
# Needed for Newport SNL programs
#!WithMPF_LIBS   += seq pv
WithMPF_LIBS   += $(EPICS_BASE_IOC_LIBS)

#!WithMPFVx_LIBS += oms
#!WithMPFVx_LIBS += Mclennan
#!WithMPFVx_LIBS += Micos
#!WithMPFVx_LIBS += MicroMo
#!WithMPFVx_LIBS += PI
#!WithMPFVx_LIBS += Acs
#!WithMPFVx_LIBS += Ims
#!WithMPFVx_LIBS += Newport
#!WithMPFVx_LIBS += softMotor
#!WithMPFVx_LIBS += motorSimSupport
WithMPFVx_LIBS += motor

# Support from IPAC R2.6 and above.
WithMPFVx_LIBS += Ipac TyGSOctal

WithMPFVx_LIBS += asyn
# Needed for Newport SNL programs
#!WithMPFVx_LIBS   += seq pv
WithMPFVx_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

