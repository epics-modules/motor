# "#!" marks lines that can be uncommented.
# Choice either IPAC R2.5 and below, or, R2.6 
# and above. The default is IPAC R2.6 and above.

# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

ifeq ($(OS_CLASS), vxWorks)
DBD += WithAsynVx.dbd
endif
DBD += WithAsyn.dbd

PROD_IOC_DEFAULT = WithAsyn
PROD_IOC_vxWorks = WithAsynVx

WithAsyn_SRCS   += WithAsyn_registerRecordDeviceDriver.cpp
WithAsynVx_SRCS += WithAsynVx_registerRecordDeviceDriver.cpp
WithAsyn_SRCS_DEFAULT += WithAsynMain.cpp

# Support from EPICS base
WithAsyn_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Support from IPAC R2.5 and below.
#!WithAsynVx_LDOBJS_vxWorks += $(IPAC_BIN)/ipacLib
#!WithAsynVx_OBJS_vxWorks   += $(IPAC_BIN)/tyGSOctal

#!WithAsyn_LIBS   += oms
#!WithAsyn_LIBS   += Mclennan
#!WithAsyn_LIBS   += Micos
#!WithAsyn_LIBS   += MicroMo
#!WithAsyn_LIBS   += PI
#!WithAsyn_LIBS   += Acs
#!WithAsyn_LIBS   += Ims
#!WithAsyn_LIBS   += Newport
#!WithAsyn_LIBS   += softMotor
#!WithAsyn_LIBS   += motorSimSupport
#!WithAsyn_LIBS   += SmartMotor
#!WithAsyn_LIBS   += KohzuMotor
WithAsyn_LIBS   += motor
WithAsyn_LIBS   += asyn
# Needed for Newport SNL programs
#!WithAsyn_LIBS   += seq pv
WithAsyn_LIBS   += $(EPICS_BASE_IOC_LIBS)

#!WithAsynVx_LIBS += oms
#!WithAsynVx_LIBS += Mclennan
#!WithAsynVx_LIBS += Micos
#!WithAsynVx_LIBS += MicroMo
#!WithAsynVx_LIBS += PI
#!WithAsynVx_LIBS += Acs
#!WithAsynVx_LIBS += Ims
#!WithAsynVx_LIBS += Newport
#!WithAsynVx_LIBS += softMotor
#!WithAsynVx_LIBS += motorSimSupport
#!WithAsynVx_LIBS += SmartMotor
#!WithAsynVx_LIBS += KohzuMotor
WithAsynVx_LIBS += motor

# Support from IPAC R2.6 and above.
WithAsynVx_LIBS += Ipac TyGSOctal

WithAsynVx_LIBS += asyn
# Needed for Newport SNL programs
#!WithAsynVx_LIBS   += seq pv
WithAsynVx_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

