# "#!" marks lines that can be uncommented.
# Choice either IPAC R2.5 and below, or, R2.6 
# and above. The default is IPAC R2.6 and above.

# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

DBD += WithMPF.dbd
DBD += WithMPFVx.dbd

PROD_IOC_DEFAULT = WithMPF
PROD_IOC_vxWorks = WithMPFVx

WithMPF_SRCS   += WithMPF_registerRecordDeviceDriver.cpp
WithMPFVx_SRCS += WithMPFVx_registerRecordDeviceDriver.cpp
WithMPF_SRCS_DEFAULT += WithMPFMain.cpp

# Support from EPICS base
WithMPF_OBJS_vxWorks	+= $(EPICS_BASE_BIN)/vxComLibrary

# Support from IPAC R2.5 and below.
#!WithMPFVx_LDOBJS_vxWorks += $(IPAC_BIN)/ipacLib
#!WithMPFVx_OBJS_vxWorks   += $(IPAC_BIN)/tyGSOctal

WithMPF_LIBS		+= PI
WithMPF_LIBS		+= Acs
WithMPF_LIBS		+= Ims
WithMPF_LIBS		+= Newport
WithMPF_LIBS		+= motorCOM_mpf
WithMPF_LIBS		+= motor
WithMPF_LIBS		+= mpfSerial mpfEpics mpf
WithMPF_LIBS 		+= $(EPICS_BASE_IOC_LIBS)

WithMPFVx_LIBS		+= PI
WithMPFVx_LIBS		+= Acs
WithMPFVx_LIBS		+= Ims
WithMPFVx_LIBS		+= Newport
WithMPFVx_LIBS		+= motorCOM_mpf
WithMPFVx_LIBS		+= motor

# Support from IPAC R2.6 and above.
WithMPFVx_LIBS 		+= Ipac TyGSOctal

WithMPFVx_LIBS		+= mpfSerial mpfEpics mpf
WithMPFVx_LIBS 		+= $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

