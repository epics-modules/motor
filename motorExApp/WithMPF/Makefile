# The WithMPF IOC can be configured either as a MPF client in a two CPU board
# configuration, or as a single CPU board with both MPF client and server on
# the same CPU board.  Remove either the "# !MPF-1-CPU! #" or the
# "# !MPF-2-CPU! #" comment to configure.

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

# Support from ipac
WithMPFVx_LDOBJS_vxWorks += $(IPAC_BIN)/ipacLib
WithMPFVx_OBJS_vxWorks	 += $(IPAC_BIN)/tyGSOctal

WithMPF_LIBS		+= Acs
WithMPF_LIBS		+= Ims
WithMPF_LIBS		+= Newport
WithMPF_LIBS		+= motorCOM_mpf
WithMPF_LIBS		+= motor
WithMPF_LIBS		+= mpfSerial mpfEpics mpf
WithMPF_LIBS 		+= $(EPICS_BASE_IOC_LIBS)

WithMPFVx_LIBS		+= Acs
WithMPFVx_LIBS		+= Ims
WithMPFVx_LIBS		+= Newport
WithMPFVx_LIBS		+= motorCOM_mpf
WithMPFVx_LIBS		+= motor
WithMPFVx_LIBS		+= mpfSerial mpfEpics mpf
WithMPFVx_LIBS 		+= $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

