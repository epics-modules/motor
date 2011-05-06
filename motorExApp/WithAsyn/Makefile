#FILENAME...     Makefile
#USAGE...        Makefile for motor application example with Asyn

#Version:        $Revision$
#Modified By:    $Author$
#Last Modified:  $Date$
#HeadURL:        $URL$

# "#!" marks lines that can be uncommented.

TOP = ../..
include $(TOP)/configure/CONFIG

PROD_IOC_DEFAULT = WithAsyn
PROD_IOC_vxWorks = WithAsynVx

COMMONDBDS  = base.dbd
COMMONDBDS += motorSupport.dbd
COMMONDBDS += devAcsMotor.dbd
COMMONDBDS += devImsMotor.dbd
COMMONDBDS += devMclennanMotor.dbd
COMMONDBDS += devMicos.dbd
COMMONDBDS += devMicroMo.dbd
COMMONDBDS += devNewport.dbd
ifdef SNCSEQ
COMMONDBDS += LdevNewportTS.dbd
endif
COMMONDBDS += devPIMotor.dbd
COMMONDBDS += devOms.dbd
COMMONDBDS += devSoftMotor.dbd
#!COMMONDBDS += motorSimSupport.dbd
COMMONDBDS += devSmartMotorMotor.dbd
COMMONDBDS += devKohzuMotor.dbd
COMMONDBDS += devAttocube.dbd
COMMONDBDS += devAerotech.dbd
COMMONDBDS += drvAsynSerialPort.dbd
COMMONDBDS += drvAsynIPPort.dbd
#!COMMONDBDS += ACRMotorSupport.dbd

DBD += WithAsyn.dbd
WithAsyn_DBD += $(COMMONDBDS)
ifdef SNCSEQ
WithAsyn_DBD += devSequencer.dbd
endif

DBD += WithAsynVx.dbd
WithAsynVx_DBD += $(COMMONDBDS)
#!WithAsynVx_DBD += drvGsIP488.dbd
ifdef IPAC
WithAsynVx_DBD += MiscVx.dbd
endif

WithAsyn_SRCS   += WithAsyn_registerRecordDeviceDriver.cpp
WithAsyn_SRCS_DEFAULT += WithAsynMain.cpp
WithAsynVx_SRCS += WithAsynVx_registerRecordDeviceDriver.cpp

WithAsyn_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

COMMONLIBS += Aerotech
COMMONLIBS += Attocube
COMMONLIBS += oms
COMMONLIBS += Mclennan
COMMONLIBS += Micos
COMMONLIBS += MicroMo
COMMONLIBS += PI
COMMONLIBS += Acs
COMMONLIBS += Ims
COMMONLIBS += Newport
COMMONLIBS += softMotor
#!COMMONLIBS += motorSimSupport
COMMONLIBS += SmartMotor
COMMONLIBS += KohzuMotor
#!COMMONLIBS += ACRMotor
COMMONLIBS += motor

# Needed for Newport SNL programs
WithAsyn_LIBS += $(COMMONLIBS)
WithAsyn_LIBS += asyn
ifdef SNCSEQ
WithAsyn_LIBS += seqDev seq pv
endif
WithAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

WithAsynVx_LIBS += $(COMMONLIBS)
ifdef IPAC
WithAsynVx_LIBS += Ipac TyGSOctal
endif
WithAsynVx_LIBS += asyn
# Needed for Newport SNL programs
ifdef SNCSEQ
WithAsynVx_LIBS   += seqDev seq pv
endif
WithAsynVx_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

