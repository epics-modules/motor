# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CXXFLAGS += -DDEBUG

DBD += devImsMotor.dbd

LIBRARY_IOC = Ims

# Intelligent Motion Systems driver support.
SRCS += devIM483SM.c drvIM483SM.c
SRCS += devIM483PL.c drvIM483PL.c

OBJS_DEFAULT += ImsRegister
OBJS_vxWorks += -nil-

Ims_LIBS += motor motorCOM_mpf
Ims_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

