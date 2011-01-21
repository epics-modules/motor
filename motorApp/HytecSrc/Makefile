TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build Hy8601 Module Library
# -------------------------------

ifdef IPAC

LIBRARY_IOC += Hy8601

DBD += drvHy8601Asyn.dbd

# The following are compiled and added to the support library
Hy8601_SRCS += drvHy8601Asyn.c
Hy8601_SRCS += registrarHy8601Asyn.c

# We need to link this IOC Application against the EPICS Base libraries
Hy8601_LIBS += motor asyn Ipac
Hy8601_LIBS += $(EPICS_BASE_IOC_LIBS)

endif

# ---------------------------------------------------

include $(TOP)/configure/RULES
