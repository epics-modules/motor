# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += devNewport.dbd

LIBRARY_IOC = Newport

SRCS += NewportRegister.cc

# MM3000 (i.e., MM3000) device driver.
SRCS += devMM3000.cc drvMM3000.cc

# MM4000/5 (i.e., MM4000) device driver.
SRCS += devMM4000.cc drvMM4000.cc

# PM500 device driver.
SRCS += devPM500.cc  drvPM500.cc

# ESP300 device driver.
SRCS += devESP300.cc drvESP300.cc

Newport_LIBS += motor motorCOM_mpf
Newport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

