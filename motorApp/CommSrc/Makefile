# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

# The following are used for debugging messages.
#USR_CFLAGS += -DDEBUG
#USR_CXXFLAGS += -DDEBUG

INC += gpibIO.h serialIO.h

LIBRARY_IOC = motorCOM_mpf

# If the EPICS GPIB Module from Benjamin Franksen is used,
# then remove the '#' from the following and add GPIB to
# the config/RELEASE file .  If GPIB driver support is from
# Marty Kraimer's mpfSerial, then leave this line commented out.
#USR_CFLAGS += -DGPIB

ifdef MPF_GPIB
SRCS += gpibIO.c
endif

# The following is for device drivers that need serial
# communication support via MPF.  Define MPF in
# <ioctop>/config/RELEASE.

ifdef MPF
SRCS += serialIOMPF.cc
endif

include $(TOP)/configure/RULES

