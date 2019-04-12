TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

DIRS += MotorSrc

# Select/deselect individual device driver modules by removing/adding a
# "#" comment.

DIRS += SoftMotorSrc
SoftMotorSrc_DEPEND_DIRS = MotorSrc

# All the following modules require ASYN.
ifdef ASYN

DIRS += MotorSimSrc
MotorSimSrc_DEPEND_DIRS = MotorSrc

DIRS += PC6KSrc
PC6KSrc_DEPEND_DIRS = MotorSrc

endif

# Install the edl files
#DIRS += opi

ifdef MX
DIRS += MXmotorSrc
endif

DIRS += Db

include $(TOP)/configure/RULES_DIRS
