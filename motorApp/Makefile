TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

DIRS += MotorSrc

# Select/deselect individual device driver modules by removing/adding a
# "#" comment.

DIRS += SoftMotorSrc
SoftMotorSrc_DEPEND_DIRS = MotorSrc

DIRS += op
DIRS += Db
DIRS += iocsh

include $(TOP)/configure/RULES_DIRS
