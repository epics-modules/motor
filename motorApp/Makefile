TOP = ..
include $(TOP)/config/CONFIG_APP

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.
DIRS += MotorSrc

# If any motor controller communication mechanism besides the VME backplane is
# required, then the "CommSrc" module must be built.  MPF_SERIAL must be defined
# in <ioctop>/config/RELEASE.
ifdef MPF_SERIAL
DIRS += CommSrc
endif

# 
# Select individual device/driver support modules by removing the comment.
#DIRS += OmsSrc
#DIRS += SoftMotorSrc
#DIRS += V544Src

# The following modules require MPF_SERIAL.
# Select individual device/driver support modules by removing the comment.
ifdef MPF_SERIAL
#DIRS += NewportSrc
#DIRS += ImsSrc
endif

DIRS += Db
include $(TOP)/config/RULES_DIRS
