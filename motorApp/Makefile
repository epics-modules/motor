TOP = ..
include $(TOP)/config/CONFIG_APP

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.
DIRS += MotorSrc

# If any motor controller communication mechanism besides the VME backplane is
# required, then the following module must be uncommented.  In addition, it
# must appear 2nd in this list of directories.
#DIRS += CommSrc

# 
# Individual device/driver support modules may be commented
# out here.
#DIRS += NewportSrc
#DIRS += OmsSrc
#DIRS += SoftMotorSrc
#DIRS += V544Src
#DIRS += ImsSrc

DIRS += Db
include $(TOP)/config/RULES_DIRS
