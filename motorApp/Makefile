TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.
DIRS += MotorSrc

# 
# Select individual device/driver support modules by removing the comment.
DIRS += OmsSrc
DIRS += SoftMotorSrc
#DIRS += V544Src

ifdef MX
DIRS += MXmotorSrc
endif

# The following modules require ASYN.
# Select individual device/driver support modules by removing the comment.
ifdef ASYN
#DIRS += NewportSrc
#DIRS += ImsSrc
#DIRS += AcsSrc
#DIRS += MclennanSrc
#DIRS += PiSrc
#DIRS += MicroMoSrc
#DIRS += MicosSrc
endif

DIRS += Db
include $(TOP)/configure/RULES_DIRS
