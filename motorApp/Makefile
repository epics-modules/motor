TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

DIRS += MotorSrc

# Select individual device/driver modules by removing the "#!" comment.

#!DIRS += DeltaTauSrc
#!DIRS += OmsSrc
#!DIRS += SoftMotorSrc
#!DIRS += V544Src

# All the following modules require ASYN.
ifdef ASYN
#!DIRS += NewportSrc
#!DIRS += ImsSrc
#!DIRS += AcsSrc
#!DIRS += MclennanSrc
#!DIRS += PiSrc
#!DIRS += MicroMoSrc
#!DIRS += MicosSrc
endif

ifdef MX
DIRS += MXmotorSrc
endif

DIRS += Db
include $(TOP)/configure/RULES_DIRS
