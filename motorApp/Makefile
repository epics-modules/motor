TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

# "#!" marks lines that can be uncommented.

DIRS += MotorSrc

# Select individual device/driver modules by removing the "#!" comment.
DIRS += DeltaTauSrc
DIRS += OmsSrc
DIRS += SoftMotorSrc
DIRS += MotorSimSrc

# All the following modules require ASYN.
ifdef ASYN
DIRS += NewportSrc
DIRS += ImsSrc
DIRS += AcsSrc
DIRS += MclennanSrc
DIRS += PiSrc
DIRS += MicroMoSrc
DIRS += MicosSrc
DIRS += FaulhaberSrc
DIRS += PC6KSrc
DIRS += NewFocusSrc
DIRS += AcsTech80Src
DIRS += OrielSrc
DIRS += ThorLabsSrc
DIRS += SmartMotorSrc
DIRS += PiJenaSrc
endif

# Install the edl files
#DIRS += opi

ifdef MX
DIRS += MXmotorSrc
endif

DIRS += Db
include $(TOP)/configure/RULES_DIRS
