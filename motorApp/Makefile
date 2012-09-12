TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

DIRS += MotorSrc

# Select/deselect individual device driver modules by removing/adding a
# "#" comment.

DIRS += DeltaTauSrc
DeltaTauSrc_DEPEND_DIRS = MotorSrc

DIRS += OmsSrc
OmsSrc_DEPEND_DIRS   = MotorSrc

DIRS += SoftMotorSrc
SoftMotorSrc_DEPEND_DIRS = MotorSrc

# All the following modules require ASYN.
ifdef ASYN

DIRS += MotorSimSrc
MotorSimSrc_DEPEND_DIRS = MotorSrc

DIRS += NewportSrc
NewportSrc_DEPEND_DIRS = MotorSrc

DIRS += ImsSrc
ImsSrc_DEPEND_DIRS = MotorSrc

DIRS += AcsSrc
AcsSrc_DEPEND_DIRS = MotorSrc

DIRS += MclennanSrc
MclennanSrc_DEPEND_DIRS = MotorSrc

DIRS += PiSrc
PiSrc_DEPEND_DIRS = MotorSrc

DIRS += PIGCS2Src
PIGCS2Src_DEPEND_DIRS = MotorSrc

DIRS += MicroMoSrc
MicroMoSrc_DEPEND_DIRS = MotorSrc

DIRS += MicosSrc
MicosSrc_DEPEND_DIRS = MotorSrc

DIRS += FaulhaberSrc
FaulhaberSrc_DEPEND_DIRS = MotorSrc

DIRS += PC6KSrc
PC6KSrc_DEPEND_DIRS = MotorSrc

DIRS += NewFocusSrc
NewFocusSrc_DEPEND_DIRS = MotorSrc

DIRS += AcsTech80Src
AcsTech80Src_DEPEND_DIRS = MotorSrc

DIRS += OrielSrc
OrielSrc_DEPEND_DIRS = MotorSrc

DIRS += ThorLabsSrc
ThorLabsSrc_DEPEND_DIRS = MotorSrc

DIRS += SmartMotorSrc
SmartMotorSrc_DEPEND_DIRS = MotorSrc

DIRS += PiJenaSrc
PiJenaSrc_DEPEND_DIRS = MotorSrc

DIRS += KohzuSrc
KohzuSrc_DEPEND_DIRS = MotorSrc

DIRS += AttocubeSrc
AttocubeSrc_DEPEND_DIRS = MotorSrc

DIRS += AerotechSrc
AerotechSrc_DEPEND_DIRS = MotorSrc

DIRS += HytecSrc
HytecSrc_DEPEND_DIRS = MotorSrc

DIRS += ACRSrc
ACRSrc_DEPEND_DIRS = MotorSrc

DIRS += SmarActMCSSrc
SmarActMCSSrc_DEPEND_DIRS = MotorSrc

endif

# Install the edl files
#DIRS += opi

ifdef MX
DIRS += MXmotorSrc
endif

DIRS += Db

include $(TOP)/configure/RULES_DIRS
