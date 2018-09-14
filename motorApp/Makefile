TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

DIRS += MotorSrc

# Select/deselect individual device driver modules by removing/adding a
# "#" comment.

ifndef NO_MOTOR_DELTATAUSRC
DIRS += DeltaTauSrc
DeltaTauSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_OMSSRC
DIRS += OmsSrc
OmsSrc_DEPEND_DIRS   = MotorSrc
endif

DIRS += SoftMotorSrc
SoftMotorSrc_DEPEND_DIRS = MotorSrc

# All the following modules require ASYN.
ifdef ASYN

ifndef NO_MOTOR_OMSASYNSRC
DIRS += OmsAsynSrc
OmsAsynSrc_DEPEND_DIRS   = MotorSrc
endif

DIRS += MotorSimSrc
MotorSimSrc_DEPEND_DIRS = MotorSrc

ifndef NO_MOTOR_NEWPORTSRC
DIRS += NewportSrc
NewportSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_IMSSRC
DIRS += ImsSrc
ImsSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_ACSSRC
DIRS += AcsSrc
AcsSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_MCLENNANSRC
DIRS += MclennanSrc
MclennanSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_PISRC
DIRS += PiSrc
PiSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_PIGCS2SRC
DIRS += PIGCS2Src
PIGCS2Src_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_MICROMOSRC
DIRS += MicroMoSrc
MicroMoSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_MICOSSRC
DIRS += MicosSrc
MicosSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_FAULHABERSRC
DIRS += FaulhaberSrc
FaulhaberSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_PC6KSRC
DIRS += PC6KSrc
PC6KSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_NEWFOCUSSRC
DIRS += NewFocusSrc
NewFocusSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_ACSTECH80SRC
DIRS += AcsTech80Src
AcsTech80Src_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_ORIELSRC
DIRS += OrielSrc
OrielSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_THORLABSSRC
DIRS += ThorLabsSrc
ThorLabsSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_SMARTMOTORSRC
DIRS += SmartMotorSrc
SmartMotorSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_PIJENASRC
DIRS += PiJenaSrc
PiJenaSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_KOHZUSRC
DIRS += KohzuSrc
KohzuSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_ATTOCUBESRC
DIRS += AttocubeSrc
AttocubeSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_AEROTECHSRC
DIRS += AerotechSrc
AerotechSrc_DEPEND_DIRS = MotorSrc
endif

# The Hytec support requires ipac
ifdef IPAC
ifndef NO_MOTOR_HYTECSRC
DIRS += HytecSrc
HytecSrc_DEPEND_DIRS = MotorSrc
endif
endif

ifndef NO_MOTOR_ACRSRC
DIRS += ACRSrc
ACRSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_SMARACTMCSSRC
DIRS += SmarActMCSSrc
SmarActMCSSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_NPOINTSRC
DIRS += NPointSrc
NPointSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_MICRONIXSRC
DIRS += MicronixSrc
MicronixSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_PHYTRONSRC
DIRS += PhytronSrc
PhytronSrc_DEPEND_DIRS = MotorSrc
endif

ifndef NO_MOTOR_AMCISRC
DIRS += AMCISrc
AMCISrc_DEPEND_DIRS = MotorSrc
endif

# The Script motor support requires the lua scripting module
ifdef LUA
DIRS += ScriptMotorSrc
ScriptMotorSrc_DEPEND_DIRS = MotorSrc
endif

endif

# Install the edl files
#DIRS += opi

ifdef MX
ifndef NO_MOTOR_MXMOTORSRC
DIRS += MXmotorSrc
endif
endif

DIRS += Db

include $(TOP)/configure/RULES_DIRS
