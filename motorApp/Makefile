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

DIRS += OmsAsynSrc
OmsAsynSrc_DEPEND_DIRS   = MotorSrc

DIRS += MotorSimSrc
MotorSimSrc_DEPEND_DIRS = MotorSrc

DIRS += ImsSrc
ImsSrc_DEPEND_DIRS = MotorSrc

DIRS += PIGCS2Src
PIGCS2Src_DEPEND_DIRS = MotorSrc

DIRS += MicosSrc
MicosSrc_DEPEND_DIRS = MotorSrc

DIRS += PC6KSrc
PC6KSrc_DEPEND_DIRS = MotorSrc

DIRS += NewFocusSrc
NewFocusSrc_DEPEND_DIRS = MotorSrc

DIRS += AerotechSrc
AerotechSrc_DEPEND_DIRS = MotorSrc

DIRS += ACRSrc
ACRSrc_DEPEND_DIRS = MotorSrc

DIRS += SmarActMCSSrc
SmarActMCSSrc_DEPEND_DIRS = MotorSrc

DIRS += PhytronSrc
PhytronSrc_DEPEND_DIRS = MotorSrc

DIRS += AMCISrc
AMCISrc_DEPEND_DIRS = MotorSrc

# The Script motor support requires the lua scripting module
ifdef LUA
DIRS += ScriptMotorSrc
ScriptMotorSrc_DEPEND_DIRS = MotorSrc
endif

endif

# Install the edl files
#DIRS += opi

ifdef MX
DIRS += MXmotorSrc
endif

DIRS += Db

include $(TOP)/configure/RULES_DIRS
