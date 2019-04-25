TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) dbd files
# <name>.dbd will be created from <name>Include.dbd
#DBD += motor.dbd

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB += asyn_auto_power.db
DB += asyn_motor.db
DB += asyn_motor_positions.req
DB += asyn_motor_settings.req
DB += basic_asyn_motor.db
DB += basic_motor.db
DB += basic_motor_settings.req
DB += coordTrans2D.db
DB += motor.db
DB += motor_positions.req
DB += motor_settings.req
DB += motorUtil.db
DB += profileMoveAxis.template
DB += profileMoveAxis_settings.req
DB += profileMoveController.template
DB += profileMoveController_settings.req
DB += pseudoMotor.db
DB += pseudoMotor_settings.req
DB += SoftMotorEx.db
DB += softMotorTest.db
DB += sumDiff2D.db
DB += trajectoryScan.db
DB += trajectoryScan_settings.req
DB += TransPos.db

#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += motor.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

ifeq ($(BASE_3_14),YES)
# Only needed for 3.14 build rules:
vpath %.req $(USR_VPATH) $(GENERIC_SRC_DIRS)
endif
