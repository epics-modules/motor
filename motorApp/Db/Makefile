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
DB += motor.db
DB += basic_motor.db
DB += basic_asyn_motor.db
DB += XPS_extra.db
DB += ACRAux.template
DB += ACRAuxLi.template
DB += ACRAuxBi.template
DB += ACRAuxBo.template
DB += ACRAuxBoRBV.template
DB += TransPos.db
DB += motorUtil.db
DB += asyn_motor.db

#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += motor.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
