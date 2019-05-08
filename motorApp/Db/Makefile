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
DB += $(patsubst ../%, %, $(wildcard ../*.template))
DB += $(patsubst ../%, %, $(wildcard ../*.db))
DB += $(patsubst ../%, %, $(wildcard ../*.vdb))

REQ += $(patsubst ../%, %, $(wildcard ../*.req))

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
