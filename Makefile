#Makefile at top of application tree
# "#!" marks lines that can be uncommented.
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure motorApp
motorApp_DEPEND_DIRS   = configure

#!DIRS += motorExApp iocBoot
#!motorExApp_DEPEND_DIRS = motorApp
#!iocBoot_DEPEND_DIRS    = motorExApp

include $(TOP)/configure/RULES_TOP
