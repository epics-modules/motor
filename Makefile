#Makefile at top of application tree
# "#!" marks lines that can be uncommented.

TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure motorApp
motorApp_DEPEND_DIRS   = configure

DIRS += modules
modules_DEPEND_DIRS = motorApp

include $(TOP)/configure/RULES_TOP
