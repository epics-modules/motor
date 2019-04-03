#Makefile at top of application tree
# "#!" marks lines that can be uncommented.

TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure motorApp
motorApp_DEPEND_DIRS   = configure

DIRS += modules
modules_DEPEND_DIRS = motorApp

# To build motor examples;
# 1st - uncomment lines below.
# 2nd - uncomment required support module lines at the bottom of
#       <motor>/configure/RELEASE.
# 3rd - make clean uninstall
# 4th - make

#!DIRS += motorExApp iocBoot
#!motorExApp_DEPEND_DIRS = motorApp
#!iocBoot_DEPEND_DIRS    = motorExApp

include $(TOP)/configure/RULES_TOP
