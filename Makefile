#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), configure)
DIRS := $(DIRS) $(filter-out $(DIRS), motorApp)
DIRS := $(DIRS) $(filter-out $(DIRS), motorExApp)
DIRS := $(DIRS) $(filter-out $(DIRS), iocBoot)
include $(TOP)/configure/RULES_TOP
