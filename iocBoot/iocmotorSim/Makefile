TOP = ../..
include $(TOP)/configure/CONFIG
ARCH = $(EPICS_HOST_ARCH)
#ARCH = linux-x86_64
#ARCH = vxWorks-ppc604
TARGETS = cdCommands
TARGETS += envPaths dllPath.bat runIOC.bat
TARGETS += motor.substitutions.local

motor.substitutions.local : motor.substitutions
	$(MSI) -M "IOC=$(MYPVPREFIX)" -o $@ $<

include $(TOP)/configure/RULES.ioc

