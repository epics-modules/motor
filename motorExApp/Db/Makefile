# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DB += NoMPF.db
DB += WithMPF.db

#===========================
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


# !!WARNING!! Delete the following line after coping
#	this file to your application.
MOTOR = $(TOP)

#Cancel implicit rule.
$(COMMON_DIR)/%.db$(RAW): %.substitutions %.template

#Replace rule with rule without %.template dependency.
$(COMMON_DIR)/%.db$(RAW): %.substitutions
	@$(RM) $@$(DEP)
	@$(DBDDEPENDS_CMD)
	@echo "Inflating database from $<"
	@$(RM) $@
	$(MSI) -V -I $(MOTOR)/db -S $^ > $@

