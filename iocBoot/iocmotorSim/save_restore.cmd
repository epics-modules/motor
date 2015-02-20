
# BEGIN save_restore.cmd ------------------------------------------------------

### save_restore setup
#
# This file does not require modification for standard use, but...

# status PVs
#save_restoreSet_UseStatusPVs(1)
save_restoreSet_status_prefix("$(MYPVPREFIX)")
dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=$(MYPVPREFIX), DEAD_SECONDS=5")

# Ok to save/restore save sets with missing values (no CA connection to PV)?
save_restoreSet_IncompleteSetsOk(1)

# Save dated backup files?
save_restoreSet_DatedBackupFiles(1)

# Number of sequenced backup files to write
save_restoreSet_NumSeqFiles(3)

# Time interval between sequenced backups
save_restoreSet_SeqPeriodInSeconds(300)

# specify where save files should be
set_savefile_path("$(TOP)/iocBoot/$(IOC)", "autosave")

###
# specify what save files should be restored.  Note these files must be
# in the directory specified in set_savefile_path(), or, if that function
# has not been called, from the directory current when iocInit is invoked
set_pass0_restoreFile("auto_positions.sav")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")

# Note that you can restore a .sav file without also autosaving to it.
#set_pass0_restoreFile("myInitData.sav")
#set_pass1_restoreFile("myInitData.sav")

###
# specify directories in which to to search for included request files
set_requestfile_path("$(TOP)/iocBoot/$(IOC)", "")
set_requestfile_path("$(TOP)/iocBoot/$(IOC)", "autosave")
set_requestfile_path("$(AREA_DETECTOR)", "ADApp/Db")
set_requestfile_path("$(AUTOSAVE)", "asApp/Db")
set_requestfile_path("$(CALC)", "calcApp/Db")
set_requestfile_path("$(CAMAC)", "camacApp/Db")
set_requestfile_path("$(DAC128V)", "dac128VApp/Db")
set_requestfile_path("$(DXP)", "dxpApp/Db")
set_requestfile_path("$(IP)", "ipApp/Db")
set_requestfile_path("$(IP330)", "ip330App/Db")
set_requestfile_path("$(IPUNIDIG)", "ipUnidigApp/Db")
set_requestfile_path("$(LOVE)", "loveApp/Db")
set_requestfile_path("$(MCA)", "mcaApp/Db")
set_requestfile_path("$(MODBUS)", "modbusApp/Db")
set_requestfile_path("$(MOTOR)", "motorApp/Db")
set_requestfile_path("$(OPTICS)", "opticsApp/Db")
set_requestfile_path("$(QUADEM)", "quadEMApp/Db")
set_requestfile_path("$(SSCAN)", "sscanApp/Db")
set_requestfile_path("$(STD)", "stdApp/Db")
set_requestfile_path("$(VAC)", "vacApp/Db")
set_requestfile_path("$(VME)", "vmeApp/Db")
set_requestfile_path("$(TOP)", "xxxApp/Db")

# Debug-output level
save_restoreSet_Debug(0)

# END save_restore.cmd --------------------------------------------------------
