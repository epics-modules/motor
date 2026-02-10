# Motor Releases

## __R7-4 (2026-02-TBD)__
R7-4 is a release based on the master branch.

### Changes since R7-3-1

#### New features
* TBD

#### Modifications to existing features
* TBD

#### Bug fixes
* TBD

#### Documentation
* TBD

#### Continuous Integration
* Upgraded ci-scripts to v3.4.1

#### Driver submodules (and noteworthy changes)

**Note:** A newer release of motorAttocube exists ([R1-1](https://github.com/epics-motor/motorAttocube/releases/tag/R1-1)), but it isn't included in this release because it requires EPICS base 7.0.3.1 or later.  It will be included in a future release when support for EPICS base 3.15 is dropped.

| Module             | Release | Changes |
| ------------------ | ------- | ------- |
| motorAcs           | [R1-1-1](https://github.com/epics-motor/motorAcs/releases/tag/R1-1-1) |         |
| **motorAcsMotion** | [R2-3-2](https://github.com/epics-motor/motorAcsMotion/releases/tag/R2-3-2) | Many bugfixes and new features since R2-2; see [RELEASE NOTES](https://github.com/epics-motor/motorAcsMotion/blob/0217e915adfafeb7983d3ef4529f7a77fd8fedb0/docs/RELEASE.md) |
| motorAcsTech80     | [R1-0-2](https://github.com/epics-motor/motorAcsTech80/releases/tag/R1-0-2) |         |
| motorAerotech      | [R1-1-1](https://github.com/epics-motor/motorAerotech/releases/tag/R1-1-1) |         |
| motorAMCI          | [R1-0-2](https://github.com/epics-motor/motorAMCI/releases/tag/R1-0-2) |         |
| motorAttocube      | [R1-0-2](https://github.com/epics-motor/motorAttocube/releases/tag/R1-0-2) | **See note above** |
| motorDeltaTau      | [R1-0-2](https://github.com/epics-motor/motorDeltaTau/releases/tag/R1-0-2) |         |
| motorFaulhaber     | [R1-0-2](https://github.com/epics-motor/motorFaulhaber/releases/tag/R1-0-2) |         |
| **motorHytec**     | [R1-0-4](https://github.com/epics-motor/motorHytec/releases/tag/R1-0-4) | Bugfix: only send positive velocities to the controller |
| **motorIms**       | [R1-0-2](https://github.com/epics-motor/motorIms/releases/tag/R1-1) | Added support for querying and clear locked rotor faults and party-line communication improvements |
| motorKohzu         | [R1-0-2](https://github.com/epics-motor/motorKohzu/releases/tag/R1-0-2) |         |
| motorMclennan      | [R1-1-1](https://github.com/epics-motor/motorMclennan/releases/tag/R1-1-1) |         |
| **motorMicos**     | [R2-1-1](https://github.com/epics-motor/motorMicos/releases/tag/R2-1-1) | Fix for build errors when using base 3.15; eliminated compiler warnings |
| motorMicroMo       | [R1-0-2](https://github.com/epics-motor/motorMicroMo/releases/tag/R1-0-2) |         |
| **motorMicronix**  | [R1-2](https://github.com/epics-motor/motorMicronix/releases/tag/R1-2) | MMC-110 and MMC-ETHERNET module now supported; resolution bugfixes |
| **motorMotorSim**  | [R1-3](https://github.com/epics-motor/motorMotorSim/releases/tag/R1-3) | Bug fix for first axis stuck in moving state |
| motorMXmotor       | [R1-0-2](https://github.com/epics-motor/motorMXmotor/releases/tag/R1-0-2) |         |
| **motorNewFocus**  | [R1-3](https://github.com/epics-motor/motorNewFocus/releases/tag/R1-3) | Added support for daisy-chained controllers to 874xMotorDriver |
| **motorNewport**   | [R1-3](https://github.com/epics-motor/motorNewport/releases/tag/R1-3) | Added support for CONEX-AGAP and HXP-D controllers; bug fixes |
| motorNPoint        | [R1-1](https://github.com/epics-motor/motorNPoint/releases/tag/R1-1) |         |
| **motorOms**       | [R1-2-1](https://github.com/epics-motor/motorOms/releases/tag/R1-2-1)  | Allow encoder ratio to be negative when motor and encoder have opposite senses of direction |
| **motorOmsAsyn**   | [R1-1](https://github.com/epics-motor/motorOmsAsyn/releases/tag/R1-1) | Added support for the MXA-21 controller |
| motorOriel         | [R1-0-2](https://github.com/epics-motor/motorOriel/releases/tag/R1-0-2) |         |
| **motorParker**    | [R1-2](https://github.com/epics-motor/motorParker/releases/tag/R1-2) | Added support for OEM series controllers |
| **motorPhytron**   | [R1-3](https://github.com/epics-motor/motorPhytron/releases/tag/R1-3) | Added support for MCC-1/MCC-2 controllers and I/O support for all Phytron controllers |
| **motorPI**        | [R1-1-2](https://github.com/epics-motor/motorPI/releases/tag/R1-1-2) | Fix for clang 16 build errors caused by use of deprecated keyword |
| **motorPIGCS2**    | [R1-3](https://github.com/epics-motor/motorPIGCS2/releases/tag/R1-3) | Added coordinate-system support for C-887 hexapod controller; print fewer debug messages by default |
| motorPiJena        | [R1-0-2](https://github.com/epics-motor/motorPiJena/releases/tag/R1-0-2) |         |
| motorScriptMotor   | [R1-2](https://github.com/epics-motor/motorScriptMotor/releases/tag/R1-2) |         |
| **motorSmarAct**   | [R2-1-1](https://github.com/epics-motor/motorSmarAct/releases/tag/R2-1-1) | Added support for sensorless positioners; eliminated compiler warnings |
| motorSmartMotor    | [R1-0-2](https://github.com/epics-motor/motorSmartMotor/releases/tag/R1-0-2) |         |
| **motorThorLabs** | [R1-0-3](https://github.com/epics-motor/motorThorLabs/releases/tag/R1-0-3) | Kinesis support now sends velo and accel to controller; fix for windows-x64-mingw compile errors |

## __R7-3-1 (2023-06-07)__
R7-3-1 is a release based on the master branch.

### Changes since R7-3

#### Modifications to existing features
* Commit [5eb994d](https://github.com/epics-modules/motor/commit/5eb994dc5e0f8a809382b300ad13f8c83e7da33b) from [Mark Rivers](https://github.com/MarkRivers): Added readController() functions to asynMotorController

#### Driver submodules (and noteworthy changes)

| Module           | Release | Changes |
| ---------------- | ------- | ------- |
| **motorAcsMotion** | [R2-2](https://github.com/epics-motor/motorAcsMotion/releases/tag/R2-2) | Improved homing and workaround for vxWorks build error |

## __R7-3 (2023-05-23)__
R7-3 is a release based on the master branch.  

### Changes since R7-2-2

#### New features
* Commit [c764bd6](https://github.com/epics-modules/motor/commit/c764bd6426c3939bded75df2909b76d148c2f8c9): Added motorAcsMotion as a submodule
* Pull request [#160](https://github.com/epics-modules/motor/pull/160) from [Torsten Bögershausen](https://github.com/tboegi): Added the Restore Mode (**RSTM**) field
* Pull request [#163](https://github.com/epics-modules/motor/pull/163): Autosave the RSTM field and allow it to be specified when motor databases are loaded
* Pull request [#197](https://github.com/epics-modules/motor/pull/197): Added model-2-specific databases to work around MOTOR_REC_{OFFSET,RESOLUTION,DIRECTION} errors at iocInit
* Pull request [#193](https://github.com/epics-modules/motor/pull/193) from [Jack Harper](https://github.com/rerpha): Added raw limit (**RHLM**, **RLLM**) fields
* Pull request [#202](https://github.com/epics-modules/motor/pull/202) and [#204](https://github.com/epics-modules/motor/pull/204): Added prop(YES) to fields in motorRecord.dbd that should generate DBE_PROPERTY callbacks
* Pull request [#122](https://github.com/epics-modules/motor/pull/122) from [Torsten Bögershausen](https://github.com/tboegi): Addded Acceleration (**ACCS**) and Acceleration Used (**ACCU**) fields
* Pull request [#203](https://github.com/epics-modules/motor/pull/203): Changed the Acceleration Used (ACCU) field from a readback to a control

#### Modifications to existing features
* Pull request [#180](https://github.com/epics-modules/motor/pull/180) from [Torsten Bögershausen](https://github.com/tboegi): Reset UEIP to 'No' if no encoder is present
* Pull request [#193](https://github.com/epics-modules/motor/pull/193) from [Jack Harper](https://github.com/rerpha): Soft limits are synced on MRES changes

#### Bug fixes
* Pull request [#176](https://github.com/epics-modules/motor/pull/176) from [Torsten Bögershausen](https://github.com/tboegi): Added shareLib.h to motordrvCom.h for compatibility with base 7.0.4
* Pull request [#182](https://github.com/epics-modules/motor/pull/182) from [Freddie Akeroyd](https://github.com/FreddieAkeroyd): Bug fix for negative backlash distance when relative moves are used 
* Pull request [#167](https://github.com/epics-modules/motor/pull/167) from [Torsten Bögershausen](https://github.com/tboegi): devMotorAsyn: remove initEvent in init_record()
* Commit [c3d6c6e](https://github.com/epics-modules/motor/commit/c3d6c6ee439e4d46fd2d4c2790091f7c76e31120) from [Mark Rivers](https://github.com/MarkRivers): Fixes for DTYP and EGU not displaying correctly in Phoebus
* Pull request [#206](https://github.com/epics-modules/motor/pull/206) from [Torsten Bögershausen](https://github.com/tboegi): Bug fix for RHLM, RLLM when MRES is negative
* Pull request [#186](https://github.com/epics-modules/motor/pull/186) from [justincslac](https://github.com/justincslac): Bug fix for .gitignore that caused the Db dir to be ignored on case-insensitive file systems
* Commit [96509ca](https://github.com/epics-modules/motor/commit/96509caba2ea1995bfeca3a0e8f11e1f273d2e05) from [Keenan Lang](https://github.com/keenanlang): Added basic_asyn_motor_settings.req which allows autosaveBuild to work with basic_asyn_motor.db

#### Documentation
* Commit [382c832](https://github.com/epics-modules/motor/commit/382c8324e5176ce769a09302bf829beca5a9cbea) from [Mark Rivers](https://github.com/MarkRivers): Added "Model 3 EPICS Motor Driver Support" slides
* Pull request [#190](https://github.com/epics-modules/motor/pull/190) from [Xiaoqiang Wang](https://github.com/xiaoqiangwang): Corrected field types in motorRecord.html

#### Continuous Integration
* Configured to build with GitHub Actions using ci-scripts v3.0.1

#### Driver submodules (and noteworthy changes)

All driver modules now use Github Actions for CI builds.

| Module           | Release | Changes |
| ---------------- | ------- | ------- |
| **motorAcs**     | [R1-1-1](https://github.com/epics-motor/motorAcs/releases/tag/R1-1-1) |         |
| **motorAcsMotion** | [R2-1](https://github.com/epics-motor/motorAcsMotion/releases/tag/R2-1) | Added as a motor submodule |
| **motorAcsTech80** | [R1-0-2](https://github.com/epics-motor/motorAcsTech80/releases/tag/R1-0-2) |         |
| **motorAerotech** | [R1-1-1](https://github.com/epics-motor/motorAerotech/releases/tag/R1-1-1) | Ensemble driver and doCommand.ab bug fixes |
| **motorAMCI** | [R1-0-2](https://github.com/epics-motor/motorAMCI/releases/tag/R1-0-2) | Replaced printf calls with asynPrint calls in ANG1Driver.cpp |
| **motorAttocube** | [R1-0-2](https://github.com/epics-motor/motorAttocube/releases/tag/R1-0-2) |         |
| **motorDeltaTau** | [R1-0-2](https://github.com/epics-motor/motorDeltaTau/releases/tag/R1-0-2) |         |
| **motorFaulhaber** | [R1-0-2](https://github.com/epics-motor/motorFaulhaber/releases/tag/R1-0-2) |         |
| **motorHytec**   | [R1-0-3](https://github.com/epics-motor/motorHytec/releases/tag/R1-0-3) |         |
| **motorIms**     | [R1-0-2](https://github.com/epics-motor/motorIms/releases/tag/R1-0-2) |         |
| **motorKohzu**   | [R1-0-2](https://github.com/epics-motor/motorKohzu/releases/tag/R1-0-2) |         |
| **motorMclennan** | [R1-1-1](https://github.com/epics-motor/motorMclennan/releases/tag/R1-1-1) |         |
| **motorMicos**   | [R2-1](https://github.com/epics-motor/motorMicos/releases/tag/R2-1) | Added support for the SMC Taurus |
| **motorMicroMo** | [R1-0-2](https://github.com/epics-motor/motorMicroMo/releases/tag/R1-0-2) |         |
| **motorMicronix** | [R1-1](https://github.com/epics-motor/motorMicronix/releases/tag/R1-1) |         |
| **motorMotorSim** | [R1-2](https://github.com/epics-motor/motorMotorSim/releases/tag/R1-2) | Added motorSim8x.iocsh. Multiple improvements to example IOC. |
| **motorMXmotor** | [R1-0-2](https://github.com/epics-motor/motorMXmotor/releases/tag/R1-0-2) |         |
| **motorNewFocus** | [R1-2-1](https://github.com/epics-motor/motorNewFocus/releases/tag/R1-2-1) | Improved example newfocus8742 configuration. iocsh files now installed. |
| **motorNewport**  | [R1-2-1](https://github.com/epics-motor/motorNewport/releases/tag/R1-2-1) | Added support for the FCL200. ESP300 driver now supports UEIP=Yes. iocsh files now installed. |
| **motorNPoint**  | [R1-1](https://github.com/epics-motor/motorNPoint/releases/tag/R1-1) |         |
| **motorOms**     | [R1-2](https://github.com/epics-motor/motorOms/releases/tag/R1-2)  | Enabled non-VxWorks IOCs & epicsMutexTryLock bug fix |
| **motorOmsAsyn** | [R1-0-3](https://github.com/epics-motor/motorOmsAsyn/releases/tag/R1-0-3) | Type fixes |
| **motorOriel**   | [R1-0-2](https://github.com/epics-motor/motorOriel/releases/tag/R1-0-2) |         |
| **motorParker**  | [R1-1-1](https://github.com/epics-motor/motorParker/releases/tag/R1-1-1) |         |
| **motorPhytron** | [R1-2](https://github.com/epics-motor/motorPhytron/releases/tag/R1-2) | Added encoder options. Bug fixes for deceleration and homing to limts. |
| **motorPI**      | [R1-1-1](https://github.com/epics-motor/motorPI/releases/tag/R1-1-1) | Added E-816 example configuration. iocsh files now installed. |
| **motorPIGCS2**  | [R1-2](https://github.com/epics-motor/motorPIGCS2/releases/tag/R1-2) | Added support for E-518 & E-873.3QTU controllers. Added closed loop commands for the E-727. Bug fixes. |
| **motorPiJena**  | [R1-0-2](https://github.com/epics-motor/motorPiJena/releases/tag/R1-0-2) |         |
| **motorScriptMotor** | [R1-2](https://github.com/epics-motor/motorScriptMotor/releases/tag/R1-2) | Added Attocube AMC support |
| **motorSmarAct** | [R2-0](https://github.com/epics-motor/motorSmarAct/releases/tag/R2-0) | Too many new features to list here; see [motorSmarAct release notes](https://github.com/epics-motor/motorSmarAct/releases/tag/R2-0) |
| **motorSmartMotor** | [R1-0-2](https://github.com/epics-motor/motorSmartMotor/releases/tag/R1-0-2) |         |
| **motorThorLabs** | [R1-0-2](https://github.com/epics-motor/motorThorLabs/releases/tag/R1-0-2) | Added support for Kinesis KDC101 and KST101 |

## __R7-2-2 (2021-02-02)__
R7-2-2 is a release based on the R7-2-1-bugfix branch.  

### Changes since R7-2-1

#### Bug fixes
* Added shareLib.h to asynMotorController.h and asynMotorAxis.h to allow building against asyn R4-41

#### Driver submodules (and noteworthy changes)
| Module           | Release | Changes |
| ---------------- | ------- | ------- |
| **motorNewport** | [070c93e](https://github.com/epics-motor/motorNewport/commits/070c93e1aa1a4dea39829f7dcb21489cfde582a1) | Added epicsExport.h to testSFTPUpload.cpp |
| **motorOms**     | [5219f8c](https://github.com/epics-motor/motorOms/commit/5219f8ceee352ec2a459f1bde316621af9c293a4)  | Fixed epicsMutexTryLock problem described [here](https://github.com/epics-modules/motor/issues/172#issuecomment-758139901) |

## __R7-2-1 (2020-05-14)__
R7-2-1 is a release based on the master branch.  

### Changes since R7-2

#### Bug fixes
* Motor drivers have been updated to resolve build problems for Windows and VxWorks

#### Driver submodules (and noteworthy changes)
| Module           | Release | Changes |
| ---------------- | ------- | ------- |
| motorAcs         | [R1-1](https://github.com/epics-motor/motorAcs/releases/tag/R1-1) |         |
| motorAcsTech80   | [R1-0-1](https://github.com/epics-motor/motorAcsTech80/releases/tag/R1-0-1) |         |
| motorAerotech    | [R1-1](https://github.com/epics-motor/motorAerotech/releases/tag/R1-1) |         |
| motorAMCI        | [R1-0-1](https://github.com/epics-motor/motorAMCI/releases/tag/R1-0-1) |         |
| motorAttocube    | [R1-0-1](https://github.com/epics-motor/motorAttocube/releases/tag/R1-0-1) |         |
| motorDeltaTau    | [R1-0-1](https://github.com/epics-motor/motorDeltaTau/releases/tag/R1-0-1) |         |
| motorFaulhaber   | [R1-0-1](https://github.com/epics-motor/motorFaulhaber/releases/tag/R1-0-1) |         |
| **motorHytec**   | [R1-0-2](https://github.com/epics-motor/motorHytec/releases/tag/R1-0-2) | Only build the example IOC on Linux and VxWorks |
| motorIms         | [R1-0-1](https://github.com/epics-motor/motorIms/releases/tag/R1-0-1) |         |
| motorKohzu       | [R1-0-1](https://github.com/epics-motor/motorKohzu/releases/tag/R1-0-1) |         |
| motorMclennan    | [R1-1](https://github.com/epics-motor/motorMclennan/releases/tag/R1-1) |         |
| motorMicos       | [R2-0](https://github.com/epics-motor/motorMicos/releases/tag/R2-0) |         |
| motorMicroMo     | [R1-0-1](https://github.com/epics-motor/motorMicroMo/releases/tag/R1-0-1) |         |
| motorMicronix    | [R1-0-1](https://github.com/epics-motor/motorMicronix/releases/tag/R1-0-1) |         |
| motorMotorSim    | [R1-1](https://github.com/epics-motor/motorMotorSim/releases/tag/R1-1) |         |
| motorMXmotor     | [R1-0-1](https://github.com/epics-motor/motorMXmotor/releases/tag/R1-0-1) |         |
| motorNewFocus    | [R1-1-1](https://github.com/epics-motor/motorNewFocus/releases/tag/R1-1-1) |         |
| motorNewport     | [R1-1](https://github.com/epics-motor/motorNewport/releases/tag/R1-1) |         |
| motorNPoint      | [R1-0-1](https://github.com/epics-motor/motorNPoint/releases/tag/R1-0-1) |         |
| motorOms         | [R1-1](https://github.com/epics-motor/motorOms/releases/tag/R1-1)  |         |
| **motorOmsAsyn** | [R1-0-2](https://github.com/epics-motor/motorOmsAsyn/releases/tag/R1-0-2) | Exclude MAXv support when building example IOC on Windows |
| motorOriel       | [R1-0-1](https://github.com/epics-motor/motorOriel/releases/tag/R1-0-1) |         |
| motorParker      | [R1-1](https://github.com/epics-motor/motorParker/releases/tag/R1-1) |         |
| motorPhytron     | [R1-1](https://github.com/epics-motor/motorPhytron/releases/tag/R1-1) |         |
| motorPI          | [R1-0-1](https://github.com/epics-motor/motorPI/releases/tag/R1-0-1) |         |
| motorPIGCS2      | [R1-1](https://github.com/epics-motor/motorPIGCS2/releases/tag/R1-1) |         |
| motorPiJena      | [R1-0-1](https://github.com/epics-motor/motorPiJena/releases/tag/R1-0-1) |         |
| motorScriptMotor | [R1-1](https://github.com/epics-motor/motorScriptMotor/releases/tag/R1-1) |         |
| **motorSmarAct** | [R1-2-1](https://github.com/epics-motor/motorSmarAct/releases/tag/R1-2-1) | Include a local definition of rint on fewer platforms |
| motorSmartMotor  | [R1-0-1](https://github.com/epics-motor/motorSmartMotor/releases/tag/R1-0-1) |         |
| motorThorLabs    | [R1-0-1](https://github.com/epics-motor/motorThorLabs/releases/tag/R1-0-1) |         |

## __R7-2 (2020-05-13)__
R7-2 is a release based on the master branch.  

### Changes since R7-1

#### Modifications to existing features
* ``CONFIG_SITE`` now includes ``$(SUPPORT)/configure/CONFIG_SITE``, which overrides ``CONFIG_SITE.local``
* ``modules/RELEASE.$(EPICS_HOST_ARCH).local`` is now rebuilt every time the build get into the ``modules`` directory
* User displays have been autoconverted

#### Bug fixes
* asynMotorAxis parameters (``motorPowerAutoOnOff_``, ``motorPowerOffDelay_``, ``motorPowerOnDelay_``) are now initialized to avoid errors
* The layout of ``motorxU.adl`` has been improved

#### Driver submodules (and noteworthy changes)
| Module           | Release | Changes |
| ---------------- | ------- | ------- |
| **motorAcs**     | [R1-1](https://github.com/epics-motor/motorAcs/releases/tag/R1-1) | iocsh files are now installed at build time |
| **motorAcsTech80** | [R1-0-1](https://github.com/epics-motor/motorAcsTech80/releases/tag/R1-0-1) |         |
| **motorAerotech** | [R1-1](https://github.com/epics-motor/motorAerotech/releases/tag/R1-1) | motorAxisHomed bit is now set in the Ensemble poller |
| **motorAMCI** | [R1-0-1](https://github.com/epics-motor/motorAMCI/releases/tag/R1-0-1) |         |
| **motorAttocube** | [R1-0-1](https://github.com/epics-motor/motorAttocube/releases/tag/R1-0-1) |         |
| **motorDeltaTau** | [R1-0-1](https://github.com/epics-motor/motorDeltaTau/releases/tag/R1-0-1) |         |
| **motorFaulhaber** | [R1-0-1](https://github.com/epics-motor/motorFaulhaber/releases/tag/R1-0-1) |         |
| **motorHytec**   | [R1-0-1](https://github.com/epics-motor/motorHytec/releases/tag/R1-0-1) |         |
| **motorIms**     | [R1-0-1](https://github.com/epics-motor/motorIms/releases/tag/R1-0-1) |         |
| **motorKohzu**   | [R1-0-1](https://github.com/epics-motor/motorKohzu/releases/tag/R1-0-1) |         |
| **motorMclennan** | [R1-1](https://github.com/epics-motor/motorMclennan/releases/tag/R1-1) | iocsh files are now installed at build time |
| **motorMicos**   | [R2-0](https://github.com/epics-motor/motorMicos/releases/tag/R2-0) | Added support for the SMC Corvus Eco |
| **motorMicroMo** | [R1-0-1](https://github.com/epics-motor/motorMicroMo/releases/tag/R1-0-1) |         |
| **motorMicronix** | [R1-0-1](https://github.com/epics-motor/motorMicronix/releases/tag/R1-0-1) |         |
| **motorMotorSim** | [R1-1](https://github.com/epics-motor/motorMotorSim/releases/tag/R1-1) | iocsh files are now installed at build time |
| **motorMXmotor** | [R1-0-1](https://github.com/epics-motor/motorMXmotor/releases/tag/R1-0-1) |         |
| **motorNewFocus** | [R1-1-1](https://github.com/epics-motor/motorNewFocus/releases/tag/R1-1-1) |         |
| **motorNewport**  | [R1-1](https://github.com/epics-motor/motorNewport/releases/tag/R1-1) | Added support for the XPS-D |
| **motorNPoint**  | [R1-0-1](https://github.com/epics-motor/motorNPoint/releases/tag/R1-0-1) |         |
| **motorOms**     | [R1-1](https://github.com/epics-motor/motorOms/releases/tag/R1-1)  | User displays can now be autoconverted at build time |
| **motorOmsAsyn** | [R1-0-1](https://github.com/epics-motor/motorOmsAsyn/releases/tag/R1-0-1) |         |
| **motorOriel**   | [R1-0-1](https://github.com/epics-motor/motorOriel/releases/tag/R1-0-1) |         |
| **motorParker**  | [R1-1](https://github.com/epics-motor/motorParker/releases/tag/R1-1) | User displays can now be autoconverted at build time |
| **motorPhytron** | [R1-1](https://github.com/epics-motor/motorPhytron/releases/tag/R1-1) | Error messages are now only printed on status changes |
| **motorPI**      | [R1-0-1](https://github.com/epics-motor/motorPI/releases/tag/R1-0-1) |         |
| **motorPIGCS2**  | [R1-1](https://github.com/epics-motor/motorPIGCS2/releases/tag/R1-1) | Added support for the E-754 and C-885 controllers |
| **motorPiJena**  | [R1-0-1](https://github.com/epics-motor/motorPiJena/releases/tag/R1-0-1) |         |
| **motorScriptMotor** | [R1-1](https://github.com/epics-motor/motorScriptMotor/releases/tag/R1-1) | User displays can now be autoconverted at build time |
| **motorSmarAct** | [R1-2](https://github.com/epics-motor/motorSmarAct/releases/tag/R1-2) | Added support for the SCU controllers |
| **motorSmartMotor** | [R1-0-1](https://github.com/epics-motor/motorSmartMotor/releases/tag/R1-0-1) |         |
| **motorThorLabs** | [R1-0-1](https://github.com/epics-motor/motorThorLabs/releases/tag/R1-0-1) |         |

## __R7-1 (2019-08-13)__
R7-1 is a release based on the master branch.  

### Changes since R7-0

#### Bug fixes
* Req files are now installed to motor's top-level db directory when building against EPICS base 3.14
* RELEASE now allows RELEASE.local files to override settings

#### Driver submodules (and noteworthy changes)
| Module           | Release | Changes |
| ---------------- | ------- | ------- |
| motorAcs         | R1-0    |         |
| motorAcsTech80   | R1-0    |         |
| **motorAerotech** | **R1-0-1** | [Improvements to EnsembleTrajectoryScan](https://github.com/epics-motor/motorAerotech/releases/tag/R1-0-1) |
| motorAMCI        | R1-0    |         |
| motorAttocube    | R1-0    |         |
| motorDeltaTau    | R1-0    |         |
| motorFaulhaber   | R1-0    |         |
| motorHytec       | R1-0    |         |
| motorIms         | R1-0    |         |
| motorKohzu       | R1-0    |         |
| motorMclennan    | R1-0    |         |
| **motorMicos**   | **R1-1** | [Improvements to SMC Hydra driver](https://github.com/epics-motor/motorMicos/releases/tag/R1-1) |
| motorMicroMo     | R1-0    |         |
| motorMicronix    | R1-0    |         |
| motorMotorSim    | R1-0    |         |
| motorMXmotor     | R1-0    |         |
| **motorNewFocus** | **R1-1** | [Improvements to 874xMotorDriver](https://github.com/epics-motor/motorNewFocus/releases/tag/R1-1) |
| **motorNewport**  | **R1-0-1**    | [Corrected typo that prevented template installation](https://github.com/epics-motor/motorNewport/releases/tag/R1-0-1) |
| motorNPoint      | R1-0    |         |
| motorOms         | R1-0    |         |
| motorOmsAsyn     | R1-0    |         |
| motorOriel       | R1-0    |         |
| motorParker      | R1-0    |         |
| motorPhytron     | R1-0    |         |
| motorPI          | R1-0    |         |
| motorPIGCS2      | R1-0    |         |
| motorPiJena      | R1-0    |         |
| motorScriptMotor | R1-0    |         |
| **motorSmarAct** | **R1-1** | [Added support for MCS2 controller](https://github.com/epics-motor/motorSmarAct/releases/tag/R1-1) |
| motorSmartMotor  | R1-0    |         |
| motorThorLabs    | R1-0    |         |

## __R7-0 (2019-04-19)__
R7-0 is a release based on the master branch.  

### Changes since R6-11

Stand-alone repositories have been created for most of the support that previously resided in ``motorApp``.  These new respositories can be found in [epics-motor](https://github.com/epics-motor).  The core motor functionality remains in this module.

The new driver repositories have been added to motor as submodules, which reside in the ``modules`` subdirectory.  When the driver modules are built as submodules, their build products are installed into motor's top-level directories: ``db``, ``dbd``, ``lib``.  **The locations of vendor-specific files that aren't installed (iocsh scripts, user displays) will differ from previous versions of motor.** These files will remain in the vendor's submodule.

The new driver modules can be built outside of the motor module, which results in the installation of the build products into the top-level directory of the driver module.

The recommended EPICS base version is 3.15.6 or later.  Submodules that install template files will fail to build against EPICS base 3.14.

#### Bug fixes
* Multiple fixes for motor displays

#### Driver submodules (and noteworthy changes)
| Module           | Release | Changes |
| ---------------- | ------- | ------- |
| motorAcs         | R1-0    |         |
| motorAcsTech80   | R1-0    |         |
| motorAerotech    | R1-0    |         |
| motorAMCI        | R1-0    |         |
| motorAttocube    | R1-0    |         |
| motorDeltaTau    | R1-0    |         |
| motorFaulhaber   | R1-0    |         |
| motorHytec       | R1-0    |         |
| motorIms         | R1-0    |         |
| motorKohzu       | R1-0    |         |
| motorMclennan    | R1-0    |         |
| motorMicos       | R1-0    |         |
| motorMicroMo     | R1-0    |         |
| motorMicronix    | R1-0    |         |
| motorMotorSim    | R1-0    |         |
| motorMXmotor     | R1-0    |         |
| motorNewFocus    | R1-0    |         |
| motorNewport     | R1-0    | [HXP driver updated](https://github.com/epics-motor/motorNewport/releases/tag/R1-0) |
| motorNPoint      | R1-0    |         |
| motorOms         | R1-0    |         |
| motorOmsAsyn     | R1-0    |         |
| motorOriel       | R1-0    |         |
| motorParker      | R1-0    |         |
| motorPhytron     | R1-0    | [phytron.dbd renamed](https://github.com/epics-motor/motorPhytron/releases/tag/R1-0) |
| motorPI          | R1-0    |         |
| motorPIGCS2      | R1-0    |         |
| motorPiJena      | R1-0    |         |
| motorScriptMotor | R1-0    |         |
| motorSmarAct     | R1-0    |         |
| motorSmartMotor  | R1-0    |         |
| motorThorLabs    | R1-0    |         |

## __R6-11 (2018-12-12)__
R6-11 is a release based on the master branch.  

### Changes since R6-10-1

#### Modifications to existing features
* Pull request [#109](https://github.com/epics-modules/motor/pull/109) motorRecord: Don't stop motor if driver sets RA_PROBLEM true
* Pull request [#108](https://github.com/epics-modules/motor/pull/108) motorRecord: Set LVIO=1 if DLLM > DHLM
* Pull request [#36](https://github.com/epics-modules/motor/pull/36) motorRecord: Update status of limit switches regardless of movement direction
* Pull request [#99](https://github.com/epics-modules/motor/pull/99) motorRecord: Reset the JOGF/JOGR fields when a limit violation occurs
* Pull request [#56](https://github.com/epics-modules/motor/pull/56) motorRecord: Recognize stopped motor while jogging
* Pull request [#84](https://github.com/epics-modules/motor/pull/84) motorRecord: Keep the sign information of the components of the encoder ratio (MRES and ERES). Drivers made consistent in pull request #98
* Pull request [#93](https://github.com/epics-modules/motor/pull/93) OmsAsyn: Synchronize motor position with encoder position before every move
* Pull request [#103](https://github.com/epics-modules/motor/pull/103) Aerotech Ensemble: If disabling torque due to a fault, clear motorAxisProblem so that user can jog off limit switch
* Commit [74a8ced](https://github.com/epics-modules/motor/commit/74a8cedc283c6f53042dea867b4e6a1363c74479) Aerotech Ensemble: update CountsPerUnit every time torque is enabled to reduce the need for a reboot after parameter file change
* Commit [c0c5b5b](https://github.com/epics-modules/motor/commit/c0c5b5b351c33549437cffbd4233edb15a9361c2) Multiple changes to Aerotech A3200 driver: 
  * restored "task number" argument
  * don't check limit switches of virtual axes
  * added single/multi axis move argument 

#### New features
* Pull request [#114](https://github.com/epics-modules/motor/pull/114) motorRecord: Added Set Point Deadband field (SPDB) enabling deadbands greater than the MRES
* Pull request [#95](https://github.com/epics-modules/motor/pull/95) Added support for AMCI ANF-series controllers (depends on the modbus module: https://github.com/epics-modules/modbus, R2-11 or later)
* Pull request [#79](https://github.com/epics-modules/motor/pull/79) Added support for Scriptable Motor Controller (depends on the lua module: https://github.com/epics-modules/lua)

#### Bug fixes
* Pull request [#105](https://github.com/epics-modules/motor/pull/105) Fix for segfault when XPSConfigAxis isn't called before using a motor
* Commit [df11f3e](https://github.com/epics-modules/motor/commit/df11f3e153e2aa6c6cb605cbc1392e0a9e663d93) Corrected a typo in motorSim.iocsh that prevented any motion by setting the DLLM and DHLM to the same value

#### Documentation updates
* Pull request [#101](https://github.com/epics-modules/motor/pull/101), commit [58976fa](https://github.com/epics-modules/motor/commit/58976fa2885660b753ab84003667638934fdd557) Made motor documentation compatible with github pages

## __R6-10-1 (2018-06-07)__ 
R6-10-1 is a bugfix release based on the R6-10-bugfix branch.  

### Changes since R6-10
The following commits to the master branch have been cherry-picked for this release.

#### Bug fixes
* pull request [#60](https://github.com/epics-modules/motor/pull/60) motor record DLY and STOP fix
* pull request [#83](https://github.com/epics-modules/motor/pull/83) asynMotor autoPower fix
* pull request [#94](https://github.com/epics-modules/motor/pull/94) Micronix MMC-x00 I/O flush
* commit [7493d50](https://github.com/epics-modules/motor/commit/7493d50bc818bb56440525f71a4ca31c9a28ee29) Don't start a new move if URIP=Yes & RDBL link error

#### Code fixes
* commit [4938a51](https://github.com/epics-modules/motor/commit/4938a51ce459e6b213ac3e157112892c86e629a8) Fixed casts
* commit [23b8c5f](https://github.com/epics-modules/motor/commit/23b8c5f528416def6ee8329caf4f0fe8331e434c) Fix for URLS in travis script
* commit [75162d1](https://github.com/epics-modules/motor/commit/75162d1fefc2aa1c3009a3bbbde7fac69345213b) Fix for example substitutions file
* commit [6998c37](https://github.com/epics-modules/motor/commit/6998c3716317e725d123c77844dfb3da84ed9768) Fix typos
* commit [c1c4407](https://github.com/epics-modules/motor/commit/c1c440764414627237580c2642bf5542aee623b2) Corrected EPICS version test
* commit [60aa414](https://github.com/epics-modules/motor/commit/60aa414b373b48e96db2bd010492ae6b29577f35) Added header files for EPICS 7.0 compatiblity
* commit [8409249](https://github.com/epics-modules/motor/commit/840924984945a3454cd114ef8391952158151308) Added header files for EPICS 7.0 compatiblity

#### Documentation updates

* commit [6529abb](https://github.com/epics-modules/motor/commit/6529abb0cfccc08a1c4ea6f0c7a3d326b48bd81b) Added R6-10-1 links
* commit [acf0d00](https://github.com/epics-modules/motor/commit/acf0d00496ce7b4edf953991ecc3037b236e37ea) Added home-search documentation to Aerotech README
* commit [4bbba98](https://github.com/epics-modules/motor/commit/4bbba98bd51f65a5efea30fa647e9d914c99d50b) Minor improvements to documentation
* commit [2cfd494](https://github.com/epics-modules/motor/commit/2cfd494c1631c9af287fde3d93de650b0de2a3b2) Added R6-10 documentation

## __R6-10 and older__

Release notes can be found here: https://epics-modules.github.io/motor/motor_release.html

Details relevant to developers can be found here: https://github.com/epics-modules/motor/blob/master/docs/RELEASE.txt

