# Motor Releases

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

