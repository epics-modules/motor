# motor
[![Build Status](https://github.com/epics-modules/motor/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-modules/motor/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-modules/motor.png)](https://travis-ci.org/epics-modules/motor)-->
[![REUSE status](https://api.reuse.software/badge/github.com/epics-modules/motor)](https://api.reuse.software/info/github.com/epics-modules/motor)

This module contains motor support for the Experimental Physics and Industrial Control System (EPICS).

The core motor functionality resides in this respository.  Starting with R7-0 the drivers have been moved from motor to standalone github repositories, which have been added to motor as submodules.  The driver repositories can be found here: https://github.com/epics-motor

## Getting started

### Updating clones created before 2019-04-02
Use the following procedure to update clones that were created before the beginning of the motor split:
```bash
$ cd motor
$ make distclean
$ git fetch origin
$ git status
<behind by many commits and can be fast-forwarded>
$ git stash
$ git rebase origin/master
$ git stash apply
$ git submodule init
$ git submodule update
```
Failure to 'make distclean' before rebasing will result in many driver source directories in motorApp with "O.*" directories that need to be removed manually.

### Cloning motor with support for all motor controllers
The following command results in a motor directory that contains all of the driver submodules:
```bash
$ git clone --recursive https://github.com/epics-modules/motor.git
```

### Cloning motor with support for a single motor controller
The following procedure allows only required drivers to be built, which can significantly reduce build times:
```bash
$ git clone https://github.com/epics-modules/motor.git
$ cd motor
$ git submodule init
$ git submodule update modules/motorMotorSim
```

## Additional Info
For more information, see:
*  [main page](https://epics-modules.github.io/motor)
*  [release notes](https://github.com/epics-modules/motor/blob/master/docs/RELEASE.md)
*  [motor wiki](https://github.com/epics-modules/motor/wiki)
*  http://www.aps.anl.gov/bcda/synApps
*  [HTML documentation](https://github.com/epics-modules/motor/blob/master/docs/README.md)

[Report an issue with Motor](https://github.com/epics-modules/motor/issues/new?title=%20ISSUE%20NAME%20HERE&body=**Describe%20the%20issue**%0A%0A**Steps%20to%20reproduce**%0A1.%20Step%20one%0A2.%20Step%20two%0A3.%20Step%20three%0A%0A**Expected%20behaivour**%0A%0A**Actual%20behaviour**%0A%0A**Build%20Environment**%0AArchitecture:%0AEpics%20Base%20Version:%0ADependent%20Module%20Versions:&labels=bug)  
[Request a feature](https://github.com/epics-modules/motor/issues/new?title=%20FEATURE%20SHORT%20DESCRIPTION&body=**Feature%20Long%20Description**%0A%0A**Why%20should%20this%20be%20added?**%0A&labels=enhancement)

