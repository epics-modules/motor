#!/bin/bash

for i in  motorMicos motorAttocube motorAcs motorAMCI motorThorLabs motorSmartMotor motorScriptMotor motorPiJena motorPhytron motorParker motorPIGCS2 motorPI motorOriel motorOmsAsyn motorOms motorNPoint motorMotorSim motorMicronix motorMicroMo motorMclennan motorMXmotor motorKohzu motorIms motorHytec motorFaulhaber motorAcsTech80
do
    # Remove the submodule entry from .git/config
    git submodule deinit -f modules/$i

    # Remove the submodule directory from the superproject's .git/modules directory
    rm -rf .git/$i

    # Remove the entry in .gitmodules and remove the submodule directory located at path/to/submodule
    git rm -f modules/$i
done

# If we have to add some submodules we can use this command 

# git submodule add -b development git@github.com:CentralLaserFacility/motorParker6K.git modules/motorParker6K

git submodule sync
git submodule update --init --recursive
