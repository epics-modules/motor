#!/usr/bin/env python

import os
import sys
import subprocess

from util import *

def get_submodule_releases():
    #
    releases = []
    for root, dirs, files in os.walk('modules'):
        for submodule in dirs:
           if 'motor' in submodule:
               release_path = "modules/{}/configure/RELEASE".format(submodule)
               if os.path.exists(release_path):
                  releases.append(release_path)
    return releases[:]

# Comment out SUPPORT from motor's RELEASE file
motor_release = "configure/RELEASE"
update_release_file(motor_release, "SUPPORT", '#')
print("{}Updated {}{}".format(ANSI_BLUE, motor_release, ANSI_RESET))
grep_release_file(motor_release, "#SUPPORT")

# Comment out SUPPORT from the driver module RELEASE files
releases = get_submodule_releases()
for release in releases:
    updated = update_release_file(release, "SUPPORT", '#')
    if updated:
        print("{}Updated {}{}".format(ANSI_BLUE, release, ANSI_RESET))
        grep_release_file(release, "#SUPPORT")
