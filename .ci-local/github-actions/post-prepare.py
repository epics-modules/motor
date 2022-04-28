#!/usr/bin/env python

import os
import shutil
import re

# Setup ANSI Colors (copied from cue.py)
ANSI_RED = "\033[31;1m"
ANSI_GREEN = "\033[32;1m"
ANSI_YELLOW = "\033[33;1m"
ANSI_BLUE = "\033[34;1m"
ANSI_MAGENTA = "\033[35;1m"
ANSI_CYAN = "\033[36;1m"
ANSI_RESET = "\033[0m"
ANSI_CLEAR = "\033[0K"

def cat(filename):
    '''
    Print the contents of a file
    '''
    with open(filename, 'r') as fh:
        for line in fh:
            print(line.strip())

def sanity_check(filename):
    '''
    Include the contents of a file in the github-actions log
    '''
    print("{}Contents of {}{}".format(ANSI_BLUE, filename, ANSI_RESET))
    cat(filename)
    print("{}End of {}{}".format(ANSI_BLUE, filename, ANSI_RESET))

if 'HOME' in os.environ:
    # Linux & OS X
    cache_dir = os.path.join(os.environ['HOME'], ".cache")
else:
    # Windows
    cache_dir = os.path.join(os.environ['HOMEDRIVE'], os.environ['HOMEPATH'], ".cache")

module_dir = os.getenv('GITHUB_WORKSPACE')

# Copy the github-actions RELEASE.local to the configure dir
filename = "configure/RELEASE.local"
shutil.copy("{}/RELEASE.local".format(cache_dir), filename)

# Get the variable from the example release file
example = "configure/EXAMPLE_RELEASE.local"
fh = open(example, "r")
lines = fh.readlines()
fh.close()
pObj = re.compile('(MOTOR_[^=]+)')
module_var = None
for line in lines:
    mObj = pObj.match(line)
    if mObj != None:
      module_var = mObj.group()
      break

# Add the path to the driver module to the RELEASE.local file, since it is needed by the example IOC
fh = open(filename, "a")
fh.write("{}={}\n".format(module_var, module_dir))
fh.close()
sanity_check(filename)

# Enable the building of example IOCs
filename = "configure/CONFIG_SITE.local"
fh = open(filename, 'w')
fh.write("BUILD_IOCS = YES")
fh.close()
sanity_check(filename)
