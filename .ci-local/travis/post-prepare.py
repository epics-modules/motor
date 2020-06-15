#!/usr/bin/env python

import os
import sys
import subprocess

# Comment out SUPPORT from motor's RELEASE file
if sys.version_info >= (3, 5):
  status = subprocess.run(['sed', '-i', '-e', "s|^\(SUPPORT=.*\)$|#\1|g", './configure/RELEASE'])
else:
  status = subprocess.call(['sed', '-i', '-e', "s|^\(SUPPORT=.*\)$|#\1|g", './configure/RELEASE'])
#
print("{}Updated motor/configure/RELEASE{}".format(ANSI_BLUE, ANSI_RESET))
#!grep SUPPORT ./configure/RELEASE || :
os.system('cat ./configure/RELEASE')
print("{}End of updated motor/configure/RELEASE{}".format(ANSI_BLUE, ANSI_RESET))

# Comment out SUPPORT from motorOms's RELEASE file
if sys.version_info >= (3, 5):
  status = subprocess.run(['sed', '-i', '-e', "s|^\(SUPPORT=.*\)$|#\1|g", './modules/motorOms/configure/RELEASE'])
else:
  status = subprocess.call(['sed', '-i', '-e', "s|^\(SUPPORT=.*\)$|#\1|g", './modules/motorOms/configure/RELEASE'])
#
print("{}Updated motor/modules/motorOms/configure/RELEASE{}".format(ANSI_BLUE, ANSI_RESET))
#!grep SUPPORT ./modules/motorOms/configure/RELEASE || :
os.system('cat ./modules/motorOms/configure/RELEASE')
print("{}End of updated motor/modules/motorOms/configure/RELEASE{}".format(ANSI_BLUE, ANSI_RESET))
