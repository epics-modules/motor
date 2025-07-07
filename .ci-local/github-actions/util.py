#!/usr/bin/env python

from __future__ import print_function

import logging
import fileinput
import os

# Setup ANSI Colors
ANSI_RED = "\033[31;1m"
ANSI_GREEN = "\033[32;1m"
ANSI_YELLOW = "\033[33;1m"
ANSI_BLUE = "\033[34;1m"
ANSI_MAGENTA = "\033[35;1m"
ANSI_CYAN = "\033[36;1m"
ANSI_RESET = "\033[0m"
ANSI_CLEAR = "\033[0K"

logger = logging.getLogger(__name__)

def create_file(filename):
    dirname = os.path.dirname(filename)
    print(dirname)
    if not os.path.exists(dirname):
        os.makedirs(dirname, 0o0755)
    if not os.path.exists(filename):
        fh = open(filename, 'w')
        fh.close()
    else:
        logger.debug("{} already exists".format(filename))

# Update a definition in a release file that already exists
# Append a defintion to a release file that already exists
# Commented out a definition in a release file (location='#')
def update_release_file(filename, var, location):
    updated_line = '{0}={1}'.format(var, location.replace('\\', '/'))

    found = False
    logger.debug("Opening '%s' for adding '%s'", filename, updated_line)
    for line in fileinput.input(filename, inplace=1):
        output_line = line.strip()
        if '{0}='.format(var) in line:
            logger.debug("Found '%s=' line, replacing", var)
            found = True
            if location != '#':
                output_line = updated_line
            else:
                if output_line[0] != '#':
                    output_line = "#{}".format(output_line)
        logger.debug("Writing line to '%s': '%s'", filename, output_line)
        print(output_line)
    fileinput.close()
    
    if not found and location != '#':
        fh = open(release_local, "a")
        logger.debug("Adding new definition: '%s'", updated_line)
        print(updated_line, file=fh)
        fh.close()
    
    return found

def grep_release_file(filename, var):
    fh = open(filename, 'r')
    for line in fh:
        if '{0}='.format(var) in line:
            print(line)
    fh.close()
    
