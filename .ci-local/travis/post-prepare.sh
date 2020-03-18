#!/bin/bash
set -e

# Set VV in .travis.yml to make scripts verbose
[ "$VV" ] && set -x

CACHEDIR=${CACHEDIR:-${HOME}/.cache}

# sanity check
pwd

# source functions
. ./.ci/travis/utils.sh

# Add SUPPORT to RELEASE.local in the cache directory
update_release_local SUPPORT ${CACHEDIR}
# Copy the RELEASE.local from the cache directory to motor's configure directory
[ -e ./configure ] && cp -f ${CACHEDIR}/RELEASE.local ./configure/RELEASE.local

echo -e "${ANSI_BLUE}Updated contents of RELEASE.local${ANSI_RESET}"
cat ${CACHEDIR}/RELEASE.local
