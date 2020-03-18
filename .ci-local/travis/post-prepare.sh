#!/bin/bash
set -e

# Set VV in .travis.yml to make scripts verbose
[ "$VV" ] && set -x

CACHEDIR=${CACHEDIR:-${HOME}/.cache}

# source functions
. ${HOME}/.ci/travis/utils.sh

# Add SUPPORT to RELEASE.local
update_release_local SUPPORT ${CACHEDIR}

echo -e "${ANSI_BLUE}Updated contents of RELEASE.local${ANSI_RESET}"
cat ${CACHEDIR}/RELEASE.local
