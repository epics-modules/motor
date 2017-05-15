#!/bin/sh
TAB=$(printf '\t')
LF=$(printf '\n')
echo TABX=${TAB}X

fileswithtab=$(find . -name "*.[ch]" | grep -v "\.AppleDouble/")
if test -n "$fileswithtab"; then
  echo fileswithtab=$fileswithtab
  cmd=$(printf "%s %s" 'egrep -n "$TAB| \$"' "$fileswithtab")
  echo cmd=$cmd
  eval $cmd
  if test $? -eq 0; then
    #TABS found
    exit 1
  fi
fi
exit 0
