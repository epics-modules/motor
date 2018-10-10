#!/bin/sh
set -e -x

SUPPORT=$HOME/.cache/support

install -d $SUPPORT

# Conditionally build IPAC
if [ -n "$IPAC" ]; then
    IPAC_PATH=$SUPPORT/ipac
else
    IPAC_PATH=
fi

RELEASE_PATH=$TRAVIS_BUILD_DIR/configure/RELEASE
EPICS_BASE=$SUPPORT/epics-base

cat << EOF > $RELEASE_PATH
IPAC=$IPAC_PATH
SNCSEQ=$SUPPORT/seq
ASYN=$SUPPORT/asyn
EPICS_BASE=$SUPPORT/epics-base
EOF

# use default selection for MSI
sed -i -e '/MSI/d' configure/CONFIG_SITE

if [ ! -e "$EPICS_BASE/built" ] 
then

    git clone --depth 10 --branch $BASE https://github.com/epics-base/epics-base.git $EPICS_BASE

    EPICS_HOST_ARCH=`sh $EPICS_BASE/startup/EpicsHostArch`

    case "$STATIC" in
    static)
        cat << EOF >> "$EPICS_BASE/configure/CONFIG_SITE"
SHARED_LIBRARIES=NO
STATIC_BUILD=YES
EOF
        ;;
    *) ;;
    esac

    case "$CMPLR" in
    clang)
      echo "Host compiler is clang"
      cat << EOF >> $EPICS_BASE/configure/os/CONFIG_SITE.Common.$EPICS_HOST_ARCH
GNU         = NO
CMPLR_CLASS = clang
CC          = clang
CCC         = clang++
EOF
      ;;
    *) echo "Host compiler is default";;
    esac

    # requires wine and g++-mingw-w64-i686
    if [ "$WINE" = "32" ]
    then
      echo "Cross mingw32"
      sed -i -e '/CMPLR_PREFIX/d' $EPICS_BASE/configure/os/CONFIG_SITE.linux-x86.win32-x86-mingw
      cat << EOF >> $EPICS_BASE/configure/os/CONFIG_SITE.linux-x86.win32-x86-mingw
CMPLR_PREFIX=i686-w64-mingw32-
EOF
      cat << EOF >> $EPICS_BASE/configure/CONFIG_SITE
CROSS_COMPILER_TARGET_ARCHS+=win32-x86-mingw
EOF
    fi

    # set RTEMS to eg. "4.9" or "4.10"
    if [ -n "$RTEMS" ]
    then
      echo "Cross RTEMS${RTEMS} for pc386"
      install -d /home/travis/.cache
      curl -L "https://github.com/mdavidsaver/rsb/releases/download/travis-20160306-2/rtems${RTEMS}-i386-trusty-20190306-2.tar.gz" \
      | tar -C /home/travis/.cache -xj

      sed -i -e '/^RTEMS_VERSION/d' -e '/^RTEMS_BASE/d' $EPICS_BASE/configure/os/CONFIG_SITE.Common.RTEMS
      cat << EOF >> $EPICS_BASE/configure/os/CONFIG_SITE.Common.RTEMS
RTEMS_VERSION=$RTEMS
RTEMS_BASE=/home/travis/.cache/rtems${RTEMS}-i386
EOF
      cat << EOF >> $EPICS_BASE/configure/CONFIG_SITE
CROSS_COMPILER_TARGET_ARCHS+=RTEMS-pc386
EOF

    fi

    make -C "$EPICS_BASE" -j2
    # get MSI for 3.14
    case "$BASE" in
    R3.14*)
        echo "Build MSI"
        install -d "$HOME/msi/extensions/src"
        curl https://epics.anl.gov/download/extensions/extensionsTop_20120904.tar.gz | tar -C "$HOME/msi" -xvz
        curl https://epics.anl.gov/download/extensions/msi1-7.tar.gz | tar -C "$HOME/msi/extensions/src" -xvz
        mv "$HOME/msi/extensions/src/msi1-7" "$HOME/msi/extensions/src/msi"

        cat << EOF > "$HOME/msi/extensions/configure/RELEASE"
EPICS_BASE=$EPICS_BASE
EPICS_EXTENSIONS=\$(TOP)
EOF
        make -C "$HOME/msi/extensions"
        cp "$HOME/msi/extensions/bin/$EPICS_HOST_ARCH/msi" "$EPICS_BASE/bin/$EPICS_HOST_ARCH/"
        echo 'MSI:=$(EPICS_BASE)/bin/$(EPICS_HOST_ARCH)/msi' >> "$EPICS_BASE/configure/CONFIG_SITE"

        cat <<EOF >> configure/CONFIG_SITE
MSI = \$(EPICS_BASE)/bin/\$(EPICS_HOST_ARCH)/msi
EOF

      ;;
    *) echo "Use MSI from Base"
      ;;
    esac

    touch $EPICS_BASE/built
else
    echo "Using cached epics-base!"
fi

# IPAC
if [ -n "$IPAC" ]; then
    if [ ! -e "$SUPPORT/ipac/built" ]; then
        echo "Build ipac"
        install -d $SUPPORT/ipac
        git clone --depth 10 --branch $IPAC https://github.com/epics-modules/ipac.git $SUPPORT/ipac
        cat << EOF > $SUPPORT/ipac/configure/RELEASE
EPICS_BASE=$SUPPORT/epics-base
EOF
        make -C $SUPPORT/ipac
        touch $SUPPORT/ipac/built
    else
        echo "Using cached ipac"
    fi
else
    echo "Skipping ipac"
fi


# sequencer
if [ ! -e "$SUPPORT/seq/built" ]; then
    echo "Build sequencer"
    install -d $SUPPORT/seq
    curl -L "http://www-csr.bessy.de/control/SoftDist/sequencer/releases/seq-${SEQ}.tar.gz" | tar -C $SUPPORT/seq -xvz --strip-components=1
    cp $RELEASE_PATH $SUPPORT/seq/configure/RELEASE
    make -C $SUPPORT/seq
    touch $SUPPORT/seq/built
else
    echo "Using cached seq"
fi


# asyn
if [ ! -e "$SUPPORT/asyn/built" ]; then
    echo "Build asyn"
    install -d $SUPPORT/asyn
    curl -L "https://epics.anl.gov/download/modules/asyn${ASYN}.tar.gz" | tar -C $SUPPORT/asyn -xvz --strip-components=1
    cp $RELEASE_PATH $SUPPORT/asyn/configure/RELEASE
    make -C "$SUPPORT/asyn" -j2
    touch $SUPPORT/asyn/built
else
    echo "Using cached asyn"
fi
