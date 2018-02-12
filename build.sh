#!/bin/bash
# Ensure we aren't building from the repo root
if [ ! -d src ]; then
	echo "Cannot build from root"
	exit
fi

if [ -z $BUILD_DIR ] || [ -z $BOARD ]; then
	echo "Build variables not set. Call build.sh in a board directory"
	exit
fi

# TODO: Change this to a command that moves the repo out of Downloads
if [[ "$PWD" = */Downloads/* ]] || [[ "$PWD" = */downloads/* ]] ; then
	figlet "Don't clone to Downloads, Will"
fi

# Build the board image
if [ ! $1 ] || [ $1 = build ]; then

	if [ ! -f makefile-mbed ]; then
		echo "Setting up local environment"
		ln -s ../shared/mbed/$BUILD_DIR $BUILD_DIR
		ln -s ../shared/mbed/mbed-os mbed-os
		ln -s ../shared/mbed/.mbed .mbed
		ln -s ../shared/mbed/makefile-mbed makefile-mbed
		ln -s ../shared/mbed/mbed_config.h mbed_config.h
		ln -s ../shared/mbed/mbed_settings.py mbed_settings.py
	else
		echo "Local environment already set up"
	fi

	# Setup files for build
	echo "Setting up build environment"
	mkdir -p ../shared/mbed/$BUILD_DIR/ubcr_$BOARD
	cp -r src/* ../shared/mbed/$BUILD_DIR/ubcr_$BOARD

	mkdir -p ../shared/mbed/$BUILD_DIR/ubcr_shared
	find ../shared/ -type d | egrep -v "(mbed|^..\/shared\/$)" | xargs -ILIST cp -r LIST ../shared/mbed/$BUILD_DIR/ubcr_shared

	# Find source files
	C_FILES=$(find ./src/* | egrep "\.c$")
	CPP_FILES=$(find ./src/* | egrep "\.cpp$")
	O_FILES=""
	O_FILES=$O_FILES$(echo $C_FILES | sed -e 's/.c/.o/g')
	O_FILES=$O_FILES$(echo $CPP_FILES | sed -e 's/.cpp/.o/g')
	O_FILES=$(echo $O_FILES | sed -e 's/src\//ubcr_'$BOARD'\//g')

	# Build
	echo Building $BOARD board
	make -j16 -f makefile-mbed \
     	 UBCR_TARGETDIR=$BUILD_DIR \
         UBCR_SHAREDDIR=ubcr_shared \
	     UBCR_O_FILES=$O_FILES

	if [ ! $? -eq 0 ]; then
		echo -e '\033[0;31m'
		figlet "Build Failed"
		echo -e '\033[0m'
		exit -1
	else
		echo -e '\033[0;32m'
		figlet "Build Successful!"
		echo -e '\033[0m'
		cp $BUILD_DIR/mbed.bin "../builds/"$BOARD.bin
		exit 0
	fi

# Flash the image to a board
elif [ $1 = flash ]; then
	echo "Flashing unimplemented"
	exit 1

# Cleanup all the build artefacts
elif [ $1 = clean ]; then
	if [ ! -f makefile-mbed ]; then
		echo "You can't clean until you build"
		exit 1
	fi

	rm -r $BUILD_DIR/*
	rm $BUILD_DIR
	rm mbed-os
	rm .mbed
	rm makefile-mbed
	rm mbed_config.h
	rm mbed_settings.py
fi
