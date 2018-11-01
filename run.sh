#!/bin/bash
set -e

# Get current directory
SCRIPT_DIR="$(pwd)"

# Get build folder
BUILD_FOLDER="$SCRIPT_DIR/build"

if [ ! -d "$BUILD_FOLDER" ] ; then
    echo "You must build first! Use:"
    echo "$ ./build.sh"
    exit 1
fi

BINARY_NAME=mpc

if [ ! -f "$BUILD_FOLDER/$BINARY_NAME" ] ; then
    echo "Could not find binary \"$BINARY_NAME\" in $BUILD_FOLDER"
    echo "Make sure you build successfully with:"
    echo "$ ./build.sh"
    exit 1
fi

# Set Docker run base command
DOCKER_IMG_NAME=nfisher89/carnd-mpc
DOCKER_RUN_BASE="docker run -it                             \
                            -v $SCRIPT_DIR:$SCRIPT_DIR      \
                            -w $BUILD_FOLDER                \
                            -p 4567:4567                    \
                            $DOCKER_IMG_NAME"

# Run binary from within Docker
$DOCKER_RUN_BASE ./$BINARY_NAME
