#!/bin/bash
set -e

# Get current directory
SCRIPT_DIR="$(pwd)"

echo "$SCRIPT_DIR"

# Create build folder
BUILD_FOLDER="$SCRIPT_DIR/build"

if [ ! -d $BUILD_FOLDER ] ; then
    mkdir $BUILD_FOLDER
fi

# Set Docker run base command
DOCKER_IMG_NAME=nfisher89/carnd-mpc
DOCKER_RUN_BASE="docker run --volume=$SCRIPT_DIR:$SCRIPT_DIR     \
                            --workdir=$BUILD_FOLDER              \
                            --user=$UID:$GROUPS                  \
                            $DOCKER_IMG_NAME"

# Run CMake from Docker image
$DOCKER_RUN_BASE cmake ..

# Run make from Docker image
$DOCKER_RUN_BASE make
