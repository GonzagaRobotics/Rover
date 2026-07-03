#!/bin/bash

# Use inside the first level of the Rover directory

CMD="colcon build --symlink-install"

case "$1" in
    "" )
        ;;
    * )
        CMD="$CMD --packages-up-to $1"
        ;;
esac

case "$2" in
    "" )
        ;;
    * )
        CMD="$CMD --cmake-args -DCMAKE_BUILD_TYPE=$2"
        ;;
esac

$CMD