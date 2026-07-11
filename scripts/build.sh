#!/bin/bash

# Use inside the first level of the Rover directory

CMD="colcon build --symlink-install"

case "$*" in
    "" )
        ;;
    * )
        CMD="$CMD --packages-up-to $*"
        ;;
esac

CMD="$CMD --cmake-args -DCMAKE_BUILD_TYPE=Release"

$CMD