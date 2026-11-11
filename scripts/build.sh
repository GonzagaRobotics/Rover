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

CMD="$CMD --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release"

$CMD