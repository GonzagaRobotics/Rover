#!/bin/bash

# Use inside the first level of the Rover directory

case "$1" in
    "" )
        colcon build --symlink-install
        ;;
    * )
        colcon build --symlink-install --packages-up-to $*
        ;;
esac