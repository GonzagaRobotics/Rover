#!/bin/bash

# Use inside the first level of the Rover directory. The only parameter is the name of the node

if [[ ! -d install/ ]]; then
    echo "Install directory does not exist!"
    exit 1
fi

source install/local_setup.bash
ros2 run $1 $1 $*