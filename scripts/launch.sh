#!/bin/bash

# Use inside the first level of the Rover directory. The only parameter is the name of the launch file (eg jetson, teleop, all)

if [[ ! -d install/ ]]; then
    echo "Install directory does not exist!"
    exit 1
fi

source install/local_setup.bash
ros2 launch launch/$1.launch.py