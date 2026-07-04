#!/bin/bash

# There are a few options for what gets published
if [[ "$1" == "gnss" ]]; then
    ros2 topic pub -1 -w 0 /fix sensor_msgs/NavSatFix '{latitude: 38.40645261293369, longitude: -110.79137336968033}'
    ros2 topic pub -1 -w 0 /imu sensor_msgs/Imu '{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
fi