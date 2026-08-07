#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

echo "Starting Mobile Sensor Bridge & SLAM Pipeline..."
ros2 launch mobile_sensor_bridge slam_launch.py
