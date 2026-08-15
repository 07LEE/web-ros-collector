#!/bin/bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
if [ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
fi

# Support both standard colcon build (isolated) and merged-install layouts
export AMENT_PREFIX_PATH="$WORKSPACE_DIR/install:$WORKSPACE_DIR/install/mobile_sensor_bridge:$AMENT_PREFIX_PATH"

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

exec ros2 launch mobile_sensor_bridge sensor_bridge_launch.py "$@"
