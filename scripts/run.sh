#!/bin/bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Clean up old build/execution log directory before launching node
if [ -d "$WORKSPACE_DIR/log" ]; then
    rm -rf "$WORKSPACE_DIR/log"
fi

source /opt/ros/jazzy/setup.bash

# Support both standard colcon build (isolated) and merged-install layouts
export AMENT_PREFIX_PATH="$WORKSPACE_DIR/install:$WORKSPACE_DIR/install/mobile_sensor_bridge:$AMENT_PREFIX_PATH"

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

ros2 run mobile_sensor_bridge mobile_sensor_bridge_node
