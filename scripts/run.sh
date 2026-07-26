#!/bin/bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
export AMENT_PREFIX_PATH="$WORKSPACE_DIR/install:$WORKSPACE_DIR/install/mobile_sensor_bridge:$AMENT_PREFIX_PATH"

ros2 run mobile_sensor_bridge mobile_sensor_bridge_node
