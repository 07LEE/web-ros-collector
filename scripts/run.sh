#!/bin/bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
export AMENT_PREFIX_PATH="$WORKSPACE_DIR/install/camera_bridge:$AMENT_PREFIX_PATH"

ros2 run camera_bridge camera_bridge_node
