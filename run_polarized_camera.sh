#!/bin/bash
#
# run_polarized_camera.sh
# Runs the arena camera driver with polarization pixel format (PHX050S1-Q camera)
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/ros2_ws"

echo "=============================================="
echo "  Arena Camera ROS2 - Polarized Camera Mode  "
echo "=============================================="
echo ""

# Source ROS2
echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Source the workspace
echo "Sourcing workspace..."
source "${WORKSPACE_DIR}/install/setup.bash"

# Default values (can be overridden via environment variables or arguments)
PIXELFORMAT="${PIXELFORMAT:-polarized_angles_0d_45d_90d_135d_bayer_rg8}"
TOPIC="${TOPIC:-/arena_camera_node/images}"
LOG_LEVEL="${LOG_LEVEL:-info}"

echo ""
echo "Configuration:"
echo "  Pixel Format: ${PIXELFORMAT}"
echo "  Topic:        ${TOPIC}"
echo "  Log Level:    ${LOG_LEVEL}"
echo ""
echo "Starting camera node..."
echo "(Press Ctrl+C to stop)"
echo ""

# Run the camera node with polarization format
ros2 run arena_camera_node start --ros-args \
  -p pixelformat:="${PIXELFORMAT}" \
  -p topic:="${TOPIC}" \
  --log-level "${LOG_LEVEL}"
