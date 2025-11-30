#!/bin/bash
#
# build_and_test.sh
# Rebuilds the arena_camera_node package and runs unit tests with summary output
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/ros2_ws"

echo "=============================================="
echo "  Arena Camera ROS2 - Build and Test Runner  "
echo "=============================================="
echo ""

# Source ROS2
echo "[1/4] Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd "${WORKSPACE_DIR}"

# Build with tests enabled
echo "[2/4] Building arena_camera_node with tests..."
colcon build --symlink-install --packages-select arena_camera_node --cmake-args -DBUILD_TESTING=ON

# Source the install
echo "[3/4] Sourcing install..."
source install/setup.bash

# Run the tests
echo "[4/4] Running unit tests..."
echo ""
colcon test --packages-select arena_camera_node --event-handlers console_direct+

# Show test results summary
echo ""
echo "=============================================="
echo "              Test Results Summary            "
echo "=============================================="
colcon test-result --verbose --all

echo ""
echo "Done!"
