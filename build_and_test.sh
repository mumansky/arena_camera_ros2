#!/bin/bash
#
# build_and_test.sh
# Rebuilds the arena_camera_node package and runs unit tests with summary output
#

set -e

show_usage() {
	echo "Usage: $0 [--clean] [--no-test]"
	echo "  --clean    Remove build/install/log before building"
	echo "  --no-test  Skip running tests"
	echo "  -h, --help Show this help message"
}

CLEAN_BUILD=false
RUN_TESTS=true

for arg in "$@"; do
	case "$arg" in
		--clean)
			CLEAN_BUILD=true
			;;
		--no-test)
			RUN_TESTS=false
			;;
		-h|--help)
			show_usage
			exit 0
			;;
		*)
			echo "Unknown option: $arg"
			show_usage
			exit 1
			;;
	esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/ros2_ws"
PKG_NAME="${PKG:-arena_camera_node}"

echo "=============================================="
echo "  Arena Camera ROS2 - Build and Test Runner  "
echo "=============================================="
echo ""

# Source ROS2
echo "[1/4] Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd "${WORKSPACE_DIR}"

if [ "${CLEAN_BUILD}" = true ]; then
	echo "[2/5] Cleaning workspace..."
	rm -rf build install log
	CLEAN_STEP=1
else
	CLEAN_STEP=0
fi

BUILD_STEP=$((2 + CLEAN_STEP))
SOURCE_STEP=$((3 + CLEAN_STEP))
TEST_STEP=$((4 + CLEAN_STEP))
TOTAL_STEPS=$((4 + CLEAN_STEP))

# Build with tests enabled and compile commands exported
echo "[${BUILD_STEP}/${TOTAL_STEPS}] Building ${PKG_NAME} with tests..."
colcon build --symlink-install --packages-select "${PKG_NAME}" \
	--cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the install
echo "[${SOURCE_STEP}/${TOTAL_STEPS}] Sourcing install..."
source install/setup.bash

if [ "${RUN_TESTS}" = true ]; then
	# Ensure test results are printed even if tests fail
	trap 'echo ""; echo "=============================================="; echo "              Test Results Summary            "; echo "=============================================="; colcon test-result --verbose --all || true' EXIT

	# Run the tests
	echo "[${TEST_STEP}/${TOTAL_STEPS}] Running unit tests..."
	echo ""
	colcon test --packages-select "${PKG_NAME}" --event-handlers console_direct+

	# Show test results summary
	echo ""
	echo "=============================================="
	echo "              Test Results Summary            "
	echo "=============================================="
	colcon test-result --verbose --all
else
	echo "[${TEST_STEP}/${TOTAL_STEPS}] Skipping tests (--no-test)"
fi

echo ""
echo "Done!"
