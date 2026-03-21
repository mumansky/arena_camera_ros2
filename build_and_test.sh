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

# Add CUDA to PATH if available (required for nvcc / polarization kernel)
if [ -d "/usr/local/cuda-12.6/bin" ]; then
	export PATH="/usr/local/cuda-12.6/bin:${PATH}"
elif [ -d "/usr/local/cuda/bin" ]; then
	export PATH="/usr/local/cuda/bin:${PATH}"
fi

# Navigate to workspace
cd "${WORKSPACE_DIR}"

if [ "${CLEAN_BUILD}" = true ]; then
	TOTAL_STEPS=5
	echo "[1/5] Sourcing ROS2 Humble..."
	source /opt/ros/humble/setup.bash
	echo "[2/5] Cleaning workspace..."
	rm -rf build install log
	BUILD_STEP=3
	SOURCE_STEP=4
	TEST_STEP=5
else
	TOTAL_STEPS=4
	echo "[1/4] Sourcing ROS2 Humble..."
	source /opt/ros/humble/setup.bash
	BUILD_STEP=2
	SOURCE_STEP=3
	TEST_STEP=4
fi

# Build with tests enabled and compile commands exported
echo "[${BUILD_STEP}/${TOTAL_STEPS}] Building ${PKG_NAME} with tests..."
colcon build --symlink-install --packages-select "${PKG_NAME}" \
	--cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the install
echo "[${SOURCE_STEP}/${TOTAL_STEPS}] Sourcing install..."
source install/setup.bash

if [ "${RUN_TESTS}" = true ]; then
	# Print test results on exit (covers both success and early exit from set -e).
	trap 'echo ""; echo "=============================================="; echo "              Test Results Summary            "; echo "=============================================="; colcon test-result --verbose --all || true' EXIT

	echo "[${TEST_STEP}/${TOTAL_STEPS}] Running unit tests..."
	echo ""
	# Temporarily allow test failures so the trap always runs and prints results.
	set +e
	colcon test --packages-select "${PKG_NAME}" --event-handlers console_direct+
	set -e
else
	echo "[${TEST_STEP}/${TOTAL_STEPS}] Skipping tests (--no-test)"
fi

echo ""
echo "Done!"
