## Purpose
Short guidance to help an AI code assistant be productive in the arena_camera_ros2 repository.

Keep answers and edits tightly scoped. Prefer small, safe changes: implement missing features, fix obvious bugs, add tests or docs for a touched area. Avoid broad refactors unless the user asks.

## Big picture (what this repo is)
- This repository provides a ROS2 driver for LUCID (Arena) cameras. The core runtime component is a C++ ROS2 package `arena_camera_node` under `ros2_ws/src/arena_camera_node` that uses the ArenaSDK C++ API to discover devices, stream images, and expose a trigger service.
- Key runtime pieces:
  - `ros2_ws/src/arena_camera_node` — C++ package (CMake/ament) that builds two executables: `start` (main node) and `trigger_image` (client/service helper).
  - `ArenaSDK` (external C++ SDK). The CMakeLists expects a findable `arena_sdk` package and uses a local `cmake/` helper (see CMakeLists `CMAKE_MODULE_PATH`).
  - Shell helpers: `ros2_arena_setup.sh` (installs ROS2 on Ubuntu Humble), `arena_camera_ros_entrypoint.sh` (container entrypoint that sets LD_LIBRARY_PATH and sources environments).
  - Docker support and examples are provided; the repo may bundle Arena SDK artifacts in `ArenaSDK_Linux_*` directories.

## Where to look (most important files)
- Build and package: `ros2_ws/src/arena_camera_node/CMakeLists.txt`, `package.xml`
- Node implementation: `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp`, `ArenaCameraNode.h`, `main.cpp`
- Adapters and mappings: `ros2_ws/src/arena_camera_node/src/rclcpp_adapter/` (pixel-format & QoS translation maps)
- Device helpers: `ros2_ws/src/arena_camera_node/src/light_arena/deviceinfo_helper.*`
- Setup and run scripts: `ros2_arena_setup.sh`, `arena_camera_ros_entrypoint.sh`, `ros2_ws/usefull_cmnds.txt`
- Workspace build: `ros2_ws/` (use `colcon`), `.vscode/launch.json` and `tasks.json` include convenient dev run configs.

## Build / test / run workflows (explicit)
- Native dev (Ubuntu Humble):
  1. Install ROS2 (script provided): `sudo sh ros2_arena_setup.sh` (runs apt installs and rosdep init).
  2. Ensure ArenaSDK is installed or available at the path expected by CMake (commonly under `/opt/ArenaSDK_*` or the repo's `ArenaSDK_Linux_*`). The node adds `cmake/` to CMAKE_MODULE_PATH — do not remove that helper.
  3. From repo root:
     - `source /opt/ros/humble/setup.bash`
     - `cd ros2_ws`
     - `rosdep update`
     - `rosdep install --from-paths src --ignore-src --rosdistro humble -r -y`
     - `colcon build --symlink-install`
     - `source install/setup.bash`

- Quick run examples (exact invocations appear in README and useful commands file):
  - Start node (defaults to first discovered camera):
    - `ros2 run arena_camera_node start --ros-args -p topic:=/arena_camera_node/images`
  - Start in trigger mode (won't publish until triggered):
    - `ros2 run arena_camera_node start --ros-args -p trigger_mode:=true -p exposure_time:=150`
    - Trigger a shot: `ros2 run arena_camera_node trigger_image`

- Docker / container notes:
  - The repo includes entrypoint scripts that set `LD_LIBRARY_PATH` so ArenaSDK libs are found. When editing Dockerfiles or entrypoints, preserve `LD_LIBRARY_PATH` manipulation (see `arena_camera_ros_entrypoint.sh`).

## Project-specific conventions and patterns
- Parameters: the node uses `declare_parameter` and checks presence with flags like `is_passed_*`. When adding parameters, follow the same pattern and update `parse_parameters_()`.
- Logging: wrapper methods `log_info`, `log_debug`, `log_warn`, `log_err` are used instead of direct RCLCPP macros — keep using those wrappers for consistent messages.
- QoS & pixel-format: mappings live under `rclcpp_adapter/`. Use those translation maps (`K_ROS2_PIXELFORMAT_TO_PFNC`, etc.) rather than inventing new mappings.
- Device lifecycle: the node creates an Arena System and device with custom deleters stored in shared_ptrs — be careful when refactoring ownership or adding multi-device support.
- Executables are installed to `lib/arena_camera_node/<name>` and invoked via `ros2 run arena_camera_node <executable>`. Don't change install paths without updating `CMakeLists.txt`.

## Integration points & external deps
- ArenaSDK C++ (critical). The CMake build expects `find_package(arena_sdk REQUIRED)`; the runtime loads Arena SDK shared libs (LD_LIBRARY_PATH must include their folder).
- ROS2 Humble (rclcpp, sensor_msgs, std_srvs, diagnostic_updater). Code assumes Humble APIs (e.g., `declare_parameter`, QoS classes).
- The repo bundles or references ArenaSDK artifacts in `ArenaSDK_Linux_*` directories and provides an `arena_api` wheel in `lucid_camera_driver_ros2` for Python integrations — mention this when adding Python wrappers.

## Examples of safe changes an assistant can make
- Fix small bugs in `ArenaCameraNode.cpp` (e.g., parameter parsing edge cases, exception handling around device creation).
- Add unit/compile-time guards or small tests if covering local translation functions in `rclcpp_adapter/`.
- Improve README snippets and add runnable examples to `ros2_ws/usefull_cmnds.txt`.

## Things to avoid or ask about before changing
- Do not move or rename the `cmake/` helper or remove the `find_package(arena_sdk)` usage unless you update the CMake flow and related instructions.
- For multi-device support or major lifecycle changes (concurrent device handling), ask the maintainer first; current code assumes single-device lifecycle.

If any of these areas are unclear or you want me to expand a specific section (examples, adding tests, or a small refactor), tell me which files to edit or what behavior to implement next.
