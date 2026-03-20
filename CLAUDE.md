# CLAUDE.md — Arena Camera ROS2 Driver

## What this repo is
ROS2 C++ driver for LUCID Arena cameras (PHX050S1-QC polarization camera). Used in PhD drone research with NVIDIA Orin + PX4, requiring tight IMU/LiDAR sensor fusion. Core implementation: `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.{cpp,h}`.

## Architecture
Event-driven, two-thread design:
1. **ArenaSDK grab thread** → `handle_camera_image_()` (fast: deep-copy image, post to mailbox, return)
2. **Worker thread** → `process_copied_image_()` (heavy: polarization processing, publish, diagnostics)

Single-slot mailbox with drop-on-backpressure between threads. There is also `publish_images_()` (~260 lines) which is **dead code** — never called in the callback architecture, do not resurrect it.

## Build / Test / Run

```bash
# Build (from repo root)
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Test (no physical camera required)
colcon test --packages-select arena_camera_node
colcon test-result --verbose

# Run node
ros2 run arena_camera_node start --ros-args --params-file etc/arena_camera/camera.yaml

# Trigger a shot (in trigger mode)
ros2 run arena_camera_node trigger_image
```

Four GTest suites: `test_pixelformat_translation`, `test_qos_translation`, `test_arena_image_raii`, `test_config_helpers`.

## Code Conventions
- **Formatting**: `clang-format` with `.clang-format` at repo root (Google style, 2-space indent). Run `clang-format -i <file>` before committing.
- **Logging**: Use `log_info`, `log_debug`, `log_warn`, `log_err` wrappers — not raw `RCLCPP_*` macros.
- **Parameters**: `declare_parameter<T>()` / `get_parameter()`. Presence flags follow the `is_passed_*` naming pattern. Update `parse_parameters_()` when adding parameters.
- **QoS & pixel format mappings**: live in `rclcpp_adapter/` — use those, don't invent new ones.
- **Device ownership**: Arena System and device use `shared_ptr` with custom deleters. Be careful with ownership when refactoring.
- **RAII**: `arena_image_raii.h` provides `ArenaImagePtr` / `ArenaImageVector` — use these for Arena image lifetime management.

## Key Files
| Path | Purpose |
|------|---------|
| `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` | Main node (2000+ lines) |
| `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.h` | Node class declaration |
| `ros2_ws/src/arena_camera_node/src/rclcpp_adapter/` | Pixel-format & QoS translation maps |
| `ros2_ws/src/arena_camera_node/src/light_arena/` | Device info helpers |
| `etc/arena_camera/camera.yaml` | All runtime parameters (single source of truth) |
| `ros2_ws/src/arena_camera_node/CMakeLists.txt` | Build config — do not move `cmake/` helper |

## Known Bugs (confirmed, not yet fixed)
Fix these before relying on the driver in production:

| Line(s) | Bug | Fix |
|---------|-----|-----|
| ~525, ~544 | Bare `throw;` outside catch → UB/terminate | `throw std::invalid_argument(...)` |
| ~1712 | Missing `return` in trigger handler → falls through to fire trigger unconditionally | Add `return;` |
| trigger loop | `waitForTriggerCount` never decrements → infinite busy-wait, pins CPU | Add sleep + max retries |
| grab/worker threads | Data race on `m_fps_frame_count_`, `m_last_frame_time_`, `m_images_published_` (plain types, not atomic) | Make atomic or consolidate ownership |
| process path | No `pImage->IsIncomplete()` check → corrupted GigE frames published silently | Add check + warning + diagnostic counter |
| all timestamps | Camera hardware clock used by default → breaks IMU/sensor fusion unless PTP configured | Default to `this->now()` (config: `use_camera_timestamp: false`) |
| ~1858 | `auto index = 0` should be `size_t` | Fix type |
| trigger handler | `std::exception` caught before `GenICam::GenericException` → GenICam catch is dead code | Swap catch order |
| startup | `AcquisitionMode` never set → relies on camera's UserSetDefault | Set `"Continuous"` explicitly before `StartStream()` |
| `m_processing_image_` | Atomic bool initialized false, never set true → backpressure flag is dead | Either use it or remove it |

## Important Patterns
- **Sensor fusion timestamps**: Always use `this->now()` unless PTP is configured. The `use_camera_timestamp: false` default in `camera.yaml` is intentional.
- **frame_id**: Must be a TF frame name (e.g., `"camera_optical_frame"`), not the integer frame counter.
- **Polarization channels**: 4 channels at 0°, 45°, 90°, 135°. DOLP and AoLP derived from Stokes parameters.
- **Pixel format code**: `0x8220020F` = polarized format, `0x02180015` = BayerRG8.

## What NOT to do
- Do not move or rename `cmake/` or remove `find_package(arena_sdk)`.
- Do not reactivate `publish_images_()` — it's dead code, the callback architecture replaced it.
- Do not add multi-device support without discussing architecture changes first (current code assumes single device lifecycle).
- Do not use `std::cout` — use the `log_*` wrappers.
- Do not change install paths in CMakeLists without updating all references.

## External Dependencies
- **ArenaSDK C++**: expected at `/opt/ArenaSDK_Linux_x64` (x86) or `/opt/ArenaSDK_Linux_ARM64` (Orin). `LD_LIBRARY_PATH` must include SDK libs (handled by `arena_camera_ros_entrypoint.sh`).
- **ROS2 Humble**: `rclcpp`, `sensor_msgs`, `std_srvs`, `diagnostic_updater`.
- Docker entrypoint: `arena_camera_ros_entrypoint.sh` — preserve `LD_LIBRARY_PATH` manipulation when editing.

## Reference Docs
- `ENGINEERING_TASKS.md` — 35 prioritized engineering tasks (bugs, SDK compliance, features)
- `CODE_REVIEW.md` — detailed code review findings
- `ros2_ws/src/arena_camera_node/doc/FRAME_RATE_CONFIGURATION.md` — frame rate / MTU tuning guide
- `docs/orin-deployment.md` — NVIDIA Orin deployment notes
