# arena_camera_ros2
Arena Camera driver for ROS2 - forked and updated by Mark Umansky

# Requirements

  - OS       : Linux (x64/amd64/arm64) (Ubuntu 22.04)
  - ROS2     : Humble Hawksbill
  - ArenaSDK : https://thinklucid.com/downloads-hub/

# Dependencies

Core ROS2 packages (install once):

    sudo apt install ros-humble-camera-info-manager

# Getting Started

- Clone repo or download release

      git clone https://github.com/lucidvisionlabs/arena_camera_ros2.git

- Install ArenaSDK: https://thinklucid.com/downloads-hub/

- Build and run tests

      bash build_and_test.sh

  Or build without tests:

      bash build_and_test.sh --no-test

  For a clean rebuild:

      bash build_and_test.sh --clean

- Source the install and run

      source ros2_ws/install/setup.bash
      ros2 run arena_camera_node start

# Configuration

All parameters are read from `etc/arena_camera/camera.yaml` on startup — the single source of truth. Edit this file to set defaults without passing command-line arguments every time.

**Priority:** ROS arguments always override the config file.

**Config file location:** `arena_camera_ros2/etc/arena_camera/camera.yaml`

## Parameters

### Device Selection

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial` | string | _(first found)_ | Serial number of the camera to connect |

### Image Topic

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `topic` | string | `/arena_camera_node/images` | Base topic for all image publications |
| `frame_id` | string | `camera_optical_frame` | TF frame ID in all image headers |
| `use_camera_timestamp` | bool | `false` | Use camera hardware clock (requires PTP sync); default uses ROS clock |

### Resolution & Format

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `width` | int | _(camera default)_ | Image width in pixels |
| `height` | int | _(camera default)_ | Image height in pixels |
| `pixelformat` | string | _(camera default)_ | ROS pixel format string; see supported values below |

Supported pixel formats: `mono8`, `mono16`, `rgb8`, `rgba8`, `rgb16`, `rgba16`, `bgr8`, `bgra8`, `bgr16`, `bgra16`, `bayer_rggb8`, `bayer_bggr8`, `bayer_gbrg8`, `bayer_grbg8`, `bayer_rggb16`, `bayer_bggr16`, `bayer_gbrg16`, `bayer_grbg16`, `yuv422`, `polarized_angles_0d_45d_90d_135d_bayer_rg8`

### Exposure & Gain

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `gain` | double | _(camera default)_ | Manual gain in dB |
| `auto_gain` | string | — | `"Off"`, `"Once"`, `"Continuous"` |
| `exposure_time` | double | _(camera default)_ | Exposure time in microseconds |
| `auto_exposure` | string | — | `"Off"`, `"Once"`, `"Continuous"` |
| `short_exposure_enable` | bool | `false` | Allow exposure below the standard minimum |
| `exposure_auto_algorithm` | string | `"Mean"` | Auto exposure algorithm: `"Mean"` or `"Median"` |
| `target_brightness` | int | — | Auto exposure target brightness (0–255) |
| `exposure_auto_damping` | double | — | Auto exposure response speed (0–100; higher = slower) |
| `exposure_auto_limit_auto` | string | — | `"Off"` or `"Continuous"` for auto limit calculation |
| `exposure_auto_upper_limit` | double | — | Upper limit for auto exposure in microseconds |
| `exposure_auto_lower_limit` | double | — | Lower limit for auto exposure in microseconds |

### Frame Rate

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `acquisition_frame_rate_enable` | bool | `false` | Enable manual frame rate control |
| `acquisition_frame_rate` | double | — | Target FPS (requires `acquisition_frame_rate_enable: true`) |

### Publishing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_raw` | bool | `true` | Publish `sensor_msgs/Image` on `<topic>` |
| `publish_compressed` | bool | `false` | Publish JPEG `sensor_msgs/CompressedImage` on `<topic>/compressed` |
| `jpeg_quality` | int | `80` | JPEG compression quality (1–100) |
| `stream_buffer_count` | int | `10` | Number of internal SDK stream buffers |

### Polarized Camera Topics

Available only when `pixelformat: polarized_angles_0d_45d_90d_135d_bayer_rg8`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_pol_channels` | bool | `true` | Publish 4 per-angle BGR images (`/pol_0deg`, `/pol_45deg`, `/pol_90deg`, `/pol_135deg`) |
| `publish_pol_max` | bool | `true` | Publish pixel-wise max of all 4 channels (`/pol_max`) |
| `publish_dolp` | bool | `false` | Publish Degree of Linear Polarization as mono8 (`/dolp`) |
| `publish_aolp` | bool | `false` | Publish Angle of Linear Polarization as mono8 (`/aolp`) |
| `publish_stokes` | bool | `false` | Publish Stokes parameters S0/S1/S2 as mono8 (`/stokes_s0`, `/stokes_s1`, `/stokes_s2`) |
| `publish_aolp_color` | bool | `false` | Publish false-color AoLP HSV visualization as JPEG (`/aolp_color/compressed`). Requires `publish_dolp: true` and `publish_aolp: true`. |

**Stokes encoding:** S0 = (I₀ + I₉₀) × 0.5 → [0,255]. S1/S2 = (x + 255) / 2 → [0,255] where 128 = zero.

**AoLP color:** HSV colormap — H = polarization angle, S = DOLP (confidence), V = 200 (fixed).

All polarization topics respect `publish_raw` and `publish_compressed`.

### Camera Info / Calibration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_info_url` | string | `""` | Path to ROS calibration YAML, e.g. `file:///home/user/cal.yaml`. Empty = uncalibrated stub. |

`sensor_msgs/CameraInfo` is always published on `<topic>/camera_info` with the same header stamp as the image. See [docs/camera-calibration.md](docs/camera-calibration.md) for the full calibration workflow.

### Trigger Mode

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trigger_mode` | bool | `false` | When `true`, images are only published when the `trigger_image` service is called |

### Watchdog & Auto-Reconnect

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `watchdog_timeout_sec` | double | `5.0` | Seconds without a new frame before declaring the camera frozen. Set to `0` to disable. |
| `reconnect_max_attempts` | int | `0` | Max reconnection attempts after a freeze (`0` = infinite). When the camera is declared frozen the node automatically tears down and re-initializes the stream. |

### Latency Profiling

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `latency_mode` | string | `"off"` | `"off"`, `"callback"` (Arena OnImage → publish), `"hardware"` (camera HW clock → publish; requires PTP), or `"both"` |

Latency values are logged at DEBUG level and exposed in `/diagnostics`.

### GPU Acceleration (Orin/Jetson)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `gpu_acceleration` | string | `"auto"` | `"auto"` — use nvJPEG + CUDA kernel if available; `"gpu"` — force GPU; `"cpu"` — always use software path |

### Profiling & Debug

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `profile_processing` | bool | `false` | Log per-frame timing breakdown at DEBUG level |
| `display_images` | bool | `false` | Show OpenCV debug window (requires `DISPLAY`). Press `s` to save tiles, `q` to close. |

### QoS

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `qos_history` | string | SensorDataQoS | `"keep_last"`, `"keep_all"`, `"system_default"` |
| `qos_history_depth` | int | `5` | Queue depth (only with `keep_last`) |
| `qos_reliability` | string | `"best_effort"` | `"best_effort"` or `"reliable"` |

All publishers (images, camera_info, DOLP, AoLP, Stokes, etc.) share the same QoS profile.

---

# Published Topics

For a default polarized camera setup with all features enabled:

| Topic | Type | Condition |
|-------|------|-----------|
| `<topic>` | `sensor_msgs/Image` | `publish_raw: true` |
| `<topic>/compressed` | `sensor_msgs/CompressedImage` | `publish_compressed: true`, non-polarized only |
| `<topic>/camera_info` | `sensor_msgs/CameraInfo` | Always |
| `<topic>/pol_0deg` | `sensor_msgs/Image` | `publish_pol_channels: true`, `publish_raw: true` |
| `<topic>/pol_0deg/compressed` | `sensor_msgs/CompressedImage` | `publish_pol_channels: true`, `publish_compressed: true` |
| `<topic>/pol_45deg[/compressed]` | … | Same as above |
| `<topic>/pol_90deg[/compressed]` | … | Same as above |
| `<topic>/pol_135deg[/compressed]` | … | Same as above |
| `<topic>/pol_max[/compressed]` | … | `publish_pol_max: true` |
| `<topic>/dolp[/compressed]` | … | `publish_dolp: true` |
| `<topic>/aolp[/compressed]` | … | `publish_aolp: true` |
| `<topic>/stokes_s0[/compressed]` | … | `publish_stokes: true` |
| `<topic>/stokes_s1[/compressed]` | … | `publish_stokes: true` |
| `<topic>/stokes_s2[/compressed]` | … | `publish_stokes: true` |
| `<topic>/aolp_color/compressed` | `sensor_msgs/CompressedImage` | `publish_aolp_color: true` |

---

# Services

| Service | Type | Description |
|---------|------|-------------|
| `<node_name>/trigger_image` | `std_srvs/Trigger` | Capture and publish one image (requires `trigger_mode: true`) |

---

# Example Usage

```bash
# Default — settings from camera.yaml
ros2 run arena_camera_node start

# Override topic and serial
ros2 run arena_camera_node start --ros-args -p serial:="904240001" -p topic:=/camera

# Polarized camera with DOLP/AoLP/Stokes/color AoLP
ros2 run arena_camera_node start --ros-args \
  -p pixelformat:=polarized_angles_0d_45d_90d_135d_bayer_rg8 \
  -p publish_dolp:=true \
  -p publish_aolp:=true \
  -p publish_stokes:=true \
  -p publish_aolp_color:=true

# Trigger mode
ros2 run arena_camera_node start --ros-args -p trigger_mode:=true -p exposure_time:=5000.0
ros2 run arena_camera_node trigger_image

# View images
ros2 run image_tools showimage --ros-args -r image:=/arena_camera_node/images

# Compressed-only (save bandwidth)
ros2 run arena_camera_node start --ros-args -p publish_raw:=false -p publish_compressed:=true
```

---

# Architecture

## Node Lifecycle

1. **Config loading** — `camera.yaml` is parsed as the parameter source of truth.
2. **System init** — Arena SDK system opened; device discovery polling starts (1 s timer).
3. **Streaming** — Once a camera is found:
   - **Continuous mode** (default): Arena SDK callback (`RegisterImageCallback`) → fast copy in `handle_camera_image_()` → worker thread processes and publishes.
   - **Trigger mode**: Blocking `GetImage()` on service call.
4. **Watchdog** — Timer in `produce_diagnostics_()` detects frozen camera. On freeze, `reconnect_()` is started on a detached thread.
5. **Auto-reconnect** — Structured teardown (stop worker → deregister callback → StopStream → DestroyDevice) followed by re-discovery, re-configuration, and stream restart. Reconnect count and errors are shown in `/diagnostics`.
6. **Shutdown** — Destructor mirrors the teardown sequence.

## Image Processing Pipeline (polarized camera)

```
Arena SDK callback
  └─ ImageFactory::Copy  →  worker thread queue
       └─ process_copied_image_()
            ├─ publish raw Image (publish_raw)
            ├─ publish CompressedImage (publish_compressed, non-polarized)
            ├─ publish CameraInfo  (always)
            ├─ measure latency     (latency_mode != "off")
            └─ ImageFactory::SplitChannels → 4 mono8 planes
                 ├─ Bayer→BGR for each channel  (publish_pol_channels / publish_pol_max)
                 ├─ GPU path: fused CUDA kernel → DOLP, AoLP, S0/S1/S2
                 │   (polar_compute_gpu — PolarKernelBuffers on device)
                 ├─ CPU fallback: OpenCV Stokes → DOLP, AoLP, S0/S1/S2
                 └─ publish DOLP, AoLP, Stokes, AoLP-color
```

## GPU Acceleration (Orin/Jetson)

When `HAS_CUDA` is defined at build time and a CUDA device is available at runtime:

- **nvJPEG** hardware JPEG encoding replaces `cv::imencode` for all compressed topics.
- **Fused CUDA kernel** (`polar_kernel`) computes Stokes parameters, DOLP, AoLP, and (optionally) S0/S1/S2 outputs in a single GPU pass, replacing ~60 ms of CPU work per frame.

Set `gpu_acceleration: "cpu"` to force the software path for benchmarking comparisons.

## Diagnostics

The node publishes to `/diagnostics` via `diagnostic_updater`. Fields include: device connection state, images published, publish errors, incomplete frames, FPS, backpressure events, processing time statistics, watchdog state, reconnect count, and (when enabled) latency measurements.

---

# Running Tests

```bash
bash build_and_test.sh          # build + test
bash build_and_test.sh --clean  # clean rebuild + test
bash build_and_test.sh --no-test
```

Or manually:

```bash
cd ros2_ws
colcon test --packages-select arena_camera_node
colcon test-result --verbose
```

---

# Documentation

| Document | Description |
|----------|-------------|
| [docs/camera-calibration.md](docs/camera-calibration.md) | End-to-end checkerboard calibration guide |
| [docs/performance-roadmap.md](docs/performance-roadmap.md) | GPU optimization history and profiling notes |
| [docs/orin-deployment.md](docs/orin-deployment.md) | NVIDIA Orin / JetPack deployment notes |

---

# Road Map

- Support Windows
- Support multiple cameras simultaneously
- Dynamic reconfigure for runtime parameter changes
- Add `-h` flag to node executables
