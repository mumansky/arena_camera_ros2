# Porting arena_camera_ros2 to NVIDIA Orin NX (PX4 carrier board, headless drone)

## Context

The current driver runs on a development x86-64 machine. The target is an NVIDIA Orin NX
on a PX4 carrier board (e.g., Holybro Pixhawk 6X Jetson baseboard or similar), running
headless, feeding image inference or navigation nodes. The C++ source code is fully
architecture-neutral — the only blocker is the ArenaSDK binary, which is currently
x86-64 only.

---

## Step 1 — Obtain ARM64 ArenaSDK from LUCID

LUCID ships a separate ARM64 tarball: `ArenaSDK_Linux_aarch64_vX.X.X.tar.gz`

- Email support@thinklucid.com with your camera model (PHX050S1-Q) and request the
  aarch64 Linux SDK. It is not publicly listed but is routinely provided.
- Expected install location: `/opt/ArenaSDK_Linux_ARM64/` (or similar)

The `cmake/Findarena_sdk.cmake` **already has** ARM64 detection logic:
```cmake
elseif(EXISTS .../GenICam/library/lib/Linux64_ARM/libGCBase_gcc54_v3_3_LUCID.so)
    set(ArenaSDK_Build "Linux64_ARM")
```
No CMake changes are required — just ensure the SDK is installed and the ld.so.conf
points to the right paths.

---

## Step 2 — Prepare the Orin NX

### OS / JetPack
- JetPack 6.x → Ubuntu 22.04 aarch64 (recommended; ships with CUDA, TensorRT, OpenCV)
- JetPack 5.x → Ubuntu 20.04 aarch64 (also fine; use ROS2 Humble or Foxy)

### ROS2
```bash
# On Orin — standard ROS2 Humble install for arm64
sudo apt install ros-humble-ros-base   # headless; no desktop GUI needed
sudo apt install ros-humble-image-transport ros-humble-compressed-image-transport
sudo apt install ros-humble-diagnostic-updater libyaml-cpp-dev
```

### GigE Network Interface
The PX4 carrier board ethernet port must be configured for jumbo frames:
```bash
sudo ip link set eth0 mtu 9000    # replace eth0 with correct interface name
# Make persistent — add to /etc/networkd-dispatcher/configured.d/ or netplan
```
Run the node — the built-in GigE health check will report "jumbo frames OK" or warn.

### GigE static IP
Configure the camera-facing NIC with a static IP in the camera's subnet (172.24.0.x):
```yaml
# /etc/netplan/01-camera.yaml
network:
  ethernets:
    eth0:
      dhcp4: false
      addresses: [172.24.0.1/24]
      mtu: 9000
```

---

## Step 3 — Install ARM64 ArenaSDK

Mirror the existing `resources/ArenaSDK/linux64/Arena_SDK_reinstall_on_linux_64.sh`
but adjust paths for ARM64:
- `lib/` instead of `lib64/`
- `Linux64_ARM` instead of `Linux64_x64`
- Install root: `/opt/ArenaSDK_Linux_ARM64`

The `/etc/ld.so.conf.d/Arena_SDK.conf` is read by `cmake/Findarena_sdk.cmake` to
auto-detect the SDK root — the install script must write the correct paths to that file:

```bash
# /etc/ld.so.conf.d/Arena_SDK.conf content for ARM64:
/opt/ArenaSDK_Linux_ARM64/lib
/opt/ArenaSDK_Linux_ARM64/GenICam/library/lib/Linux64_ARM
/opt/ArenaSDK_Linux_ARM64/ffmpeg
```

---

## Step 4 — Build on the Orin

Build natively on the Orin (no cross-compilation needed; Orin NX is fast enough):

```bash
cd ~/arena_camera_ros2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If CMake cannot find the SDK, set the root explicitly:
```bash
colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -Darena_sdk_installation_root=/opt/ArenaSDK_Linux_ARM64
```

---

## Step 5 — Update camera.yaml for headless drone operation

**File:** `etc/arena_camera/camera.yaml`

Key changes from the current defaults:

| Parameter | Current | Drone headless |
|-----------|---------|----------------|
| `display_images` | `true` | **`false`** (no display) |
| `use_camera_timestamp` | `false` | Keep `false` — ROS clock is correct for IMU fusion |
| `frame_id` | `camera_optical_frame` | Match your URDF/TF tree |
| `acquisition_frame_rate` | `10` | Tune to inference throughput (10-30 fps) |
| `publish_compressed` | off | **Enable** — reduces DDS bandwidth between nodes |
| `jpeg_quality` | `50` | 50–70 for inference; lower for bandwidth-limited links |

The node already warns and auto-disables `display_images` if `$DISPLAY` is unset,
but it's cleaner to set it explicitly false in config.

---

## Step 6 — PX4 / ROS2 integration

### ROS2 ↔ PX4 bridge
PX4 v1.14+ supports **uXRCE-DDS** natively — recommended over MAVROS for new projects:
```bash
# On Orin:
sudo apt install ros-humble-px4-msgs ros-humble-px4-ros-com
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```
This publishes PX4 topics (`/fmu/out/vehicle_imu`, `/fmu/out/vehicle_local_position`,
etc.) directly into the ROS2 graph alongside camera images.

### Timestamp synchronization
- Keep `use_camera_timestamp: false` — ROS clock (`this->now()`) on all sensors
- For tighter IMU-camera sync: investigate hardware trigger mode (PHX050S1 supports it)
  or software sync via the existing `/trigger_image` service

---

## Step 7 — Downstream node integration

### Image inference node (e.g., TensorRT/DeepStream on Orin)
- Subscribe to `/arena_camera_node/images/pol_0deg` (raw) or `/pol_0deg/compressed`
- Use `best_effort` QoS (already the camera default) — avoids backpressure stalling the camera
- For zero-copy on Orin: run camera + inference in the same process with ROS2
  intra-process communication, or use CycloneDDS + Iceoryx shared memory transport

### Navigation node
- Subscribe to polarization channels, DOLP (`/images/dolp`), AoLP (`/images/aolp`) as needed
- Camera TF frame (`camera_optical_frame`) must be in URDF/TF tree relative to `base_link`

### Recommended QoS for inference consumers
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # only latest frame matters for real-time inference
)
```

---

## Step 8 — Auto-start on boot (systemd)

```ini
# /etc/systemd/system/arena_camera_node.service
[Unit]
Description=Arena Camera ROS2 Node
After=network-online.target
Wants=network-online.target

[Service]
User=mark
WorkingDirectory=/home/mark/arena_camera_ros2/ros2_ws
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && \
  source /home/mark/arena_camera_ros2/ros2_ws/install/setup.bash && \
  ros2 run arena_camera_node start --ros-args \
    --params-file /home/mark/arena_camera_ros2/etc/arena_camera/camera.yaml"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable arena_camera_node
sudo systemctl start arena_camera_node
journalctl -u arena_camera_node -f   # monitor logs
```

---

## Verification

1. On Orin: `ros2 run arena_camera_node start ...` → check for "Streaming started" and no WARN
2. `ros2 topic hz /arena_camera_node/images/pol_0deg/compressed` → expect configured rate
3. `ros2 topic echo /diagnostics` → "Camera operating normally", 0 incomplete frames
4. Check timestamps align with IMU: compare `/arena_camera_node/images/pol_0deg header.stamp`
   vs `/fmu/out/vehicle_imu header.stamp` — should be within a few milliseconds
5. Clean shutdown: `sudo systemctl stop arena_camera_node` → journalctl shows
   "Destroyed arena_camera_node node" with no segfault (exit code 0)
