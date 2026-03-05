# Camera Calibration Guide

This guide covers how to calibrate the LUCID Arena camera and load the resulting
calibration file into the node so it is published on `<topic>/camera_info`.

---

## Overview

Camera calibration computes the intrinsic matrix **K** (focal length, principal point),
distortion coefficients **D**, and optionally the rectification matrix **R** and
projection matrix **P**.  Without calibration the node still publishes a valid
`sensor_msgs/CameraInfo` message but with all-zero K/D/R/P, which is the standard
ROS way of signalling "uncalibrated".

The standard ROS2 tool for monocular calibration is `camera_calibration` from the
`image_pipeline` package.  It uses a checkerboard or ChArUco target and the
OpenCV calibration backend.

---

## 1. Install the calibration tool

```bash
sudo apt install ros-humble-camera-calibration
```

---

## 2. Print (or display) a calibration target

The tool supports two target types:

| Type | Recommended size | Notes |
|------|-----------------|-------|
| Checkerboard | 8×6 inner corners, 25 mm squares | Classic; works well indoors |
| ChArUco | Any size | More robust; works even with partial occlusion |

For a 5 MP polarized camera (PHX050S1-QC) a **9×7 checkerboard with 30 mm squares**
on A3 paper is a good starting point.  The board must be **flat** — foam board or
aluminium sheet is better than paper alone.

When you specify the board to the calibration tool, use the **inner corner count**:
- an 8-column × 6-row board has **7×5 inner corners** (`--size 7x5`)
- square size in **metres** (`--square 0.030`)

---

## 3. Capture calibration images

### Option A — Use `ros2 run camera_calibration cameracalibrator` (recommended)

Start the Arena camera node first:

```bash
# Terminal 1
cd ~/Documents/arena_camera_ros2
bash build_and_test.sh --no-test
source ros2_ws/install/setup.bash
ros2 run arena_camera_node start
```

Then run the calibration GUI (note: needs a display):

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 7x5 \
  --square 0.030 \
  --ros-args \
    -r image:=/arena_camera_node/images \
    -r camera:=/arena_camera_node
```

> **Polarized camera note:** calibrate on the **main image topic**
> (`/arena_camera_node/images`), not on the per-angle or DOLP/AoLP topics.
> The main topic publishes the raw polarized sensor image; the calibration
> geometry is the same for all derived topics.
>
> If the main topic publishes the raw Bayer/polarized mosaic instead of a
> debayered image (when `publish_raw: true` and `pixelformat` is set to the
> polarized format), set `publish_compressed: true` and use the `/compressed`
> topic, or temporarily set `pixelformat: "bgr8"` in `camera.yaml` for
> calibration only.

**Movement guidelines during capture:**

- Move the board to cover all parts of the frame (corners, edges, centre).
- Tilt the board ±30° in X and Y.
- Vary the distance (50 cm to 2 m for a typical 5 MP camera).
- Aim for at least **40–60 accepted images** (the GUI shows a progress bar).

The GUI colours the progress bars green when enough data has been collected.
Click **CALIBRATE** — this may take 30–60 seconds.  When done, click **SAVE**
to write a `calibrationdata.tar.gz` archive to `/tmp/`.

### Option B — Save images manually and calibrate offline

If you prefer a headless workflow:

```bash
# Save 50 images (adjust topic and count as needed)
mkdir -p ~/calibration_images
ros2 run image_view extract_images \
  --ros-args \
    -r image:=/arena_camera_node/images \
    -p filename_format:="$HOME/calibration_images/frame%04i.png" \
    -p sec_per_frame:=0.5
```

Then run OpenCV calibration offline using the saved images and the
`opencv_interactive-calibration` tool or a custom Python script.

---

## 4. Extract the calibration file

```bash
cd /tmp
tar xzf calibrationdata.tar.gz
ls ost.yaml   # The calibration file
```

The `ost.yaml` file looks like:

```yaml
image_width: 2448
image_height: 2048
camera_name: arena_camera_node
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx,   0, fy, cy,   0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0,  0, 1, 0,  0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [fx', 0, cx', Tx,  0, fy', cy', Ty,  0, 0, 1, 0]
```

Copy it to a permanent location:

```bash
mkdir -p ~/calibration
cp /tmp/ost.yaml ~/calibration/arena_camera.yaml
```

---

## 5. Load the calibration into the node

Edit `etc/arena_camera/camera.yaml` and set `camera_info_url`:

```yaml
camera_info_url: "file:///home/YOUR_USER/calibration/arena_camera.yaml"
```

Use an **absolute path** prefixed with `file://`.

At startup the node logs:

```
[INFO] Camera calibration loaded from: file:///home/.../arena_camera.yaml
```

If the file is missing or malformed it logs a warning and falls back to the
uncalibrated stub:

```
[WARN] camera_info_url set but calibration failed to load: ... — publishing uncalibrated stub
```

---

## 6. Verify the calibration is active

```bash
source ros2_ws/install/setup.bash
ros2 topic echo /arena_camera_node/camera_info --once
```

A calibrated camera will show non-zero values in `k` (camera matrix):

```
header:
  frame_id: camera_optical_frame
width: 2448
height: 2048
distortion_model: plumb_bob
d: [k1, k2, p1, p2, k3]
k: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
p: [fx', 0.0, cx', 0.0, 0.0, fy', cy', 0.0, 0.0, 0.0, 1.0, 0.0]
```

An uncalibrated stub will have all-zero `k`, `d`, `r`, `p`.

---

## 7. Reprojection error

After calibration the GUI reports the **mean reprojection error** in pixels.
Typical values:

| Error | Quality |
|-------|---------|
| < 0.5 px | Excellent |
| 0.5–1.0 px | Good |
| 1.0–2.0 px | Acceptable |
| > 2.0 px | Poor — retake with better board coverage |

If the error is high, retake images with:
- More board positions near the image edges and corners.
- Less motion blur (use a shorter exposure or brighter lighting).
- A flatter, larger board.

---

## 8. Re-calibrating after optical changes

Re-calibrate whenever:
- The lens focus is changed.
- The camera is disassembled or the lens is removed.
- Large temperature swings cause measurable drift (rare for fixed-focus lenses).

For a polarized camera the calibration applies equally to all derived topics
(DOLP, AoLP, Stokes, per-angle channels) because they all share the same
physical aperture and focal plane.

---

## Quick-reference checklist

```
[ ] ros-humble-camera-calibration installed
[ ] Checkerboard printed flat on rigid board
[ ] Arena node running, images visible on topic
[ ] Calibration tool launched with correct --size and --square
[ ] 40+ images accepted (all progress bars green)
[ ] CALIBRATE clicked, reprojection error < 1 px
[ ] ost.yaml saved to permanent location
[ ] camera_info_url set in camera.yaml
[ ] Node restarted, "calibration loaded" logged
[ ] ros2 topic echo /camera_info shows non-zero K matrix
```
