# Frame Rate Configuration

This document explains how to configure and control the acquisition frame rate for the Arena Camera ROS2 node.

## Overview

The Arena Camera node now supports explicit frame rate control through two parameters:
- `acquisition_frame_rate_enable`: Enable/disable manual frame rate control
- `acquisition_frame_rate`: Set the target frame rate in FPS

## Parameter Details

### `acquisition_frame_rate_enable` (boolean)
- **Default**: `false` (camera runs at maximum frame rate)
- **When `false`**: Camera runs at maximum possible frame rate based on resolution, exposure, and bandwidth
- **When `true`**: Allows setting a specific target frame rate using the `acquisition_frame_rate` parameter

### `acquisition_frame_rate` (double)
- **Unit**: Frames per second (FPS)
- **Only active when**: `acquisition_frame_rate_enable` is `true`
- **Range**: Depends on camera model, resolution, and exposure time
- The driver automatically clamps values to the camera's min/max limits

## Important Relationship: Frame Rate & Exposure Time

Frame rate and exposure time are interdependent:
- **Lower frame rate** → Allows longer maximum exposure time
- **Higher frame rate** → Limits maximum exposure time
- **Rule of thumb**: `max_exposure_time ≈ 1 / frame_rate`

For example, at 30 FPS:
- Maximum exposure time ≈ 1/30 = 0.0333 seconds = 33.3 milliseconds

## Usage Examples

### 1. Using Command Line Arguments

**Maximum Frame Rate (Default)**:
```bash
ros2 run arena_camera_node start
# Frame rate is not limited, camera runs at maximum speed
```

**Set Specific Frame Rate**:
```bash
ros2 run arena_camera_node start --ros-args \
  -p acquisition_frame_rate_enable:=true \
  -p acquisition_frame_rate:=30.0
```

**Low Frame Rate for Long Exposure**:
```bash
ros2 run arena_camera_node start --ros-args \
  -p acquisition_frame_rate_enable:=true \
  -p acquisition_frame_rate:=10.0 \
  -p exposure_time:=90000.0
# 10 FPS allows up to ~100ms exposure, we use 90ms
```

### 2. Using YAML Configuration File

Edit `config/arena_camera.yaml`:

```yaml
/**:
  ros__parameters:
    # For controlled 30 FPS video recording
    acquisition_frame_rate_enable: true
    acquisition_frame_rate: 30.0
    exposure_time: 10000.0  # 10ms exposure
```

Then launch with the config file:
```bash
ros2 run arena_camera_node start --ros-args --params-file config/arena_camera.yaml
```

Or use the launch file:
```bash
ros2 launch arena_camera_node arena_camera.launch.py
```

### 3. Using Launch File

Create a custom launch file or use the provided `arena_camera.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_camera_node',
            parameters=[{
                'acquisition_frame_rate_enable': True,
                'acquisition_frame_rate': 25.0,
                'exposure_time': 15000.0,
                'width': 1920,
                'height': 1200,
            }],
            output='screen',
        ),
    ])
```

## Monitoring Frame Rate

The current frame rate is reported in the diagnostics topic. Monitor it with:

```bash
ros2 topic echo /diagnostics
```

Look for these fields in the diagnostic status:
- `Frame Rate Enabled`: Shows if manual frame rate control is active
- `Frame Rate (FPS)`: Shows the current frame rate setting (or "auto (max)" if disabled)

## Common Use Cases

### High-Speed Acquisition
Maximize frame rate for fast-moving objects:
```yaml
acquisition_frame_rate_enable: false  # Let camera run at max speed
exposure_time: 1000.0  # Short 1ms exposure
```

### Video Recording at Standard Frame Rates
Record smooth video at common frame rates:
```yaml
acquisition_frame_rate_enable: true
acquisition_frame_rate: 30.0  # or 24.0, 25.0, 60.0
exposure_time: 10000.0  # 10ms (well under 33ms limit)
```

### Low-Light / Long Exposure Imaging
Reduce frame rate to allow longer exposures:
```yaml
acquisition_frame_rate_enable: true
acquisition_frame_rate: 5.0  # Low frame rate
exposure_time: 150000.0  # 150ms exposure (under 200ms limit)
```

### Synchronized Multi-Camera
Match frame rates across multiple cameras:
```yaml
acquisition_frame_rate_enable: true
acquisition_frame_rate: 20.0  # Same rate for all cameras
```

## Troubleshooting

### Frame Rate Not Changing
- Ensure `acquisition_frame_rate_enable` is set to `true`
- Check that your requested frame rate is within the camera's supported range
- Verify exposure time is compatible with the frame rate

### Lower Frame Rate Than Expected
- Check if exposure time is too long for the target frame rate
- Reduce `exposure_time` to allow higher frame rates
- Verify network bandwidth is sufficient (for GigE cameras)

### Diagnostics Shows "error reading"
- Camera may not be connected or properly initialized
- Check camera connection and permissions
- Review node startup logs for errors

## See Also
- [config/arena_camera.yaml](../config/arena_camera.yaml) - Full configuration file with examples
- [Arena SDK Documentation](https://support.thinklucid.com/app-note-long-exposure/) - Long exposure application note
