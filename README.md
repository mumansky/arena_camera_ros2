# arena_camera_ros2
Arena Camera deriver for ROS2 - forked and updated by Mark Umansky

# Requirements

  - OS       : Linux (x64/amd64/arm64) (==22.04) 
  - ROS2     : Humble Hawksbill 
  - ArenaSDK and arena_api : https://thinklucid.com/downloads-hub/

  
# Getting Started
- clone repo or download release
    
    `git clone https://github.com/lucidvisionlabs/arena_camera_ros2.git`

- install ArenaSDK and arena_api
  - https://thinklucid.com/downloads-hub/


- build workspace and its dependencies

    `source /opt/ros/humble/setup.bash` 

    `cd arena_camera_ros2/ros2_ws`

    `colcon build --symlink-install # build workspace for dev`

- install the build

    `. install/setup.bash`

# Explore
- explore nodes
    - arena_camera_node
      - this is the main node. It represent one LUCID Camera.
      - it has two executable `start` and `trigger_image`
      
      ## Configuration
      
      The node automatically loads settings from `etc/arena_camera/camera.yaml` in the project root on startup. This allows you to set default camera parameters without using command-line arguments every time.
      
      **Priority:** Command-line ROS arguments always override settings from the config file.
      
      **Config file location:** `arena_camera_ros2/etc/arena_camera/camera.yaml`
      
      Edit this file to set your preferred defaults for parameters like resolution, exposure, gain, frame rate, etc.
      
      ## ROS Arguments
      
      All parameters can be set via ROS arguments, which will override any values in the config file:
      
        - serial 
          - a string representing the serial of the device to create.
          - if not provided the node, it will represent the first dicovered camera.
        - topic
          - the topic the camera publish images on.
          - default value is /arena_camera_node/images.
          - if passed as a ros argument, it should be preceded with "/"
        - width
          - the width of desired image
          - default value is the one in `default` user profile.
        - height
          - the height of desired image
          - default value is the one in `default` user profile.
        - pixelformat
          - the pixel format of the deisred image
          - supported pixelformats are "rgb8", "rgba8", "rgb16", "rgba16", "bgr8", "bgra8", "bgr16", "bgra16",
                                       "mono8", "mono16", "bayer_rggb8", "bayer_bggr8", "bayer_gbrg8",
                                       "bayer_grbg8", "bayer_rggb16", "bayer_bggr16", "bayer_gbrg16", "bayer_grbg16", 
                                       "yuv422", "polarized_angles_0d_45d_90d_135d_bayer_rg8"
          - the "polarized_angles_0d_45d_90d_135d_bayer_rg8" format is for polarization cameras like PHX050S1-QC
        - gain
          - a double value represents the gain of the image.

        - auto_gain
          - sets camera gain auto mode via GenICam `GainAuto`.
          - common values: "Off", "Once", "Continuous" (device-dependent).
          - when set to "Once" or "Continuous", manual `gain` is ignored.

        - exposure_time
          - the time elapsed before the camera sensor creates the image.
          - units is micro seconds.
          - big values might makes the image take too long before it is view/published.
          - if trigger_mode is passed to node then it is recommended to set exposure_time as well so the
            triggered images do not take longer than expected.

        - auto_exposure
          - sets camera exposure auto mode via GenICam `ExposureAuto`.
          - common values: "Off", "Once", "Continuous" (device-dependent).
          - when set to "Once" or "Continuous", manual `exposure_time` is ignored.

        - acquisition_frame_rate_enable
          - enables manual control of the acquisition frame rate.
          - when false (default), the camera runs at maximum frame rate.
          - when true, allows setting a specific frame rate using acquisition_frame_rate parameter.
          - values are true and false.

        - acquisition_frame_rate
          - target acquisition frame rate in frames per second (FPS).
          - only takes effect when acquisition_frame_rate_enable is true.
          - valid range depends on camera model, resolution, and exposure time.
          - note: frame rate and exposure time are interdependent - lower frame rate allows longer exposure.

        - trigger_mode
          - puts the device in ready state where it will wait for a `trigger_image` client to request an image.
          - default value is false. It means the device will be publishing images to the
            default topic `/arena_camera_node/images`.
          - values are true and false.
          - when `false`, images can be viewed

            `ros2 run arena_camera_node start --ros-args -p qos_reliability:=reliable -p topic:=image`

            `ros2 run image_tools showimage`

          - when `true`, image would not be published unless requested/triggered

            `ros2 run arena_camera_node start --ros-args -p qos_reliability:=reliable -p topic:=image -p exposure_time:=<proper value> -p trigger_mode:=true`

            `ros2 run image_tools showimage # no image will be displayed yet`

            `ros2 run arena_camera_node trigger_image`

        - publish_raw
          - controls whether to publish raw uncompressed images (sensor_msgs/Image).
          - default value is true.
          - values are true and false.
          - raw images published to main topic (e.g., `/arena_camera_node/images`)

        - publish_compressed
          - controls whether to publish JPEG compressed images (sensor_msgs/CompressedImage).
          - default value is false.
          - values are true and false.
          - compressed images published to `<topic>/compressed`
          - both publish_raw and publish_compressed can be enabled simultaneously
       
      - QoS related parameters
        - if using these images with some subscriber make sure: 
          - both `arena_camera_node` and the subscriber on the same topic.
          - both have the same `QoS` settings else the images will be published but the subscriber would not see them because the image mags have a different `QoS` than the subscriber.
          - `QoS` parameter
          - qos_history
            - represents the history value of `QoS` for the image publisher.
            - default value is `keep_last`. 
            - supported values are "system_default","keep_last", "keep_all", "unknown".
            - more about `QoS`: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/
          
          - qos_history_depth
            - represents the depth value of `QoS` for the image publisher.
            - default value is `5`.
            - more about `QoS`: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/
          
          - qos_reliability
            - represents the reliability value of `QoS` for the image publisher.
            - default value is `best_effort`
            - supported values are "system_default", "reliable", "best_effort", "unknown".
            - more about `QoS`: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/

      ## Polarized Camera Support
      
      The node supports polarized cameras (e.g., PHX050S1-QC) with the `polarized_angles_0d_45d_90d_135d_bayer_rg8` pixel format.
      
      When using a polarized camera:
      - The node automatically extracts and publishes all 4 polarization angle channels (0°, 45°, 90°, 135°)
      - The main topic publishes raw 4-channel polarized data: `/arena_camera_node/images`
      - **Note:** The main topic compressed (`/images/compressed`) is not used for polarized cameras (the 4-channel format cannot be compressed to JPEG)
      - Eight polarization channel topics are published (depending on publish_raw/publish_compressed settings):
        - `/arena_camera_node/pol_0deg` - raw BGR8 image of 0° channel (if publish_raw=true)
        - `/arena_camera_node/pol_0deg/compressed` - JPEG compressed 0° (if publish_compressed=true)
        - `/arena_camera_node/pol_45deg` - raw BGR8 image of 45° channel (if publish_raw=true)
        - `/arena_camera_node/pol_45deg/compressed` - JPEG compressed 45° (if publish_compressed=true)
        - `/arena_camera_node/pol_90deg` - raw BGR8 image of 90° channel (if publish_raw=true)
        - `/arena_camera_node/pol_90deg/compressed` - JPEG compressed 90° (if publish_compressed=true)
        - `/arena_camera_node/pol_135deg` - raw BGR8 image of 135° channel (if publish_raw=true)
        - `/arena_camera_node/pol_135deg/compressed` - JPEG compressed 135° (if publish_compressed=true)
      - The BGR8 conversion and compression happens automatically for each channel
      
      ## Example Usage

        # Simple example with config file (settings loaded from etc/arena_camera/camera.yaml)
        `ros2 run arena_camera_node start`

        # Simple example overriding config with arguments

        # Simple example overriding config with arguments
        `ros2 run arena_camera_node start --ros-args -p serial:="904240001" -p topic:=/special_images -p width:=100 -p height:=200 -p pixelformat:=rgb8 -p gain:=10 -p exposure_time:=150 -p trigger_mode:=true` 

        # Example with frame rate control
        `ros2 run arena_camera_node start --ros-args -p acquisition_frame_rate_enable:=true -p acquisition_frame_rate:=30.0 -p exposure_time:=10000.0`

        # Example with polarized camera
        `ros2 run arena_camera_node start --ros-args -p pixelformat:=polarized_angles_0d_45d_90d_135d_bayer_rg8`

        # Example publishing compressed images only (save bandwidth)
        `ros2 run arena_camera_node start --ros-args -p publish_raw:=false -p publish_compressed:=true`

        # Example publishing both raw and compressed
        `ros2 run arena_camera_node start --ros-args -p publish_raw:=true -p publish_compressed:=true`

- explore excutables

  - `ros2 pkg executables | grep arena`
    
  - all excutables can be run by using 

    `ros2 run <pakg name> <executable name>`

- explore actions
  
  - None

- explore services 
  - trigger_image 
    - trigger image form a device that is running in trigger_mode.
    - To run a device in trigger mode
      `ros2 run arena_camera_node start --ros-args -p exposure_time:=<proper value> -p trigger_mode=true`
    - To trigger an image run 
      `ros2 run arena_camera_node trigger_image`

# Logging Verbosity
The node uses standard ROS2 logging levels. By default, per-image publish messages are logged at DEBUG level to reduce console noise.

To enable verbose debug logging, set the log level when starting the node:

    `ros2 run arena_camera_node start --ros-args --log-level debug`

Or for even more detail:

    `ros2 run arena_camera_node start --ros-args --log-level arena_camera_node:=debug`

Available log levels (from most to least verbose): DEBUG, INFO, WARN, ERROR, FATAL

# Architecture

## Node Lifecycle
1. **Config Loading**: `camera.yaml` is read on startup as the single source of truth for all parameters.
2. **System Init**: Arena SDK system is opened and device discovery begins.
3. **Device Connection**: A 1-second timer polls for camera availability.
4. **Streaming**: Once a camera is found, the node starts streaming:
   - **Continuous mode** (default): Uses ArenaSDK's callback-based image acquisition (`RegisterImageCallback`). Each frame is processed in `handle_camera_image_()`.
   - **Trigger mode**: Waits for `trigger_image` service calls. Uses blocking `GetImage()`.
5. **Shutdown**: Destructor deregisters callbacks, stops streaming, destroys device and system in correct order.

## Image Processing Pipeline
- Raw images from the camera are published as `sensor_msgs/Image`
- For non-polarized cameras, images are optionally converted to BGR8 and compressed to JPEG
- For polarized cameras (pixel format `0x8220020F` / `PolarizedAngles_0d_45d_90d_135d_BayerRG8`):
  1. The raw 4-channel image is split into 4 separate channels using `ImageFactory::SplitChannels()`
  2. Channels are ordered: 0°, 45°, 90°, 135° polarization angles
  3. Each channel is converted from BayerRG8 to BGR8 using `ImageFactory::Convert()`
  4. A "max-combined" image is computed using per-pixel maximum across all 4 channels
  5. Each channel and the max-combined image are published as both raw and compressed

## Watchdog
The node includes a watchdog that detects when the camera stops sending frames. If no new frame arrives within `watchdog_timeout_sec` seconds (default: 5.0), the diagnostics report an ERROR status. The watchdog is only active in continuous (non-trigger) mode. Recovery is automatically detected when frames resume.

## QoS Configuration
The node uses `SensorDataQoS` as the base profile, which defaults to:
- History: keep_last (depth 5)
- Reliability: best_effort
- Durability: volatile

These can be overridden in `camera.yaml` with `qos_history`, `qos_history_depth`, and `qos_reliability`. See the config file for trade-off documentation.

## Compression
JPEG compression quality is configurable via `jpeg_quality` (1-100, default 80). All compressed image topics use this setting.

# Running Tests

Unit tests can be run after building the workspace:

    cd ros2_ws
    colcon test --packages-select arena_camera_node
    colcon test-result --verbose

# Road map
- support windows
- add -h flag to nodes
- showimage node to view 2D and 3D images
- camera_info
- access to nodemaps
- support multiple devices simultaneously
- dynamic reconfigure for runtime parameter changes
