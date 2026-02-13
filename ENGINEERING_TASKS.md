# Arena Camera ROS2 Driver - Engineering Tasks

## 🔴 HIGH PRIORITY (Risk/Stability)

### Task 1: Resource Cleanup on Crash/Disconnect
**Priority:** Critical | **Effort:** Medium | **Risk:** High

**Problem:** Arena SDK buffers and device connections may not be released if node crashes or camera disconnects mid-stream. This can leave the camera in a locked state requiring manual power cycle.

**What to Fix:**
- Add proper cleanup in node destructor for Arena SDK resources
- Implement graceful handling of camera disconnects mid-stream
- Add a timeout/watchdog mechanism to detect hung connections
- Ensure all device and buffer objects are properly released

**Success Criteria:**
- Node destructor properly releases Arena::IDevice and Arena::ISystem
- Camera recovers gracefully from network disconnects
- No resource leaks on abnormal exit
- Can restart node without manual camera reset

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.h` (destructor)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (run_() function for connection monitoring)

---

### Task 2: Memory Management in Polarization Processing
**Priority:** Critical | **Effort:** Medium | **Risk:** High

**Problem:** `ImageFactory::SplitChannels()` and `ImageFactory::Convert()` create new image objects in loops. While cleanup exists, there are no RAII wrappers. Any exception during processing could cause buffer leaks.

**What to Fix:**
- Wrap Arena::IImage pointers in RAII smart pointers with custom deleters
- Review all code paths in `publish_one_image_()` to ensure cleanup on exception
- Add exception safety guarantees (strong exception safety where possible)
- Create custom RAII wrapper class for Arena images

**Success Criteria:**
- No memory leaks on exception paths
- All Arena::IImage objects properly destroyed
- RAII wrapper prevents manual cleanup errors
- Passes memory sanitizer on stress test

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 700-850, polarization channel processing)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 450-480, main image publishing)

---

### Task 3: Frame Rate Control Not Fully Implemented
**Priority:** Critical | **Effort:** Small | **Risk:** Medium

**Problem:** Config reads `acquisition_frame_rate_enable_` but it's never applied to the camera. The function `set_nodes_frame_rate_()` exists but is called unconditionally regardless of the enable flag. This causes unexpected frame rate behavior.

**What to Fix:**
- Check `acquisition_frame_rate_enable_` flag before setting frame rate in `set_nodes_frame_rate_()`
- Only apply frame rate if flag is true AND `acquisition_frame_rate_` value is valid
- Add validation that frame rate doesn't conflict with exposure time
- Add logging when frame rate is skipped or applied

**Success Criteria:**
- Frame rate only applied when `acquisition_frame_rate_enable: true`
- Camera respects configured frame rate value
- Conflicts with exposure time are logged/handled
- Default behavior (no frame rate limit) works when flag is false

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 1200-1230, search for `set_nodes_frame_rate_()`)

---

### Task 4: Timer Backpressure Risk
**Priority:** High | **Effort:** Medium | **Risk:** High

**Problem:** 1ms timer interval is very aggressive for image processing. No queue depth limits on publishers. If image processing takes >1ms, callbacks will queue up and cause memory pressure.

**What to Fix:**
- Measure actual image processing time to validate 1ms interval is realistic
- Add publisher queue depth monitoring
- Skip frames if processing falls behind (don't queue indefinitely)
- Add metrics to diagnostics showing backpressure events
- Consider increasing timer interval or adding dynamic scheduling

**Success Criteria:**
- Image processing typically completes in <1ms (99th percentile)
- Publisher queue never exceeds 10 messages
- Backpressure events logged and counted
- No memory growth over 1-hour runtime
- Performance metrics available in diagnostics

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 380-395, timer creation and callback)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 665-730, publish_one_image_() function)

---

## 🟡 MEDIUM PRIORITY (Feature Gaps/Reliability)

### Task 5: Single Device Only - Add Multi-Camera Support
**Priority:** High | **Effort:** High | **Risk:** Medium

**Problem:** No multi-camera support despite SDK supporting it. This is a scalability blocker for future development.

**What to Fix:**
- Add `device_serial` parameter to select which camera to use
- Support discovering and listing all available cameras
- Extend architecture to support multiple simultaneous cameras (future)
- Add camera detection and fallback logic

**Success Criteria:**
- Can specify camera by serial number in config
- If serial not provided, connects to first available
- Lists available cameras on startup
- Can manually select different camera without code changes
- Architecture supports future multi-camera extension

**Relevant Files:**
- `etc/arena_camera/camera.yaml` (add serial parameter)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 1100-1150, device creation)

---

### Task 6: No Live Parameter Adjustment
**Priority:** High | **Effort:** Medium | **Risk:** Low

**Problem:** Config file only - no runtime exposure/brightness tuning. User must restart node to change camera settings. This goes against ROS2 parameter service paradigm and isn't practical for field work.

**What to Fix:**
- Re-implement parameter callback service for `auto_exposure` and `target_brightness`
- Verify parameter changes apply immediately to camera
- Handle parameter validation (reject invalid values)
- Add parameter change logging

**Success Criteria:**
- `ros2 param set /arena_camera_node auto_exposure "Off"` works immediately
- `ros2 param set /arena_camera_node target_brightness 100` applies to camera
- Invalid parameter values are rejected gracefully
- Foxglove Studio can control these parameters
- Parameter changes logged at info level

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.h` (add parameter callback declaration)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (implement on_parameter_change() function)

---

### Task 7: Trigger Mode Untested with Polarized Format
**Priority:** Medium | **Effort:** Medium | **Risk:** Medium

**Problem:** Trigger mode code exists but it's unclear if tested with polarized camera. Could have undiscovered bugs specific to polarized format triggers.

**What to Fix:**
- Test trigger mode with polarized camera extensively
- Verify all 4 polarization channels are captured on trigger
- Test max-combined image generation on trigger
- Document expected behavior and limitations
- Add integration test for trigger mode with polarization

**Success Criteria:**
- Trigger service works reliably with polarized format
- All 4 channels published on each trigger
- Max-combined image generated correctly
- No frame drops or missing data
- Integration test demonstrates full workflow

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 930-1000, trigger service handler)
- `etc/arena_camera/camera.yaml` (trigger_mode parameter)

---

### Task 8: Hardcoded Pixel Format Checks
**Priority:** Medium | **Effort:** Small | **Risk:** Medium

**Problem:** Only checks for `0x8220020F` (polarized). Any other format falls through silently. Brittle design for format detection.

**What to Fix:**
- Create constants for known pixel formats at top of file
- Add explicit handling for non-polarized formats (log warning)
- Implement format detection helper function
- Document all supported formats

**Success Criteria:**
- Named constants for all supported formats
- Clear logging when unsupported format detected
- Helper function `is_polarized_format()` and `is_supported_format()`
- Graceful fallback for unexpected formats
- Code comments explain each format

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (top of file, constants section)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 730-750, pixel format checks)

---

### Task 9: Exception Handling Gaps
**Priority:** Medium | **Effort:** Medium | **Risk:** Medium

**Problem:** Some exceptions logged at warn level but execution continues. Camera disconnect mid-stream may not be caught properly. `m_device_connected_` flag only set at initialization.

**What to Fix:**
- Add try-catch around GetImage() calls in timer callback
- Detect camera disconnects by checking for consecutive timeouts
- Update `m_device_connected_` flag when disconnect detected
- Pause publishing and log errors appropriately
- Implement reconnection logic

**Success Criteria:**
- Camera disconnect detected within 5 seconds
- Publishing pauses gracefully on disconnect
- Error logged at ERROR level (not warn)
- Node doesn't crash on camera disconnect
- Manual reconnection doesn't require node restart
- Consecutive timeout threshold configurable

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 665-750, publish_one_image_())
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 375-395, initialize_())

---

## 🟠 LOW PRIORITY (Polish/Usability)

### Task 10: Hardcoded Compression Settings
**Priority:** Low | **Effort:** Small | **Risk:** Low

**Problem:** JPEG quality always 90, no configuration. No PNG option despite being common in robotics.

**What to Fix:**
- Add `compression_quality` parameter (1-100) to config
- Add `compression_format` parameter (jpeg, png, uncompressed) to config
- Apply compression quality to all JPEG encoding calls
- Document compression format trade-offs

**Success Criteria:**
- Config supports `compression_quality` (default 90)
- Config supports `compression_format` (default jpeg)
- All image compression uses configured values
- PNG option works for lossless compression
- README documents compression options

**Relevant Files:**
- `etc/arena_camera/camera.yaml` (add compression parameters)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 650-690, compression code)

---

### Task 11: No Per-Channel Publishing Control
**Priority:** Low | **Effort:** Medium | **Risk:** Low

**Problem:** Can only enable/disable polarization channels as group. Can't request just 0° and 90° angles if desired.

**What to Fix:**
- Add `publish_pol_0deg`, `publish_pol_45deg`, `publish_pol_90deg`, `publish_pol_135deg` boolean flags to config
- Add `publish_pol_max` boolean flag
- Update publisher creation logic to respect individual flags
- Update publishing logic to skip unpublished channels

**Success Criteria:**
- Each polarization channel can be independently enabled/disabled
- Max-combined can be independently enabled/disabled
- Config clearly documents each channel flag
- Publishers not created for disabled channels (saves memory)
- Bandwidth usage scales with enabled channels

**Relevant Files:**
- `etc/arena_camera/camera.yaml` (add per-channel flags)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 270-330, publisher creation)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 740-800, channel publishing)

---

### Task 12: QoS Configuration Limited
**Priority:** Low | **Effort:** Small | **Risk:** Low

**Problem:** Config reads QoS settings but they're either never used or only partially applied. Fixed to keep_last/depth=5 - no user override.

**What to Fix:**
- Fully implement QoS configuration from config file
- Support `qos_history` (keep_last, keep_all)
- Support `qos_depth` (configurable when keep_last)
- Support `qos_reliability` (best_effort, reliable)
- Document QoS trade-offs in README

**Success Criteria:**
- QoS config values are read and applied
- All publishers use configured QoS
- README explains QoS settings and trade-offs
- Default values reasonable for typical robotics use
- Can switch between best_effort and reliable without code changes

**Relevant Files:**
- `etc/arena_camera/camera.yaml` (ensure QoS parameters documented)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 220-250, QoS setup and publisher creation)

---

### Task 13: Missing Documentation
**Priority:** Low | **Effort:** Small | **Risk:** Low

**Problem:** No comments explaining pixel format codes (0x8220020F, 0x02180015, etc.). Polarization channel ordering not documented. Timeout values unexplained.

**What to Fix:**
- Add file header comment explaining pixel formats
- Document all magic numbers (pixel formats, timeouts, etc.)
- Add comments explaining polarization channel ordering (0°, 45°, 90°, 135°)
- Add inline comments for non-obvious logic
- Create/update README with architecture overview

**Success Criteria:**
- All pixel format codes have named constants and comments
- Polarization channel order explained in code
- All timeout values documented
- README includes architecture section
- Future maintainers can understand code without Arena SDK docs

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (top section, add constants)
- `README.md` (create/update architecture section)

---

### Task 14: No Unit Tests
**Priority:** Low | **Effort:** High | **Risk:** Medium

**Problem:** All validation is manual/integration testing. No regression protection. Refactoring risk is high.

**What to Fix:**
- Create unit tests for image processing logic
- Create integration tests for end-to-end publishing
- Test polarization channel extraction
- Test max-combined image generation
- Test error handling paths

**Success Criteria:**
- Unit test coverage for core logic >80%
- Integration tests verify 4 channels publish correctly
- Tests run in CI/CD pipeline
- README documents running tests
- Tests pass consistently on clean system

**Relevant Files:**
- Create `ros2_ws/src/arena_camera_node/test/` directory
- Add CMake test configuration

---

### Task 15: Type Conversion Errors Not Validated
**Priority:** Low | **Effort:** Small | **Risk:** Small

**Problem:** `param.as_int()`, `as_string()`, `as_bool()` calls could throw exceptions. No type checking before conversion.

**What to Fix:**
- Wrap parameter type conversions in try-catch
- Validate parameter types before conversion
- Log conversion errors clearly
- Use default values on conversion failure

**Success Criteria:**
- Type conversion failures caught and logged
- Node continues running with default values on conversion error
- No unhandled exceptions from parameter conversion
- Error messages help user identify config file issues

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 73-160, parse_parameters_())

---

### Task 16: Diagnostics Update Frequency Fixed
**Priority:** Low | **Effort:** Small | **Risk:** Low

**Problem:** FPS calculated every 1 second only. No way to get higher frequency diagnostics for performance analysis.

**What to Fix:**
- Make diagnostics update frequency configurable
- Add per-frame diagnostics (optional, high-frequency)
- Document diagnostic fields and units
- Consider adding histogram of frame times

**Success Criteria:**
- Config supports `diagnostics_update_hz` (default 1.0)
- Diagnostics update at configured frequency
- README documents all diagnostic fields
- Optional high-frequency diagnostics available

**Relevant Files:**
- `etc/arena_camera/camera.yaml` (add diagnostics_update_hz)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 1440-1490, produce_diagnostics_())

---

### Task 17: Default Values Scattered
**Priority:** Low | **Effort:** Small | **Risk:** Low

**Problem:** Hardcoded in parse_parameters_() instead of config file. Hard to maintain/update defaults.

**What to Fix:**
- Move all hardcoded defaults to camera.yaml
- Use camera.yaml as single source of truth for defaults
- Document each default value and why it was chosen
- Consider creating commented-out examples in config

**Success Criteria:**
- No hardcoded defaults in code
- camera.yaml documents all parameters and defaults
- Easy to update defaults without recompiling
- README references camera.yaml for configuration
- Example configs for common use cases

**Relevant Files:**
- `etc/arena_camera/camera.yaml` (audit for completeness)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 73-160, remove hardcoded defaults)

---

### Task 18: Image Format Assumptions
**Priority:** Low | **Effort:** Small | **Risk:** Small

**Problem:** Assumes BGR8 output from polarization conversion. No validation that conversion actually produced BGR8.

**What to Fix:**
- Validate pixel format after ImageFactory::Convert()
- Add assertions or throw on unexpected format
- Document why BGR8 is expected
- Handle gracefully if conversion produces different format

**Success Criteria:**
- Pixel format validated after conversion
- Clear error if unexpected format detected
- Code comment explains BGR8 requirement
- No silent data corruption if format differs

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 750-770, after Convert() calls)

---

### Task 19: Trigger Service Response Unclear
**Priority:** Low | **Effort:** Small | **Risk:** Small

**Problem:** Success/failure criteria not well documented. What constitutes a successful trigger?

**What to Fix:**
- Document trigger service response logic
- Clarify success criteria (image captured? published?)
- Add comments explaining response codes
- Update README with trigger usage examples

**Success Criteria:**
- Trigger service response documented in code
- README includes trigger usage examples
- Success/failure conditions clearly defined
- Client knows what to expect from response

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 930-1000, trigger_an_image_srv_())
- `README.md` (add trigger usage section)

---

### Task 20: No Heartbeat/Watchdog
**Priority:** Low | **Effort:** Medium | **Risk:** Low

**Problem:** No detection of camera "frozen" state (stopped sending frames). Could publish stale diagnostics indefinitely.

**What to Fix:**
- Add frame timestamp tracking
- Detect if frame timestamp hasn't advanced in N seconds
- Log warning/error on freeze detection
- Pause publishing and alert user
- Implement automatic recovery attempt

**Success Criteria:**
- Frozen camera detected within 5 seconds
- Publishing pauses on freeze
- ERROR logged with last valid timestamp
- Operator alerted via diagnostics
- Automatic recovery attempted
- Configurable freeze timeout threshold

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 665-730, publish_one_image_())
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (lines 1440-1490, diagnostics)

---

## Summary by Effort Level

**Quick Wins (1-2 hours each):**
- Task 3: Frame Rate Control
- Task 8: Hardcoded Pixel Format Checks
- Task 10: Hardcoded Compression Settings
- Task 13: Missing Documentation
- Task 15: Type Conversion Errors
- Task 16: Diagnostics Update Frequency
- Task 17: Default Values Scattered
- Task 18: Image Format Assumptions
- Task 19: Trigger Service Response

**Medium Effort (4-8 hours each):**
- Task 1: Resource Cleanup on Crash
- Task 2: Memory Management in Polarization
- Task 4: Timer Backpressure Risk
- Task 6: Live Parameter Adjustment
- Task 7: Trigger Mode Testing
- Task 9: Exception Handling Gaps
- Task 11: Per-Channel Publishing Control
- Task 12: QoS Configuration
- Task 20: Heartbeat/Watchdog

**High Effort (1-2 days):**
- Task 5: Multi-Camera Support
- Task 14: Unit Tests

---

---

## 🔵 SDK COMPLIANCE (Differences vs ArenaSDK Examples & Documentation)

The following items were identified by comparing the driver implementation against
the official ArenaSDK C++ examples (`Cpp_Acquisition`, `Cpp_Trigger`,
`Cpp_Callback_ImageCallbacks`, `Cpp_Exposure`, `Cpp_Acquisition_RapidAcquisition`,
`Cpp_Callback_OnDeviceDisconnected`, `Cpp_Enumeration`, `Cpp_Acquisition_MultiDevice`)
and the SDK performance documentation.

### Task 21: Missing `AcquisitionMode` Setting
**Priority:** Medium | **Effort:** Low | **Risk:** Medium

**Problem:** Every SDK acquisition example explicitly sets `AcquisitionMode` to `"Continuous"` before `StartStream()`. The driver never sets it — it relies on the UserSetDefault profile, which is not guaranteed to be `"Continuous"` (e.g., if someone previously saved the camera with `SingleFrame` mode).

**What to Fix:**
- Add `Arena::SetNodeValue<GenICam::gcstring>(nodemap, "AcquisitionMode", "Continuous");` in `set_nodes_()` (or a dedicated helper)
- For trigger mode, consider using `"SingleFrame"` instead

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`set_nodes_()`)

---

### Task 22: No Device Disconnect Callback
**Priority:** Medium | **Effort:** Medium | **Risk:** Medium

**Problem:** `Cpp_Callback_OnDeviceDisconnected` shows registering `Arena::IDisconnectCallback` via `pSystem->RegisterDeviceDisconnectCallback()` to detect GigE disconnects at runtime. The driver has no disconnect detection — if the camera is unplugged mid-stream, the node silently stalls until the watchdog flags "frozen."

**What to Fix:**
- Implement `Arena::IDisconnectCallback` subclass
- Register it via `m_pSystem->RegisterDeviceDisconnectCallback(m_pDevice.get(), &cb)`
- In the callback: log disconnect, set `m_device_connected_ = false`, update diagnostics to ERROR
- Deregister in destructor before `DestroyDevice()`
- Optionally trigger reconnection logic

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.h` (new callback class)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`run_()`, destructor)

---

### Task 23: `GetImage` Timeout Too Short in Trigger Mode
**Priority:** Medium | **Effort:** Low | **Risk:** Medium

**Problem:** SDK trigger example uses `GetImage(2000)` (2s). The driver uses `GetImage(1000)` (1s) in `publish_an_image_on_trigger_()`. With long exposure times (e.g., 900ms), this leaves almost no margin for trigger arming + transfer latency, causing intermittent `TimeoutException`.

**What to Fix:**
- Increase to at least `2000`, or compute dynamically: `max(2000, (int)(exposure_time_ / 1000.0) + 2000)`

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`publish_an_image_on_trigger_()`)

---

### Task 24: Unbounded Trigger-Armed Busy-Wait Loop
**Priority:** High | **Effort:** Low | **Risk:** High

**Problem:** The trigger polling loop has a `waitForTriggerCount` variable that is never decremented or checked — it's an infinite busy-wait that pins a CPU core at 100% if the trigger never arms.

```cpp
auto waitForTriggerCount = 10;
do {
    triggerArmed = Arena::GetNodeValue<bool>(..., "TriggerArmed");
    if (triggerArmed == false && (waitForTriggerCount % 10) == 0) { ... }
} while (triggerArmed == false);
```

**What to Fix:**
- Add `std::this_thread::sleep_for(std::chrono::milliseconds(10))` per iteration
- Add maximum retry count or timeout with an exception/error response
- Decrement the counter so the modulo check actually does something

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`publish_an_image_on_trigger_()`)

---

### Task 25: Exception Catch Order Bug — Dead Code
**Priority:** Medium | **Effort:** Low | **Risk:** Medium

**Problem:** In `publish_an_image_on_trigger_()`, `std::exception` is caught **before** `GenICam::GenericException`. Since `GenICam::GenericException` inherits from `std::exception`, the GenICam catch block is unreachable dead code. All SDK examples catch `GenICam::GenericException` **first**.

**What to Fix:**
- Swap the catch order so `GenICam::GenericException&` comes before `std::exception&`

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`publish_an_image_on_trigger_()`)

---

### Task 26: No Configurable Stream Buffer Count
**Priority:** Low | **Effort:** Low | **Risk:** Low

**Problem:** `StartStream()` is called with no argument (defaults to 10 buffers). `Cpp_Acquisition_RapidAcquisition` uses up to 500 buffers for high-throughput scenarios. For callback-based streaming, having enough buffers prevents starvation.

**What to Fix:**
- Add `stream_buffer_count` parameter to `camera.yaml` (default 10)
- Pass it to `m_pDevice->StartStream(stream_buffer_count_)`

**Relevant Files:**
- `etc/arena_camera/camera.yaml`
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`run_()`)

---

### Task 27: No Settings Restoration on Shutdown
**Priority:** Low | **Effort:** Medium | **Risk:** Low

**Problem:** Every SDK example saves initial node values before modification and restores them during cleanup, leaving the camera in its original state. The driver loads UserSetDefault on startup but never restores settings on shutdown. If the node crashes after setting trigger mode to `"On"`, the camera stays in trigger mode.

**What to Fix:**
- Save initial values of critical settings before modification (TriggerMode, ExposureAuto, GainAuto, PixelFormat, AcquisitionMode)
- Restore them in the destructor after `StopStream()`
- Alternatively, re-load UserSetDefault in the destructor

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.h` (store initial values)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (destructor, `set_nodes_*()` functions)

---

### Task 28: No `IsIncomplete()` Check on Received Images
**Priority:** High | **Effort:** Low | **Risk:** High

**Problem:** ArenaSDK images can be incomplete (missing GigE packets). The `Cpp_Acquisition` and other examples check `pImage->IsIncomplete()` and skip/log incomplete frames. The driver never checks — incomplete images with corrupted data are silently published.

**What to Fix:**
- Add `pImage->IsIncomplete()` check in `process_copied_image_()`, `publish_images_()`, and `publish_an_image_on_trigger_()`
- Log a warning and count incomplete frames
- Add incomplete frame count to diagnostics

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`process_copied_image_()`, `publish_images_()`, `publish_an_image_on_trigger_()`)

---

### Task 29: ROI Width/Height Not Clamped or Increment-Aligned
**Priority:** Medium | **Effort:** Low | **Risk:** Medium

**Problem:** `Cpp_Acquisition_RapidAcquisition` carefully enforces integer node increment alignment:
```cpp
value = (((value - pInt->GetMin()) / pInt->GetInc()) * pInt->GetInc()) + pInt->GetMin();
```
The driver sets Width/Height directly without checking min/max/increment. Many cameras require Width/Height to be multiples of 4, 8, or 16 — setting a non-aligned value throws `GenICam::OutOfRangeException`.

**What to Fix:**
- Fetch the `CIntegerPtr` for Width/Height
- Clamp to min/max and align to increment before setting
- Log the adjusted value if it differs from the requested value

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`set_nodes_roi_()`)

---

### Task 30: `frame_id` Is Frame Counter, Not TF Frame Name
**Priority:** Medium | **Effort:** Low | **Risk:** Medium

**Problem:** ROS convention: `sensor_msgs/Image.header.frame_id` should be a TF frame name (e.g., `"camera_optical_frame"`) for correct 3D integration with URDF/TF. The driver sets `frame_id = std::to_string(pImage->GetFrameId())` — the integer frame counter, not a TF frame.

**What to Fix:**
- Add `frame_id` string parameter to `camera.yaml` (default `"arena_camera_frame"`)
- Use it for all published message headers
- Optionally publish the image counter as a separate diagnostic field

**Relevant Files:**
- `etc/arena_camera/camera.yaml`
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`msg_form_image_()`, `process_copied_image_()`)

---

### Task 31: Timestamps Use Camera Clock, Not ROS Clock
**Priority:** High | **Effort:** Low | **Risk:** High

**Problem:** `pImage->GetTimestampNs()` returns the camera's internal PTP/hardware timestamp, which is **not synchronized** with the host system clock (unless PTP is explicitly configured). Timestamps won't align with other ROS nodes' wall-clock times, breaking time-based sensor fusion (LiDAR, IMU, etc.).

**What to Fix:**
- Default to `this->now()` (ROS clock) for message timestamps
- Add `use_camera_timestamp` boolean parameter (default `false`)
- When `true`, use `GetTimestampNs()` (for PTP-synchronized setups)
- Document the trade-off in `camera.yaml`

**Relevant Files:**
- `etc/arena_camera/camera.yaml`
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`msg_form_image_()`, `process_copied_image_()`, all timestamp assignments)

---

### Task 32: Parameters Bypass ROS2 Parameter System
**Priority:** High | **Effort:** High | **Risk:** Medium

**Problem:** All parameters are read via a custom YAML parser (`config_string(m_config_params_, ...)`) instead of ROS2's `declare_parameter()` / `get_parameter()`. Consequences:
- Parameters don't appear in `ros2 param list`
- Launch file overrides (`--ros-args -p key:=value`) don't work
- Dynamic reconfigure is impossible
- Parameter descriptors/types not declared

**What to Fix:**
- Replace custom YAML parsing with `declare_parameter<T>("name", default_value)` / `get_parameter()`
- Pass the config file via `--params-file camera.yaml` in the launch file (ROS2 handles YAML loading)
- Add parameter descriptors for all parameters
- Remove the custom `load_config_file_()` and `config_*()` helpers

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`load_config_file_()`, `parse_parameters_()`)
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.h` (remove `m_config_params_`)

---

### Task 33: Bare `throw;` Outside Catch Block (Undefined Behavior)
**Priority:** Medium | **Effort:** Low | **Risk:** Medium

**Problem:** QoS error handling uses bare `throw;` which is only valid inside a catch block. Called outside a catch block, this is undefined behavior (typically calls `std::terminate()`).

```cpp
log_err(pub_qos_history_ + " is not supported for this node");
throw;  // UB — not inside a catch block
```

**What to Fix:**
- Replace `throw;` with `throw std::invalid_argument("Unsupported QoS history: " + pub_qos_history_);`
- Apply same fix for the QoS reliability `throw;`

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`initialize_()`, QoS setup section)

---

### Task 34: Data Race on FPS/Watchdog Counters
**Priority:** Medium | **Effort:** Low | **Risk:** Medium

**Problem:** `handle_camera_image_()` (SDK grab thread) and `process_copied_image_()` (worker thread) both update `m_fps_frame_count_`, `m_last_frame_time_`, `m_images_published_` concurrently. These are plain `uint64_t` / `chrono::time_point` — not `std::atomic` — so concurrent writes are a data race (undefined behavior per C++ standard).

**What to Fix:**
- Remove duplicate counter updates from `handle_camera_image_()` (the worker thread in `process_copied_image_()` is the natural owner since it does the actual publish)
- Or make the shared counters `std::atomic<uint64_t>` and use `std::atomic` for the time_point

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`handle_camera_image_()`, `process_copied_image_()`)

---

### Task 35: Missing `DeviceLinkThroughputReserve` Configuration
**Priority:** Low | **Effort:** Low | **Risk:** Low

**Problem:** SDK performance docs recommend setting `DeviceLinkThroughputReserve > 0` when `StreamPacketResendEnable` is true, to reserve bandwidth for retransmissions. The driver enables packet resend but never sets throughput reserve.

**What to Fix:**
- Add `Arena::SetNodeValue<double>(nodemap, "DeviceLinkThroughputReserve", 10.0)` alongside packet resend enable
- Make it a configurable parameter (default 10%)

**Relevant Files:**
- `ros2_ws/src/arena_camera_node/src/ArenaCameraNode.cpp` (`set_nodes_()`, stream configuration section)

---

### SDK Compliance Summary

| Task | Issue | Severity | Effort |
|------|-------|----------|--------|
| 21 | Missing `AcquisitionMode` | Medium | Low |
| 22 | No disconnect callback | Medium | Medium |
| 23 | Short `GetImage` timeout in trigger | Medium | Low |
| 24 | Unbounded trigger-armed busy-wait | High | Low |
| 25 | Wrong exception catch order (dead code) | Medium | Low |
| 26 | No configurable stream buffer count | Low | Low |
| 27 | No settings restoration on shutdown | Low | Medium |
| 28 | No `IsIncomplete()` check | High | Low |
| 29 | ROI not increment-aligned | Medium | Low |
| 30 | `frame_id` is counter, not TF frame | Medium | Low |
| 31 | Timestamps from camera clock, not ROS | High | Low |
| 32 | Parameters bypass ROS2 parameter system | High | High |
| 33 | Bare `throw;` (undefined behavior) | Medium | Low |
| 34 | Data race on FPS/watchdog counters | Medium | Low |
| 35 | Missing `DeviceLinkThroughputReserve` | Low | Low |

**Quick wins (highest impact, lowest effort):** Tasks 24, 25, 28, 31, 33, 34

---

## Notes
- Tasks marked 🔴 should be completed before production release
- Tasks marked 🟡 important for robustness in field conditions
- Tasks marked 🟠 nice-to-have for polish and usability
- Tasks marked 🔵 identified by comparing against ArenaSDK official examples and documentation
