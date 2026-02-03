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

## Notes
- Tasks marked 🔴 should be completed before production release
- Tasks marked 🟡 important for robustness in field conditions
- Tasks marked 🟠 nice-to-have for polish and usability
