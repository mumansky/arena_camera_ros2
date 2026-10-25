/**
 * @file camera_config.cpp
 * @brief Camera GenICam node configuration (set_nodes_* family)
 *
 * Configures the camera via Arena SDK GenICam nodes: ROI, gain, pixel format,
 * exposure, frame rate, trigger mode, and stream transport settings.
 * Split from ArenaCameraNode.cpp for readability.
 */

#include <algorithm>  // std::clamp
#include <stdexcept>  // std::invalid_argument
#include <string>

#include "ArenaCameraNode.h"
#include "rclcpp_adapter/pixelformat_translation.h"

bool ArenaCameraNode::set_enum_node_(GenApi::INodeMap* nodemap, const char* node_name,
                                     const std::string& value)
{
  try {
    GenApi::CEnumerationPtr node = nodemap->GetNode(node_name);
    if (!node || !GenApi::IsWritable(node)) {
      log_warn(std::string("\t") + node_name + " node not writable");
      return false;
    }
    if (!node->GetEntryByName(value.c_str())) {
      log_warn(std::string("\t") + node_name + " value '" + value +
               "' not supported by this camera. Available values:");
      GenApi::StringList_t entries;
      node->GetSymbolics(entries);
      for (const auto& entry : entries) {
        log_warn(std::string("\t  - ") + entry.c_str());
      }
      return false;
    }
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, node_name, value.c_str());
    log_info(std::string("\t") + node_name + " set to " + value);
    return true;
  } catch (std::exception& e) {
    // Covers GenICam::GenericException, which derives from std::exception.
    // Config failures are non-fatal here: warn and let the caller carry on.
    log_warn(std::string("\tFailed to set ") + node_name + ": " + e.what());
    return false;
  }
}

void ArenaCameraNode::set_nodes_()
{
  // Load default profile. On reconnect after a crash the camera may still be
  // in a streaming state, which makes UserSet nodes read-only (AccessException).
  // Treat this as non-fatal — remaining set_nodes_* calls will still apply config.
  try {
    set_nodes_load_default_profile_();
  } catch (std::exception& e) {
    log_warn(std::string("\tCould not load default profile (camera may be in a bad state): ") + e.what());
  }

  // Explicitly set AcquisitionMode to "Continuous" regardless of the saved UserSet.
  // SDK examples always set this — if someone previously saved the camera in
  // SingleFrame mode, streaming will silently produce no frames without this.
  try {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "AcquisitionMode", "Continuous");
    log_info("\tAcquisitionMode set to Continuous");
  } catch (std::exception& e) {
    log_warn(std::string("\tCould not set AcquisitionMode: ") + e.what());
  }

  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  log_debug("set_nodes_pixelformat_() completed");
  set_nodes_frame_rate_();
  log_debug("set_nodes_frame_rate_() completed");
  set_nodes_exposure_();
  log_debug("set_nodes_exposure_() completed");
  set_nodes_trigger_mode_();
  log_debug("set_nodes_trigger_mode_() completed");
  // configure Auto Negotiate Packet Size and Packet Resend
  try {
    auto pTLStreamNodeMap = m_pDevice->GetTLStreamNodeMap();

    // "NewestOnly" drops old frames when the consumer can't keep up, preventing
    // buffer exhaustion that deadlocks the grab thread.  Every Arena SDK example
    // sets this — the default "OldestFirst" will eventually stall the stream.
    Arena::SetNodeValue<GenICam::gcstring>(pTLStreamNodeMap, "StreamBufferHandlingMode", "NewestOnly");
    log_debug("StreamBufferHandlingMode set to NewestOnly");

    Arena::SetNodeValue<bool>(pTLStreamNodeMap, "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(pTLStreamNodeMap, "StreamPacketResendEnable", true);
  } catch (std::exception& e) {
    log_warn(std::string("\tStream configuration warning: ") + e.what());
  }

  // Reserve 10% of link bandwidth for packet resend retransmissions.
  // SDK performance docs recommend this when StreamPacketResendEnable is true.
  try {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<int64_t>(nodemap, "DeviceLinkThroughputReserve", 10);
    log_debug("\tDeviceLinkThroughputReserve set to 10%");
  } catch (std::exception& e) {
    log_warn(std::string("\tDeviceLinkThroughputReserve warning: ") + e.what());
  }

  // Maximize GigE packet size for best throughput (follows RapidAcquisition example)
  try {
    auto nodemap = m_pDevice->GetNodeMap();
    GenApi::CIntegerPtr pPacketSize = nodemap->GetNode("DeviceStreamChannelPacketSize");
    if (pPacketSize && GenApi::IsReadable(pPacketSize) && GenApi::IsWritable(pPacketSize)) {
      int64_t maxPacketSize = pPacketSize->GetMax();
      pPacketSize->SetValue(maxPacketSize);
      log_info(std::string("\tDeviceStreamChannelPacketSize set to ") +
               std::to_string(maxPacketSize) + " bytes");
    }
  } catch (std::exception& e) {
    log_warn(std::string("\tPacket size configuration warning: ") + e.what());
  }
  log_debug("Stream configuration completed");

  // Enable PTP in slave-only mode — camera will sync to an external master
  // but will never self-elect as master via the BMCA algorithm.
  try {
    auto nodemap = m_pDevice->GetNodeMap();

    // Prefer PtpSlaveOnly if available — hard prevents master election.
    // Fall back to setting PtpPriority1=255 (worst possible) which makes
    // master election practically impossible even if SlaveOnly is unsupported.
    bool slave_only_set = false;
    try {
      Arena::SetNodeValue<bool>(nodemap, "PtpSlaveOnly", true);
      slave_only_set = true;
      log_info("\tPTP slave-only mode enabled");
    } catch (...) {
      try {
        Arena::SetNodeValue<int64_t>(nodemap, "PtpPriority1", 255);
        log_info("\tPTP PtpPriority1 set to 255 (slave-only fallback)");
      } catch (...) {}
    }
    (void)slave_only_set;

    Arena::SetNodeValue<bool>(nodemap, "PtpEnable", true);
    log_info("\tPTP enabled — camera will sync to network master, never self-elect");
  } catch (std::exception& e) {
    log_warn(std::string("\tCould not enable PTP (camera may not support it): ") + e.what());
  }
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width and Height take identical treatment; increments are read per-axis.
  auto set_dimension = [&](const char* name, size_t& value, bool is_passed) {
    if (!is_passed) {
      value = static_cast<size_t>(Arena::GetNodeValue<int64_t>(nodemap, name));
      return;
    }
    GenApi::CIntegerPtr node = nodemap->GetNode(name);
    if (!node || !GenApi::IsReadable(node) || !GenApi::IsWritable(node)) {
      Arena::SetNodeValue<int64_t>(nodemap, name, static_cast<int64_t>(value));
      return;
    }
    // Align to camera increment (many cameras require multiples of 4, 8, or 16)
    const int64_t min_val = node->GetMin();
    const int64_t inc = node->GetInc();
    const int64_t requested = static_cast<int64_t>(value);
    const int64_t aligned = std::clamp(((requested - min_val) / inc * inc) + min_val,
                                       min_val, node->GetMax());
    if (aligned != requested) {
      log_warn(std::string("\t") + name + " " + std::to_string(requested) + " adjusted to " +
               std::to_string(aligned) + " (increment=" + std::to_string(inc) + ")");
      value = static_cast<size_t>(aligned);
    }
    node->SetValue(aligned);
  };

  set_dimension("Width", width_, is_passed_width);
  set_dimension("Height", height_, is_passed_height);

  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (is_passed_auto_gain_) {
    if (!set_enum_node_(nodemap, "GainAuto", auto_gain_)) {
      log_warn("\tSkipping GainAuto setting.");
      return;
    }
    if (auto_gain_ != "Off") {
      return;  // auto gain enabled; skip manual gain
    }
  }

  if (is_passed_gain_) {  // manual override
    if (!is_passed_auto_gain_) {
      try {
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Off");
        log_info("\tGainAuto set to Off");
      } catch (GenICam::GenericException& e) {
        log_warn(std::string("\tFailed to set GainAuto to Off: ") + e.what());
      }
    }
    try {
      Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
      log_info(std::string("\tGain set to ") + std::to_string(gain_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set Gain: ") + e.what());
    }
  }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    auto pf_it = K_ROS2_PIXELFORMAT_TO_PFNC.find(pixelformat_ros_);
    pixelformat_pfnc_ = (pf_it != K_ROS2_PIXELFORMAT_TO_PFNC.end()) ? pf_it->second : "";
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    auto ros_it = K_PFNC_TO_ROS2_PIXELFORMAT.find(pixelformat_pfnc_);
    pixelformat_ros_ = (ros_it != K_PFNC_TO_ROS2_PIXELFORMAT.end()) ? ros_it->second : "";

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelformat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // --- Step 1: Ensure ExposureMode is "Timed" ---
  // Other modes (e.g. TriggerWidth) make ExposureAuto read-only because
  // exposure duration is controlled externally by the trigger signal width.
  try {
    GenApi::CEnumerationPtr pExposureMode = nodemap->GetNode("ExposureMode");
    if (pExposureMode && GenApi::IsWritable(pExposureMode)) {
      GenICam::gcstring currentMode = pExposureMode->GetCurrentEntry()->GetSymbolic();
      if (currentMode != "Timed") {
        log_info(std::string("\tExposureMode is '") + currentMode.c_str() +
                 "', switching to 'Timed' to allow auto exposure control");
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureMode", "Timed");
      }
    } else if (pExposureMode) {
      GenICam::gcstring currentMode = pExposureMode->GetCurrentEntry()->GetSymbolic();
      log_debug(std::string("\tExposureMode is '") + currentMode.c_str() + "' (read-only)");
    }
  } catch (GenICam::GenericException& e) {
    log_debug(std::string("\tExposureMode check: ") + e.what());
  }

  // --- Step 2: Enable ExposureAuto BEFORE fine-tuning params ---
  // Many auto-exposure sub-nodes (limits, algorithm, damping) are only
  // writable/meaningful when ExposureAuto is "Continuous".  Setting them
  // while ExposureAuto is "Off" can cause the camera to auto-calculate
  // collapsed limits (e.g. upper == lower == sensor minimum).
  bool auto_exposure_set = false;
  if (is_passed_auto_exposure_) {
    auto_exposure_set = set_enum_node_(nodemap, "ExposureAuto", auto_exposure_);
    if (!auto_exposure_set) {
      // ExposureAuto is most often locked by ExposureMode or TriggerMode —
      // report both so the cause is visible in one log line.
      std::string ctx;
      for (const char* n : {"ExposureMode", "TriggerMode"}) {
        try {
          GenICam::gcstring v = Arena::GetNodeValue<GenICam::gcstring>(nodemap, n);
          ctx += " " + std::string(n) + ": " + v.c_str() + ".";
        } catch (...) {}
      }
      log_warn("\tOther exposure parameters will still be applied." + ctx);
    }
  }

  // --- Step 3: Fine-tune auto exposure parameters (now that ExposureAuto is set) ---
  if (is_passed_exposure_auto_algorithm_) {
    if (!set_enum_node_(nodemap, "ExposureAutoAlgorithm", exposure_auto_algorithm_)) {
      log_warn("\t(ExposureAutoAlgorithm requires ExposureAuto = Continuous)");
    }
  }

  if (is_passed_target_brightness_) {
    try {
      Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness", target_brightness_);
      log_info(std::string("\tTargetBrightness set to ") + std::to_string(target_brightness_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set TargetBrightness: ") + e.what());
    }
  }

  if (is_passed_exposure_auto_damping_) {
    try {
      Arena::SetNodeValue<double>(nodemap, "ExposureAutoDamping", exposure_auto_damping_);
      log_info(std::string("\tExposureAutoDamping set to ") + std::to_string(exposure_auto_damping_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAutoDamping: ") + e.what());
    }
  }

  // Set exposure limit control (set mode before explicit limits)
  if (is_passed_exposure_auto_limit_auto_) {
    set_enum_node_(nodemap, "ExposureAutoLimitAuto", exposure_auto_limit_auto_);
  }

  // Set manual exposure limits (only effective when ExposureAutoLimitAuto is "Off")
  if (is_passed_exposure_auto_upper_limit_) {
    try {
      Arena::SetNodeValue<double>(nodemap, "ExposureAutoUpperLimit", exposure_auto_upper_limit_);
      log_info(std::string("\tExposureAutoUpperLimit set to ") + std::to_string(exposure_auto_upper_limit_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAutoUpperLimit: ") + e.what());
    }
  }

  if (is_passed_exposure_auto_lower_limit_) {
    try {
      Arena::SetNodeValue<double>(nodemap, "ExposureAutoLowerLimit", exposure_auto_lower_limit_);
      log_info(std::string("\tExposureAutoLowerLimit set to ") + std::to_string(exposure_auto_lower_limit_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAutoLowerLimit: ") + e.what());
    }
  }

  // --- Step 4: If auto exposure is on, we're done (skip manual exposure) ---
  if (auto_exposure_set && auto_exposure_ != "Off") {
    return;
  }

  // --- Step 5: Manual exposure time ---
  if (is_passed_exposure_time_) {
    if (!is_passed_auto_exposure_) {
      try {
        // Ensure ExposureMode is Timed before disabling auto
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
        log_info("\tExposureAuto set to Off");
      } catch (GenICam::GenericException& e) {
        log_warn(std::string("\tFailed to set ExposureAuto to Off: ") + e.what());
      }
    }
    try {
      Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
      log_info(std::string("\tExposureTime set to ") + std::to_string(exposure_time_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureTime: ") + e.what());
    }
  }
}

void ArenaCameraNode::set_nodes_frame_rate_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Set short exposure enable if specified
  if (is_passed_short_exposure_enable_) {
    try {
      Arena::SetNodeValue<bool>(nodemap, "ShortExposureEnable", short_exposure_enable_);
      log_info(std::string("\tShortExposureEnable set to: ") +
               (short_exposure_enable_ ? "true" : "false"));
    } catch (std::exception& e) {
      log_warn(std::string("\tFailed to set ShortExposureEnable: ") + e.what());
    }
  }

  try {
    // Set frame rate enable if specified
    if (is_passed_acquisition_frame_rate_enable_) {
      Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", acquisition_frame_rate_enable_);
      log_info(std::string("\tAcquisitionFrameRateEnable set to ") +
               (acquisition_frame_rate_enable_ ? "true" : "false"));
    }

    // Determine if frame rate control is enabled (either explicitly set or from camera)
    bool frame_rate_enabled = is_passed_acquisition_frame_rate_enable_
        ? acquisition_frame_rate_enable_
        : Arena::GetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable");

    // If frame rate control is disabled, log and skip setting the frame rate value
    if (!frame_rate_enabled) {
      if (is_passed_acquisition_frame_rate_) {
        log_info("\tFrame rate value specified but acquisition_frame_rate_enable is false. "
                 "Frame rate control disabled, camera will run at maximum rate.");
      } else {
        log_debug("\tFrame rate control disabled, camera will run at maximum rate.");
      }
      return;
    }

    // Set frame rate value if specified and enabled
    if (is_passed_acquisition_frame_rate_) {
      // Get the node to validate min/max
      GenApi::CFloatPtr pAcquisitionFrameRate = nodemap->GetNode("AcquisitionFrameRate");

      if (pAcquisitionFrameRate && GenApi::IsWritable(pAcquisitionFrameRate)) {
        double frame_rate = acquisition_frame_rate_;

        // Ensure frame rate is within valid range
        if (frame_rate < pAcquisitionFrameRate->GetMin()) {
          log_warn(std::string("\tRequested frame rate ") + std::to_string(frame_rate) +
                   " is below minimum. Setting to " + std::to_string(pAcquisitionFrameRate->GetMin()));
          frame_rate = pAcquisitionFrameRate->GetMin();
        }
        if (frame_rate > pAcquisitionFrameRate->GetMax()) {
          log_warn(std::string("\tRequested frame rate ") + std::to_string(frame_rate) +
                   " is above maximum. Setting to " + std::to_string(pAcquisitionFrameRate->GetMax()));
          frame_rate = pAcquisitionFrameRate->GetMax();
        }

        // Validate frame rate vs exposure time conflict
        // Frame rate and exposure time are interdependent:
        //   - max_exposure_time (in microseconds) ≈ 1,000,000 / frame_rate
        // If the configured exposure time exceeds this limit, warn the user
        if (is_passed_exposure_time_ && frame_rate > 0) {
          // Calculate max exposure time in microseconds for given frame rate
          double max_exposure_for_frame_rate = 1000000.0 / frame_rate;
          if (exposure_time_ > max_exposure_for_frame_rate) {
            log_warn(std::string("\tPotential conflict: Configured exposure time (") +
                     std::to_string(exposure_time_) + " us) exceeds maximum allowed by frame rate (" +
                     std::to_string(max_exposure_for_frame_rate) + " us at " +
                     std::to_string(frame_rate) + " FPS). The camera may limit actual exposure time.");
          }
        }

        Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", frame_rate);
        log_info(std::string("\tAcquisitionFrameRate set to ") + std::to_string(frame_rate) + " FPS");
      } else {
        log_warn("\tAcquisitionFrameRate node not writable");
      }
    } else {
      log_info("\tFrame rate control enabled but no frame rate value specified. Using camera default.");
    }
  } catch (std::exception& e) {
    log_warn(std::string("\tFrame rate configuration warning: ") + e.what());
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  try {
    auto nodemap = m_pDevice->GetNodeMap();
    if (trigger_mode_activated_) {
      if (exposure_time_ < 0) {
        log_warn(
            "\tavoid long waits waiting for triggered images by providing proper "
            "exposure_time.");
      }
      // Enable trigger mode before setting the source and selector
      // and before starting the stream. Trigger mode cannot be turned
      // on and off while the device is streaming.

      // Make sure Trigger Mode set to 'Off' after finishing this example
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

      // Set the trigger source to software in order to trigger buffers
      // without the use of any additional hardware.
      // Lines of the GPIO can also be used to trigger.
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                             "Software");
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                             "FrameStart");
      auto msg =
          std::string(
              "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
          this->get_name() + " trigger_image`";
      log_warn(msg);
    }
    // unset device from being in trigger mode if user did not pass trigger
    // mode parameter because the trigger nodes are not rest when loading
    // the user default profile
    else {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
    }
  } catch (std::exception& e) {
    // GenICam::GenericException derives from std::exception — one handler covers both.
    log_warn(std::string("\tTrigger mode configuration skipped: ") + e.what());
  }
}
