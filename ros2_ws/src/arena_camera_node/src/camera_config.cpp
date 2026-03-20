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

void ArenaCameraNode::set_nodes_()
{
  // Load default profile. On reconnect after a crash the camera may still be
  // in a streaming state, which makes UserSet nodes read-only (AccessException).
  // Treat this as non-fatal — remaining set_nodes_* calls will still apply config.
  try {
    set_nodes_load_default_profile_();
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tCould not load default profile (camera may be in a bad state): ") + e.what());
  } catch (std::exception& e) {
    log_warn(std::string("\tCould not load default profile: ") + e.what());
  }

  // Explicitly set AcquisitionMode to "Continuous" regardless of the saved UserSet.
  // SDK examples always set this — if someone previously saved the camera in
  // SingleFrame mode, streaming will silently produce no frames without this.
  try {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "AcquisitionMode", "Continuous");
    log_info("\tAcquisitionMode set to Continuous");
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tCould not set AcquisitionMode: ") + e.what());
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
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tStream configuration warning: ") + e.what());
  } catch (std::exception& e) {
    log_warn(std::string("\tStream configuration warning: ") + e.what());
  }

  // Reserve 10% of link bandwidth for packet resend retransmissions.
  // SDK performance docs recommend this when StreamPacketResendEnable is true.
  try {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<int64_t>(nodemap, "DeviceLinkThroughputReserve", 10);
    log_debug("\tDeviceLinkThroughputReserve set to 10%");
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tDeviceLinkThroughputReserve warning: ") + e.what());
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
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tPacket size configuration warning: ") + e.what());
  } catch (std::exception& e) {
    log_warn(std::string("\tPacket size configuration warning: ") + e.what());
  }
  log_debug("Stream configuration completed");

  //set_nodes_test_pattern_image_();
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

  // Width -------------------------------------------------
  if (is_passed_width) {
    // Align to camera increment (many cameras require multiples of 4, 8, or 16)
    GenApi::CIntegerPtr pWidth = nodemap->GetNode("Width");
    if (pWidth && GenApi::IsReadable(pWidth) && GenApi::IsWritable(pWidth)) {
      int64_t min_val = pWidth->GetMin();
      int64_t inc     = pWidth->GetInc();
      int64_t max_val = pWidth->GetMax();
      int64_t requested = static_cast<int64_t>(width_);
      int64_t aligned = ((requested - min_val) / inc * inc) + min_val;
      aligned = std::clamp(aligned, min_val, max_val);
      if (aligned != requested) {
        log_warn("\tWidth " + std::to_string(requested) + " adjusted to " +
                 std::to_string(aligned) + " (increment=" + std::to_string(inc) + ")");
        width_ = static_cast<size_t>(aligned);
      }
      pWidth->SetValue(aligned);
    } else {
      Arena::SetNodeValue<int64_t>(nodemap, "Width", static_cast<int64_t>(width_));
    }
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    // Align to camera increment
    GenApi::CIntegerPtr pHeight = nodemap->GetNode("Height");
    if (pHeight && GenApi::IsReadable(pHeight) && GenApi::IsWritable(pHeight)) {
      int64_t min_val = pHeight->GetMin();
      int64_t inc     = pHeight->GetInc();
      int64_t max_val = pHeight->GetMax();
      int64_t requested = static_cast<int64_t>(height_);
      int64_t aligned = ((requested - min_val) / inc * inc) + min_val;
      aligned = std::clamp(aligned, min_val, max_val);
      if (aligned != requested) {
        log_warn("\tHeight " + std::to_string(requested) + " adjusted to " +
                 std::to_string(aligned) + " (increment=" + std::to_string(inc) + ")");
        height_ = static_cast<size_t>(aligned);
      }
      pHeight->SetValue(aligned);
    } else {
      Arena::SetNodeValue<int64_t>(nodemap, "Height", static_cast<int64_t>(height_));
    }
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (is_passed_auto_gain_) {
    try {
      // Verify the node is writable and the value is valid
      GenApi::CEnumerationPtr pGainAuto = nodemap->GetNode("GainAuto");
      if (pGainAuto && GenApi::IsWritable(pGainAuto)) {
        // Check if the requested value is available
        if (pGainAuto->GetEntryByName(auto_gain_.c_str())) {
          Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", auto_gain_.c_str());
          log_info(std::string("\tGainAuto set to ") + auto_gain_);
        } else {
          log_warn(std::string("\tGainAuto value '") + auto_gain_ + "' not supported by this camera. Available values:");
          GenApi::StringList_t entries;
          pGainAuto->GetSymbolics(entries);
          for (const auto& entry : entries) {
            log_warn(std::string("\t  - ") + entry.c_str());
          }
          log_warn("\tSkipping GainAuto setting.");
          return;
        }
      } else {
        log_warn("\tGainAuto node not writable");
        return;
      }

      if (auto_gain_ != "Off") {
        return;  // auto gain enabled; skip manual gain
      }
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set GainAuto: ") + e.what());
      return;
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
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
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
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
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
    try {
      GenApi::CEnumerationPtr pExposureAuto = nodemap->GetNode("ExposureAuto");
      if (pExposureAuto && GenApi::IsWritable(pExposureAuto)) {
        if (pExposureAuto->GetEntryByName(auto_exposure_.c_str())) {
          Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", auto_exposure_.c_str());
          log_info(std::string("\tExposureAuto set to ") + auto_exposure_);
          auto_exposure_set = true;
        } else {
          log_warn(std::string("\tExposureAuto value '") + auto_exposure_ + "' not supported by this camera. Available values:");
          GenApi::StringList_t entries;
          pExposureAuto->GetSymbolics(entries);
          for (const auto& entry : entries) {
            log_warn(std::string("\t  - ") + entry.c_str());
          }
          log_warn("\tSkipping ExposureAuto setting. Other exposure parameters will still be applied.");
        }
      } else {
        // Detailed diagnostic: why is the node not writable?
        std::string reason = "ExposureAuto node not writable.";
        try {
          GenICam::gcstring currentVal = pExposureAuto
              ? pExposureAuto->GetCurrentEntry()->GetSymbolic()
              : "null";
          reason += " Current value: " + std::string(currentVal.c_str()) + ".";
        } catch (...) {}
        try {
          GenICam::gcstring mode = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "ExposureMode");
          reason += " ExposureMode: " + std::string(mode.c_str()) + ".";
        } catch (...) {}
        try {
          GenICam::gcstring trig = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode");
          reason += " TriggerMode: " + std::string(trig.c_str()) + ".";
        } catch (...) {}
        log_warn(std::string("\t") + reason + " Other exposure parameters will still be applied.");
      }
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAuto: ") + e.what() + ". Other exposure parameters will still be applied.");
    }
  }

  // --- Step 3: Fine-tune auto exposure parameters (now that ExposureAuto is set) ---
  if (is_passed_exposure_auto_algorithm_) {
    try {
      GenApi::CEnumerationPtr pAlgorithm = nodemap->GetNode("ExposureAutoAlgorithm");
      if (pAlgorithm && GenApi::IsWritable(pAlgorithm)) {
        if (pAlgorithm->GetEntryByName(exposure_auto_algorithm_.c_str())) {
          Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAutoAlgorithm", exposure_auto_algorithm_.c_str());
          log_info(std::string("\tExposureAutoAlgorithm set to ") + exposure_auto_algorithm_);
        } else {
          log_warn(std::string("\tExposureAutoAlgorithm value '") + exposure_auto_algorithm_ + "' not supported. Available values:");
          GenApi::StringList_t entries;
          pAlgorithm->GetSymbolics(entries);
          for (const auto& entry : entries) {
            log_warn(std::string("\t  - ") + entry.c_str());
          }
        }
      } else {
        log_warn("\tExposureAutoAlgorithm node not writable (requires ExposureAuto = Continuous)");
      }
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAutoAlgorithm: ") + e.what());
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
    try {
      GenApi::CEnumerationPtr pLimitAuto = nodemap->GetNode("ExposureAutoLimitAuto");
      if (pLimitAuto && GenApi::IsWritable(pLimitAuto)) {
        if (pLimitAuto->GetEntryByName(exposure_auto_limit_auto_.c_str())) {
          Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAutoLimitAuto", exposure_auto_limit_auto_.c_str());
          log_info(std::string("\tExposureAutoLimitAuto set to ") + exposure_auto_limit_auto_);
        } else {
          log_warn(std::string("\tExposureAutoLimitAuto value '") + exposure_auto_limit_auto_ + "' not supported. Available values:");
          GenApi::StringList_t entries;
          pLimitAuto->GetSymbolics(entries);
          for (const auto& entry : entries) {
            log_warn(std::string("\t  - ") + entry.c_str());
          }
        }
      } else {
        log_warn("\tExposureAutoLimitAuto node not writable");
      }
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAutoLimitAuto: ") + e.what());
    }
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
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ShortExposureEnable: ") + e.what());
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
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tFrame rate configuration warning: ") + e.what());
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
            "\tavoid long waits wating for triggered images by providing proper "
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
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tTrigger mode configuration skipped: ") + e.what());
  } catch (std::exception& e) {
    log_warn(std::string("\tTrigger mode configuration skipped: ") + e.what());
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}
