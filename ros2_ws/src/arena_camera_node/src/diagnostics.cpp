/**
 * @file diagnostics.cpp
 * @brief ROS2 diagnostics reporting for ArenaCameraNode
 *
 * Implements produce_diagnostics_() which publishes camera health,
 * performance counters, and rate-limited GenICam register reads
 * to the /diagnostics topic.
 * Split from ArenaCameraNode.cpp for readability.
 */

#include <chrono>
#include <map>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "ArenaCameraNode.h"

void ArenaCameraNode::produce_diagnostics_(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  // Take a snapshot of stats under the mutex to avoid data races with the grab/worker threads.
  double calculated_fps;
  double last_processing_time_ms, max_processing_time_ms, total_processing_time_ms;
  uint64_t processing_time_samples;
  bool watchdog_initialized, camera_frozen, just_froze = false;
  double elapsed_since_last_frame = 0.0;
  {
    std::lock_guard<std::mutex> slock(m_stats_mutex_);
    calculated_fps            = m_calculated_fps_;
    watchdog_initialized      = m_watchdog_initialized_;
    camera_frozen             = m_camera_frozen_;
    last_processing_time_ms   = m_last_processing_time_ms_;
    max_processing_time_ms    = m_max_processing_time_ms_;
    total_processing_time_ms  = m_total_processing_time_ms_;
    processing_time_samples   = m_processing_time_samples_;

    if (watchdog_initialized) {
      auto now = std::chrono::steady_clock::now();
      elapsed_since_last_frame = std::chrono::duration<double>(now - m_last_frame_time_).count();

      // Watchdog: detect frozen camera (no new frames within timeout)
      if (m_device_connected_ && !trigger_mode_activated_ &&
          elapsed_since_last_frame > watchdog_timeout_sec_ && !m_camera_frozen_) {
        m_camera_frozen_ = true;
        camera_frozen    = true;
        just_froze       = true;
      }
    }
  }

  // Snapshot atomic counters (no mutex needed — atomics are safe to read directly)
  uint64_t images_published    = m_images_published_.load();
  uint64_t image_publish_errors = m_image_publish_errors_.load();
  uint64_t backpressure_events = m_backpressure_events_.load();
  uint64_t incomplete_frames   = m_incomplete_frames_.load();

  // Log outside the lock (log calls can be slow)
  if (just_froze) {
    log_err("Watchdog: Camera appears frozen - no new frames for " +
            std::to_string(elapsed_since_last_frame) + "s (timeout: " +
            std::to_string(watchdog_timeout_sec_) + "s). Last frame " +
            std::to_string(images_published) + " published.");
  }

  if (m_device_connected_) {
    if (camera_frozen) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "Camera frozen - no new frames received");
    } else if (image_publish_errors > 0 || backpressure_events > 0 || incomplete_frames > 0) {
      std::string warn_msg = "Camera connected with";
      if (image_publish_errors > 0) {
        warn_msg += " errors";
      }
      if (backpressure_events > 0) {
        if (image_publish_errors > 0) warn_msg += " and";
        warn_msg += " backpressure";
      }
      if (incomplete_frames > 0) {
        if (image_publish_errors > 0 || backpressure_events > 0) warn_msg += " and";
        warn_msg += " incomplete frames";
      }
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   "Camera operating normally");
    }
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Camera not connected");
  }

  stat.add("Device Connected", m_device_connected_ ? "true" : "false");
  stat.add("Images Published", std::to_string(images_published));
  stat.add("Publish Errors", std::to_string(image_publish_errors));
  stat.add("Incomplete Frames", std::to_string(incomplete_frames));
  stat.add("Calculated FPS", std::to_string(calculated_fps));
  stat.add("Trigger Mode", trigger_mode_activated_ ? "enabled" : "disabled");
  stat.add("Topic", topic_);

  // Backpressure metrics
  stat.add("Backpressure Events", std::to_string(backpressure_events));
  stat.add("Last Processing Time (ms)", std::to_string(last_processing_time_ms));
  stat.add("Max Processing Time (ms)", std::to_string(max_processing_time_ms));
  if (processing_time_samples > 0) {
    double avg_processing_time = total_processing_time_ms / processing_time_samples;
    stat.add("Avg Processing Time (ms)", std::to_string(avg_processing_time));
  } else {
    stat.add("Avg Processing Time (ms)", "N/A");
  }
  stat.add("Processing Time Samples", std::to_string(processing_time_samples));

  // Watchdog metrics
  stat.add("Camera Frozen", camera_frozen ? "true" : "false");
  stat.add("Watchdog Timeout (sec)", std::to_string(watchdog_timeout_sec_));
  if (watchdog_initialized) {
    stat.add("Time Since Last Frame (sec)", std::to_string(elapsed_since_last_frame));
  } else {
    stat.add("Time Since Last Frame (sec)", "N/A (no frames yet)");
  }

  if (m_device_connected_) {
    stat.add("Serial", serial_.empty() ? "first discovered" : serial_);
    stat.add("Width", std::to_string(width_));
    stat.add("Height", std::to_string(height_));
    stat.add("Pixel Format", pixelformat_ros_);

    // Rate-limited GenICam register reads — only every DIAG_GENICAM_READ_INTERVAL_SEC
    // to avoid competing with GigE Vision streaming for register access.
    // m_diag_genicam_cache_valid_ and m_last_diag_genicam_read_time_ are only accessed
    // from this (diagnostics timer) thread, so no mutex is needed for them.
    // The map m_diag_genicam_cache_ is shared with the worker thread overlay; we build
    // into a local new_cache then swap under m_diag_cache_mutex_.
    auto now_diag = std::chrono::steady_clock::now();
    double diag_elapsed = m_diag_genicam_cache_valid_
        ? std::chrono::duration<double>(now_diag - m_last_diag_genicam_read_time_).count()
        : DIAG_GENICAM_READ_INTERVAL_SEC + 1.0;  // force first read

    if (diag_elapsed >= DIAG_GENICAM_READ_INTERVAL_SEC) {
      std::map<std::string, std::string> new_cache;
      try {
        auto nodemap = m_pDevice->GetNodeMap();

        // Helper: attempt a read, store on success, silently skip on failure
        auto try_read = [&](const char* diag_key, auto reader) {
          try { new_cache[diag_key] = reader(nodemap); } catch (...) {}
        };

        // Frame rate
        try_read("AcquisitionFrameRateEnable", [](auto nm) {
          return Arena::GetNodeValue<bool>(nm, "AcquisitionFrameRateEnable") ? "true" : "false";
        });
        try_read("AcquisitionFrameRate (FPS)", [](auto nm) {
          bool en = Arena::GetNodeValue<bool>(nm, "AcquisitionFrameRateEnable");
          if (en) return std::to_string(Arena::GetNodeValue<double>(nm, "AcquisitionFrameRate"));
          return std::string("disabled (max)");
        });

        // Gain
        try_read("Gain (dB)", [](auto nm) { return std::to_string(Arena::GetNodeValue<double>(nm, "Gain")); });
        try_read("GainRaw", [](auto nm) { return std::to_string(Arena::GetNodeValue<int64_t>(nm, "GainRaw")); });
        try_read("GainAuto", [](auto nm) {
          GenICam::gcstring v = Arena::GetNodeValue<GenICam::gcstring>(nm, "GainAuto"); return std::string(v.c_str());
        });

        // Exposure
        try_read("ExposureTime (us)", [](auto nm) { return std::to_string(Arena::GetNodeValue<double>(nm, "ExposureTime")); });
        try_read("ExposureTimeRaw", [](auto nm) { return std::to_string(Arena::GetNodeValue<int64_t>(nm, "ExposureTimeRaw")); });
        try_read("ExposureAuto", [](auto nm) {
          GenICam::gcstring v = Arena::GetNodeValue<GenICam::gcstring>(nm, "ExposureAuto"); return std::string(v.c_str());
        });
        // Image statistics (next to exposure for easy comparison)
        try_read("TargetBrightness", [](auto nm) { return std::to_string(Arena::GetNodeValue<int64_t>(nm, "TargetBrightness")); });
        try_read("CalculatedMean", [](auto nm) { return std::to_string(Arena::GetNodeValue<int64_t>(nm, "CalculatedMean")); });
        try_read("CalculatedMedian", [](auto nm) { return std::to_string(Arena::GetNodeValue<int64_t>(nm, "CalculatedMedian")); });

        // Auto exposure details (optional nodes)
        try_read("ExposureAutoUpperLimit (us)", [](auto nm) { return std::to_string(Arena::GetNodeValue<double>(nm, "ExposureAutoUpperLimit")); });
        try_read("ExposureAutoLowerLimit (us)", [](auto nm) { return std::to_string(Arena::GetNodeValue<double>(nm, "ExposureAutoLowerLimit")); });
        try_read("ExposureAutoLimitAuto", [](auto nm) {
          GenICam::gcstring v = Arena::GetNodeValue<GenICam::gcstring>(nm, "ExposureAutoLimitAuto"); return std::string(v.c_str());
        });
        try_read("ExposureAutoAlgorithm", [](auto nm) {
          GenICam::gcstring v = Arena::GetNodeValue<GenICam::gcstring>(nm, "ExposureAutoAlgorithm"); return std::string(v.c_str());
        });
        try_read("ExposureAutoDamping", [](auto nm) { return std::to_string(Arena::GetNodeValue<double>(nm, "ExposureAutoDamping")); });

        // PTP sync status — single combined entry; also updates the atomic used by fill_header_()
        try {
          GenICam::gcstring s = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PtpStatus");
          std::string status(s.c_str());
          bool synced = (status == "Slave");
          m_ptp_synced_.store(synced);
          if (synced) {
            int64_t offset_ns = 0;
            try { offset_ns = Arena::GetNodeValue<int64_t>(nodemap, "PtpOffsetFromMaster"); } catch (...) {}
            new_cache["PTP"] = "Synchronized (offset: " + std::to_string(offset_ns) + " ns)";
          } else {
            new_cache["PTP"] = "NOT synchronized (status: " + status + ")";
          }
        } catch (...) {
          new_cache["PTP"] = "unavailable";
        }

        // Misc
        try_read("ShortExposureEnable", [](auto nm) {
          return Arena::GetNodeValue<bool>(nm, "ShortExposureEnable") ? "true" : "false";
        });
        try_read("DevicePower (W)", [](auto nm) { return std::to_string(Arena::GetNodeValue<double>(nm, "DevicePower")); });
        try_read("DeviceUpTime (ms)", [](auto nm) { return std::to_string(Arena::GetNodeValue<int64_t>(nm, "DeviceUpTime")); });
        try_read("LinkUpTime (ms)", [](auto nm) { return std::to_string(Arena::GetNodeValue<int64_t>(nm, "LinkUpTime")); });
        try_read("DeviceTemperature (C)", [](auto nm) { return std::to_string(Arena::GetNodeValue<double>(nm, "DeviceTemperature")); });

        m_last_diag_genicam_read_time_ = now_diag;
        m_diag_genicam_cache_valid_ = true;

      } catch (GenICam::GenericException& e) {
        new_cache["Camera Parameters"] = std::string("GenICam error: ") + e.what();
        m_diag_genicam_cache_valid_ = true;
        m_last_diag_genicam_read_time_ = now_diag;
      } catch (const std::exception& e) {
        new_cache["Camera Parameters"] = std::string("error: ") + e.what();
        m_diag_genicam_cache_valid_ = true;
        m_last_diag_genicam_read_time_ = now_diag;
      } catch (...) {
        new_cache["Camera Parameters"] = "unknown error reading camera parameters";
        m_diag_genicam_cache_valid_ = true;
        m_last_diag_genicam_read_time_ = now_diag;
      }

      // Atomically swap the completed cache so the overlay thread sees a consistent map
      {
        std::lock_guard<std::mutex> clock(m_diag_cache_mutex_);
        m_diag_genicam_cache_ = std::move(new_cache);
      }
    }

    // Report cached values — snapshot under mutex so we don't race with the overlay reader
    std::map<std::string, std::string> cache_snapshot;
    {
      std::lock_guard<std::mutex> clock(m_diag_cache_mutex_);
      cache_snapshot = m_diag_genicam_cache_;
    }
    for (const auto& kv : cache_snapshot) {
      stat.add(kv.first, kv.second);
    }
    if (m_diag_genicam_cache_valid_) {
      stat.add("Diag GenICam Age (sec)", std::to_string(
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - m_last_diag_genicam_read_time_).count()));
    }
  }
}
