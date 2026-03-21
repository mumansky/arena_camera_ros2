/**
 * @file ArenaCameraNode.cpp
 * @brief ROS2 node for LUCID (Arena) cameras
 *
 * This file implements the ArenaCameraNode class which uses the ArenaSDK C++ API
 * to discover and stream images from LUCID cameras (e.g., PHX050S1-QC polarized cameras).
 *
 * == Pixel Format Reference ==
 * The node works with PFNC (PixelFormat Naming Convention) values from Arena SDK:
 *   - PFNC_BGR8 (0x02180015): 3-channel Blue-Green-Red, 8 bits per channel.
 *     This is the standard output format for OpenCV and compressed image publishing.
 *   - PFNC_POLARIZED_BAYER_RG8 (0x8220020F): Polarized 4-angle Bayer pattern.
 *     Used by PHX050S1-QC cameras. Each 2x2 superpixel contains one pixel for each
 *     of 4 polarization angles (0°, 45°, 90°, 135°) in a BayerRG8 pattern.
 *
 * == Polarization Channel Ordering ==
 * When the polarized format is detected, ImageFactory::SplitChannels() returns
 * a vector of 4 images in this order:
 *   channels[0] = 0° polarization angle
 *   channels[1] = 45° polarization angle
 *   channels[2] = 90° polarization angle
 *   channels[3] = 135° polarization angle
 * Each channel is in BayerRG8 format and must be converted to BGR8 before publishing.
 *
 * == Timeout Values ==
 *   - Device discovery timeout: 100ms (UpdateDevices call)
 *   - GetImage timeout: 1000ms (1 second) for blocking image retrieval
 *   - Device connection timer: 1s interval for polling camera availability
 *   - FPS calculation window: 1000ms (1 second rolling window)
 *   - Watchdog timeout: configurable via watchdog_timeout_sec (default 5.0s)
 *
 * == Architecture ==
 * The node operates in two modes:
 *   1. Continuous mode (default): Uses ArenaSDK callbacks (RegisterImageCallback)
 *      for event-driven image acquisition. The handle_camera_image_() method
 *      processes each frame as it arrives.
 *   2. Trigger mode: Waits for service calls via the trigger_image service.
 *      Uses blocking GetImage() in publish_an_image_on_trigger_().
 *
 * Trigger mode uses blocking GetImage() in publish_an_image_on_trigger_().
 */

#include <cinttypes>  // PRId64, PRIu64
#include <cstring>    // memcpy
#include <cmath>      // std::atan2, std::acos, std::clamp
#include <algorithm>  // std::clamp
#include <stdexcept>  // std::runtime_error
#include <string>
#include <fstream>    // file I/O
#include <vector>     // vector
#include <cstdlib>    // getenv
#include <filesystem> // directory creation for image saving
#include <iomanip>    // put_time for session directory naming
#include <sstream>    // ostringstream for diagnostics overlay
#include <future>     // std::async for parallel channel debayer
#include <yaml-cpp/yaml.h>  // YAML parsing

// OpenCV
#include <opencv2/opencv.hpp>

// Optional CUDA runtime (for cudaGetDeviceCount probe; no-op if HAS_CUDA not defined)
#ifdef HAS_CUDA
#include <cuda_runtime.h>
#endif

// ROS
#include "rmw/types.h"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// ArenaSDK
#include "ArenaCameraNode.h"
#include "arena_image_raii.h"
#include "config_helpers.h"
#include "pixel_format_helpers.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quality_of_service_translation.h"

/**
 * @brief Validate that an image has the expected BGR8 pixel format after conversion.
 *
 * Arena SDK's ImageFactory::Convert() is expected to produce BGR8 output when
 * requested. BGR8 (PFNC value 0x02180015) uses 3 bytes per pixel in Blue-Green-Red
 * order and is the format assumed by downstream OpenCV operations (cv::Mat CV_8UC3)
 * and ROS message encoding ("bgr8"). If conversion produces a different format,
 * image data would be silently misinterpreted.
 *
 * @param image Pointer to the converted Arena::IImage
 * @param context Description of where the conversion happened (for log messages)
 * @return true if the format is BGR8, false otherwise
 */
inline bool validate_bgr8_format(Arena::IImage* image, const std::string& context) {
  if (!image) return false;
  uint64_t actual = image->GetPixelFormat();
  if (actual != PixelFormat::BGR8) {
    // Log as error since downstream code assumes BGR8 layout
    RCLCPP_ERROR(rclcpp::get_logger("arena_camera_node"),
        "Unexpected pixel format after conversion in %s: expected BGR8 (0x%lx), got 0x%lx. "
        "Image data may be corrupted.",
        context.c_str(), static_cast<unsigned long>(PixelFormat::BGR8), static_cast<unsigned long>(actual));
    return false;
  }
  return true;
}

void ArenaCameraNode::load_config_file_()
{
  // Try multiple possible locations for the config file
  std::vector<std::string> possible_paths = {
    "../../../etc/arena_camera/camera.yaml",  // From install/lib/arena_camera_node/
    "../../../../etc/arena_camera/camera.yaml", // Alternative depth
    "../etc/arena_camera/camera.yaml",        // From workspace root
    "etc/arena_camera/camera.yaml",           // Direct relative
    std::string(getenv("HOME") ? getenv("HOME") : "") + "/Documents/git/arena_camera_ros2/etc/arena_camera/camera.yaml" // Absolute fallback
  };
  
  std::string config_path;
  bool found = false;
  
  for (const auto& path : possible_paths) {
    if (path.empty()) continue;
    std::ifstream test_file(path);
    if (test_file.good()) {
      config_path = path;
      found = true;
      break;
    }
  }
  
  if (!found) {
    log_warn("Config file not found in any expected location, using default parameters or command line args");
    return;
  }
  
  try {
    YAML::Node config = YAML::LoadFile(config_path);
    
    // Navigate to the parameters section
    if (!config["/**"] || !config["/**"]["ros__parameters"]) {
      log_warn("Config file format incorrect, expected /**/ros__parameters structure");
      return;
    }
    
    m_config_params_ = config["/**"]["ros__parameters"];
    
    log_info("Loading configuration from " + config_path);
    log_info("Configuration loaded successfully");
    
  } catch (const YAML::Exception& e) {
    log_err(std::string("Error parsing config file: ") + e.what());
  } catch (const std::exception& e) {
    log_err(std::string("Error loading config file: ") + e.what());
  }
}

void ArenaCameraNode::parse_parameters_()
{
  std::string currentParam = "";
  try {
    // All parameters are read from camera.yaml (the single source of truth).
    // Type conversion errors are caught and logged; defaults are used on failure.

    currentParam = "topic";
    topic_ = config_string(m_config_params_, "topic", "/arena_camera_node/images");
    is_passed_topic_ = topic_ != "/arena_camera_node/images";

    currentParam = "frame_id";
    frame_id_ = config_string(m_config_params_, "frame_id", "camera_optical_frame");

    // use_camera_timestamp_ removed — timestamping is automatic:
    // PTP synced (PtpStatus=Slave) → camera hardware clock; otherwise → ROS clock.

    currentParam = "camera_info_url";
    camera_info_url_ = config_string(m_config_params_, "camera_info_url", "");

    currentParam = "serial";
    serial_ = config_string(m_config_params_, "serial", "");
    is_passed_serial_ = !serial_.empty();

    currentParam = "pixelformat";
    pixelformat_ros_ = config_string(m_config_params_, "pixelformat", "");
    is_passed_pixelformat_ros_ = !pixelformat_ros_.empty();

    currentParam = "width";
    width_ = static_cast<size_t>(config_int64(m_config_params_, "width", 0));
    is_passed_width = width_ > 0;

    currentParam = "height";
    height_ = static_cast<size_t>(config_int64(m_config_params_, "height", 0));
    is_passed_height = height_ > 0;

    currentParam = "gain";
    gain_ = config_double(m_config_params_, "gain", -1.0);
    is_passed_gain_ = gain_ >= 0.0;

    currentParam = "auto_gain";
    auto_gain_ = config_string(m_config_params_, "auto_gain", "");
    is_passed_auto_gain_ = !auto_gain_.empty();

    currentParam = "exposure_time";
    exposure_time_ = config_double(m_config_params_, "exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0.0;

    currentParam = "auto_exposure";
    auto_exposure_ = config_string(m_config_params_, "auto_exposure", "");
    is_passed_auto_exposure_ = !auto_exposure_.empty();

    currentParam = "short_exposure_enable";
    short_exposure_enable_ = config_bool(m_config_params_, "short_exposure_enable", false);
    is_passed_short_exposure_enable_ = config_has_value(m_config_params_, "short_exposure_enable");

    currentParam = "exposure_auto_algorithm";
    exposure_auto_algorithm_ = config_string(m_config_params_, "exposure_auto_algorithm", "");
    is_passed_exposure_auto_algorithm_ = !exposure_auto_algorithm_.empty();

    currentParam = "target_brightness";
    target_brightness_ = config_int64(m_config_params_, "target_brightness", -1);
    is_passed_target_brightness_ = target_brightness_ >= 0;

    currentParam = "exposure_auto_damping";
    exposure_auto_damping_ = config_double(m_config_params_, "exposure_auto_damping", -1.0);
    is_passed_exposure_auto_damping_ = exposure_auto_damping_ >= 0.0;

    currentParam = "exposure_auto_limit_auto";
    exposure_auto_limit_auto_ = config_string(m_config_params_, "exposure_auto_limit_auto", "");
    is_passed_exposure_auto_limit_auto_ = !exposure_auto_limit_auto_.empty();

    currentParam = "exposure_auto_upper_limit";
    exposure_auto_upper_limit_ = config_double(m_config_params_, "exposure_auto_upper_limit", -1.0);
    is_passed_exposure_auto_upper_limit_ = exposure_auto_upper_limit_ >= 0.0;

    currentParam = "exposure_auto_lower_limit";
    exposure_auto_lower_limit_ = config_double(m_config_params_, "exposure_auto_lower_limit", -1.0);
    is_passed_exposure_auto_lower_limit_ = exposure_auto_lower_limit_ >= 0.0;

    currentParam = "acquisition_frame_rate_enable";
    acquisition_frame_rate_enable_ = config_bool(m_config_params_, "acquisition_frame_rate_enable", false);
    is_passed_acquisition_frame_rate_enable_ = config_has_value(m_config_params_, "acquisition_frame_rate_enable");

    currentParam = "acquisition_frame_rate";
    acquisition_frame_rate_ = config_double(m_config_params_, "acquisition_frame_rate", -1.0);
    is_passed_acquisition_frame_rate_ = acquisition_frame_rate_ >= 0.0;

    currentParam = "trigger_mode";
    trigger_mode_activated_ = config_bool(m_config_params_, "trigger_mode", false);

    currentParam = "stream_buffer_count";
    stream_buffer_count_ = config_int(m_config_params_, "stream_buffer_count", 10);

    // QoS settings
    currentParam = "qos_history";
    pub_qos_history_ = config_string(m_config_params_, "qos_history", "");
    is_passed_pub_qos_history_ = !pub_qos_history_.empty();

    currentParam = "qos_history_depth";
    pub_qos_history_depth_ = static_cast<size_t>(config_int64(m_config_params_, "qos_history_depth", 0));
    is_passed_pub_qos_history_depth_ = config_has_value(m_config_params_, "qos_history_depth") && pub_qos_history_depth_ > 0;

    currentParam = "qos_reliability";
    pub_qos_reliability_ = config_string(m_config_params_, "qos_reliability", "");
    is_passed_pub_qos_reliability_ = !pub_qos_reliability_.empty();

    // Image publishing options
    currentParam = "publish_raw";
    publish_raw_ = config_bool(m_config_params_, "publish_raw", true);

    currentParam = "publish_compressed";
    publish_compressed_ = config_bool(m_config_params_, "publish_compressed", false);

    currentParam = "publish_pol_channels";
    publish_pol_channels_ = config_bool(m_config_params_, "publish_pol_channels", true);

    currentParam = "publish_pol_max";
    publish_pol_max_ = config_bool(m_config_params_, "publish_pol_max", true);

    currentParam = "publish_dolp";
    publish_dolp_ = config_bool(m_config_params_, "publish_dolp", false);

    currentParam = "publish_aolp";
    publish_aolp_ = config_bool(m_config_params_, "publish_aolp", false);

    currentParam = "publish_stokes";
    publish_stokes_ = config_bool(m_config_params_, "publish_stokes", false);

    currentParam = "publish_aolp_color";
    publish_aolp_color_ = config_bool(m_config_params_, "publish_aolp_color", false);
    if (publish_aolp_color_ && (!publish_dolp_ || !publish_aolp_)) {
      log_warn("publish_aolp_color requires publish_dolp: true and publish_aolp: true — disabling");
      publish_aolp_color_ = false;
    }

    currentParam = "profile_processing";
    profile_processing_ = config_bool(m_config_params_, "profile_processing", false);
    if (profile_processing_) {
      log_info("Processing profiler enabled — per-frame timing will be logged at DEBUG level");
    }

    currentParam = "display_images";
    display_images_ = config_bool(m_config_params_, "display_images", false);
    display_images_active_ = display_images_;
    // Disable display in headless environments (no X display available)
    if (display_images_active_) {
      const char* disp = getenv("DISPLAY");
      if (!disp || disp[0] == '\0') {
        log_warn("display_images is enabled but no DISPLAY environment variable is set. "
                 "Disabling debug display (headless environment).");
        display_images_active_ = false;
      }
    }

    currentParam = "jpeg_quality";
    jpeg_quality_ = config_int(m_config_params_, "jpeg_quality", 80);

    currentParam = "gpu_acceleration";
    gpu_acceleration_ = config_string(m_config_params_, "gpu_acceleration", "auto");

    // Watchdog settings (Task 20)
    currentParam = "watchdog_timeout_sec";
    watchdog_timeout_sec_ = config_double(m_config_params_, "watchdog_timeout_sec", 5.0);

    // Camera info / calibration
    currentParam = "camera_info_url";
    camera_info_url_ = config_string(m_config_params_, "camera_info_url", "");

    // Latency profiling
    currentParam = "latency_mode";
    latency_mode_ = config_string(m_config_params_, "latency_mode", "off");
    profile_latency_ = (latency_mode_ != "off");
    use_hw_timestamp_for_latency_ = (latency_mode_ == "hardware" || latency_mode_ == "both");
    if (profile_latency_) {
      log_info("Latency profiling enabled: mode=" + latency_mode_);
    }

    // Camera reconnection
    currentParam = "reconnect_max_attempts";
    reconnect_max_attempts_ = config_int(m_config_params_, "reconnect_max_attempts", 0);

  } catch (std::exception& e) {
    log_err("Error parsing parameter '" + currentParam + "': " + std::string(e.what()) +
            ". Check that the value type matches the expected type in camera.yaml.");
    throw;
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Use no-op deleters since cleanup is handled explicitly in the destructor
  // to avoid accessing 'this' after partial destruction
  // shared_ptr with no-op deleter: CloseSystem() is called explicitly in the
  // destructor, so the shared_ptr must never call delete on the raw pointer.
  // Using shared_ptr(ptr, deleter) preserves the no-op through the lifetime;
  // reset(ptr) would silently replace it with the default (delete) deleter.
  m_pSystem = std::shared_ptr<Arena::ISystem>(
      Arena::OpenSystem(), [](Arena::ISystem*) { /* no-op */ });

  // m_pDevice is default-initialized (null). The no-op deleter is applied in
  // run_() when the device is created, so DestroyDevice() handles cleanup.

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  this->declare_parameter("target_brightness", target_brightness_);
  m_param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArenaCameraNode::on_set_parameters_, this, _1));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_history_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY.at(pub_qos_history_));
    } else {
      throw std::invalid_argument("Unsupported QoS history policy: " + pub_qos_history_);
    }
  }
  // QoS depth — only apply if history policy is KEEP_LAST; use find() to avoid
  // inserting into the map when pub_qos_history_ is unset (empty string).
  if (is_passed_pub_qos_history_depth_) {
    auto hist_it = K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY.find(pub_qos_history_);
    if (hist_it != K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY.end() &&
        hist_it->second == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
      pub_qos_.keep_last(pub_qos_history_depth_);
    }
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY.at(pub_qos_reliability_));
    } else {
      throw std::invalid_argument("Unsupported QoS reliability policy: " + pub_qos_reliability_);
    }
  }

  // rmw_qos_history_policy_t history_policy_ = RMW_QOS_
  // rmw_qos_history_policy_t;
  // auto pub_qos_init = rclcpp::QoSInitialization(history_policy_, );

  // Create publishers based on configuration
  if (publish_raw_) {
    m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_, pub_qos_);
  }
  
  // Only create main compressed publisher for non-polarized formats
  // Polarized format: "polarized_angles_0d_45d_90d_135d_bayer_rg8"
  bool is_polarized = (pixelformat_ros_ == "polarized_angles_0d_45d_90d_135d_bayer_rg8");
  
  if (publish_compressed_ && !is_polarized) {
    m_pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/compressed", pub_qos_);
  }
  
  // Create publishers for all polarization channels
  if (publish_pol_channels_ && (publish_raw_ || publish_compressed_)) {
    m_pub_pol_0deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_0deg", pub_qos_);
    m_pub_pol_45deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_45deg", pub_qos_);
    m_pub_pol_90deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_90deg", pub_qos_);
    m_pub_pol_135deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_135deg", pub_qos_);
  }
  if (publish_pol_channels_ && publish_compressed_) {
    m_pub_pol_0deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/pol_0deg/compressed", pub_qos_);
    m_pub_pol_45deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/pol_45deg/compressed", pub_qos_);
    m_pub_pol_90deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/pol_90deg/compressed", pub_qos_);
    m_pub_pol_135deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/pol_135deg/compressed", pub_qos_);
  }

  // Create publishers for max-combined polarization image
  if (publish_pol_max_ && (publish_raw_ || publish_compressed_)) {
    m_pub_pol_max_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_max", pub_qos_);
  }
  if (publish_pol_max_ && publish_compressed_) {
    m_pub_pol_max_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/pol_max/compressed", pub_qos_);
  }
  
  // Create publishers for DOLP (Degree of Linear Polarization) — mono8
  if (is_polarized && publish_dolp_) {
    m_pub_dolp_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/dolp", pub_qos_);
    if (publish_compressed_) {
      m_pub_dolp_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          topic_ + "/dolp/compressed", pub_qos_);
    }
  }
  
  // Create publishers for AoLP (Angle of Linear Polarization) — mono8
  if (is_polarized && publish_aolp_) {
    m_pub_aolp_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/aolp", pub_qos_);
    if (publish_compressed_) {
      m_pub_aolp_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          topic_ + "/aolp/compressed", pub_qos_);
    }
  }

  // Create publishers for Stokes parameters (S0, S1, S2) — polarized cameras only
  if (is_polarized && publish_stokes_) {
    if (publish_raw_) {
      m_pub_stokes_s0_ = this->create_publisher<sensor_msgs::msg::Image>(
          topic_ + "/stokes_s0", pub_qos_);
      m_pub_stokes_s1_ = this->create_publisher<sensor_msgs::msg::Image>(
          topic_ + "/stokes_s1", pub_qos_);
      m_pub_stokes_s2_ = this->create_publisher<sensor_msgs::msg::Image>(
          topic_ + "/stokes_s2", pub_qos_);
    }
    if (publish_compressed_) {
      m_pub_stokes_s0_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          topic_ + "/stokes_s0/compressed", pub_qos_);
      m_pub_stokes_s1_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          topic_ + "/stokes_s1/compressed", pub_qos_);
      m_pub_stokes_s2_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          topic_ + "/stokes_s2/compressed", pub_qos_);
    }
  }

  // Create publisher for false-color AoLP visualization
  if (is_polarized && publish_aolp_color_) {
    m_pub_aolp_color_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/aolp_color/compressed", pub_qos_);
  }

  // CameraInfo publisher — always created; publishes once per frame with matching header.
  // Provides uncalibrated stub (all-zero K/D/R/P) unless camera_info_url is set.
  m_pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      topic_ + "/camera_info", pub_qos_);
  {
    // Camera name defaults to the node name; CameraInfoManager uses it as the calibration key.
    std::string cam_name = std::string(this->get_name());
    m_camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, cam_name, camera_info_url_);
    if (!camera_info_url_.empty()) {
      if (m_camera_info_manager_->isCalibrated()) {
        log_info("Camera calibration loaded from: " + camera_info_url_);
      } else {
        log_warn("camera_info_url set but calibration failed to load: " + camera_info_url_ +
                 " — publishing uncalibrated stub");
      }
    } else {
      log_info("No camera_info_url set — publishing uncalibrated CameraInfo stub");
    }
  }

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());

  //
  // Diagnostics --------------------------------------------------------------
  //
  m_diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  m_diagnostic_updater_->setHardwareID("arena_camera");
  m_diagnostic_updater_->add("Camera Status", this, &ArenaCameraNode::produce_diagnostics_);

  //
  // GPU JPEG encoder (optional) --------------------------------------------
  // Probed at runtime so a cpu-only x86 build still works without HAS_CUDA.
  //
#ifdef HAS_CUDA
  if (gpu_acceleration_ != "cpu") {
    int device_count = 0;
    cudaError_t cuda_err = cudaGetDeviceCount(&device_count);
    if (cuda_err == cudaSuccess && device_count > 0) {
      nvjpeg_encoder_ = std::make_unique<NvJpegEncoder>(jpeg_quality_);
      if (nvjpeg_encoder_->is_valid()) {
        use_gpu_jpeg_ = true;
        log_info("nvJPEG hardware JPEG encoding enabled (GPU)");
      } else {
        nvjpeg_encoder_.reset();
        log_warn("nvJPEG init failed — falling back to cv::imencode");
      }
#ifdef HAS_POLAR_KERNEL
      // Polarization kernel buffers are allocated lazily on first frame;
      // just mark the flag here so compute_and_publish_dolp_aolp_ uses GPU.
      use_gpu_polar_ = true;
      log_info("CUDA polarization kernel enabled (fused Stokes+DOLP+AoLP)");
#endif
    } else {
      log_info("No CUDA device found — using software JPEG encoding + CPU polarization");
    }
  } else {
    log_info("gpu_acceleration=cpu — using software JPEG encoding");
  }
#else
  log_info("Built without CUDA — using software JPEG encoding");
#endif
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  // Preserve no-op deleter: DestroyDevice() is called explicitly in destructor.
  m_pDevice = std::shared_ptr<Arena::IDevice>(device, [](Arena::IDevice*) { /* no-op */ });
  set_nodes_();
  log_debug("set_nodes_() completed, starting stream...");

  // --- GigE health check: warn about MTU < 9000 ---
  // USB Ethernet adapters typically cap at MTU 1500 which forces ~3400 packets
  // per 5 MP frame.  This is fine at low frame rates but causes packet loss and
  // stream stalls at high rates.  A native PCIe NIC with jumbo frames (9000) is
  // strongly recommended for production use.
  try {
    // Get the camera's IP address from its GenICam node so the route lookup
    // works on any subnet, not just a hardcoded IP.
    std::string cam_ip_str;
    try {
      auto nodemap = m_pDevice->GetNodeMap();
      // GevCurrentIPAddress is a 32-bit integer encoded as dotted-decimal
      int64_t ip_int = Arena::GetNodeValue<int64_t>(nodemap, "GevCurrentIPAddress");
      cam_ip_str =
          std::to_string((ip_int >> 24) & 0xFF) + "." +
          std::to_string((ip_int >> 16) & 0xFF) + "." +
          std::to_string((ip_int >>  8) & 0xFF) + "." +
          std::to_string( ip_int        & 0xFF);
    } catch (...) {
      // Non-GigE camera or node unavailable — skip MTU check
    }

    if (!cam_ip_str.empty()) {
    // Walk the routing table to find which interface reaches the camera.
    std::string cam_iface;
    int cam_mtu = 0;
    std::string route_cmd = "ip -o route get " + cam_ip_str + " 2>/dev/null | awk '{print $3}'";
    FILE* fp = popen(route_cmd.c_str(), "r");
    if (fp) {
      char buf[128]{};
      if (fgets(buf, sizeof(buf), fp)) {
        cam_iface = buf;
        // trim newline
        while (!cam_iface.empty() && (cam_iface.back() == '\n' || cam_iface.back() == '\r'))
          cam_iface.pop_back();
      }
      pclose(fp);
    }
    if (!cam_iface.empty()) {
      std::string mtu_path = "/sys/class/net/" + cam_iface + "/mtu";
      std::ifstream mtu_file(mtu_path);
      if (mtu_file.is_open()) {
        mtu_file >> cam_mtu;
      }
    }
    if (cam_mtu > 0 && cam_mtu < 9000) {
      log_warn("GigE health: interface '" + cam_iface + "' MTU is " +
               std::to_string(cam_mtu) + " (recommended: 9000 for jumbo frames). "
               "Streaming at high frame rates may cause packet loss and freezes. "
               "Consider using a native PCIe GigE NIC or limiting frame rate.");
    } else if (cam_mtu >= 9000) {
      log_info("GigE health: interface '" + cam_iface + "' MTU " +
               std::to_string(cam_mtu) + " — jumbo frames OK");
    }
    }  // end if (!cam_ip_str.empty())
  } catch (...) {
    // Non-critical — don't let a health check prevent streaming
  }

  // StartStream can fail with transient GigE timeouts (e.g., after unclean
  // shutdown or link hiccup).  Retry a few times before giving up.
  constexpr int MAX_STREAM_START_ATTEMPTS = 3;
  constexpr int STREAM_RETRY_DELAY_MS = 2000;
  for (int attempt = 1; attempt <= MAX_STREAM_START_ATTEMPTS; ++attempt) {
    try {
      m_pDevice->StartStream(stream_buffer_count_);
      m_is_streaming_.store(true);
      log_debug("StartStream() completed");
      break;  // success
    } catch (GenICam::GenericException& e) {
      if (attempt < MAX_STREAM_START_ATTEMPTS) {
        log_warn(std::string("StartStream attempt ") + std::to_string(attempt) +
                 "/" + std::to_string(MAX_STREAM_START_ATTEMPTS) +
                 " failed: " + e.what() + " — retrying in " +
                 std::to_string(STREAM_RETRY_DELAY_MS) + "ms...");
        std::this_thread::sleep_for(std::chrono::milliseconds(STREAM_RETRY_DELAY_MS));
      } else {
        log_err(std::string("Failed to start stream after ") +
                std::to_string(MAX_STREAM_START_ATTEMPTS) + " attempts: " + e.what());
        throw;
      }
    } catch (std::exception& e) {
      if (attempt < MAX_STREAM_START_ATTEMPTS) {
        log_warn(std::string("StartStream attempt ") + std::to_string(attempt) +
                 "/" + std::to_string(MAX_STREAM_START_ATTEMPTS) +
                 " failed: " + e.what() + " — retrying in " +
                 std::to_string(STREAM_RETRY_DELAY_MS) + "ms...");
        std::this_thread::sleep_for(std::chrono::milliseconds(STREAM_RETRY_DELAY_MS));
      } else {
        log_err(std::string("Failed to start stream after ") +
                std::to_string(MAX_STREAM_START_ATTEMPTS) + " attempts: " + e.what());
        throw;
      }
    }
  }
  
  m_device_connected_ = true;

  if (!trigger_mode_activated_) {
    log_info("Streaming started with event-driven callbacks - publishing images to " + topic_);
    // Register callback handler for event-driven image acquisition
    m_image_callback_handler_ = std::make_unique<ImageCallbackHandler>(this);
    m_pDevice->RegisterImageCallback(m_image_callback_handler_.get());
    // Start worker thread for async image processing
    // (ArenaSDK requires OnImage to return quickly or the grab thread stalls)
    m_worker_stop_ = false;
    m_worker_has_image_ = false;
    m_worker_thread_ = std::thread(&ArenaCameraNode::worker_thread_func_, this);
  } else {
    log_info("Trigger mode enabled - waiting for trigger service calls");
  }
}

// ---------------------------------------------------------------------------
// JPEG encode helper: uses nvJPEG hardware encoder when available, otherwise
// falls back to cv::imencode. All 7 compressed-image publish sites call this.
// ---------------------------------------------------------------------------
void ArenaCameraNode::jpeg_encode_(const cv::Mat& img, std::vector<uint8_t>& out) {
#ifdef HAS_CUDA
  if (use_gpu_jpeg_ && nvjpeg_encoder_->encode(img, out)) return;
#endif
  std::vector<int> enc_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  cv::imencode(".jpg", img, out, enc_params);
}

// ---------------------------------------------------------------------------
// Lightweight per-frame profiling timer for identifying processing bottlenecks.
// Accumulates per-stage stats across frames and emits an averaged INFO summary
// every log_interval frames. Per-frame raw detail is logged at DEBUG level.
// Zero overhead when disabled.
// ---------------------------------------------------------------------------
namespace {

// Persistent accumulator — one instance per call site, lives for the process lifetime.
struct StageAccumulator {
  struct StageStat {
    std::string name;
    double sum{0};
    double min{std::numeric_limits<double>::max()};
    double max{0};
  };
  std::vector<StageStat> stages;
  uint64_t frame_count{0};

  void add(size_t idx, const std::string& name, double ms) {
    if (idx >= stages.size()) stages.resize(idx + 1);
    stages[idx].name = name;
    stages[idx].sum += ms;
    if (ms < stages[idx].min) stages[idx].min = ms;
    if (ms > stages[idx].max) stages[idx].max = ms;
  }

  // Returns avg summary string and resets accumulators.
  std::string flush_summary() {
    if (stages.empty() || frame_count == 0) return {};
    std::string out;
    double total_avg = 0;
    char buf[80];
    for (const auto& s : stages) {
      double avg = s.sum / frame_count;
      total_avg += avg;
      std::snprintf(buf, sizeof(buf), " %s=%.1f(%.0f-%.0f)",
                    s.name.c_str(), avg, s.min, s.max);
      out += buf;
    }
    std::snprintf(buf, sizeof(buf), " TOTAL=%.1f", total_avg);
    out += buf;
    // Reset
    for (auto& s : stages) { s.sum = 0; s.min = std::numeric_limits<double>::max(); s.max = 0; }
    frame_count = 0;
    return out;
  }
};

struct StageTimer {
  bool enabled;
  std::chrono::steady_clock::time_point start;
  std::chrono::steady_clock::time_point last;
  // Per-frame detail (for DEBUG log)
  std::string frame_result;
  // Pointer to persistent accumulator for this call site
  StageAccumulator* acc{nullptr};
  size_t stage_idx{0};

  explicit StageTimer(bool enable, StageAccumulator* accumulator = nullptr)
      : enabled(enable), acc(accumulator) {
    if (enabled) {
      start = std::chrono::steady_clock::now();
      last = start;
    }
  }

  void mark(const char* name) {
    if (!enabled) return;
    auto now = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(now - last).count();
    char buf[64];
    std::snprintf(buf, sizeof(buf), " %s=%.1f", name, ms);
    frame_result += buf;
    if (acc) acc->add(stage_idx, name, ms);
    stage_idx++;
    last = now;
  }

  // Raw per-frame summary (for DEBUG log).
  std::string frame_summary() const {
    if (!enabled || frame_result.empty()) return {};
    double total = std::chrono::duration<double, std::milli>(last - start).count();
    char buf[64];
    std::snprintf(buf, sizeof(buf), " TOTAL=%.1f", total);
    return frame_result + buf;
  }
};

}  // anonymous namespace

void ArenaCameraNode::compute_and_publish_dolp_aolp_(
    Arena::IImage* pImage,
    const cv::Mat cached_bgr[4],
    const uint8_t* const raw_ch[4],
    int raw_w, int raw_h,
    cv::Mat* out_dolp,
    cv::Mat* out_aolp)
{
  static StageAccumulator dolp_acc;
  StageTimer dtimer(profile_processing_, &dolp_acc);

  const size_t stokes_h = static_cast<size_t>(cached_bgr[0].rows);
  const size_t stokes_w = static_cast<size_t>(cached_bgr[0].cols);

  cv::Mat dolp_u8, aolp_u8;

#ifdef HAS_POLAR_KERNEL
  // --- GPU fast path: fused Stokes + DOLP + AoLP in one CUDA kernel ---
  // Uses raw mono8 channel data directly — no BGR→gray conversion needed.
  bool gpu_ok = false;
  if (use_gpu_polar_ && (publish_dolp_ || publish_aolp_) &&
      raw_ch[0] && raw_ch[1] && raw_ch[2] && raw_ch[3]) {

    dolp_u8.create(raw_h, raw_w, CV_8U);
    aolp_u8.create(raw_h, raw_w, CV_8U);

    cv::Mat s0_u8, s1_u8, s2_u8;
    uint8_t* s0_ptr = nullptr, *s1_ptr = nullptr, *s2_ptr = nullptr;
    if (publish_stokes_) {
      s0_u8.create(raw_h, raw_w, CV_8U);
      s1_u8.create(raw_h, raw_w, CV_8U);
      s2_u8.create(raw_h, raw_w, CV_8U);
      s0_ptr = s0_u8.data; s1_ptr = s1_u8.data; s2_ptr = s2_u8.data;
    }

    gpu_ok = polar_compute_gpu(
      raw_ch, raw_w, raw_h,
      dolp_u8.data, aolp_u8.data,
      polar_bufs_,
      s0_ptr, s1_ptr, s2_ptr);

    if (!gpu_ok) {
      log_warn("CUDA polarization kernel failed — falling back to CPU this frame");
    }

    // stokes_conv captures full upload+kernel+download; stokes/dolp/aolp are zero.
    dtimer.mark("stokes_conv");
    dtimer.mark("stokes");
    dtimer.mark("dolp");
    dtimer.mark("aolp");

    if (gpu_ok && publish_stokes_) {
      // Helper lambda to publish one Stokes component as raw + compressed
      auto pub_stokes = [&](cv::Mat& img,
                            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& raw_pub,
                            rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& cmp_pub) {
        if (publish_raw_ && raw_pub) {
          auto msg = std::make_unique<sensor_msgs::msg::Image>();
          fill_header_(msg->header, pImage);
          msg->height = static_cast<uint32_t>(raw_h);
          msg->width  = static_cast<uint32_t>(raw_w);
          msg->encoding = "mono8"; msg->is_bigendian = 0;
          msg->step = static_cast<uint32_t>(raw_w);
          msg->data.assign(img.data, img.data + img.total());
          raw_pub->publish(std::move(msg));
        }
        if (publish_compressed_ && cmp_pub) {
          auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
          fill_header_(msg->header, pImage);
          msg->format = "jpeg";
          jpeg_encode_(img, msg->data);
          cmp_pub->publish(std::move(msg));
        }
      };
      pub_stokes(s0_u8, m_pub_stokes_s0_, m_pub_stokes_s0_compressed_);
      pub_stokes(s1_u8, m_pub_stokes_s1_, m_pub_stokes_s1_compressed_);
      pub_stokes(s2_u8, m_pub_stokes_s2_, m_pub_stokes_s2_compressed_);
      dtimer.mark("stokes_pub");
    }
  }

  if (gpu_ok) {
    // Publish DOLP
    if (publish_dolp_ && m_pub_dolp_) {
      if (out_dolp) *out_dolp = dolp_u8.clone();
      {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        fill_header_(msg->header, pImage);
        msg->height = stokes_h; msg->width = stokes_w;
        msg->encoding = "mono8"; msg->is_bigendian = 0; msg->step = stokes_w;
        msg->data.assign(dolp_u8.data, dolp_u8.data + dolp_u8.total());
        m_pub_dolp_->publish(std::move(msg));
      }
      if (publish_compressed_ && m_pub_dolp_compressed_) {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        fill_header_(msg->header, pImage);
        msg->format = "jpeg";
        jpeg_encode_(dolp_u8, msg->data);
        m_pub_dolp_compressed_->publish(std::move(msg));
      }
    }
    // Publish AoLP
    if (publish_aolp_ && m_pub_aolp_) {
      if (out_aolp) *out_aolp = aolp_u8.clone();
      {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        fill_header_(msg->header, pImage);
        msg->height = stokes_h; msg->width = stokes_w;
        msg->encoding = "mono8"; msg->is_bigendian = 0; msg->step = stokes_w;
        msg->data.assign(aolp_u8.data, aolp_u8.data + aolp_u8.total());
        m_pub_aolp_->publish(std::move(msg));
      }
      if (publish_compressed_ && m_pub_aolp_compressed_) {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        fill_header_(msg->header, pImage);
        msg->format = "jpeg";
        jpeg_encode_(aolp_u8, msg->data);
        m_pub_aolp_compressed_->publish(std::move(msg));
      }
    }
    // Publish false-color AoLP (HSV: H=angle, S=DOLP, V=200)
    if (publish_aolp_color_ && m_pub_aolp_color_compressed_ &&
        !aolp_u8.empty() && !dolp_u8.empty()) {
      cv::Mat hsv(raw_h, raw_w, CV_8UC3);
      for (int i = 0; i < raw_h * raw_w; ++i) {
        // H: AoLP [0,255] remapped to [0,180] (OpenCV hue range)
        hsv.data[i * 3 + 0] = static_cast<uint8_t>(aolp_u8.data[i] * 180 / 255);
        // S: DOLP directly [0,255]
        hsv.data[i * 3 + 1] = dolp_u8.data[i];
        // V: fixed brightness
        hsv.data[i * 3 + 2] = 200;
      }
      cv::Mat bgr;
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      fill_header_(msg->header, pImage);
      msg->format = "jpeg";
      jpeg_encode_(bgr, msg->data);
      m_pub_aolp_color_compressed_->publish(std::move(msg));
      dtimer.mark("aolp_color");
    }
  } else {
#else
  {
#endif  // HAS_POLAR_KERNEL
    // --- CPU fallback path ---
    // Derive grayscale intensity from the pre-converted BGR mats via cvtColor.
    // cvtColor(BGR→GRAY) uses BT.601 weighting, consistent across all 4 channels.
    auto to_gray_float = [&](size_t idx) -> cv::Mat {
      cv::Mat gray, gray_f;
      cv::cvtColor(cached_bgr[idx], gray, cv::COLOR_BGR2GRAY);
      gray.convertTo(gray_f, CV_32F);
      return gray_f;
    };

    cv::Mat f0   = to_gray_float(0);   // I_0°
    cv::Mat f45  = to_gray_float(1);   // I_45°
    cv::Mat f90  = to_gray_float(2);   // I_90°
    cv::Mat f135 = to_gray_float(3);   // I_135°
    dtimer.mark("stokes_conv");

    // Stokes parameters (vectorized OpenCV operations)
    cv::Mat S0 = f0 + f90;       // Total intensity
    cv::Mat S1 = f0 - f90;       // Horizontal vs vertical
    cv::Mat S2 = f45 - f135;     // Diagonal
    dtimer.mark("stokes");

    // --- DOLP ---
    if (publish_dolp_ && m_pub_dolp_) {
      cv::Mat S1_sq, S2_sq, numerator, dolp_float;
      cv::multiply(S1, S1, S1_sq);
      cv::multiply(S2, S2, S2_sq);
      cv::sqrt(S1_sq + S2_sq, numerator);

      dolp_float = cv::Mat::zeros(stokes_h, stokes_w, CV_32F);
      cv::Mat s0_safe;
      cv::max(S0, 1e-5f, s0_safe);
      cv::divide(numerator, s0_safe, dolp_float);
      dolp_float.setTo(0.0f, S0 <= 1e-5f);
      cv::min(dolp_float, 1.0f, dolp_float);
      cv::max(dolp_float, 0.0f, dolp_float);
      dolp_float.convertTo(dolp_u8, CV_8U, 255.0);

      if (out_dolp) *out_dolp = dolp_u8.clone();
      {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        fill_header_(msg->header, pImage);
        msg->height = stokes_h; msg->width = stokes_w;
        msg->encoding = "mono8"; msg->is_bigendian = 0; msg->step = stokes_w;
        msg->data.assign(dolp_u8.data, dolp_u8.data + dolp_u8.total());
        m_pub_dolp_->publish(std::move(msg));
      }
      if (publish_compressed_ && m_pub_dolp_compressed_) {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        fill_header_(msg->header, pImage);
        msg->format = "jpeg";
        jpeg_encode_(dolp_u8, msg->data);
        m_pub_dolp_compressed_->publish(std::move(msg));
      }
    }
    dtimer.mark("dolp");

    // --- AoLP ---
    if (publish_aolp_ && m_pub_aolp_) {
      cv::Mat phase_mat;
      cv::phase(S1, S2, phase_mat, false);  // atan2(S2, S1) remapped to [0, 2π)
      constexpr float inv_2pi = 0.15915494309189534561f;
      cv::multiply(phase_mat, inv_2pi, phase_mat);
      cv::add(phase_mat, 0.5f, phase_mat);
      cv::subtract(phase_mat, 1.0f, phase_mat, phase_mat >= 1.0f);
      phase_mat *= 255.0f;
      cv::min(phase_mat, 255.0f, phase_mat);
      cv::max(phase_mat, 0.0f, phase_mat);
      phase_mat.convertTo(aolp_u8, CV_8U);

      if (out_aolp) *out_aolp = aolp_u8.clone();
      {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        fill_header_(msg->header, pImage);
        msg->height = stokes_h; msg->width = stokes_w;
        msg->encoding = "mono8"; msg->is_bigendian = 0; msg->step = stokes_w;
        msg->data.assign(aolp_u8.data, aolp_u8.data + aolp_u8.total());
        m_pub_aolp_->publish(std::move(msg));
      }
      if (publish_compressed_ && m_pub_aolp_compressed_) {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        fill_header_(msg->header, pImage);
        msg->format = "jpeg";
        jpeg_encode_(aolp_u8, msg->data);
        m_pub_aolp_compressed_->publish(std::move(msg));
      }
    }
    dtimer.mark("aolp");

    // --- Stokes outputs (CPU path) ---
    if (publish_stokes_) {
      // S0 = f0 + f90 ∈ [0,510] → scale 0.5 → [0,255]
      cv::Mat s0_f = S0 * 0.5f;
      cv::min(s0_f, 255.0f, s0_f);
      cv::Mat s0_u8; s0_f.convertTo(s0_u8, CV_8U);

      // S1 = f0 - f90 ∈ [-255,255] → (x+255)/2 → [0,255]
      cv::Mat s1_f = (S1 + 255.0f) * 0.5f;
      cv::min(cv::max(s1_f, 0.0f), 255.0f, s1_f);
      cv::Mat s1_u8; s1_f.convertTo(s1_u8, CV_8U);

      // S2 = f45 - f135 → same encoding as S1
      cv::Mat s2_f = (S2 + 255.0f) * 0.5f;
      cv::min(cv::max(s2_f, 0.0f), 255.0f, s2_f);
      cv::Mat s2_u8; s2_f.convertTo(s2_u8, CV_8U);

      auto pub_stokes = [&](cv::Mat& img,
                            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& raw_pub,
                            rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& cmp_pub) {
        if (publish_raw_ && raw_pub) {
          auto msg = std::make_unique<sensor_msgs::msg::Image>();
          fill_header_(msg->header, pImage);
          msg->height = static_cast<uint32_t>(stokes_h);
          msg->width  = static_cast<uint32_t>(stokes_w);
          msg->encoding = "mono8"; msg->is_bigendian = 0;
          msg->step = static_cast<uint32_t>(stokes_w);
          msg->data.assign(img.data, img.data + img.total());
          raw_pub->publish(std::move(msg));
        }
        if (publish_compressed_ && cmp_pub) {
          auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
          fill_header_(msg->header, pImage);
          msg->format = "jpeg";
          jpeg_encode_(img, msg->data);
          cmp_pub->publish(std::move(msg));
        }
      };
      pub_stokes(s0_u8, m_pub_stokes_s0_, m_pub_stokes_s0_compressed_);
      pub_stokes(s1_u8, m_pub_stokes_s1_, m_pub_stokes_s1_compressed_);
      pub_stokes(s2_u8, m_pub_stokes_s2_, m_pub_stokes_s2_compressed_);
      dtimer.mark("stokes_pub");
    }

    // --- False-color AoLP (CPU path) ---
    if (publish_aolp_color_ && m_pub_aolp_color_compressed_ &&
        !aolp_u8.empty() && !dolp_u8.empty()) {
      cv::Mat hsv(stokes_h, stokes_w, CV_8UC3);
      for (int i = 0; i < static_cast<int>(stokes_h * stokes_w); ++i) {
        hsv.data[i * 3 + 0] = static_cast<uint8_t>(aolp_u8.data[i] * 180 / 255);
        hsv.data[i * 3 + 1] = dolp_u8.data[i];
        hsv.data[i * 3 + 2] = 200;
      }
      cv::Mat bgr;
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      fill_header_(msg->header, pImage);
      msg->format = "jpeg";
      jpeg_encode_(bgr, msg->data);
      m_pub_aolp_color_compressed_->publish(std::move(msg));
      dtimer.mark("aolp_color");
    }
  }  // end CPU fallback

  // Log DOLP/AoLP profiling detail if enabled
  if (profile_processing_) {
    log_debug("Frame " + std::to_string(pImage->GetFrameId()) +
              " DOLP/AoLP detail (ms):" + dtimer.frame_summary());
    dolp_acc.frame_count++;
    if (dolp_acc.frame_count >= 30) {
      log_info("[profiler] DOLP/AoLP avg over 30 frames (ms):" + dolp_acc.flush_summary());
    }
  }

  // mono_for_stokes automatically cleaned up by RAII when going out of scope
}


// ---------------------------------------------------------------------------
// Worker thread: waits for a deep-copied image, then does the heavy processing
// (SplitChannels, Convert, publish) off the Arena SDK grab thread.
// ---------------------------------------------------------------------------
void ArenaCameraNode::worker_thread_func_()
{
  log_info("Image processing worker thread started");
  while (true) {
    Arena::IImage* image_to_process = nullptr;
    {
      std::unique_lock<std::mutex> lock(m_worker_mutex_);
      m_worker_cv_.wait(lock, [this]{ return m_worker_has_image_ || m_worker_stop_; });
      if (m_worker_stop_ && !m_worker_has_image_) {
        break;  // Clean shutdown
      }
      image_to_process = m_pending_image_;
      m_pending_image_ = nullptr;
      m_worker_has_image_ = false;
    }
    if (image_to_process) {
      process_copied_image_(image_to_process);
      // Destroy the deep copy we made in the callback
      Arena::ImageFactory::Destroy(image_to_process);
    }
  }
  log_info("Image processing worker thread exiting");
}

// ---------------------------------------------------------------------------
// process_copied_image_: heavy-weight image processing on the worker thread.
// Receives a deep copy (ImageFactory::Copy) so Arena SDK buffers are not held.
// ---------------------------------------------------------------------------
void ArenaCameraNode::process_copied_image_(Arena::IImage* pImage)
{
  auto processing_start = std::chrono::steady_clock::now();

  try {
    if (!pImage) return;

    // Check for incomplete frames (missing GigE packets) — common on MTU 1500 links
    if (pImage->IsIncomplete()) {
      m_incomplete_frames_++;
      log_warn("Incomplete frame received (frame " + std::to_string(pImage->GetFrameId()) +
               ", total incomplete: " + std::to_string(m_incomplete_frames_) +
               ") — skipping. Check GigE MTU and cable quality.");
      return;
    }

    // Log pixel format on first image
    static bool format_logged = false;
    if (!format_logged) {
      uint64_t pf = pImage->GetPixelFormat();
      std::string info = "Camera pixel format detected: " + get_pixel_format_name(pf);
      if (is_polarized_format(pf)) {
        info += " (polarized camera - will extract 4 channels: 0\u00b0, 45\u00b0, 90\u00b0, 135\u00b0)";
      } else {
        info += " (standard camera - will convert to BGR8 for publishing)";
      }
      log_info(info);
      format_logged = true;
    }

    // On first frame: read PTP sync status and measure the clock offset between
    // the camera's hardware clock and the ROS system clock. Sets m_ptp_synced_
    // so fill_header_() can choose the right timestamp source for all subsequent frames.
    if (!m_clock_offset_logged_) {
      // Read PTP status from camera
      std::string ptp_status = "unknown";
      int64_t ptp_offset_ns = 0;
      try {
        auto nodemap = m_pDevice->GetNodeMap();
        GenICam::gcstring s = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PtpStatus");
        ptp_status = std::string(s.c_str());
        try {
          ptp_offset_ns = Arena::GetNodeValue<int64_t>(nodemap, "PtpOffsetFromMaster");
        } catch (...) {}
      } catch (...) {}

      bool ptp_synced = (ptp_status == "Slave");
      m_ptp_synced_.store(ptp_synced);

      uint64_t cam_ns    = pImage->GetTimestampNs();
      int64_t  ros_ns    = this->now().nanoseconds();
      int64_t  offset_ms = (ros_ns - static_cast<int64_t>(cam_ns)) / 1000000;

      if (ptp_synced) {
        log_info("PTP synchronized (Slave) — using camera hardware timestamps. "
                 "Camera-ROS offset: " + std::to_string(offset_ms) +
                 " ms, PtpOffsetFromMaster: " + std::to_string(ptp_offset_ns) + " ns");
      } else {
        log_warn("PTP NOT synchronized (PtpStatus: " + ptp_status +
                 ") — falling back to ROS system clock. Camera-ROS offset: " +
                 std::to_string(offset_ms) +
                 " ms. Run ptp4l on the host for accurate timestamps.");
      }
      m_clock_offset_logged_ = true;
    }

    // --- Publish raw image if enabled ---
    if (publish_raw_ && m_pub_) {
      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      msg_form_image_(pImage, *p_image_msg);
      m_pub_->publish(std::move(p_image_msg));
    }

    // --- Publish compressed image (non-polarized only on main topic) ---
    if (publish_compressed_ && m_pub_compressed_) {
      uint64_t pixel_format = pImage->GetPixelFormat();
      if (!is_polarized_format(pixel_format)) {
        auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        fill_header_(compressed_msg->header, pImage);
        compressed_msg->format = "jpeg";
        arena_camera::ArenaImagePtr converted_image;
        Arena::IImage* image_to_compress = nullptr;
        try {
          converted_image = arena_camera::make_arena_image_ptr(
              Arena::ImageFactory::Convert(pImage, PFNC_BGR8));
          image_to_compress = converted_image.get();
        } catch (...) {
          image_to_compress = pImage;
        }
        int cvType = CV_8UC3;
        if (image_to_compress->GetBitsPerPixel() == 8) cvType = CV_8UC1;
        cv::Mat img_mat(image_to_compress->GetHeight(),
                       image_to_compress->GetWidth(), cvType,
                       const_cast<void*>(static_cast<const void*>(image_to_compress->GetData())));
        jpeg_encode_(img_mat, compressed_msg->data);
        m_pub_compressed_->publish(std::move(compressed_msg));
      }
    }

    m_images_published_++;

    // --- Publish CameraInfo (once per frame, always) ---
    if (m_pub_camera_info_ && m_camera_info_manager_) {
      auto info_msg = m_camera_info_manager_->getCameraInfo();
      fill_header_(info_msg.header, pImage);
      info_msg.width = static_cast<uint32_t>(pImage->GetWidth());
      info_msg.height = static_cast<uint32_t>(pImage->GetHeight());
      m_pub_camera_info_->publish(info_msg);
    }

    // --- Latency profiling ---
    if (profile_latency_) {
      auto t1 = std::chrono::steady_clock::now();
      // Callback latency: Arena callback entry → publish
      int64_t t0_ns = m_callback_t0_ns_.load(std::memory_order_relaxed);
      if (t0_ns != 0) {
        auto t0 = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(t0_ns));
        double cb_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        m_last_callback_latency_ms_.store(cb_ms, std::memory_order_relaxed);
        if (latency_mode_ == "callback" || latency_mode_ == "both") {
          log_debug("Frame " + std::to_string(pImage->GetFrameId()) +
                    " callback_latency=" + std::to_string(cb_ms) + "ms");
        }
      }
      // Hardware latency: camera hardware timestamp → publish
      if (use_hw_timestamp_for_latency_) {
        uint64_t hw_ts_ns = pImage->GetTimestampNs();
        if (hw_ts_ns != 0) {
          // Hardware timestamps use camera clock; compare using wall clock epoch if PTP-synced.
          // For a best-effort estimate, compute using the ROS system clock.
          auto now_ns = static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now().time_since_epoch()).count());
          if (now_ns > hw_ts_ns) {
            double hw_ms = static_cast<double>(now_ns - hw_ts_ns) / 1e6;
            m_last_hw_latency_ms_.store(hw_ms, std::memory_order_relaxed);
            log_debug("Frame " + std::to_string(pImage->GetFrameId()) +
                      " hw_latency=" + std::to_string(hw_ms) + "ms");
          }
        }
      }
    }

    // Note: FPS / watchdog bookkeeping is handled in handle_camera_image_() (the
    // callback on the grab thread) so that timestamps are accurate even when the
    // worker is slow.  Do NOT update m_fps_frame_count_ or m_last_frame_time_ here
    // to avoid data races between the callback thread and this worker thread.

    log_debug(std::string("image ") + std::to_string(pImage->GetFrameId()) +
             " published to " + topic_);

    // --- Extract and publish polarization channels ---
    try {
      uint64_t pixel_format = pImage->GetPixelFormat();
      if (is_polarized_format(pixel_format)) {
          static StageAccumulator proc_acc;
          StageTimer ptimer(profile_processing_, &proc_acc);

          arena_camera::ArenaImageVector channels(
              Arena::ImageFactory::SplitChannels(pImage));
          ptimer.mark("split");

          if (channels.size() == 4) {
            // Display mat storage — filled during processing, used for tiled debug window
            cv::Mat display_ch[4];    // 4 polarization channels (BGR)
            cv::Mat display_max;      // max-combined (BGR)
            cv::Mat display_dolp;     // DOLP (mono8)
            cv::Mat display_aolp;     // AoLP (mono8)

            // Cached BGR mats — reused for max-combined to avoid duplicate Bayer→BGR conversion
            cv::Mat cached_bgr[4];

            // Raw mono8 pointers from SplitChannels — used by GPU kernel to skip BGR→gray.
            // The ArenaImageVector 'channels' stays alive for the entire block, so these
            // pointers remain valid until compute_and_publish_dolp_aolp_ returns.
            const uint8_t* raw_ch[4] = {
              static_cast<const uint8_t*>(channels[0]->GetData()),
              static_cast<const uint8_t*>(channels[1]->GetData()),
              static_cast<const uint8_t*>(channels[2]->GetData()),
              static_cast<const uint8_t*>(channels[3]->GetData()),
            };
            const int raw_w = static_cast<int>(channels[0]->GetWidth());
            const int raw_h = static_cast<int>(channels[0]->GetHeight());

            struct ChannelInfo {
              size_t index;
              std::string name;
              rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr* raw_pub;
              rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr* compressed_pub;
            };
            std::vector<ChannelInfo> channel_infos = {
              {0, "0deg", &m_pub_pol_0deg_, &m_pub_pol_0deg_compressed_},
              {1, "45deg", &m_pub_pol_45deg_, &m_pub_pol_45deg_compressed_},
              {2, "90deg", &m_pub_pol_90deg_, &m_pub_pol_90deg_compressed_},
              {3, "135deg", &m_pub_pol_135deg_, &m_pub_pol_135deg_compressed_}
            };
            // Debayer needed when publishing channels or max (both require BGR).
            const bool need_bgr = publish_pol_channels_ ||
                                  ((publish_raw_ && m_pub_pol_max_) ||
                                   (publish_compressed_ && m_pub_pol_max_compressed_));
            if (need_bgr) {
              for (const auto& info : channel_infos) {
                arena_camera::ArenaImagePtr bgr_image(
                    Arena::ImageFactory::Convert(channels[info.index], PFNC_BGR8));
                if (!validate_bgr8_format(bgr_image.get(), "polarization channel " + info.name)) {
                  continue;
                }
                cv::Mat bgr_mat(bgr_image->GetHeight(), bgr_image->GetWidth(), CV_8UC3,
                               const_cast<void*>(static_cast<const void*>(bgr_image->GetData())));
                cached_bgr[info.index] = bgr_mat.clone();
                if (display_images_active_ && info.index < 4) {
                  display_ch[info.index] = cached_bgr[info.index];
                }
              }
              ptimer.mark("ch_debayer");
            }

            // Per-channel publish (only when publish_pol_channels_ is true)
            if (publish_pol_channels_) {
              for (const auto& info : channel_infos) {
                if (cached_bgr[info.index].empty()) continue;
                const cv::Mat& bgr_mat = cached_bgr[info.index];
                if (publish_raw_ && *info.raw_pub) {
                  auto pol_msg = std::make_unique<sensor_msgs::msg::Image>();
                  fill_header_(pol_msg->header, pImage);
                  pol_msg->height = bgr_mat.rows;
                  pol_msg->width = bgr_mat.cols;
                  pol_msg->encoding = "bgr8";
                  pol_msg->is_bigendian = 0;
                  pol_msg->step = pol_msg->width * 3;
                  size_t data_size = static_cast<size_t>(bgr_mat.rows) * bgr_mat.cols * 3;
                  pol_msg->data.resize(data_size);
                  std::memcpy(&pol_msg->data[0], bgr_mat.data, data_size);
                  (*info.raw_pub)->publish(std::move(pol_msg));
                }
                if (publish_compressed_ && *info.compressed_pub) {
                  auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                  fill_header_(compressed_msg->header, pImage);
                  compressed_msg->format = "jpeg";
                  jpeg_encode_(bgr_mat, compressed_msg->data);
                  (*info.compressed_pub)->publish(std::move(compressed_msg));
                }
              }
              ptimer.mark("ch_jpeg");
            }

            // Max-combined image (reuses cached BGR mats from per-channel loop)
            if ((publish_raw_ && m_pub_pol_max_) || (publish_compressed_ && m_pub_pol_max_compressed_)) {
              bool all_valid = !cached_bgr[0].empty() && !cached_bgr[1].empty() &&
                               !cached_bgr[2].empty() && !cached_bgr[3].empty();
              if (!all_valid) {
                log_warn("Skipping max-combined image — not all channels converted successfully");
              } else {
                cv::Mat max01, max23, max_combined;
                cv::max(cached_bgr[0], cached_bgr[1], max01);
                cv::max(cached_bgr[2], cached_bgr[3], max23);
                cv::max(max01, max23, max_combined);
                // Capture for debug display window
                if (display_images_active_) display_max = max_combined.clone();
                if (publish_raw_ && m_pub_pol_max_) {
                  size_t height = max_combined.rows;
                  size_t width = max_combined.cols;
                  auto max_msg = std::make_unique<sensor_msgs::msg::Image>();
                  fill_header_(max_msg->header, pImage);
                  max_msg->height = height;
                  max_msg->width = width;
                  max_msg->encoding = "bgr8";
                  max_msg->is_bigendian = 0;
                  max_msg->step = width * 3;
                  size_t data_size = height * width * 3;
                  max_msg->data.resize(data_size);
                  std::memcpy(&max_msg->data[0], max_combined.data, data_size);
                  m_pub_pol_max_->publish(std::move(max_msg));
                }
                if (publish_compressed_ && m_pub_pol_max_compressed_) {
                  auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                  fill_header_(compressed_msg->header, pImage);
                  compressed_msg->format = "jpeg";
                  jpeg_encode_(max_combined, compressed_msg->data);
                  m_pub_pol_max_compressed_->publish(std::move(compressed_msg));
                }
              }
            }
            ptimer.mark("max");

            // Compute and publish DOLP / AoLP
            // GPU path uses raw_ch directly; CPU fallback uses cached_bgr (requires
            // publish_pol_channels_ to have been run first to populate it).
            if ((publish_dolp_ && m_pub_dolp_) || (publish_aolp_ && m_pub_aolp_)) {
              compute_and_publish_dolp_aolp_(pImage, cached_bgr,
                  raw_ch, raw_w, raw_h,
                  display_images_active_ ? &display_dolp : nullptr,
                  display_images_active_ ? &display_aolp : nullptr);
            }
            ptimer.mark("dolp_aolp");

            // Log per-stage profiling if enabled
            if (profile_processing_) {
              // DEBUG: raw per-frame detail (visible with --log-level DEBUG)
              log_debug("Frame " + std::to_string(pImage->GetFrameId()) +
                        " processing (ms):" + ptimer.frame_summary());
              // INFO: averaged summary every 30 frames (~3s at 10 FPS)
              proc_acc.frame_count++;
              if (proc_acc.frame_count >= 30) {
                log_info("[profiler] processing avg over 30 frames (ms):" + proc_acc.flush_summary());
              }
            }

            // ==============================================================
            // Debug display window: 4x2 tiled grid
            // Row 1: 0°, 45°, 90°, 135°
            // Row 2: Max, DOLP, AoLP, (blank)
            // Press 's' to save all tiles, 'q' to close window
            // ==============================================================
            if (display_images_active_) {
              try {
                // Determine tile size from first available channel
                int tile_w = 0, tile_h = 0;
                for (int i = 0; i < 4; i++) {
                  if (!display_ch[i].empty()) {
                    tile_w = display_ch[i].cols;
                    tile_h = display_ch[i].rows;
                    break;
                  }
                }
                
                if (tile_w > 0 && tile_h > 0) {
                  // Build fisheye undistortion maps once from calibration
                  if (!m_undistort_maps_ready_ && m_camera_info_manager_ &&
                      m_camera_info_manager_->isCalibrated()) {
                    auto ci = m_camera_info_manager_->getCameraInfo();
                    if (ci.distortion_model == "equidistant" && ci.d.size() == 4) {
                      cv::Mat K = (cv::Mat_<double>(3, 3) <<
                          ci.k[0], ci.k[1], ci.k[2],
                          ci.k[3], ci.k[4], ci.k[5],
                          ci.k[6], ci.k[7], ci.k[8]);
                      cv::Mat D = (cv::Mat_<double>(1, 4) <<
                          ci.d[0], ci.d[1], ci.d[2], ci.d[3]);
                      cv::fisheye::initUndistortRectifyMap(
                          K, D, cv::Mat::eye(3, 3, CV_64F), K,
                          cv::Size(tile_w, tile_h), CV_16SC2,
                          m_undistort_map1_, m_undistort_map2_);
                      m_undistort_maps_ready_ = true;
                      log_info("Fisheye undistortion maps built for debug display");
                    }
                  }

                  // Optionally remap a mat through the undistortion maps
                  auto maybe_undistort = [&](const cv::Mat& src) -> cv::Mat {
                    if (!m_display_undistorted_ || !m_undistort_maps_ready_ || src.empty())
                      return src;
                    cv::Mat dst;
                    cv::remap(src, dst, m_undistort_map1_, m_undistort_map2_, cv::INTER_LINEAR);
                    return dst;
                  };

                  // Scale tiles down for display — target ~1920px wide for 4 columns
                  int target_tile_w = 480;
                  double scale = static_cast<double>(target_tile_w) / tile_w;
                  int scaled_w = target_tile_w;
                  int scaled_h = static_cast<int>(tile_h * scale);
                  
                  // Helper: resize a BGR or mono8 mat to BGR at display scale
                  auto to_display_tile = [&](const cv::Mat& src, const std::string& label) -> cv::Mat {
                    cv::Mat display;
                    if (src.empty()) {
                      display = cv::Mat::zeros(scaled_h, scaled_w, CV_8UC3);
                    } else if (src.channels() == 1) {
                      cv::Mat colored;
                      cv::cvtColor(src, colored, cv::COLOR_GRAY2BGR);
                      cv::resize(colored, display, cv::Size(scaled_w, scaled_h));
                    } else {
                      cv::resize(src, display, cv::Size(scaled_w, scaled_h));
                    }
                    // Add label text
                    cv::putText(display, label, cv::Point(10, 25),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    return display;
                  };
                  
                  // Build tiles (with optional fisheye undistortion)
                  std::string undist_suffix = m_display_undistorted_ ? " [U]" : "";
                  cv::Mat t0   = to_display_tile(maybe_undistort(display_ch[0]), "0 deg" + undist_suffix);
                  cv::Mat t45  = to_display_tile(maybe_undistort(display_ch[1]), "45 deg" + undist_suffix);
                  cv::Mat t90  = to_display_tile(maybe_undistort(display_ch[2]), "90 deg" + undist_suffix);
                  cv::Mat t135 = to_display_tile(maybe_undistort(display_ch[3]), "135 deg" + undist_suffix);
                  cv::Mat tmax = to_display_tile(maybe_undistort(display_max), "Max Combined" + undist_suffix);
                  cv::Mat tdolp = to_display_tile(maybe_undistort(display_dolp), "DOLP" + undist_suffix);
                  cv::Mat taolp = to_display_tile(maybe_undistort(display_aolp), "AoLP" + undist_suffix);
                  cv::Mat tblank = cv::Mat::zeros(scaled_h, scaled_w, CV_8UC3);
                  // Snapshot thread-shared data before the overlay block
                  double overlay_fps;
                  {
                    std::lock_guard<std::mutex> slock(m_stats_mutex_);
                    overlay_fps = m_calculated_fps_;
                  }
                  std::string overlay_exposure, overlay_target_bright, overlay_calc_mean;
                  {
                    std::lock_guard<std::mutex> clock(m_diag_cache_mutex_);
                    auto get = [&](const char* k) -> std::string {
                      auto it = m_diag_genicam_cache_.find(k);
                      return it != m_diag_genicam_cache_.end() ? it->second : "N/A";
                    };
                    overlay_exposure      = get("ExposureTime (us)");
                    overlay_target_bright = get("TargetBrightness");
                    overlay_calc_mean     = get("CalculatedMean");
                  }

                  // Overlay diagnostics info on the info tile
                  {
                    int y = 25;
                    const int line_h = 28;
                    auto put_line = [&](const std::string& text, cv::Scalar color = cv::Scalar(0, 200, 200)) {
                      cv::putText(tblank, text, cv::Point(10, y),
                                  cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 1);
                      y += line_h;
                    };

                    put_line("--- Diagnostics ---", cv::Scalar(0, 255, 0));

                    // FPS (from live counter, not cache)
                    std::ostringstream fps_ss;
                    fps_ss << std::fixed << std::setprecision(1) << overlay_fps;
                    put_line("FPS: " + fps_ss.str());

                    put_line("ExposureTime: " + overlay_exposure + " us");
                    put_line("TargetBright: " + overlay_target_bright);
                    put_line("CalcMean: " + overlay_calc_mean);
                    
                    y += 10;  // spacer
                    std::string undist_status = m_undistort_maps_ready_
                        ? (m_display_undistorted_ ? "ON" : "off")
                        : "no cal";
                    put_line("Undistort [u]: " + undist_status,
                             m_display_undistorted_ ? cv::Scalar(0, 255, 100) : cv::Scalar(100, 100, 100));
                    put_line("'s' save | 'q' quit", cv::Scalar(100, 100, 100));
                  }
                  
                  // Compose 4x2 grid
                  cv::Mat row1, row2, tiled;
                  cv::hconcat(std::vector<cv::Mat>{t0, t45, t90, t135}, row1);
                  cv::hconcat(std::vector<cv::Mat>{tmax, tdolp, taolp, tblank}, row2);
                  cv::vconcat(row1, row2, tiled);
                  
                  cv::imshow("Polarization Debug", tiled);
                  int key = cv::waitKey(1) & 0xFF;
                  
                  if (key == 'u' || key == 'U') {
                    if (m_undistort_maps_ready_) {
                      m_display_undistorted_ = !m_display_undistorted_;
                      log_info(std::string("Debug display undistortion ") +
                               (m_display_undistorted_ ? "enabled" : "disabled"));
                    } else {
                      log_warn("No calibration loaded — undistortion unavailable");
                    }
                  } else if (key == 'q' || key == 'Q') {
                    display_images_active_ = false;
                    cv::destroyAllWindows();
                    cv::waitKey(1);  // flush the destroy event so the window actually closes
                    log_info("Debug display window closed by user - press Ctrl+C to fully stop the node");
                  } else if (key == 's' || key == 'S') {
                    auto now = std::chrono::system_clock::now();
                    auto time_t_now = std::chrono::system_clock::to_time_t(now);
                    struct tm local_tm {};
                    localtime_r(&time_t_now, &local_tm);

                    // Create the session folder once per run (on first save)
                    if (save_session_dir_.empty()) {
                      std::ostringstream session_oss;
                      session_oss << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
                      const char* home_env = getenv("HOME");
                      std::string home = home_env ? home_env : "/tmp";
                      save_session_dir_ = home + "/lucid_camera_images/session_" + session_oss.str();
                      std::filesystem::create_directories(save_session_dir_);
                    }

                    // Per-save timestamp + frame ID prefix
                    std::ostringstream ts_oss;
                    ts_oss << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
                    uint64_t frame_id = pImage->GetFrameId();
                    std::string prefix = save_session_dir_ + "/" + ts_oss.str() +
                                         "_frame_" + std::to_string(frame_id) + "_";
                    
                    // Save full-resolution images — always both raw and undistorted
                    struct SavePair { cv::Mat mat; std::string name; };
                    std::vector<SavePair> to_save = {
                      {display_ch[0], "pol_0deg"}, {display_ch[1], "pol_45deg"},
                      {display_ch[2], "pol_90deg"}, {display_ch[3], "pol_135deg"},
                      {display_max, "pol_max"}, {display_dolp, "dolp"}, {display_aolp, "aolp"}
                    };
                    int saved = 0;
                    for (const auto& sp : to_save) {
                      if (!sp.mat.empty()) {
                        cv::imwrite(prefix + sp.name + ".png", sp.mat);
                        saved++;
                        if (m_undistort_maps_ready_) {
                          cv::Mat undist;
                          cv::remap(sp.mat, undist, m_undistort_map1_, m_undistort_map2_,
                                    cv::INTER_LINEAR);
                          cv::imwrite(prefix + sp.name + "_undist.png", undist);
                          saved++;
                        }
                      }
                    }
                    log_info("Saved " + std::to_string(saved) + " images to " + save_session_dir_ +
                             " (frame " + std::to_string(frame_id) + ")");
                  }
                }
              } catch (const std::exception& e) {
                log_warn(std::string("Debug display error: ") + e.what());
                display_images_active_ = false;
              }
            }
          }
      }
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("GenICam error extracting polarization channels: ") + e.what());
    } catch (std::exception& e) {
      log_warn(std::string("Error extracting polarization channels: ") + e.what());
    } catch (...) {
      log_warn("Unknown exception extracting polarization channels");
    }

    // Processing time metrics — guard with m_stats_mutex_ (read by diagnostics timer thread)
    {
      auto processing_end = std::chrono::steady_clock::now();
      double elapsed_ms = std::chrono::duration<double, std::milli>(
          processing_end - processing_start).count();
      std::lock_guard<std::mutex> slock(m_stats_mutex_);
      m_last_processing_time_ms_ = elapsed_ms;
      if (elapsed_ms > m_max_processing_time_ms_) {
        m_max_processing_time_ms_ = elapsed_ms;
      }
      m_total_processing_time_ms_ += elapsed_ms;
      m_processing_time_samples_++;
    }

  } catch (GenICam::GenericException& e) {
    m_image_publish_errors_++;
    log_err(std::string("GenICam exception while publishing an image: ") + e.what());
  } catch (std::exception& e) {
    m_image_publish_errors_++;
    log_err(std::string("Exception while publishing an image: ") + e.what());
  } catch (...) {
    m_image_publish_errors_++;
    log_err("Unknown exception while publishing an image");
  }
}

// ---------------------------------------------------------------------------
// update_fps_locked_: update FPS rolling average.
// MUST be called while holding m_stats_mutex_.
// ---------------------------------------------------------------------------
void ArenaCameraNode::update_fps_locked_(const std::chrono::steady_clock::time_point& now)
{
  m_fps_frame_count_++;
  if (m_fps_frame_count_ == 1) {
    m_fps_last_time_ = now;
  } else {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - m_fps_last_time_).count();
    if (elapsed >= 1000) {
      m_calculated_fps_ = (m_fps_frame_count_ * 1000.0) / elapsed;
      m_fps_frame_count_ = 0;
      m_fps_last_time_ = now;
    }
  }
}

// ---------------------------------------------------------------------------
// handle_camera_image_: called on the Arena SDK grab thread.
// Must return FAST. Deep-copies the image and hands off to the worker thread.
// ---------------------------------------------------------------------------
void ArenaCameraNode::handle_camera_image_(Arena::IImage* pImage)
{
  if (!m_pDevice || !m_is_streaming_.load()) {
    return;
  }

  if (!pImage) return;

  try {
    // Deep-copy the image so the SDK buffer is released immediately when
    // this callback returns.  ImageFactory::Copy is a fast memcpy.
    Arena::IImage* copy = Arena::ImageFactory::Copy(pImage);

    // Record callback-entry timestamp for latency profiling (if enabled).
    // Stored as ns since epoch so it can be read atomically by the worker thread.
    if (profile_latency_) {
      auto t0 = std::chrono::steady_clock::now();
      m_callback_t0_ns_.store(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count(),
          std::memory_order_relaxed);
    }

    {
      std::lock_guard<std::mutex> lock(m_worker_mutex_);
      // If the worker hasn't consumed the previous frame yet, drop it
      if (m_pending_image_) {
        Arena::ImageFactory::Destroy(m_pending_image_);
        m_backpressure_events_++;
        if (m_backpressure_events_ % 100 == 1) {
          log_warn(std::string("Backpressure: dropping frame (total dropped: ") +
                   std::to_string(m_backpressure_events_) + ")");
        }
      }
      m_pending_image_ = copy;
      m_worker_has_image_ = true;
    }
    m_worker_cv_.notify_one();

    // Update watchdog / FPS counters under m_stats_mutex_ to prevent data race
    // with produce_diagnostics_() which reads these from the ROS timer thread.
    bool just_recovered = false;
    {
      std::lock_guard<std::mutex> slock(m_stats_mutex_);
      auto now = std::chrono::steady_clock::now();
      m_last_frame_time_ = now;
      m_watchdog_initialized_ = true;
      if (m_camera_frozen_) {
        m_camera_frozen_ = false;
        just_recovered = true;
      }
      update_fps_locked_(now);
    }
    // Log outside the lock (logging can be slow / involve I/O)
    if (just_recovered) {
      log_info("Camera recovered - frames are being received again");
    }
  } catch (...) {
    log_err("Exception in OnImage callback during image copy");
  }
}

void ArenaCameraNode::fill_header_(std_msgs::msg::Header& header, Arena::IImage* pImage)
{
  if (m_ptp_synced_.load()) {
    // PTP synchronized: camera clock tracks wall time — use hardware timestamp
    // for accurate exposure-time alignment with IMU/LiDAR.
    uint64_t ts_ns = pImage->GetTimestampNs();
    header.stamp.sec     = static_cast<int32_t>(ts_ns / 1000000000ULL);
    header.stamp.nanosec = static_cast<uint32_t>(ts_ns % 1000000000ULL);
  } else {
    // No PTP: fall back to ROS system clock at processing time.
    // This has ~2-15 ms jitter vs true exposure time but is at least
    // in the correct time domain for downstream consumers.
    header.stamp = this->now();
  }
  header.frame_id = frame_id_;
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // Get actual image dimensions from the image itself
    auto image_width = pImage->GetWidth();
    auto image_height = pImage->GetHeight();

    // 1 ) Header — timestamp and TF frame_id via shared helper
    fill_header_(image_msg.header, pImage);

    //
    // 2 ) Height
    //
    image_msg.height = static_cast<uint32_t>(image_height);

    //
    // 3 ) Width
    //
    image_msg.width = static_cast<uint32_t>(image_width);

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = image_width * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    // Use GetSizeFilled() to get the actual payload size from the camera
    auto image_data_length_in_bytes = pImage->GetSizeFilled();
    image_msg.data.resize(image_data_length_in_bytes);
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Check if device is still valid (protects against race with destructor)
  if (!m_pDevice) {
    response->message = "Device not available";
    response->success = false;
    return;
  }
  
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
    return;
  }

  log_debug("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // Wait for trigger to be armed with a timeout to prevent infinite busy-spin.
    // SDK docs recommend polling TriggerArmed before executing TriggerSoftware.
    bool triggerArmed = false;
    constexpr int MAX_TRIGGER_WAIT_ITERATIONS = 500;   // 500 × 10ms = 5 seconds
    int waitForTriggerCount = 0;
    do {
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");
      if (!triggerArmed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        waitForTriggerCount++;
        if (waitForTriggerCount % 10 == 0) {
          log_debug("waiting for trigger to be armed (" +
                    std::to_string(waitForTriggerCount * 10) + "ms elapsed)");
        }
        if (waitForTriggerCount >= MAX_TRIGGER_WAIT_ITERATIONS) {
          std::string msg = "Timed out waiting for trigger to arm after " +
                            std::to_string(MAX_TRIGGER_WAIT_ITERATIONS * 10) + "ms";
          log_err(msg);
          response->message = msg;
          response->success = false;
          return;
        }
      }
    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    int trigger_timeout_ms = std::max(5000, static_cast<int>(exposure_time_ / 1000.0) + 2000);
    pImage = m_pDevice->GetImage(trigger_timeout_ms);

    // Check for incomplete frames (missing GigE packets)
    if (pImage->IsIncomplete()) {
      m_incomplete_frames_++;
      std::string msg = "Incomplete frame on trigger (frame " +
                        std::to_string(pImage->GetFrameId()) + ")";
      log_warn(msg);
      response->message = msg;
      response->success = false;
      this->m_pDevice->RequeueBuffer(pImage);
      return;
    }

    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    m_images_published_++;
    {
      std::lock_guard<std::mutex> slock(m_stats_mutex_);
      update_fps_locked_(std::chrono::steady_clock::now());
    }
    response->message = msg;
    response->success = true;

    log_debug(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (GenICam::GenericException& e) {
    // Catch GenICam before std::exception — GenericException inherits std::exception
    m_image_publish_errors_++;
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  catch (std::exception& e) {
    m_image_publish_errors_++;
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  // Update diagnostics after trigger operation
  m_diagnostic_updater_->force_update();
}

rcl_interfaces::msg::SetParametersResult ArenaCameraNode::on_set_parameters_(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
    if (param.get_name() == "target_brightness") {
      int64_t brightness = param.as_int();
      if (brightness < 0 || brightness > 255) {
        result.successful = false;
        result.reason = "target_brightness " + std::to_string(brightness) + " out of range [0, 255]";
        log_warn(result.reason);
        return result;
      }

      if (!m_pDevice) {
        target_brightness_ = brightness;
        log_info("target_brightness cached to " + std::to_string(brightness) + " (device not yet connected)");
      } else {
        try {
          auto nodemap = m_pDevice->GetNodeMap();
          Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness", brightness);
          target_brightness_ = brightness;
          log_info("TargetBrightness set to " + std::to_string(brightness));
        } catch (GenICam::GenericException& e) {
          result.successful = false;
          result.reason = std::string("GenICam error: ") + e.what();
          log_err(result.reason);
        } catch (std::exception& e) {
          result.successful = false;
          result.reason = std::string("Error: ") + e.what();
          log_err(result.reason);
        }
      }
    }
  }

  return result;
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  size_t index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::reconnect_()
{
  // Guard against concurrent reconnect attempts
  bool expected = false;
  if (!m_reconnecting_.compare_exchange_strong(expected, true)) {
    return;  // another reconnect is already in progress
  }

  m_reconnect_count_++;
  log_info("Reconnect attempt #" + std::to_string(m_reconnect_count_) + " starting...");

  // -------------------------------------------------------------------------
  // Step 1: Teardown — mirrors the destructor, but without closing the system
  // -------------------------------------------------------------------------

  // Stop worker thread
  {
    std::lock_guard<std::mutex> lock(m_worker_mutex_);
    m_worker_stop_ = true;
  }
  m_worker_cv_.notify_one();
  if (m_worker_thread_.joinable()) {
    m_worker_thread_.join();
    log_info("Reconnect: worker thread stopped");
  }
  // Discard any pending image
  if (m_pending_image_) {
    Arena::ImageFactory::Destroy(m_pending_image_);
    m_pending_image_ = nullptr;
  }

  // Deregister image callback
  if (m_image_callback_handler_ && m_pDevice) {
    try {
      m_pDevice->DeregisterImageCallback(m_image_callback_handler_.get());
      m_image_callback_handler_.reset();
      log_info("Reconnect: image callback deregistered");
    } catch (...) {
      log_warn("Reconnect: failed to deregister image callback (continuing)");
    }
  }

  // Stop streaming
  bool was_streaming = m_is_streaming_.exchange(false);
  if (m_pDevice && was_streaming) {
    try {
      m_pDevice->StopStream();
      log_info("Reconnect: stream stopped");
    } catch (...) {
      log_warn("Reconnect: failed to stop stream (continuing)");
    }
  }

  // Destroy device
  if (m_pDevice && m_pSystem) {
    try {
      Arena::IDevice* pDevice = m_pDevice.get();
      m_pDevice.reset();
      m_pSystem->DestroyDevice(pDevice);
      log_info("Reconnect: device destroyed");
    } catch (...) {
      log_warn("Reconnect: failed to destroy device (continuing)");
      m_pDevice.reset();
    }
  }
  m_device_connected_ = false;

  // Short delay to let the camera hardware reset
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // -------------------------------------------------------------------------
  // Step 2: Re-discover and re-create device
  // -------------------------------------------------------------------------
  constexpr int RECONNECT_RETRY_DELAY_SEC = 3;
  int attempt = 0;
  bool success = false;

  while (rclcpp::ok()) {
    attempt++;
    if (reconnect_max_attempts_ > 0 && attempt > reconnect_max_attempts_) {
      log_err("Reconnect: gave up after " + std::to_string(reconnect_max_attempts_) +
              " attempts. Node requires manual restart.");
      m_reconnect_errors_++;
      m_reconnecting_.store(false);
      return;
    }

    log_info("Reconnect: device discovery attempt " + std::to_string(attempt) + "...");
    try {
      m_pSystem->UpdateDevices(100);
      auto device_infos = m_pSystem->GetDevices();
      if (device_infos.empty()) {
        log_info("Reconnect: no cameras found — retrying in " +
                 std::to_string(RECONNECT_RETRY_DELAY_SEC) + "s...");
        std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_RETRY_DELAY_SEC));
        continue;
      }

      // create_device_ros_ selects the correct camera (by serial if set)
      auto* pDevice = create_device_ros_();
      m_pDevice = std::shared_ptr<Arena::IDevice>(pDevice, [](Arena::IDevice*) { /* no-op */ });
      success = true;
      log_info("Reconnect: camera found and device created");
      break;
    } catch (const std::exception& e) {
      m_reconnect_errors_++;
      log_warn("Reconnect attempt " + std::to_string(attempt) +
               " failed: " + e.what() + " — retrying in " +
               std::to_string(RECONNECT_RETRY_DELAY_SEC) + "s...");
      std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_RETRY_DELAY_SEC));
    }
  }

  if (!success) {
    log_err("Reconnect: aborted (rclcpp shutdown)");
    m_reconnecting_.store(false);
    return;
  }

  // -------------------------------------------------------------------------
  // Step 3: Re-configure and restart stream
  // -------------------------------------------------------------------------
  try {
    set_nodes_();

    // StartStream with retry (same logic as run_())
    constexpr int MAX_STREAM_START_ATTEMPTS = 3;
    constexpr int STREAM_RETRY_DELAY_MS = 2000;
    for (int sa = 1; sa <= MAX_STREAM_START_ATTEMPTS; ++sa) {
      try {
        m_pDevice->StartStream(stream_buffer_count_);
        m_is_streaming_.store(true);
        break;
      } catch (const std::exception& e) {
        if (sa < MAX_STREAM_START_ATTEMPTS) {
          std::this_thread::sleep_for(std::chrono::milliseconds(STREAM_RETRY_DELAY_MS));
        } else {
          throw;
        }
      }
    }

    m_device_connected_ = true;

    if (!trigger_mode_activated_) {
      m_image_callback_handler_ = std::make_unique<ImageCallbackHandler>(this);
      m_pDevice->RegisterImageCallback(m_image_callback_handler_.get());
      {
        std::lock_guard<std::mutex> lock(m_worker_mutex_);
        m_worker_stop_ = false;
        m_worker_has_image_ = false;
      }
      m_worker_thread_ = std::thread(&ArenaCameraNode::worker_thread_func_, this);
    }

    // Reset watchdog state so the watchdog doesn't immediately re-trigger
    {
      std::lock_guard<std::mutex> slock(m_stats_mutex_);
      m_camera_frozen_ = false;
      m_watchdog_initialized_ = false;
      m_last_frame_time_ = std::chrono::steady_clock::now();
    }

    log_info("Reconnect #" + std::to_string(m_reconnect_count_) + " successful — streaming resumed");
  } catch (const std::exception& e) {
    m_reconnect_errors_++;
    log_err("Reconnect: failed to restart stream: " + std::string(e.what()));
  }

  m_reconnecting_.store(false);
}
