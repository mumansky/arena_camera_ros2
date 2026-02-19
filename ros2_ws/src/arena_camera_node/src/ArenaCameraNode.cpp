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
 * The legacy blocking publish_images_() loop is retained but not used in the
 * current callback-based architecture.
 */

#include <cstring>    // memcpy
#include <stdexcept>  // std::runtime_error
#include <string>
#include <fstream>    // file I/O
#include <vector>     // vector
#include <cstdlib>    // getenv
#include <yaml-cpp/yaml.h>  // YAML parsing

// OpenCV
#include <opencv2/opencv.hpp>

// ROS
#include "rmw/types.h"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// ArenaSDK
#include "ArenaCameraNode.h"
#include "arena_image_raii.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

// ============================================================================
// Pixel Format Constants
// ============================================================================
// These constants define the PFNC (PixelFormat Naming Convention) values used
// by Arena SDK for different image formats. Using named constants instead of
// magic numbers improves code readability and maintainability.
// Note: PFNC_BGR8 is already defined as a macro in Arena SDK's PFNC.h

namespace PixelFormat {
  // PolarizedAngles_0d_45d_90d_135d_BayerRG8 format
  // Used by polarized cameras (e.g., PHX050S1-QC) that capture 4 polarization
  // angles (0°, 45°, 90°, 135°) in a single Bayer pattern image.
  // Each 2x2 Bayer quad contains one pixel for each polarization angle.
  constexpr uint64_t PFNC_POLARIZED_BAYER_RG8 = 0x8220020F;
}

// ============================================================================
// Helper Functions for Pixel Format Detection
// ============================================================================

/**
 * @brief Check if the given pixel format is a polarized format
 * @param format PFNC pixel format value from Arena SDK
 * @return true if the format is a polarized format, false otherwise
 */
inline bool is_polarized_format(uint64_t format) {
  return format == PixelFormat::PFNC_POLARIZED_BAYER_RG8;
}

/**
 * @brief Check if the given pixel format is supported by this node
 * @param format PFNC pixel format value from Arena SDK
 * @return true if the format is supported, false otherwise
 */
inline bool is_supported_format(uint64_t format) {
  // Currently we support:
  // 1. Polarized format for PHX050S1-QC cameras
  // 2. Any format that can be converted to BGR8 (handled by Arena SDK Convert)
  // If format is polarized, we handle it specially
  // Otherwise, we attempt conversion to BGR8
  return true;  // We attempt to handle all formats via conversion
}

/**
 * @brief Get a human-readable name for a pixel format
 * @param format PFNC pixel format value from Arena SDK
 * @return String describing the pixel format
 */
inline std::string get_pixel_format_name(uint64_t format) {
  if (format == PixelFormat::PFNC_POLARIZED_BAYER_RG8) {
    return "PolarizedAngles_0d_45d_90d_135d_BayerRG8";
  } else if (format == PFNC_BGR8) {
    return "BGR8";
  } else {
    char buf[32];
    snprintf(buf, sizeof(buf), "0x%08lX", static_cast<unsigned long>(format));
    return std::string("Unknown (") + buf + ")";
  }
}

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
  if (actual != PFNC_BGR8) {
    // Log as error since downstream code assumes BGR8 layout
    RCLCPP_ERROR(rclcpp::get_logger("arena_camera_node"),
        "Unexpected pixel format after conversion in %s: expected BGR8 (0x%lx), got 0x%lx. "
        "Image data may be corrupted.",
        context.c_str(), static_cast<unsigned long>(PFNC_BGR8), static_cast<unsigned long>(actual));
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

/**
 * @brief Helper to safely read a string value from YAML config
 * @param config YAML node to read from
 * @param key Parameter key name
 * @param default_val Default value if key not found or conversion fails
 * @return The parameter value or default
 */
static std::string config_string(const YAML::Node& config, const std::string& key, const std::string& default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<std::string>();
    }
  } catch (const YAML::Exception&) {
    // Type conversion failed; fall through to default
  }
  return default_val;
}

/**
 * @brief Helper to safely read a bool value from YAML config
 * @param config YAML node to read from
 * @param key Parameter key name
 * @param default_val Default value if key not found or conversion fails
 * @return The parameter value or default
 */
static bool config_bool(const YAML::Node& config, const std::string& key, bool default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<bool>();
    }
  } catch (const YAML::Exception&) {
    // Type conversion failed; fall through to default
  }
  return default_val;
}

/**
 * @brief Helper to safely read a double value from YAML config
 * @param config YAML node to read from
 * @param key Parameter key name
 * @param default_val Default value if key not found or conversion fails
 * @return The parameter value or default
 */
static double config_double(const YAML::Node& config, const std::string& key, double default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<double>();
    }
  } catch (const YAML::Exception&) {
    // Type conversion failed; fall through to default
  }
  return default_val;
}

/**
 * @brief Helper to safely read an int64 value from YAML config
 * @param config YAML node to read from
 * @param key Parameter key name
 * @param default_val Default value if key not found or conversion fails
 * @return The parameter value or default
 */
static int64_t config_int64(const YAML::Node& config, const std::string& key, int64_t default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<int64_t>();
    }
  } catch (const YAML::Exception&) {
    // Type conversion failed; fall through to default
  }
  return default_val;
}

/**
 * @brief Helper to safely read an int value from YAML config
 * @param config YAML node to read from
 * @param key Parameter key name
 * @param default_val Default value if key not found or conversion fails
 * @return The parameter value or default
 */
static int config_int(const YAML::Node& config, const std::string& key, int default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<int>();
    }
  } catch (const YAML::Exception&) {
    // Type conversion failed; fall through to default
  }
  return default_val;
}

/**
 * @brief Helper to check if a key exists in YAML config
 * @param config YAML node to check
 * @param key Parameter key name
 * @return true if key exists, false otherwise
 */
static bool config_has(const YAML::Node& config, const std::string& key)
{
  return config && config[key];
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
    is_passed_short_exposure_enable_ = short_exposure_enable_;

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
    is_passed_acquisition_frame_rate_enable_ = config_has(m_config_params_, "acquisition_frame_rate_enable");

    currentParam = "acquisition_frame_rate";
    acquisition_frame_rate_ = config_double(m_config_params_, "acquisition_frame_rate", -1.0);
    is_passed_acquisition_frame_rate_ = acquisition_frame_rate_ >= 0.0;

    currentParam = "trigger_mode";
    trigger_mode_activated_ = config_bool(m_config_params_, "trigger_mode", false);

    // QoS settings
    currentParam = "qos_history";
    pub_qos_history_ = config_string(m_config_params_, "qos_history", "");
    is_passed_pub_qos_history_ = !pub_qos_history_.empty();

    currentParam = "qos_history_depth";
    pub_qos_history_depth_ = static_cast<size_t>(config_int64(m_config_params_, "qos_history_depth", 0));
    is_passed_pub_qos_history_depth_ = config_has(m_config_params_, "qos_history_depth") && pub_qos_history_depth_ > 0;

    currentParam = "qos_reliability";
    pub_qos_reliability_ = config_string(m_config_params_, "qos_reliability", "");
    is_passed_pub_qos_reliability_ = !pub_qos_reliability_.empty();

    // Image publishing options
    currentParam = "publish_raw";
    publish_raw_ = config_bool(m_config_params_, "publish_raw", true);

    currentParam = "publish_compressed";
    publish_compressed_ = config_bool(m_config_params_, "publish_compressed", false);

    currentParam = "jpeg_quality";
    jpeg_quality_ = config_int(m_config_params_, "jpeg_quality", 80);

    // Watchdog settings (Task 20)
    currentParam = "watchdog_timeout_sec";
    watchdog_timeout_sec_ = config_double(m_config_params_, "watchdog_timeout_sec", 5.0);

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
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [](Arena::ISystem*) {
        // No-op: cleanup handled in destructor
      });
  m_pSystem.reset(Arena::OpenSystem());

  // No-op deleter for device - cleanup handled in destructor
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [](Arena::IDevice*) {
        // No-op: cleanup handled in destructor
      });

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
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // TODO
      // should thorow instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
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
  
  // Create publishers for all polarization channels (if either raw or compressed is enabled)
  if (publish_raw_ || publish_compressed_) {
    m_pub_pol_0deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_0deg", pub_qos_);
    m_pub_pol_45deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_45deg", pub_qos_);
    m_pub_pol_90deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_90deg", pub_qos_);
    m_pub_pol_135deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_135deg", pub_qos_);
  }
  
  // Create compressed publishers for all polarization channels (only if compressed enabled)
  if (publish_compressed_) {
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
  if (publish_raw_ || publish_compressed_) {
    m_pub_pol_max_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_ + "/pol_max", pub_qos_);
  }
  if (publish_compressed_) {
    m_pub_pol_max_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_ + "/pol_max/compressed", pub_qos_);
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
  m_pDevice.reset(device);
  set_nodes_();
  log_debug("set_nodes_() completed, starting stream...");

  // --- GigE health check: warn about MTU < 9000 ---
  // USB Ethernet adapters typically cap at MTU 1500 which forces ~3400 packets
  // per 5 MP frame.  This is fine at low frame rates but causes packet loss and
  // stream stalls at high rates.  A native PCIe NIC with jumbo frames (9000) is
  // strongly recommended for production use.
  try {
    // Walk /sys/class/net/*/address looking for the interface whose route
    // covers the camera subnet.  Lightweight — no privileges needed.
    std::string cam_iface;
    int cam_mtu = 0;
    FILE* fp = popen("ip -o route get 172.24.0.30 2>/dev/null | awk '{print $3}'", "r");
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
  } catch (...) {
    // Non-critical — don't let a health check prevent streaming
  }

  // StartStream can fail with transient GigE timeouts (e.g., after unclean
  // shutdown or link hiccup).  Retry a few times before giving up.
  constexpr int MAX_STREAM_START_ATTEMPTS = 3;
  constexpr int STREAM_RETRY_DELAY_MS = 2000;
  for (int attempt = 1; attempt <= MAX_STREAM_START_ATTEMPTS; ++attempt) {
    try {
      m_pDevice->StartStream();
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

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  auto last_diagnostics_update = std::chrono::steady_clock::now();
  const auto diagnostics_update_interval = std::chrono::seconds(1);

  log_info("Streaming started - publishing images to " + topic_);

  while (rclcpp::ok()) {
    try {
      pImage = m_pDevice->GetImage(1000);
      
      // Publish raw image if enabled
      if (publish_raw_ && m_pub_) {
        auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
        msg_form_image_(pImage, *p_image_msg);
        m_pub_->publish(std::move(p_image_msg));
      }
      
      // Publish compressed image if enabled
      if (publish_compressed_ && m_pub_compressed_) {
        uint64_t pixel_format = pImage->GetPixelFormat();
        
        // Skip compression for polarized formats on main topic
        // (compressed polarized data is available on pol_0deg/compressed topic)
        if (!is_polarized_format(pixel_format)) {
          auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
          compressed_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
          compressed_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
          compressed_msg->header.frame_id = std::to_string(pImage->GetFrameId());
          compressed_msg->format = "jpeg";
          
          // Use RAII for converted image to ensure cleanup on exception
          arena_camera::ArenaImagePtr converted_image;
          Arena::IImage* image_to_compress = nullptr;
          
          // Try to convert to BGR8 for compression compatibility
          try {
            converted_image = arena_camera::make_arena_image_ptr(
                Arena::ImageFactory::Convert(pImage, PFNC_BGR8));
            image_to_compress = converted_image.get();
          } catch (...) {
            // If conversion fails, try the original format
            // This handles cases where the image is already in a JPEG-compatible format
            image_to_compress = pImage;
          }
          
          // Create cv::Mat and compress
          int cvType = CV_8UC3;
          if (image_to_compress->GetBitsPerPixel() == 8) {
            cvType = CV_8UC1;
          }
          
          cv::Mat img_mat(image_to_compress->GetHeight(), 
                         image_to_compress->GetWidth(), 
                         cvType,
                         const_cast<void*>(static_cast<const void*>(image_to_compress->GetData())));
          
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
          cv::imencode(".jpg", img_mat, compressed_msg->data, params);
          
          // converted_image is automatically destroyed by RAII when going out of scope
          
          m_pub_compressed_->publish(std::move(compressed_msg));
        }
      }
      
      m_images_published_++;
      
      // Update FPS calculation
      m_fps_frame_count_++;
      auto now = std::chrono::steady_clock::now();
      if (m_fps_frame_count_ == 1) {
        m_fps_last_time_ = now;
      } else {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_fps_last_time_).count();
        if (elapsed >= 1000) {  // Update FPS every second
          m_calculated_fps_ = (m_fps_frame_count_ * 1000.0) / elapsed;
          m_fps_frame_count_ = 0;
          m_fps_last_time_ = now;
        }
      }

      log_debug(std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_);
      
      // Extract and publish all polarization channels if format is polarized
      try {
        uint64_t pixel_format = pImage->GetPixelFormat();
        // Check if this is a polarized format
        if (is_polarized_format(pixel_format)) {
            // Use Arena SDK to split channels - wrap in RAII for exception safety
            arena_camera::ArenaImageVector channels(
                Arena::ImageFactory::SplitChannels(pImage));
            
            if (channels.size() == 4) {
              // Define channel info: index, name, publisher, compressed_publisher
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
              
              // Process each channel - use RAII for converted BGR images
              for (const auto& info : channel_infos) {
                // Convert channel from BayerRG8 to BGR8 with RAII wrapper
                arena_camera::ArenaImagePtr bgr_image(
                    Arena::ImageFactory::Convert(channels[info.index], PFNC_BGR8));
                
                // Validate that conversion actually produced BGR8 format
                if (!validate_bgr8_format(bgr_image.get(), "polarization channel " + info.name)) {
                  continue;  // Skip this channel if format is unexpected
                }
                
                // Publish raw polarization channel if enabled
                if (publish_raw_ && *info.raw_pub) {
                  auto pol_msg = std::make_unique<sensor_msgs::msg::Image>();
                  pol_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                  pol_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                  pol_msg->header.frame_id = std::to_string(pImage->GetFrameId());
                  pol_msg->height = bgr_image->GetHeight();
                  pol_msg->width = bgr_image->GetWidth();
                  pol_msg->encoding = "bgr8";
                  pol_msg->is_bigendian = 0;
                  pol_msg->step = pol_msg->width * 3;
                  
                  size_t data_size = bgr_image->GetHeight() * bgr_image->GetWidth() * 3;
                  pol_msg->data.resize(data_size);
                  std::memcpy(&pol_msg->data[0], bgr_image->GetData(), data_size);
                  
                  (*info.raw_pub)->publish(std::move(pol_msg));
                }
                
                // Publish compressed polarization channel if enabled
                if (publish_compressed_ && *info.compressed_pub) {
                  auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                  compressed_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                  compressed_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                  compressed_msg->header.frame_id = std::to_string(pImage->GetFrameId());
                  compressed_msg->format = "jpeg";
                  
                  // Convert to cv::Mat and compress with JPEG
                  cv::Mat bgr_mat(bgr_image->GetHeight(), bgr_image->GetWidth(), CV_8UC3, 
                                 const_cast<void*>(static_cast<const void*>(bgr_image->GetData())));
                  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                  cv::imencode(".jpg", bgr_mat, compressed_msg->data, params);
                  
                  (*info.compressed_pub)->publish(std::move(compressed_msg));
                }
                
                // bgr_image is automatically destroyed by RAII when going out of scope
              }
              
              // Create max-combined image from all 4 polarization channels
              if ((publish_raw_ && m_pub_pol_max_) || (publish_compressed_ && m_pub_pol_max_compressed_)) {
                // Convert all channels to BGR8 first - use RAII wrapper for exception safety
                arena_camera::ArenaImageVector bgr_channels;
                bool all_valid = true;
                for (size_t i = 0; i < 4; i++) {
                  Arena::IImage* converted = Arena::ImageFactory::Convert(channels[i], PFNC_BGR8);
                  bgr_channels.push_back(converted);
                  if (!validate_bgr8_format(converted, "max-combined channel " + std::to_string(i))) {
                    all_valid = false;
                  }
                }
                
                if (!all_valid) {
                  log_warn("Skipping max-combined image due to unexpected pixel format after conversion");
                } else {
                
                // Get dimensions from first channel
                size_t height = bgr_channels[0]->GetHeight();
                size_t width = bgr_channels[0]->GetWidth();
                
                // Create cv::Mat wrappers for all channels (no data copy)
                cv::Mat mat0(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[0]->GetData())));
                cv::Mat mat1(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[1]->GetData())));
                cv::Mat mat2(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[2]->GetData())));
                cv::Mat mat3(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[3]->GetData())));
                
                // Combine using vectorized OpenCV operations (like NumPy)
                cv::Mat temp1, temp2, max_mat;
                cv::max(mat0, mat1, temp1);
                cv::max(mat2, mat3, temp2);
                cv::max(temp1, temp2, max_mat);
                
                // Publish raw max-combined image
                if (publish_raw_ && m_pub_pol_max_) {
                  auto max_msg = std::make_unique<sensor_msgs::msg::Image>();
                  max_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                  max_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                  max_msg->header.frame_id = std::to_string(pImage->GetFrameId());
                  max_msg->height = height;
                  max_msg->width = width;
                  max_msg->encoding = "bgr8";
                  max_msg->is_bigendian = 0;
                  max_msg->step = width * 3;
                  max_msg->data.assign(max_mat.data, max_mat.data + max_mat.total() * max_mat.elemSize());
                  
                  m_pub_pol_max_->publish(std::move(max_msg));
                }
                
                // Publish compressed max-combined image
                if (publish_compressed_ && m_pub_pol_max_compressed_) {
                  auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                  compressed_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                  compressed_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                  compressed_msg->header.frame_id = std::to_string(pImage->GetFrameId());
                  compressed_msg->format = "jpeg";
                  
                  // Compress with JPEG (max_mat already computed above)
                  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                  cv::imencode(".jpg", max_mat, compressed_msg->data, params);
                  
                  m_pub_pol_max_compressed_->publish(std::move(compressed_msg));
                }
                
                // bgr_channels automatically cleaned up by RAII when going out of scope
                }  // end else (all_valid)
              }
            }
            
            // channels automatically cleaned up by RAII when going out of scope
          }
      } catch (std::exception& e) {
        log_warn(std::string("Error extracting polarization channels: ") + e.what());
      }
      
      this->m_pDevice->RequeueBuffer(pImage);

    } catch (std::exception& e) {
      m_image_publish_errors_++;
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
        log_warn(std::string("Exception occurred while publishing an image\n") +
                 e.what());
      }
    }

    // Force update diagnostics at a fixed interval since we're in a blocking loop
    auto now = std::chrono::steady_clock::now();
    if (now - last_diagnostics_update >= diagnostics_update_interval) {
      m_diagnostic_updater_->force_update();
      last_diagnostics_update = now;
    }
  };
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
        compressed_msg->header.stamp.sec  = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
        compressed_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
        compressed_msg->header.frame_id = std::to_string(pImage->GetFrameId());
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
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        cv::imencode(".jpg", img_mat, compressed_msg->data, params);
        m_pub_compressed_->publish(std::move(compressed_msg));
      }
    }

    m_images_published_++;

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
          arena_camera::ArenaImageVector channels(
              Arena::ImageFactory::SplitChannels(pImage));
          if (channels.size() == 4) {
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
            for (const auto& info : channel_infos) {
              arena_camera::ArenaImagePtr bgr_image(
                  Arena::ImageFactory::Convert(channels[info.index], PFNC_BGR8));
              if (!validate_bgr8_format(bgr_image.get(), "polarization channel " + info.name)) {
                continue;
              }
              if (publish_raw_ && *info.raw_pub) {
                auto pol_msg = std::make_unique<sensor_msgs::msg::Image>();
                pol_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                pol_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                pol_msg->header.frame_id = std::to_string(pImage->GetFrameId());
                pol_msg->height = bgr_image->GetHeight();
                pol_msg->width = bgr_image->GetWidth();
                pol_msg->encoding = "bgr8";
                pol_msg->is_bigendian = 0;
                pol_msg->step = pol_msg->width * 3;
                size_t data_size = bgr_image->GetHeight() * bgr_image->GetWidth() * 3;
                pol_msg->data.resize(data_size);
                std::memcpy(&pol_msg->data[0], bgr_image->GetData(), data_size);
                (*info.raw_pub)->publish(std::move(pol_msg));
              }
              if (publish_compressed_ && *info.compressed_pub) {
                auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                compressed_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                compressed_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                compressed_msg->header.frame_id = std::to_string(pImage->GetFrameId());
                compressed_msg->format = "jpeg";
                cv::Mat bgr_mat(bgr_image->GetHeight(), bgr_image->GetWidth(), CV_8UC3,
                               const_cast<void*>(static_cast<const void*>(bgr_image->GetData())));
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                cv::imencode(".jpg", bgr_mat, compressed_msg->data, params);
                (*info.compressed_pub)->publish(std::move(compressed_msg));
              }
            }
            // Max-combined image
            if ((publish_raw_ && m_pub_pol_max_) || (publish_compressed_ && m_pub_pol_max_compressed_)) {
              arena_camera::ArenaImageVector bgr_channels;
              bool all_valid = true;
              for (size_t i = 0; i < 4; i++) {
                Arena::IImage* converted = Arena::ImageFactory::Convert(channels[i], PFNC_BGR8);
                bgr_channels.push_back(converted);
                if (!validate_bgr8_format(converted, "max-combined channel " + std::to_string(i))) {
                  all_valid = false;
                }
              }
              if (!all_valid) {
                log_warn("Skipping max-combined image due to unexpected pixel format");
              } else {
                size_t height = bgr_channels[0]->GetHeight();
                size_t width  = bgr_channels[0]->GetWidth();
                cv::Mat mat0(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[0]->GetData())));
                cv::Mat mat1(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[1]->GetData())));
                cv::Mat mat2(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[2]->GetData())));
                cv::Mat mat3(height, width, CV_8UC3, const_cast<void*>(static_cast<const void*>(bgr_channels[3]->GetData())));
                cv::Mat max01, max23, max_combined;
                cv::max(mat0, mat1, max01);
                cv::max(mat2, mat3, max23);
                cv::max(max01, max23, max_combined);
                if (publish_raw_ && m_pub_pol_max_) {
                  auto max_msg = std::make_unique<sensor_msgs::msg::Image>();
                  max_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                  max_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                  max_msg->header.frame_id = std::to_string(pImage->GetFrameId());
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
                  compressed_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
                  compressed_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
                  compressed_msg->header.frame_id = std::to_string(pImage->GetFrameId());
                  compressed_msg->format = "jpeg";
                  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                  cv::imencode(".jpg", max_combined, compressed_msg->data, params);
                  m_pub_pol_max_compressed_->publish(std::move(compressed_msg));
                }
              }  // end all_valid
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

    // Processing time metrics
    auto processing_end = std::chrono::steady_clock::now();
    m_last_processing_time_ms_ = std::chrono::duration<double, std::milli>(
        processing_end - processing_start).count();
    if (m_last_processing_time_ms_ > m_max_processing_time_ms_) {
      m_max_processing_time_ms_ = m_last_processing_time_ms_;
    }
    m_total_processing_time_ms_ += m_last_processing_time_ms_;
    m_processing_time_samples_++;

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

    // Update watchdog / FPS counters (lightweight, safe in callback)
    m_fps_frame_count_++;
    auto now = std::chrono::steady_clock::now();
    m_last_frame_time_ = now;
    m_watchdog_initialized_ = true;
    if (m_camera_frozen_) {
      log_info("Camera recovered - frames are being received again");
      m_camera_frozen_ = false;
    }
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
  } catch (...) {
    log_err("Exception in OnImage callback during image copy");
  }
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // Get actual image dimensions from the image itself
    auto image_width = pImage->GetWidth();
    auto image_height = pImage->GetHeight();

    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

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
  }

  log_debug("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_debug("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    m_images_published_++;
    
    // Update FPS calculation
    m_fps_frame_count_++;
    auto now = std::chrono::steady_clock::now();
    if (m_fps_frame_count_ == 1) {
      m_fps_last_time_ = now;
    } else {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_fps_last_time_).count();
      if (elapsed >= 1000) {  // Update FPS every second
        m_calculated_fps_ = (m_fps_frame_count_ * 1000.0) / elapsed;
        m_fps_frame_count_ = 0;
        m_fps_last_time_ = now;
      }
    }
    
    response->message = msg;
    response->success = true;

    log_debug(msg);
    this->m_pDevice->RequeueBuffer(pImage);

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

  catch (GenICam::GenericException& e) {
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

  // Update diagnostics after trigger operation
  m_diagnostic_updater_->force_update();
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

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
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
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // TODO only if it was passed by ros arg
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
          log_warn("\tSkipping ExposureAuto setting.");
          return;
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
        log_warn(std::string("\t") + reason);
        return;
      }
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAuto: ") + e.what());
      return;
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

void ArenaCameraNode::produce_diagnostics_(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  // Watchdog: detect frozen camera (no new frames within timeout)
  if (m_device_connected_ && m_watchdog_initialized_ && !trigger_mode_activated_) {
    auto now = std::chrono::steady_clock::now();
    double elapsed_sec = std::chrono::duration<double>(now - m_last_frame_time_).count();
    
    if (elapsed_sec > watchdog_timeout_sec_) {
      if (!m_camera_frozen_) {
        m_camera_frozen_ = true;
        log_err("Watchdog: Camera appears frozen - no new frames for " +
                std::to_string(elapsed_sec) + "s (timeout: " +
                std::to_string(watchdog_timeout_sec_) + "s). Last frame " +
                std::to_string(m_images_published_) + " published.");
      }
    }
  }

  if (m_device_connected_) {
    if (m_camera_frozen_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "Camera frozen - no new frames received");
    } else if (m_image_publish_errors_ > 0 || m_backpressure_events_ > 0) {
      std::string warn_msg = "Camera connected with";
      if (m_image_publish_errors_ > 0) {
        warn_msg += " errors";
      }
      if (m_backpressure_events_ > 0) {
        if (m_image_publish_errors_ > 0) warn_msg += " and";
        warn_msg += " backpressure";
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
  stat.add("Images Published", std::to_string(m_images_published_));
  stat.add("Publish Errors", std::to_string(m_image_publish_errors_));
  stat.add("Calculated FPS", std::to_string(m_calculated_fps_));
  stat.add("Trigger Mode", trigger_mode_activated_ ? "enabled" : "disabled");
  stat.add("Topic", topic_);
  
  // Backpressure metrics (Task 4)
  stat.add("Backpressure Events", std::to_string(m_backpressure_events_));
  stat.add("Last Processing Time (ms)", std::to_string(m_last_processing_time_ms_));
  stat.add("Max Processing Time (ms)", std::to_string(m_max_processing_time_ms_));
  if (m_processing_time_samples_ > 0) {
    double avg_processing_time = m_total_processing_time_ms_ / m_processing_time_samples_;
    stat.add("Avg Processing Time (ms)", std::to_string(avg_processing_time));
  } else {
    stat.add("Avg Processing Time (ms)", "N/A");
  }
  stat.add("Processing Time Samples", std::to_string(m_processing_time_samples_));
  
  // Watchdog metrics
  stat.add("Camera Frozen", m_camera_frozen_ ? "true" : "false");
  stat.add("Watchdog Timeout (sec)", std::to_string(watchdog_timeout_sec_));
  if (m_watchdog_initialized_) {
    auto now = std::chrono::steady_clock::now();
    double since_last = std::chrono::duration<double>(now - m_last_frame_time_).count();
    stat.add("Time Since Last Frame (sec)", std::to_string(since_last));
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
    auto now_diag = std::chrono::steady_clock::now();
    double diag_elapsed = m_diag_genicam_cache_valid_
        ? std::chrono::duration<double>(now_diag - m_last_diag_genicam_read_time_).count()
        : DIAG_GENICAM_READ_INTERVAL_SEC + 1.0;  // force first read

    if (diag_elapsed >= DIAG_GENICAM_READ_INTERVAL_SEC) {
      m_diag_genicam_cache_.clear();
      try {
        auto nodemap = m_pDevice->GetNodeMap();

        // Helper: attempt a read, cache on success, silently skip on failure
        auto try_read = [&](const char* diag_key, auto reader) {
          try { m_diag_genicam_cache_.emplace_back(diag_key, reader(nodemap)); } catch (...) {}
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
        m_diag_genicam_cache_.emplace_back("Camera Parameters", std::string("GenICam error: ") + e.what());
        m_diag_genicam_cache_valid_ = true;
        m_last_diag_genicam_read_time_ = now_diag;
      } catch (const std::exception& e) {
        m_diag_genicam_cache_.emplace_back("Camera Parameters", std::string("error: ") + e.what());
        m_diag_genicam_cache_valid_ = true;
        m_last_diag_genicam_read_time_ = now_diag;
      } catch (...) {
        m_diag_genicam_cache_.emplace_back("Camera Parameters", "unknown error reading camera parameters");
        m_diag_genicam_cache_valid_ = true;
        m_last_diag_genicam_read_time_ = now_diag;
      }
    }

    // Report cached values
    for (const auto& kv : m_diag_genicam_cache_) {
      stat.add(kv.first, kv.second);
    }
    if (m_diag_genicam_cache_valid_) {
      stat.add("Diag GenICam Age (sec)", std::to_string(
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - m_last_diag_genicam_read_time_).count()));
    }
  }
}
