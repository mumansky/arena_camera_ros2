#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
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
    return "Unknown (0x" + std::to_string(format) + ")";
  }
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
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "topic";
    std::string topic_default = (m_config_params_ && m_config_params_["topic"]) ?
                      m_config_params_["topic"].as<std::string>() : "/arena_camera_node/images";
    topic_ = this->declare_parameter("topic", topic_default);
    is_passed_topic_ = topic_ != "/arena_camera_node/images";

    nextParameterToDeclare = "auto_exposure";
    std::string auto_exposure_default = (m_config_params_ && m_config_params_["auto_exposure"]) ?
                      m_config_params_["auto_exposure"].as<std::string>() : "";
    auto_exposure_ = this->declare_parameter("auto_exposure", auto_exposure_default);
    is_passed_auto_exposure_ = auto_exposure_ != "";

    nextParameterToDeclare = "target_brightness";
    int64_t target_brightness_default = (m_config_params_ && m_config_params_["target_brightness"]) ?
                                        m_config_params_["target_brightness"].as<int64_t>() : -1;
    target_brightness_ = this->declare_parameter("target_brightness", target_brightness_default);
    is_passed_target_brightness_ = target_brightness_ >= 0;

    // Set defaults for other internal parameters that are used but not exposed as ROS parameters
    serial_ = "";
    is_passed_serial_ = false;
    pixelformat_ros_ = (m_config_params_ && m_config_params_["pixelformat"]) ?
                       m_config_params_["pixelformat"].as<std::string>() : "";
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";
    width_ = 0;
    is_passed_width = false;
    height_ = 0;
    is_passed_height = false;
    gain_ = -1.0;
    is_passed_gain_ = false;
    auto_gain_ = "";
    is_passed_auto_gain_ = false;
    exposure_time_ = -1.0;
    is_passed_exposure_time_ = false;
    short_exposure_enable_ = false;
    is_passed_short_exposure_enable_ = false;
    exposure_auto_algorithm_ = "";
    is_passed_exposure_auto_algorithm_ = false;
    exposure_auto_damping_ = -1.0;
    is_passed_exposure_auto_damping_ = false;
    exposure_auto_limit_auto_ = "";
    is_passed_exposure_auto_limit_auto_ = false;
    exposure_auto_upper_limit_ = -1.0;
    is_passed_exposure_auto_upper_limit_ = false;
    exposure_auto_lower_limit_ = -1.0;
    is_passed_exposure_auto_lower_limit_ = false;
    acquisition_frame_rate_enable_ = false;
    is_passed_acquisition_frame_rate_enable_ = false;
    acquisition_frame_rate_ = -1.0;
    is_passed_acquisition_frame_rate_ = false;
    trigger_mode_activated_ = (m_config_params_ && m_config_params_["trigger_mode"]) ?
                              m_config_params_["trigger_mode"].as<bool>() : false;
    
    // Read other config parameters
    auto_gain_ = (m_config_params_ && m_config_params_["auto_gain"]) ?
                 m_config_params_["auto_gain"].as<std::string>() : "";
    is_passed_auto_gain_ = auto_gain_ != "";
    
    short_exposure_enable_ = (m_config_params_ && m_config_params_["short_exposure_enable"]) ?
                             m_config_params_["short_exposure_enable"].as<bool>() : false;
    is_passed_short_exposure_enable_ = short_exposure_enable_;
    
    pub_qos_history_ = "";
    is_passed_pub_qos_history_ = false;
    pub_qos_history_depth_ = 0;
    is_passed_pub_qos_history_depth_ = false;
    pub_qos_reliability_ = "";
    is_passed_pub_qos_reliability_ = false;
    
    // Read publish flags from config file
    publish_raw_ = (m_config_params_ && m_config_params_["publish_raw"]) ?
                   m_config_params_["publish_raw"].as<bool>() : true;
    publish_compressed_ = (m_config_params_ && m_config_params_["publish_compressed"]) ?
                          m_config_params_["publish_compressed"].as<bool>() : false;
<<<<<<< HEAD
    
    // Read compression settings from config file
    compression_format_ = (m_config_params_ && m_config_params_["compression_format"]) ?
                          m_config_params_["compression_format"].as<std::string>() : "jpeg";
    compression_quality_ = (m_config_params_ && m_config_params_["compression_quality"]) ?
                           m_config_params_["compression_quality"].as<int>() : 90;
    
    // Validate compression quality
    if (compression_quality_ < 1 || compression_quality_ > 100) {
      log_warn("compression_quality must be between 1 and 100, using default 90");
      compression_quality_ = 90;
    }
    
    // Validate compression format
    if (compression_format_ != "jpeg" && compression_format_ != "png") {
      log_warn("compression_format must be 'jpeg' or 'png', using default 'jpeg'");
      compression_format_ = "jpeg";
    }
    
    log_info("Compression settings: format=" + compression_format_ + 
             ", quality=" + std::to_string(compression_quality_));
=======
    jpeg_quality_ = (m_config_params_ && m_config_params_["jpeg_quality"]) ?
                    m_config_params_["jpeg_quality"].as<int>() : 80;
>>>>>>> 93e07e7e68dac14bd0d529ff359de29749f1bf52

  } catch (rclcpp::ParameterTypeException& e) {
    log_err("Parameter exception for: " + nextParameterToDeclare + " - " + std::string(e.what()));
    throw;
  } catch (std::exception& e) {
    log_err("General exception in parse_parameters_(): " + std::string(e.what()));
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
        this->get_parameter("topic").as_string(), pub_qos_);
  }
  
  // Only create main compressed publisher for non-polarized formats
  // Polarized format: "polarized_angles_0d_45d_90d_135d_bayer_rg8"
  bool is_polarized = (pixelformat_ros_ == "polarized_angles_0d_45d_90d_135d_bayer_rg8");
  
  if (publish_compressed_ && !is_polarized) {
    m_pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->get_parameter("topic").as_string() + "/compressed", pub_qos_);
  }
  
  // Create publishers for all polarization channels (if either raw or compressed is enabled)
  if (publish_raw_ || publish_compressed_) {
    m_pub_pol_0deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("topic").as_string() + "/pol_0deg", pub_qos_);
    m_pub_pol_45deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("topic").as_string() + "/pol_45deg", pub_qos_);
    m_pub_pol_90deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("topic").as_string() + "/pol_90deg", pub_qos_);
    m_pub_pol_135deg_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("topic").as_string() + "/pol_135deg", pub_qos_);
  }
  
  // Create compressed publishers for all polarization channels (only if compressed enabled)
  if (publish_compressed_) {
    m_pub_pol_0deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->get_parameter("topic").as_string() + "/pol_0deg/compressed", pub_qos_);
    m_pub_pol_45deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->get_parameter("topic").as_string() + "/pol_45deg/compressed", pub_qos_);
    m_pub_pol_90deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->get_parameter("topic").as_string() + "/pol_90deg/compressed", pub_qos_);
    m_pub_pol_135deg_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->get_parameter("topic").as_string() + "/pol_135deg/compressed", pub_qos_);
  }
  
  // Create publishers for max-combined polarization image
  if (publish_raw_) {
    m_pub_pol_max_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->get_parameter("topic").as_string() + "/pol_max", pub_qos_);
  }
  if (publish_compressed_) {
    m_pub_pol_max_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->get_parameter("topic").as_string() + "/pol_max/compressed", pub_qos_);
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
  
  try {
    m_pDevice->StartStream();
    m_is_streaming_.store(true);
    log_debug("StartStream() completed");
  } catch (GenICam::GenericException& e) {
    log_err(std::string("Failed to start stream: ") + e.what());
    throw;
  } catch (std::exception& e) {
    log_err(std::string("Failed to start stream: ") + e.what());
    throw;
  }
  
  m_device_connected_ = true;

  if (!trigger_mode_activated_) {
    log_info("Streaming started with event-driven callbacks - publishing images to " + topic_);
    // Register callback handler for event-driven image acquisition
    m_image_callback_handler_ = std::make_unique<ImageCallbackHandler>(this);
    m_pDevice->RegisterImageCallback(m_image_callback_handler_.get());
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
          compressed_msg->format = compression_format_;
          
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
          
<<<<<<< HEAD
          if (compression_format_ == "jpeg") {
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compression_quality_};
            cv::imencode(".jpg", img_mat, compressed_msg->data, params);
          } else if (compression_format_ == "png") {
            // Map compression_quality (1-100) to PNG compression level (0-9)
            // Higher quality = lower compression level (faster encoding)
            // 100 -> 0 (no compression, fastest), 1 -> 9 (max compression, slowest)
            int png_level = 9 - ((compression_quality_ - 1) * 9 / 99);
            std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, png_level};
            cv::imencode(".png", img_mat, compressed_msg->data, params);
          }
=======
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
          cv::imencode(".jpg", img_mat, compressed_msg->data, params);
          
          // converted_image is automatically destroyed by RAII when going out of scope
>>>>>>> 93e07e7e68dac14bd0d529ff359de29749f1bf52
          
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
                  compressed_msg->format = compression_format_;
                  
                  // Convert to cv::Mat and compress
                  cv::Mat bgr_mat(bgr_image->GetHeight(), bgr_image->GetWidth(), CV_8UC3, 
                                 const_cast<void*>(static_cast<const void*>(bgr_image->GetData())));
                  
                  if (compression_format_ == "jpeg") {
                    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compression_quality_};
                    cv::imencode(".jpg", bgr_mat, compressed_msg->data, params);
                  } else if (compression_format_ == "png") {
                    // Map compression_quality (1-100) to PNG compression level (0-9)
                    int png_level = 9 - ((compression_quality_ - 1) * 9 / 99);
                    std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, png_level};
                    cv::imencode(".png", bgr_mat, compressed_msg->data, params);
                  }
                  
                  (*info.compressed_pub)->publish(std::move(compressed_msg));
                }
                
                // bgr_image is automatically destroyed by RAII when going out of scope
              }
              
              // Create max-combined image from all 4 polarization channels
              if ((publish_raw_ && m_pub_pol_max_) || (publish_compressed_ && m_pub_pol_max_compressed_)) {
                // Convert all channels to BGR8 first - use RAII wrapper for exception safety
                arena_camera::ArenaImageVector bgr_channels;
                for (size_t i = 0; i < 4; i++) {
                  bgr_channels.push_back(Arena::ImageFactory::Convert(channels[i], PFNC_BGR8));
                }
                
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
                  compressed_msg->format = compression_format_;
                  
                  // Compress (max_mat already computed above)
                  if (compression_format_ == "jpeg") {
                    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compression_quality_};
                    cv::imencode(".jpg", max_mat, compressed_msg->data, params);
                  } else if (compression_format_ == "png") {
                    // Map compression_quality (1-100) to PNG compression level (0-9)
                    int png_level = 9 - ((compression_quality_ - 1) * 9 / 99);
                    std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, png_level};
                    cv::imencode(".png", max_mat, compressed_msg->data, params);
                  }
                  
                  m_pub_pol_max_compressed_->publish(std::move(compressed_msg));
                }
                
                // bgr_channels automatically cleaned up by RAII when going out of scope
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

void ArenaCameraNode::handle_camera_image_(Arena::IImage* pImage)
{
  // Event-driven image handler called by ArenaSDK when new image arrives
  
  // Check if device is still valid and streaming (protects against race with destructor)
  if (!m_pDevice || !m_is_streaming_.load()) {
    log_debug("Skipping image acquisition - device shutting down");
    return;
  }
  
  // Backpressure detection: Skip this callback if still processing previous image
  // This prevents queue buildup when image processing takes longer than frame arrival
  bool expected = false;
  if (!m_processing_image_.compare_exchange_strong(expected, true)) {
    // Still processing previous image - skip this frame
    m_backpressure_events_++;
    if (m_backpressure_events_ % 100 == 1) {
      // Log on 1st event and every 100th thereafter (1, 101, 201, ...) to catch initial issues
      log_warn(std::string("Backpressure detected: skipping frame (total skipped: ") + 
               std::to_string(m_backpressure_events_) + 
               ", last processing time: " + std::to_string(m_last_processing_time_ms_) + "ms)");
    }
    return;
  }
  
  // RAII scope guard to automatically reset processing flag on function exit
  // This ensures the flag is always reset regardless of how we exit (return, exception, etc.)
  struct ProcessingGuard {
    std::atomic<bool>& flag;
    explicit ProcessingGuard(std::atomic<bool>& f) : flag(f) {}
    ~ProcessingGuard() { flag.store(false); }
  } processing_guard(m_processing_image_);
  
  // Measure processing time
  auto processing_start = std::chrono::steady_clock::now();
  
  try {
    // Image is provided by callback - no GetImage() call needed
    if (!pImage) {
      return;
    }
    
    // Log pixel format on first image (for debugging/verification)
    static bool format_logged = false;
    if (!format_logged) {
      uint64_t pixel_format = pImage->GetPixelFormat();
      std::string format_info = "Camera pixel format detected: " + get_pixel_format_name(pixel_format);
      if (is_polarized_format(pixel_format)) {
        format_info += " (polarized camera - will extract 4 channels: 0°, 45°, 90°, 135°)";
      } else {
        format_info += " (standard camera - will convert to BGR8 for publishing)";
      }
      log_info(format_info);
      format_logged = true;
    }
    
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
      if (!is_polarized_format(pixel_format)) {
        auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        compressed_msg->header.stamp.sec = static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
        compressed_msg->header.stamp.nanosec = static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
        compressed_msg->header.frame_id = std::to_string(pImage->GetFrameId());
        compressed_msg->format = compression_format_;
        
        // Use RAII for converted image to ensure cleanup on exception
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
        if (image_to_compress->GetBitsPerPixel() == 8) {
          cvType = CV_8UC1;
        }
        
        cv::Mat img_mat(image_to_compress->GetHeight(), 
                       image_to_compress->GetWidth(), 
                       cvType,
                       const_cast<void*>(static_cast<const void*>(image_to_compress->GetData())));
        
<<<<<<< HEAD
        if (compression_format_ == "jpeg") {
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compression_quality_};
          cv::imencode(".jpg", img_mat, compressed_msg->data, params);
        } else if (compression_format_ == "png") {
          // Map compression_quality (1-100) to PNG compression level (0-9)
          int png_level = 9 - ((compression_quality_ - 1) * 9 / 99);
          std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, png_level};
          cv::imencode(".png", img_mat, compressed_msg->data, params);
        }
=======
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        cv::imencode(".jpg", img_mat, compressed_msg->data, params);
        
        // converted_image is automatically destroyed by RAII when going out of scope
>>>>>>> 93e07e7e68dac14bd0d529ff359de29749f1bf52
        
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
      if (elapsed >= 1000) {
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
      if (is_polarized_format(pixel_format)) {
          // Use RAII wrapper for exception safety
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
            
            // Process each channel - use RAII for converted BGR images
            for (const auto& info : channel_infos) {
              arena_camera::ArenaImagePtr bgr_image(
                  Arena::ImageFactory::Convert(channels[info.index], PFNC_BGR8));
              
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
                compressed_msg->format = compression_format_;
                
                cv::Mat bgr_mat(bgr_image->GetHeight(), bgr_image->GetWidth(), CV_8UC3, 
                               const_cast<void*>(static_cast<const void*>(bgr_image->GetData())));
<<<<<<< HEAD
                
                if (compression_format_ == "jpeg") {
                  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compression_quality_};
                  cv::imencode(".jpg", bgr_mat, compressed_msg->data, params);
                } else if (compression_format_ == "png") {
                  // Map compression_quality (1-100) to PNG compression level (0-9)
                  int png_level = 9 - ((compression_quality_ - 1) * 9 / 99);
                  std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, png_level};
                  cv::imencode(".png", bgr_mat, compressed_msg->data, params);
                }
=======
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                cv::imencode(".jpg", bgr_mat, compressed_msg->data, params);
>>>>>>> 93e07e7e68dac14bd0d529ff359de29749f1bf52
                
                (*info.compressed_pub)->publish(std::move(compressed_msg));
              }
              
              // bgr_image is automatically destroyed by RAII when going out of scope
            }
            
            // Create max-combined image
            if ((publish_raw_ && m_pub_pol_max_) || (publish_compressed_ && m_pub_pol_max_compressed_)) {
              // Use RAII wrapper for exception safety
              arena_camera::ArenaImageVector bgr_channels;
              for (size_t i = 0; i < 4; i++) {
                bgr_channels.push_back(Arena::ImageFactory::Convert(channels[i], PFNC_BGR8));
              }
              
              size_t height = bgr_channels[0]->GetHeight();
              size_t width = bgr_channels[0]->GetWidth();
              
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
                compressed_msg->format = compression_format_;
                
<<<<<<< HEAD
                if (compression_format_ == "jpeg") {
                  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, compression_quality_};
                  cv::imencode(".jpg", max_combined, compressed_msg->data, params);
                } else if (compression_format_ == "png") {
                  // Map compression_quality (1-100) to PNG compression level (0-9)
                  int png_level = 9 - ((compression_quality_ - 1) * 9 / 99);
                  std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, png_level};
                  cv::imencode(".png", max_combined, compressed_msg->data, params);
                }
=======
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                cv::imencode(".jpg", max_combined, compressed_msg->data, params);
>>>>>>> 93e07e7e68dac14bd0d529ff359de29749f1bf52
                
                m_pub_pol_max_compressed_->publish(std::move(compressed_msg));
              }
              
              // bgr_channels automatically cleaned up by RAII when going out of scope
            }
            
            // channels automatically cleaned up by RAII when going out of scope
          }
      }
    } catch (std::exception& e) {
      log_warn(std::string("Error extracting polarization channels: ") + e.what());
    }
    
    // Note: With callback-based acquisition, ArenaSDK manages the image lifecycle
    // No manual RequeueBuffer needed - image is automatically returned when callback exits
    
    // Calculate and store processing time
    auto processing_end = std::chrono::steady_clock::now();
    m_last_processing_time_ms_ = std::chrono::duration<double, std::milli>(
        processing_end - processing_start).count();
    
    // Update max processing time
    if (m_last_processing_time_ms_ > m_max_processing_time_ms_) {
      m_max_processing_time_ms_ = m_last_processing_time_ms_;
    }
    
    // Update running average
    m_total_processing_time_ms_ += m_last_processing_time_ms_;
    m_processing_time_samples_++;

  } catch (std::exception& e) {
    m_image_publish_errors_++;
    log_err(std::string("Exception occurred while publishing an image: ") + e.what());
    // Note: With callbacks, image is automatically returned by ArenaSDK when callback exits
    // processing_guard RAII will automatically reset m_processing_image_ flag
    return;
  }
  // Note: processing_guard RAII will automatically reset m_processing_image_ flag on exit
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
    Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tStream configuration warning: ") + e.what());
  } catch (std::exception& e) {
    log_warn(std::string("\tStream configuration warning: ") + e.what());
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

  // Set advanced exposure parameters first (before enabling auto exposure)
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

  // Set exposure limit control
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

  if (is_passed_auto_exposure_) {
    try {
      // Verify the node is writable and the value is valid
      GenApi::CEnumerationPtr pExposureAuto = nodemap->GetNode("ExposureAuto");
      if (pExposureAuto && GenApi::IsWritable(pExposureAuto)) {
        // Check if the requested value is available
        if (pExposureAuto->GetEntryByName(auto_exposure_.c_str())) {
          Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", auto_exposure_.c_str());
          log_info(std::string("\tExposureAuto set to ") + auto_exposure_);
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
        log_warn("\tExposureAuto node not writable");
        return;
      }

      if (auto_exposure_ != "Off") {
        return;  // auto exposure enabled; skip manual exposure
      }
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tFailed to set ExposureAuto: ") + e.what());
      return;
    }
  }

  if (is_passed_exposure_time_) {
    if (!is_passed_auto_exposure_) {
      try {
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
  if (m_device_connected_) {
    if (m_image_publish_errors_ > 0 || m_backpressure_events_ > 0) {
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

  if (m_device_connected_) {
    stat.add("Serial", serial_.empty() ? "first discovered" : serial_);
    stat.add("Width", std::to_string(width_));
    stat.add("Height", std::to_string(height_));
    stat.add("Pixel Format", pixelformat_ros_);
    
    // Add camera parameter values
    try {
      auto nodemap = m_pDevice->GetNodeMap();
      
      // Frame rate settings
      bool frame_rate_enabled = Arena::GetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable");
      stat.add("AcquisitionFrameRateEnable", frame_rate_enabled ? "true" : "false");
      
      if (frame_rate_enabled) {
        double current_frame_rate = Arena::GetNodeValue<double>(nodemap, "AcquisitionFrameRate");
        stat.add("AcquisitionFrameRate (FPS)", std::to_string(current_frame_rate));
      } else {
        stat.add("AcquisitionFrameRate (FPS)", "disabled (max)");
      }
      
      // Gain values
      double gain = Arena::GetNodeValue<double>(nodemap, "Gain");
      int64_t gain_raw = Arena::GetNodeValue<int64_t>(nodemap, "GainRaw");
      stat.add("Gain (dB)", std::to_string(gain));
      stat.add("GainRaw", std::to_string(gain_raw));
      
      GenICam::gcstring gain_auto = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "GainAuto");
      stat.add("GainAuto", std::string(gain_auto.c_str()));
      
      // Exposure values
      double exposure_time = Arena::GetNodeValue<double>(nodemap, "ExposureTime");
      int64_t exposure_time_raw = Arena::GetNodeValue<int64_t>(nodemap, "ExposureTimeRaw");
      stat.add("ExposureTime (us)", std::to_string(exposure_time));
      stat.add("ExposureTimeRaw", std::to_string(exposure_time_raw));
      
      GenICam::gcstring exposure_auto = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto");
      stat.add("ExposureAuto", std::string(exposure_auto.c_str()));
      
      // Auto exposure limits
      try {
        double exposure_auto_upper = Arena::GetNodeValue<double>(nodemap, "ExposureAutoUpperLimit");
        stat.add("ExposureAutoUpperLimit (us)", std::to_string(exposure_auto_upper));
      } catch (...) {
        // If not available, skip
      }
      
      try {
        double exposure_auto_lower = Arena::GetNodeValue<double>(nodemap, "ExposureAutoLowerLimit");
        stat.add("ExposureAutoLowerLimit (us)", std::to_string(exposure_auto_lower));
      } catch (...) {
        // If not available, skip
      }
      
      try {
        GenICam::gcstring exposure_auto_limit_auto = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "ExposureAutoLimitAuto");
        stat.add("ExposureAutoLimitAuto", std::string(exposure_auto_limit_auto.c_str()));
      } catch (...) {
        // If not available, skip
      }
      
      // Auto exposure algorithm and settings
      try {
        int64_t target_brightness = Arena::GetNodeValue<int64_t>(nodemap, "TargetBrightness");
        stat.add("TargetBrightness", std::to_string(target_brightness));
      } catch (...) {
        // If not available, skip
      }
      
      try {
        GenICam::gcstring exposure_auto_algorithm = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "ExposureAutoAlgorithm");
        stat.add("ExposureAutoAlgorithm", std::string(exposure_auto_algorithm.c_str()));
      } catch (...) {
        // If not available, skip
      }
      
      try {
        double exposure_auto_damping = Arena::GetNodeValue<double>(nodemap, "ExposureAutoDamping");
        stat.add("ExposureAutoDamping", std::to_string(exposure_auto_damping));
      } catch (...) {
        // If not available, skip
      }
      
      // Image statistics that auto exposure uses
      try {
        int64_t calculated_mean = Arena::GetNodeValue<int64_t>(nodemap, "CalculatedMean");
        stat.add("CalculatedMean", std::to_string(calculated_mean));
      } catch (...) {
        // If not available, skip
      }
      
      try {
        int64_t calculated_median = Arena::GetNodeValue<int64_t>(nodemap, "CalculatedMedian");
        stat.add("CalculatedMedian", std::to_string(calculated_median));
      } catch (...) {
        // If not available, skip
      }
      
      // Short exposure setting we added
      try {
        bool short_exposure_enable = Arena::GetNodeValue<bool>(nodemap, "ShortExposureEnable");
        stat.add("ShortExposureEnable", short_exposure_enable ? "true" : "false");
      } catch (...) {
        // If not available, skip
      }
      
      // Device power
      try {
        double device_power = Arena::GetNodeValue<double>(nodemap, "DevicePower");
        stat.add("DevicePower (W)", std::to_string(device_power));
      } catch (...) {
        // If not available, skip
      }
      
      // Device uptime
      try {
        int64_t device_uptime = Arena::GetNodeValue<int64_t>(nodemap, "DeviceUpTime");
        stat.add("DeviceUpTime (ms)", std::to_string(device_uptime));
      } catch (...) {
        // If not available, skip
      }
      
      // Link uptime
      try {
        int64_t link_uptime = Arena::GetNodeValue<int64_t>(nodemap, "LinkUpTime");
        stat.add("LinkUpTime (ms)", std::to_string(link_uptime));
      } catch (...) {
        // If not available, skip
      }
      
      // Device temperature
      try {
        double device_temperature = Arena::GetNodeValue<double>(nodemap, "DeviceTemperature");
        stat.add("DeviceTemperature (°C)", std::to_string(device_temperature));
      } catch (...) {
        // If not available, skip
      }
      
    } catch (const std::exception& e) {
      stat.add("Camera Parameters", std::string("error: ") + e.what());
    }
  }
}
