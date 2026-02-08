#pragma once

#include <type_traits>
#include <yaml-cpp/yaml.h>

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <atomic>      // std::atomic for thread-safe flags (streaming state, backpressure)
#include <chrono>      // chrono_literals
#include <cstddef>     // size_t
#include <cstdint>     // int64_t
#include <cstdio>      // setvbuf, stdout, BUFSIZ
#include <exception>   // std::exception
#include <functional>  // std::bind, std::placeholders
#include <memory>      // std::shared_ptr, std::unique_ptr
#include <string>      // std::string

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <sensor_msgs/msg/compressed_image.hpp>  // compressed image
#include <std_srvs/srv/trigger.hpp>   // Trigger
#include <diagnostic_updater/diagnostic_updater.hpp>  // diagnostics

// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 private:
  // Forward declaration for image callback handler
  class ImageCallbackHandler;
  
 public:
  ArenaCameraNode() : Node("arena_camera_node"),
    m_images_published_(0),
    m_image_publish_errors_(0),
    m_device_connected_(false),
    m_is_streaming_(false)
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    load_config_file_();
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
    
    // Capture streaming state before setting to false.
    // Note: There's a small race window between this capture and the store below,
    // but any callback that starts in this window will see m_is_streaming_=false
    // before accessing m_pDevice, which is the critical safety property.
    bool was_streaming = m_is_streaming_.load();
    
    // Set streaming flag to false FIRST to signal any in-flight callbacks to exit early
    // This prevents race conditions where a callback tries to use m_pDevice during cleanup
    m_is_streaming_.store(false);
    
    // Deregister image callback before stopping stream
    if (m_image_callback_handler_ && m_pDevice) {
      try {
        m_pDevice->DeregisterImageCallback(m_image_callback_handler_.get());
        m_image_callback_handler_.reset();
        log_info("Image callback deregistered");
      } catch (const std::exception& e) {
        log_warn(std::string("Warning during callback deregistration: ") + e.what());
      }
    }
    
    // Cancel the wait for device timer
    if (m_wait_for_device_timer_callback_) {
      m_wait_for_device_timer_callback_->cancel();
      m_wait_for_device_timer_callback_.reset();
      log_info("Device timer cancelled");
    }
    
    // Stop streaming on device if it was streaming
    if (m_pDevice && was_streaming) {
      try {
        m_pDevice->StopStream();
        log_info("Camera stream stopped");
      } catch (const std::exception& e) {
        log_warn(std::string("Warning during stream stop: ") + e.what());
      } catch (...) {
        log_warn("Unknown exception during stream stop");
      }
    }
    
    // Destroy device before system (order matters)
    // Extract raw pointer, call cleanup, then release shared_ptr
    // The deleters are no-op so we handle cleanup explicitly here
    if (m_pDevice && m_pSystem) {
      try {
        Arena::IDevice* pDevice = m_pDevice.get();
        m_pSystem->DestroyDevice(pDevice);
        log_info("Device is destroyed");
      } catch (const std::exception& e) {
        log_warn(std::string("Warning during device cleanup: ") + e.what());
      } catch (...) {
        log_warn("Unknown exception during device cleanup");
      }
      m_pDevice.reset();  // Release shared_ptr (deleter is no-op)
    }
    
    // Close system
    if (m_pSystem) {
      try {
        Arena::ISystem* pSystem = m_pSystem.get();
        Arena::CloseSystem(pSystem);
        log_info("System is destroyed");
      } catch (const std::exception& e) {
        log_warn(std::string("Warning during system cleanup: ") + e.what());
      } catch (...) {
        log_warn("Unknown exception during system cleanup");
      }
      m_pSystem.reset();  // Release shared_ptr (deleter is no-op)
    }
    
    m_device_connected_ = false;
    log_info(std::string("Destroyed \"") + this->get_name() + "\" node");
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
  // Image callback handler class for ArenaSDK event-driven acquisition
  class ImageCallbackHandler : public Arena::IImageCallback
  {
  public:
    explicit ImageCallbackHandler(ArenaCameraNode* node) : m_node_(node) {}
    
    void OnImage(Arena::IImage* pImage) override
    {
      if (m_node_) {
        m_node_->handle_camera_image_(pImage);
      }
    }
    
  private:
    ArenaCameraNode* m_node_;
  };
  
  // Image handler called by ArenaSDK callback
  void handle_camera_image_(Arena::IImage* pImage);
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_pol_0deg_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_pol_0deg_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_pol_45deg_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_pol_45deg_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_pol_90deg_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_pol_90deg_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_pol_135deg_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_pol_135deg_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_pol_max_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_pol_max_compressed_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;
  
  // Image callback handler for event-driven acquisition
  std::unique_ptr<ImageCallbackHandler> m_image_callback_handler_;

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> m_diagnostic_updater_;
  uint64_t m_images_published_;
  uint64_t m_image_publish_errors_;
  bool m_device_connected_;
  
  // Streaming state for proper cleanup (atomic for thread safety between destructor and deleters)
  std::atomic<bool> m_is_streaming_;
  
  // FPS calculation
  std::chrono::steady_clock::time_point m_fps_last_time_;
  uint64_t m_fps_frame_count_;
  double m_calculated_fps_;

  // Backpressure monitoring (Task 4)
  std::atomic<bool> m_processing_image_{false};  // Flag to detect if still processing
  uint64_t m_backpressure_events_{0};            // Count of skipped frames due to backpressure
  double m_last_processing_time_ms_{0.0};        // Last frame processing time in ms
  double m_max_processing_time_ms_{0.0};         // Max processing time observed
  double m_total_processing_time_ms_{0.0};       // Sum of processing times for average calculation
  uint64_t m_processing_time_samples_{0};        // Number of processing time samples

  std::string serial_;
  bool is_passed_serial_;

  std::string topic_;
  bool is_passed_topic_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  std::string auto_gain_;
  bool is_passed_auto_gain_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  std::string auto_exposure_;
  bool is_passed_auto_exposure_;

  bool short_exposure_enable_;
  bool is_passed_short_exposure_enable_;

  std::string exposure_auto_algorithm_;
  bool is_passed_exposure_auto_algorithm_;

  int64_t target_brightness_;
  bool is_passed_target_brightness_;

  double exposure_auto_damping_;
  bool is_passed_exposure_auto_damping_;

  std::string exposure_auto_limit_auto_;
  bool is_passed_exposure_auto_limit_auto_;

  double exposure_auto_upper_limit_;
  bool is_passed_exposure_auto_upper_limit_;

  double exposure_auto_lower_limit_;
  bool is_passed_exposure_auto_lower_limit_;

  bool acquisition_frame_rate_enable_;
  bool is_passed_acquisition_frame_rate_enable_;

  double acquisition_frame_rate_;
  bool is_passed_acquisition_frame_rate_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  bool publish_raw_;
  bool publish_compressed_;
  
  int jpeg_quality_;  // JPEG compression quality (1-100, default 80)
  
  // Watchdog settings (Task 20)
  double watchdog_timeout_sec_;  // Seconds without a new frame before declaring camera frozen (default 5.0)
  std::chrono::steady_clock::time_point m_last_frame_time_;  // Timestamp of last successfully received frame
  bool m_watchdog_initialized_{false};  // Whether we've received at least one frame
  bool m_camera_frozen_{false};  // Whether the camera is currently detected as frozen

  YAML::Node m_config_params_;

  void load_config_file_();
  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_frame_rate_();
  void set_nodes_trigger_mode_();
  void set_nodes_test_pattern_image_();
  void publish_images_();  // Legacy blocking implementation

  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);

  // Diagnostics
  void produce_diagnostics_(diagnostic_updater::DiagnosticStatusWrapper& stat);
};
