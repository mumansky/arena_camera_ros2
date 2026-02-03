#pragma once

#include <yaml-cpp/yaml.h>

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders

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
 public:
  ArenaCameraNode() : Node("arena_camera_node"),
    m_images_published_(0),
    m_image_publish_errors_(0),
    m_device_connected_(false),
    m_consecutive_failures_(0),
    m_consecutive_failure_threshold_(50)
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
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
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
  rclcpp::TimerBase::SharedPtr m_image_acquisition_timer_;  // Timer for non-blocking image acquisition
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> m_diagnostic_updater_;
  uint64_t m_images_published_;
  uint64_t m_image_publish_errors_;
  bool m_device_connected_;
  
  // FPS calculation
  std::chrono::steady_clock::time_point m_fps_last_time_;
  uint64_t m_fps_frame_count_;
  double m_calculated_fps_;
  
  // Camera disconnect detection
  uint32_t m_consecutive_failures_;
  uint32_t m_consecutive_failure_threshold_;
  std::chrono::steady_clock::time_point m_last_successful_image_time_;

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
  void publish_one_image_();  // Non-blocking timer callback for image acquisition
  void check_and_handle_disconnect_();  // Helper for camera disconnect detection

  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);

  // Diagnostics
  void produce_diagnostics_(diagnostic_updater::DiagnosticStatusWrapper& stat);
};
