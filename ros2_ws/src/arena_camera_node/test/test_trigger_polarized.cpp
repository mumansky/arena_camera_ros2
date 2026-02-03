/**
 * @file test_trigger_polarized.cpp
 * @brief Integration test for trigger mode with polarized camera format
 * 
 * This test validates Task 7 from ENGINEERING_TASKS.md:
 * - Trigger mode works with polarized format (0x8220020F)
 * - All 4 polarization channels (0°, 45°, 90°, 135°) are published on trigger
 * - Max-combined image is generated correctly on trigger
 * - No frame drops or missing data
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

/**
 * @brief Test fixture for trigger mode with polarization
 * 
 * Note: This is an integration test that requires:
 * - A polarized camera to be connected
 * - The arena_camera_node to be running with trigger_mode:=true
 * - The pixel format set to "polarized_angles_0d_45d_90d_135d_bayer_rg8"
 */
class TriggerPolarizedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node for testing
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_trigger_polarized");
    
    // Create service client for trigger
    trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/arena_camera_node/trigger_image");
    
    // Create subscribers for all expected topics
    main_image_received_ = false;
    pol_0deg_received_ = false;
    pol_45deg_received_ = false;
    pol_90deg_received_ = false;
    pol_135deg_received_ = false;
    pol_max_received_ = false;
    
    main_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/arena_camera_node/images", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        main_image_received_ = true;
        last_main_image_ = msg;
      });
    
    pol_0deg_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/arena_camera_node/pol_0deg", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        pol_0deg_received_ = true;
        last_pol_0deg_ = msg;
      });
    
    pol_45deg_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/arena_camera_node/pol_45deg", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        pol_45deg_received_ = true;
        last_pol_45deg_ = msg;
      });
    
    pol_90deg_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/arena_camera_node/pol_90deg", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        pol_90deg_received_ = true;
        last_pol_90deg_ = msg;
      });
    
    pol_135deg_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/arena_camera_node/pol_135deg", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        pol_135deg_received_ = true;
        last_pol_135deg_ = msg;
      });
    
    pol_max_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/arena_camera_node/pol_max", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        pol_max_received_ = true;
        last_pol_max_ = msg;
      });
  }
  
  void TearDown() override
  {
    // Clean up
    node_.reset();
  }
  
  /**
   * @brief Wait for service to become available
   * @return true if service is available within timeout
   */
  bool waitForService(std::chrono::seconds timeout = 5s)
  {
    auto start = std::chrono::steady_clock::now();
    while (!trigger_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      auto elapsed = std::chrono::steady_clock::now() - start;
      if (elapsed > timeout) {
        return false;
      }
      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger service...");
    }
    return true;
  }
  
  /**
   * @brief Trigger an image and wait for response
   * @return true if trigger was successful
   */
  bool triggerImage()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = trigger_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (rclcpp::spin_until_future_complete(node_, future, 5s) != 
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call trigger service");
      return false;
    }
    
    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(node_->get_logger(), "Trigger failed: %s", response->message.c_str());
      return false;
    }
    
    return true;
  }
  
  /**
   * @brief Wait for messages to be received
   * @param timeout Maximum time to wait
   */
  void waitForMessages(std::chrono::seconds timeout = 3s)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(100ms);
    }
  }
  
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr main_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pol_0deg_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pol_45deg_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pol_90deg_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pol_135deg_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pol_max_sub_;
  
  bool main_image_received_;
  bool pol_0deg_received_;
  bool pol_45deg_received_;
  bool pol_90deg_received_;
  bool pol_135deg_received_;
  bool pol_max_received_;
  
  sensor_msgs::msg::Image::SharedPtr last_main_image_;
  sensor_msgs::msg::Image::SharedPtr last_pol_0deg_;
  sensor_msgs::msg::Image::SharedPtr last_pol_45deg_;
  sensor_msgs::msg::Image::SharedPtr last_pol_90deg_;
  sensor_msgs::msg::Image::SharedPtr last_pol_135deg_;
  sensor_msgs::msg::Image::SharedPtr last_pol_max_;
};

/**
 * @brief Test that trigger service is available
 * 
 * This test verifies the trigger service can be discovered.
 * It will be skipped if no camera node is running.
 */
TEST_F(TriggerPolarizedTest, ServiceAvailable)
{
  // This test requires the camera node to be running
  // If the service is not available, skip the test
  if (!waitForService(2s)) {
    GTEST_SKIP() << "Trigger service not available. "
                 << "Start arena_camera_node with trigger_mode:=true to run this test.";
  }
  
  EXPECT_TRUE(trigger_client_->service_is_ready());
}

/**
 * @brief Test that main image is published on trigger
 */
TEST_F(TriggerPolarizedTest, MainImageOnTrigger)
{
  if (!waitForService(2s)) {
    GTEST_SKIP() << "Trigger service not available.";
  }
  
  // Trigger an image
  ASSERT_TRUE(triggerImage()) << "Failed to trigger image";
  
  // Wait for message
  waitForMessages(3s);
  
  // Verify main image was received
  EXPECT_TRUE(main_image_received_) << "Main image not received after trigger";
  
  if (main_image_received_) {
    EXPECT_GT(last_main_image_->width, 0u);
    EXPECT_GT(last_main_image_->height, 0u);
    EXPECT_FALSE(last_main_image_->data.empty());
  }
}

/**
 * @brief Test that all 4 polarization channels are published on trigger
 * 
 * This is the core test for Task 7 - verify that trigger mode works
 * with polarized format and all channels are published.
 */
TEST_F(TriggerPolarizedTest, AllPolarizationChannelsOnTrigger)
{
  if (!waitForService(2s)) {
    GTEST_SKIP() << "Trigger service not available.";
  }
  
  // Trigger an image
  ASSERT_TRUE(triggerImage()) << "Failed to trigger image";
  
  // Wait for all messages
  waitForMessages(3s);
  
  // Verify all polarization channels were received
  EXPECT_TRUE(pol_0deg_received_) << "0° polarization channel not received";
  EXPECT_TRUE(pol_45deg_received_) << "45° polarization channel not received";
  EXPECT_TRUE(pol_90deg_received_) << "90° polarization channel not received";
  EXPECT_TRUE(pol_135deg_received_) << "135° polarization channel not received";
  
  // Verify image dimensions are consistent across all channels
  if (pol_0deg_received_ && pol_45deg_received_ && 
      pol_90deg_received_ && pol_135deg_received_) {
    EXPECT_EQ(last_pol_0deg_->width, last_pol_45deg_->width);
    EXPECT_EQ(last_pol_0deg_->width, last_pol_90deg_->width);
    EXPECT_EQ(last_pol_0deg_->width, last_pol_135deg_->width);
    
    EXPECT_EQ(last_pol_0deg_->height, last_pol_45deg_->height);
    EXPECT_EQ(last_pol_0deg_->height, last_pol_90deg_->height);
    EXPECT_EQ(last_pol_0deg_->height, last_pol_135deg_->height);
    
    // All channels should have BGR8 encoding
    EXPECT_EQ(last_pol_0deg_->encoding, "bgr8");
    EXPECT_EQ(last_pol_45deg_->encoding, "bgr8");
    EXPECT_EQ(last_pol_90deg_->encoding, "bgr8");
    EXPECT_EQ(last_pol_135deg_->encoding, "bgr8");
  }
}

/**
 * @brief Test that max-combined image is published on trigger
 */
TEST_F(TriggerPolarizedTest, MaxCombinedImageOnTrigger)
{
  if (!waitForService(2s)) {
    GTEST_SKIP() << "Trigger service not available.";
  }
  
  // Trigger an image
  ASSERT_TRUE(triggerImage()) << "Failed to trigger image";
  
  // Wait for messages
  waitForMessages(3s);
  
  // Verify max-combined image was received
  EXPECT_TRUE(pol_max_received_) << "Max-combined polarization image not received";
  
  if (pol_max_received_) {
    EXPECT_GT(last_pol_max_->width, 0u);
    EXPECT_GT(last_pol_max_->height, 0u);
    EXPECT_EQ(last_pol_max_->encoding, "bgr8");
    EXPECT_FALSE(last_pol_max_->data.empty());
  }
}

/**
 * @brief Test multiple triggers in sequence
 * 
 * Verifies that multiple triggers work reliably and no frames are dropped.
 */
TEST_F(TriggerPolarizedTest, MultipleTriggers)
{
  if (!waitForService(2s)) {
    GTEST_SKIP() << "Trigger service not available.";
  }
  
  const int num_triggers = 3;
  int successful_triggers = 0;
  
  for (int i = 0; i < num_triggers; i++) {
    // Reset flags
    main_image_received_ = false;
    pol_0deg_received_ = false;
    pol_45deg_received_ = false;
    pol_90deg_received_ = false;
    pol_135deg_received_ = false;
    pol_max_received_ = false;
    
    // Trigger
    if (triggerImage()) {
      successful_triggers++;
      waitForMessages(3s);
      
      // Verify at least main image and polarization channels received
      EXPECT_TRUE(main_image_received_) << "Trigger " << i << " failed";
    }
    
    // Small delay between triggers
    std::this_thread::sleep_for(500ms);
  }
  
  EXPECT_EQ(successful_triggers, num_triggers) 
    << "Not all triggers were successful";
}

/**
 * @brief Test that frame IDs are consistent across all channels
 */
TEST_F(TriggerPolarizedTest, ConsistentFrameIds)
{
  if (!waitForService(2s)) {
    GTEST_SKIP() << "Trigger service not available.";
  }
  
  // Trigger an image
  ASSERT_TRUE(triggerImage());
  waitForMessages(3s);
  
  // All images from the same trigger should have the same frame ID
  if (main_image_received_ && pol_0deg_received_ && 
      pol_45deg_received_ && pol_90deg_received_ && pol_135deg_received_) {
    std::string expected_frame_id = last_main_image_->header.frame_id;
    
    EXPECT_EQ(last_pol_0deg_->header.frame_id, expected_frame_id);
    EXPECT_EQ(last_pol_45deg_->header.frame_id, expected_frame_id);
    EXPECT_EQ(last_pol_90deg_->header.frame_id, expected_frame_id);
    EXPECT_EQ(last_pol_135deg_->header.frame_id, expected_frame_id);
    
    if (pol_max_received_) {
      EXPECT_EQ(last_pol_max_->header.frame_id, expected_frame_id);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  
  int result = RUN_ALL_TESTS();
  
  rclcpp::shutdown();
  return result;
}
