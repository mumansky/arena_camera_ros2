/**
 * @file test_pixelformat_translation.cpp
 * @brief Unit tests for pixel format translation between ROS2 and PFNC formats
 */

#include <gtest/gtest.h>
#include <string>
#include <sensor_msgs/image_encodings.hpp>

// Include the translation header
#include "rclcpp_adapter/pixelformat_translation.h"

/**
 * @brief Test that standard ROS2 pixel formats map to correct PFNC formats
 */
TEST(PixelFormatTranslationTest, ROS2ToPFNC_StandardFormats)
{
  // Test RGB formats
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::RGB8], "RGB8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::RGBA8], "RGBa8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::RGB16], "RGB16");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::RGBA16], "RGBa16");

  // Test BGR formats
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BGR8], "BGR8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BGRA8], "BGRa8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BGR16], "BGR16");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BGRA16], "BGRa16");

  // Test Mono formats
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::MONO8], "Mono8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::MONO16], "Mono16");
}

/**
 * @brief Test that Bayer pixel formats map correctly
 */
TEST(PixelFormatTranslationTest, ROS2ToPFNC_BayerFormats)
{
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_RGGB8], "BayerRG8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_BGGR8], "BayerBG8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_GBRG8], "BayerGB8");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_GRBG8], "BayerGR8");

  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_RGGB16], "BayerRG16");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_BGGR16], "BayerBG16");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_GBRG16], "BayerGB16");
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::BAYER_GRBG16], "BayerGR16");
}

/**
 * @brief Test polarization format translation
 */
TEST(PixelFormatTranslationTest, ROS2ToPFNC_PolarizationFormat)
{
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC["polarized_angles_0d_45d_90d_135d_bayer_rg8"],
            "PolarizedAngles_0d_45d_90d_135d_BayerRG8");
}

/**
 * @brief Test YUV format translation
 */
TEST(PixelFormatTranslationTest, ROS2ToPFNC_YUVFormat)
{
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC[sensor_msgs::image_encodings::YUV422], "YUV422_8");
}

/**
 * @brief Test reverse mapping: PFNC to ROS2 standard formats
 */
TEST(PixelFormatTranslationTest, PFNCToROS2_StandardFormats)
{
  // Test RGB formats
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["RGB8"], sensor_msgs::image_encodings::RGB8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["RGBa8"], sensor_msgs::image_encodings::RGBA8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["RGB16"], sensor_msgs::image_encodings::RGB16);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["RGBa16"], sensor_msgs::image_encodings::RGBA16);

  // Test BGR formats
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BGR8"], sensor_msgs::image_encodings::BGR8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BGRa8"], sensor_msgs::image_encodings::BGRA8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BGR16"], sensor_msgs::image_encodings::BGR16);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BGRa16"], sensor_msgs::image_encodings::BGRA16);

  // Test Mono formats
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["Mono8"], sensor_msgs::image_encodings::MONO8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["Mono16"], sensor_msgs::image_encodings::MONO16);
}

/**
 * @brief Test reverse mapping: PFNC to ROS2 Bayer formats
 */
TEST(PixelFormatTranslationTest, PFNCToROS2_BayerFormats)
{
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerRG8"], sensor_msgs::image_encodings::BAYER_RGGB8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerBG8"], sensor_msgs::image_encodings::BAYER_BGGR8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerGB8"], sensor_msgs::image_encodings::BAYER_GBRG8);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerGR8"], sensor_msgs::image_encodings::BAYER_GRBG8);

  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerRG16"], sensor_msgs::image_encodings::BAYER_RGGB16);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerBG16"], sensor_msgs::image_encodings::BAYER_BGGR16);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerGB16"], sensor_msgs::image_encodings::BAYER_GBRG16);
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["BayerGR16"], sensor_msgs::image_encodings::BAYER_GRBG16);
}

/**
 * @brief Test reverse mapping for polarization format
 */
TEST(PixelFormatTranslationTest, PFNCToROS2_PolarizationFormat)
{
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT["PolarizedAngles_0d_45d_90d_135d_BayerRG8"],
            "polarized_angles_0d_45d_90d_135d_bayer_rg8");
}

/**
 * @brief Test that bidirectional mapping is consistent
 */
TEST(PixelFormatTranslationTest, BidirectionalConsistency)
{
  // For each ROS2 format, verify round-trip conversion
  for (const auto& pair : K_ROS2_PIXELFORMAT_TO_PFNC) {
    const std::string& ros2_format = pair.first;
    const std::string& pfnc_format = pair.second;
    
    // Check if the PFNC format maps back to the same ROS2 format
    auto it = K_PFNC_TO_ROS2_PIXELFORMAT.find(pfnc_format);
    ASSERT_NE(it, K_PFNC_TO_ROS2_PIXELFORMAT.end()) 
        << "PFNC format '" << pfnc_format << "' not found in reverse map";
    EXPECT_EQ(it->second, ros2_format) 
        << "Round-trip failed for ROS2 format: " << ros2_format;
  }
}

/**
 * @brief Test that unknown formats are not in the map
 */
TEST(PixelFormatTranslationTest, UnknownFormatsNotPresent)
{
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC.find("unknown_format"), 
            K_ROS2_PIXELFORMAT_TO_PFNC.end());
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC.find(""), 
            K_ROS2_PIXELFORMAT_TO_PFNC.end());
  EXPECT_EQ(K_PFNC_TO_ROS2_PIXELFORMAT.find("UnknownPFNCFormat"), 
            K_PFNC_TO_ROS2_PIXELFORMAT.end());
}

/**
 * @brief Test map sizes match (same number of entries in both directions)
 */
TEST(PixelFormatTranslationTest, MapSizesMatch)
{
  EXPECT_EQ(K_ROS2_PIXELFORMAT_TO_PFNC.size(), K_PFNC_TO_ROS2_PIXELFORMAT.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
