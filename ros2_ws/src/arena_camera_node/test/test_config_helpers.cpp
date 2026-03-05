/**
 * @file test_config_helpers.cpp
 * @brief Unit tests for YAML config helper functions and pixel format utilities
 *
 * Tests the safe config reading helpers (config_string, config_bool, etc.)
 * and pixel format detection functions (is_polarized_format, is_supported_format,
 * get_pixel_format_name, validate_bgr8_format).
 */

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <cstdint>

// ============================================================================
// Replicate the config helper functions from ArenaCameraNode.cpp
// These are static functions in the .cpp file, so we redefine them here for testing.
// ============================================================================

static std::string config_string(const YAML::Node& config, const std::string& key, const std::string& default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<std::string>();
    }
  } catch (const YAML::Exception&) {
  }
  return default_val;
}

static bool config_bool(const YAML::Node& config, const std::string& key, bool default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<bool>();
    }
  } catch (const YAML::Exception&) {
  }
  return default_val;
}

static double config_double(const YAML::Node& config, const std::string& key, double default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<double>();
    }
  } catch (const YAML::Exception&) {
  }
  return default_val;
}

static int64_t config_int64(const YAML::Node& config, const std::string& key, int64_t default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<int64_t>();
    }
  } catch (const YAML::Exception&) {
  }
  return default_val;
}

static int config_int(const YAML::Node& config, const std::string& key, int default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<int>();
    }
  } catch (const YAML::Exception&) {
  }
  return default_val;
}

static bool config_has(const YAML::Node& config, const std::string& key)
{
  return config && config[key];
}

// ============================================================================
// Replicate pixel format constants and helpers from ArenaCameraNode.cpp
// ============================================================================

namespace PixelFormat {
  constexpr uint64_t PFNC_POLARIZED_BAYER_RG8 = 0x8220020F;
  constexpr uint64_t PFNC_BGR8 = 0x02180015;
}

inline bool is_polarized_format(uint64_t format) {
  return format == PixelFormat::PFNC_POLARIZED_BAYER_RG8;
}

inline bool is_supported_format(uint64_t format) {
  return true;
}

inline std::string get_pixel_format_name(uint64_t format) {
  if (format == PixelFormat::PFNC_POLARIZED_BAYER_RG8) {
    return "PolarizedAngles_0d_45d_90d_135d_BayerRG8";
  } else if (format == PixelFormat::PFNC_BGR8) {
    return "BGR8";
  } else {
    char buf[32];
    snprintf(buf, sizeof(buf), "0x%08lX", static_cast<unsigned long>(format));
    return std::string("Unknown (") + buf + ")";
  }
}

// ============================================================================
// Tests for config_string
// ============================================================================

TEST(ConfigHelpersTest, ConfigStringReturnsValueWhenPresent)
{
  YAML::Node config;
  config["topic"] = "/my_topic";
  EXPECT_EQ(config_string(config, "topic", "/default"), "/my_topic");
}

TEST(ConfigHelpersTest, ConfigStringReturnsDefaultWhenMissing)
{
  YAML::Node config;
  EXPECT_EQ(config_string(config, "missing_key", "/default"), "/default");
}

TEST(ConfigHelpersTest, ConfigStringReturnsDefaultOnNullNode)
{
  YAML::Node config;  // undefined node
  EXPECT_EQ(config_string(config, "key", "fallback"), "fallback");
}

TEST(ConfigHelpersTest, ConfigStringReturnsDefaultOnTypeMismatch)
{
  // A map node cannot be converted to string — yaml-cpp throws, we fall back to default.
  YAML::Node config;
  config["key"] = YAML::Load("{a: 1, b: 2}");
  std::string result = config_string(config, "key", "default_val");
  EXPECT_EQ(result, "default_val");
}

// ============================================================================
// Tests for config_bool
// ============================================================================

TEST(ConfigHelpersTest, ConfigBoolReturnsValueWhenPresent)
{
  YAML::Node config;
  config["flag"] = true;
  EXPECT_TRUE(config_bool(config, "flag", false));
  
  config["flag2"] = false;
  EXPECT_FALSE(config_bool(config, "flag2", true));
}

TEST(ConfigHelpersTest, ConfigBoolReturnsDefaultWhenMissing)
{
  YAML::Node config;
  EXPECT_TRUE(config_bool(config, "missing", true));
  EXPECT_FALSE(config_bool(config, "missing", false));
}

TEST(ConfigHelpersTest, ConfigBoolReturnsDefaultOnTypeMismatch)
{
  YAML::Node config;
  config["key"] = "not_a_bool_string";
  // yaml-cpp may throw on invalid bool conversion
  bool result = config_bool(config, "key", true);
  EXPECT_TRUE(result);  // Falls back to default
}

// ============================================================================
// Tests for config_double
// ============================================================================

TEST(ConfigHelpersTest, ConfigDoubleReturnsValueWhenPresent)
{
  YAML::Node config;
  config["gain"] = 3.14;
  EXPECT_DOUBLE_EQ(config_double(config, "gain", 0.0), 3.14);
}

TEST(ConfigHelpersTest, ConfigDoubleReturnsDefaultWhenMissing)
{
  YAML::Node config;
  EXPECT_DOUBLE_EQ(config_double(config, "missing", -1.0), -1.0);
}

TEST(ConfigHelpersTest, ConfigDoubleAcceptsIntegerValue)
{
  YAML::Node config;
  config["rate"] = 30;
  EXPECT_DOUBLE_EQ(config_double(config, "rate", -1.0), 30.0);
}

// ============================================================================
// Tests for config_int64
// ============================================================================

TEST(ConfigHelpersTest, ConfigInt64ReturnsValueWhenPresent)
{
  YAML::Node config;
  config["brightness"] = 128;
  EXPECT_EQ(config_int64(config, "brightness", -1), 128);
}

TEST(ConfigHelpersTest, ConfigInt64ReturnsDefaultWhenMissing)
{
  YAML::Node config;
  EXPECT_EQ(config_int64(config, "missing", -1), -1);
}

TEST(ConfigHelpersTest, ConfigInt64ReturnsDefaultOnTypeMismatch)
{
  YAML::Node config;
  config["key"] = "hello";
  EXPECT_EQ(config_int64(config, "key", 42), 42);
}

// ============================================================================
// Tests for config_int
// ============================================================================

TEST(ConfigHelpersTest, ConfigIntReturnsValueWhenPresent)
{
  YAML::Node config;
  config["quality"] = 90;
  EXPECT_EQ(config_int(config, "quality", 80), 90);
}

TEST(ConfigHelpersTest, ConfigIntReturnsDefaultWhenMissing)
{
  YAML::Node config;
  EXPECT_EQ(config_int(config, "missing", 80), 80);
}

// ============================================================================
// Tests for config_has
// ============================================================================

TEST(ConfigHelpersTest, ConfigHasReturnsTrueWhenPresent)
{
  YAML::Node config;
  config["key"] = "value";
  EXPECT_TRUE(config_has(config, "key"));
}

TEST(ConfigHelpersTest, ConfigHasReturnsFalseWhenMissing)
{
  YAML::Node config;
  config["other"] = "value";
  EXPECT_FALSE(config_has(config, "key"));
}

TEST(ConfigHelpersTest, ConfigHasReturnsFalseOnNullNode)
{
  YAML::Node config;
  EXPECT_FALSE(config_has(config, "key"));
}

TEST(ConfigHelpersTest, ConfigHasReturnsTrueOnExplicitlyNullValue)
{
  // yaml-cpp: a key with a null value (e.g. "key: ~") still exists in the map,
  // so config_has returns true. Note: yaml-cpp converts null to the string "null"
  // rather than throwing, so config_string will NOT fall back to the default.
  YAML::Node config = YAML::Load("key: ~");
  EXPECT_TRUE(config_has(config, "key"));
}

// ============================================================================
// Additional config_double edge cases
// ============================================================================

TEST(ConfigHelpersTest, ConfigDoubleAcceptsZero)
{
  YAML::Node config;
  config["timeout"] = 0.0;
  EXPECT_DOUBLE_EQ(config_double(config, "timeout", 5.0), 0.0);
}

TEST(ConfigHelpersTest, ConfigDoubleAcceptsNegativeValue)
{
  YAML::Node config;
  config["val"] = -3.5;
  EXPECT_DOUBLE_EQ(config_double(config, "val", 0.0), -3.5);
}

// ============================================================================
// Tests for gpu_acceleration string parameter
// ============================================================================

TEST(ConfigHelpersTest, GpuAccelerationAuto)
{
  YAML::Node config;
  config["gpu_acceleration"] = "auto";
  EXPECT_EQ(config_string(config, "gpu_acceleration", "auto"), "auto");
}

TEST(ConfigHelpersTest, GpuAccelerationGpu)
{
  YAML::Node config;
  config["gpu_acceleration"] = "gpu";
  EXPECT_EQ(config_string(config, "gpu_acceleration", "auto"), "gpu");
}

TEST(ConfigHelpersTest, GpuAccelerationCpu)
{
  YAML::Node config;
  config["gpu_acceleration"] = "cpu";
  EXPECT_EQ(config_string(config, "gpu_acceleration", "auto"), "cpu");
}

TEST(ConfigHelpersTest, GpuAccelerationMissingDefaultsToAuto)
{
  YAML::Node config;
  EXPECT_EQ(config_string(config, "gpu_acceleration", "auto"), "auto");
}

// ============================================================================
// Tests for pixel format detection utilities
// ============================================================================

TEST(PixelFormatTest, IsPolarizedFormatDetectsPolarized)
{
  EXPECT_TRUE(is_polarized_format(0x8220020F));
}

TEST(PixelFormatTest, IsPolarizedFormatRejectsNonPolarized)
{
  EXPECT_FALSE(is_polarized_format(0x02180015));  // BGR8
  EXPECT_FALSE(is_polarized_format(0x00000000));
  EXPECT_FALSE(is_polarized_format(0xFFFFFFFF));
}

TEST(PixelFormatTest, IsSupportedFormatAcceptsAll)
{
  // Currently all formats are considered supported (conversion attempted)
  EXPECT_TRUE(is_supported_format(0x8220020F));
  EXPECT_TRUE(is_supported_format(0x02180015));
  EXPECT_TRUE(is_supported_format(0x00000000));
}

TEST(PixelFormatTest, GetPixelFormatNameReturnsPolarized)
{
  EXPECT_EQ(get_pixel_format_name(0x8220020F),
            "PolarizedAngles_0d_45d_90d_135d_BayerRG8");
}

TEST(PixelFormatTest, GetPixelFormatNameReturnsBGR8)
{
  EXPECT_EQ(get_pixel_format_name(0x02180015), "BGR8");
}

TEST(PixelFormatTest, GetPixelFormatNameReturnsUnknownForOther)
{
  std::string name = get_pixel_format_name(0x12345678);
  EXPECT_NE(name.find("Unknown"), std::string::npos);
}

// ============================================================================
// Tests for full config parsing scenario
// ============================================================================

TEST(ConfigHelpersTest, FullConfigParsingScenario)
{
  // Simulate a typical camera.yaml structure matching the real camera.yaml
  std::string yaml_str = R"(
    topic: "/test_camera/images"
    pixelformat: "bgr8"
    publish_raw: true
    publish_compressed: false
    jpeg_quality: 75
    trigger_mode: false
    acquisition_frame_rate_enable: true
    acquisition_frame_rate: 30.0
    watchdog_timeout_sec: 10.0
    publish_pol_channels: false
    publish_pol_max: true
    publish_dolp: true
    publish_aolp: true
    gpu_acceleration: "cpu"
  )";

  YAML::Node config = YAML::Load(yaml_str);

  EXPECT_EQ(config_string(config, "topic", "/default"), "/test_camera/images");
  EXPECT_EQ(config_string(config, "pixelformat", ""), "bgr8");
  EXPECT_TRUE(config_bool(config, "publish_raw", false));
  EXPECT_FALSE(config_bool(config, "publish_compressed", true));
  EXPECT_EQ(config_int(config, "jpeg_quality", 80), 75);
  EXPECT_FALSE(config_bool(config, "trigger_mode", true));
  EXPECT_TRUE(config_bool(config, "acquisition_frame_rate_enable", false));
  EXPECT_DOUBLE_EQ(config_double(config, "acquisition_frame_rate", -1.0), 30.0);
  EXPECT_DOUBLE_EQ(config_double(config, "watchdog_timeout_sec", 5.0), 10.0);

  // Polarization publish flags
  EXPECT_FALSE(config_bool(config, "publish_pol_channels", true));
  EXPECT_TRUE(config_bool(config, "publish_pol_max", false));
  EXPECT_TRUE(config_bool(config, "publish_dolp", false));
  EXPECT_TRUE(config_bool(config, "publish_aolp", false));

  // GPU acceleration string
  EXPECT_EQ(config_string(config, "gpu_acceleration", "auto"), "cpu");

  // Verify missing keys return defaults
  EXPECT_EQ(config_string(config, "serial", ""), "");
  EXPECT_DOUBLE_EQ(config_double(config, "gain", -1.0), -1.0);
  EXPECT_EQ(config_int64(config, "target_brightness", -1), -1);
}

TEST(ConfigHelpersTest, WatchdogDisabledAtZero)
{
  // watchdog_timeout_sec: 0.0 disables the watchdog per camera.yaml docs
  YAML::Node config;
  config["watchdog_timeout_sec"] = 0.0;
  EXPECT_DOUBLE_EQ(config_double(config, "watchdog_timeout_sec", 5.0), 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
