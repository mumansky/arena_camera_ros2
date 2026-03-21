#pragma once

#include <cstdint>
#include <string>
#include <yaml-cpp/yaml.h>

/**
 * @brief Safe YAML config reading helpers.
 *
 * Each function reads a value from a YAML node, returning a default if the key
 * is missing or the type conversion fails.  No exceptions escape.
 */

inline std::string config_string(const YAML::Node& config, const std::string& key,
                                 const std::string& default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<std::string>();
    }
  } catch (const YAML::Exception&) {}
  return default_val;
}

inline bool config_bool(const YAML::Node& config, const std::string& key, bool default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<bool>();
    }
  } catch (const YAML::Exception&) {}
  return default_val;
}

inline double config_double(const YAML::Node& config, const std::string& key, double default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<double>();
    }
  } catch (const YAML::Exception&) {}
  return default_val;
}

inline int64_t config_int64(const YAML::Node& config, const std::string& key, int64_t default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<int64_t>();
    }
  } catch (const YAML::Exception&) {}
  return default_val;
}

inline int config_int(const YAML::Node& config, const std::string& key, int default_val)
{
  try {
    if (config && config[key]) {
      return config[key].as<int>();
    }
  } catch (const YAML::Exception&) {}
  return default_val;
}

inline bool config_has(const YAML::Node& config, const std::string& key)
{
  return config && config[key];
}

inline bool config_has_value(const YAML::Node& config, const std::string& key)
{
  return config && config[key] && !config[key].IsNull();
}
