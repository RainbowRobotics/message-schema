#pragma once

#include <utility>
#include <string>
#include <zenoh.h>

namespace rb::io {

struct ZenohConfig {
  z_owned_config_t config_{};

  ZenohConfig() { z_config_default(&config_); }

  explicit ZenohConfig(z_owned_config_t config) : config_(std::move(config)) {}

  explicit ZenohConfig(const ZenohConfig&) = delete;
  ZenohConfig& operator=(const ZenohConfig&) = delete;

  ZenohConfig(ZenohConfig&& other) noexcept { z_config_take(&config_, z_move(other.config_)); }

  ZenohConfig& operator=(ZenohConfig&& other) noexcept {
    if (this != &other) {
      z_config_drop(z_move(config_));
      z_config_take(&config_, z_move(other.config_));
    }
    return *this;
  }

  ~ZenohConfig() { z_config_drop(z_move(config_)); }

  static ZenohConfig FromFile(const char* path) {
    z_owned_config_t config;
    auto result = zc_config_from_file(&config, path);
    if (result == 0) {
      return ZenohConfig(std::move(config));
    } else {
      throw std::runtime_error("Failed to load Zenoh config from file (error code: " + std::to_string(result) + ")");
    }
  }

  static ZenohConfig FromString(const char* str) {
    z_owned_config_t config;
    auto result = zc_config_from_str(&config, str);
    if (result == 0) {
      return ZenohConfig(std::move(config));
    } else {
      throw std::runtime_error("Failed to load Zenoh config from string (error code: " + std::to_string(result) + ")");
    }
  }
};

}  // namespace rb::io