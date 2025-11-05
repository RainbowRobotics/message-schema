#pragma once

#include <string>
#include <string_view>

namespace rb::io {

enum class SessionBackendType {
  kZenoh,
};

inline SessionBackendType ParseSessionBackendType(std::string_view str) {
  if (str == "zenoh") {
    return SessionBackendType::kZenoh;
  }
  throw std::invalid_argument("Unknown Session backend type: " + std::string(str));
}

}  // namespace rb::io