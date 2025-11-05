#pragma once

#include <optional>
#include <string>
#include "session_backend_type.h"
#include "zenoh_config.h"

namespace rb::io {

struct SessionOptions {
  SessionBackendType backend = SessionBackendType::kZenoh;
  std::string ns = "rb";

  std::optional<ZenohConfig> zenoh_config{};
};

}  // namespace rb::io