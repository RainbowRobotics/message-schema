#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <string_view>
#include <vector>

namespace rb::io {

class SessionBackend {
 public:
  using Bytes = std::span<const uint8_t>;
  using SubCallback = std::function<void(Bytes)>;
  using QueryHandler = std::function<std::vector<uint8_t>(Bytes)>;

  virtual ~SessionBackend() = default;

  virtual void Publish(std::string_view resource, Bytes bytes) = 0;
  virtual void Subscribe(std::string_view resource, SubCallback cb) = 0;

  virtual std::vector<uint8_t> Query(std::string_view resource, Bytes req, int timeout_ms) = 0;
  virtual void DeclareQueryable(std::string_view resource, QueryHandler handler) = 0;
};

}  // namespace rb::io