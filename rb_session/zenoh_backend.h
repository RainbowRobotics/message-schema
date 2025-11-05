#pragma once

#include <memory>
#include "session_backend.h"
#include "zenoh_config.h"

namespace rb::io {

class ZenohBackend : public SessionBackend {
 public:
  ZenohBackend(ZenohConfig config);
  ~ZenohBackend() override;

  void Publish(std::string_view resource, Bytes bytes) override;
  void Subscribe(std::string_view resource, SubCallback cb) override;

  std::vector<uint8_t> Query(std::string_view resource, Bytes req, int timeout_ms) override;
  void DeclareQueryable(std::string_view resource, QueryHandler handler) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rb::io