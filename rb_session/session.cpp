#include "session.h"
#include "zenoh_backend.h"
#include "zenoh_config.h"

namespace {
inline std::string normalize_join(std::string_view ns, std::string_view p) {
  if (ns.empty()) {
    std::string out;
    out.reserve(p.size());
    if (!p.empty() && p.front() == '/') {
      out.append(p.substr(1));
    } else {
      out.append(p);
    }
    return out;
  }

  std::string out;
  out.reserve(ns.size() + 1 + p.size());
  if (ns.back() == '/') {
    out.append(ns.substr(0, ns.size() - 1));
  } else
    out.append(ns);

  if (!p.empty()) {
    out.push_back('/');
    if (p.front() == '/') {
      out.append(p.substr(1));
    } else {
      out.append(p);
    }
  }
  return out;
}
}  // namespace

namespace rb::io {
Session Session::Open(SessionOptions options) {
  switch (options.backend) {
    case SessionBackendType::kZenoh: {
      ZenohConfig cfg = options.zenoh_config ? std::move(*options.zenoh_config) : ZenohConfig{};
      auto backend = std::make_unique<ZenohBackend>(std::move(cfg));
      return Session(std::move(backend), options.ns);
    }
    default:
      throw std::runtime_error("session: unsupported backend");
  }
}

std::string Session::Resource(std::string_view path) const {
  return normalize_join(ns_, path);
}

}  // namespace rb::io