#ifndef RB_MIDDLEWARE_SESSION_SESSION_OPTIONS_H_
#define RB_MIDDLEWARE_SESSION_SESSION_OPTIONS_H_

#include <string>

namespace rb::io {

struct SessionOptions {
  std::string backend = "zenoh";  // "zenoh" | "inproc" (NOT IMPLEMENTED YET)
  std::string ns = "rb";
  std::string locator = "tcp/127.0.0.1:7447";  // zenohd endpoint
};

}  // namespace rb::io

#endif