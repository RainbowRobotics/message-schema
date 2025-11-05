#include <iostream>
#include <chrono>
#include "session.h"
#include "session_options.h"
#include "rb/v1/power_on_generated.h"

using namespace std::chrono_literals;

bool PowerOn(rb::io::Session& session, const rb::v1::PowerOnRequestT& req, rb::v1::PowerOnResponseT* res) {
  bool rv = session.CallWith<rb::v1::PowerOnRequest, rb::v1::PowerOnResponse>(
      "power_on",
      [&req](flatbuffers::FlatBufferBuilder& fbb) {
        auto id_offset = fbb.CreateString(req.id);
        return rb::v1::CreatePowerOnRequest(fbb, id_offset);
      },
      [res](const rb::v1::PowerOnResponse* r) { r->UnPackTo(res); }, 100);
  return rv;
}

int main(int argc, char** argv) {
  rb::io::SessionOptions options;
  options.backend = rb::io::SessionBackendType::kZenoh;
  options.ns = "example";
  if (argc > 1) {
    std::cout << "[INFO] Loading Zenoh config from " << argv[1] << std::endl;
    options.zenoh_config = rb::io::ZenohConfig::FromFile(argv[1]);
  }

  auto session = rb::io::Session::Open(std::move(options));

  {
    rb::v1::PowerOnRequestT req;
    req.id = "valid";
    rb::v1::PowerOnResponseT res;
    if (!PowerOn(session, req, &res)) {
      std::cerr << "PowerOn RPC failed" << std::endl;
      return 1;
    }
    std::cout << "power_on(id = " << req.id << ") = {id: '" << res.id
              << "', state: " << (res.state == rb::v1::PowerState_On ? "ON" : "OFF") << ", error: " << res.error << "}"
              << std::endl;
  }

  {
    rb::v1::PowerOnRequestT req;
    req.id = "invalid";
    rb::v1::PowerOnResponseT res;
    if (!PowerOn(session, req, &res)) {
      std::cerr << "PowerOn RPC failed" << std::endl;
      return 1;
    }
    std::cout << "power_on(" << req.id << ") = {id: '" << res.id
              << "', state: " << (res.state == rb::v1::PowerState_On ? "ON" : "OFF") << ", error: " << res.error << "}"
              << std::endl;
  }

  return 0;
}