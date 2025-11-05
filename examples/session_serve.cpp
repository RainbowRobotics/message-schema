#include <iostream>
#include <thread>
#include "session.h"
#include "session_options.h"
#include "rb/v1/power_on_generated.h"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rb::io::SessionOptions options;
  options.backend = rb::io::SessionBackendType::kZenoh;
  options.ns = "example";
  if (argc > 1) {
    std::cout << "[INFO] Loading Zenoh config from " << argv[1] << std::endl;
    options.zenoh_config = rb::io::ZenohConfig::FromFile(argv[1]);
  }

  auto session = rb::io::Session::Open(std::move(options));

  session.Serve<rb::v1::PowerOnRequest, rb::v1::PowerOnResponse>(
      "power_on", [](flatbuffers::FlatBufferBuilder& fbb, const rb::v1::PowerOnRequest* req) {
        using namespace rb::v1;

        std::string id(req->id()->str());
        std::cout << "Received 'power_on' request with id = " << id << std::endl;

        // (예시) id 가 'valid' 인 경우 상태를 PowerState_On 리턴
        bool on = (id == "valid");

        auto id_off = fbb.CreateString(id);
        PowerState state = on ? PowerState_On : PowerState_Pending;
        ErrorCode err = on ? ErrorCode_OK : ErrorCode_INVALID_ARGUMENT;

        auto resp = CreatePowerOnResponse(fbb, id_off, state, err);
        return resp;
      });

  while (true) {
    std::this_thread::sleep_for(1s);
  }

  return 0;
}