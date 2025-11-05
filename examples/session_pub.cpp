#include <Eigen/Core>
#include <iostream>
#include <thread>
#include "session.h"
#include "session_options.h"
#include "rb/v1/state_generated.h"

using namespace std::chrono_literals;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

int main(int argc, char** argv) {
  rb::io::SessionOptions options;
  options.backend = rb::io::SessionBackendType::kZenoh;
  options.ns = "example";
  if (argc > 1) {
    std::cout << "[INFO] Loading Zenoh config from " << argv[1] << std::endl;
    options.zenoh_config = rb::io::ZenohConfig::FromFile(argv[1]);
  }

  auto session = rb::io::Session::Open(std::move(options));

  Eigen::Vector<float, 7> q, qdot;
  q << 0, 1, 2, 3, 4, 5, 6;
  qdot << 6, 5, 4, 3, 2, 1, 0;

  while (true)
  {
    q.array() += 1.f;
    qdot.array() += 1.f;

    rb::v1::RootStateT root_state;
    root_state.state = std::make_unique<rb::v1::State>(flatbuffers::span<const float, 7>(q.data(), 7),
                                                       flatbuffers::span<const float, 7>(qdot.data(), 7));

    session.Publish<rb::v1::RootState>("state", root_state);
    std::cout << "Published state with q = " << q.transpose().format(CleanFmt) << std::endl;

    std::this_thread::sleep_for(1s);
  }

  return 0;
}