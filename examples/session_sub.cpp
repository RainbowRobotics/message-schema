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

  session.Subscribe<rb::v1::RootState>("state", [](rb::v1::RootStateT root_state) {
    Eigen::Vector<float, 7> q = Eigen::Map<const Eigen::Vector<float, 7>>(root_state.state->q()->data());
    Eigen::Vector<float, 7> qdot = Eigen::Map<const Eigen::Vector<float, 7>>(root_state.state->qdot()->data());

    std::cout << "Received state with q = " << q.transpose().format(CleanFmt) << std::endl;
  });

  // Keep the main thread alive to continue receiving messages
  while (true) {
    std::this_thread::sleep_for(1s);
  }

  return 0;
}