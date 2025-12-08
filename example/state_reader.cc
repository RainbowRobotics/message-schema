#include <flatbuffers/flatbuffers.h>
#include <fstream>
#include <iostream>
#include "rb/example/state_generated.h"

using namespace rb::example;

int main(int argc, char** argv) {
  const char* path = (argc > 1) ? argv[1] : "/tmp/state";

  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    std::cerr << "open failed\n";
    return 1;
  }
  std::vector<uint8_t> buf(std::istreambuf_iterator<char>(ifs), {});
  ifs.close();

  flatbuffers::Verifier v(buf.data(), buf.size());
  if (!VerifyRootStateBuffer(v)) {
    std::cerr << "verify failed\n";
    return 1;
  }

  const auto* rs = GetRootState(buf.data());
  const auto* state = rs->state();

  std::cout << "q:\n";
  for (const auto& q : *state->q()) {
    std::cout << q << " ";
  }
  std::cout << "\n";

  std::cout << "qdot:\n";
  for (const auto& qdot : *state->qdot()) {
    std::cout << qdot << " ";
  }
  std::cout << "\n";

  return 0;
}
